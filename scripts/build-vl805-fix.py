#!/usr/bin/env python3
"""Build the upstream VL805 DMA fix for Ubuntu's exact 6.8.0-1068.72 kernel.

Build-only by default. --activate replaces only an unused xhci_pci module;
--install persists the built override and regenerates this kernel's initramfs.
Never unloads xhci_hcd, touches RP1, restarts services, or reboots the robot.
"""

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import re
import subprocess
import urllib.request


ROOT = Path(__file__).resolve().parents[1]
KERNEL = '6.8.0-1068-raspi'
SOURCE = 'd72599130ac74334b2742e194f5946302d809dc9'
BASE = ('https://git.launchpad.net/~ubuntu-kernel/ubuntu/+source/linux-raspi/+git/noble/plain/'
        'drivers/usb/host/')
FIX = 'https://github.com/raspberrypi/linux/commit/35fc4a654ca629be18d954c402486d9221f6cdf2'


def patch_source(source):
    pattern = (r'(if \(pdev->vendor == PCI_VENDOR_ID_VIA && '
               r'pdev->device == PCI_DEVICE_ID_VIA_VL805\) \{\s*'
               r'xhci->dma_mask_bits = )36;')
    patched, count = re.subn(pattern, r'\g<1>40;', source)
    if count != 1:
        raise RuntimeError('expected exactly one known VL805 36-bit DMA quirk')
    return patched


def activation_target(pci_root, driver_root, usb_root):
    """Prove this module owns no running controller before unloading it."""
    devices = []
    for device in pci_root.iterdir():
        vendor, product = device / 'vendor', device / 'device'
        if (vendor.exists() and vendor.read_text().strip() == '0x1106'
                and product.read_text().strip() == '0x3483'):
            devices.append(device)
    if len(devices) != 1:
        raise RuntimeError('expected exactly one VIA VL805 PCIe controller')
    target = devices[0]
    if (target / 'driver').exists() or list(driver_root.glob('????:??:??.?')):
        raise RuntimeError('xhci_pci already has a bound controller; refusing to unload it')
    if (target / 'dma_mask_bits').read_text().strip() != '36':
        raise RuntimeError('unbound controller does not exhibit the known 36-bit mask')
    motors = []
    for device in usb_root.iterdir():
        vendor, product = device / 'idVendor', device / 'idProduct'
        if (vendor.exists() and vendor.read_text().strip() == '1a86'
                and product.read_text().strip() == '55d3'):
            motors.append(device)
    if len(motors) != 1 or 'xhci-hcd.' not in str(motors[0].resolve()):
        raise RuntimeError('motor USB adapter must be on the separate RP1 controller')
    return target


def run(argv):
    print('+ ' + ' '.join(map(str, argv)), flush=True)
    subprocess.run(list(map(str, argv)), check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-root', required=True, type=Path)
    parser.add_argument('--activate', action='store_true')
    parser.add_argument('--install', action='store_true')
    args = parser.parse_args()
    if platform.release() != KERNEL or platform.machine() != 'aarch64':
        parser.error('this backport requires the exact Ubuntu 6.8.0-1068-raspi arm64 kernel')
    headers = Path('/lib/modules') / KERNEL / 'build'
    if not headers.exists():
        parser.error(f'install linux-headers-{KERNEL} before building')
    if (args.activate or args.install) and os.geteuid() != 0:
        parser.error('--activate/--install require root')
    directory = args.build_root.resolve()
    directory.mkdir(parents=True, exist_ok=True)
    hashes = json.loads((ROOT / 'thirdparty/vl805/ubuntu-6.8.0-1068.72.json').read_text())
    for name, expected in hashes.items():
        path = directory / name
        # Reuse only verified originals. The patched C file is always fetched
        # again, so a second build cannot accidentally apply the patch twice.
        data = path.read_bytes() if path.exists() else b''
        if hashlib.sha256(data).hexdigest() != expected:
            data = urllib.request.urlopen(BASE + name + '?id=' + SOURCE, timeout=30).read()
        if hashlib.sha256(data).hexdigest() != expected:
            raise RuntimeError('Ubuntu source checksum mismatch: ' + name)
        path.write_bytes(data)
    c_file = directory / 'xhci-pci.c'
    c_file.write_text(patch_source(c_file.read_text()))
    (directory / 'Makefile').write_text('obj-m += xhci-pci.o\n')
    run(['make', '-C', headers, 'M=' + str(directory), 'CC=aarch64-linux-gnu-gcc-13', '-j2', 'modules'])
    module = directory / 'xhci-pci.ko'
    original = Path(subprocess.check_output(['modinfo', '-n', 'xhci_pci'], text=True).strip())
    vermagic = subprocess.check_output(['modinfo', '-F', 'vermagic', module], text=True).strip()
    original_magic = subprocess.check_output(['modinfo', '-F', 'vermagic', original], text=True).strip()
    if vermagic != original_magic:
        raise RuntimeError('replacement module ABI does not match the installed kernel')
    receipt = {'kernel': KERNEL, 'source': SOURCE, 'fix': FIX, 'sources_sha256': hashes,
               'module_sha256': hashlib.sha256(module.read_bytes()).hexdigest(),
               'vermagic': vermagic, 'activated': False, 'installed': False}
    if args.activate:
        target = activation_target(Path('/sys/bus/pci/devices'),
                                   Path('/sys/bus/pci/drivers/xhci_hcd'),
                                   Path('/sys/bus/usb/devices'))
        if Path('/sys/module/xhci_pci/refcnt').read_text().strip() != '0':
            raise RuntimeError('xhci_pci has users; refusing to unload it')
        run(['rmmod', 'xhci_pci'])
        try:
            run(['insmod', module])
            if (not (target / 'driver').exists()
                    or (target / 'dma_mask_bits').read_text().strip() != '40'):
                raise RuntimeError('VL805 did not bind with the corrected 40-bit DMA mask')
        except Exception:
            # Reload the packaged module only if the replacement did not load.
            # A successfully loaded replacement is never unloaded here: devices
            # may already be live and normal driver policy may have armed them.
            if not Path('/sys/module/xhci_pci').exists():
                run(['insmod', original])
            raise
        receipt.update(activated=True, pci_device=target.name)
    if args.install:
        destination = Path('/lib/modules') / KERNEL / 'updates/lekiwi/xhci-pci.ko'
        run(['install', '-D', '-m', '0644', module, destination])
        run(['depmod', '-a', KERNEL])
        selected = subprocess.check_output(['modinfo', '-n', 'xhci_pci'], text=True).strip()
        if Path(selected) != destination:
            raise RuntimeError('depmod did not select the corrected module')
        run(['update-initramfs', '-u', '-k', KERNEL])
        receipt.update(installed=True, installed_module=str(destination))
    (directory / 'receipt.json').write_text(json.dumps(receipt, indent=2) + '\n')
    print(json.dumps(receipt), flush=True)


if __name__ == '__main__':
    main()
