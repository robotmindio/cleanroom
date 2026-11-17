"""The kernel backport changes only VL805's mask and cannot unload live USB."""

import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'vl805_fix', Path(__file__).parents[1] / 'scripts/build-vl805-fix.py')
fix = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(fix)

SOURCE = '''if (pdev->vendor == PCI_VENDOR_ID_VIA && pdev->device == PCI_DEVICE_ID_VIA_VL805) {
        xhci->dma_mask_bits = 36;
        xhci->quirks |= XHCI_LPM_SUPPORT;
}
other_device->dma_mask_bits = 36;
'''


def test_backport_changes_only_the_known_vendor_device_dma_width():
    result = fix.patch_source(SOURCE)
    assert result.replace('xhci->dma_mask_bits = 40;', 'xhci->dma_mask_bits = 36;') == SOURCE
    assert 'other_device->dma_mask_bits = 36;' in result


@pytest.mark.parametrize('source', (SOURCE * 2, SOURCE.replace('36;', '40;'),
                                  SOURCE.replace('PCI_VENDOR_ID_VIA', 'PCI_VENDOR_ID_OTHER')))
def test_unknown_source_or_already_fixed_kernel_requires_review(source):
    with pytest.raises(RuntimeError):
        fix.patch_source(source)


def files(tmp_path):
    pci, driver, usb = (tmp_path / name for name in ('pci', 'driver', 'usb'))
    for path in (pci, driver, usb):
        path.mkdir()
    target = pci / '0000:01:00.0'
    target.mkdir()
    for name, value in {'vendor': '0x1106', 'device': '0x3483', 'dma_mask_bits': '36'}.items():
        (target / name).write_text(value)
    motor = tmp_path / 'xhci-hcd.1' / '4-1'
    motor.mkdir(parents=True)
    (motor / 'idVendor').write_text('1a86')
    (motor / 'idProduct').write_text('55d3')
    (usb / '4-1').symlink_to(motor)
    return pci, driver, usb, target


def test_only_failed_unbound_controller_with_motor_on_separate_rp1_can_be_recovered(tmp_path):
    pci, driver, usb, target = files(tmp_path)
    assert fix.activation_target(pci, driver, usb) == target


@pytest.mark.parametrize('fault', ('bound_target', 'other_bound_pci', 'unknown_mask', 'missing_motor',
                                  'wrong_controller', 'motor_not_rp1'))
def test_live_or_ambiguous_controller_can_never_be_unloaded(tmp_path, fault):
    pci, driver, usb, target = files(tmp_path)
    if fault == 'bound_target':
        (target / 'driver').symlink_to(driver)
    elif fault == 'other_bound_pci':
        (driver / '0000:02:00.0').mkdir()
    elif fault == 'unknown_mask':
        (target / 'dma_mask_bits').write_text('40')
    elif fault == 'missing_motor':
        (usb / '4-1').unlink()
    elif fault == 'wrong_controller':
        (target / 'device').write_text('0000')
    else:
        motor = tmp_path / 'other-controller'
        motor.mkdir()
        (motor / 'idVendor').write_text('1a86')
        (motor / 'idProduct').write_text('55d3')
        (usb / '4-1').unlink()
        (usb / '4-1').symlink_to(motor)
    with pytest.raises(RuntimeError):
        fix.activation_target(pci, driver, usb)
