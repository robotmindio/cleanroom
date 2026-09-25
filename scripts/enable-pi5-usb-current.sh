#!/usr/bin/env bash
# Persist the Pi 5 high-current USB mode. Install root-owned for deployment use.
set -Eeuo pipefail

model=$(tr -d '\0' </proc/device-tree/model 2>/dev/null || true)
[[ $model == *"Raspberry Pi 5"* ]] || exit 0
[[ $EUID -eq 0 ]] || { echo "$0: run as root" >&2; exit 1; }
config=/boot/firmware/config.txt
[[ -f $config ]] || config=/boot/config.txt
[[ -f $config ]] || { echo "$0: cannot find Raspberry Pi boot config.txt" >&2; exit 1; }

python3 - "$config" <<'PY'
import os
import pathlib
import sys
import tempfile

path = pathlib.Path(sys.argv[1])
begin = "# BEGIN cleanroom Pi 5 USB current setting"
end = "# END cleanroom Pi 5 USB current setting"
lines = path.read_text(encoding="utf-8").splitlines()
kept = []
inside = False
for line in lines:
    if line == begin:
        if inside:
            raise SystemExit(f"duplicate managed USB setting in {path}")
        inside = True
    elif line == end:
        if not inside:
            raise SystemExit(f"unmatched managed USB setting in {path}")
        inside = False
    elif not inside:
        kept.append(line)
if inside:
    raise SystemExit(f"unterminated managed USB setting in {path}")

content = "\n".join(kept).rstrip() + f"\n\n[all]\n{begin}\nusb_max_current_enable=1\n{end}\n"
if path.read_text(encoding="utf-8") == content:
    print(f"Pi 5 USB current setting already enabled in {path}")
    raise SystemExit(0)
fd, temporary = tempfile.mkstemp(dir=path.parent, prefix=f".{path.name}.")
try:
    with os.fdopen(fd, "w", encoding="utf-8") as output:
        output.write(content)
    os.chmod(temporary, path.stat().st_mode & 0o777)
    os.replace(temporary, path)
finally:
    if os.path.exists(temporary):
        os.unlink(temporary)
print(f"Enabled Pi 5 USB current setting in {path}; reboot to apply (requires a 5 V / 5 A supply)")
PY
