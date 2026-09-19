"""No-root behavioral checks for the service self-heal helper."""

from __future__ import annotations

import os
import pathlib
import signal
import subprocess
import time

import pytest

ROOT = pathlib.Path(__file__).parents[1]
LIB = ROOT / "scripts" / "lib" / "self-heal.sh"

# Fake network state: the robot is on wlan0 with one address.
BASELINE = '''
self_heal_addresses() { echo "wlan0 10.0.0.2/24"; }
'''
QUIET_EVENTS = "_self_heal_events() { sleep 60; }\n"


def events(*lines: str) -> str:
    body = "".join(f"echo {line!r}; " for line in lines)
    return f"_self_heal_events() {{ sleep 0.5; {body}sleep 60; }}\n"


@pytest.fixture
def launch(tmp_path):
    """Start a bash script as a service main process; reap everything it left behind."""
    started: list[subprocess.Popen] = []

    def start(body: str, systemd: bool = True):
        env = {**os.environ, "LEKIWI_SELF_HEAL_SETTLE": "0"}
        env.pop("INVOCATION_ID", None)
        if systemd:
            env["INVOCATION_ID"] = "test"
        script = f'set -Eeuo pipefail\nsource "{LIB}"\n{body}'
        process = subprocess.Popen(
            ["bash", "-c", script], env=env, start_new_session=True,
            stdout=subprocess.DEVNULL, stderr=(tmp_path / "stderr").open("w"),
        )
        started.append(process)
        return process

    yield start
    for process in started:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        process.wait()


def test_manual_runs_start_no_watchers(launch):
    process = launch(f"{QUIET_EVENTS}self_heal /dev/null\n[[ -z $(jobs -p) ]]", systemd=False)
    assert process.wait(timeout=5) == 0


def test_replaced_device_node_kills_the_main_process(launch, tmp_path):
    device = tmp_path / "ttyUSB0"
    device.write_text("")
    process = launch(f'{BASELINE}{QUIET_EVENTS}self_heal "{device}"\nexec sleep 60')
    time.sleep(1)
    replacement = tmp_path / "ttyUSB1"
    replacement.write_text("")
    replacement.rename(device)  # a new inode under the same name, as udev does
    assert process.wait(timeout=15) == -signal.SIGKILL
    assert "was re-enumerated or removed" in (tmp_path / "stderr").read_text()


@pytest.mark.parametrize("event", [
    "Deleted 3: wlan0    inet 10.0.0.2/24 brd 10.0.0.255 scope global wlan0",
    "4: eth0    inet 10.1.1.5/24 brd 10.1.1.255 scope global dynamic eth0",
])
def test_lost_or_new_address_kills_the_main_process(launch, tmp_path, event):
    process = launch(f"{BASELINE}{events(event)}self_heal\nexec sleep 60")
    assert process.wait(timeout=15) == -signal.SIGKILL
    assert "IPv4 address" in (tmp_path / "stderr").read_text()


def test_renewals_and_unrelated_interfaces_are_ignored(launch):
    process = launch(BASELINE + events(
        "3: wlan0    inet 10.0.0.2/24 brd 10.0.0.255 scope global dynamic wlan0",
        "5: docker0    inet 172.17.0.1/16 brd 172.17.255.255 scope global docker0",
        "Deleted 9: br-1a2b    inet 172.20.0.1/16 brd 172.20.255.255 scope global br-1a2b",
        "6: tailscale0    inet 100.64.0.9/32 scope global tailscale0",
        "3: wlan0    inet 169.254.1.1/16 scope link wlan0",
    ) + "self_heal\nexec sleep 60")
    with pytest.raises(subprocess.TimeoutExpired):
        process.wait(timeout=4)
