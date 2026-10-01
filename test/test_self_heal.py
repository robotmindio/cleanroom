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
    process = launch("self_heal /dev/null\n[[ -z $(jobs -p) ]]", systemd=False)
    assert process.wait(timeout=5) == 0


def test_replaced_device_node_kills_the_main_process(launch, tmp_path):
    device = tmp_path / "ttyUSB0"
    device.write_text("")
    process = launch(f'self_heal "{device}"\nexec sleep 60')
    time.sleep(1)
    replacement = tmp_path / "ttyUSB1"
    replacement.write_text("")
    replacement.rename(device)  # a new inode under the same name, as udev does
    assert process.wait(timeout=15) == -signal.SIGKILL
    assert "was re-enumerated or removed" in (tmp_path / "stderr").read_text()


def test_network_only_services_start_no_watchers(launch):
    process = launch("self_heal\n[[ -z $(jobs -p) ]]")
    assert process.wait(timeout=5) == 0
