import os
import subprocess
import time
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_camera_supervisor_kills_reparented_camera_process(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    child_pid_file = tmp_path / "camera-child.pid"
    device = tmp_path / "video"
    device.touch()
    v4l2 = fake_bin / "v4l2-ctl"
    v4l2.write_text("#!/bin/sh\nexit 0\n")
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        "#!/bin/bash\n"
        'if [[ $1 == run ]]; then\n'
        '  (trap "" TERM; exec sleep 60) &\n'
        '  child=$!\n'
        '  echo "$child" > "$CAMERA_CHILD_PID"\n'
        '  wait "$child"\n'
        "fi\n"
    )
    v4l2.chmod(0o755)
    ros2.chmod(0o755)
    env = {
        **os.environ,
        "PATH": f"{fake_bin}:{os.environ['PATH']}",
        "CAMERA_CHILD_PID": str(child_pid_file),
    }
    process = subprocess.Popen(
        [
            str(ROOT / "scripts" / "camera-supervisor.sh"),
            "--device", str(device), "--name", "test_camera",
            "--namespace", "/test", "--camera-name", "test",
            "--frame", "test_frame", "--size", "[320, 240]",
            "--camera-info-url", "none", "--startup-grace", "30",
        ],
        env=env,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        deadline = time.monotonic() + 5
        while not child_pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert child_pid_file.exists()
        child_pid = int(child_pid_file.read_text())
        process.terminate()
        process.wait(timeout=8)
        stat = Path(f"/proc/{child_pid}/stat")
        assert not stat.exists() or stat.read_text().split()[2] == "Z"
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=2)
