import os
import subprocess
import time
from pathlib import Path

import pytest
import rclpy
from rclpy.context import Context
from rclpy.node import Node


ROOT = Path(__file__).resolve().parents[1]


def test_camera_supervisor_kills_reparented_camera_process(tmp_path):
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    child_pid_file = tmp_path / "camera-child.pid"
    device = tmp_path / "video"
    device.touch()
    query_log = tmp_path / "v4l-queries.log"
    ros_log = tmp_path / "ros-queries.log"
    v4l2 = fake_bin / "v4l2-ctl"
    v4l2.write_text('#!/bin/sh\nprintf "%s\\n" "$*" >> "$CAMERA_V4L_LOG"\n'
                   'case " $* " in *" --info "*) exit 0 ;; *) exit 2 ;; esac\n')
    ros2 = fake_bin / "ros2"
    ros2.write_text(
        "#!/bin/bash\n"
        'printf "%s\\n" "$*" >> "$CAMERA_ROS_LOG"\n'
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
        "CAMERA_V4L_LOG": str(query_log),
        "CAMERA_ROS_LOG": str(ros_log),
        "LEKIWI_RUNTIME_DIR": str(tmp_path / "runtime"),
    }
    process = subprocess.Popen(
        [
            str(ROOT / "scripts" / "camera-supervisor.sh"),
            "--device", str(device), "--name", "test_camera",
            "--namespace", "/test", "--camera-name", "test",
            "--frame", "test_frame", "--size", "[320, 240]",
            "--camera-info-url", "none", "--startup-grace", "0",
            "--heartbeat-interval", "1", "--jpeg-quality", "50",
        ],
        env=env,
        start_new_session=True,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        # Device qualification shares the Pi with live camera and motor services.
        deadline = time.monotonic() + 20
        while not child_pid_file.exists() and time.monotonic() < deadline:
            time.sleep(0.05)
        assert child_pid_file.exists()
        parameters = tmp_path / "runtime/camera-info/test_camera-compression.yaml"
        assert f"--params-file {parameters}" in ros_log.read_text()
        context = Context()
        rclpy.init(args=["--ros-args", "--params-file", str(parameters)], context=context)
        node = Node("compression_parameter_test", context=context)
        try:
            for name in (".image_raw.compressed.jpeg_quality", ".image_raw.jpeg_quality"):
                assert node.declare_parameter(name, 95).value == 50
        finally:
            node.destroy_node()
            context.try_shutdown()
        queries = query_log.read_text().splitlines()
        assert queries and set(queries) == {f"--device {device} --info"}
        deadline = time.monotonic() + 20
        while not any(line.startswith("topic echo") for line in ros_log.read_text().splitlines()):
            assert time.monotonic() < deadline, "camera heartbeat was never checked"
            time.sleep(0.05)
        heartbeat = next(line for line in ros_log.read_text().splitlines() if line.startswith("topic echo"))
        assert "--field header" in heartbeat
        assert "/test/image_raw" in heartbeat
        child_pid = int(child_pid_file.read_text())
        process.terminate()
        process.wait(timeout=8)
        stat = Path(f"/proc/{child_pid}/stat")
        assert not stat.exists() or stat.read_text().split()[2] == "Z"
    finally:
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)


@pytest.mark.parametrize('quality', ['-1', '101', '1.5', 'nan', '50\nother: 1'])
def test_camera_supervisor_rejects_invalid_jpeg_quality(quality):
    result = subprocess.run([
        str(ROOT / "scripts/camera-supervisor.sh"),
        "--device", "/dev/null", "--name", "test_camera", "--namespace", "/test",
        "--camera-name", "test", "--frame", "test_frame", "--size", "[320, 240]",
        "--camera-info-url", "none", "--jpeg-quality", quality,
    ], capture_output=True, text=True, timeout=3)
    assert result.returncode == 2
    assert 'usage:' in result.stderr
