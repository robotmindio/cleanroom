#!/usr/bin/env bash
# Run every LeKiwi calibration that is still missing. Motor and pose calibration
# run on the machine that owns the motor bus; camera calibration runs on the
# machine that owns that USB camera (the robot-side Pi in a split deployment).
# Calibrations that already have a saved result are skipped. Physical steps stay
# manual; the script shows what to do at each prompt.
#
#   scripts/calibrate.sh                     run whichever required calibrations are missing
#   scripts/calibrate.sh motor               redo Lerobot's servo-range calibration
#   scripts/calibrate.sh pose               redo the folded-CAD zero-pose capture
#   scripts/calibrate.sh camera             redo the front-camera intrinsics
#   scripts/calibrate.sh wrist              redo the wrist-camera intrinsics
#   scripts/calibrate.sh height             redo the laser height/pitch (free_space)
#   scripts/calibrate.sh wheels             redo the odometry scale measurement
#
# What each step saves or produces:
#   motor      Lerobot's cache  -> ~/.cache/huggingface/lerobot/calibration/robots/lekiwi/<id>.json
#   pose       arm folded-CAD-zero mapping -> ~/.ros/lekiwi_arm_calibration.json
#   camera     front intrinsics -> ~/.ros/camera_info/lekiwi_front.yaml
#   wrist      wrist intrinsics -> ~/.ros/camera_info/lekiwi_wrist.yaml
#   height     saves camera_height/camera_pitch to ~/.ros/lekiwi_launch_calibration.conf
#   wheels     saves xy/yaw_velocity_scale to ~/.ros/lekiwi_launch_calibration.conf
#
#   The wheel/height tools save their measured values to the machine-local launch file.
#   Set CALIBRATE_DRYRUN=1 to only print what auto would do (no services touched).
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/runtime-common.sh

set +u
# shellcheck source=/dev/null
source scripts/setup.bash
set -u

mode="${1:-auto}"
case "$mode" in
  auto|camera|wrist|height|motor|pose|wheels) ;;
  *) echo "usage: $0 [motor|pose|camera|wrist|height|wheels]" >&2; exit 1 ;;
esac

LOGS="${LEKIWI_LOGS:-$HOME/.ros/lekiwi}"
mkdir -p "$LOGS"
ID="${LEKIWI_ID:-lekiwi_1}"

POSE_FILE="$HOME/.ros/lekiwi_arm_calibration.json"
CAMERA_FILE="${LEKIWI_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_front.yaml}"
WRIST_CAMERA_FILE="${LEKIWI_WRIST_CAMERA_INFO:-$HOME/.ros/camera_info/lekiwi_wrist.yaml}"
HF_CACHE="${HF_HOME:-$HOME/.cache/huggingface}"
LEROBOT_HOME="${HF_LEROBOT_HOME:-$HF_CACHE/lerobot}"
CALIBRATION_DIR="${HF_LEROBOT_CALIBRATION:-$LEROBOT_HOME/calibration}"
MOTOR_FILE="$CALIBRATION_DIR/robots/lekiwi/$ID.json"

host_up() {
  ss -tln 2>/dev/null | grep -q ':5555' \
    && ss -tln 2>/dev/null | grep -q ':5557' \
    && pgrep -f '[t]orque-host\.py|[l]erobot\.robots\.lekiwi\.lekiwi_host' >/dev/null
}

legacy_host_up() {
  # Detect an older stock host only so calibration tears it down before it
  # tries to own the serial bus. It is never accepted as a safety-capable host.
  ss -tln 2>/dev/null | grep -q ':5555' \
    && pgrep -f '[l]erobot\.robots\.lekiwi\.lekiwi_host' >/dev/null
}

stack_up() {
  pgrep -f 'ros2 launch lekiwi_rmf' >/dev/null
}

# The host and the ROS stack both hold the motor bus; stop whichever is up so the
# next step can have the port and the pose capture gets a clean driver.
calibrating() {
  host_up || legacy_host_up || stack_up
}

motor_calibration_valid() {
  [ -s "$MOTOR_FILE" ]
}

pose_calibration_valid() {
  python3 - "$POSE_FILE" <<'PY'
import json
import sys

try:
    with open(sys.argv[1]) as source:
        calibration = json.load(source)
    zeroes = calibration["zero_positions"]
    directions = calibration["directions"]
    required = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper",
    }
    assert required <= set(zeroes) and required <= set(directions)
    assert all(isinstance(zeroes[name], (int, float)) for name in required)
    assert all(directions[name] in (-1, 1) for name in required)
except (OSError, ValueError, KeyError, TypeError, AssertionError):
    raise SystemExit(1)
PY
}

port_owned() { # the motor bus has an owner: a serial fd holder or a live LeRobot host
  local port="${LEKIWI_PORT:-$(first_match '/dev/serial/by-id/*USB_Single_Serial*')}"
  fuser -s "$(readlink -f "$port")" 2>/dev/null && return 0
  # A LeRobot host may not hold the serial fd open yet (mid-handshake); fuser alone
  # would miss it, and lerobot-calibrate would then fight it for the bus.
  pgrep -f '[t]orque-host\.py|[l]erobot\.robots\.lekiwi\.lekiwi_host' >/dev/null
}

stop_stack_if_running() {
  if calibrating; then
    echo "Stopping the running host/stack so calibration can hold the hardware clean."
    scripts/ros-stop.sh
  fi
}

unit_active() { # unit_active <name> -- the boot service is up and owns the device
  command -v systemctl >/dev/null || return 1
  systemctl is-active --quiet "$1" 2>/dev/null
}

as_root() { # as_root <command...>
  if [[ $EUID -eq 0 ]]; then "$@"; else
    command -v sudo >/dev/null || {
      echo "root/sudo required for: $* (this script stops the boot services it needs)" >&2
      exit 1
    }
    sudo "$@"
  fi
}

# The boot services own the hardware by design: lekiwi-host.service keeps the motor
# host alive on the bus and lekiwi-cameras.service publishes the cameras. Both would
# block -- or race -- a calibration run, so stop whichever one the step needs and
# remember it; the end-of-run message tells the user how to bring the services back.
stopped_host=0
stopped_cameras=0

stop_host_service() {
  if unit_active lekiwi-host.service; then
    echo "Stopping lekiwi-host.service (it owns the motor bus)."
    if ! as_root systemctl stop lekiwi-host.service; then
      echo "$0: could not stop lekiwi-host.service; run 'sudo systemctl stop lekiwi-host.service' and retry" >&2
      exit 1
    fi
    stopped_host=1
    sleep 2  # let the supervisor and its python child release the bus
  fi
}

stop_cameras_service() {
  if unit_active lekiwi-cameras.service; then
    echo "Stopping lekiwi-cameras.service (it owns the cameras)."
    if ! as_root systemctl stop lekiwi-cameras.service; then
      echo "$0: could not stop lekiwi-cameras.service; run 'sudo systemctl stop lekiwi-cameras.service' and retry" >&2
      exit 1
    fi
    stopped_cameras=1
    sleep 2
  fi
}

service_restore_hint() {
  if (( stopped_host || stopped_cameras )); then
    echo
    echo "Boot services stopped during calibration:"
    (( stopped_host )) && echo "  sudo systemctl start lekiwi-host.service     (re-enable the headless device flow)"
    (( stopped_cameras )) && echo "  sudo systemctl start lekiwi-cameras.service"
    echo "scripts/up.sh starts its own host and cameras, so the wired local flow needs none of these."
  fi
}

start_host_and_driver() {
  # The pose capture needs a LeKiwi host (motor values -> ZMQ) and the ROS driver
  # (reads them -> /joint_states). No cameras, RMF, MoveIt, or rosbridge needed.
  stop_host_service
  echo "Starting the LeRobot host and a slim stack (no cameras, RMF, or MoveIt)."
  if ! host_up; then
    setsid scripts/robot-host.sh --no-cameras > "$LOGS/host.log" 2>&1 &
    if ! wait_for 90 host_up; then
      echo "$0: LeKiwi host did not come up:" >&2
      tail -5 "$LOGS/host.log" >&2
      exit 1
    fi
    echo "host: up"
  else
    echo "host: already up"
  fi

  rm -f "$LOGS/stack-calib.log"
  setsid scripts/ros-start.sh camera_source:=remote start_rmf:=false \
    start_rosbridge:=false start_moveit:=false > "$LOGS/stack-calib.log" 2>&1 &
  if ! wait_for 120 grep -q "Connected to LeKiwi host" "$LOGS/stack-calib.log"; then
    echo "$0: ROS driver did not connect to the LeKiwi host:" >&2
    tail -10 "$LOGS/stack-calib.log" >&2
    exit 1
  fi
  echo "driver: connected"
}

calibrate_motor() {
  stop_host_service
  if port_owned; then
    echo "$0: the motor bus is still owned (a process holds the serial device or the host)." >&2
    echo "Stop it first and re-run -- e.g. pgrep -f lekiwi_host to see who, then kill it." >&2
    exit 1
  fi
  echo "Motor calibration: Lerobot will ask what to do with the stored ranges."
  echo "  Press Enter (or 'c') to re-sweep: first move every joint to the middle"
  echo "  of its range and press Enter, then sweep each arm joint through its full"
  echo "  range when asked."
  echo "  (If a completed calibration already exists, Lerobot offers to reuse it:"
  echo "   Enter = reuse the stored ranges and write them to the servos,"
  echo "   c + Enter = full re-sweep.)"
  scripts/robot-host.sh calibrate
  [ -s "$MOTOR_FILE" ] || {
    echo "$0: calibration finished but did not create $MOTOR_FILE" >&2
    exit 1
  }
  echo "Saved $MOTOR_FILE"
}

calibrate_pose() {
  local pose_backup=""
  if [ -s "$POSE_FILE" ]; then
    pose_backup="$POSE_FILE.bak"
    mv "$POSE_FILE" "$pose_backup"
    echo "Moved existing $POSE_FILE to $pose_backup."
  fi
  start_host_and_driver

  echo
  echo "Support the arm in RViz's folded CAD home pose (all arm joints at zero),"
  echo "with the gripper at its zero position. This is the vendor CAD's folded"
  echo "rest pose, not an upright arm. Keep holding it aligned; press Enter when ready:"
  read -r _

  echo "Capturing the held pose."
  if ! ros2 run lekiwi_rmf arm_calibration.py > "$LOGS/pose.log" 2>&1; then
    echo "$0: pose capture failed:" >&2
    cat "$LOGS/pose.log" >&2
    # A failed capture must not silently leave the robot using uncalibrated
    # zeros.  Put the last known calibration back and release the slim stack.
    if [ -n "$pose_backup" ] && [ -s "$pose_backup" ]; then
      mv "$pose_backup" "$POSE_FILE"
      echo "Restored the previous arm calibration." >&2
    fi
    scripts/ros-stop.sh
    exit 1
  fi
  [ -s "$POSE_FILE" ] || {
    echo "$0: arm_calibration ran but left no $POSE_FILE" >&2
    exit 1
  }
  echo "Saved $POSE_FILE"
}

calibrate_camera() {
  # The camera node holds the camera; stop anything reading it first.
  stop_stack_if_running
  stop_cameras_service
  if [ ! -e "${LEKIWI_FRONT:-$(first_match '/dev/v4l/by-id/*WEBCAM*-video-index0')}" ]; then
    echo "$0: the front camera is not attached to this machine." >&2
    echo "Run this command on the machine that owns the camera (the robot-side Pi in a split setup)." >&2
    exit 1
  fi
  echo "Camera calibration: use the printed checkerboard target -- scripts/checkerboard.py 8 6 25"
  echo "and keep the same resolution, lens focus, and mounting you use in operation."
  scripts/calibrate-camera.sh "$CAMERA_FILE"
  echo
  if ! camera_calibration_valid; then
    echo "$0: camera calibration is still missing or invalid: $CAMERA_FILE" >&2
    exit 1
  fi
  echo "Saved $CAMERA_FILE"
}

calibrate_wrist() {
  # Intrinsics are optional for navigation, but required before treating the wrist
  # image as a calibrated camera in RViz, recording, or manipulation perception.
  stop_stack_if_running
  stop_cameras_service
  if [ ! -e "${LEKIWI_WRIST:-$(first_match '/dev/v4l/by-id/*JYU2C*-video-index0')}" ]; then
    echo "$0: the wrist camera is not attached to this machine." >&2
    echo "Run this command on the machine that owns the camera (the robot-side Pi in a split setup)." >&2
    exit 1
  fi
  echo "Wrist-camera calibration: point the wrist camera at the printed checkerboard"
  echo "(scripts/checkerboard.py 8 6 25). Keep its normal 352x288 operating mode."
  echo "Hold the arm or use a stable support; this calibration never commands motion."
  scripts/calibrate-camera.sh --wrist "$WRIST_CAMERA_FILE"
  echo
  if ! camera_calibration_valid "$WRIST_CAMERA_FILE"; then
    echo "$0: wrist-camera calibration is still missing or invalid: $WRIST_CAMERA_FILE" >&2
    exit 1
  fi
  echo "Saved $WRIST_CAMERA_FILE"
}

calibrate_height() {
  stop_stack_if_running
  camera_calibration_valid || {
    echo "$0: front-camera calibration is missing or invalid: $CAMERA_FILE" >&2
    echo "Run scripts/calibrate.sh camera first." >&2
    exit 1
  }
  echo "Height calibration: the robot will not move. Lay the checkerboard flat on the"
  echo "floor in front of the camera, fully visible and level."
  echo
  printf 'Using camera calibration at %s\n' "$CAMERA_FILE"
  read -r -p "Press Enter when the board is in place; I will start the stack and measure: " _

  scripts/up.sh
  echo "Measuring camera height and pitch (up to 120 seconds; keep the board still)."
  rm -f "$LOGS/height.log"
  setsid ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true \
    -r image:=/camera/front/image_raw -r camera_info:=/camera/front/camera_info \
    > "$LOGS/height.log" 2>&1 &
  local calibration_pid=$!
  if ! wait_for 120 grep -q 'Saved launch calibration' "$LOGS/height.log"; then
    # `setsid` gives ros2 and its node a private process group. Signal the group so
    # the wrapper cannot exit while its child keeps `wait` blocked forever.
    kill -INT -- "-$calibration_pid" 2>/dev/null || true
    wait "$calibration_pid" 2>/dev/null || true
    echo "$0: did not detect a checkerboard measurement within 120 seconds." >&2
    echo "See $LOGS/height.log; keep the whole board flat and in view, then retry." >&2
    exit 1
  fi
  kill -INT -- "-$calibration_pid" 2>/dev/null || true
  wait "$calibration_pid" 2>/dev/null || true
  echo
  echo "Saved height/pitch for future scripts/up.sh launches. The full stack remains up."
}

calibrate_wheels() {
  stop_stack_if_running
  echo "Wheel calibration: the robot drives by itself. Put the checkerboard flat in"
  echo "front of the camera, keep the floor clear, and stay near the power switch."
  echo "It needs the stack up, the front camera calibrated, and the board in view:"
  echo "  scripts/up.sh"
  echo "  ros2 run lekiwi_rmf odom_scale.py --axis linear"
  echo "  ros2 run lekiwi_rmf odom_scale.py --axis angular"
  echo
  echo "Each measured result is saved automatically for future scripts/up.sh launches."
}

finish_pose() {
  if camera_calibration_valid; then
    echo
    echo "Stopping the slim calibration stack so the full stack can start clean."
    scripts/ros-stop.sh
    echo
    echo "Starting the full stack (RViz opens) so MoveIt uses the new pose."
    scripts/up.sh
    echo
    echo "If a joint moves opposite in RViz, flip that joint's directions value"
    echo "in $POSE_FILE between 1 and -1, then scripts/up.sh again."
  else
    echo
    echo "Front-camera calibration is missing, so the stack was not started."
    echo "Run scripts/calibrate.sh camera first, then scripts/up.sh."
    echo "Stopping the slim calibration stack so it cannot keep the motor port."
    scripts/ros-stop.sh
  fi
}

case "$mode" in
  auto)
    if motor_calibration_valid; then need_motor=0; else need_motor=1; fi
    if pose_calibration_valid; then need_pose=0; else need_pose=1; fi
    if camera_calibration_valid; then need_camera=0; else need_camera=1; fi
    echo "Calibration scan for this robot (ID $ID):"
    printf '  motor:   %s\n' "$(motor_calibration_valid && echo "already set -- skip" || echo "missing -- will run")"
    printf '  pose:    %s\n' "$(pose_calibration_valid && echo "already set -- skip" || echo "missing -- will run")"
    printf '  camera:  %s\n' "$(camera_calibration_valid && echo "already set -- skip" || echo "missing -- will run")"
    printf '  wrist:   %s\n' "optional; run '$0 wrist' before calibrated wrist perception"
    printf '  height:  %s\n' "run when needed; result is saved"
    printf '  wheels:  %s\n' "run when needed; result is saved"
    if [ "${CALIBRATE_DRYRUN:-0}" = 1 ]; then
      printf 'Dry run mode -- would run: motor=%d pose=%d camera=%d\n' "$need_motor" "$need_pose" "$need_camera"
      exit 0
    fi

    if [[ "$need_motor" = 1 ]] || [[ "$need_camera" = 1 ]] || [[ "$need_pose" = 1 ]]; then
      if calibrating; then
        echo "Stopping the running host and stack so calibration can hold the hardware clean."
        scripts/ros-stop.sh
      fi
    fi

    if [[ "$need_motor" = 1 ]]; then
      echo; calibrate_motor
    fi

    if [[ "$need_camera" = 1 ]]; then
      echo; calibrate_camera
    fi

    if [[ "$need_pose" = 1 ]]; then
      echo; calibrate_pose
      finish_pose
    fi

    echo
    echo "The height and wheel tools need the stack running. Their results are saved to"
    echo "~/.ros/lekiwi_launch_calibration.conf. When ready, once the stack is up:"
    echo "  height:  ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true ..."
    echo "  linear:  ros2 run lekiwi_rmf odom_scale.py --axis linear"
    echo "  angular: ros2 run lekiwi_rmf odom_scale.py --axis angular"
    echo
    echo "Done. Saved calibration results:"
    echo "  motor   $MOTOR_FILE"
    echo "  pose    $POSE_FILE"
    echo "  camera  $CAMERA_FILE"
    echo "  wrist   $WRIST_CAMERA_FILE (optional)"
    service_restore_hint
    ;;
  motor)
    if calibrating; then
      scripts/ros-stop.sh
    fi
    calibrate_motor
    echo
    echo "Motor calibration done. Start the robot with scripts/up.sh"
    service_restore_hint
    ;;
  pose)
    if calibrating; then
      echo "Stopping the running host/stack for the pose capture."
      scripts/ros-stop.sh
    fi
    calibrate_pose
    finish_pose
    ;;
  camera)
    calibrate_camera
    service_restore_hint
    ;;
  wrist)
    calibrate_wrist
    service_restore_hint
    ;;
  height)
    calibrate_height
    ;;
  wheels)
    calibrate_wheels
    ;;
esac
