#!/usr/bin/env bash
# Run every LeKiwi calibration that is still missing. Motor and pose calibration
# run on the machine that owns the motor bus; camera calibration runs on the
# machine that owns that USB camera (the robot-side Pi in a split deployment).
# Calibrations that already have a saved result are skipped. Physical steps stay
# manual; the script shows what to do at each prompt.
#
#   scripts/calibrate.sh                     run whichever required calibrations are missing
#   scripts/calibrate.sh motor               redo Lerobot's servo-range calibration
#   scripts/calibrate.sh pose               capture the SO-101 new-calibration zero pose
#   scripts/calibrate.sh camera             redo the front-camera intrinsics
#   scripts/calibrate.sh wrist              redo the wrist-camera intrinsics
#   scripts/calibrate.sh height             redo the laser height/pitch (free_space)
#   scripts/calibrate.sh wheels             redo the odometry scale measurement
#
# What each step saves or produces:
#   motor      Lerobot's cache  -> ~/.cache/huggingface/lerobot/calibration/robots/lekiwi/<id>.json
#   pose       SO-101 zero mapping -> ~/.ros/lekiwi_arm_calibration.json
#   camera     front intrinsics -> ~/.ros/camera_info/lekiwi_front.yaml
#   wrist      wrist intrinsics -> ~/.ros/camera_info/lekiwi_wrist.yaml
#   height     saves camera_height/camera_pitch to ~/.ros/lekiwi_launch_calibration.conf
#   wheels     saves xy/yaw_velocity_scale to ~/.ros/lekiwi_launch_calibration.conf
#
#   The wheel/height tools save their measured values to the machine-local launch file.
#   Set CALIBRATE_DRYRUN=1 to only print what auto would do (no services touched).
#   Boot services stopped for a step are started again when the script exits,
#   whether it finished, failed, or was interrupted.
set -Eeuo pipefail

cd "$(dirname "$0")/.."
# shellcheck source=/dev/null
source scripts/lib/runtime-common.sh
die() { echo "$0: $*" >&2; exit 1; }
# shellcheck source=/dev/null
source scripts/lib/service-install-common.sh

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
  lekiwi_safety_ports_listening \
    && pgrep -f '[t]orque-host\.py|[l]erobot\.robots\.lekiwi\.lekiwi_host' >/dev/null
}

legacy_host_up() {
  # Detect an older stock host only so calibration tears it down before it
  # tries to own the serial bus. It is never accepted as a safety-capable host.
  lekiwi_motion_port_listening \
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
import math
import sys

try:
    with open(sys.argv[1]) as source:
        calibration = json.load(source)
    zeroes = calibration["zero_positions"]
    assert calibration.get("model") == "so101_new_calib"
    directions = calibration["directions"]
    required = {
        "arm_shoulder_pan", "arm_shoulder_lift", "arm_elbow_flex",
        "arm_wrist_flex", "arm_wrist_roll", "arm_gripper",
    }
    assert required <= set(zeroes) and required <= set(directions)
    assert all(type(zeroes[name]) in (int, float) and math.isfinite(zeroes[name]) for name in required)
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

# The boot services own the hardware by design: lekiwi-host.service keeps the motor
# host alive on the bus and lekiwi-cameras.service publishes the cameras. Both would
# block -- or race -- a calibration run, so stop whichever one the step needs and
# start exactly those again on exit.
stopped_units=()

stop_service() { # stop_service <unit> <what it owns>
  unit_active "$1" || return 0
  echo "Stopping $1 ($2)."
  if ! as_root systemctl stop "$1"; then
    echo "$0: could not stop $1; run 'sudo systemctl stop $1' and retry" >&2
    exit 1
  fi
  stopped_units+=("$1")
  sleep 2  # let the service's processes release the device
}

restore_services() {
  local unit
  for unit in "${stopped_units[@]}"; do
    echo "Starting $unit again."
    as_root systemctl start "$unit" || echo "$0: could not start $unit; run 'sudo systemctl start $unit'" >&2
  done
}
trap restore_services EXIT
trap 'exit 130' INT
trap 'exit 143' TERM HUP

calibrate_motor() {
  stop_service lekiwi-host.service "it owns the motor bus"
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
  # The live driver's raw topic works on both local and split deployments and
  # is unaffected by its currently loaded pose calibration.
  if ! ros2 topic info /arm/raw_joint_states 2>/dev/null | grep -q 'Publisher count: 1'; then
    echo "$0: start the updated repository stack first; /arm/raw_joint_states needs one publisher." >&2
    exit 1
  fi
  local pose_backup=""
  local capture_args=()

  echo
  echo "Disarm using the existing operator control before manually positioning the arm."
  echo "Support it in the generated SO-101 new-calibration zero pose, including the gripper."
  echo "Use the reference in urdf/README.md; the old folded pose is incorrect."
  echo "Keep holding it aligned; press Enter when ready:"
  read -r _

  if [ -s "$POSE_FILE" ]; then
    pose_backup=$(mktemp "${POSE_FILE}.XXXXXX.bak")
    mv "$POSE_FILE" "$pose_backup"
    capture_args=(--directions-from "$pose_backup")
    echo "Moved existing $POSE_FILE to $pose_backup."
  fi
  echo "Capturing the held pose."
  if ! ros2 run lekiwi_rmf arm_calibration.py "${capture_args[@]}" > "$LOGS/pose.log" 2>&1; then
    echo "$0: pose capture failed:" >&2
    cat "$LOGS/pose.log" >&2
    # A failed capture must not silently leave the robot using uncalibrated
    # zeros. Put the last known calibration back.
    if [ -n "$pose_backup" ] && [ -s "$pose_backup" ]; then
      mv "$pose_backup" "$POSE_FILE"
      echo "Restored the previous arm calibration." >&2
    fi
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
  stop_service lekiwi-cameras.service "it owns the cameras"
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
  stop_service lekiwi-cameras.service "it owns the cameras"
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
  echo "Restart the driver through your repository launch/deploy workflow to apply $POSE_FILE."
  echo "Then verify each joint's direction and several poses before executing a trajectory."
  echo "Pose capture has not changed torque or restarted the running services."
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

    if [[ "$need_motor" = 1 ]] || [[ "$need_camera" = 1 ]]; then
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
      echo "Pose calibration needs the updated stack running and the arm held in its reference pose."
      echo "Start the stack through your deployment workflow, then run: scripts/calibrate.sh pose"
    fi

    echo
    echo "The height and wheel tools need the stack running. Their results are saved to"
    echo "$HOME/.ros/lekiwi_launch_calibration.conf. When ready, once the stack is up:"
    echo "  height:  ros2 run lekiwi_rmf free_space.py --ros-args -p calibrate:=true ..."
    echo "  linear:  ros2 run lekiwi_rmf odom_scale.py --axis linear"
    echo "  angular: ros2 run lekiwi_rmf odom_scale.py --axis angular"
    echo
    echo "Done. Saved calibration results:"
    echo "  motor   $MOTOR_FILE"
    echo "  pose    $POSE_FILE"
    echo "  camera  $CAMERA_FILE"
    echo "  wrist   $WRIST_CAMERA_FILE (optional)"
    ;;
  motor)
    if calibrating; then
      scripts/ros-stop.sh
    fi
    calibrate_motor
    echo
    echo "Motor calibration done. Start the robot with scripts/up.sh"
    ;;
  pose)
    calibrate_pose
    finish_pose
    ;;
  camera)
    calibrate_camera
    ;;
  wrist)
    calibrate_wrist
    ;;
  height)
    calibrate_height
    ;;
  wheels)
    calibrate_wheels
    ;;
esac
