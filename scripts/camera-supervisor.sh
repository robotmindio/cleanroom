#!/usr/bin/env bash
# Keep one v4l2_camera node alive across a USB reset/re-enumeration.
# v4l2_camera currently continues dequeuing after ENODEV, flooding the log and
# never reopening its device. This wrapper owns that recovery boundary.
set -Eeuo pipefail

usage() {
  echo "usage: $0 --device PATH --name NAME --namespace NS --camera-name NAME --frame FRAME --size '[W, H]' --camera-info-url URL [--heartbeat-interval SEC --heartbeat-timeout SEC --startup-grace SEC]" >&2
  exit 2
}

device=""; node_name=""; namespace=""; camera_name=""; frame=""; size=""; info_url=""; jpeg_quality=""
# A UVC device can remain queryable while its userspace capture loop has wedged.
# Treat receipt of an Image message as the camera heartbeat as well as checking
# the device node. These defaults deliberately leave room for first-frame
# negotiation on slow USB hubs without waiting forever after a real stall.
# ROS graph discovery can briefly lag while RTAB-Map is converting a frame on
# the Pi.  Three seconds made an otherwise healthy UVC feed restart forever.
# These bounds still detect a genuinely wedged camera without destabilising one
# that is merely under load.
heartbeat_interval=10
heartbeat_timeout=8
startup_grace=30
while (( $# )); do
  case "$1" in
    --device) device="${2:-}"; shift 2 ;;
    --name) node_name="${2:-}"; shift 2 ;;
    --namespace) namespace="${2:-}"; shift 2 ;;
    --camera-name) camera_name="${2:-}"; shift 2 ;;
    --frame) frame="${2:-}"; shift 2 ;;
    --size) size="${2:-}"; shift 2 ;;
    --camera-info-url) info_url="${2:-}"; shift 2 ;;
    --jpeg-quality) jpeg_quality="${2:-}"; shift 2 ;;
    --heartbeat-interval) heartbeat_interval="${2:-}"; shift 2 ;;
    --heartbeat-timeout) heartbeat_timeout="${2:-}"; shift 2 ;;
    --startup-grace) startup_grace="${2:-}"; shift 2 ;;
    *) usage ;;
  esac
done
[[ -n $device && -n $node_name && -n $namespace && -n $camera_name && -n $frame && -n $size && -n $info_url ]] || usage
[[ $heartbeat_interval =~ ^[1-9][0-9]*$ && $heartbeat_timeout =~ ^[1-9][0-9]*$ && $startup_grace =~ ^[0-9]+$ ]] || usage

child=""; stopping=0
stop_tree() { # stop_tree <signal> <pid>
  local signal=$1 pid=$2 descendant
  for descendant in $(pgrep -P "$pid" 2>/dev/null || true); do
    stop_tree "$signal" "$descendant"
  done
  kill "-$signal" "$pid" 2>/dev/null || true
}
stop() {
  stopping=1
  [[ -z $child ]] || stop_child "shutdown"
}
trap stop INT TERM

stop_child() { # stop_child <reason>; bounded TERM -> KILL for a wedged camera
  local reason=$1 deadline
  [[ -n $child ]] || return 0
  echo "$node_name: stopping camera ($reason)" >&2
  stop_tree TERM "$child"
  deadline=$((SECONDS + 5))
  while kill -0 "$child" 2>/dev/null && (( SECONDS < deadline )); do
    sleep 1
  done
  if kill -0 "$child" 2>/dev/null; then
    echo "$node_name: camera ignored TERM; sending KILL" >&2
    stop_tree KILL "$child"
  fi
  wait "$child" 2>/dev/null || true
  child=""
}

# A by-id path disappears during a real USB reset. Querying capabilities also
# catches drivers that leave the symlink behind while returning ENODEV.
device_healthy() {
  [ -e "$device" ] || return 1
  command -v v4l2-ctl >/dev/null 2>&1 || return 0
  timeout 2 v4l2-ctl --device "$device" --all >/dev/null 2>&1
}

topic_healthy() {
  # `echo --once` verifies that a fresh Image message crosses the ROS graph;
  # topic discovery alone does not prove v4l2_camera is dequeuing frames.
  timeout "$heartbeat_timeout" ros2 topic echo --once --qos-reliability reliable \
    "${namespace}/image_raw" >/dev/null 2>&1
}

driver_camera_name() {
  # v4l2_camera does not expose a `camera_name` parameter.  It derives the
  # CameraInfoManager identity from the V4L card name instead (for example,
  # "GENERAL WEBCAM: GENERAL WEBCAM" -> "general_webcam:_general_webcam").
  # Calibration tooling uses logical
  # names such as `lekiwi_front`, so adapt a private runtime copy below rather
  # than rejecting calibrated intrinsics or overwriting the user's source YAML.
  command -v v4l2-ctl >/dev/null 2>&1 || return 1
  v4l2-ctl --device "$device" --all 2>/dev/null |
    sed -nE 's/^[[:space:]]*Card type[[:space:]]*:[[:space:]]*(.*)$/\1/p' |
    head -n 1 |
    tr '[:upper:]' '[:lower:]' |
    sed -E 's/[[:space:]]+/_/g; s/^_+//; s/_+$//'
}

camera_info_url_for_driver() {
  local source_path required_name runtime_dir runtime_path temporary image_width image_height
  [[ $info_url == file://* ]] || { printf '%s' "$info_url"; return 0; }
  source_path=${info_url#file://}
  [[ -r $source_path ]] || { printf '%s' "$info_url"; return 0; }
  required_name=$(driver_camera_name) || { printf '%s' "$info_url"; return 0; }
  [[ -n $required_name ]] || { printf '%s' "$info_url"; return 0; }
  if [[ ! $size =~ ^\[[[:space:]]*([1-9][0-9]*)[[:space:]]*,[[:space:]]*([1-9][0-9]*)[[:space:]]*\]$ ]]; then
    echo "$node_name: invalid requested image size $size; using calibration unchanged" >&2
    printf '%s' "$info_url"
    return 0
  fi
  image_width=${BASH_REMATCH[1]}
  image_height=${BASH_REMATCH[2]}

  runtime_dir="${LEKIWI_RUNTIME_DIR:-${XDG_RUNTIME_DIR:-/tmp}/lekiwi-runtime}/camera-info"
  (umask 077 && mkdir -p "$runtime_dir")
  runtime_path="$runtime_dir/${node_name}.yaml"
  temporary=$(mktemp "$runtime_dir/.${node_name}.XXXXXX")
  # The physical camera supports a low-resolution stream even when it was
  # calibrated at 640x480.  Scale the focal lengths/principal point in the
  # private copy so every consumer sees internally consistent Image and
  # CameraInfo messages.  This avoids both a costly full-resolution pipeline
  # on the 4 GB Pi and incorrect obstacle ranges from stale intrinsics.
  if python3 - "$source_path" "$temporary" "$required_name" "$image_width" "$image_height" <<'PY'
import sys

import yaml

source, destination, camera_name, target_width, target_height = sys.argv[1:]
target_width, target_height = int(target_width), int(target_height)
with open(source, encoding="utf-8") as input_file:
    calibration = yaml.safe_load(input_file)
if not isinstance(calibration, dict):
    raise ValueError("camera calibration is not a YAML mapping")
source_width = int(calibration.get("image_width", 0))
source_height = int(calibration.get("image_height", 0))
if source_width <= 0 or source_height <= 0:
    raise ValueError("camera calibration has invalid dimensions")
matrix = calibration.get("camera_matrix", {}).get("data")
if not isinstance(matrix, list) or len(matrix) != 9:
    raise ValueError("camera calibration has no 3x3 camera matrix")
scale_x, scale_y = target_width / source_width, target_height / source_height
matrix[0] *= scale_x
matrix[2] *= scale_x
matrix[4] *= scale_y
matrix[5] *= scale_y
projection = calibration.get("projection_matrix", {}).get("data")
if isinstance(projection, list) and len(projection) == 12:
    for index in (0, 2, 3):
        projection[index] *= scale_x
    for index in (5, 6, 7):
        projection[index] *= scale_y
calibration["camera_name"] = camera_name
calibration["image_width"] = target_width
calibration["image_height"] = target_height
with open(destination, "w", encoding="utf-8") as output_file:
    yaml.safe_dump(calibration, output_file, sort_keys=False)
PY
  then
    mv -f -- "$temporary" "$runtime_path"
    echo "$node_name: using calibrated ${image_width}x${image_height} intrinsics for V4L2 camera $required_name" >&2
    printf 'file://%s' "$runtime_path"
  else
    rm -f -- "$temporary"
    echo "$node_name: unable to adapt calibration in $source_path; using it unchanged" >&2
    printf '%s' "$info_url"
  fi
}

while (( ! stopping )); do
  until device_healthy; do
    (( stopping )) && exit 0
    echo "$node_name: camera unavailable ($device); waiting for USB reconnect" >&2
    sleep 1
  done
  echo "$node_name: starting camera on $device" >&2
  effective_info_url=$(camera_info_url_for_driver)
  camera_args=(ros2 run v4l2_camera v4l2_camera_node --ros-args \
    -r __node:="$node_name" -r __ns:="$namespace" \
    -p video_device:="$device" -p camera_info_url:="$effective_info_url" \
    -p camera_frame_id:="$frame" \
    -p pixel_format:=YUYV -p output_encoding:=rgb8 -p image_size:="$size")
  # ROS 2 parameter names may be dot-qualified but cannot begin with a dot.
  # A leading dot aborts v4l2_camera during argument parsing, leaving the
  # supervisor to retry forever without publishing a front-camera frame.
  [[ -z $jpeg_quality ]] || camera_args+=(-p "image_raw.compressed.jpeg_quality:=$jpeg_quality")
  "${camera_args[@]}" &
  child=$!
  disconnected=0
  started=$SECONDS
  last_heartbeat=$SECONDS
  while kill -0 "$child" 2>/dev/null; do
    if ! device_healthy; then
      disconnected=1
      echo "$node_name: USB camera disconnected; restarting when it returns" >&2
      stop_child "USB disconnect"
      break
    fi
    if (( SECONDS - started >= startup_grace && SECONDS - last_heartbeat >= heartbeat_interval )); then
      last_heartbeat=$SECONDS
      if ! topic_healthy; then
        disconnected=1
        echo "$node_name: no image heartbeat on ${namespace}/image_raw; restarting" >&2
        stop_child "missing image heartbeat"
        break
      fi
    fi
    sleep 1
  done
  [[ -z $child ]] || { wait "$child" 2>/dev/null || true; child=""; }
  (( stopping )) && exit 0
  (( disconnected )) || echo "$node_name: camera process exited; retrying" >&2
  sleep 2
done
