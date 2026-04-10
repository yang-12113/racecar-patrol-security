#!/bin/bash
set -e

STATE_FILE=/tmp/face_follow_state.json
ARBITER_STATE_FILE=/tmp/drive_arbiter_state.json
FOLLOW_CTRL_STATE_FILE=/tmp/follow_controller_state.json
FOLLOW_CTRL_EVENT_LOG=/tmp/follow_controller_events.jsonl
ARBITER_EVENT_LOG=/tmp/drive_arbiter_events.jsonl
PROTOCOL_INIT_PY=/root/face_tools/initialize_protocol_states.py
ROS_PY=/usr/bin/python3
ACL_PY=/usr/local/miniconda3/bin/python
CTRL_TARGET_MODE=any
CTRL_TARGET_NAME=""
MODE="normal"
YOLO_ARGS=()
MODE_DEFAULT_ARGS=()
CLEANUP_ENABLED=0
FORCE_HEADLESS=0
EVENT_LOG=""
CTRL_EXTRA_ARGS=()
CAR_STACK_PID=""
CTRL_CMD_TOPIC="/track_cmd_vel"
START_ARBITER=1

usage() {
  cat <<'EOF'
usage: bash run_face_follow_stack.sh [--mode normal|fast|profile|competition] [forwarded yolo args...]

mode defaults:
  normal  -> baseline behavior
  fast    -> add --fast-mode
  profile -> add --fast-mode --profile-every 30 --no-show --no-record --event-log /tmp/face_follow_events.jsonl
  competition -> fixed competition-safe params + event log

other arguments are forwarded to yolo_face_track_alarm.py
EOF
}

publish_stop() {
  if command -v ros2 >/dev/null 2>&1; then
    timeout 3s ros2 topic pub --once /car_cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 1500.0}, angular: {z: 90.0}}" >/dev/null 2>&1 || true
    sleep 0.1
    timeout 3s ros2 topic pub --once /car_cmd_vel geometry_msgs/msg/Twist \
      "{linear: {x: 1500.0}, angular: {z: 90.0}}" >/dev/null 2>&1 || true
  fi
  return 0
}

initialize_protocol_states() {
  "$ROS_PY" "$PROTOCOL_INIT_PY" \
    --follow-input-state-file "$STATE_FILE" \
    --follow-controller-state-file "$FOLLOW_CTRL_STATE_FILE" \
    --drive-arbiter-state-file "$ARBITER_STATE_FILE" \
    --arbiter-mode ALERT_TRACK >/dev/null 2>&1 || {
      printf '{}' > "$STATE_FILE" 2>/dev/null || true
      printf '{}' > "$ARBITER_STATE_FILE" 2>/dev/null || true
      printf '{}' > "$FOLLOW_CTRL_STATE_FILE" 2>/dev/null || true
    }
}

terminate_pid() {
  local pid="$1"
  local sig="${2:-TERM}"
  local wait_loops="${3:-20}"
  if [ -z "$pid" ]; then
    return 0
  fi
  kill "-$sig" "$pid" 2>/dev/null || kill "$pid" 2>/dev/null || true
  local i=0
  while kill -0 "$pid" 2>/dev/null; do
    i=$((i + 1))
    if [ "$i" -ge "$wait_loops" ]; then
      kill -KILL "$pid" 2>/dev/null || true
      break
    fi
    sleep 0.1
  done
  wait "$pid" 2>/dev/null || true
}

cleanup() {
  if [ "${CLEANUP_ENABLED:-0}" != "1" ]; then
    return 0
  fi
  if [ -n "$YOLO_PID" ]; then
    terminate_pid "$YOLO_PID" TERM 25
  fi
  if [ -n "$CTRL_PID" ]; then
    terminate_pid "$CTRL_PID" TERM 20
  fi
  if [ -n "$ARB_PID" ]; then
    sleep 0.4
    terminate_pid "$ARB_PID" TERM 20
  fi
  if [ -n "$CAR_STACK_PID" ]; then
    terminate_pid "$CAR_STACK_PID" TERM 20
  fi
  pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
  pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
  pkill -f "/root/face_tools/drive_arbiter.py" 2>/dev/null || true
  initialize_protocol_states
  : > "$FOLLOW_CTRL_EVENT_LOG" 2>/dev/null || true
  : > "$ARBITER_EVENT_LOG" 2>/dev/null || true
  publish_stop || true
}

ensure_car_stack() {
  if pgrep -f "racecar_driver_node" >/dev/null 2>&1; then
    return 0
  fi
  echo "[INFO] racecar_driver_node not running, launching Run_car.launch.py"
  ros2 launch racecar Run_car.launch.py >/tmp/run_car_follow.log 2>&1 &
  CAR_STACK_PID=$!
  sleep 5
}

trap cleanup EXIT INT TERM

for arg in "$@"; do
  if [ "$arg" = "--help" ] || [ "$arg" = "-h" ]; then
    usage
    exit 0
  fi
done

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi
source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /usr/local/Ascend/nnae/set_env.sh
CLEANUP_ENABLED=1

while [ $# -gt 0 ]; do
  case "$1" in
    --mode)
      MODE="$2"
      shift 2
      ;;
    --mode=*)
      MODE="${1#--mode=}"
      shift 1
      ;;
    --follow-policy)
      if [ -n "$2" ]; then
        if [ "$2" = "unknown" ]; then
          CTRL_TARGET_MODE=unknown
          CTRL_TARGET_NAME=""
        elif [ "$2" = "owner" ] || [ "$2" = "named" ] || [ "$2" = "none" ]; then
          CTRL_TARGET_MODE=any
          CTRL_TARGET_NAME=""
        fi
      fi
      YOLO_ARGS+=("$1" "$2")
      shift 2
      ;;
    --follow-policy=unknown)
      CTRL_TARGET_MODE=unknown
      CTRL_TARGET_NAME=""
      YOLO_ARGS+=("$1")
      shift 1
      ;;
    --follow-policy=owner|--follow-policy=named|--follow-policy=none)
      CTRL_TARGET_MODE=any
      CTRL_TARGET_NAME=""
      YOLO_ARGS+=("$1")
      shift 1
      ;;
    --follow-name)
      if [ -n "$2" ]; then
        CTRL_TARGET_NAME="$2"
      fi
      YOLO_ARGS+=("$1" "$2")
      shift 2
      ;;
    --follow-name=*)
      CTRL_TARGET_NAME="${1#--follow-name=}"
      YOLO_ARGS+=("$1")
      shift 1
      ;;
    --event-log)
      EVENT_LOG="$2"
      YOLO_ARGS+=("$1" "$2")
      shift 2
      ;;
    --event-log=*)
      EVENT_LOG="${1#--event-log=}"
      YOLO_ARGS+=("$1")
      shift 1
      ;;
    *)
      YOLO_ARGS+=("$1")
      shift 1
      ;;
  esac
done

case "$MODE" in
  normal)
    MODE_DEFAULT_ARGS=()
    ;;
  fast)
    MODE_DEFAULT_ARGS=(--fast-mode)
    ;;
  profile)
    MODE_DEFAULT_ARGS=(--fast-mode --profile-every 30 --no-show --no-record)
    ;;
  competition)
    MODE_DEFAULT_ARGS=(
      --detector-mode local
      --camera /dev/v4l/by-id/usb-LRCP_HD60fps_USB_2.0_Camera-video-index0
      --width 640
      --height 480
      --camera-fps 30
      --camera-fourcc YUYV
      --no-show
      --infer-interval 1
      --face-interval 12
      --face-known-interval 24
      --track-max-lost 30
      --track-low-conf 0.12
      --follow-max-lost 8
      --follow-unknown-min-frames 1
      --unknown-min-conf 0.40
      --unknown-face-evidence-max-gap 30
      --unknown-face-evidence-score 0.20
      --face-det-thres 0.85
      --match-thres 0.40
      --owner-accept-score 0.78
      --owner-confirm-hits 6
      --known-hold-score 0.65
      --min-person-width 24
      --min-person-height 48
      --min-person-area 1500
      --stream-port 8081
      --stream-max-fps 12
      --stream-jpeg-quality 75
      --event-log /tmp/face_follow_events_competition.jsonl
      --no-record
    )
    CTRL_EXTRA_ARGS+=(
      --allow-stale-scan-motion
      --allow-unknown-without-face-evidence
      --max-speed 1524
      --speed-gain 55
      --desired-area 0.16
      --area-tol 0.03
      --obstacle-slow-speed 1518
      --obstacle-avoid-speed 1514
      --turn-slow-down 4
    )
    CTRL_CMD_TOPIC="/car_cmd_vel"
    START_ARBITER=0
    ;;
  *)
    echo "unknown mode: $MODE"
    echo "use --mode normal|fast|profile|competition"
    exit 1
    ;;
esac

if [ "$MODE" = "profile" ] && [ -z "$EVENT_LOG" ]; then
  MODE_DEFAULT_ARGS+=(--event-log /tmp/face_follow_events.jsonl)
fi

if [ -z "${DISPLAY:-}" ]; then
  FORCE_HEADLESS=1
fi

if [ "$FORCE_HEADLESS" = "1" ]; then
  HAS_NO_SHOW=0
  for arg in "${MODE_DEFAULT_ARGS[@]}" "${YOLO_ARGS[@]}"; do
    if [ "$arg" = "--no-show" ]; then
      HAS_NO_SHOW=1
      break
    fi
  done
  if [ "$HAS_NO_SHOW" != "1" ]; then
    MODE_DEFAULT_ARGS+=(--no-show)
    echo "[INFO] DISPLAY is not set, forcing --no-show"
  fi
fi

pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
pkill -f "/root/face_tools/drive_arbiter.py" 2>/dev/null || true
publish_stop || true
sleep 0.2

initialize_protocol_states
: > "$FOLLOW_CTRL_EVENT_LOG"
: > "$ARBITER_EVENT_LOG"

ensure_car_stack

if [ "$START_ARBITER" = "1" ]; then
  "$ROS_PY" /root/face_tools/drive_arbiter.py \
    --mode-topic "" \
    --default-mode ALERT_TRACK \
    --output-topic /car_cmd_vel \
    --track-topic /track_cmd_vel \
    --state-topic /drive_arbiter/state \
    --status-file "$ARBITER_STATE_FILE" \
    --event-log "$ARBITER_EVENT_LOG" \
    &
  ARB_PID=$!
else
  ARB_PID=""
fi

"$ROS_PY" /root/face_tools/face_follow_controller.py \
  --state-file "$STATE_FILE" \
  --state-topic /follow_controller/state \
  --status-file "$FOLLOW_CTRL_STATE_FILE" \
  --event-log "$FOLLOW_CTRL_EVENT_LOG" \
  --target-mode "$CTRL_TARGET_MODE" \
  --target-name "$CTRL_TARGET_NAME" \
  --cmd-topic "$CTRL_CMD_TOPIC" \
  "${CTRL_EXTRA_ARGS[@]}" \
  &
CTRL_PID=$!

"$ACL_PY" /root/face_tools/yolo_face_track_alarm.py \
  "${MODE_DEFAULT_ARGS[@]}" \
  --state-file "$STATE_FILE" \
  --follow-name owner \
  "${YOLO_ARGS[@]}" &
YOLO_PID=$!
wait "$YOLO_PID"
