#!/bin/bash
set -eo pipefail

STATE_FILE=/tmp/face_follow_state.json
SUPERVISOR_STATUS_FILE=/tmp/patrol_supervisor_state.json
ARBITER_STATE_FILE=/tmp/drive_arbiter_state.json
FOLLOW_CTRL_STATE_FILE=/tmp/follow_controller_state.json
FOLLOW_CTRL_EVENT_LOG=/tmp/follow_controller_events.jsonl
ARBITER_EVENT_LOG=/tmp/drive_arbiter_events.jsonl
PROTOCOL_INIT_PY=/root/face_tools/initialize_protocol_states.py
WAYPOINTS_FILE="/root/face_tools/patrol_waypoints_demo.json"
WAYPOINTS_FILE_SET=""
MAP_FILE="/home/racecar/src/racecar/map/test_map.yaml"
NAV_PARAMS_FILE="/home/racecar/src/racecar/config/nav/navigation.yaml"
NAV_STACK_SCRIPT="/root/face_tools/run_nav_stack_with_params.sh"
FACE_TOOLS_DIR="/root/face_tools"
ROS_PY=/usr/bin/python3
ACL_PY=/usr/local/miniconda3/bin/python
INIT_X=""
INIT_Y=""
INIT_YAW=""
PRECHECK_SKIP=0
PRECHECK_TIMEOUT="12"
PRECHECK_RETRIES="4"
PRECHECK_RETRY_GAP="8"
INIT_WAIT_SUBS_TIMEOUT="25"
INITIALPOSE_SUB_TIMEOUT="60"
LOCALIZATION_TIMEOUT="60"
LOCALIZATION_REPUBLISH_INTERVAL="6"
NAV_STACK_SETTLE_SECONDS="35"
POST_INITPOSE_SETTLE_SECONDS="8"
LOCALIZATION_WAIT_ENABLED=0
MAX_FOLLOW_SECONDS="25"
MAX_FOLLOW_DISTANCE="3.0"
MODE="normal"
FAST_MODE=0
PROFILE_EVERY=""
PATROL_ONLY=0
STARTUP_SMOKE_ONLY=0
EVENT_LOG=""
SUPERVISOR_EVENT_LOG=""
CAMERA_ARGS=()
NAV_STACK_ARGS=()
NAV_PID=""
ARBITER_PID=""
YOLO_PID=""
SUP_PID=""
CLEANUP_ENABLED=0
FORCE_HEADLESS=0

usage() {
  cat <<'EOF'
usage: bash run_intruder_patrol_demo_full_stack.sh [options]

core options:
  --waypoints FILE
  --map FILE
  --nav-params FILE
  --max-follow-seconds SEC
  --max-follow-distance METERS
  --patrol-only
  --startup-smoke-only
  --enable-localization-wait
  --mode normal|fast|profile
  --event-log PATH
  --supervisor-event-log PATH
  --no-rviz
  --rviz-config FILE

yolo passthrough:
  any unknown option is forwarded to yolo_face_track_alarm.py
EOF
}

publish_stop() {
  return 0
}

initialize_protocol_states() {
  "$ROS_PY" "$PROTOCOL_INIT_PY" \
    --follow-input-state-file "$STATE_FILE" \
    --follow-controller-state-file "$FOLLOW_CTRL_STATE_FILE" \
    --drive-arbiter-state-file "$ARBITER_STATE_FILE" \
    --supervisor-state-file "$SUPERVISOR_STATUS_FILE" \
    --arbiter-mode SAFE_STOP >/dev/null 2>&1 || {
      printf '{}' > "$STATE_FILE" 2>/dev/null || true
      printf '{}' > "$SUPERVISOR_STATUS_FILE" 2>/dev/null || true
      printf '{}' > "$ARBITER_STATE_FILE" 2>/dev/null || true
      printf '{}' > "$FOLLOW_CTRL_STATE_FILE" 2>/dev/null || true
    }
}

extract_last_json_line() {
  local src="$1"
  local dst="$2"
  "$ROS_PY" - "$src" "$dst" <<'PY'
import json
import pathlib
import sys

src = pathlib.Path(sys.argv[1])
dst = pathlib.Path(sys.argv[2])
last = None
if src.exists():
    for line in src.read_text(encoding="utf-8", errors="replace").splitlines():
        line = line.strip()
        if not line:
            continue
        try:
            last = json.loads(line)
        except Exception:
            continue
if last is None:
    sys.exit(1)
dst.write_text(json.dumps(last, ensure_ascii=False), encoding="utf-8")
PY
}

json_overall_ok() {
  local src="$1"
  "$ROS_PY" - "$src" <<'PY'
import json
import pathlib
import sys

src = pathlib.Path(sys.argv[1])
if not src.exists():
    raise SystemExit(1)
obj = json.loads(src.read_text(encoding="utf-8", errors="replace"))
raise SystemExit(0 if bool(obj.get("overall_ok")) else 1)
PY
}

terminate_pid() {
  local pid="$1"
  local sig="${2:-TERM}"
  local wait_loops="${3:-25}"
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

derive_init_pose_from_nav_params() {
  if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
    return 0
  fi
  if [ ! -f "$NAV_PARAMS_FILE" ]; then
    return 0
  fi
  local parsed
  parsed=$($ROS_PY - "$NAV_PARAMS_FILE" <<'PY'
import re, sys
text = open(sys.argv[1], 'r', encoding='utf-8').read()
patterns = {
    'x': r'initial_pose_x:\s*([-+]?\d+(?:\.\d+)?)',
    'y': r'initial_pose_y:\s*([-+]?\d+(?:\.\d+)?)',
    'yaw': r'initial_pose_a:\s*([-+]?\d+(?:\.\d+)?)',
}
vals = []
for key in ('x', 'y', 'yaw'):
    m = re.search(patterns[key], text)
    vals.append(m.group(1) if m else '')
print('|'.join(vals))
PY
)
  local px py pyaw
  IFS='|' read -r px py pyaw <<EOF
$parsed
EOF
  if [ -z "$INIT_X" ] && [ -n "$px" ]; then INIT_X="$px"; fi
  if [ -z "$INIT_Y" ] && [ -n "$py" ]; then INIT_Y="$py"; fi
  if [ -z "$INIT_YAW" ] && [ -n "$pyaw" ]; then INIT_YAW="$pyaw"; fi
}

publish_initial_pose_once() {
  if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_YAW" ]; then
    return 0
  fi
  if ! wait_initialpose_subscriber "$INITIALPOSE_SUB_TIMEOUT"; then
    echo "[WARN] /initialpose subscriber not ready within ${INITIALPOSE_SUB_TIMEOUT}s"
  fi
  "$ROS_PY" /root/face_tools/publish_initial_pose.py \
    --x "$INIT_X" \
    --y "$INIT_Y" \
    --yaw "$INIT_YAW" \
    --wait-subs-timeout "$INIT_WAIT_SUBS_TIMEOUT" \
    > /tmp/patrol_init_pose.log 2>&1
}

wait_initialpose_subscriber() {
  local timeout_s="$1"
  local deadline=$((SECONDS + timeout_s))
  while [ $SECONDS -lt $deadline ]; do
    if ros2 topic info /initialpose -v 2>/dev/null | grep -Eq 'Subscription count:[[:space:]]*[1-9]'; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_localization_ready() {
  if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_YAW" ]; then
    return 0
  fi
  rm -f /tmp/patrol_localization_ready.raw.log /tmp/patrol_localization_ready.stderr.log /tmp/patrol_localization_ready.json
  if ! wait_initialpose_subscriber "$INITIALPOSE_SUB_TIMEOUT"; then
    echo "[WARN] /initialpose subscriber not ready before localization wait"
  fi
  "$ROS_PY" /root/face_tools/wait_localization_ready.py \
    --x "$INIT_X" \
    --y "$INIT_Y" \
    --yaw "$INIT_YAW" \
    --timeout "$LOCALIZATION_TIMEOUT" \
    --republish-interval "$LOCALIZATION_REPUBLISH_INTERVAL" \
    --publish-wait-subs-timeout "$INIT_WAIT_SUBS_TIMEOUT" \
    > /tmp/patrol_localization_ready.raw.log 2> /tmp/patrol_localization_ready.stderr.log || true
  extract_last_json_line /tmp/patrol_localization_ready.raw.log /tmp/patrol_localization_ready.json || return 1
  json_overall_ok /tmp/patrol_localization_ready.json
}

run_preflight_with_retries() {
  local attempt=1
  local max_attempts
  max_attempts=$(printf "%d" "$PRECHECK_RETRIES" 2>/dev/null || echo 1)
  if [ "$max_attempts" -lt 1 ]; then
    max_attempts=1
  fi
  while [ "$attempt" -le "$max_attempts" ]; do
    rm -f /tmp/patrol_preflight.raw.log /tmp/patrol_preflight.stderr.log /tmp/patrol_preflight.json
    "$ROS_PY" /root/face_tools/patrol_preflight_check.py \
      --waypoints-file "$WAYPOINTS_FILE" \
      --map-yaml "$MAP_FILE" \
      --timeout "$PRECHECK_TIMEOUT" \
      > /tmp/patrol_preflight.raw.log 2> /tmp/patrol_preflight.stderr.log || true
    if extract_last_json_line /tmp/patrol_preflight.raw.log /tmp/patrol_preflight.json && \
       json_overall_ok /tmp/patrol_preflight.json; then
      return 0
    fi
    if [ "$attempt" -lt "$max_attempts" ] && [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
      publish_initial_pose_once || true
      sleep "$PRECHECK_RETRY_GAP"
    fi
    attempt=$((attempt + 1))
  done
  return 1
}

cleanup() {
  if [ "${CLEANUP_ENABLED:-0}" != "1" ]; then
    return 0
  fi
  [ -n "$SUP_PID" ] && terminate_pid "$SUP_PID" TERM 20
  [ -n "$YOLO_PID" ] && terminate_pid "$YOLO_PID" TERM 25
  [ -n "$ARBITER_PID" ] && terminate_pid "$ARBITER_PID" TERM 20
  [ -n "$NAV_PID" ] && terminate_pid "$NAV_PID" INT 35
  pkill -f "/root/face_tools/patrol_supervisor.py" 2>/dev/null || true
  pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
  pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
  pkill -f "/root/face_tools/drive_arbiter.py" 2>/dev/null || true
  initialize_protocol_states
  : > "$FOLLOW_CTRL_EVENT_LOG" 2>/dev/null || true
  : > "$ARBITER_EVENT_LOG" 2>/dev/null || true
  publish_stop || true
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
if [ -z "${DISPLAY:-}" ]; then
  FORCE_HEADLESS=1
fi
set -u

while [ $# -gt 0 ]; do
  case "$1" in
    --waypoints)
      WAYPOINTS_FILE="$2"
      shift 2
      ;;
    --map)
      MAP_FILE="$2"
      shift 2
      ;;
    --nav-params)
      NAV_PARAMS_FILE="$2"
      shift 2
      ;;
    --nav-stack-script)
      NAV_STACK_SCRIPT="$2"
      shift 2
      ;;
    --no-rviz)
      NAV_STACK_ARGS+=(--no-rviz)
      shift 1
      ;;
    --rviz-config)
      NAV_STACK_ARGS+=(--rviz-config "$2")
      shift 2
      ;;
    --init-x)
      INIT_X="$2"
      shift 2
      ;;
    --init-y)
      INIT_Y="$2"
      shift 2
      ;;
    --init-yaw)
      INIT_YAW="$2"
      shift 2
      ;;
    --skip-preflight)
      PRECHECK_SKIP=1
      shift 1
      ;;
    --preflight-timeout)
      PRECHECK_TIMEOUT="$2"
      shift 2
      ;;
    --preflight-retries)
      PRECHECK_RETRIES="$2"
      shift 2
      ;;
    --preflight-retry-gap)
      PRECHECK_RETRY_GAP="$2"
      shift 2
      ;;
    --init-wait-subs-timeout)
      INIT_WAIT_SUBS_TIMEOUT="$2"
      shift 2
      ;;
    --initialpose-sub-timeout)
      INITIALPOSE_SUB_TIMEOUT="$2"
      shift 2
      ;;
    --localization-timeout)
      LOCALIZATION_TIMEOUT="$2"
      shift 2
      ;;
    --localization-republish-interval)
      LOCALIZATION_REPUBLISH_INTERVAL="$2"
      shift 2
      ;;
    --nav-stack-settle-seconds)
      NAV_STACK_SETTLE_SECONDS="$2"
      shift 2
      ;;
    --post-initpose-settle-seconds)
      POST_INITPOSE_SETTLE_SECONDS="$2"
      shift 2
      ;;
    --enable-localization-wait)
      LOCALIZATION_WAIT_ENABLED=1
      shift 1
      ;;
    --max-follow-seconds)
      MAX_FOLLOW_SECONDS="$2"
      shift 2
      ;;
    --max-follow-distance)
      MAX_FOLLOW_DISTANCE="$2"
      shift 2
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --mode=*)
      MODE="${1#--mode=}"
      shift 1
      ;;
    --fast-mode)
      FAST_MODE=1
      shift 1
      ;;
    --profile-every)
      PROFILE_EVERY="$2"
      shift 2
      ;;
    --profile-every=*)
      PROFILE_EVERY="${1#--profile-every=}"
      shift 1
      ;;
    --event-log)
      EVENT_LOG="$2"
      shift 2
      ;;
    --event-log=*)
      EVENT_LOG="${1#--event-log=}"
      shift 1
      ;;
    --supervisor-event-log)
      SUPERVISOR_EVENT_LOG="$2"
      shift 2
      ;;
    --supervisor-event-log=*)
      SUPERVISOR_EVENT_LOG="${1#--supervisor-event-log=}"
      shift 1
      ;;
    --patrol-only)
      PATROL_ONLY=1
      shift 1
      ;;
    --startup-smoke-only)
      STARTUP_SMOKE_ONLY=1
      shift 1
      ;;
    *)
      if [ -z "$WAYPOINTS_FILE_SET" ] && [ -f "$1" ]; then
        WAYPOINTS_FILE="$1"
        WAYPOINTS_FILE_SET=1
        shift 1
      else
        CAMERA_ARGS+=("$1")
        shift 1
      fi
      ;;
  esac
done

case "$MODE" in
  normal)
    ;;
  fast)
    FAST_MODE=1
    ;;
  profile)
    FAST_MODE=1
    if [ -z "$PROFILE_EVERY" ]; then
      PROFILE_EVERY=30
    fi
    ;;
  *)
    echo "unknown mode: $MODE"
    echo "use --mode normal|fast|profile"
    exit 1
    ;;
esac

if [ "$FAST_MODE" -eq 1 ]; then
  CAMERA_ARGS+=(--fast-mode)
fi
if [ -n "$PROFILE_EVERY" ]; then
  CAMERA_ARGS+=(--profile-every "$PROFILE_EVERY")
fi
if [ "$MODE" = "profile" ] && [ -z "$EVENT_LOG" ]; then
  EVENT_LOG="/tmp/patrol_face_events.jsonl"
fi
if [ -n "$EVENT_LOG" ]; then
  CAMERA_ARGS+=(--event-log "$EVENT_LOG")
fi
if [ "$MODE" = "profile" ] && [ -z "$SUPERVISOR_EVENT_LOG" ]; then
  SUPERVISOR_EVENT_LOG="/tmp/patrol_supervisor_events.jsonl"
fi
if [ "$FORCE_HEADLESS" = "1" ]; then
  HAS_NO_SHOW=0
  for arg in "${CAMERA_ARGS[@]}"; do
    if [ "$arg" = "--no-show" ]; then
      HAS_NO_SHOW=1
      break
    fi
  done
  if [ "$HAS_NO_SHOW" != "1" ]; then
    CAMERA_ARGS+=(--no-show)
  fi
fi

for required in "$WAYPOINTS_FILE" "$MAP_FILE" "$NAV_PARAMS_FILE" "$NAV_STACK_SCRIPT"; do
  if [ ! -f "$required" ]; then
    echo "required file not found: $required"
    exit 1
  fi
done

derive_init_pose_from_nav_params

pkill -f "/root/face_tools/patrol_supervisor.py" 2>/dev/null || true
pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
pkill -f "/root/face_tools/drive_arbiter.py" 2>/dev/null || true
publish_stop || true
sleep 0.2

initialize_protocol_states
: > "$FOLLOW_CTRL_EVENT_LOG"
: > "$ARBITER_EVENT_LOG"

echo "[INFO] patrol mode=$MODE patrol_only=$PATROL_ONLY waypoints=$WAYPOINTS_FILE"
if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
  echo "[INFO] using initial pose x=$INIT_X y=$INIT_Y yaw=$INIT_YAW"
fi

"$ROS_PY" "$FACE_TOOLS_DIR/drive_arbiter.py" \
  --default-mode SAFE_STOP \
  --mode-topic /drive_mode \
  --mode-state-file "$SUPERVISOR_STATUS_FILE" \
  --state-topic /drive_arbiter/state \
  --status-file "$ARBITER_STATE_FILE" \
  --event-log "$ARBITER_EVENT_LOG" \
  --output-topic /car_cmd_vel \
  --nav-topic /nav_cmd_vel \
  --track-topic /track_cmd_vel \
  > /tmp/drive_arbiter.log 2>&1 &
ARBITER_PID=$!

cd /home/racecar
bash "$NAV_STACK_SCRIPT" --map "$MAP_FILE" --params "$NAV_PARAMS_FILE" "${NAV_STACK_ARGS[@]}" > /tmp/patrol_nav.log 2>&1 &
NAV_PID=$!

sleep "$NAV_STACK_SETTLE_SECONDS"

if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
  publish_initial_pose_once || true
  sleep "$POST_INITPOSE_SETTLE_SECONDS"
  if [ "$LOCALIZATION_WAIT_ENABLED" = "1" ]; then
    if ! wait_localization_ready; then
      echo "[WARN] localization readiness not yet satisfied; continuing after initial pose publish"
    fi
  fi
fi

if [ "$PRECHECK_SKIP" != "1" ]; then
  if ! run_preflight_with_retries; then
    echo "patrol preflight failed; see /tmp/patrol_preflight.json"
    cat /tmp/patrol_preflight.json 2>/dev/null || true
    exit 2
  fi
fi

if [ "$PATROL_ONLY" != "1" ]; then
  "$ACL_PY" "$FACE_TOOLS_DIR/yolo_face_track_alarm.py" \
    --state-file "$STATE_FILE" \
    --follow-policy unknown \
    --follow-unknown-min-frames 6 \
    "${CAMERA_ARGS[@]}" \
    > /tmp/patrol_yolo.log 2>&1 &
  YOLO_PID=$!
else
  echo "[INFO] patrol-only mode enabled: skip yolo/face detection stack"
fi

if [ "$STARTUP_SMOKE_ONLY" = "1" ]; then
  echo "[INFO] startup smoke passed: nav/localization/preflight are ready; exiting before supervisor movement"
  exit 0
fi

SUP_ARGS=(
  --waypoints-file "$WAYPOINTS_FILE"
  --state-file "$STATE_FILE"
  --mode-topic /drive_mode
  --alert-name unknown
  --follow-controller-state-topic /follow_controller/state
  --follow-controller-status-file "$FOLLOW_CTRL_STATE_FILE"
  --follow-controller-event-log "$FOLLOW_CTRL_EVENT_LOG"
  --follow-cmd-topic /track_cmd_vel
  --follow-target-mode unknown
  --follow-target-name ""
  --max-follow-seconds "$MAX_FOLLOW_SECONDS"
  --max-follow-distance "$MAX_FOLLOW_DISTANCE"
  --nav-server-timeout 25.0
  --status-file "$SUPERVISOR_STATUS_FILE"
)
if [ -n "$SUPERVISOR_EVENT_LOG" ]; then
  SUP_ARGS+=(--event-log "$SUPERVISOR_EVENT_LOG")
fi
if [ "$PATROL_ONLY" = "1" ]; then
  SUP_ARGS+=(--disable-alert)
fi

"$ROS_PY" "$FACE_TOOLS_DIR/patrol_supervisor.py" \
  "${SUP_ARGS[@]}" \
  > /tmp/patrol_supervisor.log 2>&1 &
SUP_PID=$!

echo "[PATROL-LOCAL] nav_pid=$NAV_PID arbiter_pid=$ARBITER_PID yolo_pid=${YOLO_PID:-} sup_pid=$SUP_PID"
wait "$SUP_PID"
