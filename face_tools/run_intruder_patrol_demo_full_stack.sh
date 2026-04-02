#!/bin/bash
set -e

STATE_FILE=/tmp/face_follow_state.json
WAYPOINTS_FILE="/root/face_tools/patrol_waypoints_demo.json"
WAYPOINTS_FILE_SET=""
MAP_FILE="/home/racecar/src/racecar/map/test_map.yaml"
NAV_PARAMS_FILE="/home/racecar/src/racecar/config/nav/navigation.yaml"
NAV_STACK_SCRIPT="/root/face_tools/run_nav_stack_with_params.sh"
INIT_X=""
INIT_Y=""
INIT_YAW=""
PRECHECK_SKIP=0
PRECHECK_TIMEOUT="12"
PRECHECK_RETRIES="3"
PRECHECK_RETRY_GAP="3"
MAX_FOLLOW_SECONDS="25"
MAX_FOLLOW_DISTANCE="3.0"
INIT_WAIT_SUBS_TIMEOUT="15"
LOCALIZATION_TIMEOUT="35"
LOCALIZATION_REPUBLISH_INTERVAL="6"
ROS_PY=/usr/bin/python3
ACL_PY=/usr/local/miniconda3/bin/python

publish_stop() {
  "$ROS_PY" - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("patrol_stop_once")
pub = node.create_publisher(Twist, "/car_cmd_vel", 10)
msg = Twist()
msg.linear.x = 1500.0
msg.angular.z = 90.0
for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

publish_initial_pose_once() {
  if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_YAW" ]; then
    return 0
  fi
  "$ROS_PY" /root/face_tools/publish_initial_pose.py \
    --x "$INIT_X" \
    --y "$INIT_Y" \
    --yaw "$INIT_YAW" \
    --wait-subs-timeout "$INIT_WAIT_SUBS_TIMEOUT" \
    > /tmp/patrol_init_pose.log 2>&1
}

run_preflight_with_retries() {
  local attempt=1
  local max_attempts
  max_attempts=$(printf "%d" "$PRECHECK_RETRIES" 2>/dev/null || echo 1)
  if [ "$max_attempts" -lt 1 ]; then
    max_attempts=1
  fi

  while [ "$attempt" -le "$max_attempts" ]; do
    if "$ROS_PY" /root/face_tools/patrol_preflight_check.py \
      --waypoints-file "$WAYPOINTS_FILE" \
      --map-yaml "$MAP_FILE" \
      --timeout "$PRECHECK_TIMEOUT" \
      > /tmp/patrol_preflight.json 2>&1; then
      return 0
    fi

    if [ "$attempt" -lt "$max_attempts" ] && [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
      echo "patrol preflight attempt $attempt/$max_attempts failed; republishing initial pose and retrying..."
      publish_initial_pose_once || true
      sleep "$PRECHECK_RETRY_GAP"
    fi
    attempt=$((attempt + 1))
  done
  return 1
}

wait_localization_ready() {
  if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_YAW" ]; then
    return 0
  fi

  "$ROS_PY" /root/face_tools/wait_localization_ready.py \
    --x "$INIT_X" \
    --y "$INIT_Y" \
    --yaw "$INIT_YAW" \
    --timeout "$LOCALIZATION_TIMEOUT" \
    --republish-interval "$LOCALIZATION_REPUBLISH_INTERVAL" \
    --publish-wait-subs-timeout "$INIT_WAIT_SUBS_TIMEOUT" \
    > /tmp/patrol_localization_ready.json 2>&1
}

cleanup() {
  if [ -n "$SUP_PID" ]; then
    kill "$SUP_PID" 2>/dev/null || true
    wait "$SUP_PID" 2>/dev/null || true
  fi
  if [ -n "$YOLO_PID" ]; then
    kill "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi
  if [ -n "$NAV_PID" ]; then
    kill -INT "$NAV_PID" 2>/dev/null || true
    wait "$NAV_PID" 2>/dev/null || true
  fi
  pkill -f "/root/face_tools/patrol_supervisor.py" 2>/dev/null || true
  pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
  pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
  printf '{}' > "$STATE_FILE" 2>/dev/null || true
  publish_stop || true
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi
source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /usr/local/Ascend/nnae/set_env.sh

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
    --max-follow-seconds)
      MAX_FOLLOW_SECONDS="$2"
      shift 2
      ;;
    --max-follow-distance)
      MAX_FOLLOW_DISTANCE="$2"
      shift 2
      ;;
    --init-wait-subs-timeout)
      INIT_WAIT_SUBS_TIMEOUT="$2"
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
    *)
      if [ -z "$WAYPOINTS_FILE_SET" ] && [ -f "$1" ]; then
        WAYPOINTS_FILE="$1"
        WAYPOINTS_FILE_SET=1
        shift 1
      else
        echo "unknown argument: $1"
        echo "usage: bash $0 [--waypoints file] [--map file] [--nav-params file] [--init-x x --init-y y --init-yaw yaw] [--skip-preflight] [--preflight-timeout sec] [--preflight-retries n] [--preflight-retry-gap sec] [--init-wait-subs-timeout sec] [--localization-timeout sec] [--localization-republish-interval sec] [--max-follow-seconds sec] [--max-follow-distance meters]"
        exit 1
      fi
      ;;
  esac
done

if [ ! -f "$WAYPOINTS_FILE" ]; then
  echo "waypoints file not found: $WAYPOINTS_FILE"
  exit 1
fi

if [ ! -f "$MAP_FILE" ]; then
  echo "map file not found: $MAP_FILE"
  exit 1
fi

if [ ! -f "$NAV_PARAMS_FILE" ]; then
  echo "nav params file not found: $NAV_PARAMS_FILE"
  exit 1
fi

if [ ! -f "$NAV_STACK_SCRIPT" ]; then
  echo "nav stack script not found: $NAV_STACK_SCRIPT"
  exit 1
fi

pkill -f "/root/face_tools/patrol_supervisor.py" 2>/dev/null || true
pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
publish_stop || true
sleep 0.2

cd /home/racecar
bash "$NAV_STACK_SCRIPT" --map "$MAP_FILE" --params "$NAV_PARAMS_FILE" > /tmp/patrol_nav.log 2>&1 &
NAV_PID=$!

sleep 10

if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
  if ! wait_localization_ready; then
    echo "localization readiness failed; see /tmp/patrol_localization_ready.json"
    cat /tmp/patrol_localization_ready.json 2>/dev/null || true
    exit 2
  fi
fi

if [ "$PRECHECK_SKIP" != "1" ]; then
  if ! run_preflight_with_retries; then
    echo "patrol preflight failed; see /tmp/patrol_preflight.json"
    cat /tmp/patrol_preflight.json 2>/dev/null || true
    exit 2
  fi
fi

"$ACL_PY" /root/face_tools/yolo_face_track_alarm.py \
  --state-file "$STATE_FILE" \
  --follow-policy unknown \
  --follow-unknown-min-frames 6 \
  --follow-name owner \
  --no-show \
  --no-record \
  > /tmp/patrol_yolo.log 2>&1 &
YOLO_PID=$!

sleep 2

"$ROS_PY" /root/face_tools/patrol_supervisor.py \
  --waypoints-file "$WAYPOINTS_FILE" \
  --state-file "$STATE_FILE" \
  --alert-name unknown \
  --follow-target-mode unknown \
  --follow-target-name "" \
  --max-follow-seconds "$MAX_FOLLOW_SECONDS" \
  --max-follow-distance "$MAX_FOLLOW_DISTANCE" \
  --nav-server-timeout 25.0 \
  > /tmp/patrol_supervisor.log 2>&1 &
SUP_PID=$!

wait "$SUP_PID"
