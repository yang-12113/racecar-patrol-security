#!/bin/bash
set -euo pipefail

STATE_FILE=/tmp/pc_vision_state.json
BRIDGE_STATUS_FILE=/tmp/pc_vision_bridge_status.json
ROS_PY=/usr/bin/python3
FACE_TOOLS_DIR=/root/face_tools
TARGET_MODE=any
TARGET_NAME=owner
VIDEO_URL="${VIDEO_URL:-tcp://192.168.5.15:5601}"
LISTEN_PORT="${LISTEN_PORT:-8890}"
FRESH_TIMEOUT_MS="${FRESH_TIMEOUT_MS:-220}"
BRIDGE_FRESH_TIMEOUT_MS="${BRIDGE_FRESH_TIMEOUT_MS:-150}"
BRIDGE_STALE_TIMEOUT_MS="${BRIDGE_STALE_TIMEOUT_MS:-300}"
CAMERA_ARGS=()
CTRL_PID=""
BRIDGE_PID=""
SENDER_PID=""
ARBITER_PID=""

publish_stop() {
  "$ROS_PY" - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("pc_led_stop_once")
pub = node.create_publisher(Twist, "/car_cmd_vel", 10)
msg = Twist()
msg.linear.x = 1500.0
msg.angular.z = 90.0
for _ in range(8):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

cleanup() {
  set +e
  [ -n "$SENDER_PID" ] && kill "$SENDER_PID" 2>/dev/null || true
  [ -n "$BRIDGE_PID" ] && kill "$BRIDGE_PID" 2>/dev/null || true
  [ -n "$CTRL_PID" ] && kill "$CTRL_PID" 2>/dev/null || true
  [ -n "$ARBITER_PID" ] && kill "$ARBITER_PID" 2>/dev/null || true
  pkill -f "/root/face_tools/car_sender.py" 2>/dev/null || true
  pkill -f "/root/face_tools/pc_status_bridge.py" 2>/dev/null || true
  pkill -f "/root/face_tools/face_follow_controller.py --state-file $STATE_FILE" 2>/dev/null || true
  pkill -f "/root/face_tools/motion_arbiter.py" 2>/dev/null || true
  printf '{}' > "$STATE_FILE" 2>/dev/null || true
  printf '{}' > "$BRIDGE_STATUS_FILE" 2>/dev/null || true
  publish_stop || true
}

trap cleanup EXIT INT TERM

while [ $# -gt 0 ]; do
  case "$1" in
    --target-mode)
      TARGET_MODE="$2"
      shift 2
      ;;
    --target-name)
      TARGET_NAME="$2"
      shift 2
      ;;
    --video-url)
      VIDEO_URL="$2"
      shift 2
      ;;
    --listen-port)
      LISTEN_PORT="$2"
      shift 2
      ;;
    --fresh-timeout-ms)
      FRESH_TIMEOUT_MS="$2"
      shift 2
      ;;
    --bridge-fresh-timeout-ms)
      BRIDGE_FRESH_TIMEOUT_MS="$2"
      shift 2
      ;;
    --bridge-stale-timeout-ms)
      BRIDGE_STALE_TIMEOUT_MS="$2"
      shift 2
      ;;
    *)
      CAMERA_ARGS+=("$1")
      shift 1
      ;;
  esac
done

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

pkill -f "/root/face_tools/car_sender.py" 2>/dev/null || true
pkill -f "/root/face_tools/pc_status_bridge.py" 2>/dev/null || true
pkill -f "/root/face_tools/face_follow_controller.py --state-file $STATE_FILE" 2>/dev/null || true
pkill -f "/root/face_tools/motion_arbiter.py" 2>/dev/null || true
publish_stop || true
sleep 0.2

printf '{}' > "$STATE_FILE"
printf '{}' > "$BRIDGE_STATUS_FILE"

"$ROS_PY" "$FACE_TOOLS_DIR/pc_status_bridge.py" \
  --listen-port "$LISTEN_PORT" \
  --state-file "$STATE_FILE" \
  --status-file "$BRIDGE_STATUS_FILE" \
  --fresh-timeout-ms "$BRIDGE_FRESH_TIMEOUT_MS" \
  --stale-timeout-ms "$BRIDGE_STALE_TIMEOUT_MS" &
BRIDGE_PID=$!

"$ROS_PY" "$FACE_TOOLS_DIR/motion_arbiter.py" \
  --state-file "$STATE_FILE" \
  --bridge-status-file "$BRIDGE_STATUS_FILE" &
ARBITER_PID=$!

"$ROS_PY" "$FACE_TOOLS_DIR/face_follow_controller.py" \
  --state-file "$STATE_FILE" \
  --target-mode "$TARGET_MODE" \
  --target-name "$TARGET_NAME" \
  --cmd-topic /security/follow_cmd_vel \
  --fresh-timeout-ms "$FRESH_TIMEOUT_MS" &
CTRL_PID=$!

"$ROS_PY" "$FACE_TOOLS_DIR/car_sender.py" \
  --output-url "$VIDEO_URL" \
  --sender-mode framed-mjpg-tcp \
  "${CAMERA_ARGS[@]}" &
SENDER_PID=$!

echo "[PC-LED] sender_pid=$SENDER_PID bridge_pid=$BRIDGE_PID arbiter_pid=$ARBITER_PID ctrl_pid=$CTRL_PID"
wait "$SENDER_PID"
