#!/bin/bash
set -e

STATE_FILE=/tmp/face_follow_state.json
ROS_PY=/usr/bin/python3
ACL_PY=/usr/local/miniconda3/bin/python
CTRL_TARGET_MODE=any
CTRL_TARGET_NAME=""

publish_stop() {
  "$ROS_PY" - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("face_follow_stop_once")
pub = node.create_publisher(Twist, "/car_cmd_vel", 10)
msg = Twist()
msg.linear.x = 1500.0
msg.linear.y = 0.0
msg.linear.z = 0.0
msg.angular.x = 0.0
msg.angular.y = 0.0
msg.angular.z = 90.0
for _ in range(5):
    pub.publish(msg)
    rclpy.spin_once(node, timeout_sec=0.05)
    time.sleep(0.05)
node.destroy_node()
rclpy.shutdown()
PY
}

cleanup() {
  if [ -n "$YOLO_PID" ]; then
    kill "$YOLO_PID" 2>/dev/null || true
    wait "$YOLO_PID" 2>/dev/null || true
  fi
  if [ -n "$CTRL_PID" ]; then
    kill "$CTRL_PID" 2>/dev/null || true
    wait "$CTRL_PID" 2>/dev/null || true
  fi
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

for arg in "$@"; do
  if [ "$arg" = "--help" ] || [ "$arg" = "-h" ]; then
    exec "$ACL_PY" /root/face_tools/yolo_face_track_alarm.py --help
  fi
done

pkill -f "/root/face_tools/yolo_face_track_alarm.py" 2>/dev/null || true
pkill -f "/root/face_tools/face_follow_controller.py" 2>/dev/null || true
publish_stop || true
sleep 0.2

PREV_ARG=""
for arg in "$@"; do
  if [ "$PREV_ARG" = "--follow-policy" ]; then
    if [ "$arg" = "unknown" ]; then
      CTRL_TARGET_MODE=unknown
      CTRL_TARGET_NAME=""
    elif [ "$arg" = "owner" ] || [ "$arg" = "named" ] || [ "$arg" = "none" ]; then
      CTRL_TARGET_MODE=any
      CTRL_TARGET_NAME=""
    fi
  fi

  case "$arg" in
    --follow-policy=unknown)
      CTRL_TARGET_MODE=unknown
      CTRL_TARGET_NAME=""
      ;;
    --follow-policy=owner|--follow-policy=named|--follow-policy=none)
      CTRL_TARGET_MODE=any
      CTRL_TARGET_NAME=""
      ;;
    --follow-name=*)
      CTRL_TARGET_NAME="${arg#--follow-name=}"
      ;;
  esac
  PREV_ARG="$arg"
done

printf '{}' > "$STATE_FILE"

"$ROS_PY" /root/face_tools/face_follow_controller.py --state-file "$STATE_FILE" --target-mode "$CTRL_TARGET_MODE" --target-name "$CTRL_TARGET_NAME" &
CTRL_PID=$!

"$ACL_PY" /root/face_tools/yolo_face_track_alarm.py --state-file "$STATE_FILE" --follow-name owner "$@" &
YOLO_PID=$!
wait "$YOLO_PID"
