#!/bin/bash
set -e

cleanup() {
  pkill -f rviz2 2>/dev/null || true
  if [ -n "$STACK_PID" ]; then
    kill "$STACK_PID" 2>/dev/null || true
    wait "$STACK_PID" 2>/dev/null || true
  fi
  if [ -n "$BASE_PID" ]; then
    kill "$BASE_PID" 2>/dev/null || true
    wait "$BASE_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
cd /home/racecar
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

for arg in "$@"; do
  if [ "$arg" = "--help" ] || [ "$arg" = "-h" ]; then
    exec /root/face_tools/run_face_follow_stack.sh --help
  fi
done

ros2 launch racecar Run_car.launch.py > /tmp/run_car_face_follow.log 2>&1 &
BASE_PID=$!

sleep 4
pkill -f rviz2 2>/dev/null || true

/root/face_tools/run_face_follow_stack.sh --follow-policy owner "$@" &
STACK_PID=$!
wait "$STACK_PID"
