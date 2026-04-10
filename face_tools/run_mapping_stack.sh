#!/bin/bash
set -e

SENSOR_LAUNCH="Run_car.launch.py"
MAPPING_PACKAGE="slam_gmapping"
MAPPING_LAUNCH="slam_gmapping.launch.py"
RVIZ_ENABLED=1
RVIZ_CONFIG="/home/racecar/install/lslidar_driver/share/lslidar_driver/rviz/nav2_default_view.rviz"
ROS_PY=/usr/bin/python3
CLEANUP_ENABLED=0

usage() {
  cat <<'EOF'
usage: bash run_mapping_stack.sh [options]

options:
  --sensor-launch NAME      default: Run_car.launch.py
  --mapping-package NAME    default: slam_gmapping
  --mapping-launch NAME     default: slam_gmapping.launch.py
  --no-rviz                 do not start rviz2
  --rviz-config PATH        default: /home/racecar/install/lslidar_driver/share/lslidar_driver/rviz/nav2_default_view.rviz

Starts the mapping stack:
  1. sensor/base driver launch
  2. encoder_imu odom fusion
  3. odom_tf broadcaster
  4. slam_gmapping
  5. rviz2 with the mapping view

Use Ctrl+C to stop the stack safely.
EOF
}

publish_stop() {
  "$ROS_PY" - <<'PY'
import time
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node("mapping_stop_once")
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

start_rviz() {
  if [ "${RVIZ_ENABLED:-1}" != "1" ]; then
    return 0
  fi
  if ! command -v rviz2 >/dev/null 2>&1; then
    echo "[WARN] rviz2 is not available, skip RViz startup"
    return 0
  fi
  if [ -z "${DISPLAY:-}" ] && [ -z "${WAYLAND_DISPLAY:-}" ]; then
    echo "[WARN] DISPLAY/WAYLAND_DISPLAY is not set, skip RViz startup"
    return 0
  fi

  local rviz_args=()
  if [ -n "${RVIZ_CONFIG:-}" ]; then
    if [ -f "$RVIZ_CONFIG" ]; then
      rviz_args=(-d "$RVIZ_CONFIG")
    else
      echo "[WARN] RViz config not found: $RVIZ_CONFIG, starting rviz2 with the default view"
    fi
  fi

  echo "[INFO] start rviz2 ${RVIZ_CONFIG:+config=$RVIZ_CONFIG}"
  rviz2 "${rviz_args[@]}" > /tmp/mapping_rviz.log 2>&1 &
  RVIZ_PID=$!
}

cleanup() {
  if [ "${CLEANUP_ENABLED:-0}" != "1" ]; then
    return 0
  fi
  for pid in "$RVIZ_PID" "$SLAM_PID" "$ODOM_PID" "$IMU_PID" "$SENSOR_PID"; do
    if [ -n "$pid" ]; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  pkill -f "slam_gmapping" 2>/dev/null || true
  pkill -f "odom_tf" 2>/dev/null || true
  pkill -f "imu_encoder_mix" 2>/dev/null || true
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
CLEANUP_ENABLED=1

while [ $# -gt 0 ]; do
  case "$1" in
    --sensor-launch)
      SENSOR_LAUNCH="$2"
      shift 2
      ;;
    --mapping-package)
      MAPPING_PACKAGE="$2"
      shift 2
      ;;
    --mapping-launch)
      MAPPING_LAUNCH="$2"
      shift 2
      ;;
    --no-rviz)
      RVIZ_ENABLED=0
      shift
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1"
      echo "usage: bash $0 [--sensor-launch name] [--mapping-package name] [--mapping-launch name] [--no-rviz] [--rviz-config path]"
      exit 1
      ;;
  esac
done

pkill -f "slam_gmapping" 2>/dev/null || true
pkill -f "odom_tf" 2>/dev/null || true
pkill -f "imu_encoder_mix" 2>/dev/null || true
publish_stop || true
sleep 0.2

cd /home/racecar

echo "[INFO] start mapping stack sensor_launch=$SENSOR_LAUNCH mapping=$MAPPING_PACKAGE/$MAPPING_LAUNCH"

ros2 launch racecar "$SENSOR_LAUNCH" > /tmp/mapping_sensor_launch.log 2>&1 &
SENSOR_PID=$!
sleep 2

ros2 run encoder_imu imu_encoder_mix > /tmp/mapping_imu_encoder_mix.log 2>&1 &
IMU_PID=$!
sleep 2

ros2 run odom_tf odom_tf > /tmp/mapping_odom_tf.log 2>&1 &
ODOM_PID=$!
sleep 3

ros2 launch "$MAPPING_PACKAGE" "$MAPPING_LAUNCH" > /tmp/mapping_slam.log 2>&1 &
SLAM_PID=$!

sleep 1
start_rviz

WAIT_PIDS=("$SENSOR_PID" "$IMU_PID" "$ODOM_PID" "$SLAM_PID")
if [ -n "${RVIZ_PID:-}" ]; then
  WAIT_PIDS+=("$RVIZ_PID")
fi

wait "${WAIT_PIDS[@]}"
