#!/bin/bash
set -e

MAP_FILE="/home/racecar/src/racecar/map/test_map.yaml"
NAV_PARAMS_FILE="/home/racecar/src/racecar/config/nav/navigation.yaml"
# Use the full hardware launch by default so /scan, static TF, encoder and base driver come up together.
SENSOR_LAUNCH="Run_car.launch.py"
CONTROLLER_LAUNCH="test.launch.py"

cleanup() {
  for pid in "$CTRL_PID" "$NAV2_PID" "$ODOM_PID" "$IMU_PID" "$SENSOR_PID"; do
    if [ -n "$pid" ]; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

while [ $# -gt 0 ]; do
  case "$1" in
    --map)
      MAP_FILE="$2"
      shift 2
      ;;
    --params)
      NAV_PARAMS_FILE="$2"
      shift 2
      ;;
    --sensor-launch)
      SENSOR_LAUNCH="$2"
      shift 2
      ;;
    --controller-launch)
      CONTROLLER_LAUNCH="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1"
      echo "usage: bash $0 [--map file] [--params file] [--sensor-launch name] [--controller-launch name]"
      exit 1
      ;;
  esac
done

if [ ! -f "$MAP_FILE" ]; then
  echo "map file not found: $MAP_FILE"
  exit 1
fi

if [ ! -f "$NAV_PARAMS_FILE" ]; then
  echo "nav params file not found: $NAV_PARAMS_FILE"
  exit 1
fi

cd /home/racecar

ros2 launch racecar "$SENSOR_LAUNCH" > /tmp/nav_sensor_launch.log 2>&1 &
SENSOR_PID=$!
sleep 2

ros2 run encoder_imu imu_encoder_mix > /tmp/nav_imu_encoder_mix.log 2>&1 &
IMU_PID=$!
sleep 2

ros2 run odom_tf odom_tf > /tmp/nav_odom_tf.log 2>&1 &
ODOM_PID=$!
sleep 4

ros2 launch nav2_bringup bringup_launch.py \
  use_sim_time:=False \
  autostart:=True \
  map:="$MAP_FILE" \
  params_file:="$NAV_PARAMS_FILE" \
  > /tmp/nav2_bringup_managed.log 2>&1 &
NAV2_PID=$!
sleep 4

ros2 launch racecar "$CONTROLLER_LAUNCH" > /tmp/nav_controller_launch.log 2>&1 &
CTRL_PID=$!

wait "$SENSOR_PID" "$IMU_PID" "$ODOM_PID" "$NAV2_PID" "$CTRL_PID"
