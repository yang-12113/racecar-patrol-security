#!/bin/bash
set -e

MAP_FILE="/home/racecar/src/racecar/map/test_map.yaml"
NAV_PARAMS_FILE="/home/racecar/src/racecar/config/nav/navigation.yaml"
# Use the full hardware launch by default so /scan, static TF, encoder and base driver come up together.
SENSOR_LAUNCH="Run_car.launch.py"
CONTROLLER_LAUNCH="test.launch.py"
START_RVIZ=1
RVIZ_CONFIG="/home/racecar/src/racecar/rviz/navigation.rviz"
RVIZ_DISPLAY="${DISPLAY:-}"
RVIZ_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RVIZ_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/0}"
RVIZ_PID=""

cleanup_existing_stack() {
  pkill -f "ros2 launch racecar $SENSOR_LAUNCH" 2>/dev/null || true
  pkill -f "ros2 launch racecar $CONTROLLER_LAUNCH" 2>/dev/null || true
  pkill -f 'ros2 launch nav2_bringup bringup_launch.py' 2>/dev/null || true
  pkill -f 'ros2 run odom_tf odom_tf' 2>/dev/null || true
  pkill -f 'ros2 run encoder_imu imu_encoder_mix' 2>/dev/null || true
  pkill -f '/home/racecar/install/odom_tf/lib/odom_tf/odom_tf' 2>/dev/null || true
  pkill -f '/home/racecar/install/encoder_imu/lib/encoder_imu/imu_encoder_mix' 2>/dev/null || true
  pkill -f '/opt/ros/humble/lib/lslidar_driver/lslidar_driver_node' 2>/dev/null || true
  pkill -f '/home/racecar/install/lslidar_driver/lib/lslidar_driver/lslidar_driver_node' 2>/dev/null || true
  pkill -f '/home/racecar/install/hipnuc_imu/lib/hipnuc_imu/talker' 2>/dev/null || true
  pkill -f '/home/racecar/install/racecar/lib/racecar/racecar_driver_node' 2>/dev/null || true
  pkill -f '/home/racecar/install/encoder_imu/lib/encoder_imu/encoder_node' 2>/dev/null || true
  pkill -f '/home/racecar/src/encoder/encoder/encoder_node.py' 2>/dev/null || true
  pkill -f 'encoder_vel' 2>/dev/null || true
  pkill -f 'static_transform_publisher' 2>/dev/null || true
  pkill -f 'component_container_isolated' 2>/dev/null || true
  pkill -f 'car_controller_new' 2>/dev/null || true
  sleep 1
}

resolve_rviz_display() {
  local xorg_auth=""
  xorg_auth=$(ps -eo args | awk '/[X]org / {for (i = 1; i <= NF; ++i) if ($i == "-auth") {print $(i + 1); exit}}')

  # First priority: current shell display (SSH X11 forwarding like localhost:10.0)
  if [ -n "${DISPLAY:-}" ]; then
    if [ -n "${XAUTHORITY:-}" ] && env DISPLAY="${DISPLAY}" XAUTHORITY="${XAUTHORITY}" xdpyinfo >/dev/null 2>&1; then
      RVIZ_DISPLAY="${DISPLAY}"
      RVIZ_XAUTHORITY="${XAUTHORITY}"
      return 0
    fi
    if env DISPLAY="${DISPLAY}" xdpyinfo >/dev/null 2>&1; then
      RVIZ_DISPLAY="${DISPLAY}"
      RVIZ_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
      return 0
    fi
  fi

  # Fallback: local physical displays
  if [ -n "$xorg_auth" ]; then
    for d in :0 :1 :1.0; do
      if env DISPLAY="$d" XAUTHORITY="$xorg_auth" xdpyinfo >/dev/null 2>&1; then
        RVIZ_DISPLAY="$d"
        RVIZ_XAUTHORITY="$xorg_auth"
        return 0
      fi
    done
  fi

  return 1
}


launch_rviz() {
  if [ "${START_RVIZ:-1}" != "1" ]; then
    return 0
  fi
  if [ ! -f "$RVIZ_CONFIG" ]; then
    echo "[WARN] rviz config not found: $RVIZ_CONFIG"
    return 0
  fi
  if ! resolve_rviz_display; then
    echo "[WARN] unable to find a usable X11 display for rviz2"
    return 0
  fi
  if pgrep -x rviz2 >/dev/null 2>&1; then
    echo "[WARN] existing rviz2 detected; relaunching on $RVIZ_DISPLAY"
    pkill -x rviz2 2>/dev/null || true
    for _ in $(seq 1 50); do
      if ! pgrep -x rviz2 >/dev/null 2>&1; then
        break
      fi
      sleep 0.1
    done
  fi
  local rviz_env=(env DISPLAY="$RVIZ_DISPLAY" XAUTHORITY="$RVIZ_XAUTHORITY" XDG_RUNTIME_DIR="$RVIZ_XDG_RUNTIME_DIR" QT_X11_NO_MITSHM=1)
  if [[ "$RVIZ_DISPLAY" == localhost:* ]] || [[ "$RVIZ_DISPLAY" == 127.0.0.1:* ]]; then
    rviz_env+=(LIBGL_ALWAYS_SOFTWARE=1 MESA_GL_VERSION_OVERRIDE=3.3 QT_OPENGL=software)
  fi
  "${rviz_env[@]}" \
    rviz2 -d "$RVIZ_CONFIG" \
    > /tmp/nav_rviz.log 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[ERROR] rviz2 exited early; see /tmp/nav_rviz.log"
    tail -n 40 /tmp/nav_rviz.log 2>/dev/null || true
    return 1
  fi
  echo "[INFO] launched rviz2 pid=$RVIZ_PID config=$RVIZ_CONFIG display=$RVIZ_DISPLAY"
}

cleanup() {
  if [ -n "${RVIZ_PID:-}" ]; then
    kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  for pid in "$CTRL_PID" "$NAV2_PID" "$ODOM_PID" "$IMU_PID" "$SENSOR_PID"; do
    if [ -n "$pid" ]; then
      kill -INT "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  pkill -f '/home/racecar/install/odom_tf/lib/odom_tf/odom_tf' 2>/dev/null || true
  pkill -f '/home/racecar/install/encoder_imu/lib/encoder_imu/imu_encoder_mix' 2>/dev/null || true
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

ROS_PY=/usr/bin/python3

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
    --no-rviz)
      START_RVIZ=0
      shift 1
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1"
      echo "usage: bash $0 [--map file] [--params file] [--sensor-launch name] [--controller-launch name] [--no-rviz] [--rviz-config file]"
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

cleanup_existing_stack

cd /home/racecar

ros2 launch racecar "$SENSOR_LAUNCH" > /tmp/nav_sensor_launch.log 2>&1 &
SENSOR_PID=$!
sleep 2

"$ROS_PY" /root/face_tools/kick_lslidar_order.py > /tmp/nav_lslidar_kick.log 2>&1 || true
sleep 1

ros2 run encoder_imu imu_encoder_mix --ros-args \
  -p imu_timeout_sec:=0.6 \
  -p imu_hard_timeout_sec:=2.0 \
  -p max_integration_dt_sec:=0.2 \
  > /tmp/nav_imu_encoder_mix.log 2>&1 &
IMU_PID=$!
sleep 2

ros2 run odom_tf odom_tf --ros-args \
  -p odom_timeout_sec:=0.3 \
  -p max_hold_timeout_sec:=5.0 \
  > /tmp/nav_odom_tf.log 2>&1 &
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

launch_rviz

ros2 launch racecar "$CONTROLLER_LAUNCH" > /tmp/nav_controller_launch.log 2>&1 &
CTRL_PID=$!

wait "$SENSOR_PID" "$IMU_PID" "$ODOM_PID" "$NAV2_PID" "$CTRL_PID"
