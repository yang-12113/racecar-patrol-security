#!/bin/bash
set -e

START_RVIZ=1
RVIZ_CONFIG="/home/racecar/src/racecar/rviz/navigation.rviz"
RVIZ_DISPLAY="${DISPLAY:-}"
RVIZ_XAUTHORITY="${XAUTHORITY:-$HOME/.Xauthority}"
RVIZ_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/0}"
RUN_CAR_PID=""
RVIZ_PID=""

usage() {
  cat <<'EOF'
usage: bash Run_car.sh [--no-rviz] [--rviz-config FILE]
EOF
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
  "${rviz_env[@]}" rviz2 -d "$RVIZ_CONFIG" > /tmp/run_car_rviz.log 2>&1 &
  RVIZ_PID=$!
  sleep 2
  if ! kill -0 "$RVIZ_PID" 2>/dev/null; then
    echo "[ERROR] rviz2 exited early; see /tmp/run_car_rviz.log"
    tail -n 40 /tmp/run_car_rviz.log 2>/dev/null || true
    return 1
  fi
  echo "[INFO] launched rviz2 pid=$RVIZ_PID config=$RVIZ_CONFIG display=$RVIZ_DISPLAY"
}

terminate() {
  echo "????????..."
  if [ -n "$RVIZ_PID" ]; then
    kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ -n "$RUN_CAR_PID" ]; then
    kill -INT "$RUN_CAR_PID" 2>/dev/null || true
    wait "$RUN_CAR_PID" 2>/dev/null || true
  fi
  echo "????????"
}

trap terminate SIGINT SIGTERM EXIT

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

while [ $# -gt 0 ]; do
  case "$1" in
    --no-rviz)
      START_RVIZ=0
      shift 1
      ;;
    --rviz-config)
      RVIZ_CONFIG="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      echo "unknown argument: $1"
      usage
      exit 1
      ;;
  esac
done

launch_rviz
ros2 launch racecar Run_car.launch.py &
RUN_CAR_PID=$!
wait "$RUN_CAR_PID"
