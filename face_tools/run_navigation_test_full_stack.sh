#!/bin/bash
set -e

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
INIT_WAIT_SUBS_TIMEOUT="15"
LOCALIZATION_TIMEOUT="35"
LOCALIZATION_REPUBLISH_INTERVAL="6"
ROS_PY=/usr/bin/python3

cleanup() {
  if [ -n "$NAVTEST_PID" ]; then
    kill -INT "$NAVTEST_PID" 2>/dev/null || true
    wait "$NAVTEST_PID" 2>/dev/null || true
  fi
  if [ -n "$NAV_PID" ]; then
    kill -INT "$NAV_PID" 2>/dev/null || true
    wait "$NAV_PID" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

source /opt/ros/humble/setup.bash
if [ -f /home/racecar/install/setup.bash ]; then
  source /home/racecar/install/setup.bash
fi

publish_initial_pose_once() {
  if [ -z "$INIT_X" ] || [ -z "$INIT_Y" ] || [ -z "$INIT_YAW" ]; then
    return 0
  fi
  "$ROS_PY" /root/face_tools/publish_initial_pose.py \
    --x "$INIT_X" \
    --y "$INIT_Y" \
    --yaw "$INIT_YAW" \
    --wait-subs-timeout "$INIT_WAIT_SUBS_TIMEOUT" \
    > /tmp/navigation_only_init_pose.log 2>&1
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
      > /tmp/navigation_only_preflight.json 2>&1; then
      return 0
    fi

    if [ "$attempt" -lt "$max_attempts" ] && [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
      echo "navigation preflight attempt $attempt/$max_attempts failed; republishing initial pose and retrying..."
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
    > /tmp/navigation_only_localization_ready.json 2>&1
}

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
        echo "usage: bash $0 [--waypoints file] [--map file] [--nav-params file] [--init-x x --init-y y --init-yaw yaw] [--skip-preflight] [--preflight-timeout sec] [--preflight-retries n] [--preflight-retry-gap sec] [--init-wait-subs-timeout sec] [--localization-timeout sec] [--localization-republish-interval sec]"
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

cd /home/racecar
bash "$NAV_STACK_SCRIPT" --map "$MAP_FILE" --params "$NAV_PARAMS_FILE" > /tmp/navigation_only_nav.log 2>&1 &
NAV_PID=$!

sleep 10

if [ -n "$INIT_X" ] && [ -n "$INIT_Y" ] && [ -n "$INIT_YAW" ]; then
  if ! wait_localization_ready; then
    echo "localization readiness failed; see /tmp/navigation_only_localization_ready.json"
    cat /tmp/navigation_only_localization_ready.json 2>/dev/null || true
    exit 2
  fi
fi

if [ "$PRECHECK_SKIP" != "1" ]; then
  if ! run_preflight_with_retries; then
    echo "navigation preflight failed; see /tmp/navigation_only_preflight.json"
    cat /tmp/navigation_only_preflight.json 2>/dev/null || true
    exit 2
  fi
fi

if ros2 pkg executables racecar 2>/dev/null | grep -q 'navigation_test'; then
  ros2 run racecar navigation_test \
    --ros-args \
    -p waypoints_file:="$WAYPOINTS_FILE" \
    -p publish_initial_pose:=false \
    > /tmp/navigation_only_test.log 2>&1 &
else
  "$ROS_PY" /home/racecar/src/racecar/scripts/navigation_test.py \
    --ros-args \
    -p waypoints_file:="$WAYPOINTS_FILE" \
    -p publish_initial_pose:=false \
    > /tmp/navigation_only_test.log 2>&1 &
fi
NAVTEST_PID=$!

wait "$NAVTEST_PID"
