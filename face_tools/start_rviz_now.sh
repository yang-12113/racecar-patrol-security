#!/bin/bash
set -e

RVIZ_CONFIG="/home/racecar/install/lslidar_driver/share/lslidar_driver/rviz/nav2_default_view.rviz"
RVIZ_DISPLAY="${DISPLAY:-:1}"
RVIZ_XAUTHORITY="${XAUTHORITY:-/root/.Xauthority}"
RVIZ_XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/0}"

pick_rviz_display() {
  local xorg_auth=""
  xorg_auth=$(ps -eo args | awk '/[X]org / {for (i = 1; i <= NF; ++i) if ($i == "-auth") {print $(i + 1); exit}}')

  if [ -n "${DISPLAY:-}" ] && [ -n "${XAUTHORITY:-}" ] && env DISPLAY="${DISPLAY}" XAUTHORITY="${XAUTHORITY}" xdpyinfo 2>/dev/null | grep -q 'GLX'; then
    RVIZ_DISPLAY="${DISPLAY}"
    RVIZ_XAUTHORITY="${XAUTHORITY}"
    return 0
  fi

  if [ -n "$xorg_auth" ] && env DISPLAY=":0" XAUTHORITY="$xorg_auth" xdpyinfo 2>/dev/null | grep -q 'GLX'; then
    RVIZ_DISPLAY=":0"
    RVIZ_XAUTHORITY="$xorg_auth"
    return 0
  fi

  if env DISPLAY="$RVIZ_DISPLAY" XAUTHORITY="$RVIZ_XAUTHORITY" xdpyinfo >/dev/null 2>&1; then
    return 0
  fi

  if env DISPLAY=":1" XAUTHORITY="/root/.Xauthority" xdpyinfo >/dev/null 2>&1; then
    RVIZ_DISPLAY=":1"
    RVIZ_XAUTHORITY="/root/.Xauthority"
    return 0
  fi

  return 0
}

pick_rviz_display || true

source /opt/ros/humble/setup.bash

pkill -x rviz2 2>/dev/null || true
rviz_env=(env DISPLAY="$RVIZ_DISPLAY" XAUTHORITY="$RVIZ_XAUTHORITY" XDG_RUNTIME_DIR="$RVIZ_XDG_RUNTIME_DIR")

"${rviz_env[@]}" rviz2 -d "$RVIZ_CONFIG" >/tmp/mapping_rviz.log 2>&1 &

sleep 2
if pgrep -x rviz2 >/dev/null 2>&1; then
  echo "rviz_started display=$RVIZ_DISPLAY auth=$RVIZ_XAUTHORITY"
else
  echo "rviz_failed"
  tail -n 40 /tmp/mapping_rviz.log 2>/dev/null || true
  exit 1
fi
