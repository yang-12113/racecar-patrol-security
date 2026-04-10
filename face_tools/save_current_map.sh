#!/bin/bash
set -e

MAP_DIR="/home/racecar/src/racecar/map"
MAP_NAME=""
FREE_THRESH=""
OCC_THRESH=""

usage() {
  cat <<'EOF'
usage: bash save_current_map.sh [options]

options:
  --map-dir DIR         default: /home/racecar/src/racecar/map
  --name NAME           map basename, default: map_YYYYMMDD_HHMMSS
  --free-thresh VALUE   optional free threshold for map_saver_cli
  --occ-thresh VALUE    optional occupied threshold for map_saver_cli

Example:
  bash save_current_map.sh --name classroom_20260403
EOF
}

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

while [ $# -gt 0 ]; do
  case "$1" in
    --map-dir)
      MAP_DIR="$2"
      shift 2
      ;;
    --name)
      MAP_NAME="$2"
      shift 2
      ;;
    --free-thresh)
      FREE_THRESH="$2"
      shift 2
      ;;
    --occ-thresh)
      OCC_THRESH="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1"
      echo "usage: bash $0 [--map-dir dir] [--name name] [--free-thresh value] [--occ-thresh value]"
      exit 1
      ;;
  esac
done

if [ -z "$MAP_NAME" ]; then
  MAP_NAME="map_$(date +%Y%m%d_%H%M%S)"
fi

mkdir -p "$MAP_DIR"
OUT_PREFIX="$MAP_DIR/$MAP_NAME"

ARGS=(-f "$OUT_PREFIX")
if [ -n "$FREE_THRESH" ]; then
  ARGS+=(--free "$FREE_THRESH")
fi
if [ -n "$OCC_THRESH" ]; then
  ARGS+=(--occ "$OCC_THRESH")
fi

echo "[INFO] saving map to ${OUT_PREFIX}.yaml/.pgm"
ros2 run nav2_map_server map_saver_cli "${ARGS[@]}"

if [ ! -f "${OUT_PREFIX}.yaml" ] || [ ! -f "${OUT_PREFIX}.pgm" ]; then
  echo "[ERROR] map save command finished but expected files are missing"
  exit 2
fi

echo "[DONE] saved:"
echo "  ${OUT_PREFIX}.yaml"
echo "  ${OUT_PREFIX}.pgm"
