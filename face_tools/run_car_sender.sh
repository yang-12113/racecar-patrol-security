#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"
OUTPUT_URL="${OUTPUT_URL:-udp://192.168.5.15:5600?pkt_size=1316}"

exec "$PYTHON_BIN" "$SCRIPT_DIR/car_sender.py" \
  --output-url "$OUTPUT_URL" \
  --camera 0 \
  --camera-scan "0,1,2,3" \
  --width 1280 \
  --height 720 \
  --camera-fps 30 \
  --camera-fourcc MJPG \
  --camera-power-line 1 \
  --camera-exposure-auto-priority 0 \
  --crf 20 \
  --preset ultrafast \
  "$@"
