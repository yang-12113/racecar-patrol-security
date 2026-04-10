#!/bin/bash
set -e

MODE="normal"
REMOTE_URL="http://192.168.5.15:8765/detect"
REMOTE_TIMEOUT="15"
REMOTE_JPEG_QUALITY="90"
REMOTE_SCALE="1.0"
REMOTE_DISCOVERY_INTERVAL="3"
REMOTE_DISCOVERY_JPEG_QUALITY="75"
REMOTE_DISCOVERY_SCALE="0.60"
REMOTE_ROI_MAX_TRACKS="2"
REMOTE_ROI_EXPAND="0.20"
REMOTE_ROI_JPEG_QUALITY="92"
REMOTE_ROI_SCALE="1.0"
MATCH_THRES="0.42"
FACE_KEEP_SCORE="0.58"
KNOWN_HOLD_SCORE="0.56"
UNKNOWN_MIN_CONF="0.55"
PERSON_EDGE_MIN_CONF="0.60"
CONF_THRES="0.28"
TRACK_LOW_CONF="0.12"
PASSTHROUGH=()

usage() {
  cat <<'EOF'
usage: bash run_intruder_patrol_demo_full_stack_pc_remote_yolo.sh [options]

defaults:
  --mode normal
  --remote-url http://192.168.5.15:8765/detect
  --remote-jpeg-quality 90
  --remote-scale 1.0
  --match-thres 0.42
  --face-keep-score 0.58
  --known-hold-score 0.56

options:
  --mode normal|fast|profile
  --remote-url URL
  --remote-timeout SEC
  --remote-jpeg-quality Q
  --remote-scale SCALE

all remaining arguments are forwarded to the existing patrol demo script.
EOF
}

check_remote_detector() {
  /usr/bin/python3 - "$REMOTE_URL" "$REMOTE_TIMEOUT" <<'PY'
import json
import sys
import urllib.parse
import urllib.request

detect_url = sys.argv[1]
timeout = float(sys.argv[2])
parsed = urllib.parse.urlparse(detect_url)
path = parsed.path or ""
if path.endswith("/detect"):
    path = path[:-len("/detect")] + "/health"
elif path.endswith("/"):
    path = path + "health"
elif not path:
    path = "/health"
else:
    path = path + "/health"
health_url = urllib.parse.urlunparse(parsed._replace(path=path, params="", query="", fragment=""))
obj = json.loads(urllib.request.urlopen(health_url, timeout=timeout).read().decode())
print(json.dumps({"health_ok": obj.get("ok"), "model": obj.get("model"), "device": obj.get("device_name", obj.get("device"))}, ensure_ascii=False))
PY
}

while [ $# -gt 0 ]; do
  case "$1" in
    --help|-h)
      usage
      exit 0
      ;;
    --mode)
      MODE="$2"
      shift 2
      ;;
    --mode=*)
      MODE="${1#--mode=}"
      shift 1
      ;;
    --remote-url)
      REMOTE_URL="$2"
      shift 2
      ;;
    --remote-url=*)
      REMOTE_URL="${1#--remote-url=}"
      shift 1
      ;;
    --remote-timeout)
      REMOTE_TIMEOUT="$2"
      shift 2
      ;;
    --remote-timeout=*)
      REMOTE_TIMEOUT="${1#--remote-timeout=}"
      shift 1
      ;;
    --remote-jpeg-quality)
      REMOTE_JPEG_QUALITY="$2"
      shift 2
      ;;
    --remote-jpeg-quality=*)
      REMOTE_JPEG_QUALITY="${1#--remote-jpeg-quality=}"
      shift 1
      ;;
    --remote-scale)
      REMOTE_SCALE="$2"
      shift 2
      ;;
    --remote-scale=*)
      REMOTE_SCALE="${1#--remote-scale=}"
      shift 1
      ;;
    *)
      PASSTHROUGH+=("$1")
      shift 1
      ;;
  esac
done

check_remote_detector

exec bash /root/face_tools/run_intruder_patrol_demo_full_stack.sh \
  --mode "$MODE" \
  --detector-mode remote \
  --remote-url "$REMOTE_URL" \
  --remote-timeout "$REMOTE_TIMEOUT" \
  --remote-jpeg-quality "$REMOTE_JPEG_QUALITY" \
  --remote-scale "$REMOTE_SCALE" \
  --remote-discovery-interval "$REMOTE_DISCOVERY_INTERVAL" \
  --remote-discovery-jpeg-quality "$REMOTE_DISCOVERY_JPEG_QUALITY" \
  --remote-discovery-scale "$REMOTE_DISCOVERY_SCALE" \
  --remote-roi-max-tracks "$REMOTE_ROI_MAX_TRACKS" \
  --remote-roi-expand "$REMOTE_ROI_EXPAND" \
  --remote-roi-jpeg-quality "$REMOTE_ROI_JPEG_QUALITY" \
  --remote-roi-scale "$REMOTE_ROI_SCALE" \
  --conf "$CONF_THRES" \
  --track-low-conf "$TRACK_LOW_CONF" \
  --person-edge-min-conf "$PERSON_EDGE_MIN_CONF" \
  --match-thres "$MATCH_THRES" \
  --face-keep-score "$FACE_KEEP_SCORE" \
  --known-hold-score "$KNOWN_HOLD_SCORE" \
  --unknown-min-conf "$UNKNOWN_MIN_CONF" \
  "${PASSTHROUGH[@]}"
