#!/bin/bash
set -e
REMOTE_URL="http://192.168.5.15:8765/detect"
PYBIN=/usr/local/miniconda3/bin/python
if [ ! -x "$PYBIN" ]; then
  PYBIN=python3
fi

while [ $# -gt 0 ]; do
  case "$1" in
    --remote-url)
      REMOTE_URL="$2"
      shift 2
      ;;
    --remote-url=*)
      REMOTE_URL="${1#--remote-url=}"
      shift 1
      ;;
    *)
      break
      ;;
  esac
done

source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /usr/local/Ascend/nnae/set_env.sh

$PYBIN - "$REMOTE_URL" <<'PY'
import json
import sys
import urllib.parse
import urllib.request
url = sys.argv[1]
parsed = urllib.parse.urlparse(url)
path = parsed.path or ''
if path.endswith('/detect'):
    path = path[:-len('/detect')] + '/health'
elif path.endswith('/'):
    path = path + 'health'
elif not path:
    path = '/health'
else:
    path = path + '/health'
health = urllib.parse.urlunparse(parsed._replace(path=path, params='', query='', fragment=''))
obj = json.loads(urllib.request.urlopen(health, timeout=8).read().decode())
print(json.dumps({'remote_ok': obj.get('ok'), 'model': obj.get('model'), 'device': obj.get('device_name', obj.get('device'))}, ensure_ascii=False))
PY

exec "$PYBIN" /root/face_tools/yolo_face_track_alarm.py   --detector-mode remote   --remote-url "$REMOTE_URL"   --follow-policy none   --face-db /tmp/nonexistent_face_db.npz   --no-record   --no-show   --print-every 0   --stream-port 8081   --stream-max-fps 12   --stream-jpeg-quality 85   --stream-scale 1.0   "$@"
