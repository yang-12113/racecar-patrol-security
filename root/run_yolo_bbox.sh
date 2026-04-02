#!/bin/bash
set -e
source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /usr/local/Ascend/nnae/set_env.sh
PYBIN=/usr/local/miniconda3/bin/python
if [ ! -x "$PYBIN" ]; then
  PYBIN=python3
fi
"$PYBIN" /root/yolo_om_cam_bbox.py "$@"