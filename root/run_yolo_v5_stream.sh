#!/bin/bash
set -e
source /usr/local/Ascend/ascend-toolkit/set_env.sh
source /usr/local/Ascend/nnae/set_env.sh
exec /usr/local/miniconda3/bin/python -u /root/yolo_v5_stream.py "$@"
