#!/bin/bash
set -e
exec /root/run_yolo_v5_stream.sh --stream-port 8091 --print-every 0 --conf 0.25 --target-cls -1 "$@"
