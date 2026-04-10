#!/usr/bin/env bash
set -euo pipefail

RUN_LABEL="${1:-manual}"
VARIANT_OVERRIDE="${2:-}"
TS="$(date +%Y%m%d_%H%M%S)"
STATE_DIR="/root/face_tools/input_chain_ab"
EVIDENCE_ROOT="${STATE_DIR}/runs"
OUT_DIR="${EVIDENCE_ROOT}/${TS}_${RUN_LABEL}"

mkdir -p "${OUT_DIR}"

copy_if_exists() {
  local src="$1"
  local dst="$2"
  if [[ -f "${src}" ]]; then
    cp -f "${src}" "${dst}"
  fi
}

copy_if_exists "/tmp/mapping_sensor_launch.log" "${OUT_DIR}/mapping_sensor_launch.log"
copy_if_exists "/tmp/mapping_imu_encoder_mix.log" "${OUT_DIR}/mapping_imu_encoder_mix.log"
copy_if_exists "/tmp/mapping_odom_tf.log" "${OUT_DIR}/mapping_odom_tf.log"
copy_if_exists "/tmp/mapping_slam.log" "${OUT_DIR}/mapping_slam.log"

if [[ -f "${STATE_DIR}/active_variant.txt" ]]; then
  cp -f "${STATE_DIR}/active_variant.txt" "${OUT_DIR}/active_variant.txt"
fi

ls -1t /root/.ros/log > "${OUT_DIR}/ros_log_listing.txt" || true
LATEST_ROS_DIR="$(ls -1dt /root/.ros/log/*/ 2>/dev/null | head -n 1 || true)"
if [[ -n "${LATEST_ROS_DIR}" ]]; then
  cp -a "${LATEST_ROS_DIR}" "${OUT_DIR}/latest_ros_log_dir"
fi

{
  if [[ -n "${VARIANT_OVERRIDE}" ]]; then
    ACTIVE_VARIANT="${VARIANT_OVERRIDE}"
  else
    ACTIVE_VARIANT="$(cat "${STATE_DIR}/active_variant.txt" 2>/dev/null || echo unknown)"
  fi
  echo "label=${RUN_LABEL}"
  echo "timestamp=${TS}"
  echo "active_variant=${ACTIVE_VARIANT}"
  echo
  echo "[sensor]"
  grep -nE 'Write timeout|input unstable|dt abnormal' "${OUT_DIR}/mapping_sensor_launch.log" 2>/dev/null || true
  echo
  echo "[imu_encoder_mix]"
  grep -nE 'large dt|stale imu|skip integration|untrusted' "${OUT_DIR}/mapping_imu_encoder_mix.log" 2>/dev/null || true
  echo
  echo "[odom_tf]"
  tail -n 20 "${OUT_DIR}/mapping_odom_tf.log" 2>/dev/null || true
  echo
  echo "[slam]"
  grep -nE 'queue is full|Scan Matching Failed|update ld=' "${OUT_DIR}/mapping_slam.log" 2>/dev/null || true
} > "${OUT_DIR}/summary.txt"

echo "[INFO] saved evidence to ${OUT_DIR}"
