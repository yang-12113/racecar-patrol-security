#!/usr/bin/env bash
set -euo pipefail

ENCODER_SRC="/home/racecar/src/encoder/encoder/encoder_node.py"
IMU_SRC="/home/racecar/src/encoder_imu/src/imu_encoder_mix.cpp"
ODOM_SRC="/home/racecar/src/odom_tf/src/odom_tf.cpp"

ENCODER_OLD="${ENCODER_SRC}.bak_codex_20260408_input_chain_round2"
IMU_OLD="${IMU_SRC}.bak_codex_20260408_input_chain_round2"
ODOM_OLD="${ODOM_SRC}.bak_codex_20260408_input_chain_round2"

STATE_DIR="/root/face_tools/input_chain_ab"
CURRENT_DIR="${STATE_DIR}/current_snapshot"
ACTIVE_FILE="${STATE_DIR}/active_variant.txt"
LAST_BUILD_LOG="${STATE_DIR}/last_build.log"

mkdir -p "${CURRENT_DIR}"

usage() {
  cat <<'EOF'
Usage:
  bash input_chain_ab_switch.sh capture-current
  bash input_chain_ab_switch.sh status
  bash input_chain_ab_switch.sh activate A|B|C|D|E|F

Variants:
  A current encoder + current imu_encoder_mix + current odom_tf
  B old encoder + current imu_encoder_mix + current odom_tf
  C current encoder + old imu_encoder_mix + current odom_tf
  D old encoder + old imu_encoder_mix + current odom_tf
  E current encoder + current imu_encoder_mix + old odom_tf
  F old encoder + old imu_encoder_mix + old odom_tf

Notes:
  - "old" means *.bak_codex_20260408_input_chain_round2
  - This script only switches the input-chain files needed for A/B diagnosis.
  - It refuses to switch while mapping-related processes are still running.
EOF
}

require_file() {
  local path="$1"
  if [[ ! -f "$path" ]]; then
    echo "[ERROR] missing file: $path" >&2
    exit 1
  fi
}

ensure_backups_exist() {
  require_file "${ENCODER_OLD}"
  require_file "${IMU_OLD}"
  require_file "${ODOM_OLD}"
}

ensure_current_snapshot() {
  if [[ ! -f "${CURRENT_DIR}/encoder_node.py" ]]; then
    cp -f "${ENCODER_SRC}" "${CURRENT_DIR}/encoder_node.py"
  fi
  if [[ ! -f "${CURRENT_DIR}/imu_encoder_mix.cpp" ]]; then
    cp -f "${IMU_SRC}" "${CURRENT_DIR}/imu_encoder_mix.cpp"
  fi
  if [[ ! -f "${CURRENT_DIR}/odom_tf.cpp" ]]; then
    cp -f "${ODOM_SRC}" "${CURRENT_DIR}/odom_tf.cpp"
  fi
}

ensure_idle() {
  local matches
  matches="$(pgrep -af 'encoder_vel|imu_encoder_mix|odom_tf|slam_gmapping|Run_car.launch.py|Run_mapping.launch.py|launch_ros' || true)"
  if [[ -n "${matches}" ]]; then
    echo "[ERROR] mapping-related processes are still running; stop them before switching variants." >&2
    echo "${matches}" >&2
    exit 1
  fi
}

copy_variant() {
  local encoder_from="$1"
  local imu_from="$2"
  local odom_from="$3"

  cp -f "${encoder_from}" "${ENCODER_SRC}"
  cp -f "${imu_from}" "${IMU_SRC}"
  cp -f "${odom_from}" "${ODOM_SRC}"
}

build_cpp_packages() {
  echo "[INFO] rebuilding encoder_imu and odom_tf ..."
  (
    cd /home/racecar
    source /opt/ros/humble/setup.bash
    colcon build --packages-select encoder_imu odom_tf
  ) >"${LAST_BUILD_LOG}" 2>&1
  echo "[INFO] build complete; log saved to ${LAST_BUILD_LOG}"
}

print_hash() {
  local label="$1"
  local path="$2"
  printf '%-24s %s  %s\n' "${label}" "$(sha256sum "$path" | awk '{print $1}')" "${path}"
}

status() {
  echo "[INFO] active variant: $(cat "${ACTIVE_FILE}" 2>/dev/null || echo unknown)"
  print_hash "encoder src" "${ENCODER_SRC}"
  print_hash "encoder current" "${CURRENT_DIR}/encoder_node.py"
  print_hash "encoder old" "${ENCODER_OLD}"
  print_hash "imu src" "${IMU_SRC}"
  print_hash "imu current" "${CURRENT_DIR}/imu_encoder_mix.cpp"
  print_hash "imu old" "${IMU_OLD}"
  print_hash "odom src" "${ODOM_SRC}"
  print_hash "odom current" "${CURRENT_DIR}/odom_tf.cpp"
  print_hash "odom old" "${ODOM_OLD}"
}

activate_variant() {
  local variant="$1"
  local encoder_from="${CURRENT_DIR}/encoder_node.py"
  local imu_from="${CURRENT_DIR}/imu_encoder_mix.cpp"
  local odom_from="${CURRENT_DIR}/odom_tf.cpp"

  case "${variant}" in
    A)
      ;;
    B)
      encoder_from="${ENCODER_OLD}"
      ;;
    C)
      imu_from="${IMU_OLD}"
      ;;
    D)
      encoder_from="${ENCODER_OLD}"
      imu_from="${IMU_OLD}"
      ;;
    E)
      odom_from="${ODOM_OLD}"
      ;;
    F)
      encoder_from="${ENCODER_OLD}"
      imu_from="${IMU_OLD}"
      odom_from="${ODOM_OLD}"
      ;;
    *)
      echo "[ERROR] unknown variant: ${variant}" >&2
      usage
      exit 1
      ;;
  esac

  ensure_idle
  copy_variant "${encoder_from}" "${imu_from}" "${odom_from}"
  build_cpp_packages
  echo "${variant}" > "${ACTIVE_FILE}"
  echo "[INFO] activated variant ${variant}"
  status
}

main() {
  if [[ $# -lt 1 ]]; then
    usage
    exit 1
  fi

  ensure_backups_exist

  case "$1" in
    capture-current)
      ensure_current_snapshot
      echo "A" > "${ACTIVE_FILE}"
      echo "[INFO] captured current snapshot in ${CURRENT_DIR}"
      status
      ;;
    status)
      ensure_current_snapshot
      status
      ;;
    activate)
      if [[ $# -ne 2 ]]; then
        usage
        exit 1
      fi
      ensure_current_snapshot
      activate_variant "$2"
      ;;
    *)
      usage
      exit 1
      ;;
  esac
}

main "$@"
