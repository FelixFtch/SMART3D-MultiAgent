#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export SMART3D_SLAM_BACKEND="${SMART3D_SLAM_BACKEND:-cartographer}"
export SMART3D_CARTO_CONFIG_BASENAME="${SMART3D_CARTO_CONFIG_BASENAME:-robotcar_2d_odom.lua}"
export SMART3D_CARTO_USE_SCAN_NORMALIZER="${SMART3D_CARTO_USE_SCAN_NORMALIZER:-1}"

export START_ODOM="${START_ODOM:-1}"
export START_ACTIVE="${START_ACTIVE:-0}"
export START_WATCHDOG="${START_WATCHDOG:-0}"
export START_STP23L="${START_STP23L:-1}"
export START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-0}"
export START_ODOM_TF_RELAY="${START_ODOM_TF_RELAY:-0}"
export START_ODOM_STATIC_TF="${START_ODOM_STATIC_TF:-0}"

echo "[pi5-fusion] backend=${SMART3D_SLAM_BACKEND} config=${SMART3D_CARTO_CONFIG_BASENAME}"
echo "[pi5-fusion] odom=${START_ODOM} active=${START_ACTIVE} watchdog=${START_WATCHDOG}"
echo "[pi5-fusion] stp23l=${START_STP23L} brake_guard=${START_STP23L_BRAKE_GUARD}"

exec "${ROOT_DIR}/start_full_native.sh" "$@"
