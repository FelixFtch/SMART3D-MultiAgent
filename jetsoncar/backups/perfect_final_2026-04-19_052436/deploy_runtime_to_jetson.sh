#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLAYGROUND_DIR="$(cd "${ROOT_DIR}/../../.." && pwd)"
PRIVATE_REPOS_DIR="$(cd "${PLAYGROUND_DIR}/../private-repos" && pwd)"
SMART3D_DEPLOY_DIR="${SMART3D_DEPLOY_DIR:-${PRIVATE_REPOS_DIR}/SMART3D-MultiAgent/deploy}"
ROBOTICS_HELPERS_DIR="${ROBOTICS_HELPERS_DIR:-${PLAYGROUND_DIR}/robotics_helpers}"

JETSON_HOST="${JETSON_HOST:-jetson@10.66.121.109}"
JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND:-}"
JETSON_RUNTIME_DIR="${JETSON_RUNTIME_DIR:-/home/jetson/SMART3D/jetson_runtime}"

run_jetson() {
  local ssh_opts=(-o StrictHostKeyChecking=no)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "${JETSON_HOST}" "$@"
}

copy_file() {
  local src="$1"
  local dst_name="${2:-$(basename "$src")}"
  local scp_opts=(-o StrictHostKeyChecking=no)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    scp_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  scp "${scp_opts[@]}" "$src" "${JETSON_HOST}:${JETSON_RUNTIME_DIR}/${dst_name}" >/dev/null
}

run_jetson "mkdir -p '${JETSON_RUNTIME_DIR}'"

copy_file "${ROOT_DIR}/mk_imu_dev.sh"
copy_file "${ROOT_DIR}/rf2o_odom_params.yaml"
copy_file "${ROOT_DIR}/rf2o_odom_for_ekf_params.yaml"
copy_file "${ROOT_DIR}/ekf_rf2o_imu.yaml"
copy_file "${ROOT_DIR}/laser_scan_matcher_params.yaml"
copy_file "${ROOT_DIR}/start_rf2o_odom.sh"
copy_file "${ROOT_DIR}/start_rf2o_ekf_imu.sh"
copy_file "${ROOT_DIR}/start_laser_scan_matcher_odom.sh"
copy_file "${ROOT_DIR}/start_kinematic_icp.sh"
copy_file "${ROOT_DIR}/measure_odom_candidate.py"
copy_file "${ROOT_DIR}/lidar_supervisor.sh"
copy_file "${ROOT_DIR}/robotcar.urdf"
copy_file "${ROOT_DIR}/start_full_native.sh"
copy_file "${ROOT_DIR}/start_pi5_three_sensor_fusion.sh"
copy_file "${ROOT_DIR}/start_pi5_reference_passive.sh"
copy_file "${ROOT_DIR}/start_pi5_reference_campus_support.sh"
copy_file "${ROOT_DIR}/start_pi5_reference_active.sh"
copy_file "${ROOT_DIR}/odom_supervisor.sh"
copy_file "${ROOT_DIR}/odom_tf_relay.py"
copy_file "${ROOT_DIR}/scan_metadata_normalizer.py"
copy_file "${ROOT_DIR}/turn_sync_probe.py"
copy_file "${ROOT_DIR}/inspect_stationary_drift.py"
copy_file "${ROOT_DIR}/inspect_imu_bias.py"
copy_file "${ROOT_DIR}/run_robotcar_wasdqe_test.sh"
copy_file "${ROOT_DIR}/slam_toolbox_launch.py"
copy_file "${ROOT_DIR}/mapper_params_online_async.yaml"
copy_file "${ROOT_DIR}/stp23l_hard_brake_guard.py"
copy_file "${ROOT_DIR}/../pi/mk_m10p_dev.sh"
copy_file "${ROOT_DIR}/../pi/mk_ddsm_dev.sh"
copy_file "${ROOT_DIR}/../pi/campus_map_bridge.py"
copy_file "${ROOT_DIR}/../pi/pi_ros_watchdog.py"
copy_file "${SMART3D_DEPLOY_DIR}/pi5_client.py"
copy_file "${SMART3D_DEPLOY_DIR}/cmd_vel_driver.py"
copy_file "${SMART3D_DEPLOY_DIR}/cmd_vel_driver.py" "cmd_vel_driver_single_owner.py"
copy_file "${SMART3D_DEPLOY_DIR}/pi5_reference/imu_qos_bridge.py"
copy_file "${SMART3D_DEPLOY_DIR}/pi5_reference/cmd_vel_driver.py" "cmd_vel_driver_pi5_reference.py"
copy_file "${SMART3D_DEPLOY_DIR}/pi5_reference/ddsm115_odom.py"
copy_file "${SMART3D_DEPLOY_DIR}/cartographer_config/cartographer_launch.py"
copy_file "${SMART3D_DEPLOY_DIR}/cartographer_config/robotcar_2d.lua"
copy_file "${SMART3D_DEPLOY_DIR}/cartographer_config/robotcar_2d_odom.lua"
copy_file "${SMART3D_DEPLOY_DIR}/cartographer_config/stp23l_range.py"
copy_file "${SMART3D_DEPLOY_DIR}/cartographer_config/mk_stp23l_dev.sh"
copy_file "${PLAYGROUND_DIR}/ddsm_hard_stop.py"
copy_file "${PLAYGROUND_DIR}/ddsm_emergency_brake.py"
copy_file "${ROBOTICS_HELPERS_DIR}/robotcar_wasdqe_test.py"

if [ -f "${ROOT_DIR}/m10p_uart.yaml" ]; then
  copy_file "${ROOT_DIR}/m10p_uart.yaml"
fi

run_jetson \
  "chmod +x '${JETSON_RUNTIME_DIR}/mk_imu_dev.sh' \
             '${JETSON_RUNTIME_DIR}/start_rf2o_odom.sh' \
             '${JETSON_RUNTIME_DIR}/start_rf2o_ekf_imu.sh' \
             '${JETSON_RUNTIME_DIR}/start_laser_scan_matcher_odom.sh' \
             '${JETSON_RUNTIME_DIR}/start_kinematic_icp.sh' \
             '${JETSON_RUNTIME_DIR}/measure_odom_candidate.py' \
             '${JETSON_RUNTIME_DIR}/lidar_supervisor.sh' \
             '${JETSON_RUNTIME_DIR}/mk_m10p_dev.sh' \
             '${JETSON_RUNTIME_DIR}/mk_ddsm_dev.sh' \
             '${JETSON_RUNTIME_DIR}/mk_stp23l_dev.sh' \
             '${JETSON_RUNTIME_DIR}/start_full_native.sh' \
             '${JETSON_RUNTIME_DIR}/start_pi5_three_sensor_fusion.sh' \
             '${JETSON_RUNTIME_DIR}/start_pi5_reference_passive.sh' \
             '${JETSON_RUNTIME_DIR}/start_pi5_reference_campus_support.sh' \
             '${JETSON_RUNTIME_DIR}/start_pi5_reference_active.sh' \
             '${JETSON_RUNTIME_DIR}/odom_supervisor.sh' \
             '${JETSON_RUNTIME_DIR}/odom_tf_relay.py' \
             '${JETSON_RUNTIME_DIR}/scan_metadata_normalizer.py' \
             '${JETSON_RUNTIME_DIR}/turn_sync_probe.py' \
             '${JETSON_RUNTIME_DIR}/inspect_stationary_drift.py' \
             '${JETSON_RUNTIME_DIR}/inspect_imu_bias.py' \
             '${JETSON_RUNTIME_DIR}/run_robotcar_wasdqe_test.sh' \
             '${JETSON_RUNTIME_DIR}/slam_toolbox_launch.py' \
             '${JETSON_RUNTIME_DIR}/stp23l_hard_brake_guard.py' \
             '${JETSON_RUNTIME_DIR}/cmd_vel_driver_pi5_reference.py' \
             '${JETSON_RUNTIME_DIR}/cmd_vel_driver_single_owner.py'"

echo "Deployed Jetson runtime to ${JETSON_HOST}:${JETSON_RUNTIME_DIR}"
