#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PLAYGROUND_DIR="$(cd "${ROOT_DIR}/../../.." && pwd)"

JETSON_HOST="${JETSON_HOST:-jetson@10.66.121.109}"
JETSON_SSH_PROXY_COMMAND="${JETSON_SSH_PROXY_COMMAND:-}"
JETSON_RUNTIME_DIR="${JETSON_RUNTIME_DIR:-/home/jetson/SMART3D/jetson_runtime}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
CYCLONE_REMOTE="${CYCLONE_REMOTE:-/tmp/cyclone-campus-jetson.xml}"
I9_IP="${I9_IP:-10.66.112.64}"
REPORT_DIR="${REPORT_DIR:-${ROOT_DIR}/reports}"

START_ACTIVE="${START_ACTIVE:-0}"
START_WATCHDOG="${START_WATCHDOG:-1}"
START_STP23L="${START_STP23L:-1}"
START_STP23L_BRAKE_GUARD="${START_STP23L_BRAKE_GUARD:-1}"

log() {
  printf '[jetson-runtime] %s\n' "$*"
}

run_jetson() {
  local ssh_opts=(-o StrictHostKeyChecking=no)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  ssh "${ssh_opts[@]}" "${JETSON_HOST}" "$@"
}

copy_to_jetson() {
  local scp_opts=(-o StrictHostKeyChecking=no)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    scp_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  scp "${scp_opts[@]}" "$1" "${JETSON_HOST}:$2" >/dev/null
}

prepare_dds_xml() {
  run_jetson <<EOF
cat > "${CYCLONE_REMOTE}" <<'XML'
<CycloneDDS>
  <Domain id="${ROS_DOMAIN_ID}">
    <General>
      <AllowMulticast>true</AllowMulticast>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>40</MaxAutoParticipantIndex>
      <Peers>
        <Peer address="${I9_IP}"/>
      </Peers>
    </Discovery>
  </Domain>
</CycloneDDS>
XML
EOF
}

wait_for_jetson() {
  local timeout_sec="${1:-180}"
  local deadline=$((SECONDS + timeout_sec))
  local ssh_opts=(-o StrictHostKeyChecking=no -o ConnectTimeout=5)
  if [ -n "${JETSON_SSH_PROXY_COMMAND}" ]; then
    ssh_opts+=(-o "ProxyCommand=${JETSON_SSH_PROXY_COMMAND}")
  fi
  while [ "${SECONDS}" -lt "${deadline}" ]; do
    if ssh "${ssh_opts[@]}" "${JETSON_HOST}" "echo ok" >/dev/null 2>&1; then
      log "Jetson is reachable"
      return 0
    fi
    sleep 3
  done
  log "Timed out waiting for Jetson"
  return 1
}

stop_runtime() {
  run_jetson <<'EOF'
pkill -f '[c]artographer_node' 2>/dev/null || true
pkill -f '[o]ccupancy_grid_node' 2>/dev/null || true
pkill -f '[l]idar_supervisor.sh' 2>/dev/null || true
pkill -f '[l]slidar_driver_node' 2>/dev/null || true
pkill -f '[h]ipnuc_imu' 2>/dev/null || true
pkill -f '[/]home/jetson/SMART3D/ws_imu/install/hipnuc_imu/lib/hipnuc_imu/talker' 2>/dev/null || true
pkill -f '[/]hipnuc_imu/talker --ros-args' 2>/dev/null || true
pkill -f '[i]mu_qos_bridge.py' 2>/dev/null || true
pkill -f '[d]dsm115_odom.py' 2>/dev/null || true
pkill -f '[s]tp23l_range.py' 2>/dev/null || true
pkill -f '[s]tp23l_hard_brake_guard.py' 2>/dev/null || true
pkill -f '[k]inematic_icp_online_node' 2>/dev/null || true
pkill -f '[o]dom_tf_relay.py' 2>/dev/null || true
pkill -f '[s]can_metadata_normalizer.py' 2>/dev/null || true
pkill -f '[s]lam_toolbox' 2>/dev/null || true
pkill -f '[s]tatic_transform_publisher .* base_link laser_link' 2>/dev/null || true
pkill -f '[s]tatic_transform_publisher .* odom base_link' 2>/dev/null || true
pkill -f '[c]ampus_map_bridge.py' 2>/dev/null || true
pkill -f '[r]obot_state_publisher' 2>/dev/null || true
pkill -f '[p]i5_client.py' 2>/dev/null || true
pkill -f '[c]md_vel_driver.py' 2>/dev/null || true
pkill -f '[p]i_ros_watchdog.py' 2>/dev/null || true
for pid in $(ps -ef | awk '/hipnuc_imu\/talker/ && !/awk/ {print $2}'); do
  kill -9 "$pid" 2>/dev/null || true
done
sleep 2
EOF
  log "Jetson runtime stop requested"
}

start_cartographer() {
  prepare_dds_xml
  local carto_config="${SMART3D_CARTO_CONFIG_BASENAME:-robotcar_2d.lua}"
  local start_odom="${START_ODOM:-1}"
  local carto_use_scan_normalizer="${SMART3D_CARTO_USE_SCAN_NORMALIZER:-1}"
  local slam_scan_topic="${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}"

  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  export SMART3D_SLAM_BACKEND=cartographer && \
  export SMART3D_CARTO_CONFIG_BASENAME=${carto_config} && \
  export SMART3D_CARTO_USE_SCAN_NORMALIZER=${carto_use_scan_normalizer} && \
  export SMART3D_SLAM_SCAN_TOPIC=${slam_scan_topic} && \
  export START_ACTIVE=${START_ACTIVE} && \
  export START_WATCHDOG=${START_WATCHDOG} && \
  export START_ODOM=${start_odom} && \
  export START_STP23L=${START_STP23L} && \
  export START_STP23L_BRAKE_GUARD=${START_STP23L_BRAKE_GUARD} && \
  ./start_full_native.sh'
EOF
}

start_slam() {
  prepare_dds_xml
  local start_odom="${START_ODOM:-0}"
  local start_odom_static_tf="${START_ODOM_STATIC_TF:-1}"
  local start_odom_tf_relay="${START_ODOM_TF_RELAY:-1}"
  local slam_scan_topic="${SMART3D_SLAM_SCAN_TOPIC:-/scan_slam}"

  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  export SMART3D_SLAM_BACKEND=slam_toolbox && \
  export SMART3D_SLAM_SCAN_TOPIC=${slam_scan_topic} && \
  export START_ACTIVE=${START_ACTIVE} && \
  export START_WATCHDOG=${START_WATCHDOG} && \
  export START_ODOM=${start_odom} && \
  export START_ODOM_STATIC_TF=${start_odom_static_tf} && \
  export START_ODOM_TF_RELAY=${start_odom_tf_relay} && \
  export START_STP23L=${START_STP23L} && \
  export START_STP23L_BRAKE_GUARD=${START_STP23L_BRAKE_GUARD} && \
  ./start_full_native.sh'
EOF
}

start_sensors() {
  prepare_dds_xml
  local start_odom="${START_ODOM:-0}"
  local start_active="${START_ACTIVE:-0}"
  local start_watchdog="${START_WATCHDOG:-0}"
  local start_odom_static_tf="${START_ODOM_STATIC_TF:-0}"
  local start_odom_tf_relay="${START_ODOM_TF_RELAY:-0}"

  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  export SMART3D_SLAM_BACKEND=none && \
  export START_ACTIVE=${start_active} && \
  export START_WATCHDOG=${start_watchdog} && \
  export START_ODOM=${start_odom} && \
  export START_ODOM_STATIC_TF=${start_odom_static_tf} && \
  export START_ODOM_TF_RELAY=${start_odom_tf_relay} && \
  export START_STP23L=${START_STP23L} && \
  export START_STP23L_BRAKE_GUARD=${START_STP23L_BRAKE_GUARD} && \
  ./start_full_native.sh'
EOF
}

stop_odom_candidate() {
  run_jetson <<'EOF'
pkill -f '[/]rf2o_laser_odometry_node' 2>/dev/null || true
pkill -f '[/]laser_scan_matcher' 2>/dev/null || true
pkill -f '[k]inematic_icp_online_node' 2>/dev/null || true
pkill -f '[/]robot_localization/.*/ekf_node' 2>/dev/null || true
pkill -f '[e]kf_node --ros-args --params-file .*/ekf_rf2o_imu.yaml' 2>/dev/null || true
sleep 1
EOF
}

start_rf2o() {
  prepare_dds_xml
  stop_odom_candidate
  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  export RF2O_PUBLISH_TF=${RF2O_PUBLISH_TF:-true} && \
  ./start_rf2o_odom.sh'
EOF
}

start_laser_scan_matcher() {
  prepare_dds_xml
  stop_odom_candidate
  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  export LASER_SCAN_MATCHER_PUBLISH_TF=${LASER_SCAN_MATCHER_PUBLISH_TF:-true} && \
  ./start_laser_scan_matcher_odom.sh'
EOF
}

start_rf2o_ekf() {
  prepare_dds_xml
  stop_odom_candidate
  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  ./start_rf2o_ekf_imu.sh'
EOF
}

start_kinematic_icp() {
  prepare_dds_xml
  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  ./start_kinematic_icp.sh'
EOF
}

measure_odom() {
  prepare_dds_xml
  local candidate="${1:-generic}"
  local duration_sec="${2:-60}"
  local process_substring

  case "${candidate}" in
    rf2o)
      process_substring="rf2o_laser_odometry_node"
      ;;
    rf2o_ekf|ekf|fused)
      process_substring="ekf_node"
      ;;
    laser_scan_matcher|lsm)
      process_substring="laser_scan_matcher"
      ;;
    *)
      process_substring="${candidate}"
      ;;
  esac

  run_jetson <<EOF
bash -lc 'cd "${JETSON_RUNTIME_DIR}" && \
  set +u && source /opt/ros/${ROS_DISTRO}/setup.bash && set -u && \
  export CYCLONEDDS_URI=file://${CYCLONE_REMOTE} && \
  export ROS_DOMAIN_ID=${ROS_DOMAIN_ID} && \
  export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION} && \
  python3 ./measure_odom_candidate.py --duration-sec ${duration_sec} --process-substring ${process_substring}'
EOF
}

status_runtime() {
  prepare_dds_xml
  run_jetson <<'EOF'
echo '==== Processes ===='
ps -ef | grep -E 'lidar_supervisor.sh|lslidar_driver_node|hipnuc_imu|ddsm115_odom|stp23l_range|stp23l_hard_brake_guard|cartographer_node|occupancy_grid_node|slam_toolbox|scan_metadata_normalizer|campus_map_bridge|robot_state_publisher|pi5_client|cmd_vel_driver|pi_ros_watchdog|v4l2_camera' | grep -v grep || true
echo
echo '==== /dev/ttyUSB0 ===='
lsof /dev/ttyUSB0 || true
echo
echo '==== Video devices ===='
ls -l /dev/video* 2>/dev/null || true
EOF
}

healthcheck_runtime() {
  prepare_dds_xml
  run_jetson <<EOF
source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
export CYCLONEDDS_URI=file://${CYCLONE_REMOTE}
python3 - <<'PY'
import glob
import json
import pathlib
import subprocess
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, LaserScan, Range, Image, CameraInfo


def tail(path, lines=12):
    p = pathlib.Path(path)
    if not p.exists():
        return []
    return p.read_text(errors='replace').splitlines()[-lines:]


rclpy.init()
node = rclpy.create_node('jetson_runtime_healthcheck')
counts = {
    'scan': 0,
    'imu': 0,
    'front_range': 0,
    'map': 0,
    'map_campus': 0,
    'pose': 0,
    'image_raw': 0,
    'camera_info': 0,
}
last = {
    'map': None,
    'map_campus': None,
    'pose': None,
    'front_range': None,
}

node.create_subscription(
    LaserScan,
    '/scan',
    lambda msg: counts.__setitem__('scan', counts['scan'] + 1),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(
    Imu,
    '/IMU_data',
    lambda msg: counts.__setitem__('imu', counts['imu'] + 1),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(
    Range,
    '/front_range',
    lambda msg: (counts.__setitem__('front_range', counts['front_range'] + 1), last.__setitem__('front_range', msg.range)),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(
    OccupancyGrid,
    '/map',
    lambda msg: (counts.__setitem__('map', counts['map'] + 1), last.__setitem__('map', (msg.info.width, msg.info.height, msg.info.resolution))),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
node.create_subscription(
    OccupancyGrid,
    '/map_campus',
    lambda msg: (counts.__setitem__('map_campus', counts['map_campus'] + 1), last.__setitem__('map_campus', (msg.info.width, msg.info.height, msg.info.resolution))),
    QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.TRANSIENT_LOCAL),
)
node.create_subscription(
    PoseStamped,
    '/car1/pose',
    lambda msg: (counts.__setitem__('pose', counts['pose'] + 1), last.__setitem__('pose', (msg.pose.position.x, msg.pose.position.y))),
    QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
)
node.create_subscription(Image, '/image_raw', lambda msg: counts.__setitem__('image_raw', counts['image_raw'] + 1), 10)
node.create_subscription(CameraInfo, '/camera_info', lambda msg: counts.__setitem__('camera_info', counts['camera_info'] + 1), 10)

end = time.time() + 6.0
while time.time() < end:
    rclpy.spin_once(node, timeout_sec=0.2)

topic_names_and_types = node.get_topic_names_and_types()
interesting_topics = {}
for topic, types in sorted(topic_names_and_types):
    if topic in ['/scan', '/IMU_data', '/front_range', '/map', '/map_campus', '/car1/pose', '/image_raw', '/camera_info']:
        interesting_topics[topic] = {
            'types': types,
            'publishers': node.count_publishers(topic),
            'subscribers': node.count_subscribers(topic),
        }

node.destroy_node()
rclpy.shutdown()

result = {
    'counts_6s': counts,
    'last': last,
    'topics': interesting_topics,
    'video_devices': glob.glob('/dev/video*'),
    'tty_usb0_owner': subprocess.run('lsof /dev/ttyUSB0 || true', shell=True, text=True, capture_output=True).stdout.strip().splitlines(),
    'processes': subprocess.run(
        "ps -ef | grep -E 'lidar_supervisor.sh|lslidar_driver_node|hipnuc_imu|ddsm115_odom|stp23l_range|stp23l_hard_brake_guard|cartographer_node|occupancy_grid_node|slam_toolbox|scan_metadata_normalizer|campus_map_bridge|pi5_client|cmd_vel_driver|pi_ros_watchdog|v4l2_camera' | grep -v grep || true",
        shell=True,
        text=True,
        capture_output=True,
    ).stdout.strip().splitlines(),
    'logs': {
        'lidar': tail('/tmp/lidar.log'),
        'imu': tail('/tmp/imu_talker.log'),
        'odom': tail('/tmp/odom.log'),
        'stp23l': tail('/tmp/stp23l.log'),
        'stp23l_guard': tail('/tmp/stp23l_hard_brake_guard.log'),
        'cartographer': tail('/tmp/cartographer.log'),
        'slam_toolbox': tail('/tmp/slam_toolbox.log'),
        'pi5_client': tail('/tmp/pi5_client_campus.log'),
        'camera': tail('/tmp/v4l2_camera.log'),
    },
}
print(json.dumps(result, indent=2))
PY
EOF
}

reboot_jetson() {
  log "Rebooting Jetson"
  if [ -n "${JETSON_SUDO_PASSWORD:-}" ]; then
    run_jetson \
      "printf '%s\n' '${JETSON_SUDO_PASSWORD}' | sudo -S reboot" >/dev/null 2>&1 || true
  else
    run_jetson "sudo reboot" >/dev/null 2>&1 || true
  fi
}

benchmark_one() {
  local backend="$1"
  local duration_sec="${2:-90}"
  local output_json="${3:-}"

  if [ -z "${output_json}" ]; then
    mkdir -p "${REPORT_DIR}"
    output_json="${REPORT_DIR}/${backend}_benchmark_$(date +%Y-%m-%d_%H%M%S).json"
  fi

  mkdir -p "$(dirname "${output_json}")"
  prepare_dds_xml

  run_jetson <<EOF >"${output_json}"
source /opt/ros/${ROS_DISTRO}/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION}
export CYCLONEDDS_URI=file://${CYCLONE_REMOTE}
export SMART3D_BENCHMARK_BACKEND=${backend}
export SMART3D_BENCHMARK_DURATION=${duration_sec}
python3 - <<'PY'
import json
import os
import pathlib
import psutil
import time

backend = os.environ['SMART3D_BENCHMARK_BACKEND']
duration_sec = float(os.environ['SMART3D_BENCHMARK_DURATION'])

patterns = {
    'lidar': ['lslidar_driver_node'],
    'imu': ['hipnuc_imu/talker', 'hipnuc_imu talker'],
    'cartographer': ['cartographer_node', 'occupancy_grid_node'],
    'slam_toolbox': ['async_slam_toolbox_node'],
    'scan_normalizer': ['scan_metadata_normalizer.py'],
    'ddsm_odom': ['ddsm115_odom.py'],
    'stp23l': ['stp23l_range.py'],
    'stp23l_guard': ['stp23l_hard_brake_guard.py'],
    'map_bridge': ['campus_map_bridge.py'],
    'pi5_client': ['pi5_client.py'],
    'watchdog': ['pi_ros_watchdog.py'],
}


def tail(path, lines=20):
    p = pathlib.Path(path)
    if not p.exists():
        return []
    return p.read_text(errors='replace').splitlines()[-lines:]


def classify(cmdline):
    text = ' '.join(cmdline)
    matched = []
    for key, variants in patterns.items():
        if any(variant in text for variant in variants):
            matched.append(key)
    return matched


prime = []
for proc in psutil.process_iter(['pid', 'cmdline']):
    try:
        if classify(proc.info['cmdline'] or []):
            proc.cpu_percent(None)
            prime.append(proc)
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        pass

time.sleep(1.0)

samples = []
start = time.time()
while time.time() - start < duration_sec:
    ts = time.time()
    totals = {}
    alive = {}
    for proc in psutil.process_iter(['pid', 'cmdline', 'memory_info']):
        try:
            cmdline = proc.info['cmdline'] or []
            groups = classify(cmdline)
            if not groups:
                continue
            cpu = proc.cpu_percent(None)
            rss = int(proc.info['memory_info'].rss)
            for group in groups:
                totals.setdefault(group, {'cpu_percent': 0.0, 'rss_bytes': 0})
                totals[group]['cpu_percent'] += cpu
                totals[group]['rss_bytes'] += rss
                alive[group] = True
        except (psutil.NoSuchProcess, psutil.AccessDenied):
            pass
    samples.append({'t': ts, 'totals': totals, 'alive': alive})
    time.sleep(2.0)

summary = {}
for sample in samples:
    for group, metrics in sample['totals'].items():
        info = summary.setdefault(group, {'cpu_samples': [], 'rss_samples': [], 'alive_samples': 0})
        info['cpu_samples'].append(metrics['cpu_percent'])
        info['rss_samples'].append(metrics['rss_bytes'])
        info['alive_samples'] += 1

for group, info in summary.items():
    cpu_samples = info.pop('cpu_samples')
    rss_samples = info.pop('rss_samples')
    info['avg_cpu_percent'] = round(sum(cpu_samples) / len(cpu_samples), 2) if cpu_samples else 0.0
    info['max_cpu_percent'] = round(max(cpu_samples), 2) if cpu_samples else 0.0
    info['avg_rss_mb'] = round((sum(rss_samples) / len(rss_samples)) / (1024 * 1024), 2) if rss_samples else 0.0
    info['max_rss_mb'] = round(max(rss_samples) / (1024 * 1024), 2) if rss_samples else 0.0

result = {
    'backend': backend,
    'duration_sec': duration_sec,
    'sample_count': len(samples),
    'summary': summary,
    'logs': {
        'lidar': tail('/tmp/lidar.log'),
        'cartographer': tail('/tmp/cartographer.log'),
        'slam_toolbox': tail('/tmp/slam_toolbox.log'),
        'odom': tail('/tmp/odom.log'),
        'pi5_client': tail('/tmp/pi5_client_campus.log'),
    },
}
print(json.dumps(result, indent=2))
PY
EOF
}

usage() {
  cat <<'EOF'
Usage:
  jetson_runtime_ctl.sh deploy
  jetson_runtime_ctl.sh reboot
  jetson_runtime_ctl.sh wait
  jetson_runtime_ctl.sh stop
  jetson_runtime_ctl.sh stop-odom-candidate
  jetson_runtime_ctl.sh start-sensors
  jetson_runtime_ctl.sh start-rf2o
  jetson_runtime_ctl.sh start-rf2o-ekf
  jetson_runtime_ctl.sh start-laser-scan-matcher
  jetson_runtime_ctl.sh start-kinematic-icp
  jetson_runtime_ctl.sh start-cartographer
  jetson_runtime_ctl.sh start-slam
  jetson_runtime_ctl.sh status
  jetson_runtime_ctl.sh healthcheck
  jetson_runtime_ctl.sh measure-odom <rf2o|laser_scan_matcher> [duration_sec]
  jetson_runtime_ctl.sh benchmark-one <cartographer|slam_toolbox> [duration_sec] [output_json]
EOF
}

cmd="${1:-}"
case "${cmd}" in
  deploy)
    bash "${ROOT_DIR}/deploy_runtime_to_jetson.sh"
    ;;
  reboot)
    reboot_jetson
    ;;
  wait)
    wait_for_jetson "${2:-180}"
    ;;
  stop)
    stop_runtime
    ;;
  stop-odom-candidate)
    stop_odom_candidate
    ;;
  start-sensors)
    start_sensors
    ;;
  start-rf2o)
    start_rf2o
    ;;
  start-rf2o-ekf)
    start_rf2o_ekf
    ;;
  start-laser-scan-matcher)
    start_laser_scan_matcher
    ;;
  start-kinematic-icp)
    start_kinematic_icp
    ;;
  start-cartographer)
    start_cartographer
    ;;
  start-slam)
    start_slam
    ;;
  status)
    status_runtime
    ;;
  healthcheck)
    healthcheck_runtime
    ;;
  measure-odom)
    measure_odom "${2:-generic}" "${3:-60}"
    ;;
  benchmark-one)
    [ $# -ge 2 ] || { usage; exit 1; }
    export SMART3D_BENCHMARK_BACKEND="$2"
    export SMART3D_BENCHMARK_DURATION="${3:-90}"
    benchmark_one "$2" "${3:-90}" "${4:-}"
    ;;
  *)
    usage
    exit 1
    ;;
esac
