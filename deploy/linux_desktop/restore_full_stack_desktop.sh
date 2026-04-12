#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${1:-$HOME/SMART3D-MultiAgent}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
SKIP_APT="${SKIP_APT:-0}"
SKIP_BUILD="${SKIP_BUILD:-0}"

SNAPSHOT_DIR="$REPO_DIR/field/jetson_m3pro_original_2026-04-12"
OVERLAY_DIR="$REPO_DIR/extras/m3pro_auto_mapper21_overlay"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

MIRCO_WS="$HOME/mircoROS_agent"
YAHBOOM_WS="$HOME/yahboomcar_ws"
M3_WS="$HOME/M3Pro_ws"

BACKUP_ROOT="$HOME/smart3d_desktop_restore_backup_$(date +%Y%m%d_%H%M%S)"
REMAINING_REF_REPORT="$HOME/smart3d_desktop_remaining_refs.txt"

need_dir() {
  if [ ! -d "$1" ]; then
    echo "Missing directory: $1"
    exit 1
  fi
}

apt_install() {
  sudo apt-get install -y "$@"
}

echo "========== [1] Validate inputs =========="
need_dir "$REPO_DIR"
need_dir "$SNAPSHOT_DIR"
need_dir "$SNAPSHOT_DIR/mircoROS_agent_src"
need_dir "$SNAPSHOT_DIR/yahboomcar_ws_src"
need_dir "$SNAPSHOT_DIR/M3Pro_ws_src"
need_dir "$OVERLAY_DIR"

if [ ! -f "$ROS_SETUP" ]; then
  echo "ROS setup not found: $ROS_SETUP"
  echo "Please install ROS 2 Humble on Ubuntu 22.04 first."
  exit 1
fi

echo "========== [2] Backup existing desktop workspaces =========="
mkdir -p "$BACKUP_ROOT"
cp -a "$HOME/.bashrc" "$BACKUP_ROOT/.bashrc" 2>/dev/null || true
cp -a "$HOME/start_agent.sh" "$BACKUP_ROOT/start_agent.sh" 2>/dev/null || true
cp -a "$HOME/start_agent_desktop_clone.sh" "$BACKUP_ROOT/start_agent_desktop_clone.sh" 2>/dev/null || true
cp -a "$HOME/smart3d_desktop_env.sh" "$BACKUP_ROOT/smart3d_desktop_env.sh" 2>/dev/null || true

for ws in "$MIRCO_WS" "$YAHBOOM_WS" "$M3_WS"; do
  if [ -d "$ws/src" ]; then
    mkdir -p "$BACKUP_ROOT"
    cp -a "$ws" "$BACKUP_ROOT/" 2>/dev/null || true
  fi
done

echo "Backup root: $BACKUP_ROOT"

if [ "$SKIP_APT" != "1" ]; then
  echo "========== [3] Install desktop build/runtime dependencies =========="
  sudo apt-get update
  apt_install \
    build-essential cmake git rsync curl gnome-terminal \
    python3-pip python3-venv python3-colcon-common-extensions \
    python3-rosdep python3-vcstool python3-argcomplete \
    python3-opencv python3-numpy python3-scipy python3-sklearn \
    python3-yaml python3-serial python3-pyaudio \
    portaudio19-dev libgflags-dev nlohmann-json3-dev \
    libgoogle-glog-dev libdw-dev libusb-1.0-0-dev libudev-dev libeigen3-dev

  apt_install \
    "ros-${ROS_DISTRO}-navigation2" \
    "ros-${ROS_DISTRO}-nav2-bringup" \
    "ros-${ROS_DISTRO}-cartographer" \
    "ros-${ROS_DISTRO}-cartographer-ros" \
    "ros-${ROS_DISTRO}-slam-toolbox" \
    "ros-${ROS_DISTRO}-rtabmap" \
    "ros-${ROS_DISTRO}-rtabmap-ros" \
    "ros-${ROS_DISTRO}-rtabmap-launch" \
    "ros-${ROS_DISTRO}-rtabmap-odom" \
    "ros-${ROS_DISTRO}-robot-localization" \
    "ros-${ROS_DISTRO}-imu-filter-madgwick" \
    "ros-${ROS_DISTRO}-rosbridge-server" \
    "ros-${ROS_DISTRO}-tf-transformations" \
    "ros-${ROS_DISTRO}-cv-bridge" \
    "ros-${ROS_DISTRO}-image-transport" \
    "ros-${ROS_DISTRO}-image-publisher" \
    "ros-${ROS_DISTRO}-camera-info-manager" \
    "ros-${ROS_DISTRO}-diagnostic-updater" \
    "ros-${ROS_DISTRO}-diagnostic-msgs" \
    "ros-${ROS_DISTRO}-statistics-msgs" \
    "ros-${ROS_DISTRO}-backward-ros" \
    "ros-${ROS_DISTRO}-xacro" \
    "ros-${ROS_DISTRO}-joint-state-publisher" \
    "ros-${ROS_DISTRO}-joint-state-publisher-gui" \
    "ros-${ROS_DISTRO}-rviz2" \
    "ros-${ROS_DISTRO}-gazebo-ros-pkgs"
fi

echo "========== [4] Install Python-side AI helpers =========="
python3 -m pip install --user --upgrade pip wheel setuptools
python3 -m pip install --user \
  dashscope==1.25.5 \
  dify-client==0.1.10 \
  openai==1.95.1 \
  PyAudio==0.2.14 \
  pyserial==3.5 \
  PyYAML==6.0.2 \
  requests

echo "========== [5] Configure rosdep =========="
source "$ROS_SETUP"
sudo rosdep init 2>/dev/null || true
rosdep update || true

echo "========== [6] Restore source trees =========="
mkdir -p "$MIRCO_WS/src" "$YAHBOOM_WS/src" "$M3_WS/src"
rsync -a "$SNAPSHOT_DIR/mircoROS_agent_src/" "$MIRCO_WS/src/"
rsync -a "$SNAPSHOT_DIR/yahboomcar_ws_src/" "$YAHBOOM_WS/src/"
rsync -a "$SNAPSHOT_DIR/M3Pro_ws_src/" "$M3_WS/src/"

echo "========== [7] Reapply mapper overlay =========="
rm -rf "$M3_WS/src/m3pro_auto_mapper"
rsync -a "$OVERLAY_DIR/" "$M3_WS/src/m3pro_auto_mapper/"

echo "========== [8] Apply desktop portability patches =========="
export M3_WS YAHBOOM_WS HOME
python3 - <<'PY'
import os
from pathlib import Path

home = Path.home()
m3_ws = Path(os.environ["M3_WS"])
replacements = {
    m3_ws / "src/largemodel/largemodel/__init__.py": [
        ("/home/jetson/M3Pro_ws/src/largemodel/utils", str(home / "M3Pro_ws/src/largemodel/utils")),
    ],
    m3_ws / "src/largemodel/config/large_model_interface.yaml": [
        ("/home/jetson/MODELS", str(home / "MODELS")),
    ],
}

for path, mapping in replacements.items():
    if not path.exists():
        continue
    text = path.read_text()
    for old, new in mapping:
        text = text.replace(old, new)
    path.write_text(text)
PY

echo "========== [9] Install Orbbec udev rules if available =========="
if [ -f "$YAHBOOM_WS/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh" ]; then
  sudo bash "$YAHBOOM_WS/src/OrbbecSDK_ROS2/orbbec_camera/scripts/install_udev_rules.sh"
  sudo udevadm control --reload-rules || true
  sudo udevadm trigger || true
fi

echo "========== [10] Run rosdep on all three workspaces =========="
rosdep install --from-paths "$MIRCO_WS/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO" || true
rosdep install --from-paths "$YAHBOOM_WS/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO" || true
rosdep install --from-paths "$M3_WS/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO" || true

echo "========== [11] Create desktop helper scripts =========="
cat > "$HOME/smart3d_desktop_env.sh" <<EOF
#!/usr/bin/env bash
source "$ROS_SETUP"
[ -f "$MIRCO_WS/install/setup.bash" ] && source "$MIRCO_WS/install/setup.bash"
[ -f "$YAHBOOM_WS/install/setup.bash" ] && source "$YAHBOOM_WS/install/setup.bash"
[ -f "$M3_WS/install/setup.bash" ] && source "$M3_WS/install/setup.bash"
export ROS_DOMAIN_ID=\${ROS_DOMAIN_ID:-30}
EOF
chmod +x "$HOME/smart3d_desktop_env.sh"

cat > "$HOME/start_agent_desktop_clone.sh" <<'EOF'
#!/usr/bin/env bash
set -euo pipefail

SERIAL_DEV="${MICROROS_SERIAL_DEV:-/dev/myserial}"
ENV_SCRIPT="$HOME/smart3d_desktop_env.sh"

if [ ! -f "$ENV_SCRIPT" ]; then
  echo "Missing $ENV_SCRIPT"
  exit 1
fi

if [ ! -e "$SERIAL_DEV" ]; then
  echo "Warning: $SERIAL_DEV does not exist yet."
  echo "Set MICROROS_SERIAL_DEV=/dev/ttyUSB0 or create your usual udev alias first."
fi

if command -v gnome-terminal >/dev/null 2>&1; then
  gnome-terminal --title=mircoROS_Agent -- bash -lc "source \"$ENV_SCRIPT\" && ros2 run micro_ros_agent micro_ros_agent serial --dev \"$SERIAL_DEV\" -b 2000000"
else
  source "$ENV_SCRIPT"
  ros2 run micro_ros_agent micro_ros_agent serial --dev "$SERIAL_DEV" -b 2000000
fi
EOF
chmod +x "$HOME/start_agent_desktop_clone.sh"

if [ "$SKIP_BUILD" != "1" ]; then
  echo "========== [12] Build the three workspaces =========="
  cd "$MIRCO_WS"
  colcon build --symlink-install

  cd "$YAHBOOM_WS"
  source "$ROS_SETUP"
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

  cd "$M3_WS"
  source "$ROS_SETUP"
  source "$YAHBOOM_WS/install/setup.bash"
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
else
  echo "========== [12] Skip build (SKIP_BUILD=1) =========="
fi

echo "========== [13] Write remaining reference report =========="
rg -n '/home/jetson|/dev/myserial|/dev/mic|MODELS/' \
  "$MIRCO_WS/src" "$YAHBOOM_WS/src" "$M3_WS/src" > "$REMAINING_REF_REPORT" || true

cat <<EOF

Desktop restore finished.

Helper env script:
  $HOME/smart3d_desktop_env.sh

micro-ROS helper:
  $HOME/start_agent_desktop_clone.sh

Remaining machine-specific refs report:
  $REMAINING_REF_REPORT

Suggested next commands:
  source "$HOME/smart3d_desktop_env.sh"
  ros2 pkg list | rg 'm3pro_auto_mapper|slam_mapping|M3Pro_navigation|orbbec_camera|multi_brains|largemodel'
  MICROROS_SERIAL_DEV=/dev/ttyUSB0 sh "$HOME/start_agent_desktop_clone.sh"
  ros2 launch orbbec_camera dabai_dcw2.launch.py
  ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py use_rviz:=true

Notes:
  - This reproduces the source tree and most ROS/runtime dependencies on Ubuntu desktop.
  - For full hardware parity you still need a real serial device, lidar on /scan, Orbbec camera, and any required API/model files under \$HOME/MODELS.
  - yahboom_yolov8 is still intentionally excluded from the repo snapshot.
EOF
