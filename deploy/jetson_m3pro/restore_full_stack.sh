#!/usr/bin/env bash
set -eo pipefail

REPO_DIR="${1:-$HOME/SMART3D-MultiAgent}"
SNAPSHOT_DIR="$REPO_DIR/field/jetson_m3pro_original_2026-04-12"
OVERLAY_DIR="$REPO_DIR/extras/m3pro_auto_mapper21_overlay"

MIRCO_WS="$HOME/mircoROS_agent"
YAHBOOM_WS="$HOME/yahboomcar_ws"
M3_WS="$HOME/M3Pro_ws"

BACKUP_ROOT="$HOME/jetson_full_restore_backup_$(date +%Y%m%d_%H%M%S)"

need_dir() {
  if [ ! -d "$1" ]; then
    echo "Missing directory: $1"
    exit 1
  fi
}

echo "========== [1] Validate repo snapshot =========="
need_dir "$SNAPSHOT_DIR"
need_dir "$SNAPSHOT_DIR/mircoROS_agent_src"
need_dir "$SNAPSHOT_DIR/yahboomcar_ws_src"
need_dir "$SNAPSHOT_DIR/M3Pro_ws_src"
need_dir "$OVERLAY_DIR"

echo "========== [2] Prepare backup =========="
mkdir -p "$BACKUP_ROOT"
cp -a "$HOME/.bashrc" "$BACKUP_ROOT/.bashrc" 2>/dev/null || true
cp -a "$HOME/start_agent.sh" "$BACKUP_ROOT/start_agent.sh" 2>/dev/null || true

if [ -d "$M3_WS/src/m3pro_auto_mapper" ]; then
  mkdir -p "$BACKUP_ROOT/M3Pro_ws/src"
  cp -a "$M3_WS/src/m3pro_auto_mapper" "$BACKUP_ROOT/M3Pro_ws/src/"
fi

echo "Backup root: $BACKUP_ROOT"

echo "========== [3] Restore source trees =========="
mkdir -p "$MIRCO_WS/src" "$YAHBOOM_WS/src" "$M3_WS/src"

rsync -a "$SNAPSHOT_DIR/mircoROS_agent_src/" "$MIRCO_WS/src/"
rsync -a "$SNAPSHOT_DIR/yahboomcar_ws_src/" "$YAHBOOM_WS/src/"
rsync -a "$SNAPSHOT_DIR/M3Pro_ws_src/" "$M3_WS/src/"

echo "========== [4] Reapply mapper overlay =========="
rm -rf "$M3_WS/src/m3pro_auto_mapper"
rsync -a "$OVERLAY_DIR/" "$M3_WS/src/m3pro_auto_mapper/"

echo "========== [5] Restore startup helpers =========="
install -m 755 "$SNAPSHOT_DIR/start_agent.sh" "$HOME/start_agent.sh"

if [ -f "$SNAPSHOT_DIR/home_support/set.sh" ]; then
  install -m 755 "$SNAPSHOT_DIR/home_support/set.sh" "$HOME/set.sh"
fi

echo "========== [6] Next build steps =========="
cat <<NEXT
cd "$MIRCO_WS"
colcon build --symlink-install

cd "$YAHBOOM_WS"
colcon build --symlink-install

cd "$M3_WS"
colcon build --symlink-install

source /opt/ros/humble/setup.bash
source "$YAHBOOM_WS/install/setup.bash"
source "$M3_WS/install/setup.bash"
export ROS_DOMAIN_ID=30

sh "$HOME/start_agent.sh"
ros2 launch m3pro_auto_mapper explore_stack_with_finisher.launch.py
NEXT

echo
echo "Restore files copied successfully."
echo "Note: yahboom_yolov8 is intentionally not restored by this script."
