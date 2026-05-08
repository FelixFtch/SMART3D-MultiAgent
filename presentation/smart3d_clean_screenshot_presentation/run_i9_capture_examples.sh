#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

source /opt/ros/humble/setup.bash
source "${HOME}/.cargo/env"

SMART3D_DIR="${SMART3D_DIR:-/home/felix/SMART3D_capture_core_clean}"
BRIDGE_DIR="${BRIDGE_DIR:-/home/felix/smart_rviz_bridge}"
PLAYBACK_SPEED="${PLAYBACK_SPEED:-30.0}"
DETOUR_THRESHOLD="${DETOUR_THRESHOLD:-1.1}"

scene_cmd() {
  local scenario="$1"
  local rrt_iters="$2"
  local max_time="$3"
  local sensor_range="${4:-10}"
  local lsr_max="${5:-80}"
  local detour="${6:-$DETOUR_THRESHOLD}"

  cd "$SMART3D_DIR"
  cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
    --scenario "$scenario" \
    --num-agents 4 \
    --agent-mode cross \
    --num-obstacles 0 \
    --playback-speed "$PLAYBACK_SPEED" \
    --sensor-range "$sensor_range" \
    --rrt-iterations "$rrt_iters" \
    --lsr-max "$lsr_max" \
    --detour-threshold "$detour" \
    --max-time "$max_time" \
    | python3 "$BRIDGE_DIR/smart_multi_agent_shared_unknown_v2_clean_display_bridge.py"
}

case "${1:-help}" in
  install)
    mkdir -p "$BRIDGE_DIR"
    install -m 0644 presentation/smart3d_clean_screenshot_presentation/rviz/*.rviz "$BRIDGE_DIR/"
    install -m 0755 presentation/smart3d_clean_screenshot_presentation/bridges/*.py "$BRIDGE_DIR/"
    python3 -m py_compile "$BRIDGE_DIR"/*.py
    ;;
  stop)
    pkill -f "smart_multi_agent_shared_unknown_v2.*capture_bridge.py" 2>/dev/null || true
    pkill -f "smart_multi_agent_shared_unknown_v2.*clean_display_bridge.py" 2>/dev/null || true
    pkill -f "smart_multi_agent_3d.*paper_capture_bridge.py" 2>/dev/null || true
    pkill -f "rviz2.*smart_shared_unknown_v2_paper_capture" 2>/dev/null || true
    pkill -f "rviz2.*smart_rviz_3d_multiagent_paper_capture" 2>/dev/null || true
    ;;
  rviz2d)
    rviz2 -d "$BRIDGE_DIR/smart_shared_unknown_v2_paper_capture.rviz"
    ;;
  rviz3d)
    rviz2 -d "$BRIDGE_DIR/smart_rviz_3d_multiagent_paper_capture.rviz"
    ;;
  building2d)
    scene_cmd building 5000 180 10 80 "$DETOUR_THRESHOLD"
    ;;
  office2d)
    scene_cmd office 5000 180 10 80 "$DETOUR_THRESHOLD"
    ;;
  warehouse2d)
    scene_cmd warehouse 50000 240 10 80 "$DETOUR_THRESHOLD"
    ;;
  hospital2d)
    scene_cmd hospital 30000 420 10 80 "$DETOUR_THRESHOLD"
    ;;
  airport2d)
    scene_cmd airport 50000 520 10 80 "$DETOUR_THRESHOLD"
    ;;
  campus2d)
    scene_cmd campus 80000 520 10 80 "$DETOUR_THRESHOLD"
    ;;
  university2d)
    scene_cmd university 160000 720 15 80 "$DETOUR_THRESHOLD"
    ;;
  bottleneck2d)
    cd "$SMART3D_DIR"
    cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
      --scenario bottleneck \
      --num-agents 4 \
      --agent-mode cross \
      --num-obstacles 0 \
      --playback-speed "$PLAYBACK_SPEED" \
      --sensor-range 10 \
      --rrt-iterations 30000 \
      --lsr-max 80 \
      --detour-threshold 0 \
      --max-time 240 \
      | python3 "$BRIDGE_DIR/smart_multi_agent_shared_unknown_v2_clean_display_bridge.py"
    ;;
  forest3d)
    cd "$SMART3D_DIR"
    cargo run --release --example run_3d_scenarios -- --output-dir "${SMART3D_3D_OUTPUT_DIR:-output/3d_scenarios}"
    ;;
  *)
    cat <<'EOF'
Usage:
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh install
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh stop
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh building2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh office2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh warehouse2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh hospital2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh airport2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh campus2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh university2d
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh bottleneck2d

Environment:
  SMART3D_DIR=/home/felix/SMART3D_capture_core_clean
  BRIDGE_DIR=/home/felix/smart_rviz_bridge
  PLAYBACK_SPEED=30.0
  DETOUR_THRESHOLD=1.1

Notes:
  Final 2D capture defaults are no people, cross goals, tree off, clean display.
  DETOUR_THRESHOLD=1.1 is strict and shortest-looking; 1.5 is looser; 0 disables it.
EOF
    ;;
esac
