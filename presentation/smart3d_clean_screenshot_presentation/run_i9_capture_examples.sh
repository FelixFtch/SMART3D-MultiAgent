#!/usr/bin/env bash
set -euo pipefail

export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/run/user/1000/gdm/Xauthority}"

source /opt/ros/humble/setup.bash
source "${HOME}/.cargo/env"
cd /home/felix/SMART3D

case "${1:-help}" in
  install)
    mkdir -p /home/felix/smart_rviz_bridge
    install -m 0644 presentation/smart3d_clean_screenshot_presentation/rviz/*.rviz /home/felix/smart_rviz_bridge/
    install -m 0755 presentation/smart3d_clean_screenshot_presentation/bridges/*.py /home/felix/smart_rviz_bridge/
    python3 -m py_compile \
      /home/felix/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py \
      /home/felix/smart_rviz_bridge/smart_multi_agent_3d_paper_capture_bridge.py
    ;;
  stop)
    pkill -f "smart_multi_agent.*paper_capture_bridge.py" 2>/dev/null || true
    pkill -f "rviz2.*paper_capture" 2>/dev/null || true
    ;;
  rviz2d)
    rviz2 -d /home/felix/smart_rviz_bridge/smart_shared_unknown_v2_paper_capture.rviz
    ;;
  rviz3d)
    rviz2 -d /home/felix/smart_rviz_bridge/smart_rviz_3d_multiagent_paper_capture.rviz
    ;;
  office2d)
    cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
      --scenario office \
      --num-agents 4 \
      --agent-mode cross \
      --include-tree \
      -n 0 \
      --playback-speed "${PLAYBACK_SPEED:-0.35}" \
      --sensor-range 10 \
      --rrt-iterations 30000 \
      --max-time 120 \
      | python3 /home/felix/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py
    ;;
  forest3d)
    cargo run --release --example smart_multi_agent_3d -- \
      --scenario forest \
      --num-agents 4 \
      --agent-mode cross3d \
      --include-tree \
      -n 0 \
      --playback-speed "${PLAYBACK_SPEED:-0.35}" \
      --sensor-range 10 \
      --rrt-iterations 30000 \
      --max-time 80 \
      | python3 /home/felix/smart_rviz_bridge/smart_multi_agent_3d_paper_capture_bridge.py
    ;;
  *)
    cat <<'EOF'
Usage:
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh install
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh stop
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz2d
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh office2d
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz3d
  ./presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh forest3d

Set PLAYBACK_SPEED=1.0 or PLAYBACK_SPEED=0.35 before running a scene if desired.
EOF
    ;;
esac
