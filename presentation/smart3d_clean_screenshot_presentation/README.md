# SMART3D Clean Screenshot Presentation

This package preserves the paper-screenshot visualization style validated on the i9 on 2026-04-24.

## Visual Contract

- RViz background is white.
- The 2D map keeps exploration semantics:
  - unknown / not scanned yet: light gray
  - scanned free space: white
  - scanned walls: black
  - faint gray full-scene wall context remains visible
- Start and goal markers are solid agent colors.
- Future paths are thick, solid colored lines.
- SMART tree edges remain colored and visible.
- Floating title / status HUD text is suppressed.
- 2D uses top-down orthographic view.
- 3D uses the clean axon-style RViz view.

## Files

- `rviz/smart_shared_unknown_v2_paper_capture.rviz`
- `rviz/smart_rviz_3d_multiagent_paper_capture.rviz`
- `bridges/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py`
- `bridges/smart_multi_agent_3d_paper_capture_bridge.py`

## Install On i9

Run this from the repo root on the i9:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /home/felix/smart_rviz_bridge
install -m 0644 presentation/smart3d_clean_screenshot_presentation/rviz/*.rviz /home/felix/smart_rviz_bridge/
install -m 0755 presentation/smart3d_clean_screenshot_presentation/bridges/*.py /home/felix/smart_rviz_bridge/
python3 -m py_compile /home/felix/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py \
  /home/felix/smart_rviz_bridge/smart_multi_agent_3d_paper_capture_bridge.py
```

## i9 Setup

Use these in every i9 terminal before running RViz or a scene:

```bash
source /opt/ros/humble/setup.bash
source ~/.cargo/env
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
cd /home/felix/SMART3D
```

Stop old screenshot sessions:

```bash
pkill -f "smart_multi_agent.*paper_capture_bridge.py" 2>/dev/null || true
pkill -f "rviz2.*paper_capture" 2>/dev/null || true
```

## 2D RViz

```bash
rviz2 -d /home/felix/smart_rviz_bridge/smart_shared_unknown_v2_paper_capture.rviz
```

## 2D Example

```bash
cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
  --scenario office \
  --num-agents 4 \
  --agent-mode cross \
  --include-tree \
  -n 0 \
  --playback-speed 0.35 \
  --sensor-range 10 \
  --rrt-iterations 30000 \
  --max-time 120 \
  | python3 /home/felix/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py
```

## 3D RViz

```bash
rviz2 -d /home/felix/smart_rviz_bridge/smart_rviz_3d_multiagent_paper_capture.rviz
```

## 3D Final-Protocol Example

```bash
cargo run --release --example smart_multi_agent_3d -- \
  --scenario forest \
  --num-agents 4 \
  --agent-mode shared_unknown_corners \
  --deployment-geometry outer_quarters \
  --deployment-local-range-scale 0.45 \
  --include-tree \
  -n 0 \
  --playback-speed 0.35 \
  --sensor-range 10 \
  --rrt-iterations 30000 \
  --max-time 80 \
  | python3 /home/felix/smart_rviz_bridge/smart_multi_agent_3d_paper_capture_bridge.py
```

## Notes

- For screenshots, use slower playback such as `--playback-speed 0.35` so the best frame is easy to catch.
- Do not use the old non-paper bridge if the image shows a dark background or large yellow/white HUD text.
- The 2D `building + cross` setup has known initialization and reachability caveats; use it for visual reference only unless the benchmark success predicate is also checked as 4 initialized / 4 goal.
