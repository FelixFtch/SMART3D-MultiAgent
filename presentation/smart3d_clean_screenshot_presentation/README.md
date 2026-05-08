# SMART3D Paper RViz Capture

This directory is the 2026-05-08 final RViz capture chain for the paper-facing
2D SMART3D/PECMAN scenes. It keeps the original simulation behavior and only
changes the RViz presentation layer.

## Final Visual Contract

- 2D view only for the seven paper floorplans.
- SMART tree display is off.
- HUD text, start labels, goal labels, and agent-name labels are off.
- Start-point circles are off.
- Goal markers are filled five-point stars, colored per agent.
- Agents keep colored body markers plus heading direction.
- Only future paths are shown; past traveled paths are not shown.
- True walls are black.
- Scanned free space is white.
- Unknown/unscanned space is gray.
- Faint gray wall context may remain visible behind the explored map.
- Default capture mode has no pedestrians.
- Pedestrians can be enabled for demos, but they are not part of the final
  clean no-people paper screenshot path.

## Important Files

- `rviz/smart_shared_unknown_v2_paper_capture.rviz`
- `bridges/smart_multi_agent_shared_unknown_v2_clean_display_bridge.py`
- `bridges/smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py`
- `bridges/smart_multi_agent_3d_paper_capture_bridge.py`
- `run_i9_capture_examples.sh`

`smart_multi_agent_shared_unknown_v2_paper_capture_bridge.py` is kept as a
compatibility filename. For the final 2026-05-08 look, prefer the explicit
`smart_multi_agent_shared_unknown_v2_clean_display_bridge.py` name.

## Install On i9

Run from the repo root on the i9:

```bash
source /opt/ros/humble/setup.bash
mkdir -p /home/felix/smart_rviz_bridge
install -m 0644 presentation/smart3d_clean_screenshot_presentation/rviz/*.rviz /home/felix/smart_rviz_bridge/
install -m 0755 presentation/smart3d_clean_screenshot_presentation/bridges/*.py /home/felix/smart_rviz_bridge/
python3 -m py_compile /home/felix/smart_rviz_bridge/*.py
```

The 2026-05-08 validated i9 runtime directory is:

```bash
/home/felix/SMART3D_capture_core_clean
```

The helper script defaults to that directory. Override it only if you know a
different checkout has the same examples:

```bash
SMART3D_DIR=/home/felix/SMART3D_capture_core_clean \
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh install
```

## Stop Old Runs

```bash
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh stop
```

This matters because stale RViz bridges can keep publishing old markers and make
new scenes look like they have wall/path ghosts.

## Open RViz

In one i9 terminal:

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority
rviz2 -d /home/felix/smart_rviz_bridge/smart_shared_unknown_v2_paper_capture.rviz
```

Or use:

```bash
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz2d
```

## Final 2D Scene Defaults

Use these defaults unless a figure needs a deliberate variant:

- `--num-obstacles 0`
- `--agent-mode cross`
- no `--include-tree`
- `--detour-threshold 1.1`
- `--playback-speed 30.0` or higher for fast review
- `--lsr-max 80` on large maps

Detour threshold is important:

- `1.1`: strict, can rebuild more often, but keeps paths closer to shortest.
- `1.5`: fewer rebuilds, but visibly allows more detours.
- `0`: disabled, useful for bottleneck/debug scenes, not the final paper path.

## Seven Paper Scenes

The helper script accepts these scene names:

```bash
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh building2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh office2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh warehouse2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh hospital2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh airport2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh campus2d
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh university2d
```

Manual command shape:

```bash
cd /home/felix/SMART3D_capture_core_clean
source /opt/ros/humble/setup.bash
source ~/.cargo/env
export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

cargo run --release --example smart_multi_agent_shared_unknown_v2 -- \
  --scenario hospital \
  --num-agents 4 \
  --agent-mode cross \
  --num-obstacles 0 \
  --playback-speed 30.0 \
  --sensor-range 10 \
  --rrt-iterations 30000 \
  --lsr-max 80 \
  --detour-threshold 1.1 \
  --max-time 420 \
  | python3 /home/felix/smart_rviz_bridge/smart_multi_agent_shared_unknown_v2_clean_display_bridge.py
```

## Bottleneck

Use no pedestrians and detour off for the bottleneck stress display:

```bash
PLAYBACK_SPEED=30 presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh bottleneck2d
```

## Benchmark Reruns

The current benchmark runner chain lives in:

- `examples/run_2d_scenarios.rs`
- `examples/run_3d_scenarios.rs`
- `examples/scenarios/`

The 2026-05-08 i9 rerun output paths are documented in `benchmarks/README.md`.
These runners are for numeric regression data. They are not the same as the
paper RViz capture path above.

## Common Pitfalls

- Do not add `--include-tree` for the final 2D screenshots.
- Do not use `shared_unknown_corners`; final screenshots use `--agent-mode cross`.
- Do not leave an old bridge running; it can leave marker ghosts.
- Do not use the old dark RViz config if the map should be white/gray/black.
- Do not judge speed from RViz rendering alone; RViz can be the bottleneck even
  when the Rust simulation has already advanced.
- Do not call Swift "Senior" in new paper-facing text. The old code/history name
  may still appear in older artifacts, but the paper-facing name is Swift.
