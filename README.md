# SMART-3D Multi-Agent Navigation

Multi-agent path planning in unknown environments with incremental tree repair and cooperative wall discovery.

**Author:** Felix Fang (Tianchonghui Fang) · [LinksLab, University of Connecticut](https://github.com/LinksLabUConn)

Built on [SMART-3D](https://arxiv.org/abs/2509.16812) by Zongyuan Shen.

## Key Results (50,000+ benchmark trials)

| Metric | Value |
|--------|-------|
| Incremental repair vs full rebuild | **22.4x faster** (Megacampus) |
| Shared vs independent wall discovery | **1.04x–2.09x faster** (all scenarios) |
| Per-agent completion time spread | **4.2x reduction** (Megacampus) |
| Success rate (4 agents, mega maps) | **100%** (1000 trials each) |

## What's New (vs Original SMART-3D)

1. **Incremental repair for permanent walls** — the original handles dynamic obstacles only
2. **Lazy + Eager adaptive architecture** — skip tree scan when path unaffected, full repair when path blocked
3. **Multi-agent coordination** — Global King priority, flee, chain push in continuous space
4. **Shared wall discovery** — Agent A's LiDAR discovers a wall, all agents' trees updated
5. **Spatial index v2** — zero-allocation grid-bucket, -26% CPU on mega maps

## Repository Structure
```
examples/
  smart_multi_agent.rs      — Multi-agent simulation (--no-wall-sharing flag)
  smart_rviz_json.rs        — Single-agent simulation
  smart_planner_server.rs   — Rust planner for real robot (JSON stdin/stdout)
src/obstacles/
  spatial_checker.rs        — Spatial index v2
deploy/
  i9_planner.py             — Central ROS2 planner node
  pi5_client.py             — Thin client (TF to pose, path to pursuit)
  cmd_vel_driver.py         — Motor RS485 driver
benchmarks/
  bench_sharing_ablation.py — Shared vs independent (12,000 trials)
  bench_single_agent.py     — Single-agent baseline (6,000 trials)
  bench_e3_vs_senior.py     — Full-tree vs path-only repair
docs/
  SMART3D_RESEARCH_DASHBOARD.html  — Interactive benchmark dashboard
  SMART3D_EXPLAINER.html           — Technical explainer (EN)
  SMART3D_EXPLAINER_CN.html        — Technical explainer (CN)
  HANDOFF.md                       — Deployment handoff guide
  FUTURE_OPTIMIZATION_SPEC.md      — Deferred Eager + local wall query
```

## Run Simulation
```bash
# Shared wall discovery (default)
cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  --playback-speed 3 --sensor-range 15 --rrt-iterations 80000 --lsr-max 80

# Independent (ablation)
cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  --playback-speed 3 --sensor-range 15 --rrt-iterations 80000 --lsr-max 80 \
  --no-wall-sharing
```

## 8 Simulation Scenarios

building (32m) · office (32m) · warehouse (32m) · bottleneck (32m) · mega_hospital (96m) · mega_airport (160m) · megacampus (128m) · mega_university (192m)

## License

See [LinksLabUConn/SMART3D](https://github.com/LinksLabUConn/SMART3D) for the base library license.
