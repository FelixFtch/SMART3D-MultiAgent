# SMART-3D Multi-Agent Navigation: Complete System Reference

> **For any AI assistant, new computer setup, or paper writing.**
> Felix Fang built the entire system solo at UConn LinksLab.
> Based on SMART-3D by Zongyuan Shen (arXiv:2509.16812).

---

## Start Here: Repository Map

This repository is the paper-facing SMART-3D / PECMAN simulation, benchmark,
and RViz capture package. It is now a complete Rust crate: a fresh clone should
contain `Cargo.toml`, `Cargo.lock`, `src/`, `examples/`, benchmarks, and the
RViz bridge files needed for paper captures.

It is not the real-car deployment source of truth.

Use this map first:

| Goal | Where to start |
|------|----------------|
| Build the Rust crate from a fresh clone | `Cargo.toml`, `src/`, `examples/` |
| Run the core 2D SMART-3D multi-agent algorithm | `examples/smart_multi_agent.rs` |
| Run the accepted 2026-05-08 paper/RViz capture chain | `examples/smart_multi_agent_shared_unknown_v2.rs` and `presentation/smart3d_clean_screenshot_presentation/` |
| Run benchmark scripts and inspect raw results | `benchmarks/` |
| Inspect 1,000-trial JSON data | `benchmarks/data/` |
| Inspect 2026-04-23 2D/3D paper-map exports | `benchmarks/data/paper_export_2026-04-23/` |
| Inspect 2026-05-08 i9 benchmark rerun aggregates | `benchmarks/data/rerun_2026-05-08/` |
| Build paper figures / screenshots | `presentation/smart3d_clean_screenshot_presentation/` |
| Read the paper source | `paper/main.tex` |
| Restore physical robot code | use the separate real-car repo `smart3d-multiagent-car-code` |

Status summary:

- Complete Rust crate: this checkout includes `Cargo.toml`, `Cargo.lock`,
  `src/`, and runnable `examples/`.
- Final paper RViz capture: 2D floorplan scenes through
  `examples/smart_multi_agent_shared_unknown_v2.rs`.
- Numeric regressions: 2D/3D aggregate runners and JSON data live under
  `benchmarks/` and `benchmarks/data/`.
- 3D note: aggregate benchmark data is included; archived 3D RViz bridge files
  are included, but the live 3D JSON producer is not part of the 2026-05-08
  final capture lane.
- Real-car deployment: use the separate `smart3d-multiagent-car-code` repo as
  the current physical robot source of truth. `deploy/`, `field/`, and
  `jetsoncar/` here are restore/provenance snapshots unless explicitly promoted.

Current naming convention:

- `SMART-3D`: paper-facing algorithm name.
- `PECMAN`: the multi-agent shared-perception system built on SMART-3D.
- `Lazy Eager` / `LE`: final recommended repair strategy in the paper.
- `Swift`: path-local/path-only repair comparison baseline, formerly named
  `Senior` in older code/comments.
- `shared_unknown_v2`: paper/RViz capture variant for clean diagonal-corner
  visualizations.

For the accepted 2026-05-08 clean RViz look, do not use tree display, text HUD,
or old corner-deploy modes. See:

```text
presentation/smart3d_clean_screenshot_presentation/README.md
```

### Accepted 2026-05-08 RViz Capture Lane

The final paper-facing 2D visual lane is:

```text
shared unknown + king + cross diagonal goals + no pedestrians + detour=1.5 + clean paper RViz
```

This is the version that was visually accepted on the i9. It is intentionally
not the old tree-heavy debug view and not the stricter `detour=1.1` lane.

- Use `examples/smart_multi_agent_shared_unknown_v2.rs`.
- Use `presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh`.
- Keep `--agent-mode cross`; do not use `shared_unknown_corners`.
- Keep `--num-obstacles 0` for clean paper screenshots.
- Keep tree display off and use the clean display bridge.
- Default `DETOUR_THRESHOLD` is now `1.5`.
- `DETOUR_THRESHOLD=1.1` is allowed only as a strict experiment. It often
  rebuilds too aggressively on hospital/airport/campus/university.
- Bottleneck is a separate stress-test scene. It is useful for King/flee
  debugging, but it is not the recommended clean screenshot lane.

On the i9, use this two-terminal shape from a full clone of this repository:

```bash
cd /path/to/SMART3D-MultiAgent
source ~/.cargo/env
set +u; source /opt/ros/humble/setup.bash; set -u
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh install
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz2d
```

Then run scenes with speed as high as RViz can tolerate:

```bash
SMART3D_DIR="$PWD" PLAYBACK_SPEED=10000 DETOUR_THRESHOLD=1.5 \
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh campus2d
```

`SMART3D_DIR="$PWD"` matters when running from a fresh/full clone: it makes the
helper build the code in this checkout rather than an older slim runtime copy.

The accepted `building` map has an upper-left doorway. If it appears sealed,
the wrong geometry is being used; the v2 scenario should match the older
`smart_multi_agent.rs` building doorway split.

### Fresh Clone Verification

On a machine with Rust installed:

```bash
git clone https://github.com/FelixFtch/SMART3D-MultiAgent.git
cd SMART3D-MultiAgent
cargo check --example smart_multi_agent_shared_unknown_v2 \
  --example run_2d_scenarios \
  --example run_3d_scenarios
```

Validation actually performed on the i9 from a fresh GitHub clone at
`ba5e7191d35fe949f723aee01c6098f4e61b4b54`:

- `cargo check` for the final 2D RViz example and both 2D/3D aggregate runners.
- `cargo build --release` for `smart_multi_agent_shared_unknown_v2`,
  `run_2d_scenarios`, and `run_3d_scenarios`.
- Short startup smoke for `building`, `office`, `warehouse`, `hospital`,
  `airport`, `campus`, `university`, and `bottleneck`.
- Full `building` no-people diagonal run reached all active agents.
- Full `run_3d_scenarios --output-dir ...` completed and wrote
  `3d_scenarios.json`.

On the i9, install the RViz bridge files after cloning:

```bash
source /opt/ros/humble/setup.bash
source ~/.cargo/env
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh install
```

Then open RViz and run a clean 2D paper scene:

```bash
presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh rviz2d
```

```bash
PLAYBACK_SPEED=30 \
  presentation/smart3d_clean_screenshot_presentation/run_i9_capture_examples.sh hospital2d
```

---

## 0. Quick Start: Available Modes and Switches

The main 2D simulation has two interactive examples and several operational
modes controlled by CLI flags:

| Mode | Binary | Flag | What it does |
|------|--------|------|-------------|
| **Lazy Eager / LE (default)** | `smart_multi_agent` | (none) | Shared walls + Lazy+Eager full-tree repair. **This is the best mode.** |
| **Swift** | `smart_multi_agent` | `--senior-mode` | Path-only/path-local repair baseline. The CLI flag keeps the old internal name for compatibility. |
| **No Wall Sharing** | `smart_multi_agent` | `--no-wall-sharing` | Each agent only knows its own LiDAR discoveries. Agents still avoid each other. |
| **Deferred Eager** | `smart_multi_agent_deferred` | `--local-horizon 30` | Only trigger Eager for walls within 30m. With `--local-horizon 0` = same as Lazy Eager. |
| **Single Agent** | `smart_rviz_json` | (none) | One robot, no multi-agent coordination. |

### Example commands (all modes)

```bash
cd ~/SMART3D

# ===== LAZY EAGER DEFAULT (best mode) =====
cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# ===== SWIFT MODE (comparison baseline; old flag name is --senior-mode) =====
cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 --senior-mode \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# ===== NO WALL SHARING (ablation) =====
cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 --no-wall-sharing \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# ===== DEFERRED EAGER (experimental, 30m horizon) =====
cargo run --release --example smart_multi_agent_deferred -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 --local-horizon 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# ===== SINGLE AGENT =====
cargo run --release --example smart_rviz_json -- \
  --scenario megacampus -n 0 -o 1.5 --sensor-range 15 \
  --rrt-iterations 80000 --playback-speed 3 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py
```

---

## 1. Architecture: How the Algorithm Works

### 1.1 The Tree

The robot plans its path using RRT* (Rapidly-exploring Random Tree star). From the start position, the algorithm randomly samples 15,000 points in the map, connecting each to its nearest neighbor in the existing tree. The result is a tree of ~15,000 nodes with one root (the start) and branches reaching toward the goal. The path from root to goal is found by following parent pointers backward from the goal node. Building this tree takes ~40ms on an i9 processor.

### 1.2 The Problem: Unknown Environment

The original SMART-3D paper assumes the robot already knows the complete map. In real deployment, the robot enters an unknown building and discovers walls one by one through its LiDAR sensor. Each newly discovered wall may invalidate edges in the tree (edges that pass through the wall). The path may now go straight through a wall.

### 1.3 Five Approaches Tried (Evolution History)

**Stage 1: Full Rebuild (baseline)**
Every time a new wall is discovered, throw away the entire 15,000-node tree and build a new one from scratch. Cost: ~40ms per wall. In megacampus (128m map, ~300 walls), this means ~300 rebuilds per run, totaling ~12 seconds of freeze time. The random rebuilds also cause quality fluctuations.

Result: Megacampus 76.3s median, 139.6 rebuilds/run, 96.6% success rate.

**Stage 2: Eager Incremental Repair (`update_static_obstacles()`)**
Instead of rebuilding, reuse the SMART paper's prune-reconnect mechanism for permanent walls:
1. Scan all 15,000 edges in the tree
2. Find ~20 edges that cross the new wall
3. Cut those edges (tree splits into main tree + disconnected fragments)
4. Search for "hot nodes" within LSR radius that can reconnect fragments to main tree
5. Update path costs and replan

This is the core extension of the original paper. The paper's `update()` handles dynamic obstacles (people) that appear and disappear. `update_static_obstacles()` handles permanent walls that never disappear, requiring different tree management (no restoration of cut edges).

Result: 8.71s (8.8x faster), 2.2 rebuilds, 99.9% success rate.

**Stage 3: Lazy Swap (`set_static_validity_checker()`)**
Observation: ~80% of newly discovered walls don't block the current path. The robot is heading east, and the wall is in the north corridor. Why scan 15,000 edges if the path is fine?

When path is not affected: just swap the collision checker (update the "rule book" so future planning knows about the wall). Cost: ~0ms. Downside: "dirty edges" accumulate in the tree (edges crossing old walls that were never cut).

Result: 3.40s (22.4x faster), 97.4% success rate.

**Stage 4: Hybrid (ABANDONED)**
Light edge cleanup when path unaffected. Result: 3.42s, 98.6%. No speed advantage over Lazy, worse reliability than Eager. Abandoned.

**Stage 5: Lazy + Eager Combo (FINAL ARCHITECTURE)**
The decision rule executed every time a new wall is discovered:
- Path not blocked by any known wall → **Lazy swap** (0ms)
- Path blocked by a wall → **Eager full-tree repair** (~5-22ms)

Eager repair cleans up ALL dirty edges from previous Lazy swaps (because the checker now contains all known walls). So dirty edges never accumulate indefinitely.

This is the default Lazy Eager mode of the current system. Older code and
benchmark files may still call this `E3`.

### 1.4 Failed Optimization Attempts

**Route A: `update_new_static_obstacles()` (ABANDONED)**
Idea: only check new walls during Eager repair, not all old walls. O(N×w) instead of O(N×W).
Failed because: Lazy swap leaves dirty edges from old walls. Route A only checks new walls, so old dirty edges are never cleaned. Path follows dirty edges through walls. ~4 hours debug to find this interaction bug.

**Spatial Index v1 (ABANDONED)**
Idea: HashMap grid-bucket for `is_edge_valid()` to only check walls near each edge.
Failed because: each call allocated `vec![false; 300]` for deduplication. 15,000 calls × heap allocation was slower than linear scan. 300 walls (4.8KB) fit entirely in L1 cache; linear scan has perfect memory prefetch.

**Spatial Index v2 (SUCCESS)**
Zero-allocation design. Allow duplicate wall checks (extra 5ns per duplicate is negligible). Single-cell fast path. Megacampus wall-clock: 66.1s → 48.6s (−26%). Small maps slightly slower due to HashMap overhead (not a problem since small maps are already fast).

### 1.5 Multi-Agent Coordination: King Logic

When multiple robots are in the same map, paths cross. Two robots heading toward each other need a rule for who yields.

**Global King Priority:**
- Every frame, check every pair of robots
- If distance < safety threshold (~2.5-3m), decide who has right of way
- Higher priority number = King. King keeps walking at full speed
- Non-King robot **actively flees** (moves away from King, not just stops)
- Same priority → use random tiebreak value (stable for entire run)

**Chain Push:**
Robot A yields to King B, but A's retreat direction is blocked by Robot C. A pushes C to also move aside. If C has lower priority than A, C also flees. Chain reaction: B pushes A, A pushes C.

**Three movement layers (decoupled):**
1. **RRT* tree path** (global): agent follows waypoint sequence via pure pursuit. Sampling-based.
2. **SMART incremental repair** (reactive): new wall or OHZ prunes edges, tree reconnects, path auto-updates. ~5ms.
3. **King/flee/push** (immediate): direct position vector math per frame. No tree involvement. This is why pushed agents move smoothly (continuous coordinate updates at 30fps), not in discrete grid jumps. After displacement, SMART replans in ~5ms to give the agent a new global path.

**Key property:** King logic operates at the execution layer (direct position adjustment per frame). It never modifies the SMART tree or the planned path. SMART handles long-term planning, King handles immediate collision avoidance. Two completely decoupled layers.

**Per-frame complexity:**
- O(1): path following (pure pursuit per agent)
- O(N): king selection scan
- O(N²): pairwise distance checks (N=4 → 6 pairs, N=8 → 28 pairs)
- O(N × W): LiDAR simulation (each agent ray-casts against W walls)
- O(N × E): shared wall repair (new wall triggers edge scan on all N agents' trees with E edges). This is the dominant cost. Spatial index reduces to O(N × k) where k << E.

### 1.6 Shared Wall Discovery

All agents share a single global `known_walls` list (a Rust Vec, which is a dynamic array in memory). When Agent A's LiDAR discovers a wall, it's added to this list. Next frame, when path_affected is checked for Agent B, the new wall is included.

In simulation: all agents are in the same process, sharing the same memory.
In real robot deployment: i9 central planner aggregates all robots' LiDAR data, extracts walls, and sends the complete wall list to each robot's planner subprocess via JSON.

**Benchmark result (12,000 trials):**

| Scenario | Shared | Independent | Speedup |
|----------|--------|-------------|---------|
| building | 23.2s | 48.4s | 2.09x |
| office | 27.8s | 33.5s | 1.21x |
| warehouse | 24.3s | 26.9s | 1.10x |
| mega_hospital | 92.8s | 108.5s | 1.17x |
| mega_airport | 115.8s | 121.0s | 1.04x |
| megacampus | 116.8s | 137.4s | 1.18x |

### 1.7 Deferred Eager (Experimental)

Hypothesis: when Agent A discovers a wall 80m from Agent B, immediate Eager repair wastes CPU on irrelevant walls. Better to wait until B is within 30m, then repair with better local tree quality.

Implementation: `--local-horizon 30` flag. `path_affected` only checks walls within 30m of the robot. Distant walls get Lazy swap only; Eager triggers when robot approaches within 30m.

**Benchmark result (12,000 trials): no improvement.**

| Scenario | Current | Deferred 30m | Diff |
|----------|---------|-------------|------|
| building | 23.1s | 23.3s | +0.2s (noise) |
| office | 28.1s | 27.9s | -0.2s (noise) |
| warehouse | 24.4s | 24.3s | -0.1s (noise) |
| mega_hospital | 92.0s | 94.9s | +2.9s (3% slower) |
| mega_airport | 115.3s | 119.8s | +4.5s (4% slower) |
| megacampus | 116.6s | 115.6s | -1.0s (noise) |

Conclusion: immediate Eager is better. It cleans dirty edges early, improving subsequent repair success rates. Deferred mode accumulates dirty edges, causing more rebuilds (mega_hospital: 21.6 vs 17.9 rebuilds). The CPU time saved by skipping distant Eager (WallMed 13.3s vs 16.0s) is offset by more expensive rebuilds.

Note: this was tested with cross mode (diagonal paths covering entire map). With random start/goal where shared walls are spatially irrelevant, deferred might show different results. Not yet tested.

### 1.8 Key Parameters

| Parameter | What it controls | Default | Recommended |
|-----------|-----------------|---------|-------------|
| `--lsr-max` | Hot-node search radius for incremental repair | 30 | **80 for mega maps**. Most impactful single parameter. Repair success rate: 70% at LSR=10 → 92% at LSR=80. |
| `--rrt-iterations` | Tree node count | 5000 | 5000 (small), 30000-80000 (mega), 160000 (mega_university) |
| `--sensor-range` | LiDAR range in meters | 10 | 10 (small), 12 (mega), 15 (megacampus) |
| `--shortcut-iters` | Random path shortcut attempts | 150 | 150. ~0% benefit in multi-agent, kept for single-agent. |
| `--detour-threshold` | Path length ratio to trigger rebuild | 1.5 | 1.5. Rarely triggers in multi-agent. |
| `--playback-speed` | Simulation speed | 1.0 | 3 for visualization, 99999 for benchmark |

---

## 2. All 8 Scenarios

| Scenario | Size | Walls | RRT iters | LSR | Sensor | Description |
|----------|------|-------|-----------|-----|--------|-------------|
| building | 32m | 41 | 5000 | 30 | 10m | Office building with rooms and corridors |
| office | 32m | 42 | 5000 | 30 | 10m | Open office with partitions |
| warehouse | 32m | 28 | 5000 | 30 | 10m | Warehouse with shelving aisles |
| bottleneck | 32m | ~20 | 5000 | 30 | 10m | 2m-wide corridor, stress test for king/flee |
| mega_hospital | 96m | ~200 | 30000 | 80 | 12m | Large hospital floor plan |
| mega_airport | 160m | ~250 | 50000 | 80 | 12m | Airport terminal with gates and corridors |
| megacampus | 128m | ~300 | 80000 | 80 | 15m | University campus with multiple buildings |
| mega_university | 192m | ~400 | 160000 | 80 | 15m | Largest scenario, extensive campus |

---

## 3. RViz Visualization Commands

### Setup RViz (do this once per session)

```bash
source /opt/ros/humble/setup.bash
rviz2 &
# In RViz: Fixed Frame = "map"
# Add → By topic → /smart/markers → MarkerArray → OK
```

### Multi-Agent: All scenarios, shared Lazy Eager default

```bash
# building 32m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario building --num-agents 4 --agent-mode cross \
  -n 20 -o 1.5 --playback-speed 3 --sensor-range 10 \
  --rrt-iterations 5000 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# office 32m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario office --num-agents 4 --agent-mode cross \
  -n 20 -o 1.5 --playback-speed 3 --sensor-range 10 \
  --rrt-iterations 5000 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# warehouse 32m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario warehouse --num-agents 4 --agent-mode cross \
  -n 20 -o 1.5 --playback-speed 3 --sensor-range 10 \
  --rrt-iterations 5000 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# bottleneck 32m (8 agents, bottleneck mode)
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario bottleneck --num-agents 8 --agent-mode bottleneck \
  -n 10 -o 1.5 --playback-speed 3 --sensor-range 10 \
  --rrt-iterations 5000 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# mega_hospital 96m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario mega_hospital --num-agents 4 --agent-mode cross \
  -n 200 -o 1.5 --playback-speed 3 --sensor-range 12 \
  --rrt-iterations 30000 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# mega_airport 160m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario mega_airport --num-agents 4 --agent-mode cross \
  -n 200 -o 1.5 --playback-speed 3 --sensor-range 12 \
  --rrt-iterations 50000 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# megacampus 128m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 200 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py

# mega_university 192m
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario mega_university --num-agents 4 --agent-mode cross \
  -n 200 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 160000 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
```

### Multi-Agent: No wall sharing (add `--no-wall-sharing` to any of the above)

```bash
# Example: mega_hospital no-sharing
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario mega_hospital --num-agents 4 --agent-mode cross \
  -n 200 -o 1.5 --playback-speed 3 --sensor-range 12 \
  --rrt-iterations 30000 --lsr-max 80 --no-wall-sharing \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
```

### Multi-Agent: Swift mode (add old compatibility flag `--senior-mode`)

```bash
# Example: mega_hospital Swift
cd ~/SMART3D && cargo run --release --example smart_multi_agent -- \
  --scenario mega_hospital --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 12 \
  --rrt-iterations 30000 --lsr-max 80 --senior-mode \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
```

### Multi-Agent: Deferred Eager (use deferred binary + `--local-horizon`)

```bash
# Example: megacampus deferred 30m
cd ~/SMART3D && cargo run --release --example smart_multi_agent_deferred -- \
  --scenario megacampus --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 15 \
  --rrt-iterations 80000 --lsr-max 80 --local-horizon 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
```

### Single Agent: All scenarios

```bash
# building
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario building -n 12 -o 1.5 --sensor-range 10 \
  --rrt-iterations 5000 --playback-speed 3 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# office
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario office -n 12 -o 1.5 --sensor-range 10 \
  --rrt-iterations 5000 --playback-speed 3 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# warehouse
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario warehouse -n 12 -o 1.5 --sensor-range 10 \
  --rrt-iterations 5000 --playback-speed 3 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# bottleneck
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario bottleneck -n 0 -o 1.5 --sensor-range 10 \
  --rrt-iterations 5000 --playback-speed 3 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# mega_hospital
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario mega_hospital -n 15 -o 2.0 --sensor-range 12 \
  --rrt-iterations 30000 --playback-speed 3 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# mega_airport
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario mega_airport -n 20 -o 2.0 --sensor-range 12 \
  --rrt-iterations 50000 --playback-speed 3 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# megacampus
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario megacampus -n 20 -o 2.0 --sensor-range 15 \
  --rrt-iterations 80000 --playback-speed 3 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py

# mega_university
cd ~/SMART3D && cargo run --release --example smart_rviz_json -- \
  --scenario mega_university -n 25 -o 2.0 --sensor-range 15 \
  --rrt-iterations 160000 --playback-speed 3 --lsr-max 80 \
  | python3 ~/smart_rviz_bridge/smart_rviz_bridge.py
```

---

## 4. Benchmark Commands

### Shared vs Independent (12,000 trials)
```bash
cd ~/SMART3D && python3 benchmarks/bench_sharing_ablation.py --trials 1000 --jobs 0
```

### Lazy Eager vs Swift (12,000 trials)
```bash
cd ~/SMART3D && python3 benchmarks/bench_e3_vs_senior.py --trials 1000 --jobs 0
```

### Single Agent baseline (6,000 trials)
```bash
cd ~/SMART3D && python3 benchmarks/bench_single_agent.py --trials 1000 --jobs 0
```

### Current vs Deferred Eager (12,000 trials)
```bash
cd ~/SMART3D && python3 benchmarks/bench_deferred.py --trials 1000 --jobs 0
```

### Fast test (any benchmark, 10 trials)
```bash
cd ~/SMART3D && python3 benchmarks/bench_sharing_ablation.py --trials 10 --scenarios mega_hospital
```

---

## 5. Benchmark Results Summary (50,000+ trials total)

### Algorithm Evolution (single agent, megacampus)

| Version | Median | Speedup | Rebuilds | Success |
|---------|--------|---------|----------|---------|
| Full Rebuild | 76.3s | 1.0x | 139.6 | 96.6% |
| Eager Incremental | 8.71s | 8.8x | 2.2 | 99.9% |
| Lazy Swap | 3.40s | 22.4x | 3.9 | 97.4% |
| Hybrid (abandoned) | 3.42s | 22.3x | 2.3 | 98.6% |
| **Lazy+Eager (final)** | **best** | **22.4x** | **low** | **high** |

### Lazy Eager vs Swift (4 agents, 1000 trials each)

| Scenario | Lazy Eager | Swift | Lazy Eager Rebuilds | Swift Rebuilds |
|----------|-----|--------|-------------|-----------------|
| building | 23.2s | 27.4s | 3.0 | 15.2 |
| mega_hospital | 91.9s | 119.3s | 34.0 | 102.7 |
| mega_airport | 116.2s | 151.7s | 30.9 | 91.8 |
| megacampus | 117.7s | 121.8s | 54.9 | 121.3 |

**Wall-clock (CPU) time tradeoff:** Lazy Eager scans all edges per repair, so CPU cost per trial is higher. Megacampus: Lazy Eager wall_time = 66.1s vs Swift wall_time = 39.5s. Lazy Eager makes the CPU work harder but the robot finishes faster (sim_time 117.7s vs 121.8s). Spatial Index v2 closes this gap.

### Shared vs Independent (4 agents, 1000 trials each)

| Scenario | Shared | Independent | Speedup | Spread reduction |
|----------|--------|-------------|---------|-----------------|
| building | 23.2s | 48.4s | 2.09x | 1.0x |
| office | 27.8s | 33.5s | 1.21x | 4.4x |
| mega_hospital | 92.8s | 108.5s | 1.17x | 2.5x |
| megacampus | 116.8s | 137.4s | 1.18x | 4.2x |

### Current vs Deferred (4 agents, 1000 trials each)

| Scenario | Current | Deferred 30m | Verdict |
|----------|---------|-------------|---------|
| building | 23.1s | 23.3s | Same |
| office | 28.1s | 27.9s | Same |
| mega_hospital | 92.0s | 94.9s | Current better |
| mega_airport | 115.3s | 119.8s | Current better |
| megacampus | 116.6s | 115.6s | Same |

### Spatial Index v2 (4 agents, wall-clock time)

| Scenario | Baseline | Spatial v2 | Change |
|----------|----------|-----------|--------|
| building | 0.5s | 0.7s | +40% (overhead > benefit) |
| mega_hospital | 13.4s | 11.2s | -16% |
| megacampus | 66.1s | 48.6s | **-26%** |

### Single Agent (1000 trials, 0 people)

| Scenario | Median | Success | Rebuilds |
|----------|--------|---------|----------|
| building | 20.4s | 100% | 0.8 |
| office | 20.6s | 100% | 1.1 |
| warehouse | 14.7s | 100% | 0.2 |
| mega_hospital | 56.6s | 100% | 1.8 |
| mega_airport | 94.4s | 100% | 2.5 |
| megacampus | 108.6s | 100% | 4.9 |

---

## 6. File Locations

### i9 Alienware Aurora R16

```
~/SMART3D/                                  ← git clone from LinksLabUConn/SMART3D
├── Cargo.toml                              ← Rust dependencies
├── src/
│   ├── smart/smart.rs                      ← SMART core algorithm (from paper, modified)
│   ├── obstacles/
│   │   ├── analytic_obstacle.rs            ← Original wall collision checker
│   │   ├── spatial_checker.rs              ← ★ Spatial index v2 (my addition)
│   │   └── mod.rs
│   └── rrt/                                ← RRT* tree builder + KD-tree
├── examples/
│   ├── smart_multi_agent.rs                ← ★ Multi-agent simulation
│   ├── smart_multi_agent_deferred.rs       ← ★ Same + --local-horizon flag
│   ├── smart_rviz_json.rs                  ← Single-agent simulation
│   └── smart_planner_server.rs             ← ★ Rust planner for real robot
├── i9_planner.py                           ← ★ Central ROS2 planner (~750 lines)
├── bench_sharing_ablation.py               ← Shared vs Independent benchmark
├── bench_single_agent.py                   ← Single agent benchmark
├── bench_e3_vs_senior.py                   ← Lazy Eager vs Swift benchmark; filename keeps old names
├── bench_deferred.py                       ← Current vs Deferred benchmark
├── sharing_ablation.json                   ← 12,000 trial results
├── single_agent_benchmark.json             ← 6,000 trial results
├── deferred_benchmark.json                 ← 12,000 trial results
└── target/release/examples/
    ├── smart_multi_agent                   ← Compiled binary
    ├── smart_multi_agent_deferred          ← Compiled binary
    ├── smart_rviz_json                     ← Compiled binary
    └── smart_planner_server                ← Compiled binary

~/smart_rviz_bridge/
├── smart_rviz_bridge.py                    ← Single-agent JSON → ROS2 Marker bridge
├── smart_multi_agent_bridge.py             ← Multi-agent JSON → ROS2 Marker bridge
└── smart_rviz.rviz                         ← RViz config (uses "map" frame)
```

### Raspberry Pi 5 (inside Docker container "ros2-humble")

```
SSH: robotcar@10.42.0.1 (AP) or robotcar@10.66.129.126 (campus), password: 234123

/root/
├── pi5_client.py                           ← ★ Thin client: TF→pose, path→pure pursuit
├── cmd_vel_driver.py                       ← RS485 motor control (4× DDSM115)
├── ddsm115_odom.py                         ← Wheel odometry
├── start_full.sh                           ← Sensor startup script
├── smart_nav_node_v4.py                    ← Legacy single-machine nav (reference)
└── cartographer_config/                    ← SLAM config
```

### Yahboom ROSMASTER M3 PRO (Jetson Orin NX)

```
SSH: jetson@10.66.147.9, password: yahboom
Mecanum wheels (linear.x + linear.y + angular.z)
Needs: m3pro_client.py (TO WRITE, based on pi5_client.py)
Key difference from Pi5: mecanum can strafe, no cmd_vel_driver needed
```

### Backups

```
~/Desktop/SMART3D_backup_complete/
├── SMART3D_HANDOFF/                        ← Complete handoff package
├── PERFECT_spatial_index_v2_*/             ← Spatial index v2 backup
├── PERFECT_sharing_ablation_*/             ← Ablation data backup
└── backup_3d_twolayer/                     ← 3D simulation backup
```

### GitHub Repos

```
https://github.com/FelixFtch/SMART3D-MultiAgent.git     ← Private, all files
https://github.com/LinksLabUConn/Multi-agent-SMART.git   ← Lab, simulation code only
https://github.com/LinksLabUConn/SMART3D.git             ← Original SMART-3D crate
```

---

## 7. What I Borrowed vs What I Built

### Borrowed from SMART-3D (unchanged)

| API | Purpose |
|-----|---------|
| `SMART::update()` | Dynamic obstacle avoidance (people, other robots via OHZ) |
| `RRTstar::new()` | Build initial tree |
| `build_planned_path()` | Backtrack parent chain to get path |
| `hot_node_search()` | Find reconnection nodes for broken subtrees |
| `KdTreeNearestNeighbors` | Spatial lookup for tree operations |

### My contributions

| Contribution | Type |
|-------------|------|
| `update_static_obstacles()` | **Novel extension**: incremental repair for permanent walls. Key difference from paper's `update()`: uses `form_single_tree_after_static_update()` which permanently discards pruned nodes (walls never disappear), whereas the paper's `form_single_tree()` restores pruned nodes when dynamic obstacles leave. |
| Lazy + Eager adaptive architecture | **Novel architecture**: decision based on path affected or not |
| Global King + flee + chain push | **Novel system**: multi-robot coordination in continuous space, analogous to PIBT priority inheritance but in continuous coordinates |
| Shared wall discovery | **Novel mechanism**: cooperative map sharing via shared Vec |
| `--no-wall-sharing` ablation mode | **Experiment infrastructure**: proves sharing value |
| `--local-horizon` deferred Eager | **Experiment**: tested and found current system optimal |
| `update_static_obstacles_path_only()` | **Baseline implementation**: replicates the path-only/path-local repair approach for fair comparison (Swift mode; old CLI flag still says `--senior-mode`) |
| Spatial index v2 | **Engineering optimization**: -26% CPU on mega maps |
| LSR = 80m tuning | **Parameter tuning**: repair success 70% → 92% |
| Shortcut / Detour | **Standard technique**: ~0% benefit in multi-agent |
| 8 simulation scenarios | **System integration**: building through mega_university |
| 3D RViz v3 (unknown-map LiDAR) | **System integration**: walls start unknown, discovered via sensor range, triggers SMART rebuild. Backup: `~/Desktop/SMART3D_backup_complete/backup_3d_twolayer/` |
| Real robot deployment | **System integration**: i9 + Pi5 + CycloneDDS |
| 50,000+ trial benchmarks | **Validation**: comprehensive statistical evidence |

---

## 8. Setup on a New Computer

```bash
# 1. Clone the original SMART-3D crate
git clone https://github.com/LinksLabUConn/SMART3D.git
cd SMART3D

# 2. Clone your private repo
git clone https://github.com/FelixFtch/SMART3D-MultiAgent.git /tmp/my_smart

# 3. Copy your files over
cp /tmp/my_smart/examples/*.rs examples/
cp /tmp/my_smart/src/obstacles/spatial_checker.rs src/obstacles/
# Add spatial_checker to mod.rs if not already there:
grep -q "spatial_checker" src/obstacles/mod.rs || echo "pub mod spatial_checker;" >> src/obstacles/mod.rs

# 4. Add example entries to Cargo.toml if needed
for name in smart_multi_agent smart_multi_agent_deferred; do
  grep -q "$name" Cargo.toml || cat >> Cargo.toml << EOF

[[example]]
name = "$name"
path = "examples/${name}.rs"
EOF
done

# 5. Compile
cargo build --release --example smart_multi_agent
cargo build --release --example smart_multi_agent_deferred
cargo build --release --example smart_rviz_json
cargo build --release --example smart_planner_server

# 6. Install ROS2 Humble + set up bridge
sudo apt install ros-humble-desktop
mkdir -p ~/smart_rviz_bridge
cp /tmp/my_smart/rviz_bridge/* ~/smart_rviz_bridge/

# 7. Copy benchmark scripts
cp /tmp/my_smart/benchmarks/*.py .
cp -r /tmp/my_smart/benchmarks/data .

# 8. Copy deploy files (for real robot)
cp -r /tmp/my_smart/deploy .

# 9. Test
source /opt/ros/humble/setup.bash
rviz2 &
# Fixed Frame = "map", Add MarkerArray on /smart/markers
cargo run --release --example smart_multi_agent -- \
  --scenario building --num-agents 4 --agent-mode cross \
  -n 0 -o 1.5 --playback-speed 3 --sensor-range 10 \
  --rrt-iterations 5000 --lsr-max 30 \
  | python3 ~/smart_rviz_bridge/smart_multi_agent_bridge.py
```

---

## 9. Network Topology (Real Robot)

```
Internet (UCONN-GUEST)
    │
    ├── i9 (campus: 10.66.112.64)
    │       └── USB WiFi → 10.42.0.160
    │               │
    │         robotcar AP (10.42.0.0/24) ← ALL ROS2 TRAFFIC
    │               │
    ├── Pi 5 (wlan1: 10.42.0.1 = AP host, campus: 10.66.129.126)
    │
    ├── M3 PRO (USB WiFi → 10.42.0.x, campus: 10.66.147.9)
    │
    └── (Future: Balance Bot → 10.42.0.x)

DDS: CycloneDDS (NOT FastDDS), ROS_DOMAIN_ID=42
CycloneDDS config: /tmp/cyclone.xml (recreate after every reboot)
```

---

## 9.5. Real Robot Deployment Procedure

### Why centralized planning

Cartographer SLAM on Pi5 already consumes ~1.5 CPU cores. RRT* 15000 iters: i9 ~40ms, Pi5 ~300ms (7.5x slower). Running SMART-3D on-board leaves no headroom for SLAM.

### Startup sequence (single car)

```bash
# === Pi5 (SSH into 10.42.0.1) ===
# 1. Start sensors + SLAM
bash start_full.sh
# Launches: Cartographer, LiDAR driver, camera (if needed)

# 2. Start thin client
python3 pi5_client.py
# Subscribes to /smart/path from i9, publishes /robot/pose
# Pure pursuit path follower + local emergency stop

# === i9 (10.42.0.160) ===
# 3. Recreate CycloneDDS config (lost after reboot)
cat > /tmp/cyclone.xml << 'EOF'
<CycloneDDS>
  <Domain><General><NetworkInterfaceAddress>10.42.0.160</NetworkInterfaceAddress></General>
  <Discovery><Peers><Peer address="10.42.0.1"/></Peers></Discovery></Domain>
</CycloneDDS>
EOF
export CYCLONEDDS_URI=file:///tmp/cyclone.xml
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=42

# 4. Start planner
source /opt/ros/humble/setup.bash
cd ~/SMART3D && python3 i9_planner.py
# Receives /scan + /tf from Pi5, runs SMART-3D in Rust subprocess,
# publishes /smart/path back to Pi5

# 5. (Optional) RViz on i9
rviz2 -d ~/smart_rviz_bridge/smart_rviz.rviz
```

### Multi-car startup

```bash
# i9 planner supports multiple cars:
python3 i9_planner.py --ros-args -p car_names:="car1,m3pro"
# Each car spawns its own Rust smart_planner_server with per-agent robot_radius
# Other cars are encoded as dynamic obstacles with OHZ
```

### Sensing pipeline on i9

LiDAR scan + SLAM pose arrive from Pi5 → i9 extracts walls from occupancy grid → compares each LiDAR point against known walls → points not near any wall = dynamic obstacle → cluster (min 8 points) → frame-to-frame tracking for velocity estimation → feed (x, y, radius, speed) to Rust SMART planner.

### Dynamic obstacle detection tuning

Cluster minimum: 8 LiDAR points. Update interval: 1.0s. Temporal persistence filter active. Robot marker: CUBE 0.40x0.35x0.15m matching physical chassis.

---

## 10. Known Issues and Gotchas

1. **CycloneDDS config must be recreated after every reboot** (lives in /tmp)
2. **Pi CycloneDDS must NOT restrict interfaces** (no `<Interfaces>` tag, breaks internal container comms)
3. **M3 PRO start_agent.sh uses DOMAIN_ID=30**, must change to 42
4. **RViz Fixed Frame must be "map"** (not "world"), matching bridge's frame_id
5. **Emoji in stderr** (👑) can cause UTF-8 decode errors in Python benchmarks. All benchmark scripts use `decode("utf-8", errors="replace")`.
6. **Building scenario only has 2 agents** (map too small for 4). Benchmark scripts handle this via `all_time is not None` success detection.
7. **Rust compiler warning about unused AnalyticValidityChecker import** in deferred binary. Harmless, ignore.
8. **`bench_parallel.py`** uses different config naming than other benchmarks. Use the newer `bench_single_agent.py` instead.
9. **Two non-King agents can deadlock in narrow corridors.** King push and flee only involve the King. If two non-King agents face each other in a corridor too narrow for flee (e.g., mega_hospital OR wing, 5m wide), neither yields. This accounts for mega_hospital's ~96% success rate (vs 100% on other mega maps). Fix would require extending priority inheritance to non-King pairs.
10. **CycloneDDS WiFi lockup on Pi5.** After WiFi reconnect, DDS hangs. Fix: `killall python3; killall rviz2` before reconnecting WiFi, then restart all ROS2 nodes.

---

## 11. Paper-Ready Claims with Evidence

| # | Claim | Evidence | Trials |
|---|-------|---------|--------|
| 1 | Incremental repair is 8.8x faster than full rebuild | Megacampus: 8.71s vs 76.27s | 1,000 |
| 2 | Lazy+Eager combo achieves 22.4x speedup | Megacampus: 3.40s vs 76.27s | 1,000 |
| 3 | Full-tree Lazy Eager repair has 4-9x fewer rebuilds than path-only Swift | mega_hospital: 34.0 vs 102.7 | 1,000 |
| 4 | Shared wall discovery provides 1.04x–2.09x speedup | All 6 scenarios | 12,000 |
| 5 | Shared reduces per-agent completion time spread by up to 4.2x | Megacampus: 12.3s → 2.9s | 12,000 |
| 6 | Stored single-agent Lazy Eager baseline is 100% successful on the six committed 2D scenarios | building through megacampus | 6,000 |
| 7 | Spatial index v2 reduces CPU time by 26% on 300-wall maps | Megacampus WallMed: 66.1s → 48.6s | 1,000 |
| 8 | 8-agent bottleneck (2m corridor) 100% pass rate | bottleneck scenario | 1,000 |
| 9 | Immediate Eager is optimal (deferred shows no improvement) | 6 scenarios, current vs deferred 30m | 12,000 |
| 10 | Independent mode has 0 incremental repairs (all rebuilds) | IncOK=0 in no-sharing mode | 12,000 |

---

*Felix Fang (Tianchonghui Fang) · LinksLab, University of Connecticut · March 2026*
*Advisor: Prof. Shalabh Gupta*
*Based on SMART-3D by Zongyuan Shen (arXiv:2509.16812)*
