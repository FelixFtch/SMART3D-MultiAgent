#!/usr/bin/env python3
"""
SMART3D Parallel Benchmark — strategy comparison for LiDAR wall discovery.

Default mode focuses on the comparison the project currently needs:
  - A0: SMART-like global static repair (closest to SMART-3D style)
  - S0: Senior path-only static repair
  - L0/L1: Lazy checker-swap baseline at LSR 10 / 80
  - R0: Recommended current method

A legacy ablation set is still available for the old LSR/shortcut/detour study.

Usage:
    python3 bench_parallel.py --trials 1000 --scenarios mega_airport --jobs 0
    python3 bench_parallel.py --config-set legacy_ablation --trials 1000 --jobs 0
    python3 bench_parallel.py --quick
"""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from datetime import datetime
from statistics import mean, median, stdev
from typing import Any

# ==================== Configuration ====================

SMART3D_DIR = os.path.expanduser("~/SMART3D")
BINARY = os.path.join(SMART3D_DIR, "target/release/examples/smart_rviz_json")

SCENARIOS: dict[str, dict[str, Any]] = {
    "building":        {"n": 12, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000},
    "office":          {"n": 12, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000},
    "warehouse":       {"n": 12, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000},
    "mega_hospital":   {"n": 15, "o": 2.0, "sensor_range": 12, "rrt_iters": 30000},
    "mega_airport":    {"n": 20, "o": 2.0, "sensor_range": 12, "rrt_iters": 50000},
    "megacampus":      {"n": 20, "o": 2.0, "sensor_range": 15, "rrt_iters": 80000},
    "mega_university": {"n": 25, "o": 2.0, "sensor_range": 15, "rrt_iters": 160000},
}


def cfg(
    *,
    wall_strategy: str,
    wall_block_check: bool,
    goal_bias: float,
    shortcut_iters: int,
    detour_threshold: float,
    lsr_max: float,
    lrz_time: float,
    desc: str,
    timeout: int = 180,
) -> dict[str, Any]:
    return {
        "wall_strategy": wall_strategy,
        "wall_block_check": wall_block_check,
        "goal_bias": goal_bias,
        "shortcut_iters": shortcut_iters,
        "detour_threshold": detour_threshold,
        "lsr_max": lsr_max,
        "lrz_time": lrz_time,
        "desc": desc,
        "timeout": timeout,
    }


# ==================== Core comparison set ====================

COMPARE_CONFIGS: dict[str, dict[str, Any]] = {
    "A0_smart_global_lsr10": cfg(
        wall_strategy="smart-global",
        wall_block_check=False,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Closest SMART-like wall handling",
    ),
    "S0_senior_path_lsr10": cfg(
        wall_strategy="senior-path",
        wall_block_check=False,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Senior path-only baseline",
    ),
    "L0_lazy_swap_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=False,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Lazy checker swap, LSR=10",
    ),
    "L1_lazy_swap_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=False,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Lazy checker swap, LSR=80",
    ),
    "R0_recommended_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Recommended current method",
    ),
    # === LSR=80 fair comparison (all at LSR=80, no extras) ===
    "A1_smart_global_lsr80": cfg(
        wall_strategy="smart-global",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="SMART-like global repair, LSR=80",
    ),
    "S1_senior_path_lsr80": cfg(
        wall_strategy="senior-path",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Senior path-only, LSR=80",
    ),
}


# ==================== Legacy ablation set ====================
# Preserves the earlier study structure, but now makes the wall strategy explicit.
# This set tracks the current method family (lazy swap + wall-block safety).

LEGACY_ABLATION_CONFIGS: dict[str, dict[str, Any]] = {
    "A0_legacy_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy baseline used in earlier dashboard",
    ),
    "B1_sc_only_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy +shortcut only",
    ),
    "B2_det_only_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=1.5,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy +detour only",
    ),
    "B3_sc_det_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy +shortcut+detour",
    ),
    "C1_alloff_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy LSR sweep 10",
    ),
    "C2_alloff_lsr30": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=30.0,
        lrz_time=1.0,
        desc="Legacy LSR sweep 30",
    ),
    "C3_alloff_lsr50": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=50.0,
        lrz_time=1.0,
        desc="Legacy LSR sweep 50",
    ),
    "C4_alloff_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy LSR sweep 80",
    ),
    "C5_alloff_lsr120": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=120.0,
        lrz_time=1.0,
        desc="Legacy LSR sweep 120",
    ),
    "D1_sc_det_lsr10": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=10.0,
        lrz_time=1.0,
        desc="Legacy LSR+ext 10",
    ),
    "D2_sc_det_lsr30": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=30.0,
        lrz_time=1.0,
        desc="Legacy LSR+ext 30",
    ),
    "D3_sc_det_lsr50": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=50.0,
        lrz_time=1.0,
        desc="Legacy LSR+ext 50",
    ),
    "D4_sc_det_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy LSR+ext 80",
    ),
    "D5_sc_det_lsr120": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=120.0,
        lrz_time=1.0,
        desc="Legacy LSR+ext 120",
    ),
    "E0_alloff_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy ablation all off",
    ),
    "E1_sc_only_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=0.0,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy ablation shortcut",
    ),
    "E2_det_only_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=0,
        detour_threshold=1.5,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy ablation detour",
    ),
    "E3_sc_det_lsr80": cfg(
        wall_strategy="lazy-swap",
        wall_block_check=True,
        goal_bias=0.0,
        shortcut_iters=150,
        detour_threshold=1.5,
        lsr_max=80.0,
        lrz_time=1.0,
        desc="Legacy ablation recommended",
    ),
}

CONFIG_SETS = {
    "compare": COMPARE_CONFIGS,
    "legacy_ablation": LEGACY_ABLATION_CONFIGS,
}

SUMMARY_RE = re.compile(r"COMPARE_SUMMARY\s+(.*)$")
KV_RE = re.compile(r"([A-Za-z_]+)=([^\s]+)")
GOAL_RE = re.compile(r"GOAL REACHED \(frame (\d+), t=([\d.]+)s\)")
TIMEOUT_RE = re.compile(r"Timeout")

SUMMARY_INT_KEYS = {
    "goal_reached",
    "total_frames",
    "lidar_events",
    "new_walls",
    "path_hits_total",
    "path_affected",
    "path_clear",
    "static_noop",
    "static_lazy",
    "static_global",
    "static_path",
    "static_ok",
    "static_fail",
    "static_rebuilds",
    "dynamic_replans",
    "wall_blocks",
    "detours",
    "uturns",
    "rebuilds_total",
    "known_walls_final",
}
SUMMARY_FLOAT_KEYS = {"sim_time"}


def parse_compare_summary(stderr_text: str) -> dict[str, Any]:
    summary: dict[str, Any] = {}
    for line in stderr_text.splitlines():
        match = SUMMARY_RE.search(line)
        if not match:
            continue
        for key, raw_value in KV_RE.findall(match.group(1)):
            if key in SUMMARY_INT_KEYS:
                try:
                    summary[key] = int(raw_value)
                except ValueError:
                    summary[key] = raw_value
            elif key in SUMMARY_FLOAT_KEYS:
                try:
                    summary[key] = float(raw_value)
                except ValueError:
                    summary[key] = raw_value
            else:
                summary[key] = raw_value
    return summary


def run_single_trial(args_tuple: tuple[Any, ...]) -> dict[str, Any]:
    """Run one trial. Returns a result dict."""
    scenario, scenario_cfg, config_name, config_cfg, trial_id, output_dir = args_tuple

    cmd = [
        BINARY,
        "--scenario", scenario,
        "-n", str(scenario_cfg["n"]),
        "-o", str(scenario_cfg["o"]),
        "--sensor-range", str(scenario_cfg["sensor_range"]),
        "--rrt-iterations", str(scenario_cfg["rrt_iters"]),
        "--playback-speed", "99999",
        "--goal-bias", str(config_cfg["goal_bias"]),
        "--shortcut-iters", str(config_cfg["shortcut_iters"]),
        "--detour-threshold", str(config_cfg["detour_threshold"]),
        "--lsr-max", str(config_cfg["lsr_max"]),
        "--lrz-time", str(config_cfg["lrz_time"]),
        "--wall-strategy", str(config_cfg["wall_strategy"]),
        "--debug-compare",
    ]
    if config_cfg.get("wall_block_check"):
        cmd.append("--wall-block-check")

    log_file = os.path.join(output_dir, f"{scenario}_{config_name}_{trial_id:04d}.log")

    t0 = time.monotonic()
    try:
        result = subprocess.run(
            cmd,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            timeout=int(config_cfg.get("timeout", 180)),
            text=True,
            cwd=SMART3D_DIR,
        )
        wall_time = time.monotonic() - t0
        stderr_text = result.stderr
    except subprocess.TimeoutExpired:
        wall_time = time.monotonic() - t0
        stderr_text = "HARD_TIMEOUT"
    except Exception as exc:  # pragma: no cover - defensive
        return {
            "scenario": scenario,
            "config": config_name,
            "trial": trial_id,
            "success": False,
            "error": str(exc),
        }

    with open(log_file, "w", encoding="utf-8") as f:
        f.write(stderr_text)

    sim_time: float | None = None
    frames: int | None = None
    goal_reached = False

    for line in stderr_text.splitlines():
        m = GOAL_RE.search(line)
        if m:
            goal_reached = True
            frames = int(m.group(1))
            sim_time = float(m.group(2))
        if TIMEOUT_RE.search(line):
            sim_time = 600.0

    summary = parse_compare_summary(stderr_text)
    if summary:
        goal_reached = bool(summary.get("goal_reached", int(goal_reached)))
        sim_time = float(summary.get("sim_time", sim_time if sim_time is not None else 600.0))
        frames = int(summary.get("total_frames", frames if frames is not None else 0))

    return {
        "scenario": scenario,
        "config": config_name,
        "trial": trial_id,
        "success": goal_reached,
        "sim_time": sim_time,
        "wall_time": round(wall_time, 3),
        "frames": frames,
        "mode": summary.get("mode", config_cfg.get("wall_strategy")),
        "rebuilds": int(summary.get("rebuilds_total", 0)),
        "lidar_events": int(summary.get("lidar_events", 0)),
        "new_walls": int(summary.get("new_walls", 0)),
        "path_hits_total": int(summary.get("path_hits_total", 0)),
        "path_affected": int(summary.get("path_affected", 0)),
        "path_clear": int(summary.get("path_clear", 0)),
        "static_noop": int(summary.get("static_noop", 0)),
        "static_lazy": int(summary.get("static_lazy", 0)),
        "static_global": int(summary.get("static_global", 0)),
        "static_path": int(summary.get("static_path", 0)),
        "static_ok": int(summary.get("static_ok", 0)),
        "static_fail": int(summary.get("static_fail", 0)),
        "static_rebuilds": int(summary.get("static_rebuilds", 0)),
        "dynamic_replans": int(summary.get("dynamic_replans", 0)),
        "wall_blocks": int(summary.get("wall_blocks", 0)),
        "detours": int(summary.get("detours", 0)),
        "uturns": int(summary.get("uturns", 0)),
        "known_walls_final": int(summary.get("known_walls_final", 0)),
    }


def avg_or_na(values: list[float | int]) -> str:
    if not values:
        return "n/a"
    return f"{mean(values):.1f}"


def med_or_na(values: list[float | int], suffix: str = "") -> str:
    if not values:
        return "n/a"
    return f"{median(values):.1f}{suffix}"


def std_or_na(values: list[float | int]) -> str:
    if len(values) < 2:
        return "n/a"
    return f"{stdev(values):.1f}"


def main() -> None:
    parser = argparse.ArgumentParser(description="SMART3D Parallel Benchmark")
    parser.add_argument("--trials", type=int, default=50)
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--scenarios", nargs="+", default=None)
    parser.add_argument("--configs", nargs="+", default=None)
    parser.add_argument(
        "--config-set",
        choices=sorted(CONFIG_SETS.keys()),
        default="compare",
        help="Which configuration family to use",
    )
    parser.add_argument("--quick", action="store_true")
    args = parser.parse_args()

    if not os.path.isfile(BINARY):
        print(f"ERROR: Binary not found: {BINARY}")
        print("Run: cd ~/SMART3D && cargo build --release --example smart_rviz_json")
        sys.exit(1)

    config_pool = CONFIG_SETS[args.config_set]

    if args.quick:
        scenarios = {
            k: v
            for k, v in SCENARIOS.items()
            if k in ["building", "office", "warehouse"]
        }
        quick_names = {
            "compare": [
                "A0_smart_global_lsr10",
                "S0_senior_path_lsr10",
                "A1_smart_global_lsr80",
                "S1_senior_path_lsr80",
                "L1_lazy_swap_lsr80",
                "R0_recommended_lsr80",
            ],
            "legacy_ablation": [
                "A0_legacy_lsr10",
                "E0_alloff_lsr80",
                "E3_sc_det_lsr80",
            ],
        }[args.config_set]
        configs = {k: config_pool[k] for k in quick_names if k in config_pool}
        trials = 10
    else:
        scenarios = {
            k: SCENARIOS[k]
            for k in (args.scenarios or SCENARIOS.keys())
            if k in SCENARIOS
        }
        configs = {
            k: config_pool[k]
            for k in (args.configs or config_pool.keys())
            if k in config_pool
        }
        trials = args.trials

    if not scenarios:
        print("ERROR: No valid scenarios selected")
        sys.exit(1)
    if not configs:
        print("ERROR: No valid configs selected")
        sys.exit(1)

    n_jobs = args.jobs if args.jobs > 0 else (os.cpu_count() or 1)
    total_runs = len(scenarios) * len(configs) * trials

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_base = os.path.expanduser(f"~/SMART3D_bench/bench_{timestamp}")
    raw_dir = os.path.join(output_base, "raw")
    os.makedirs(raw_dir, exist_ok=True)

    print("╔══════════════════════════════════════════════════════════════════╗")
    print(f"║  SMART3D Parallel Benchmark ({args.config_set}){' ' * max(0, 21-len(args.config_set))}║")
    print(
        f"║  Scenarios: {len(scenarios):3d}  |  Configs: {len(configs):3d}  |  Trials: {trials:4d}              ║"
    )
    print(
        f"║  Total runs: {total_runs:5d}  |  Parallel jobs: {n_jobs:3d}                          ║"
    )
    print(f"║  Output: {output_base:<53s}║")
    print("╚══════════════════════════════════════════════════════════════════╝")
    print()

    tasks: list[tuple[Any, ...]] = []
    for scenario, scfg in scenarios.items():
        for config_name, ccfg in configs.items():
            for t in range(trials):
                tasks.append((scenario, scfg, config_name, ccfg, t, raw_dir))

    results: list[dict[str, Any]] = []
    t_start = time.monotonic()
    completed = 0

    with ProcessPoolExecutor(max_workers=n_jobs) as executor:
        futures = {executor.submit(run_single_trial, task): task for task in tasks}
        for future in as_completed(futures):
            r = future.result()
            results.append(r)
            completed += 1
            if completed % 10 == 0 or completed == total_runs:
                elapsed = time.monotonic() - t_start
                eta = (elapsed / completed) * (total_runs - completed) if completed > 0 else 0
                ok = sum(1 for x in results if x.get("success"))
                print(
                    f"\r  [{completed}/{total_runs}] {ok} success | elapsed {elapsed:.0f}s | ETA {eta:.0f}s   ",
                    end="",
                    flush=True,
                )

    total_time = time.monotonic() - t_start
    print(f"\n\n  Done in {total_time:.0f}s ({total_time / 60:.1f}min)")

    grouped: dict[tuple[str, str], list[dict[str, Any]]] = defaultdict(list)
    for r in results:
        grouped[(r["scenario"], r["config"])] .append(r)

    lines: list[str] = []
    lines.append("=" * 168)
    lines.append(f"  SMART3D Benchmark — {timestamp} — config_set={args.config_set}")
    lines.append(
        f"  Trials per config: {trials} | Parallel jobs: {n_jobs} | Total time: {total_time:.0f}s"
    )
    lines.append("=" * 168)
    lines.append("")
    lines.append(
        f"  {'Config':<25s} │ {'Mode':<12s} │ {'WBlk':>4s} │ {'sc':>4s} │ {'det':>4s} │ {'lsr':>4s} │ {'Description'}"
    )
    lines.append(
        f"  {'─'*25}─┼─{'─'*12}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*70}"
    )
    for cname, ccfg in configs.items():
        lines.append(
            f"  {cname:<25s} │ {ccfg['wall_strategy']:<12s} │ {int(ccfg['wall_block_check']):>4d} │ {ccfg['shortcut_iters']:>4d} │ {ccfg['detour_threshold']:>4.1f} │ {ccfg['lsr_max']:>4.0f} │ {ccfg['desc']}"
        )
    lines.append("")

    for scenario in scenarios:
        lines.append(f"  ┌─── {scenario} ───")
        lines.append(
            f"  │ {'Config':<25s} │ {'Success':>7s} │ {'Med':>8s} │ {'Mean':>8s} │ {'Std':>6s} │ {'RB':>4s} │ {'Noop':>4s} │ {'Lazy':>4s} │ {'GRep':>4s} │ {'PRep':>4s} │ {'SFail':>5s} │ {'WBlk':>4s} │ {'Det':>4s} │ {'Dyn':>4s} │"
        )
        lines.append(
            f"  │{'─'*25}─┼─{'─'*7}─┼─{'─'*8}─┼─{'─'*8}─┼─{'─'*6}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*5}─┼─{'─'*4}─┼─{'─'*4}─┼─{'─'*4}─│"
        )

        for config_name in configs:
            runs = grouped.get((scenario, config_name), [])
            if not runs:
                continue

            successes = [r for r in runs if r.get("success")]
            rate = f"{len(successes)}/{len(runs)}"
            time_vals = [r["sim_time"] for r in successes if r.get("sim_time") is not None]
            rb_vals = [r["rebuilds"] for r in runs]
            noop_vals = [r["static_noop"] for r in runs]
            lazy_vals = [r["static_lazy"] for r in runs]
            grep_vals = [r["static_global"] for r in runs]
            prep_vals = [r["static_path"] for r in runs]
            sfail_vals = [r["static_fail"] for r in runs]
            wblk_vals = [r["wall_blocks"] for r in runs]
            det_vals = [r["detours"] for r in runs]
            dyn_vals = [r["dynamic_replans"] for r in runs]

            lines.append(
                f"  │ {config_name:<25s} │ {rate:>7s} │ {med_or_na(time_vals, 's'):>8s} │ {avg_or_na(time_vals):>7s}s │ {std_or_na(time_vals):>6s} │ {avg_or_na(rb_vals):>4s} │ {avg_or_na(noop_vals):>4s} │ {avg_or_na(lazy_vals):>4s} │ {avg_or_na(grep_vals):>4s} │ {avg_or_na(prep_vals):>4s} │ {avg_or_na(sfail_vals):>5s} │ {avg_or_na(wblk_vals):>4s} │ {avg_or_na(det_vals):>4s} │ {avg_or_na(dyn_vals):>4s} │"
            )

        lines.append(f"  └{'─'*162}")
        lines.append("")

    report = "\n".join(lines)
    print(report)

    report_file = os.path.join(output_base, "results.txt")
    with open(report_file, "w", encoding="utf-8") as f:
        f.write(report)

    json_file = os.path.join(output_base, "results.json")
    with open(json_file, "w", encoding="utf-8") as f:
        json.dump(results, f, indent=2)

    print(f"\n  Report: {report_file}")
    print(f"  JSON:   {json_file}")
    print(f"  Logs:   {raw_dir}/")


if __name__ == "__main__":
    main()
