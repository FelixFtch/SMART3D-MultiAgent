#!/usr/bin/env python3
"""
Single-Agent Benchmark — 1000 trials per scenario.
Matches multi-agent E3 parameters for direct comparison.
"""

import argparse
import json
import os
import re
import subprocess
import sys
import time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from statistics import mean, median, stdev

BINARY = os.path.expanduser("~/SMART3D/target/release/examples/smart_rviz_json")

SCENARIOS = {
    "building":      {"n": 0, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000,  "lsr_max": 30,  "timeout": 90},
    "office":        {"n": 0, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000,  "lsr_max": 30,  "timeout": 90},
    "warehouse":     {"n": 0, "o": 1.5, "sensor_range": 10, "rrt_iters": 5000,  "lsr_max": 30,  "timeout": 90},
    "mega_hospital": {"n": 0, "o": 1.5, "sensor_range": 12, "rrt_iters": 30000, "lsr_max": 60,  "timeout": 300},
    "mega_airport":  {"n": 0, "o": 1.5, "sensor_range": 12, "rrt_iters": 30000, "lsr_max": 60,  "timeout": 300},
    "megacampus":    {"n": 0, "o": 1.5, "sensor_range": 15, "rrt_iters": 50000, "lsr_max": 80,  "timeout": 300},
}

def run_trial(args):
    scenario, cfg, trial_id = args
    cmd = [
        BINARY,
        "--scenario", scenario,
        "-n", str(cfg["n"]),
        "-o", str(cfg["o"]),
        "--sensor-range", str(cfg["sensor_range"]),
        "--rrt-iterations", str(cfg["rrt_iters"]),
        "--lsr-max", str(cfg["lsr_max"]),
        "--shortcut-iters", "150",
        "--detour-threshold", "1.5",
        "--playback-speed", "99999",
    ]
    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                           timeout=cfg["timeout"], text=True)
        wall = time.monotonic() - t0
        err = p.stderr
    except subprocess.TimeoutExpired:
        return {"scenario": scenario, "trial": trial_id, "success": False,
                "sim_time": None, "wall_time": cfg["timeout"], "rebuilds": 0}
    except Exception as e:
        return {"scenario": scenario, "trial": trial_id, "success": False,
                "sim_time": None, "wall_time": 0, "rebuilds": 0}

    sim_time = None
    success = False
    rebuilds = 0
    inc_ok = 0
    inc_fail = 0

    for line in err.splitlines():
        m = re.search(r"GOAL REACHED \(frame \d+, t=([\d.]+)s\)", line)
        if m:
            success = True
            sim_time = float(m.group(1))
        if "Rebuilding SMART" in line or "falling back" in line or "rebuild" in line.lower():
            rebuilds += 1
        if "Incremental OK" in line or "Incremental update OK" in line or "incremental OK" in line:
            inc_ok += 1
        if "Incremental FAILED" in line or "ReplanningFailed" in line:
            inc_fail += 1

    return {
        "scenario": scenario, "trial": trial_id, "success": success,
        "sim_time": sim_time, "wall_time": round(wall, 2),
        "rebuilds": rebuilds, "inc_ok": inc_ok, "inc_fail": inc_fail,
    }

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--scenarios", nargs="+", default=None)
    parser.add_argument("--output", default="single_agent_benchmark.json")
    args = parser.parse_args()

    if not os.path.isfile(BINARY):
        print(f"ERROR: {BINARY} not found")
        sys.exit(1)

    scenarios = {k: SCENARIOS[k] for k in (args.scenarios or SCENARIOS.keys()) if k in SCENARIOS}
    jobs = args.jobs if args.jobs > 0 else os.cpu_count()
    total = len(scenarios) * args.trials

    print("=" * 80)
    print(f"  Single-Agent Benchmark | Trials: {args.trials} | Jobs: {jobs}")
    print(f"  Scenarios: {', '.join(scenarios.keys())}")
    print(f"  Total runs: {total}")
    print("=" * 80)

    tasks = []
    for sc, cfg in scenarios.items():
        for t in range(args.trials):
            tasks.append((sc, cfg, t))

    results = []
    t_start = time.monotonic()
    done = 0

    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(run_trial, t): t for t in tasks}
        for f in as_completed(futures):
            r = f.result()
            results.append(r)
            done += 1
            if done % 50 == 0 or done == total:
                elapsed = time.monotonic() - t_start
                eta = (elapsed / done) * (total - done) if done > 0 else 0
                ok = sum(1 for x in results if x["success"])
                print(f"\r  [{done}/{total}] {ok} success | {elapsed:.0f}s elapsed | ETA {eta:.0f}s   ", end="", flush=True)

    print(f"\n\n  Done in {time.monotonic() - t_start:.0f}s\n")

    # Summary
    grouped = defaultdict(list)
    for r in results:
        grouped[r["scenario"]].append(r)

    print("=" * 100)
    print(f"  SUMMARY: {args.trials} trials | Single Agent | 0 people")
    print("=" * 100)
    print(f"  {'Scenario':<18s} {'Success':>10s} {'Rate':>6s} {'SimMed':>8s} {'SimP95':>8s} "
          f"{'Rebuilds':>9s} {'IncOK':>7s} {'IncFail':>8s} {'WallMed':>8s}")
    print("  " + "-" * 94)

    summary = {}
    for sc in scenarios:
        runs = grouped[sc]
        ok = [r for r in runs if r["success"]]
        n_ok = len(ok)
        rate = f"{n_ok / len(runs) * 100:.1f}%"
        succ_str = f"{n_ok}/{len(runs)}"

        if ok:
            st = sorted([r["sim_time"] for r in ok if r["sim_time"]])
            wt = sorted([r["wall_time"] for r in ok])
            sim_med = f"{median(st):.1f}s" if st else "n/a"
            sim_p95 = f"{st[int(len(st)*0.95)]:.1f}s" if len(st) > 20 else "n/a"
            rb = f"{mean([r['rebuilds'] for r in ok]):.1f}"
            iok = f"{mean([r['inc_ok'] for r in ok]):.1f}"
            ifail = f"{mean([r['inc_fail'] for r in ok]):.1f}"
            wall_med = f"{median(wt):.1f}s"
            summary[sc] = {
                "success": n_ok, "total": len(runs), "rate": n_ok/len(runs),
                "sim_med": median(st) if st else None,
                "sim_p95": st[int(len(st)*0.95)] if len(st) > 20 else None,
                "rebuilds": mean([r['rebuilds'] for r in ok]),
                "inc_ok": mean([r['inc_ok'] for r in ok]),
                "inc_fail": mean([r['inc_fail'] for r in ok]),
                "wall_med": median(wt),
            }
        else:
            sim_med = sim_p95 = rb = iok = ifail = wall_med = "n/a"
            summary[sc] = {"success": 0, "total": len(runs)}

        print(f"  {sc:<18s} {succ_str:>10s} {rate:>6s} {sim_med:>8s} {sim_p95:>8s} "
              f"{rb:>9s} {iok:>7s} {ifail:>8s} {wall_med:>8s}")

    print("=" * 100)

    # Save
    out_path = os.path.join(os.path.expanduser("~/SMART3D"), args.output)
    with open(out_path, "w") as f:
        json.dump({"summary": summary, "raw": results}, f, indent=2)
    print(f"\n  Results saved to {out_path}")

if __name__ == "__main__":
    main()
