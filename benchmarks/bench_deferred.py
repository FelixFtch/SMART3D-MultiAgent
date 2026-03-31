#!/usr/bin/env python3
"""Benchmark: current vs deferred Eager (local-horizon=30m)"""
import argparse, json, os, re, subprocess, sys, time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from statistics import mean, median

BINARY_CURRENT = os.path.expanduser("~/SMART3D/target/release/examples/smart_multi_agent")
BINARY_DEFERRED = os.path.expanduser("~/SMART3D/target/release/examples/smart_multi_agent_deferred")

SCENARIOS = {
    "building":      {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "office":        {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "warehouse":     {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "mega_hospital": {"rrt_iters": 30000, "lsr_max": 60, "sensor_range": 12, "timeout": 300},
    "mega_airport":  {"rrt_iters": 50000, "lsr_max": 80, "sensor_range": 12, "timeout": 300},
    "megacampus":    {"rrt_iters": 80000, "lsr_max": 80, "sensor_range": 15, "timeout": 300},
}

MODES = {
    "current":     (BINARY_CURRENT, []),
    "deferred_30": (BINARY_DEFERRED, ["--local-horizon", "30"]),
}

def run_trial(args_tuple):
    scenario, scfg, mode_name, binary, mode_flags, trial_id = args_tuple
    cmd = [binary, "--scenario", scenario,
        "--num-agents", "4", "--agent-mode", "cross",
        "-n", "0", "-o", "1.5", "--playback-speed", "99999",
        "--sensor-range", str(scfg["sensor_range"]),
        "--rrt-iterations", str(scfg["rrt_iters"]),
        "--lsr-max", str(scfg["lsr_max"]),
        "--shortcut-iters", "150", "--detour-threshold", "1.5",
    ] + mode_flags
    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE, timeout=scfg["timeout"])
        wall_time = time.monotonic() - t0
        err = p.stderr.decode("utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return {"scenario": scenario, "mode": mode_name, "trial": trial_id,
                "success": False, "all_time": None, "wall_time": scfg["timeout"]}
    except Exception:
        return {"scenario": scenario, "mode": mode_name, "trial": trial_id,
                "success": False, "all_time": None, "wall_time": 0}

    agents = {}
    for m in re.finditer(r'(A\d+) GOAL REACHED at \(([\d.]+),([\d.]+)\), t=([\d.]+)s, rebuilds=(\d+)', err):
        agents[m.group(1)] = {"time": float(m.group(4)), "rebuilds": int(m.group(5))}
    all_time = None
    m_all = re.search(r'ALL AGENTS DONE at t=([\d.]+)s', err)
    if m_all: all_time = float(m_all.group(1))
    inc_ok = len(re.findall(r'incremental OK', err))
    rebuilds_total = sum(a.get("rebuilds", 0) for a in agents.values())
    success = all_time is not None
    return {"scenario": scenario, "mode": mode_name, "trial": trial_id,
            "success": success, "all_time": all_time, "wall_time": round(wall_time, 2),
            "agents": agents, "rebuilds_total": rebuilds_total, "inc_ok": inc_ok}

def pct(data, p):
    s = sorted(data)
    return s[min(int(len(s) * p / 100), len(s) - 1)]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--scenarios", nargs="+", default=None)
    args = parser.parse_args()
    scenarios = {k: SCENARIOS[k] for k in (args.scenarios or SCENARIOS.keys()) if k in SCENARIOS}
    jobs = args.jobs if args.jobs > 0 else os.cpu_count()
    total = len(scenarios) * len(MODES) * args.trials
    print("=" * 90)
    print(f"  Current vs Deferred Eager | Trials: {args.trials} | Jobs: {jobs} | Total: {total}")
    print(f"  Scenarios: {', '.join(scenarios.keys())}")
    print("=" * 90)
    tasks = []
    for sc, scfg in scenarios.items():
        for mn, (binary, mf) in MODES.items():
            for t in range(args.trials):
                tasks.append((sc, scfg, mn, binary, mf, t))
    results = []
    t_start = time.monotonic()
    done = 0
    with ProcessPoolExecutor(max_workers=jobs) as pool:
        futures = {pool.submit(run_trial, t): t for t in tasks}
        for f in as_completed(futures):
            results.append(f.result())
            done += 1
            if done % 50 == 0 or done == total:
                elapsed = time.monotonic() - t_start
                eta = (elapsed / done) * (total - done)
                ok = sum(1 for x in results if x["success"])
                print(f"\r  [{done}/{total}] {ok} ok | {elapsed:.0f}s | ETA {eta:.0f}s   ", end="", flush=True)
    print(f"\n  Done in {time.monotonic() - t_start:.0f}s\n")
    grouped = defaultdict(list)
    for r in results: grouped[(r["scenario"], r["mode"])].append(r)
    print("=" * 110)
    print(f"  RESULTS ({args.trials} trials, 4 agents)")
    print("=" * 110)
    for sc in scenarios:
        print(f"\n  --- {sc} ---")
        print(f"  {'Mode':<16s} {'Success':>10s} {'Rate':>6s} {'AllMed':>8s} {'AllP95':>8s} "
              f"{'RB':>5s} {'IncOK':>6s} {'WallMed':>8s}")
        print("  " + "-" * 75)
        for mn in MODES:
            ok = [r for r in grouped.get((sc, mn), []) if r["success"]]
            runs = grouped.get((sc, mn), [])
            if not ok:
                print(f"  {mn:<16s} {len(ok)}/{len(runs):>6s} {'0%':>6s}")
                continue
            at = sorted([r["all_time"] for r in ok if r["all_time"]])
            wt = [r["wall_time"] for r in ok]
            print(f"  {mn:<16s} {len(ok)}/{len(runs)} {len(ok)/len(runs)*100:>5.1f}% "
                  f"{median(at):>7.1f}s {pct(at,95):>7.1f}s "
                  f"{mean([r['rebuilds_total'] for r in ok]):>5.1f} "
                  f"{mean([r['inc_ok'] for r in ok]):>6.1f} "
                  f"{median(wt):>7.1f}s")
    print("\n" + "=" * 110)
    print("  COMPARISON")
    print("=" * 110)
    print(f"  {'Scenario':<18s} {'Current':>10s} {'Deferred':>10s} {'Diff':>8s} {'Speedup':>8s}")
    print("  " + "-" * 58)
    for sc in scenarios:
        c_ok = [r for r in grouped.get((sc,"current"),[]) if r["success"]]
        d_ok = [r for r in grouped.get((sc,"deferred_30"),[]) if r["success"]]
        cm = median([r["all_time"] for r in c_ok if r["all_time"]]) if c_ok else None
        dm = median([r["all_time"] for r in d_ok if r["all_time"]]) if d_ok else None
        if cm and dm:
            print(f"  {sc:<18s} {cm:>9.1f}s {dm:>9.1f}s {dm-cm:>+7.1f}s {cm/dm:>7.2f}x")
        else:
            print(f"  {sc:<18s} {'n/a':>10s} {'n/a':>10s}")
    print("=" * 110)
    out = os.path.expanduser("~/SMART3D/deferred_benchmark.json")
    with open(out, "w") as f: json.dump({"raw": results}, f)
    print(f"\n  Saved to {out}")

if __name__ == "__main__":
    main()
