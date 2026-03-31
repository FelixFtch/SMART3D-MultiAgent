#!/usr/bin/env python3
"""
Multi-Agent Benchmark: Shared vs Independent Wall Discovery
Tracks per-agent and overall completion times.
"""

import argparse, json, os, re, subprocess, sys, time
from collections import defaultdict
from concurrent.futures import ProcessPoolExecutor, as_completed
from statistics import mean, median

BINARY = os.path.expanduser("~/SMART3D/target/release/examples/smart_multi_agent")

SCENARIOS = {
    "building":      {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "office":        {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "warehouse":     {"rrt_iters": 5000,  "lsr_max": 30, "sensor_range": 10, "timeout": 90},
    "mega_hospital": {"rrt_iters": 30000, "lsr_max": 60, "sensor_range": 12, "timeout": 300},
    "mega_airport":  {"rrt_iters": 50000, "lsr_max": 80, "sensor_range": 12, "timeout": 300},
    "megacampus":    {"rrt_iters": 80000, "lsr_max": 80, "sensor_range": 15, "timeout": 300},
}

MODES = {
    "shared":      [],
    "independent": ["--no-wall-sharing"],
}


def run_trial(args_tuple):
    scenario, scfg, mode_name, mode_flags, trial_id = args_tuple
    cmd = [
        BINARY, "--scenario", scenario,
        "--num-agents", "4", "--agent-mode", "cross",
        "-n", "0", "-o", "1.5", "--playback-speed", "99999",
        "--sensor-range", str(scfg["sensor_range"]),
        "--rrt-iterations", str(scfg["rrt_iters"]),
        "--lsr-max", str(scfg["lsr_max"]),
        "--shortcut-iters", "150", "--detour-threshold", "1.5",
    ] + mode_flags

    t0 = time.monotonic()
    try:
        p = subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE,
                           timeout=scfg["timeout"])
        wall_time = time.monotonic() - t0
        err = p.stderr.decode("utf-8", errors="replace")
    except subprocess.TimeoutExpired:
        return {"scenario": scenario, "mode": mode_name, "trial": trial_id,
                "success": False, "all_time": None, "wall_time": scfg["timeout"],
                "agents": {}, "rebuilds_total": 0, "inc_ok": 0, "inc_fail": 0,
                "n_reached": 0, "n_actual": 0}
    except Exception as e:
        return {"scenario": scenario, "mode": mode_name, "trial": trial_id,
                "success": False, "all_time": None, "wall_time": 0,
                "agents": {}, "rebuilds_total": 0, "inc_ok": 0, "inc_fail": 0,
                "n_reached": 0, "n_actual": 0, "error": str(e)}

    # Parse per-agent GOAL times
    # "  ✓ A0 GOAL REACHED at (29.4,28.7), t=14.4s, rebuilds=0"
    # Note: may have trailing emoji like 👑
    agents = {}
    for line in err.splitlines():
        m = re.search(r'(A\d+) GOAL REACHED at \(([\d.]+),([\d.]+)\), t=([\d.]+)s, rebuilds=(\d+)', line)
        if m:
            agents[m.group(1)] = {
                "time": float(m.group(4)),
                "rebuilds": int(m.group(5)),
            }

    # Parse SUMMARY for status and actual agent count
    # "  A0 [GOAL] pos=(29.4,28.7) dist=1.5m rebuilds=0"
    # "  SUMMARY: building | 2 agents | mode=cross"
    n_actual = 0
    m_sum = re.search(r'SUMMARY:.*?(\d+) agents', err)
    if m_sum:
        n_actual = int(m_sum.group(1))

    for line in err.splitlines():
        m = re.search(r'(A\d+) \[GOAL\].*rebuilds=(\d+)', line)
        if m and m.group(1) not in agents:
            agents[m.group(1)] = {"time": None, "rebuilds": int(m.group(2))}
        m = re.search(r'(A\d+) \[FAIL\].*rebuilds=(\d+)', line)
        if m and m.group(1) not in agents:
            agents[m.group(1)] = {"time": None, "rebuilds": int(m.group(2)), "failed": True}

    # Parse ALL DONE time
    all_time = None
    m_all = re.search(r'ALL AGENTS DONE at t=([\d.]+)s', err)
    if m_all:
        all_time = float(m_all.group(1))

    inc_ok = len(re.findall(r'incremental OK', err))
    inc_fail = len(re.findall(r'incremental failed|ReplanningFailed', err))
    rebuilds_total = sum(a.get("rebuilds", 0) for a in agents.values())

    n_reached = sum(1 for a in agents.values() if a.get("time") is not None)
    success = (n_reached >= n_actual and n_actual > 0) or (all_time is not None)

    return {
        "scenario": scenario, "mode": mode_name, "trial": trial_id,
        "success": success, "all_time": all_time, "wall_time": round(wall_time, 2),
        "agents": agents, "n_reached": n_reached, "n_actual": n_actual,
        "rebuilds_total": rebuilds_total, "inc_ok": inc_ok, "inc_fail": inc_fail,
    }


def pct(data, p):
    s = sorted(data)
    return s[min(int(len(s) * p / 100), len(s) - 1)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--trials", type=int, default=1000)
    parser.add_argument("--jobs", type=int, default=0)
    parser.add_argument("--scenarios", nargs="+", default=None)
    parser.add_argument("--modes", nargs="+", default=None)
    parser.add_argument("--output", default="sharing_ablation.json")
    args = parser.parse_args()

    if not os.path.isfile(BINARY):
        print(f"ERROR: {BINARY} not found"); sys.exit(1)

    scenarios = {k: SCENARIOS[k] for k in (args.scenarios or SCENARIOS.keys()) if k in SCENARIOS}
    modes = {k: MODES[k] for k in (args.modes or MODES.keys()) if k in MODES}
    jobs = args.jobs if args.jobs > 0 else os.cpu_count()
    total = len(scenarios) * len(modes) * args.trials

    print("=" * 90)
    print(f"  Shared vs Independent Wall Discovery | Trials: {args.trials} | Jobs: {jobs} | Total: {total}")
    print(f"  Scenarios: {', '.join(scenarios.keys())}")
    print(f"  Modes: {', '.join(modes.keys())}")
    print("=" * 90)

    tasks = []
    for sc, scfg in scenarios.items():
        for mn, mf in modes.items():
            for t in range(args.trials):
                tasks.append((sc, scfg, mn, mf, t))

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
    for r in results:
        grouped[(r["scenario"], r["mode"])].append(r)

    # ===== Table 1: Overall =====
    print("=" * 130)
    print(f"  TABLE 1: OVERALL ({args.trials} trials, 0 people)")
    print("=" * 130)
    for sc in scenarios:
        print(f"\n  --- {sc} ---")
        print(f"  {'Mode':<14s} {'Success':>10s} {'Rate':>6s} {'AllMed':>8s} {'AllP95':>8s} "
              f"{'A0':>7s} {'A1':>7s} {'A2':>7s} {'A3':>7s} "
              f"{'RB':>5s} {'IncOK':>6s} {'Wall':>7s}")
        print("  " + "-" * 118)
        for mn in modes:
            runs = grouped.get((sc, mn), [])
            ok = [r for r in runs if r["success"]]
            if not ok:
                print(f"  {mn:<14s} {len(ok)}/{len(runs)} {'0.0%':>6s}" + "     n/a" * 9)
                continue
            at = sorted([r["all_time"] for r in ok if r["all_time"]])
            amed = {}
            for a in ["A0","A1","A2","A3"]:
                t_list = [r["agents"][a]["time"] for r in ok if a in r["agents"] and r["agents"][a].get("time")]
                amed[a] = f"{median(t_list):.1f}" if t_list else "n/a"
            wt = [r["wall_time"] for r in ok]
            print(f"  {mn:<14s} {len(ok)}/{len(runs)} {len(ok)/len(runs)*100:>5.1f}% "
                  f"{median(at):>7.1f}s {pct(at,95):>7.1f}s "
                  f"{amed['A0']:>7s} {amed['A1']:>7s} {amed['A2']:>7s} {amed['A3']:>7s} "
                  f"{mean([r['rebuilds_total'] for r in ok]):>5.1f} "
                  f"{mean([r['inc_ok'] for r in ok]):>6.1f} "
                  f"{median(wt):>6.1f}s")

    # ===== Table 2: Per-agent spread =====
    print("\n" + "=" * 130)
    print("  TABLE 2: PER-AGENT SPREAD (median seconds)")
    print("=" * 130)
    print(f"  {'Scenario':<18s} {'Mode':<14s} {'A0':>8s} {'A1':>8s} {'A2':>8s} {'A3':>8s} {'Slow':>8s} {'Fast':>8s} {'Spread':>8s}")
    print("  " + "-" * 100)
    for sc in scenarios:
        for mn in modes:
            ok = [r for r in grouped.get((sc, mn), []) if r["success"]]
            if not ok: continue
            pa = {}
            for a in ["A0","A1","A2","A3"]:
                t_list = [r["agents"][a]["time"] for r in ok if a in r["agents"] and r["agents"][a].get("time")]
                pa[a] = median(t_list) if t_list else None
            vals = [v for v in pa.values() if v]
            if not vals: continue
            sl, fa = max(vals), min(vals)
            def f(v): return f"{v:.1f}" if v else "n/a"
            print(f"  {sc:<18s} {mn:<14s} {f(pa.get('A0')):>8s} {f(pa.get('A1')):>8s} "
                  f"{f(pa.get('A2')):>8s} {f(pa.get('A3')):>8s} {f(sl):>8s} {f(fa):>8s} {f(sl-fa):>8s}")

    # ===== Table 3: Sharing benefit =====
    print("\n" + "=" * 130)
    print("  TABLE 3: SHARING BENEFIT")
    print("=" * 130)
    print(f"  {'Scenario':<18s} {'Shared':>10s} {'Independent':>12s} {'Diff':>8s} {'Speedup':>8s} {'Sh.Rate':>8s} {'In.Rate':>8s}")
    print("  " + "-" * 70)
    for sc in scenarios:
        s_ok = [r for r in grouped.get((sc,"shared"),[]) if r["success"]]
        i_ok = [r for r in grouped.get((sc,"independent"),[]) if r["success"]]
        s_all = grouped.get((sc,"shared"),[])
        i_all = grouped.get((sc,"independent"),[])
        sm = median([r["all_time"] for r in s_ok if r["all_time"]]) if s_ok else None
        im = median([r["all_time"] for r in i_ok if r["all_time"]]) if i_ok else None
        sr = f"{len(s_ok)/len(s_all)*100:.1f}%" if s_all else "n/a"
        ir = f"{len(i_ok)/len(i_all)*100:.1f}%" if i_all else "n/a"
        if sm and im:
            print(f"  {sc:<18s} {sm:>9.1f}s {im:>11.1f}s {im-sm:>+7.1f}s {im/sm:>7.2f}x {sr:>8s} {ir:>8s}")
        else:
            print(f"  {sc:<18s} {'n/a':>10s} {'n/a':>12s} {'':>8s} {'':>8s} {sr:>8s} {ir:>8s}")
    print("=" * 130)

    out_path = os.path.join(os.path.expanduser("~/SMART3D"), args.output)
    with open(out_path, "w") as f:
        json.dump({"raw": results}, f)
    print(f"\n  Saved to {out_path}")


if __name__ == "__main__":
    main()
