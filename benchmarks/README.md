# SMART-3D Benchmark Data And Reproduction Notes

This folder stores the benchmark runners and raw data used for the SMART-3D /
PECMAN paper work.

## What Is Here

Scripts:

- `bench_single_agent.py`: single-agent Lazy Eager baseline.
- `bench_sharing_ablation.py`: Shared vs Independent perception.
- `bench_deferred.py`: current eager repair vs deferred-30m repair.
- `bench_e3_vs_senior.py`: Lazy Eager vs Swift/path-only baseline; filename keeps old internal names.
- `bench_parallel.py`: larger configurable benchmark harness.

Raw data:

- `data/single_agent_benchmark.json`
- `data/sharing_ablation.json`
- `data/deferred_benchmark.json`
- `data/paper_export_2026-04-23/2d_scenarios.json`
- `data/paper_export_2026-04-23/3d_scenarios.json`
- `data/rerun_2026-05-08/2d_scenarios.json`
- `data/rerun_2026-05-08/3d_scenarios.json`

The `paper_export_2026-04-23` files are aggregate exports from the i9 paper-map
batch. The original export directory also contained per-run artifacts; one 3D
per-run JSON was observed truncated, so the aggregate JSONs are the reliable
paper-facing source.

## Scenario Name Mapping

Raw code names stay unchanged in JSON:

| Raw ID | Paper display label |
|--------|---------------------|
| `building` | Building |
| `office` | Office |
| `warehouse` | Warehouse |
| `mega_hospital` | Hospital |
| `mega_airport` | Airport |
| `megacampus` | Campus |
| `mega_university` | University |
| `forest` | Forest |
| `building3d` | Building3D |
| `mega_hive_3d` | Hive3D |
| `mega_parking_3d` | Parking3D |

Do not rename the raw scenario IDs inside the JSON unless every script and
figure generator is updated together. Rename labels only at plot/table time.

Note: the committed `run_3d_scenarios.rs` aggregate runner is not the archived
multi-map 3D RViz producer. Its JSON is grouped by obstacle count and obstacle
speed, not by `forest/building3d/mega_hive_3d/mega_parking_3d` scene IDs. The
3D scene-ID rows above are kept for older paper artifacts and figure labels.

## Known Result Summary

Single-agent Lazy Eager, 1,000 trials each:

- `building`: 1000/1000 success, median 20.4 s.
- `office`: 1000/1000 success, median 20.6 s.
- `warehouse`: 1000/1000 success, median 14.7 s.
- `mega_hospital`: 1000/1000 success, median 56.6 s.
- `mega_airport`: 1000/1000 success, median 94.4 s.
- `megacampus`: 1000/1000 success, median 108.6 s.

Shared vs Independent, 1,000 trials each:

- Shared is faster on all six stored 2D scenarios.
- Median all-agent time examples:
  - `building`: 23.2 s shared vs 48.4 s independent.
  - `mega_hospital`: 92.8 s shared vs 108.5 s independent.
  - `megacampus`: 116.8 s shared vs 137.4 s independent.

2026-04-23 paper-map export:

- 2D sweep: 1600 trials, 1492 successes, all-trials median travel time 11.1 s
  and successful-trials median travel time 11.2 s.
- 3D sweep: 1600 trials, 1513 successes, median travel time 15.4 s.

2026-05-08 i9 rerun copied into this repo:

- 2D sweep: 1600 trials, 1496 successes, median successful travel time 11.2 s.
- 3D sweep: 1600 trials, 1520 successes, median successful travel time 15.4 s.
- i9 source outputs:
  - `/home/felix/SMART3D_benchmark_reruns/2d_20260508_final_20260508_162339`
  - `/home/felix/SMART3D_benchmark_reruns/3d_20260508_final_20260508_162657`
- Only aggregate JSON files are committed. The per-run `.bin` files stay on the
  i9 because they are bulky regression artifacts, not README-friendly source.
- The per-run artifact paths inside the aggregate JSONs point to the machine and
  output directory used during that run. Treat those paths as provenance, not as
  guaranteed local files after cloning this repository.

## Re-run Commands

Run from the i9 repo root, normally `/home/felix/SMART3D` or
`/home/felix/SMART3D_capture_core_clean`:

```bash
source ~/.cargo/env 2>/dev/null || true
```

Legacy Python benchmark runners:

```bash
cd /home/felix/SMART3D
python3 benchmarks/bench_single_agent.py --trials 1000 --jobs 0 --output single_agent_benchmark.json
python3 benchmarks/bench_sharing_ablation.py --trials 1000 --jobs 0 --output sharing_ablation.json
python3 benchmarks/bench_deferred.py --trials 1000 --jobs 0
python3 benchmarks/bench_e3_vs_senior.py --trials 1000 --scenarios building office warehouse mega_hospital mega_airport megacampus
python3 benchmarks/bench_parallel.py --trials 1000 --jobs 0
```

Current Rust 2D/3D aggregate runners:

```bash
cd /home/felix/SMART3D_capture_core_clean
mkdir -p /home/felix/SMART3D_benchmark_reruns

cargo run --release --example run_2d_scenarios -- \
  --output-dir /home/felix/SMART3D_benchmark_reruns/2d_$(date +%Y%m%d_%H%M%S)

cargo run --release --example run_3d_scenarios -- \
  --output-dir /home/felix/SMART3D_benchmark_reruns/3d_$(date +%Y%m%d_%H%M%S)
```

Validation note: `run_2d_scenarios` and `run_3d_scenarios` both accept only
`--output-dir` and `--help` for changing where aggregate outputs go; the 2D
runner also has `--save-tree` for bulky debug artifacts. Neither runner exposes
a trial-count flag. Use the Python benchmark scripts when you need custom trial
counts.

Runtime note: these Rust aggregate runners are full fixed-size sweeps, not quick
smoke tests. On the i9 fresh-clone verification pass, `run_3d_scenarios` needed
a multi-minute timeout window and completed by writing `3d_scenarios.json`.
Do not add `--trials`; that option belongs to the Python benchmark scripts.

## Paper Naming Notes

Use paper-facing names in captions and README prose:

- Use `Lazy Eager (LE)` instead of `E3` when matching the current paper text.
- Use `Swift` instead of `Senior`.
- Keep an internal note that older scripts may still mention `E3` or `Senior`.
- Use `Hospital`, `Airport`, `Campus`, `University` in charts; keep raw
  `mega_*` IDs in code and data.
- If `paper/main.tex` in this repository still contains an older naming pass,
  prefer the current README terminology for new handoff/deployment prose rather
  than editing benchmark JSON or script filenames.
