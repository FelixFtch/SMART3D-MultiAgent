# Canonical Benchmark Tables

Generated: 2026-04-17T15:20:36Z

## Source Freeze

- Table 1: [main.tex](/home/felix/SMART3D_github/paper/main.tex) scenario definitions + benchmark script params in [bench_single_agent.py](/home/felix/SMART3D_github/benchmarks/bench_single_agent.py), [bench_e3_vs_senior.py](/home/felix/SMART3D_github/benchmarks/bench_e3_vs_senior.py), [bench_sharing_ablation.py](/home/felix/SMART3D_github/benchmarks/bench_sharing_ablation.py)
- Table 2: [single_agent_benchmark.json](/home/felix/SMART3D_github/benchmarks/data/single_agent_benchmark.json)
- Table 3: [final_E3_only.json](/home/felix/SMART3D/final_E3_only.json) + [final_senior_only.json](/home/felix/SMART3D/final_senior_only.json)
- Table 4: [sharing_ablation.json](/home/felix/SMART3D_github/benchmarks/data/sharing_ablation.json)

Important: Table 2 is the checked-in single-agent baseline. Its checked-in script uses smaller RRT budgets on the two largest maps than the multi-agent scripts (`mega_airport`: 30K vs 50K, `megacampus`: 50K vs 80K), so do not describe Table 2 as a strict same-parameter match to Tables 3-4.

## Table 1: Scenario Specs

| Scenario | Size | Walls | Multi-agent RRT* iters | Sensor range | LSR max | Agents |
|---|---:|---:|---:|---:|---:|---:|
| building | 32 m | ~41 | 5K | 10 | 30 | 2-4 |
| office | 32 m | ~42 | 5K | 10 | 30 | 2-4 |
| warehouse | 32 m | ~28 | 5K | 10 | 30 | 2-4 |
| mega_hospital | 96 m | ~200 | 30K | 12 | 60 | 4 |
| mega_airport | 160 m | ~250 | 50K | 12 | 80 | 4 |
| megacampus | 128 m | ~300 | 80K | 15 | 80 | 4 |

## Table 2: Single-agent E3

| Scenario | Median sim time | Success rate | Mean rebuilds | Mean incremental OK | Median wall time |
|---|---:|---:|---:|---:|---:|
| building | 20.4s | 100.0% | 0.8 | 8.3 | 0.18s |
| office | 20.6s | 100.0% | 1.1 | 7.4 | 0.14s |
| warehouse | 14.7s | 100.0% | 0.2 | 4.6 | 0.10s |
| mega_hospital | 56.6s | 100.0% | 1.8 | 22.7 | 1.98s |
| mega_airport | 94.4s | 100.0% | 2.5 | 23.2 | 2.33s |
| megacampus | 108.6s | 100.0% | 4.9 | 33.3 | 6.72s |

## Table 3: E3 vs Senior

| Scenario | E3 median sim | Senior median sim | E3 success | Senior success | E3 rebuilds | Senior rebuilds | Speedup (Senior/E3) |
|---|---:|---:|---:|---:|---:|---:|---:|
| building | 23.2s | 27.4s | 100.0% | 100.0% | 3.0 | 15.2 | 1.18x |
| office | 27.5s | 31.2s | 99.6% | 99.5% | 6.8 | 30.2 | 1.13x |
| warehouse | 24.2s | 27.2s | 99.9% | 99.8% | 4.7 | 15.1 | 1.12x |
| mega_hospital | 91.9s | 119.3s | 100.0% | 99.8% | 34.0 | 102.7 | 1.30x |
| mega_airport | 116.2s | 151.7s | 100.0% | 100.0% | 30.9 | 91.8 | 1.31x |
| megacampus | 117.7s | 121.8s | 100.0% | 99.9% | 54.9 | 121.3 | 1.03x |

## Table 4: Shared vs Independent + Fairness

| Scenario | Shared median sim | Independent median sim | Shared success | Independent success | Speedup (Ind/Shared) | Fairness gap shared | Fairness gap independent | Gap reduction |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| building | 23.2s | 48.4s | 100.0% | 99.9% | 2.09x | 1.20s | 1.20s | 1.00x |
| office | 27.8s | 33.5s | 100.0% | 100.0% | 1.21x | 1.40s | 6.10s | 4.36x |
| warehouse | 24.3s | 26.9s | 100.0% | 100.0% | 1.10x | 1.80s | 1.65s | 0.92x |
| mega_hospital | 92.8s | 108.5s | 100.0% | 100.0% | 1.17x | 7.05s | 17.70s | 2.51x |
| mega_airport | 115.8s | 121.0s | 100.0% | 100.0% | 1.04x | 10.45s | 14.70s | 1.41x |
| megacampus | 116.8s | 137.4s | 100.0% | 100.0% | 1.18x | 2.90s | 12.35s | 4.26x |

## Notes

- The fairness gap is the same quantity used in the paper text: compute each agent's median completion time first, then report `slowest - fastest`. This reproduces the `megacampus 12.3s -> 2.9s` style claim.
- Table 3 uses the canonical 1000-trial E3/Senior runs currently stored on i9. Those JSONs are not yet mirrored into `benchmarks/data`, so the markdown records the exact source paths.
- If you want the paper to be internally airtight, the next cleanup step is to import the E3/Senior JSONs into `benchmarks/data/` and update the README summary to reference exactly the same files.

