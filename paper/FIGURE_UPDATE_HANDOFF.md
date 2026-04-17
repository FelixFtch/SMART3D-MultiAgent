# Benchmark Figure Update Handoff

This note is for the next AI that will edit the LaTeX paper and generate benchmark figures.

## Repo + source freeze

Target repo:
- `/home/felix/SMART3D_github`

Canonical benchmark summary:
- `/home/felix/SMART3D_github/paper/CANONICAL_BENCHMARK_TABLES.md`

Paper to update:
- `/home/felix/SMART3D_github/paper/main.tex`

Raw benchmark sources:
- `/home/felix/SMART3D_github/benchmarks/data/single_agent_benchmark.json`
- `/home/felix/SMART3D_github/benchmarks/data/sharing_ablation.json`
- `/home/felix/SMART3D/final_E3_only.json`
- `/home/felix/SMART3D/final_senior_only.json`

Important caveat:
- `Table 2: Single-agent E3` is valid as a baseline table, but it is **not** a strict same-parameter comparison to the multi-agent tables on the two largest maps.
- Checked-in single-agent script uses smaller RRT budgets on:
  - `mega_airport`: `30K` vs multi-agent `50K`
  - `megacampus`: `50K` vs multi-agent `80K`

## Already-updated tables in main.tex

The following sections in `main.tex` were aligned to the canonical benchmark summary:
- `E3 vs. Senior`
- `Shared vs. Independent`

Before touching those tables again, re-check against:
- `/home/felix/SMART3D_github/paper/CANONICAL_BENCHMARK_TABLES.md`

## What figures to generate

Do **not** generate a standalone success-rate figure unless needed for rebuttal/reviewer response.
Reason: most success values are saturated near 100%, so the more informative plots are time, rebuilds, and fairness.

### Figure 1: E3 vs Senior — median completion time

Recommended caption:
- `E3 vs. Senior on 4-agent navigation tasks. Bars show median team completion time over 1,000 trials per scenario.`

Plot type:
- grouped bar chart

X axis:
- `building`, `office`, `warehouse`, `mega_hospital`, `mega_airport`, `megacampus`

Y axis:
- median sim time in seconds

Series:
- `E3`
- `Senior`

Use these values:

| Scenario | E3 | Senior |
|---|---:|---:|
| building | 23.2 | 27.4 |
| office | 27.5 | 31.2 |
| warehouse | 24.2 | 27.2 |
| mega_hospital | 91.9 | 119.3 |
| mega_airport | 116.2 | 151.7 |
| megacampus | 117.7 | 121.8 |

Main message:
- E3 is consistently faster in task completion, especially on large maps.

### Figure 2: E3 vs Senior — rebuild burden

Recommended caption:
- `E3 vs. Senior on 4-agent navigation tasks. Bars show mean full rebuilds per trial over 1,000 trials per scenario.`

Plot type:
- grouped bar chart

X axis:
- same six scenarios

Y axis:
- mean rebuild count

Series:
- `E3`
- `Senior`

Use these values:

| Scenario | E3 | Senior |
|---|---:|---:|
| building | 3.0 | 15.2 |
| office | 6.8 | 30.2 |
| warehouse | 4.7 | 15.1 |
| mega_hospital | 34.0 | 102.7 |
| mega_airport | 30.9 | 91.8 |
| megacampus | 54.9 | 121.3 |

Main message:
- Senior leaves dirty edges behind and pays with many more rebuilds.

### Figure 3: Shared vs Independent — median completion time

Recommended caption:
- `Shared wall discovery reduces team completion time across all benchmark scenarios. Bars show median team completion time over 1,000 trials per scenario.`

Plot type:
- grouped bar chart

X axis:
- same six scenarios

Y axis:
- median sim time in seconds

Series:
- `Shared`
- `Independent`

Use these values:

| Scenario | Shared | Independent |
|---|---:|---:|
| building | 23.2 | 48.4 |
| office | 27.8 | 33.5 |
| warehouse | 24.3 | 26.9 |
| mega_hospital | 92.8 | 108.5 |
| mega_airport | 115.8 | 121.0 |
| megacampus | 116.8 | 137.4 |

Main message:
- Shared discovery helps everywhere, and dramatically on `building`.

### Figure 4: Shared vs Independent — fairness gap

Recommended caption:
- `Shared wall discovery usually improves per-agent fairness. Bars show the gap between the slowest and fastest agent median completion times within each scenario.`

Plot type:
- grouped bar chart

X axis:
- same six scenarios

Y axis:
- fairness gap in seconds

Series:
- `Shared gap`
- `Independent gap`

Use these values:

| Scenario | Shared gap | Independent gap |
|---|---:|---:|
| building | 1.20 | 1.20 |
| office | 1.40 | 6.10 |
| warehouse | 1.80 | 1.65 |
| mega_hospital | 7.05 | 17.70 |
| mega_airport | 10.45 | 14.70 |
| megacampus | 2.90 | 12.35 |

Main message:
- Fairness improves strongly on `office`, `mega_hospital`, and `megacampus`.
- `warehouse` is basically neutral and should be described honestly.

## Optional figure

### Optional Figure 5: Single-agent scaling

Only add this if the paper needs one more systems/scale plot.

Plot type:
- line plot or bar chart

Use:
- median sim time by scenario for single-agent E3

Values:

| Scenario | Median sim time |
|---|---:|
| building | 20.4 |
| office | 20.6 |
| warehouse | 14.7 |
| mega_hospital | 56.6 |
| mega_airport | 94.4 |
| megacampus | 108.6 |

Why optional:
- This is less central than Figures 1–4 because Table 2 already covers it well.

## What not to plot

Avoid these as main figures:
- standalone success-rate chart for all tables
- incremental OK / incremental fail counts alone
- wall-clock-only comparison as the headline figure

Reason:
- success is mostly saturated
- incremental counts are less interpretable than rebuild burden
- wall-clock is hardware-dependent and better kept in tables/secondary text

## Preferred implementation style

Preferred order:
1. keep the 4 tables in LaTeX
2. add 4 compact benchmark figures
3. if space is tight, keep Figures 1, 2, and 4 first

Preferred rendering:
- `pgfplots` if editing directly in LaTeX
- otherwise generate vector PDF figures and include them from `paper/figures/`

## Short writing guidance

Use wording like:
- `success is saturated`
- `rebuild burden`
- `team completion time`
- `per-agent fairness gap`

Avoid wording like:
- `all scenarios 100% success`
- `fairness always improves`

Because the canonical numbers no longer support those stronger claims.
