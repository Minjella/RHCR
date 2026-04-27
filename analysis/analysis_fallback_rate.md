# Fallback Rate Analysis (PBSSection)

Per-call fallback rate = number of times PBSSection's primary section solver
failed and the fallback PBS+opts solver was invoked, divided by total planning calls.
Computed from `diag_fallback / diag_total_calls` across all PBSSection logs.

If primary fallback is rare, the Section abstraction is doing the actual work.
If fallback is frequent, PBSSection effectively becomes PBS+opts with overhead.

## budget (benchmark_results_budget100)

| budget | m | n | fallback / 1000 calls | total wall % | sim_complete |
|---|---|---|---|---|---|
| 5000 | 500 | 100 | 1.72 | — | 100/100 |
| 5000 | 700 | 100 | 18.02 | — | 100/100 |
| 10000 | 500 | 100 | 1.72 | — | 100/100 |
| 10000 | 700 | 100 | 17.85 | — | 100/100 |
| 50000 | 500 | 100 | 1.69 | — | 100/100 |
| 50000 | 700 | 100 | 17.47 | — | 100/100 |
| 100000 | 500 | 100 | 1.69 | — | 100/100 |
| 100000 | 700 | 100 | 17.52 | — | 100/100 |

## wsweep (benchmark_results_wsweep100)

| window | n | fallback / 1000 calls | total wall % | sim_complete |
|---|---|---|---|---|
| 5 | 100 | 1.72 | — | 100/100 |
| 10 | 100 | 0.48 | — | 100/100 |
| 20 | 100 | 0.77 | — | 100/100 |
| 100 | 100 | 1.15 | — | 98/100 |

## msweep (benchmark_results_msweep100)

| window | m | n | fallback / 1000 calls | total wall % | sim_complete |
|---|---|---|---|---|---|
| 5 | 300 | 100 | 0.02 | — | 100/100 |
| 5 | 400 | 100 | 0.24 | — | 100/100 |
| 5 | 500 | 100 | 1.72 | — | 100/100 |
| 5 | 600 | 100 | 5.89 | — | 100/100 |
| 5 | 700 | 100 | 17.85 | — | 100/100 |
| 5 | 800 | 100 | 53.83 | — | 100/100 |
| 10 | 300 | 100 | 0.00 | — | 100/100 |
| 10 | 400 | 100 | 0.11 | — | 100/100 |
| 10 | 500 | 100 | 0.48 | — | 100/100 |
| 10 | 600 | 100 | 2.05 | — | 100/100 |
| 10 | 700 | 100 | 7.19 | — | 98/100 |
| 10 | 800 | 100 | 25.82 | — | 93/100 |
| 100 | 300 | 100 | 0.03 | — | 100/100 |
| 100 | 400 | 100 | 0.13 | — | 100/100 |
| 100 | 500 | 100 | 1.15 | — | 100/100 |
| 100 | 600 | 100 | 5.53 | — | 97/100 |
| 100 | 700 | 100 | 35.32 | — | 84/100 |
| 100 | 800 | 100 | 239.90 | — | 41/100 |

## hsweep (benchmark_results_hsweep100)

| h | n | fallback / 1000 calls | total wall % | sim_complete |
|---|---|---|---|---|
| 1 | 100 | 0.15 | — | 100/100 |
| 3 | 100 | 0.25 | — | 100/100 |
| 5 | 100 | 0.48 | — | 100/100 |
| 7 | 100 | 0.83 | — | 100/100 |
| 10 | 100 | 2.69 | — | 98/100 |
