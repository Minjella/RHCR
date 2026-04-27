# Main 3-way Comparison Table (Paper Headline)

Direct extraction from `benchmark_results_wsweep100` (ECBS, PBS+opts, PBS-Section)
and `benchmark_results_ablation_engineering` (PBS_pure). All at m=500, 100 seeds, sim_time=5000.

## Throughput per timestep (mean ± std)

| Solver | w=5 | w=10 | w=100 |
|---|---:|---:|---:|
| ECBS (1.1) | 14.7755 ± 0.0370 | 15.0061 ± 0.0412 | 15.1209 ± 0.0396 |
| PBS_pure | 15.0980 ± 0.0372 | 15.3279 ± 0.0391 | 15.3780 ± 0.0427 |
| PBS⁺ (PBS+opts) | 15.0806 ± 0.0397 | 15.3082 ± 0.0444 | 15.3289 ± 0.0395 |
| **PBS-Section** | 15.0953 ± 0.0386 | 15.2911 ± 0.0420 | 15.1088 ± 1.4548 |

## Σ solver runtime (s)

| Solver | w=5 | w=10 | w=100 |
|---|---:|---:|---:|
| ECBS (1.1) | 25.17 ± 3.17 | 36.17 ± 0.73 | 314.49 ± 4.47 |
| PBS_pure | 168.82 ± 10.84 | 214.07 ± 9.17 | 387.87 ± 15.69 |
| PBS⁺ (PBS+opts) | 159.45 ± 4.55 | 201.90 ± 6.33 | 338.90 ± 9.04 |
| **PBS-Section** | 20.29 ± 0.28 | 31.06 ± 0.47 | 80.70 ± 7.84 |

## End-to-end wall (s)

| Solver | w=5 | w=10 | w=100 |
|---|---:|---:|---:|
| ECBS (1.1) | 28.9 ± 3.2 | 40.2 ± 0.8 | 325.0 ± 17.6 |
| PBS_pure | 227.6 ± 15.1 | 273.8 ± 12.2 | 453.1 ± 19.0 |
| PBS⁺ (PBS+opts) | 216.6 ± 6.3 | 261.4 ± 8.5 | 403.2 ± 11.0 |
| **PBS-Section** | 80.3 ± 2.6 | 92.2 ± 1.9 | 149.5 ± 15.5 |

## sim_complete

| Solver | w=5 | w=10 | w=100 |
|---|---:|---:|---:|
| ECBS (1.1) | 100/100 | 100/100 | 100/100 |
| PBS_pure | 100/100 | 100/100 | 100/100 |
| PBS⁺ (PBS+opts) | 100/100 | 100/100 | 100/100 |
| **PBS-Section** | 100/100 | 100/100 | 98/100 |

## Section gain ratio (PBS⁺ / PBS-Section)

| w=5 | w=10 | w=100 |
|---:|---:|---:|
| **7.86×** | **6.50×** | **4.20×** |
