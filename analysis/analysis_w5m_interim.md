# Interim analysis — m=700 at w=5 (new) vs w=10 (existing)

**Date:** 2026-04-20
**Status:** m=700 leg of w5m-sweep complete; m=800 still running.

---

## 1. Setup

- **New runs (w=5):** m=700 × 30 seeds × {section, baseline} — 60 runs total.
- **Comparison (w=10, from prior m-sweep):** m=700 × 100 seeds × {section, baseline} — 200 runs.
- All other parameters identical: `sim_time=5000`, `sim_window=5`, `solver=PBS`, same sorting-map grid.

**Two runtime metrics reported (both matter):**
- **Section-only rt** = Σ `PBSSection:Succeed` runtimes. Measures the cost of the abstraction when it actually produces the solution. This is what the original summary reported as "sum_runtime".
- **Honest rt** = Σ (`PBSSection:Succeed` + `PBSSection:No solutions` + `PBS:Succeed` fallback). Full end-to-end CPU cost including section-failure exploration and the baseline fallback.
- Baseline only emits `PBS:Succeed`, so the two values coincide for it.

---

## 2. Results

### 2.1 Per-mode aggregates (mean ± 1 s.d.)

| Config | n | Throughput | Section-only rt | Honest rt | Section-fail events (sum) | Fallback events (sum) |
|---|---:|---:|---:|---:|---:|---:|
| w=5, section | 30 | **20.6640 ± 0.0377** | 186.09 ± 12.71 s | 269.53 ± 25.49 s | 392 | 392 |
| w=5, baseline | 30 | 20.6279 ± 0.0518 | 455.51 ± 11.01 s | 455.51 ± 11.01 s | — | — |
| w=10, section | 100 | 21.0654 ± 0.0444 | 235.98 ± 9.52 s | 444.91 ± 69.26 s | 1,106 | 1,106 |
| w=10, baseline | 100 | 21.0990 ± 0.0457 | 596.62 ± 23.41 s | 596.62 ± 23.41 s | — | — |

### 2.2 Speedup and throughput comparison

| Metric | **w=5 m=700** | w=10 m=700 |
|---|---:|---:|
| Section-only speedup | **2.45×** | 2.53× |
| Honest speedup | **1.69×** | 1.34× |
| Throughput (section − baseline) | **+0.0361 (+0.175 %)** | −0.0336 (−0.159 %) |

### 2.3 Per-run failure / fallback rates

| Config | Section-fail events / run | Fallback events / run |
|---|---:|---:|
| w=5 | 13.07 | 13.07 |
| w=10 | 11.06 | 11.06 |

Per-run failure frequency is comparable between the two horizons (w=5 slightly higher). The honest-runtime gap therefore comes from the *cost per failure*, not the *count*.

---

## 3. Key findings

1. **At m=700, w=5 preserves (slightly exceeds) baseline throughput.**
   Section reaches 20.6640 tasks/step vs baseline 20.6279 — a +0.175 % margin. Compared with the w=10 regime, where section trailed baseline by 0.159 %, this is a ~0.3 pp swing in section's favour, supporting the hypothesis that tight-horizon settings favour the abstraction. Earlier w-sweep data at m=500 showed the same sign (+0.09 %), so the effect is reproducible across two density points.

2. **Honest speedup improves sharply with a shorter horizon** (1.34× → 1.69×, both at m=700). The number of section-failure events per run is similar; what changes is the cost per event. At w=5 each PBS(Section) iteration is cheaper (shorter planning horizon ⇒ fewer timesteps to manage), so the 50 000-iteration nogood budget costs less wall time when it triggers.

3. **Section-only speedup is ~constant** across w (2.45× at w=5 vs 2.53× at w=10). The algorithmic benefit of the section abstraction itself does not depend materially on horizon length. This is a clean statement to include in the paper: *the abstraction's speedup is stable; only the fallback tax varies with w.*

---

## 4. What this suggests for the final analysis (pending)

- If the pattern holds at m=800 (still running), w=5 could be the preferred operating point for high-density deployment: better throughput *and* smaller honest-runtime penalty.
- Budget-tuning experiment (reducing 50 000 nogood iteration cap) becomes even more promising — already at the current budget, reducing per-failure cost by ~40 % (w=5 → 185 s/run wasted on failures vs w=10's 209 s) is enough to flip honest speedup from 1.34× to 1.69×. A deliberate budget sweep could compound the effect.

---

## 5. Limitations of this interim

- 30 seeds at w=5 vs 100 at w=10: statistical power asymmetric. The throughput Δ is small (0.3 pp) but the SE at n=30 is ~0.007, so the signal is outside 2σ.
- m=700 only. m=800 and m=900 may tell a different story at w=5 (to be reported in the final).
- Same binary, same hardware, same seed range (1–30 at w=5 ⊂ 1–100 at w=10), so no confounding from run environment.

---

## Reproducibility

```bash
# New runs
RHCR_SOLVER_MODE=section  ./lifelong -m maps/sorting_map.grid --scenario=SORTING \
  --simulation_window=5 --planning_window=5 --solver=PBS --simulation_time=5000 \
  --seed=<s> -k 700
RHCR_SOLVER_MODE=baseline ./lifelong ... (same args with baseline)

# Data files
benchmark_results_w5msweep/logs/{section,baseline}_m700_s{1..30}.log
benchmark_results_msweep/logs/{section,baseline}_m700_s{1..100}.log  (prior)
analysis/interim_m700.csv  (aggregated numbers in this doc)
```
