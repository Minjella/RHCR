# ECBSSection Asymmetry — Section abstraction × HL strategy

## Setup

This experiment quantifies the asymmetric interaction between the Section abstraction
and the high-level (HL) search strategy. PBSSection uses **priority-based** HL search;
ECBSSection uses **constraint-based** HL search. We test whether the Section abstraction
that accelerates PBS also accelerates ECBS, by comparing:

- **ECBS_baseline** — stock ECBS (suboptimal_bound = 1.1)
- **ECBSSection** — ECBS extended with the Section abstraction (`RHCR_SOLVER_MODE=section`)

Both at m=500, w ∈ {5, 10}, 100 seeds, sim_time=5000, parallel=10. Total wall: 0.94 h.

## Headline result

| Metric | w=5 ECBS_baseline | w=5 **ECBSSection** | w=10 ECBS_baseline | w=10 **ECBSSection** |
|---|---:|---:|---:|---:|
| sim_complete | 100/100 | **100/100** | 100/100 | **100/100** |
| Throughput | 14.78 | 14.95 (+1.1 %) | 15.01 | 15.10 (+0.6 %) |
| **Σ solver runtime (s)** | 24.86 | 15.05 | 36.38 | 19.60 |
| **End-to-end wall (s)** | **28.60** | **107.67** | **40.43** | **158.18** |
| **Fallback rate / 1000** | n/a | **46.62** | n/a | **104.87** |
| HL expanded | 90 281 | 168 062 (1.86 ×) | 81 266 | 246 644 (3.04 ×) |
| LL expanded | 20.6 M | 8.6 M | 21.0 M | 10.1 M |

## Interpretation

ECBSSection **looks** faster than ECBS_baseline if we read only the primary-solver
runtime: 15 s vs 25 s at w=5. But this is a misleading metric — the primary section
solver in ECBSSection achieves its speed by **giving up early**, after which the
fallback path (which re-runs full ECBS at the cell level) does most of the actual work:

- **Wall-clock**: ECBSSection takes **3.8×** longer end-to-end at w=5 (107.67 s vs 28.60 s)
  and **3.9×** longer at w=10 (158.18 s vs 40.43 s).
- **Fallback rate**: ECBSSection's primary section solver fails on **4.66 % of planning
  calls at w=5 and 10.49 % at w=10**, requiring the fallback ECBS path to re-solve.
- **HL expansions**: nearly doubled at w=5, tripled at w=10 — adding section-level
  constraints inflates the constraint tree size in ECBS rather than reducing it.

## Comparison to PBSSection (the asymmetry)

At the same (m=500, w) configurations:

| Metric | PBSSection w=5 | ECBSSection w=5 | PBSSection w=10 | ECBSSection w=10 |
|---|---:|---:|---:|---:|
| Wall (s) | **80.7** | 107.7 | **92.2** | 158.2 |
| Σ solver runtime (s) | **20.3** | 15.0 | **31.1** | 19.6 |
| **Fallback rate / 1000** | **1.72** | 46.62 | **0.48** | 104.87 |

The Section abstraction's interaction with HL strategy is **highly asymmetric**:

- **PBS-side**: Section abstraction reduces both primary-solver runtime (5–8×) and end-to-end wall (~3×). Fallback is rare (0.05–0.2 %), so the primary section solver is doing the actual work.
- **ECBS-side**: Section abstraction inflates HL expansions (2–3×) and end-to-end wall (~4×). Fallback fires often (5–10 %), so the section solver is mostly a fast-fail filter that triggers full re-solves.

**The fallback rate ratio (PBSSection : ECBSSection) is 27× at w=5 and 210× at w=10.**

## Why the asymmetry — proposed mechanism

The Section abstraction provides a coarser-than-cell representation of state. PBS's HL
search operates on **agent priorities**, which are orthogonal to the spatial granularity
of the abstraction — a priority edge between two agents is the same regardless of whether
their conflict is identified at section or cell granularity.

ECBS's HL search, by contrast, operates on **constraints over agent-position-time
triples**. Section-entry constraints are coarser than cell-level constraints — they
prune larger sets of LL plans, but cannot distinguish between cell-level alternatives
inside a section. This causes the CT search to oscillate: a section-entry constraint
that resolves the surface conflict may not resolve the underlying cell-level conflict,
forcing many re-expansions and ultimately falling back to ECBS_baseline (which uses
cell-level constraints).

This is consistent with diagnostic data from earlier ECBSSection development sessions:
the primary solver routinely reaches the per-call iteration cap with `min_collisions`
stuck at a non-zero plateau, whereas PBSSection's primary almost always converges on
first attempt or via priority-edge addition.

## Decision implications for paper

This result is **not a failure mode to hide**; it's a **first-class finding** worth a
short subsection in the main body. It shows that:

1. **Our claim is specific to PBS** ("Section abstraction accelerates PBS-style search");
2. **Section abstraction is not a universal solver-agnostic improvement** — it interacts
   with the HL strategy.
3. This **delineates the contribution clearly** for future work: a constraint-based
   variant would need a different abstraction (e.g., per-agent-disjoint constraints)
   to benefit from coarse-grained state representations.

## Paper-text passage (drop-in)

### Discussion / Limitations subsection

> *Section × HL interaction is asymmetric.* We tested whether the Section abstraction
> that accelerates PBS-Section also accelerates ECBS. At m=500 over 100 seeds, ECBS-Section
> (i.e., ECBS with the same section conversion and section-entry constraints) is 3.8 – 3.9×
> *slower* in end-to-end wall than ECBS_baseline, with the primary section solver failing
> on 5 – 11 % of planning calls (vs 0.05 – 0.17 % for PBS-Section). We attribute this to the
> precision mismatch between section-entry constraints and ECBS's cell-level CT search:
> a coarse constraint that prunes a section may not resolve the underlying cell-level
> conflict, leading the CT search to thrash. PBS's priority-edge HL search, by contrast,
> is orthogonal to the spatial granularity of constraints. The Section abstraction's
> benefit is therefore specific to priority-based HL search, not constraint-based.

## One-line summary for paper

> *"Section abstraction provides 5–8× speedup for priority-based PBS but **slows** constraint-based ECBS by ~4× — an asymmetry that we attribute to the granularity mismatch between section-entry constraints and ECBS's cell-level CT search."*
