#!/usr/bin/env python3
"""Budget sweep: vary RHCR_NOGOOD_BUDGET at high-density m values.

Design (Plan E):
- Budgets ∈ {5000, 10000, 25000} × m ∈ {700, 800} × 30 seeds (section only)
- + m=900 × budget=10000 × 30 seeds (pilot at the extreme)
- All at w=10 (problem region where 50k budget causes 0.48× / 0.20× honest speedup)

Baseline (PBS) is unaffected by this env var — reuse existing m-sweep baseline data.

Output: benchmark_results_budgetsweep/logs/<budget>_<mode>_m<m>_s<seed>.log
"""
import csv
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

REPO = Path("/Users/minji/Documents/GitHub/RHCR")
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_results_budgetsweep"
LOG_DIR = OUT_DIR / "logs"

# (m, budget, seed_count) tuples. All section-only, w=10.
JOBS_SPEC = []
for budget in [5000, 10000, 25000]:
    for m in [700, 800]:
        for seed in range(1, 31):
            JOBS_SPEC.append((budget, m, seed))
# m=900 pilot at budget=10000 only
for seed in range(1, 31):
    JOBS_SPEC.append((10000, 900, seed))

SIM_TIME = 5000
SIM_WINDOW = 5
PLAN_WINDOW = 10  # w=10
MAP = "maps/sorting_map.grid"
PARALLEL = 10
TIMEOUT_SEC = 36000  # 10 hours hard cap per run


def run_single(args):
    budget, m, seed = args
    log_file = LOG_DIR / f"b{budget}_section_m{m}_s{seed}.log"
    env = os.environ.copy()
    env["RHCR_SOLVER_MODE"] = "section"
    env["RHCR_NOGOOD_BUDGET"] = str(budget)

    t0 = time.time()
    try:
        result = subprocess.run(
            [
                str(BINARY), "-m", MAP, "--scenario=SORTING",
                f"--simulation_window={SIM_WINDOW}",
                f"--planning_window={PLAN_WINDOW}",
                "--solver=PBS", f"--simulation_time={SIM_TIME}",
                f"--seed={seed}", "-k", str(m),
            ],
            cwd=str(REPO), env=env, capture_output=True, text=True,
            timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return budget, m, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return budget, m, seed, elapsed, -999


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)

    print(f"Running {len(JOBS_SPEC)} simulations, {PARALLEL} in parallel...")
    print(f"  sim_time={SIM_TIME} sim_window={SIM_WINDOW} plan_window={PLAN_WINDOW}")
    print(f"  Budgets: {sorted({b for b,_,_ in JOBS_SPEC})}")
    print(f"  (m, n_seeds): {sorted({(m, 30) for _, m, _ in JOBS_SPEC})}")
    sys.stdout.flush()

    t_start = time.time()
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in JOBS_SPEC}
        done = 0
        for f in as_completed(futures):
            budget, m, seed, elapsed, rc = f.result()
            done += 1
            status = "OK" if rc == 0 else f"rc={rc}"
            if done % 10 == 0 or done == len(JOBS_SPEC):
                print(f"[{done}/{len(JOBS_SPEC)}] b={budget:>5d} m={m:>3d} seed={seed:>3d}"
                      f" {elapsed:>7.1f}s {status}")
                sys.stdout.flush()

    total_wall = time.time() - t_start
    print(f"\nAll runs done in {total_wall:.1f}s ({total_wall/60:.1f} min)")


if __name__ == "__main__":
    main()
