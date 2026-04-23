#!/usr/bin/env python3
"""
Quick PBS-baseline-improved vs PBSSection time comparison.

Purpose: sanity-check that tangled-tiebreak + fork-heuristic port to PBS baseline
doesn't break anything and to see raw timing delta vs PBSSection.

5 seeds x 2 windows x 2 solvers x m=500 x sim_time=200 (short — just timing).
"""
import csv
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from statistics import mean, median

REPO = Path(__file__).resolve().parent.parent
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_quick_improved"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(5))
WINDOWS = [5, 10]
AGENTS = 500
SIM_TIME = 200
SIM_WINDOW = 5
MAP = "maps/sorting_map.grid"
PARALLEL = 4
TIMEOUT_SEC = 15 * 60

SOLVERS = [
    ("PBS_baseline_improved", "PBS", "baseline", "PBS:"),
    ("PBS_section",           "PBS", "section",  "PBSSection:"),
]


def run_single(args):
    label, solver, mode, _prefix, window, seed = args
    tag = f"{label}_w{window}_seed{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={window}",
        f"--solver={solver}",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(AGENTS),
        "--screen=1",
    ]
    env = os.environ.copy()
    if mode == "baseline":
        env["RHCR_SOLVER_MODE"] = "baseline"
    else:
        env.pop("RHCR_SOLVER_MODE", None)

    t0 = time.time()
    try:
        result = subprocess.run(
            cmd, cwd=str(REPO), env=env,
            capture_output=True, text=True, timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return label, window, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return label, window, seed, elapsed, -999


def parse_log(label, prefix, window, seed):
    log_file = LOG_DIR / f"{label}_w{window}_seed{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    succ_lines = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    fail_lines = [l for l in content.splitlines()
                  if l.startswith(prefix) and not l.startswith(prefix + "Succeed,")]

    runtimes = []
    for l in succ_lines:
        parts = l.split(",")
        if len(parts) >= 2:
            try: runtimes.append(float(parts[1]))
            except ValueError: pass

    tasks = sum(int(m.group(1))
                for m in re.finditer(r"(\d+) tasks has been finished", content))
    errs = len(re.findall(r"Solution invalid|jump from", content))
    timesteps_sim = len(re.findall(r"^Timestep \d+", content, re.M))
    expected_calls = SIM_TIME // SIM_WINDOW

    row = {
        "solver_label": label,
        "window": window,
        "seed": seed,
        "tasks_finished": tasks,
        "throughput_per_step": tasks / SIM_TIME,
        "timesteps_simulated": timesteps_sim,
        "expected_calls": expected_calls,
        "call_total": len(succ_lines) + len(fail_lines),
        "call_succeed": len(succ_lines),
        "call_fail": len(fail_lines),
        "solver_runtime_sum": sum(runtimes),
        "solver_runtime_max": max(runtimes) if runtimes else 0,
        "errors": errs,
        "sim_complete": int(errs == 0 and timesteps_sim >= expected_calls),
    }
    return row


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)

    jobs = []
    for (label, solver, mode, prefix) in SOLVERS:
        for w in WINDOWS:
            for seed in SEEDS:
                jobs.append((label, solver, mode, prefix, w, seed))

    print(f"Running {len(jobs)} jobs, parallel={PARALLEL}")
    sys.stdout.flush()

    wall = {}
    t_start = time.time()
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            label, window, seed, elapsed, rc = f.result()
            wall[(label, window, seed)] = elapsed
            done += 1
            print(f"[{done}/{len(jobs)}] {label:26s} w={window} seed={seed} "
                  f"{elapsed:6.1f}s rc={rc}")
            sys.stdout.flush()
    total = time.time() - t_start
    print(f"\nAll runs in {total:.1f}s\n")

    rows = []
    for (label, _s, _m, prefix, w, seed) in jobs:
        r = parse_log(label, prefix, w, seed)
        if r:
            r["wall_time_sec"] = wall.get((label, w, seed), 0.0)
            rows.append(r)

    for (label, _s, _m, _p) in SOLVERS:
        for w in WINDOWS:
            sel = [r for r in rows if r["solver_label"] == label and r["window"] == w]
            if not sel:
                continue
            print(f"=== {label} w={w} ===")
            print(f"  sim_complete:        {sum(r['sim_complete'] for r in sel)}/{len(sel)}")
            print(f"  tasks_finished:      mean={mean(r['tasks_finished'] for r in sel):7.1f}  "
                  f"median={median(r['tasks_finished'] for r in sel):7.1f}")
            print(f"  throughput/step:     mean={mean(r['throughput_per_step'] for r in sel):7.4f}")
            print(f"  Σ solver runtime:    mean={mean(r['solver_runtime_sum'] for r in sel):7.3f}  "
                  f"median={median(r['solver_runtime_sum'] for r in sel):7.3f}")
            print(f"  max per-call (s):    mean={mean(r['solver_runtime_max'] for r in sel):7.3f}")
            print(f"  per-call fails:      mean={mean(r['call_fail'] for r in sel):7.2f}")
            print(f"  wall end-to-end (s): mean={mean(r['wall_time_sec'] for r in sel):7.1f}")
            print()

    keys = sorted({k for r in rows for k in r.keys()})
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=keys)
        wt.writeheader()
        wt.writerows(rows)
    print(f"Per-run CSV: {OUT_DIR/'per_run.csv'}")


if __name__ == "__main__":
    main()
