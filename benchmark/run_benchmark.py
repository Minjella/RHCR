#!/usr/bin/env python3
"""
Benchmark RHCR section-based PBS vs baseline RHCR-PBS.

100 seeds x 2 modes x simulation_time=1000 x 500 agents.
Captures throughput, computation time, and computation amount (HL/LL node counts).

Outputs:
    benchmark_results/logs/<mode>_<seed>.log   raw stdout+stderr
    benchmark_results/per_seed.csv             aggregated per seed
    benchmark_results/summary.csv              aggregated per mode
    benchmark_results/summary.txt              human-readable summary
"""
import csv
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from statistics import mean, median, stdev

REPO = Path("/Users/minji/Documents/GitHub/RHCR")
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_results"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(1, 101))
MODES = ["section", "baseline"]
SIM_TIME = 1000
SIM_WINDOW = 5
PLAN_WINDOW = 10
AGENTS = 500
MAP = "maps/sorting_map.grid"
PARALLEL = 6
TIMEOUT_SEC = 900

# Column order for PBS:Succeed / PBSSection:Succeed lines
COLS = [
    "runtime", "hl_expanded", "hl_generated", "sol_cost", "min_sum_cost",
    "avg_path_len", "num_collisions", "runtime_plan_paths", "runtime_rt",
    "runtime_get_hp", "runtime_copy_prio", "runtime_detect_conf",
    "runtime_copy_conf", "runtime_choose_conf", "runtime_find_consistent",
    "runtime_find_replan",
]


def run_single(args):
    mode, seed = args
    log_file = LOG_DIR / f"{mode}_{seed}.log"
    env = os.environ.copy()
    env["RHCR_SOLVER_MODE"] = mode

    t0 = time.time()
    try:
        result = subprocess.run(
            [
                str(BINARY), "-m", MAP, "--scenario=SORTING",
                f"--simulation_window={SIM_WINDOW}",
                f"--planning_window={PLAN_WINDOW}",
                "--solver=PBS", f"--simulation_time={SIM_TIME}",
                f"--seed={seed}", "-k", str(AGENTS),
            ],
            cwd=str(REPO), env=env, capture_output=True, text=True,
            timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return mode, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return mode, seed, elapsed, -999


def parse_log(mode, seed):
    log_file = LOG_DIR / f"{mode}_{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    prefix = "PBSSection:" if mode == "section" else "PBS:"
    succeed_lines = [
        l for l in content.splitlines() if l.startswith(prefix + "Succeed,")
    ]

    totals = {c: 0.0 for c in COLS}
    maxvals = {c: 0.0 for c in COLS}
    for line in succeed_lines:
        values = line.split(",")[1:]
        for i, c in enumerate(COLS):
            if i < len(values):
                try:
                    v = float(values[i])
                    totals[c] += v
                    if v > maxvals[c]:
                        maxvals[c] = v
                except ValueError:
                    pass

    tasks_finished = sum(
        int(m.group(1))
        for m in re.finditer(r"(\d+) tasks has been finished", content)
    )
    errors = len(re.findall(r"NO SOLUTION|Solution invalid|jump from", content))
    timesteps_executed = len(succeed_lines)
    expected = SIM_TIME // SIM_WINDOW

    return {
        "mode": mode,
        "seed": seed,
        "tasks_finished": tasks_finished,
        "throughput_per_step": tasks_finished / SIM_TIME,
        "timesteps_executed": timesteps_executed,
        "expected_timesteps": expected,
        "errors": errors,
        "success": int(errors == 0 and timesteps_executed == expected),
        **{f"sum_{k}": v for k, v in totals.items()},
        **{f"max_{k}": v for k, v in maxvals.items()},
    }


def summarize(rows, mode):
    mode_rows = [r for r in rows if r and r["mode"] == mode]
    ok_rows = [r for r in mode_rows if r["success"]]

    s = {
        "mode": mode,
        "n": len(mode_rows),
        "success_count": len(ok_rows),
        "success_rate": len(ok_rows) / len(mode_rows) if mode_rows else 0,
    }

    keys = [
        "tasks_finished", "throughput_per_step",
        "sum_runtime", "sum_hl_expanded", "sum_hl_generated",
        "sum_runtime_detect_conf", "sum_runtime_plan_paths",
        "sum_runtime_find_consistent", "sum_runtime_rt",
        "max_runtime",
    ]
    for k in keys:
        vals = [r[k] for r in ok_rows if k in r]
        if vals:
            s[f"{k}_mean"] = mean(vals)
            s[f"{k}_median"] = median(vals)
            s[f"{k}_std"] = stdev(vals) if len(vals) > 1 else 0.0
            s[f"{k}_min"] = min(vals)
            s[f"{k}_max"] = max(vals)
    return s


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)

    jobs = [(m, s) for m in MODES for s in SEEDS]
    print(f"Running {len(jobs)} simulations, {PARALLEL} in parallel...")
    print(f"  agents={AGENTS} sim_time={SIM_TIME} "
          f"sim_window={SIM_WINDOW} plan_window={PLAN_WINDOW}")
    sys.stdout.flush()

    wall_times = {}  # (mode, seed) -> elapsed
    t_start = time.time()
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            mode, seed, elapsed, rc = f.result()
            wall_times[(mode, seed)] = elapsed
            done += 1
            status = "OK" if rc == 0 else f"rc={rc}"
            print(f"[{done}/{len(jobs)}] {mode:8s} seed={seed:3d} "
                  f"{elapsed:6.1f}s {status}")
            sys.stdout.flush()

    total_wall = time.time() - t_start
    print(f"\nAll runs done in {total_wall:.1f}s "
          f"({total_wall/60:.1f} min)")

    # Parse
    rows = []
    for m, s in jobs:
        r = parse_log(m, s)
        if r:
            r["wall_time_sec"] = wall_times.get((m, s), 0.0)
            rows.append(r)

    # Per-seed CSV
    fields = list(rows[0].keys())
    with (OUT_DIR / "per_seed.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # Summary CSV
    summaries = [summarize(rows, m) for m in MODES]
    sfields = sorted({k for s in summaries for k in s.keys()})
    with (OUT_DIR / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sfields)
        w.writeheader()
        w.writerows(summaries)

    # Human-readable summary
    lines = []
    lines.append(f"Benchmark: {len(SEEDS)} seeds x {len(MODES)} modes")
    lines.append(f"agents={AGENTS} sim_time={SIM_TIME} "
                 f"sim_window={SIM_WINDOW} plan_window={PLAN_WINDOW}")
    lines.append(f"Total wall time: {total_wall:.1f}s ({total_wall/60:.1f} min)")
    lines.append("")
    for s in summaries:
        lines.append(f"=== {s['mode'].upper()} ===")
        lines.append(f"  success:               {s['success_count']}/{s['n']} "
                     f"({s['success_rate']*100:.1f}%)")
        if "tasks_finished_mean" in s:
            lines.append(
                f"  tasks finished (mean): {s['tasks_finished_mean']:.1f} "
                f"(min {s['tasks_finished_min']:.0f}, max {s['tasks_finished_max']:.0f})")
            lines.append(
                f"  throughput/step (mean): {s['throughput_per_step_mean']:.4f}")
            lines.append(
                f"  internal PBS runtime sum: mean={s['sum_runtime_mean']:.2f}s "
                f"median={s['sum_runtime_median']:.2f}s")
            lines.append(
                f"  HL nodes expanded sum:  mean={s['sum_hl_expanded_mean']:.0f}")
            lines.append(
                f"  HL nodes generated sum: mean={s['sum_hl_generated_mean']:.0f}")
            lines.append(
                f"  detect_conflicts sum:   mean={s['sum_runtime_detect_conf_mean']:.2f}s")
            lines.append(
                f"  plan_paths sum:         mean={s['sum_runtime_plan_paths_mean']:.2f}s")
            lines.append(
                f"  find_consistent sum:    mean={s['sum_runtime_find_consistent_mean']:.2f}s")
            lines.append(
                f"  max per-timestep PBS:   mean={s['max_runtime_mean']:.3f}s")
        lines.append("")

    # Head-to-head
    if all("tasks_finished_mean" in s for s in summaries):
        section_s = next(s for s in summaries if s["mode"] == "section")
        baseline_s = next(s for s in summaries if s["mode"] == "baseline")
        lines.append("=== SECTION vs BASELINE ratios (section/baseline) ===")
        for k in ["tasks_finished", "sum_runtime", "sum_hl_expanded",
                  "sum_hl_generated", "sum_runtime_detect_conf",
                  "sum_runtime_plan_paths", "sum_runtime_find_consistent"]:
            sm = section_s.get(f"{k}_mean")
            bm = baseline_s.get(f"{k}_mean")
            if sm and bm:
                lines.append(f"  {k:35s} {sm/bm:.3f}")

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print("\n" + txt)


if __name__ == "__main__":
    main()
