#!/usr/bin/env python3
"""
Engineering ablation (5a) — PBS_pure (RHCR_PBS_OPTS=0) only.

Reuses w-sweep data for PBS+opts and PBSSection at m=500 (3-way table merge in analysis).
Only runs PBS_pure here.

Sweep:
  PBS_pure × w ∈ {5, 10, 100} × m=500 × 100 seeds = 300 runs
  (RHCR_PBS_OPTS=0 → stock RHCR PBS: earliest-ts only, f_val primary)

Output:
  benchmark_results_ablation_engineering/logs/{label}_w{w}_s{seed}.log
  benchmark_results_ablation_engineering/per_run.csv
  benchmark_results_ablation_engineering/summary.txt
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

REPO = Path(__file__).resolve().parent.parent
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_results_ablation_engineering"
LOG_DIR = OUT_DIR / "logs"

WINDOWS = [5, 10, 100]
AGENTS = 500
SEEDS = list(range(100))
SIM_TIME = 5000
SIM_WINDOW = 5
MAP = "maps/sorting_map.grid"
PARALLEL = 10
TIMEOUT_SEC = 90 * 60

LABEL = "PBS_pure"
PREFIX = "PBS:"


def run_single(args):
    w, seed = args
    tag = f"{LABEL}_w{w}_s{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={w}",
        "--solver=PBS",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(AGENTS),
        "--screen=1",
    ]
    env = os.environ.copy()
    env["RHCR_SOLVER_MODE"] = "baseline"
    env["RHCR_PBS_OPTS"] = "0"  # disable tangled tiebreak + Variant-E fork

    t0 = time.time()
    try:
        result = subprocess.run(
            cmd, cwd=str(REPO), env=env,
            capture_output=True, text=True, timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return w, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return w, seed, elapsed, -999


def parse_log(w, seed):
    log_file = LOG_DIR / f"{LABEL}_w{w}_s{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    succ_lines = [l for l in content.splitlines() if l.startswith(PREFIX + "Succeed,")]
    fail_lines = [l for l in content.splitlines()
                  if l.startswith(PREFIX) and not l.startswith(PREFIX + "Succeed,")]

    runtimes, hl_exp_total, ll_exp_total = [], 0, 0
    for l in succ_lines:
        parts = l.split(",")
        if len(parts) >= 2:
            try: runtimes.append(float(parts[1]))
            except ValueError: pass
        if len(parts) >= 3:
            try: hl_exp_total += int(parts[2])
            except ValueError: pass
        if len(parts) >= 5:
            try: ll_exp_total += int(parts[4])
            except ValueError: pass

    tasks = sum(int(m.group(1))
                for m in re.finditer(r"(\d+) tasks has been finished", content))
    errs = len(re.findall(r"Solution invalid|jump from", content))
    timesteps_sim = len(re.findall(r"^Timestep \d+", content, re.M))
    expected_calls = SIM_TIME // SIM_WINDOW

    row = {
        "solver_label": LABEL,
        "window": w,
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
        "hl_expanded_sum": hl_exp_total,
        "ll_expanded_sum": ll_exp_total,
        "errors": errs,
        "sim_complete": int(errs == 0 and timesteps_sim >= expected_calls),
    }
    return row


def summarize(rows, w):
    sel = [r for r in rows if r["window"] == w]
    ok = [r for r in sel if r["sim_complete"]]
    s = {"solver": LABEL, "window": w,
         "n": len(sel), "sim_complete": len(ok),
         "sim_complete_rate": len(ok)/len(sel) if sel else 0}
    for k in [
        "tasks_finished", "throughput_per_step",
        "wall_time_sec", "solver_runtime_sum", "solver_runtime_max",
        "call_fail", "hl_expanded_sum", "ll_expanded_sum",
    ]:
        vals = [r[k] for r in sel if k in r]
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
    (OUT_DIR / "progress.log").write_text("")

    jobs = [(w, seed) for w in WINDOWS for seed in SEEDS]
    jobs.sort(key=lambda j: (j[0], j[1]))

    print(f"Running {len(jobs)} jobs (PBS_pure only), parallel={PARALLEL}")
    print(f"w={WINDOWS}  m={AGENTS}  seeds={len(SEEDS)}  sim_time={SIM_TIME}  RHCR_PBS_OPTS=0")
    sys.stdout.flush()

    wall = {}
    t_start = time.time()
    prog = OUT_DIR / "progress.log"
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            w, seed, elapsed, rc = f.result()
            wall[(w, seed)] = elapsed
            done += 1
            line = (f"[{done}/{len(jobs)}] PBS_pure w={w:3d} s={seed:3d} "
                    f"{elapsed:7.1f}s rc={rc}")
            print(line)
            with prog.open("a") as pf: pf.write(line + "\n")
            sys.stdout.flush()
    total = time.time() - t_start
    print(f"\nAll runs in {total:.1f}s ({total/3600:.2f} h)")

    rows = []
    for (w, seed) in jobs:
        r = parse_log(w, seed)
        if r:
            r["wall_time_sec"] = wall.get((w, seed), 0.0)
            rows.append(r)

    all_keys = set()
    for r in rows: all_keys.update(r.keys())
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=sorted(all_keys))
        wt.writeheader()
        wt.writerows(rows)

    summaries = []
    for w in WINDOWS:
        sm = summarize(rows, w)
        if sm: summaries.append(sm)

    if summaries:
        sfields = sorted({k for s in summaries for k in s.keys()})
        with (OUT_DIR / "summary.csv").open("w", newline="") as f:
            wt = csv.DictWriter(f, fieldnames=sfields)
            wt.writeheader()
            wt.writerows(summaries)

    lines = []
    lines.append(f"Engineering ablation (100 seeds): PBS_pure (RHCR_PBS_OPTS=0)")
    lines.append(f"w={WINDOWS}  m={AGENTS}  sim_time={SIM_TIME}")
    lines.append(f"Total wall: {total:.1f}s ({total/3600:.2f} h)")
    lines.append("")
    for s in summaries:
        lines.append(f"=== PBS_pure w={s['window']} ===")
        lines.append(f"  sim_complete:    {s['sim_complete']}/{s['n']} "
                     f"({s['sim_complete_rate']*100:.1f}%)")
        for k, lbl in [
            ("tasks_finished",     "tasks finished"),
            ("throughput_per_step","throughput / step"),
            ("wall_time_sec",      "end-to-end wall (s)"),
            ("solver_runtime_sum", "Σ solver runtime (s)"),
            ("solver_runtime_max", "max per-call (s)"),
            ("call_fail",          "per-call fails"),
            ("hl_expanded_sum",    "Σ HL expanded"),
            ("ll_expanded_sum",    "Σ LL expanded"),
        ]:
            mk = f"{k}_mean"
            if mk in s:
                lines.append(
                    f"    {lbl:28s} mean={s[mk]:12.4f}  std={s.get(f'{k}_std',0):10.4f}  "
                    f"min={s.get(f'{k}_min',0):10.4f}  max={s.get(f'{k}_max',0):10.4f}"
                )
        lines.append("")

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print("\n" + txt)


if __name__ == "__main__":
    main()
