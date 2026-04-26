#!/usr/bin/env python3
"""
Budget sweep (100 seeds) — find PBSSection optimal RHCR_NOGOOD_BUDGET.

Sweep:
  budgets: {5000, 10000, 50000, 100000}
  w = 5
  m in {500, 700}
  seeds: 0..99
  PBSSection only (ECBS/PBS do not have this knob)

Total: 4 budgets x 2 m x 100 seeds = 800 runs, parallel=10.

Output:
  benchmark_results_budget100/logs/b{budget}_m{m}_s{seed}.log
  benchmark_results_budget100/per_run.csv
  benchmark_results_budget100/summary.txt
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
OUT_DIR = REPO / "benchmark_results_budget100"
LOG_DIR = OUT_DIR / "logs"

BUDGETS = [5000, 10000, 50000, 100000]
AGENTS = [500, 700]
SEEDS = list(range(100))
PLAN_WINDOW = 5
SIM_TIME = 5000
SIM_WINDOW = 5
MAP = "maps/sorting_map.grid"
PARALLEL = 10
TIMEOUT_SEC = 60 * 60  # 1h per run hard cap


def run_single(args):
    budget, m, seed = args
    tag = f"b{budget}_m{m}_s{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    env = os.environ.copy()
    env["RHCR_SOLVER_MODE"] = "section"
    env["RHCR_NOGOOD_BUDGET"] = str(budget)
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={PLAN_WINDOW}",
        "--solver=PBS",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(m),
        "--screen=1",
    ]
    t0 = time.time()
    try:
        result = subprocess.run(
            cmd, cwd=str(REPO), env=env,
            capture_output=True, text=True, timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return budget, m, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return budget, m, seed, elapsed, -999


def parse_log(budget, m, seed):
    log_file = LOG_DIR / f"b{budget}_m{m}_s{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    prefix = "PBSSection:"
    succ_lines = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    fail_lines = [l for l in content.splitlines()
                  if l.startswith(prefix) and not l.startswith(prefix + "Succeed,")]

    runtimes = []
    hl_exp_total = 0
    for l in succ_lines:
        parts = l.split(",")
        if len(parts) >= 2:
            try: runtimes.append(float(parts[1]))
            except ValueError: pass
        if len(parts) >= 3:
            try: hl_exp_total += int(parts[2])
            except ValueError: pass

    tasks = sum(int(mm.group(1))
                for mm in re.finditer(r"(\d+) tasks has been finished", content))
    errs = len(re.findall(r"Solution invalid|jump from", content))
    timesteps_sim = len(re.findall(r"^Timestep \d+", content, re.M))
    expected_calls = SIM_TIME // SIM_WINDOW

    diag = {}
    for k, pat in [
        ("diag_total_calls",  r"total calls:\s+(\d+)"),
        ("diag_primary_ok",   r"primary success:\s+(\d+)"),
        ("diag_fallback",     r"fallback invocations:\s+(\d+)"),
        ("diag_total_wall",   r"TOTAL solve_by_Section:\s+([\d.]+)\s+s"),
        ("diag_primary_wall", r"primary run_section:\s+([\d.]+)\s+s"),
    ]:
        mm = re.search(pat, content)
        if mm:
            s = mm.group(1)
            try: diag[k] = float(s) if "." in s else int(s)
            except ValueError: diag[k] = 0

    row = {
        "budget": budget,
        "m": m,
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
        "errors": errs,
        "sim_complete": int(errs == 0 and timesteps_sim >= expected_calls),
    }
    row.update(diag)
    return row


def summarize(rows, budget, m):
    sel = [r for r in rows if r["budget"] == budget and r["m"] == m]
    ok = [r for r in sel if r["sim_complete"]]
    s = {"budget": budget, "m": m,
         "n": len(sel), "sim_complete": len(ok),
         "sim_complete_rate": len(ok)/len(sel) if sel else 0}
    keys = [
        "tasks_finished", "throughput_per_step",
        "wall_time_sec", "solver_runtime_sum", "solver_runtime_max",
        "call_fail", "hl_expanded_sum",
        "diag_fallback", "diag_total_wall", "diag_primary_wall",
        "diag_primary_ok",
    ]
    for k in keys:
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

    jobs = [(b, m, s) for b in BUDGETS for m in AGENTS for s in SEEDS]
    # Faster jobs first (small m) to warm up pool
    jobs.sort(key=lambda j: (j[1], j[0], j[2]))

    print(f"Running {len(jobs)} jobs, parallel={PARALLEL}")
    print(f"budgets={BUDGETS}  m={AGENTS}  seeds={len(SEEDS)}  w={PLAN_WINDOW}  sim_time={SIM_TIME}")
    sys.stdout.flush()

    wall = {}
    t_start = time.time()
    prog = OUT_DIR / "progress.log"
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            budget, m, seed, elapsed, rc = f.result()
            wall[(budget, m, seed)] = elapsed
            done += 1
            line = (f"[{done}/{len(jobs)}] b={budget:6d} m={m} s={seed:3d} "
                    f"{elapsed:7.1f}s rc={rc}")
            print(line)
            with prog.open("a") as pf: pf.write(line + "\n")
            sys.stdout.flush()
    total = time.time() - t_start
    print(f"\nAll runs in {total:.1f}s ({total/60:.1f} min = {total/3600:.2f} h)")

    rows = []
    for (b, m, s) in jobs:
        r = parse_log(b, m, s)
        if r:
            r["wall_time_sec"] = wall.get((b, m, s), 0.0)
            rows.append(r)

    all_keys = set()
    for r in rows: all_keys.update(r.keys())
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=sorted(all_keys))
        wt.writeheader()
        wt.writerows(rows)

    summaries = []
    for b in BUDGETS:
        for m in AGENTS:
            sm = summarize(rows, b, m)
            if sm: summaries.append(sm)

    if summaries:
        sfields = sorted({k for s in summaries for k in s.keys()})
        with (OUT_DIR / "summary.csv").open("w", newline="") as f:
            wt = csv.DictWriter(f, fieldnames=sfields)
            wt.writeheader()
            wt.writerows(summaries)

    lines = []
    lines.append(f"Budget sweep (100 seeds): PBSSection only")
    lines.append(f"budgets={BUDGETS}  m={AGENTS}  w={PLAN_WINDOW}  sim_time={SIM_TIME}")
    lines.append(f"Total wall: {total:.1f}s ({total/3600:.2f} h)")
    lines.append("")
    for s in summaries:
        lines.append(f"=== budget={s['budget']} m={s['m']} ===")
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
            ("diag_fallback",      "fallback count"),
            ("diag_primary_ok",    "primary success"),
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
