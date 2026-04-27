#!/usr/bin/env python3
"""
h-sweep (100 seeds) — sensitivity to simulation_window (replan frequency).

Sweep:
  h (simulation_window) ∈ {1, 3, 5, 7, 10}
  solvers: ECBS(1.1), PBS+opts (baseline mode), PBSSection (BEST_BUDGET=10000)
  m = 500, w (planning_window) = 10
  seeds: 0..99

Total: 3 × 5 × 100 = 1500 jobs

Output:
  benchmark_results_hsweep100/logs/{label}_h{h}_s{seed}.log
  benchmark_results_hsweep100/per_run.csv, summary.txt
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
OUT_DIR = REPO / "benchmark_results_hsweep100"
LOG_DIR = OUT_DIR / "logs"

BEST_BUDGET = 10000

H_VALUES = [1, 3, 5, 7, 10]   # simulation_window
PLAN_WINDOW = 10              # fixed best PBSSection w
AGENTS = 500
SEEDS = list(range(100))
SIM_TIME = 5000
MAP = "maps/sorting_map.grid"
SUBOPT = 1.1
PARALLEL = 10
TIMEOUT_SEC = 90 * 60

SOLVERS = [
    ("ECBS_baseline", "ECBS", "baseline", "ECBS:"),
    ("PBS_opts",      "PBS",  "baseline", "PBS:"),
    ("PBS_section",   "PBS",  "section",  "PBSSection:"),
]


def run_single(args):
    label, solver, mode, _prefix, h, seed = args
    tag = f"{label}_h{h}_s{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={h}",
        f"--planning_window={PLAN_WINDOW}",
        f"--solver={solver}",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(AGENTS),
        "--screen=1",
    ]
    if solver == "ECBS":
        cmd.append(f"--suboptimal_bound={SUBOPT}")

    env = os.environ.copy()
    if mode == "baseline":
        env["RHCR_SOLVER_MODE"] = "baseline"
    else:
        env["RHCR_SOLVER_MODE"] = "section"
        env["RHCR_NOGOOD_BUDGET"] = str(BEST_BUDGET)

    t0 = time.time()
    try:
        result = subprocess.run(
            cmd, cwd=str(REPO), env=env,
            capture_output=True, text=True, timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return label, h, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return label, h, seed, elapsed, -999


def parse_log(label, prefix, h, seed):
    log_file = LOG_DIR / f"{label}_h{h}_s{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    succ_lines = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    fail_lines = [l for l in content.splitlines()
                  if l.startswith(prefix) and not l.startswith(prefix + "Succeed,")]

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
    expected_calls = SIM_TIME // h

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
        "solver_label": label,
        "h": h,
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
    row.update(diag)
    return row


def summarize(rows, label, h):
    sel = [r for r in rows if r["solver_label"] == label and r["h"] == h]
    ok = [r for r in sel if r["sim_complete"]]
    s = {"solver": label, "h": h,
         "n": len(sel), "sim_complete": len(ok),
         "sim_complete_rate": len(ok)/len(sel) if sel else 0}
    for k in [
        "tasks_finished", "throughput_per_step",
        "wall_time_sec", "solver_runtime_sum", "solver_runtime_max",
        "call_fail", "hl_expanded_sum", "ll_expanded_sum",
        "diag_fallback",
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

    jobs = []
    for (label, solver, mode, prefix) in SOLVERS:
        for h in H_VALUES:
            for seed in SEEDS:
                jobs.append((label, solver, mode, prefix, h, seed))
    # PBSSection first per (h,seed) so we get fast jobs early. Within solver, larger h first
    # (cheaper per run) — actually smaller h means more calls per run, slower. So sort by larger h first.
    order = {"PBS_section": 0, "ECBS_baseline": 1, "PBS_opts": 2}
    jobs.sort(key=lambda j: (-j[4], order[j[0]], j[5]))

    print(f"Running {len(jobs)} jobs, parallel={PARALLEL}")
    print(f"h={H_VALUES}  m={AGENTS}  w={PLAN_WINDOW}  seeds={len(SEEDS)}  sim_time={SIM_TIME}  BEST_BUDGET={BEST_BUDGET}")
    sys.stdout.flush()

    wall = {}
    t_start = time.time()
    prog = OUT_DIR / "progress.log"
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            label, h, seed, elapsed, rc = f.result()
            wall[(label, h, seed)] = elapsed
            done += 1
            line = (f"[{done}/{len(jobs)}] {label:14s} h={h:2d} s={seed:3d} "
                    f"{elapsed:7.1f}s rc={rc}")
            print(line)
            with prog.open("a") as pf: pf.write(line + "\n")
            sys.stdout.flush()
    total = time.time() - t_start
    print(f"\nAll runs in {total:.1f}s ({total/3600:.2f} h)")

    rows = []
    for (label, _s, _m, prefix, h, seed) in jobs:
        r = parse_log(label, prefix, h, seed)
        if r:
            r["wall_time_sec"] = wall.get((label, h, seed), 0.0)
            rows.append(r)

    all_keys = set()
    for r in rows: all_keys.update(r.keys())
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=sorted(all_keys))
        wt.writeheader()
        wt.writerows(rows)

    summaries = []
    for (label, _s, _m, _p) in SOLVERS:
        for h in H_VALUES:
            sm = summarize(rows, label, h)
            if sm: summaries.append(sm)

    if summaries:
        sfields = sorted({k for s in summaries for k in s.keys()})
        with (OUT_DIR / "summary.csv").open("w", newline="") as f:
            wt = csv.DictWriter(f, fieldnames=sfields)
            wt.writeheader()
            wt.writerows(summaries)

    lines = []
    lines.append(f"h-sweep (100 seeds): 3 solvers x {len(H_VALUES)} h values")
    lines.append(f"h={H_VALUES}  m={AGENTS}  w={PLAN_WINDOW}  sim_time={SIM_TIME}  subopt(ECBS)={SUBOPT}")
    lines.append(f"Total wall: {total:.1f}s ({total/3600:.2f} h)")
    lines.append("")
    for s in summaries:
        lines.append(f"=== {s['solver']} h={s['h']} ===")
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
            ("diag_fallback",      "fallback count"),
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
