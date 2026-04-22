#!/usr/bin/env python3
"""
Full 3-way comparison at sim_time=5000.

Solvers:
  ECBS_baseline  : --solver=ECBS, RHCR_SOLVER_MODE=baseline, subopt=1.1
  PBS_baseline   : --solver=PBS,  RHCR_SOLVER_MODE=baseline
  PBS_section    : --solver=PBS   (default section mode)

Grid:
  planning_window : 5, 10
  simulation_window (h) : 5  (fixed)
  agents : 500, 600
  seeds  : 0..9 (10)
  simulation_time : 5000
  ECBS suboptimal_bound : 1.1

Total: 3 × 2 × 2 × 10 = 120 runs.

Per-call failure counted: lines like 'SOLVER:Timeout/No solutions/Nodesout'.
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
OUT_DIR = REPO / "benchmark_results_full"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(10))
SIM_TIME = 5000
SIM_WINDOW = 5
WINDOWS = [5, 10]
AGENTS_LIST = [500, 600]
MAP = "maps/sorting_map.grid"
SUBOPT = 1.1
PARALLEL = 4
TIMEOUT_SEC = 45 * 60  # 45 min safety cap per run

# (label, solver_flag, mode, prefix_in_log)
SOLVERS = [
    ("ECBS_baseline", "ECBS", "baseline", "ECBS:"),
    ("PBS_baseline",  "PBS",  "baseline", "PBS:"),
    ("PBS_section",   "PBS",  "section",  "PBSSection:"),
]


def run_single(args):
    label, solver, mode, _prefix, window, agents, seed = args
    tag = f"{label}_w{window}_m{agents}_seed{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={window}",
        f"--solver={solver}",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(agents),
        "--screen=1",
    ]
    if solver == "ECBS":
        cmd.append(f"--suboptimal_bound={SUBOPT}")
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
        return label, window, agents, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return label, window, agents, seed, elapsed, -999


def parse_log(label, solver, mode, prefix, window, agents, seed):
    log_file = LOG_DIR / f"{label}_w{window}_m{agents}_seed{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    succ_lines = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    # Per-call failure variants
    fail_pats = ["Timeout,", "No solutions,", "Nodesout,"]
    fail_lines = [l for l in content.splitlines()
                  if any(l.startswith(prefix + p) for p in fail_pats)]

    # Solver internal runtime (col 1 after prefix+status)
    runtimes = []
    hl_exp_total = 0
    for l in succ_lines:
        parts = l.split(",")
        if len(parts) >= 2:
            try:
                runtimes.append(float(parts[1]))
            except ValueError:
                pass
        # HL expanded: col after runtime for both PBS and ECBS prefixes.
        if len(parts) >= 3:
            try:
                hl_exp_total += int(parts[2])
            except ValueError:
                pass

    tasks = sum(int(m.group(1))
                for m in re.finditer(r"(\d+) tasks has been finished", content))
    errs = len(re.findall(r"Solution invalid|jump from", content))
    timesteps_sim = len(re.findall(r"^Timestep \d+", content, re.M))
    expected_calls = SIM_TIME // SIM_WINDOW  # planner calls

    # [DIAG] section mode only
    diag = {}
    for k, pat in [
        ("diag_total_calls",      r"total calls:\s+(\d+)"),
        ("diag_primary_ok",       r"primary success:\s+(\d+)"),
        ("diag_fallback",         r"fallback invocations:\s+(\d+)"),
        ("diag_total_wall",       r"TOTAL solve_by_Section:\s+([\d.]+)\s+s"),
        ("diag_primary_wall",     r"primary run_section:\s+([\d.]+)\s+s"),
    ]:
        m = re.search(pat, content)
        if m:
            s = m.group(1)
            try:
                diag[k] = float(s) if "." in s else int(s)
            except ValueError:
                diag[k] = 0

    total_calls = len(succ_lines) + len(fail_lines)
    row = {
        "solver_label": label,
        "solver": solver,
        "mode": mode,
        "window": window,
        "agents": agents,
        "seed": seed,
        "tasks_finished": tasks,
        "throughput_per_step": tasks / SIM_TIME,
        "timesteps_simulated": timesteps_sim,
        "expected_calls": expected_calls,
        "call_total": total_calls,
        "call_succeed": len(succ_lines),
        "call_fail": len(fail_lines),
        "call_fail_rate": len(fail_lines) / total_calls if total_calls else 0,
        "solver_runtime_sum": sum(runtimes),
        "solver_runtime_mean": mean(runtimes) if runtimes else 0,
        "solver_runtime_max": max(runtimes) if runtimes else 0,
        "hl_expanded_sum": hl_exp_total,
        "errors": errs,
        "sim_complete": int(errs == 0 and timesteps_sim >= expected_calls),
    }
    row.update(diag)
    return row


def summarize(rows, label, window, agents):
    sel = [r for r in rows if r["solver_label"] == label
           and r["window"] == window and r["agents"] == agents]
    if not sel:
        return None
    ok = [r for r in sel if r["sim_complete"]]
    s = {
        "solver": label,
        "window": window,
        "agents": agents,
        "n": len(sel),
        "sim_complete": len(ok),
        "sim_complete_rate": len(ok) / len(sel),
    }
    keys = [
        "tasks_finished", "throughput_per_step",
        "wall_time_sec", "solver_runtime_sum", "solver_runtime_max",
        "call_fail_rate", "call_fail", "hl_expanded_sum",
        "diag_total_wall", "diag_primary_wall",
        "diag_fallback",
    ]
    for k in keys:
        vals = [r[k] for r in sel if k in r and r[k] is not None]
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
        for w in WINDOWS:
            for m in AGENTS_LIST:
                for seed in SEEDS:
                    jobs.append((label, solver, mode, prefix, w, m, seed))
    # Run cheaper configs first so failures surface early.
    # ECBS baseline is cheapest; PBS section/baseline at m=600 w=10 heaviest.
    order = {"ECBS_baseline": 0, "PBS_section": 1, "PBS_baseline": 2}
    jobs.sort(key=lambda j: (order[j[0]], j[5], j[4], j[6]))

    print(f"Running {len(jobs)} jobs, parallel={PARALLEL}")
    print(f"  sim_time={SIM_TIME} sim_window={SIM_WINDOW} windows={WINDOWS} agents={AGENTS_LIST}")
    sys.stdout.flush()

    wall = {}
    t_start = time.time()
    prog = OUT_DIR / "progress.log"
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            label, window, agents, seed, elapsed, rc = f.result()
            wall[(label, window, agents, seed)] = elapsed
            done += 1
            line = (f"[{done}/{len(jobs)}] {label:14s} w={window} m={agents} "
                    f"seed={seed:2d} {elapsed:7.1f}s rc={rc}")
            print(line)
            with prog.open("a") as pf:
                pf.write(line + "\n")
            sys.stdout.flush()
    total = time.time() - t_start
    print(f"\nAll runs in {total:.1f}s ({total/60:.1f} min)")

    rows = []
    for (label, solver, mode, prefix, w, m, seed) in jobs:
        r = parse_log(label, solver, mode, prefix, w, m, seed)
        if r:
            r["wall_time_sec"] = wall.get((label, w, m, seed), 0.0)
            rows.append(r)

    all_keys = set()
    for r in rows:
        all_keys.update(r.keys())
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sorted(all_keys))
        w.writeheader()
        w.writerows(rows)

    summaries = []
    for (label, _s, _m, _p) in SOLVERS:
        for w in WINDOWS:
            for m in AGENTS_LIST:
                s = summarize(rows, label, w, m)
                if s:
                    summaries.append(s)
    if summaries:
        sfields = sorted({k for s in summaries for k in s.keys()})
        with (OUT_DIR / "summary.csv").open("w", newline="") as f:
            wt = csv.DictWriter(f, fieldnames=sfields)
            wt.writeheader()
            wt.writerows(summaries)

    lines = []
    lines.append(f"Full comparison: {len(SEEDS)} seeds × 3 solvers × {len(WINDOWS)} windows × {len(AGENTS_LIST)} agents")
    lines.append(f"sim_time={SIM_TIME} sim_window={SIM_WINDOW} subopt={SUBOPT}")
    lines.append(f"Total wall: {total:.1f}s ({total/60:.1f} min)")
    lines.append("")
    for s in summaries:
        h = f"=== {s['solver']} w={s['window']} m={s['agents']} ==="
        lines.append(h)
        lines.append(f"  sim_complete:    {s['sim_complete']}/{s['n']} "
                     f"({s['sim_complete_rate']*100:.0f}%)")
        for k, lbl in [
            ("tasks_finished", "tasks finished"),
            ("throughput_per_step", "throughput / step"),
            ("wall_time_sec", "end-to-end wall (s)"),
            ("solver_runtime_sum", "Σ solver runtime (s)"),
            ("solver_runtime_max", "max per-call solver rt (s)"),
            ("call_fail", "per-call fails (count)"),
            ("call_fail_rate", "per-call fail rate"),
            ("hl_expanded_sum", "Σ HL expanded"),
            ("diag_fallback", "fallback count (section only)"),
        ]:
            mk = f"{k}_mean"
            if mk in s:
                lines.append(
                    f"    {lbl:30s} mean={s[mk]:12.4f}  std={s.get(f'{k}_std',0):10.4f}  "
                    f"min={s.get(f'{k}_min',0):10.4f}  max={s.get(f'{k}_max',0):10.4f}"
                )
        lines.append("")

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print("\n" + txt)


if __name__ == "__main__":
    main()
