#!/usr/bin/env python3
"""
Baseline ECBS vs PBSSection comparison.

Config:
  PBS  runs section mode   (PBSSection, RHCR_SOLVER_MODE unset / 'section')
  ECBS runs baseline mode  (RHCR_SOLVER_MODE=baseline, prints 'ECBS:...')
  planning win : 5, 10
  seeds        : 0..9 (10 seeds)
  m=500, simulation_time=1000, simulation_window=5
  ECBS suboptimal_bound=1.1

Output: benchmark_results_ecbs_vs_pbs/
  logs/<solver>_w<W>_seed<N>.log
  per_run.csv
  summary.csv
  summary.txt
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
OUT_DIR = REPO / "benchmark_results_ecbs_vs_pbs"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(10))
SOLVERS = ["PBS", "ECBS"]
WINDOWS = [5, 10]
SIM_TIME = 1000
SIM_WINDOW = 5
AGENTS = 500
MAP = "maps/sorting_map.grid"
SUBOPT = 1.1
PARALLEL = 4
TIMEOUT_SEC = 60 * 60  # 60 min per run safety cap (ECBS w=5 pilot was ~13 min)


def run_single(args):
    solver, window, seed = args
    tag = f"{solver}_w{window}_seed{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={window}",
        f"--solver={solver}",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(AGENTS),
        "--screen=1",  # baseline print_results() gated on screen>0
    ]
    if solver == "ECBS":
        cmd.append(f"--suboptimal_bound={SUBOPT}")

    env = os.environ.copy()
    if solver == "ECBS":
        env["RHCR_SOLVER_MODE"] = "baseline"
    # PBS: leave unset → section mode (default)

    t0 = time.time()
    try:
        result = subprocess.run(
            cmd, cwd=str(REPO), env=env, capture_output=True, text=True,
            timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return solver, window, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return solver, window, seed, elapsed, -999


PBS_COLS = [
    "runtime", "hl_expanded", "hl_generated", "sol_cost", "min_sum_cost",
    "avg_path_len", "num_collisions", "rt_plan_paths", "rt_rt",
    "rt_get_hp", "rt_copy_prio", "rt_detect_conf",
    "rt_copy_conf", "rt_choose_conf", "rt_find_consistent",
    "rt_find_replan",
]
# ECBS print_results columns after Succeed,
#   runtime, HL_exp, HL_gen, LL_exp, LL_gen, sol_cost, min_f, avg_path,
#   num_col, window, suboptimal, snf=X, snc=Y
ECBS_COLS = [
    "runtime", "hl_expanded", "hl_generated", "ll_expanded", "ll_generated",
    "sol_cost", "min_sum_cost", "avg_path_len", "num_collisions",
    "window", "subopt_bound",
]


def parse_diag(content):
    """Extract section-mode wall-clock diagnostics."""
    d = {}
    patterns = {
        "diag_total_calls":    r"total calls:\s+(\d+)",
        "diag_primary_ok":     r"primary success:\s+(\d+)",
        "diag_fallback":       r"fallback invocations:\s+(\d+)",
        "diag_total_wall":     r"TOTAL solve_by_Section:\s+([\d.]+)\s+s",
        "diag_primary_wall":   r"primary run_section:\s+([\d.]+)\s+s",
        "diag_success_subtotal": r"success subtotal:\s+([\d.]+)\s+s",
        "diag_failure_subtotal": r"failure subtotal:\s+([\d.]+)\s+s",
        "diag_fallback_wall":  r"fallback baseline PBS:\s+([\d.]+)\s+s",
    }
    for k, pat in patterns.items():
        m = re.search(pat, content)
        if m:
            try:
                d[k] = float(m.group(1)) if "." in m.group(1) else int(m.group(1))
            except ValueError:
                d[k] = 0
    return d


def parse_log(solver, window, seed):
    log_file = LOG_DIR / f"{solver}_w{window}_seed{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    # PBS uses section mode (prefix 'PBSSection:'),
    # ECBS runs baseline (prefix 'ECBS:').
    prefix = "PBSSection:" if solver == "PBS" else "ECBS:"
    cols = PBS_COLS if solver == "PBS" else ECBS_COLS

    succ = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    fail = [l for l in content.splitlines()
            if l.startswith(prefix) and not l.startswith(prefix + "Succeed,")]

    totals = {c: 0.0 for c in cols}
    maxvals = {c: 0.0 for c in cols}
    for line in succ:
        values = line.split(":", 1)[1].split(",")[1:]  # drop "Succeed"
        for i, c in enumerate(cols):
            if i < len(values):
                s = values[i]
                if "=" in s:
                    s = s.split("=")[1]
                try:
                    v = float(s)
                    totals[c] += v
                    if v > maxvals[c]:
                        maxvals[c] = v
                except ValueError:
                    pass

    tasks_finished = sum(
        int(m.group(1))
        for m in re.finditer(r"(\d+) tasks has been finished", content)
    )
    errors = len(re.findall(r"Solution invalid|jump from", content))
    timesteps_simulated = len(re.findall(r"^Timestep \d+", content, re.M))
    expected = SIM_TIME // SIM_WINDOW

    row = {
        "solver": solver,
        "window": window,
        "seed": seed,
        "tasks_finished": tasks_finished,
        "throughput_per_step": tasks_finished / SIM_TIME,
        "timesteps_ok": len(succ),
        "timesteps_simulated": timesteps_simulated,
        "expected_timesteps": expected,
        "section_fail_lines": len(fail),
        "errors": errors,
        "success": int(errors == 0 and timesteps_simulated >= expected),
        **{f"sum_{k}": v for k, v in totals.items()},
        **{f"max_{k}": v for k, v in maxvals.items()},
    }
    row.update(parse_diag(content))
    return row


def summarize(rows, solver, window):
    sel = [r for r in rows if r and r["solver"] == solver and r["window"] == window]
    ok = [r for r in sel if r["success"]]
    s = {
        "solver": solver,
        "window": window,
        "n": len(sel),
        "success_count": len(ok),
        "success_rate": len(ok) / len(sel) if sel else 0.0,
    }
    keys = [
        "tasks_finished", "throughput_per_step",
        "sum_runtime", "sum_hl_expanded", "sum_hl_generated",
        "max_runtime", "wall_time_sec",
        "diag_total_wall", "diag_primary_wall",
        "diag_success_subtotal", "diag_failure_subtotal",
        "diag_fallback_wall", "diag_fallback",
    ]
    for k in keys:
        vals = [r[k] for r in ok if k in r]
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

    # Order: run shortest-first by a rough estimate so faster jobs
    # don't sit behind the long ECBS w=5 ones.
    jobs = []
    for w in WINDOWS:
        for s in SEEDS:
            for solver in SOLVERS:
                jobs.append((solver, w, s))
    # stable sort: PBS first (cheap), w=10 before w=5 for same solver
    jobs.sort(key=lambda j: (0 if j[0] == "PBS" else 1, -j[1], j[2]))

    print(f"Running {len(jobs)} runs, {PARALLEL} in parallel")
    print(f"  solvers={SOLVERS} windows={WINDOWS} seeds={SEEDS}")
    print(f"  m={AGENTS} sim_time={SIM_TIME} sim_window={SIM_WINDOW} subopt={SUBOPT}")
    sys.stdout.flush()

    wall_times = {}
    t_start = time.time()
    progress_file = OUT_DIR / "progress.log"
    progress_file.write_text("")

    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            solver, window, seed, elapsed, rc = f.result()
            wall_times[(solver, window, seed)] = elapsed
            done += 1
            status = "OK" if rc == 0 else f"rc={rc}"
            line = (f"[{done}/{len(jobs)}] {solver:5s} w={window} "
                    f"seed={seed:2d} {elapsed:7.1f}s {status}")
            print(line)
            with progress_file.open("a") as pf:
                pf.write(line + "\n")
            sys.stdout.flush()

    total_wall = time.time() - t_start
    print(f"\nAll runs done in {total_wall:.1f}s ({total_wall/60:.1f} min)")

    rows = []
    for s, w, d in [(jb[0], jb[1], jb[2]) for jb in jobs]:
        r = parse_log(s, w, d)
        if r:
            r["wall_time_sec"] = wall_times.get((s, w, d), 0.0)
            rows.append(r)

    if not rows:
        print("No rows parsed")
        return

    # Per-run CSV
    all_keys = set()
    for r in rows:
        all_keys.update(r.keys())
    fields = sorted(all_keys)
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    # Summary per (solver, window)
    summaries = [summarize(rows, solver, window)
                 for solver in SOLVERS for window in WINDOWS]
    sfields = sorted({k for s in summaries for k in s.keys()})
    with (OUT_DIR / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sfields)
        w.writeheader()
        w.writerows(summaries)

    lines = []
    lines.append(f"ECBS vs PBSSection: {len(SEEDS)} seeds × {len(SOLVERS)} × {len(WINDOWS)}")
    lines.append(f"m={AGENTS} sim_time={SIM_TIME} sim_window={SIM_WINDOW} subopt={SUBOPT}")
    lines.append(f"Total wall: {total_wall:.1f}s ({total_wall/60:.1f} min)")
    lines.append("")

    for s in summaries:
        h = f"=== {s['solver']} w={s['window']} ==="
        lines.append(h)
        lines.append(f"  success:   {s['success_count']}/{s['n']} "
                     f"({s['success_rate']*100:.1f}%)")
        for key, label in [
            ("tasks_finished", "tasks finished"),
            ("throughput_per_step", "throughput / step"),
            ("wall_time_sec", "end-to-end wall (s)"),
            ("sum_runtime", "Σ internal runtime (s)"),
            ("diag_total_wall", "Σ solve_by_Section wall (s)"),
            ("diag_primary_wall", "Σ primary wall (s)"),
            ("diag_failure_subtotal", "Σ primary failure wall (s)"),
            ("diag_fallback_wall", "Σ fallback wall (s)"),
            ("diag_fallback", "fallback count"),
            ("sum_hl_expanded", "Σ HL expanded"),
            ("sum_hl_generated", "Σ HL generated"),
            ("max_runtime", "max per-step runtime (s)"),
        ]:
            mk, stk = f"{key}_mean", f"{key}_std"
            if mk in s:
                lines.append(f"    {label:32s} mean={s[mk]:10.3f} std={s.get(stk,0):8.3f} "
                             f"min={s.get(f'{key}_min',0):.3f} max={s.get(f'{key}_max',0):.3f}")
        lines.append("")

    # pairwise ratios PBS vs ECBS per window
    lines.append("=== RATIO  ECBS / PBS  (mean of ratios per seed, success-only) ===")
    for w in WINDOWS:
        pbs_rows = {r["seed"]: r for r in rows
                    if r["solver"] == "PBS" and r["window"] == w and r["success"]}
        ecbs_rows = {r["seed"]: r for r in rows
                     if r["solver"] == "ECBS" and r["window"] == w and r["success"]}
        common = sorted(set(pbs_rows) & set(ecbs_rows))
        if not common:
            lines.append(f"  w={w}: no common successful seeds")
            continue
        lines.append(f"  w={w}: {len(common)} common seeds")
        for k in ["tasks_finished", "sum_runtime", "wall_time_sec",
                  "sum_hl_expanded", "sum_hl_generated"]:
            ratios = []
            for d in common:
                a = ecbs_rows[d].get(k, 0)
                b = pbs_rows[d].get(k, 0)
                if b > 0:
                    ratios.append(a / b)
            if ratios:
                lines.append(f"    {k:28s} ratio mean={mean(ratios):.3f}")

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print("\n" + txt)


if __name__ == "__main__":
    main()
