#!/usr/bin/env python3
"""PBSSection regression: 10 seeds × {w=5, w=10}. Read existing per_run.csv
for baseline, compare current binary."""
import csv
import os
import re
import subprocess
import sys
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from statistics import mean, stdev

REPO = Path(__file__).resolve().parent.parent
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_results_pbs_opt"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(10))
WINDOWS = [5, 10]
SIM_TIME = 1000
SIM_WINDOW = 5
AGENTS = 500
MAP = "maps/sorting_map.grid"
PARALLEL = 4
TIMEOUT_SEC = 1800


def run_single(args):
    window, seed = args
    tag = f"PBS_w{window}_seed{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [
        str(BINARY), "-m", MAP, "--scenario=SORTING",
        f"--simulation_window={SIM_WINDOW}",
        f"--planning_window={window}",
        "--solver=PBS",
        f"--simulation_time={SIM_TIME}",
        f"--seed={seed}",
        "-k", str(AGENTS),
        "--screen=1",
    ]
    t0 = time.time()
    try:
        result = subprocess.run(cmd, cwd=str(REPO), capture_output=True,
                                text=True, timeout=TIMEOUT_SEC)
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return window, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return window, seed, elapsed, -999


PBS_COLS = [
    "runtime", "hl_expanded", "hl_generated", "sol_cost", "min_sum_cost",
    "avg_path_len", "num_collisions", "rt_plan_paths", "rt_rt",
    "rt_get_hp", "rt_copy_prio", "rt_detect_conf",
    "rt_copy_conf", "rt_choose_conf", "rt_find_consistent",
    "rt_find_replan",
]


def parse_diag(content):
    d = {}
    patterns = {
        "diag_total_calls":      r"total calls:\s+(\d+)",
        "diag_primary_ok":       r"primary success:\s+(\d+)",
        "diag_fallback":         r"fallback invocations:\s+(\d+)",
        "diag_total_wall":       r"TOTAL solve_by_Section:\s+([\d.]+)\s+s",
        "diag_primary_wall":     r"primary run_section:\s+([\d.]+)\s+s",
        "diag_success_subtotal": r"success subtotal:\s+([\d.]+)\s+s",
        "diag_failure_subtotal": r"failure subtotal:\s+([\d.]+)\s+s",
        "diag_fallback_wall":    r"fallback baseline PBS:\s+([\d.]+)\s+s",
    }
    for k, pat in patterns.items():
        m = re.search(pat, content)
        if m:
            s = m.group(1)
            try:
                d[k] = float(s) if "." in s else int(s)
            except ValueError:
                d[k] = 0
    return d


def parse_log(window, seed):
    log_file = LOG_DIR / f"PBS_w{window}_seed{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()
    prefix = "PBSSection:"
    succ = [l for l in content.splitlines() if l.startswith(prefix + "Succeed,")]
    fail = [l for l in content.splitlines()
            if l.startswith(prefix) and not l.startswith(prefix + "Succeed,")]
    totals = {c: 0.0 for c in PBS_COLS}
    for line in succ:
        values = line.split(":", 1)[1].split(",")[1:]
        for i, c in enumerate(PBS_COLS):
            if i < len(values):
                try:
                    totals[c] += float(values[i])
                except ValueError:
                    pass
    tasks = sum(int(m.group(1)) for m in re.finditer(r"(\d+) tasks has been finished", content))
    errs = len(re.findall(r"Solution invalid|jump from", content))
    timesteps = len(re.findall(r"^Timestep \d+", content, re.M))
    expected = SIM_TIME // SIM_WINDOW
    row = {
        "window": window, "seed": seed,
        "tasks_finished": tasks,
        "timesteps_simulated": timesteps,
        "section_fail_lines": len(fail),
        "errors": errs,
        "success": int(errs == 0 and timesteps >= expected),
        **{f"sum_{k}": v for k, v in totals.items()},
    }
    row.update(parse_diag(content))
    return row


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)
    jobs = [(w, s) for w in WINDOWS for s in SEEDS]
    print(f"Running {len(jobs)} PBS runs, {PARALLEL} parallel")
    sys.stdout.flush()
    wall = {}
    t0 = time.time()
    prog = OUT_DIR / "progress.log"
    prog.write_text("")
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            w, s, el, rc = f.result()
            wall[(w, s)] = el
            done += 1
            line = f"[{done}/{len(jobs)}] PBS w={w} seed={s:2d} {el:6.1f}s rc={rc}"
            print(line)
            with prog.open("a") as pf:
                pf.write(line + "\n")
            sys.stdout.flush()
    total = time.time() - t0
    print(f"\nDone in {total:.1f}s ({total/60:.1f} min)")

    rows = []
    for w in WINDOWS:
        for s in SEEDS:
            r = parse_log(w, s)
            if r:
                r["wall_time_sec"] = wall.get((w, s), 0.0)
                rows.append(r)

    all_keys = set()
    for r in rows:
        all_keys.update(r.keys())
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=sorted(all_keys))
        wt.writeheader()
        wt.writerows(rows)

    # Load baseline
    base_path = REPO / "benchmark_results_ecbs_vs_pbs" / "per_run.csv"
    base = {}
    if base_path.exists():
        with base_path.open() as f:
            for r in csv.DictReader(f):
                if r["solver"] == "PBS":
                    base[(int(r["window"]), int(r["seed"]))] = r

    def summarize(rows_subset, label):
        ok = [r for r in rows_subset if r["success"]]
        print(f"  {label:25s} n={len(rows_subset)} success={len(ok)}/{len(rows_subset)}")
        if not ok:
            return
        keys = ["tasks_finished", "wall_time_sec", "sum_runtime",
                "sum_rt_detect_conf", "diag_total_wall",
                "diag_primary_wall", "diag_fallback",
                "sum_hl_expanded"]
        for k in keys:
            vals = [float(r[k]) for r in ok if k in r]
            if vals:
                print(f"    {k:26s} mean={mean(vals):.3f}  std={stdev(vals) if len(vals)>1 else 0:.3f}")

    for w in WINDOWS:
        print(f"\n=== w={w} ===")
        new_rows = [r for r in rows if r["window"] == w]
        summarize(new_rows, "OPTIMIZED")

        if base:
            # Seed-by-seed comparison
            print(f"  --- per-seed delta (optimized - baseline) ---")
            print(f"  {'seed':>4s}  {'wall Δ':>9s}  {'rt Δ':>9s}  {'detect Δ':>9s}  {'primary_wall Δ':>14s}")
            for s in SEEDS:
                nr = next((r for r in new_rows if r["seed"] == s), None)
                b = base.get((w, s))
                if nr and b:
                    def d(k):
                        try:
                            return float(nr[k]) - float(b[k])
                        except (KeyError, ValueError):
                            return 0
                    print(f"  {s:4d}  {d('wall_time_sec'):+9.3f}  {d('sum_runtime'):+9.3f}  "
                          f"{d('sum_rt_detect_conf'):+9.3f}  {d('diag_primary_wall'):+14.3f}")


if __name__ == "__main__":
    main()
