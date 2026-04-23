#!/usr/bin/env python3
"""Quick ECBSSection regression: 10 seeds × (w=5, w=10), sim=500, m=500."""
import csv
import os
import re
import subprocess
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path
from statistics import mean, stdev

REPO = Path(__file__).resolve().parent.parent
BINARY = REPO / "lifelong"
OUT_DIR = REPO / "benchmark_results_ecbs_section_sweep"
LOG_DIR = OUT_DIR / "logs"

SEEDS = list(range(10))
WINDOWS = [5, 10]
SIM_TIME = 500
SIM_WINDOW = 5
AGENTS = 500
MAP = "maps/sorting_map.grid"
SUBOPT = 1.1
PARALLEL = 4
TIMEOUT_SEC = 30 * 60


def run_single(args):
    window, seed = args
    tag = f"ECBS_section_w{window}_seed{seed}"
    log_file = LOG_DIR / f"{tag}.log"
    cmd = [str(BINARY), "-m", MAP, "--scenario=SORTING",
           f"--simulation_window={SIM_WINDOW}",
           f"--planning_window={window}",
           "--solver=ECBS", f"--suboptimal_bound={SUBOPT}",
           f"--simulation_time={SIM_TIME}",
           f"--seed={seed}", "-k", str(AGENTS), "--screen=1"]
    env = os.environ.copy()
    env.pop("RHCR_SOLVER_MODE", None)
    t0 = time.time()
    try:
        r = subprocess.run(cmd, cwd=str(REPO), env=env, capture_output=True,
                           text=True, timeout=TIMEOUT_SEC)
        elapsed = time.time() - t0
        log_file.write_text(r.stdout + "\n---STDERR---\n" + r.stderr)
        return window, seed, elapsed, r.returncode
    except subprocess.TimeoutExpired:
        log_file.write_text(f"TIMEOUT after {time.time()-t0:.1f}s")
        return window, seed, time.time()-t0, -999


def parse(window, seed):
    log = LOG_DIR / f"ECBS_section_w{window}_seed{seed}.log"
    if not log.exists():
        return None
    c = log.read_text()
    succ = len(re.findall(r"^ECBSSection:Succeed", c, re.M))
    fail = len(re.findall(r"^ECBSSection:(Timeout|No solutions|Nodesout)", c, re.M))
    tasks = sum(int(m.group(1)) for m in re.finditer(r"(\d+) tasks has been finished", c))
    ts = len(re.findall(r"^Timestep \d+", c, re.M))
    d = {}
    for k, pat in [("diag_primary_ok", r"primary success:\s+(\d+)"),
                   ("diag_fallback", r"fallback invocations:\s+(\d+)"),
                   ("diag_total_wall", r"TOTAL solve_by_Section:\s+([\d.]+)\s+s"),
                   ("diag_fail_subtotal", r"failure subtotal:\s+([\d.]+)\s+s"),
                   ("diag_success_subtotal", r"success subtotal:\s+([\d.]+)\s+s")]:
        m = re.search(pat, c)
        if m:
            s = m.group(1)
            try: d[k] = float(s) if "." in s else int(s)
            except ValueError: d[k] = 0
    return {"window": window, "seed": seed, "primary_ok": succ, "primary_fail": fail,
            "tasks": tasks, "timesteps_simulated": ts,
            "sim_complete": int(ts >= SIM_TIME // SIM_WINDOW),
            **d}


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)
    prog = OUT_DIR / "progress.log"
    prog.write_text("")
    jobs = [(w, s) for w in WINDOWS for s in SEEDS]
    print(f"Running {len(jobs)} ECBSSection runs, parallel={PARALLEL}")
    wall = {}
    t0 = time.time()
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            w, s, el, rc = f.result()
            wall[(w, s)] = el
            done += 1
            line = f"[{done}/{len(jobs)}] ECBS_section w={w} seed={s} {el:6.1f}s rc={rc}"
            print(line, flush=True)
            with prog.open("a") as pf:
                pf.write(line + "\n")
    total = time.time() - t0
    print(f"\nDone {total:.1f}s")

    rows = []
    for (w, s) in jobs:
        r = parse(w, s)
        if r:
            r["wall_time_sec"] = wall.get((w, s), 0.0)
            rows.append(r)
    with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
        wt = csv.DictWriter(f, fieldnames=sorted({k for r in rows for k in r.keys()}))
        wt.writeheader(); wt.writerows(rows)

    for w in WINDOWS:
        sel = [r for r in rows if r["window"] == w]
        ok = [r for r in sel if r["sim_complete"]]
        print(f"\n=== w={w} ===  sim_complete {len(ok)}/{len(sel)}")
        for k in ["primary_ok", "primary_fail", "diag_fallback", "tasks",
                  "wall_time_sec", "diag_fail_subtotal", "diag_success_subtotal"]:
            vals = [float(r[k]) for r in sel if r.get(k) is not None]
            if vals:
                print(f"  {k:22s} mean={mean(vals):10.3f} std={stdev(vals) if len(vals)>1 else 0:8.3f}  sum={sum(vals):.1f}")

if __name__ == "__main__":
    main()
