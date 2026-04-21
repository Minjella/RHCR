#!/usr/bin/env python3
"""
M (agent count) sweep experiment for scaling analysis.

Varies number of agents m at fixed w=10 sim_time=5000. The range is
chosen based on viability checks:
    m <= 800: 100 seeds (rigorous benchmark)
    m = 900:  30 seeds (degradation point demonstration)

m=1000 and above are excluded — section hits deadlock on nearly every
planning call and falls back to baseline, making the comparison
meaningless at those densities.

Output:
    benchmark_results_msweep/logs/<mode>_m<m>_<seed>.log
    benchmark_results_msweep/per_run.csv
    benchmark_results_msweep/summary.csv
    benchmark_results_msweep/summary.txt
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
OUT_DIR = REPO / "benchmark_results_msweep"
LOG_DIR = OUT_DIR / "logs"

# (m, seed_count) — asymmetric to match viability
M_CONFIG = [
    (300, 100),
    (400, 100),
    (500, 100),
    (600, 100),
    (700, 100),
    (800, 100),
    (900, 30),
]
MODES = ["section", "baseline"]
SIM_TIME = 5000
SIM_WINDOW = 5
PLAN_WINDOW = 10
MAP = "maps/sorting_map.grid"
PARALLEL = 10
TIMEOUT_SEC = 36000  # 10 hours hard cap per single run (for m=900 worst case)

COLS = [
    "runtime", "hl_expanded", "hl_generated", "sol_cost", "min_sum_cost",
    "avg_path_len", "num_collisions", "runtime_plan_paths", "runtime_rt",
    "runtime_get_hp", "runtime_copy_prio", "runtime_detect_conf",
    "runtime_copy_conf", "runtime_choose_conf", "runtime_find_consistent",
    "runtime_find_replan",
]


def run_single(args):
    mode, m, seed = args
    log_file = LOG_DIR / f"{mode}_m{m}_s{seed}.log"
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
                f"--seed={seed}", "-k", str(m),
            ],
            cwd=str(REPO), env=env, capture_output=True, text=True,
            timeout=TIMEOUT_SEC,
        )
        elapsed = time.time() - t0
        log_file.write_text(result.stdout + "\n---STDERR---\n" + result.stderr)
        return mode, m, seed, elapsed, result.returncode
    except subprocess.TimeoutExpired:
        elapsed = time.time() - t0
        log_file.write_text(f"TIMEOUT after {elapsed:.1f}s")
        return mode, m, seed, elapsed, -999


def parse_log(mode, m, seed):
    log_file = LOG_DIR / f"{mode}_m{m}_s{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    primary_prefix = "PBSSection:" if mode == "section" else "PBS:"

    totals = {c: 0.0 for c in COLS}
    maxvals = {c: 0.0 for c in COLS}
    primary_succeed = 0
    primary_nosol = 0
    for line in content.splitlines():
        if not line.startswith(primary_prefix):
            continue
        if line.startswith(primary_prefix + "Succeed,"):
            primary_succeed += 1
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
        elif line.startswith(primary_prefix + "No solutions") or line.startswith(primary_prefix + "Timeout"):
            primary_nosol += 1

    fallback_used = 0
    if mode == "section":
        fallback_used = sum(
            1 for l in content.splitlines() if l.startswith("PBS:Succeed,")
        )

    tasks_finished = sum(
        int(m_.group(1))
        for m_ in re.finditer(r"(\d+) tasks has been finished", content)
    )
    has_done = content.count("Done!") >= 2
    jump_err = bool(re.search(r"jump from", content))
    invalid_err = bool(re.search(r"Solution invalid", content))

    total_plans = primary_succeed + primary_nosol
    if mode == "section":
        total_plans = primary_succeed + fallback_used

    expected_plans = SIM_TIME // SIM_WINDOW
    system_success = (
        has_done and not jump_err and not invalid_err
        and total_plans >= expected_plans
    )

    return {
        "mode": mode,
        "m": m,
        "seed": seed,
        "tasks_finished": tasks_finished,
        "throughput_per_step": tasks_finished / SIM_TIME,
        "primary_succeeds": primary_succeed,
        "primary_nosol": primary_nosol,
        "fallback_used": fallback_used,
        "total_plans": total_plans,
        "expected_plans": expected_plans,
        "has_done": int(has_done),
        "jump_err": int(jump_err),
        "invalid_err": int(invalid_err),
        "system_success": int(system_success),
        "pure_section": int(mode == "section" and fallback_used == 0 and system_success),
        **{f"sum_{k}": v for k, v in totals.items()},
        **{f"max_{k}": v for k, v in maxvals.items()},
    }


def summarize(rows, mode, m):
    grp = [r for r in rows if r and r["mode"] == mode and r["m"] == m]
    ok = [r for r in grp if r["system_success"]]

    s = {
        "mode": mode, "m": m, "n": len(grp),
        "system_success_count": len(ok),
        "system_success_rate": len(ok) / len(grp) if grp else 0,
    }
    if mode == "section":
        pure = [r for r in grp if r["pure_section"]]
        s["pure_section_count"] = len(pure)
        s["total_fallback_events"] = sum(r["fallback_used"] for r in grp)

    keys = [
        "tasks_finished", "throughput_per_step",
        "sum_runtime", "sum_hl_expanded", "sum_hl_generated",
        "sum_runtime_detect_conf", "sum_runtime_plan_paths",
        "sum_runtime_find_consistent", "max_runtime",
    ]
    for k in keys:
        vals = [r[k] for r in ok if k in r]
        if vals:
            s[f"{k}_mean"] = mean(vals)
            s[f"{k}_median"] = median(vals)
            s[f"{k}_std"] = stdev(vals) if len(vals) > 1 else 0.0
    return s


def main():
    OUT_DIR.mkdir(exist_ok=True)
    LOG_DIR.mkdir(exist_ok=True)

    jobs = []
    for m, n_seeds in M_CONFIG:
        for mode in MODES:
            for seed in range(1, n_seeds + 1):
                jobs.append((mode, m, seed))

    print(f"Running {len(jobs)} simulations, {PARALLEL} in parallel...")
    print(f"  sim_time={SIM_TIME} sim_window={SIM_WINDOW} "
          f"plan_window={PLAN_WINDOW}")
    print(f"  M config: {M_CONFIG}")
    sys.stdout.flush()

    wall_times = {}
    t_start = time.time()
    with ProcessPoolExecutor(max_workers=PARALLEL) as ex:
        futures = {ex.submit(run_single, j): j for j in jobs}
        done = 0
        for f in as_completed(futures):
            mode, m, seed, elapsed, rc = f.result()
            wall_times[(mode, m, seed)] = elapsed
            done += 1
            status = "OK" if rc == 0 else f"rc={rc}"
            if done % 10 == 0 or done == len(jobs):
                print(f"[{done}/{len(jobs)}] {mode:8s} m={m:4d} seed={seed:3d} "
                      f"{elapsed:7.1f}s {status}")
                sys.stdout.flush()

    total_wall = time.time() - t_start
    print(f"\nAll runs done in {total_wall:.1f}s ({total_wall/60:.1f} min)")

    rows = []
    for mode, m, seed in jobs:
        r = parse_log(mode, m, seed)
        if r:
            r["wall_time_sec"] = wall_times.get((mode, m, seed), 0.0)
            rows.append(r)

    if rows:
        fields = list(rows[0].keys())
        with (OUT_DIR / "per_run.csv").open("w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=fields)
            wr.writeheader()
            wr.writerows(rows)

    summaries = []
    for mode in MODES:
        for m, _ in M_CONFIG:
            summaries.append(summarize(rows, mode, m))

    if summaries:
        sfields = sorted({k for s in summaries for k in s.keys()})
        with (OUT_DIR / "summary.csv").open("w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=sfields)
            wr.writeheader()
            wr.writerows(summaries)

    lines = []
    lines.append(f"M sweep — w={PLAN_WINDOW} sim_time={SIM_TIME} sim_window={SIM_WINDOW}")
    lines.append(f"Total wall time: {total_wall/60:.1f} min")
    lines.append("")
    lines.append(f"{'m':>6} | {'mode':>10} | {'n':>4} | {'throughput':>11} "
                 f"| {'PBS runtime':>12} | {'HL exp':>10} | {'success':>10}")
    lines.append("-" * 90)
    for s in summaries:
        th = s.get("throughput_per_step_mean", float("nan"))
        rt = s.get("sum_runtime_mean", float("nan"))
        he = s.get("sum_hl_expanded_mean", float("nan"))
        succ = s.get("system_success_count", 0)
        n = s.get("n", 0)
        lines.append(f"{s['m']:>6} | {s['mode']:>10} | {n:>4} | "
                     f"{th:>11.4f} | {rt:>10.2f}s | {he:>10.0f} | {succ:>4d}/{n}")
    lines.append("")
    lines.append("=== Section pure vs fallback by m ===")
    for m, _ in M_CONFIG:
        sec = next((s for s in summaries if s["mode"] == "section" and s["m"] == m), None)
        if sec and "pure_section_count" in sec:
            lines.append(f"  m={m:4d}: pure_section={sec['pure_section_count']}/{sec['n']}, "
                         f"fallback_events={sec['total_fallback_events']}")
    lines.append("")
    lines.append("=== Section vs Baseline ratios (per m) ===")
    for m, _ in M_CONFIG:
        sec = next((s for s in summaries if s["mode"] == "section" and s["m"] == m), None)
        base = next((s for s in summaries if s["mode"] == "baseline" and s["m"] == m), None)
        if not sec or not base or "sum_runtime_mean" not in sec or "sum_runtime_mean" not in base:
            continue
        speedup = base["sum_runtime_mean"] / sec["sum_runtime_mean"] if sec["sum_runtime_mean"] else float("inf")
        th_ratio = sec["throughput_per_step_mean"] / base["throughput_per_step_mean"] if base.get("throughput_per_step_mean") else float("inf")
        lines.append(
            f"  m={m:4d}: speedup={speedup:.2f}x, throughput_ratio={th_ratio:.4f} "
            f"(section {sec['throughput_per_step_mean']:.4f} vs "
            f"baseline {base['throughput_per_step_mean']:.4f})"
        )

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print("\n" + txt)


if __name__ == "__main__":
    main()
