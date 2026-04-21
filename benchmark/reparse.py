#!/usr/bin/env python3
"""Re-parse existing benchmark logs with corrected success metric.

Success = simulation completed to 'Done!' with no jump/invalid errors.
Also tracks fallback frequency (how often section solver hit No-solutions
and had to defer to baseline PBS).
"""
import csv
import re
from pathlib import Path
from statistics import mean, median, stdev

OUT_DIR = Path("/Users/minji/Documents/GitHub/RHCR/benchmark_results")
LOG_DIR = OUT_DIR / "logs"
SIM_TIME = 1000
SIM_WINDOW = 5
EXPECTED_PLANS = SIM_TIME // SIM_WINDOW  # 200

MODES = ["section", "baseline"]
SEEDS = list(range(1, 101))

COLS = [
    "runtime", "hl_expanded", "hl_generated", "sol_cost", "min_sum_cost",
    "avg_path_len", "num_collisions", "runtime_plan_paths", "runtime_rt",
    "runtime_get_hp", "runtime_copy_prio", "runtime_detect_conf",
    "runtime_copy_conf", "runtime_choose_conf", "runtime_find_consistent",
    "runtime_find_replan",
]


def parse_log(mode, seed):
    log_file = LOG_DIR / f"{mode}_{seed}.log"
    if not log_file.exists():
        return None
    content = log_file.read_text()

    # ALL PBS solver invocations — prefix is the same PBS: for baseline,
    # but section runs may also contain PBS: lines (fallback). We count
    # timesteps solved by the PRIMARY solver plus fallback separately.
    primary_prefix = "PBSSection:" if mode == "section" else "PBS:"

    # Sum metrics from PRIMARY solver only (the thing we're measuring)
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
        elif line.startswith(primary_prefix + "No solutions"):
            primary_nosol += 1

    # Fallback count (section mode only): PBS:Succeed lines that appear
    # inside a section run are fallback invocations.
    fallback_used = 0
    if mode == "section":
        fallback_used = sum(
            1 for l in content.splitlines() if l.startswith("PBS:Succeed,")
        )

    tasks_finished = sum(
        int(m.group(1))
        for m in re.finditer(r"(\d+) tasks has been finished", content)
    )

    # System-level success: ran all expected planning cycles to "Done!"
    # with no validator errors.
    has_done = content.count("Done!") >= 2  # simulate + save_results
    jump_err = bool(re.search(r"jump from", content))
    invalid_err = bool(re.search(r"Solution invalid", content))
    total_plans = primary_succeed + primary_nosol  # section_run counts both
    if mode == "section":
        # In section mode, every timestep either section succeeded or
        # fell back. Primary-failure means section ran, returned no-sol,
        # baseline ran as fallback — total plans == primary_succeed + fallback.
        total_plans = primary_succeed + fallback_used

    system_success = (
        has_done and not jump_err and not invalid_err
        and total_plans >= EXPECTED_PLANS
    )

    return {
        "mode": mode,
        "seed": seed,
        "tasks_finished": tasks_finished,
        "throughput_per_step": tasks_finished / SIM_TIME,
        "primary_succeeds": primary_succeed,
        "primary_nosol": primary_nosol,
        "fallback_used": fallback_used,
        "total_plans": total_plans,
        "expected_plans": EXPECTED_PLANS,
        "has_done": int(has_done),
        "jump_err": int(jump_err),
        "invalid_err": int(invalid_err),
        "system_success": int(system_success),
        "pure_section": int(mode == "section" and fallback_used == 0
                            and system_success),
        **{f"sum_{k}": v for k, v in totals.items()},
        **{f"max_{k}": v for k, v in maxvals.items()},
    }


def summarize(rows, mode):
    mode_rows = [r for r in rows if r and r["mode"] == mode]
    ok_rows = [r for r in mode_rows if r["system_success"]]

    s = {
        "mode": mode,
        "n": len(mode_rows),
        "system_success_count": len(ok_rows),
        "system_success_rate": len(ok_rows) / len(mode_rows) if mode_rows else 0,
    }

    if mode == "section":
        pure = [r for r in mode_rows if r["pure_section"]]
        s["pure_section_count"] = len(pure)
        s["pure_section_rate"] = len(pure) / len(mode_rows)
        s["total_fallback_events"] = sum(r["fallback_used"] for r in mode_rows)

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
    rows = []
    for m in MODES:
        for s in SEEDS:
            r = parse_log(m, s)
            if r:
                rows.append(r)

    # Per-seed CSV
    fields = list(rows[0].keys())
    with (OUT_DIR / "per_seed.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=fields)
        w.writeheader()
        w.writerows(rows)

    summaries = [summarize(rows, m) for m in MODES]
    sfields = sorted({k for s in summaries for k in s.keys()})
    with (OUT_DIR / "summary.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=sfields)
        w.writeheader()
        w.writerows(summaries)

    lines = []
    lines.append("Benchmark: 100 seeds x 2 modes")
    lines.append("agents=500 sim_time=1000 sim_window=5 plan_window=10")
    lines.append("")
    for s in summaries:
        lines.append(f"=== {s['mode'].upper()} ===")
        lines.append(
            f"  system success:        {s['system_success_count']}/{s['n']} "
            f"({s['system_success_rate']*100:.1f}%)")
        if s["mode"] == "section":
            lines.append(
                f"  pure section (no fallback): {s['pure_section_count']}/{s['n']} "
                f"({s['pure_section_rate']*100:.1f}%)")
            lines.append(
                f"  total fallback events: {s['total_fallback_events']} "
                f"(out of {s['n'] * EXPECTED_PLANS} total planning cycles)")
        if "tasks_finished_mean" in s:
            lines.append("")
            lines.append(
                f"  tasks finished:         mean={s['tasks_finished_mean']:.1f}"
                f" (min {s['tasks_finished_min']:.0f},"
                f" max {s['tasks_finished_max']:.0f},"
                f" std {s['tasks_finished_std']:.1f})")
            lines.append(
                f"  throughput/step:        mean={s['throughput_per_step_mean']:.4f}")
            lines.append(
                f"  total PBS runtime sum:  mean={s['sum_runtime_mean']:.2f}s"
                f"  median={s['sum_runtime_median']:.2f}s"
                f"  std={s['sum_runtime_std']:.2f}s")
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

    if all("tasks_finished_mean" in s for s in summaries):
        section_s = next(s for s in summaries if s["mode"] == "section")
        baseline_s = next(s for s in summaries if s["mode"] == "baseline")
        lines.append("=== SECTION vs BASELINE (section / baseline) ===")
        for label, k in [
            ("tasks finished        ", "tasks_finished"),
            ("total PBS runtime     ", "sum_runtime"),
            ("HL nodes expanded     ", "sum_hl_expanded"),
            ("HL nodes generated    ", "sum_hl_generated"),
            ("detect_conflicts time ", "sum_runtime_detect_conf"),
            ("plan_paths time       ", "sum_runtime_plan_paths"),
            ("find_consistent time  ", "sum_runtime_find_consistent"),
            ("max per-timestep      ", "max_runtime"),
        ]:
            sm = section_s.get(f"{k}_mean")
            bm = baseline_s.get(f"{k}_mean")
            if sm and bm:
                lines.append(f"  {label} {sm/bm:6.3f}x "
                             f"(section {sm:.2f} vs baseline {bm:.2f})")

    txt = "\n".join(lines)
    (OUT_DIR / "summary.txt").write_text(txt)
    print(txt)


if __name__ == "__main__":
    main()
