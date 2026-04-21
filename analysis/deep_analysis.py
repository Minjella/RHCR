#!/usr/bin/env python3
"""Deep analysis for paper submission: mean±std, runtime decomposition, error modes."""
import json
from pathlib import Path

import pandas as pd

REPO = Path("/Users/minji/Documents/GitHub/RHCR")
W = pd.read_csv(REPO / "benchmark_results_wsweep/per_run.csv")
M = pd.read_csv(REPO / "benchmark_results_msweep/per_run.csv")


def agg(df, var):
    rows = []
    for key, g in df.groupby([var, "mode"]):
        x, mode = key
        ok = g[g.system_success == 1]
        row = {var: int(x), "mode": mode, "n": len(g), "succ": len(ok)}
        metrics = {
            "throughput": "throughput_per_step",
            "runtime": "sum_runtime",
            "hl_exp": "sum_hl_expanded",
            "detect_conf": "sum_runtime_detect_conf",
            "plan_paths": "sum_runtime_plan_paths",
            "find_consistent": "sum_runtime_find_consistent",
            "find_replan": "sum_runtime_find_replan",
        }
        for label, col in metrics.items():
            if col in ok.columns:
                row[f"{label}_mean"] = ok[col].mean()
                row[f"{label}_std"] = ok[col].std()
        # Fallback / error
        if "fallback_used" in g.columns:
            row["fallback_sum"] = int(g["fallback_used"].sum())
        if "pure_section" in g.columns:
            row["pure_n"] = int(g["pure_section"].sum())
        row["jump_err"] = int(g.get("jump_err", pd.Series([0]*len(g))).sum())
        row["invalid_err"] = int(g.get("invalid_err", pd.Series([0]*len(g))).sum())
        row["wall_mean"] = g["wall_time_sec"].mean() if "wall_time_sec" in g.columns else None
        row["wall_std"] = g["wall_time_sec"].std() if "wall_time_sec" in g.columns else None
        rows.append(row)
    return pd.DataFrame(rows).sort_values([var, "mode"])


def comp_table(agg_df, var):
    rows = []
    for x in sorted(agg_df[var].unique()):
        sec = agg_df[(agg_df.mode_=="section") & (agg_df[var]==x)].iloc[0] if False else \
              agg_df[(agg_df["mode"]=="section") & (agg_df[var]==x)].iloc[0]
        base = agg_df[(agg_df["mode"]=="baseline") & (agg_df[var]==x)].iloc[0]
        rows.append({
            var: x,
            "sec_thr": sec["throughput_mean"],
            "sec_thr_std": sec.get("throughput_std"),
            "base_thr": base["throughput_mean"],
            "base_thr_std": base.get("throughput_std"),
            "thr_ratio": sec["throughput_mean"]/base["throughput_mean"] if base["throughput_mean"] else None,
            "sec_rt": sec["runtime_mean"],
            "sec_rt_std": sec.get("runtime_std"),
            "base_rt": base["runtime_mean"],
            "base_rt_std": base.get("runtime_std"),
            "speedup": base["runtime_mean"]/sec["runtime_mean"] if sec["runtime_mean"] else None,
            "sec_hl": sec["hl_exp_mean"],
            "base_hl": base["hl_exp_mean"],
            "hl_ratio": sec["hl_exp_mean"]/base["hl_exp_mean"] if base["hl_exp_mean"] else None,
            "sec_detect": sec.get("detect_conf_mean"),
            "base_detect": base.get("detect_conf_mean"),
            "sec_consistent": sec.get("find_consistent_mean"),
            "base_consistent": base.get("find_consistent_mean"),
            "sec_wall": sec.get("wall_mean"),
            "base_wall": base.get("wall_mean"),
            "sec_succ": int(sec["succ"]),
            "base_succ": int(base["succ"]),
            "sec_n": int(sec["n"]),
            "base_n": int(base["n"]),
            "fallback_sum": int(sec.get("fallback_sum", 0)),
            "pure_n": int(sec.get("pure_n", 0)),
            "jump_err": int(sec.get("jump_err", 0)) + int(base.get("jump_err", 0)),
            "invalid_err": int(sec.get("invalid_err", 0)) + int(base.get("invalid_err", 0)),
        })
    return pd.DataFrame(rows)


w_agg = agg(W, "w")
m_agg = agg(M, "m")
w_comp = comp_table(w_agg, "w")
m_comp = comp_table(m_agg, "m")

# Save to JSON for easy consumption
out = {
    "w_sweep": w_comp.to_dict(orient="records"),
    "m_sweep": m_comp.to_dict(orient="records"),
    "w_raw_agg": w_agg.to_dict(orient="records"),
    "m_raw_agg": m_agg.to_dict(orient="records"),
}
(REPO / "analysis/agg.json").write_text(json.dumps(out, indent=2, default=float))

# Print key tables
print("\n=== W-sweep detailed (m=500) ===")
for r in w_comp.to_dict(orient="records"):
    print(f"w={r['w']:>3} | "
          f"sec: thr={r['sec_thr']:.4f}±{r['sec_thr_std']:.4f} rt={r['sec_rt']:.2f}±{r['sec_rt_std']:.2f}s HL={r['sec_hl']:.0f} | "
          f"base: thr={r['base_thr']:.4f}±{r['base_thr_std']:.4f} rt={r['base_rt']:.2f}±{r['base_rt_std']:.2f}s HL={r['base_hl']:.0f} | "
          f"speedup={r['speedup']:.2f}x thr_ratio={r['thr_ratio']:.4f}")

print("\n=== M-sweep detailed (w=10) ===")
for r in m_comp.to_dict(orient="records"):
    print(f"m={r['m']:>3} | "
          f"sec: thr={r['sec_thr']:.4f}±{r['sec_thr_std']:.4f} rt={r['sec_rt']:.2f}±{r['sec_rt_std']:.2f}s HL={r['sec_hl']:.0f} | "
          f"base: thr={r['base_thr']:.4f}±{r['base_thr_std']:.4f} rt={r['base_rt']:.2f}±{r['base_rt_std']:.2f}s HL={r['base_hl']:.0f} | "
          f"speedup={r['speedup']:.2f}x thr_ratio={r['thr_ratio']:.4f}")

print("\n=== Fallback analysis (section m-sweep) ===")
for r in m_comp.to_dict(orient="records"):
    pure_rate = r['pure_n']/r['sec_n'] if r['sec_n'] else 0
    print(f"m={r['m']:>3}: pure_section={r['pure_n']:>3}/{r['sec_n']:<3} ({pure_rate*100:>5.1f}%) | fallback_events={r['fallback_sum']:>5}")

print("\n=== Runtime decomposition w-sweep ===")
for r in w_comp.to_dict(orient="records"):
    print(f"w={r['w']:>3} | "
          f"detect: sec={r.get('sec_detect', 0):.2f}s base={r.get('base_detect', 0):.2f}s | "
          f"find_consistent: sec={r.get('sec_consistent', 0):.2f}s base={r.get('base_consistent', 0):.2f}s")

print("\n=== Runtime decomposition m-sweep ===")
for r in m_comp.to_dict(orient="records"):
    print(f"m={r['m']:>3} | "
          f"detect: sec={r.get('sec_detect', 0):.2f}s base={r.get('base_detect', 0):.2f}s | "
          f"find_consistent: sec={r.get('sec_consistent', 0):.2f}s base={r.get('base_consistent', 0):.2f}s")

print("\n=== Error modes ===")
for var, comp in [("w", w_comp), ("m", m_comp)]:
    for r in comp.to_dict(orient="records"):
        if r["jump_err"] or r["invalid_err"]:
            print(f"  {var}={r[var]}: jump_err={r['jump_err']} invalid_err={r['invalid_err']}")
print("  (none flagged elsewhere)")

print(f"\nWrote /Users/minji/Documents/GitHub/RHCR/analysis/agg.json")
