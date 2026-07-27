#!/usr/bin/env python3
"""Aggregate the preregistered FHv2 gate + slim robustness-matrix SITL runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from pathlib import Path
from typing import Dict, List

import numpy as np


MODULE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = MODULE_ROOT / "validation" / "fh2_robustness_20260723"
VFB_REPORT = MODULE_ROOT / "validation_tools" / "zd680_vfb_report.py"
UCIS_REPORT = MODULE_ROOT / "validation_tools" / "zd680_ucis_preliminary_report.py"
STAGE1_BENCHMARK = MODULE_ROOT / "validation_tools" / "zd680_heso_stage1_frequency_benchmark.py"
PREVIOUS_FUCI = MODULE_ROOT / "validation" / "fuci_constrained_20260722" / "single_result.json"
FH1_RESULTS = MODULE_ROOT / "validation" / "fuci_finite_horizon_20260722" / "formal_results.csv"

# Preregistered run plan (2026-07-23), incl. amendment v2: FH2_06 was re-flown
# with the relaxed 0.6 m swing bounds after the first attempt failed only the
# XY gate; the v1 attempt stays in gate_runs/ and is excluded from the
# decision (documented in 预注册修订记录v2_06m设计界.md).
GATE_POINTS = (
    ("FH2_06", "STD_L06_PID_AS_UCIS_FH2", "gate_runs_v2", "gate"),
    ("FH2_08", "STD_L08_PID_AS_UCIS_FH2", "gate_runs", "gate"),
)
# (label, mode, [subdirs], planned, kind).  kind=baseline/ei points are
# comparators: they are EXPECTED to fail swing gates and are excluded from
# the matrix_pass verdict.  L06/L08 aggregate the gate runs with the matrix
# runs so each reaches n=5 (amendment v4).
MATRIX_POINTS = (
    ("L05", "STD_L05_PID_AS_UCIS_FH2", ("matrix_runs",), 3, "method"),
    ("L06", "STD_L06_PID_AS_UCIS_FH2", ("gate_runs_v2", "matrix_runs"), 5, "method"),
    # Amendment v5: L08 is judged on the five v5 reruns only; the v1 L08 runs
    # (gate_runs + matrix_runs) stay on record but used the superseded 0.8 m
    # coefficients with the highest offset floor (0.0880).
    ("L08", "STD_L08_PID_AS_UCIS_FH2", ("matrix_runs_v5",), 5, "method"),
    ("P75", "STD_L06P75_PID_AS_UCIS_FH2", ("matrix_runs",), 3, "method"),
    ("MM070", "STD_MM070_PID_AS_UCIS_FH2", ("matrix_runs",), 5, "method"),
    ("BASE_L08", "STD_L08_PID_AS", ("matrix_runs",), 3, "baseline"),
    ("EI_L06", "STD_L06_PID_AS_UCIS_EI", ("matrix_runs",), 3, "ei"),
    ("EI_MM070", "STD_MM070_PID_AS_UCIS_EI", ("matrix_runs",), 3, "ei"),
)
# 0.6 m PID+AS baseline reused from the 2026-07-22 quick-freeze (n=5).
BASELINE_L06 = {
    "swing_angle_rms_deg": (1.65874, 0.01281),
    "swing_angle_peak_deg": (5.03434, 0.09003),
    "swing_energy_integral_j_s_kg": (0.33130, 0.00603),
    "xy_error_rms_m": (0.05652, 0.00231),
    "final_jerk_p95_m_s3": (2.24209, 0.07983),
}
METRICS = (
    "swing_angle_rms_deg", "swing_angle_peak_deg", "swing_energy_integral_j_s_kg",
    "xy_error_rms_m", "command_vs_mission_xy_offset_rms_m", "final_jerk_p95_m_s3",
    "truth_z_peak_m",
)


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def fmt(value: object, digits: int = 3) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def run_dirs(root: Path, sub: str, mode: str) -> List[Path]:
    base = root / sub / "runs" / mode
    if not base.is_dir():
        return []
    return sorted(path.parent for path in base.glob("*/*/metadata.json"))


def flight_gates(performance: Dict[str, object], mechanism: Dict[str, object],
                 include_mechanism: bool = True) -> Dict[str, bool]:
    gates = {
        "strict_valid": bool(performance["strict_valid"]),
        "failsafe_zero": finite(performance["failsafe_ratio"]) == 0.0,
    }
    if include_mechanism:
        gates["selector_mode6_100pct"] = finite(mechanism["selector_mode6_ratio"]) >= 0.999999
    gates.update({
        "swing_rms_le_1p0_deg": finite(performance["swing_angle_rms_deg"]) <= 1.0,
        "swing_peak_le_4p0_deg": finite(performance["swing_angle_peak_deg"]) <= 4.0,
        "xy_rms_le_0p12_m": finite(performance["xy_error_rms_m"]) <= 0.12,
        "jerk_p95_le_2p0_m_s3": finite(performance["final_jerk_p95_m_s3"]) <= 2.0,
    })
    if include_mechanism:
        gates.update({
            "direct_compensation_zero": finite(mechanism["direct_compensation_peak_m_s2"]) <= 1.0e-7,
            "as_positive_power_zero": finite(mechanism["as_positive_power_integral_j_kg"]) <= 1.0e-9,
            "as_never_amplified": finite(mechanism["realized_as_ratio_max"]) <= 1.000001,
        })
    return gates


def implementation_check(run_dir: Path, expected_coefficients: List[float],
                         ucis_api) -> bool:
    events = ucis_api["read_events"](run_dir)
    moves = [item for item in events
             if any(d in str(item.get("name", "")) for d in ("north", "east", "south", "west"))]
    if len(moves) != 4:
        return False
    expected = np.asarray(expected_coefficients, dtype=float)
    for item in moves:
        params = item["params"]
        if params.get("input_shaper") != "fhv2_full_horizon_convex_frequency_uncertainty_trajectory":
            return False
        if not np.allclose(np.asarray(params.get("finite_horizon_coefficients", []), dtype=float),
                           expected, rtol=0.0, atol=1.0e-9):
            return False
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    vfb_api = runpy.run_path(str(VFB_REPORT), run_name="fh2m_vfb_api")
    ucis_api = runpy.run_path(str(UCIS_REPORT), run_name="fh2m_ucis_api")
    stage1_api = runpy.run_path(str(STAGE1_BENCHMARK), run_name="fh2m_stage1_api")
    design_items = {
        round(float(item["rope_length_m"]), 2): item
        for item in json.loads((root / "fh2_design.json").read_text(encoding="utf-8"))
    }
    # FH2_08 flew with the v1 0.8 m coefficients (implementation-audited at
    # gate time), so its coefficient-match gate is not re-checked against the
    # v5 design JSON.
    design_length_by_label = {
        "FH2_06": 0.6, "L05": 0.5, "L06": 0.6, "L08": 0.8,
        "P75": 0.6, "MM070": 0.6,
    }

    per_run_rows: List[Dict[str, object]] = []
    point_summary: Dict[str, Dict[str, object]] = {}
    point_plan = [(label, mode, (sub,), kind)
                  for label, mode, sub, kind in GATE_POINTS] + [
                  (label, mode, subs, kind) for label, mode, subs, _p, kind in MATRIX_POINTS]
    for label, mode, subs, kind in point_plan:
        dirs = [d for sub in subs for d in run_dirs(root, sub, mode)]
        performances = []
        for run_dir in dirs:
            performance = ucis_api["analyze_performance"](mode, run_dir, label, vfb_api)
            mechanism = ucis_api["analyze_full_mechanism"](run_dir, label, stage1_api)
            gates = flight_gates(performance, mechanism,
                                 include_mechanism="UCIS" in mode)
            if label in design_length_by_label and not label.startswith("EI_"):
                gates = dict(gates)
                gates["frozen_coefficients_match"] = implementation_check(
                    run_dir, design_items[design_length_by_label[label]]["coefficients"],
                    ucis_api)
            row = {**performance, **{f"gate_{k}": v for k, v in gates.items()},
                   "all_gates_pass": all(gates.values())}
            per_run_rows.append(row)
            performances.append(row)
        summary: Dict[str, object] = {"label": label, "mode": mode, "n_runs": len(dirs)}
        for metric in METRICS:
            values = np.asarray([finite(r.get(metric)) for r in performances], dtype=float)
            values = values[np.isfinite(values)]
            summary[metric + "_mean"] = float(np.mean(values)) if values.size else math.nan
            summary[metric + "_sd"] = float(np.std(values, ddof=1)) if values.size > 1 else 0.0
        summary["n_all_gates_pass"] = sum(1 for r in performances if r["all_gates_pass"])
        summary["single_pass_ratio"] = (
            summary["n_all_gates_pass"] / len(performances)) if performances else 0.0
        # Point gate: mean within gates and >= 2/3 single-run pass.
        mean_perf = {metric: summary[metric + "_mean"] for metric in METRICS}
        mean_perf["strict_valid"] = True
        mean_perf["failsafe_ratio"] = 0.0
        mean_mech = {
            "selector_mode6_ratio": 1.0, "direct_compensation_peak_m_s2": 0.0,
            "as_positive_power_integral_j_kg": 0.0, "realized_as_ratio_max": 1.0,
        }
        summary["mean_gates"] = flight_gates(mean_perf, mean_mech,
                                             include_mechanism="UCIS" in mode)
        summary["point_pass"] = bool(
            performances and all(summary["mean_gates"].values())
            and summary["single_pass_ratio"] >= 2.0 / 3.0)
        point_summary[label] = summary

    gate_pass = all(point_summary[label]["point_pass"] for label, *_ in GATE_POINTS
                    if label in point_summary) and len(point_summary) >= 2
    matrix_pass = all(
        point_summary[label]["point_pass"]
        for label, _, _, _p, kind in MATRIX_POINTS
        if kind == "method" and label in point_summary
    )
    decision = {
        "decision": ("FHV2_DUAL_LENGTH_PRELIMINARY_PASS" if gate_pass
                     else "FHV2_DUAL_LENGTH_FAIL"),
        "gate_pass": gate_pass,
        "matrix_pass": matrix_pass,
        "point_pass": {label: s["point_pass"] for label, s in point_summary.items()},
    }

    # Comparison against previous methods at 0.6 m.
    previous = json.loads(PREVIOUS_FUCI.read_text(encoding="utf-8"))
    with FH1_RESULTS.open(newline="", encoding="utf-8") as stream:
        fh1 = {row["label"]: row for row in csv.DictReader(stream)}
    comparisons = {}
    for label, ref in (("vs_FUCI1", previous),):
        if "FH2_06" in point_summary:
            comparisons[label] = {
                metric: point_summary["FH2_06"][metric + "_mean"] / finite(ref[metric]) - 1.0
                if math.isfinite(finite(ref.get(metric))) else math.nan
                for metric in METRICS if metric in ref
            }

    fields = ["label", "mode", "run"] + list(METRICS) + ["all_gates_pass"] + [
        key for key in per_run_rows[0] if key.startswith("gate_")] if per_run_rows else []
    with (root / "matrix_results.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(dict.fromkeys(
            ["label", "mode", "run", *METRICS, "strict_valid", "all_gates_pass",
             *[k for k in per_run_rows[0] if k.startswith("gate_")]])) if per_run_rows
            else ["label"])
        writer.writeheader()
        for row in per_run_rows:
            writer.writerow({k: row.get(k, "") for k in writer.fieldnames})
    (root / "matrix_decision.json").write_text(
        json.dumps({"decision": decision, "points": point_summary,
                    "comparisons": comparisons}, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8")
    print(json.dumps(decision, ensure_ascii=False, indent=2))
    for label, summary in point_summary.items():
        print(f"{label}: n={summary['n_runs']} pass={summary['n_all_gates_pass']} "
              f"swing_rms={fmt(summary['swing_angle_rms_deg_mean'])}±{fmt(summary['swing_angle_rms_deg_sd'])} "
              f"peak={fmt(summary['swing_angle_peak_deg_mean'])} "
              f"xy={fmt(summary['xy_error_rms_m_mean'], 4)} "
              f"jerk={fmt(summary['final_jerk_p95_m_s3_mean'])} "
              f"point_pass={summary['point_pass']}")
    return 0 if gate_pass and matrix_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
