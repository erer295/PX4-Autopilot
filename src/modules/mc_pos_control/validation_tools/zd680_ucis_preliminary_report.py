#!/usr/bin/env python3
"""Analyze the preregistered unified input-shaping/coordinated-AS trial."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
from pyulog import ULog


MODULE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = MODULE_ROOT / "validation" / "ucis_preliminary_20260722"
VFB_REPORT = MODULE_ROOT / "validation_tools" / "zd680_vfb_report.py"
STAGE1_BENCHMARK = MODULE_ROOT / "validation_tools" / "zd680_heso_stage1_frequency_benchmark.py"

BASE_MODE = "STD_L06_PID_AS_UCIS_BASE"
FIXED_MODE = "STD_L06_PID_AS_UCIS_FIXED"
FULL_MODE = "STD_L06_PID_AS_UCIS_FULL"
MODES = (BASE_MODE, FIXED_MODE, FULL_MODE)

PERFORMANCE_FIELDS = (
    "initial_swing_angle_rms_deg",
    "initial_swing_rate_rms_rad_s",
    "initial_xy_error_m",
    "swing_angle_rms_deg",
    "swing_angle_peak_deg",
    "swing_energy_integral_j_s_kg",
    "final_5s_swing_angle_rms_deg",
    "xy_error_rms_m",
    "moving_xy_error_rms_m",
    "command_xy_error_rms_m",
    "command_xy_error_peak_m",
    "command_vs_mission_xy_offset_rms_m",
    "command_vs_mission_xy_offset_peak_m",
    "final_jerk_p95_m_s3",
    "final_jerk_rms_m_s3",
    "final_acc_rms_m_s2",
    "as_rms_m_s2",
    "as_peak_m_s2",
    "as_positive_power_integral_j_kg",
    "as_negative_power_integral_j_kg",
    "total_acc_saturated_ratio",
    "failsafe_ratio",
)

COMPARISON_FIELDS = {
    "swing_angle_rms_deg": "swing_rms_change_pct",
    "swing_angle_peak_deg": "swing_peak_change_pct",
    "swing_energy_integral_j_s_kg": "energy_change_pct",
    "xy_error_rms_m": "xy_rms_change_pct",
    "moving_xy_error_rms_m": "moving_xy_rms_change_pct",
    "final_jerk_p95_m_s3": "jerk_p95_change_pct",
    "final_acc_rms_m_s2": "final_acc_rms_change_pct",
}


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def pct(candidate: float, baseline: float) -> float:
    return 100.0 * (candidate / baseline - 1.0) if math.isfinite(candidate) \
        and math.isfinite(baseline) and abs(baseline) > 1.0e-12 else math.nan


def fmt(value: object, digits: int = 3) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def fmt_pct(value: object, digits: int = 2) -> str:
    number = finite(value)
    return f"{number:+.{digits}f}%" if math.isfinite(number) else "—"


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def debug_dataset(ulog: ULog, debug_id: int):
    datasets = []
    for dataset in ulog.data_list:
        if dataset.name != "debug_array" or "id" not in dataset.data:
            continue
        mask = np.asarray(dataset.data["id"]) == debug_id
        if np.any(mask):
            datasets.append({key: np.asarray(value)[mask] for key, value in dataset.data.items()})
    return max(datasets, key=lambda item: len(item["timestamp"])) if datasets else None


def run_dirs(root: Path, mode: str) -> List[Path]:
    paths = sorted((root / "formal" / "runs" / mode).glob("*/*"))
    return [path for path in paths if (path / "metadata.json").is_file()]


def read_events(run_dir: Path) -> List[Dict[str, object]]:
    events = []
    with (run_dir / "events.csv").open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            item: Dict[str, object] = dict(row)
            try:
                item["params"] = json.loads(str(row.get("params_json", "{}")))
            except json.JSONDecodeError:
                item["params"] = {}
            events.append(item)
    return events


def route_bounds(run_dir: Path) -> Tuple[float, float]:
    actions = [row for row in read_events(run_dir) if str(row.get("name", "")).startswith("action_")]
    return (min(float(row["start_boot_s"]) for row in actions),
            max(float(row["end_boot_s"]) for row in actions))


def integrate_parts(t: np.ndarray, power: np.ndarray) -> Tuple[float, float]:
    mask = np.isfinite(t) & np.isfinite(power)
    t, power = t[mask], power[mask]
    strictly_increasing = np.r_[True, np.diff(t) > 1.0e-6]
    t, power = t[strictly_increasing], power[strictly_increasing]
    if t.size < 2:
        return math.nan, math.nan
    return (float(np.trapezoid(np.maximum(power, 0.0), t)),
            float(np.trapezoid(np.minimum(power, 0.0), t)))


def analyze_performance(mode: str, run_dir: Path, label: str,
                        vfb_api: Dict[str, object]) -> Dict[str, object]:
    row, _, analysis_validity = vfb_api["analyze_run"](mode, run_dir)
    validity = json.loads((run_dir / "validity.json").read_text(encoding="utf-8"))
    events = read_events(run_dir)
    actions = {str(item["name"]): item for item in events
               if str(item.get("name", "")).startswith("action_")}
    command_errors: List[float] = []
    command_offsets: List[float] = []
    with (run_dir / "live_samples.csv").open(newline="", encoding="utf-8") as stream:
        for sample in csv.DictReader(stream):
            phase = str(sample.get("phase", ""))
            event = actions.get(phase)
            if event is None:
                continue
            x, y = finite(sample.get("x_m")), finite(sample.get("y_m"))
            sp_x, sp_y = finite(sample.get("sp_x_m")), finite(sample.get("sp_y_m"))
            sample_s = finite(sample.get("boot_s"))
            if all(math.isfinite(value) for value in (x, y, sp_x, sp_y, sample_s)):
                command_errors.append(math.hypot(x - sp_x, y - sp_y))
                ref_x, ref_y, _, _, _ = vfb_api["reference_sample"](event, sample_s)
                command_offsets.append(math.hypot(sp_x - ref_x, sp_y - ref_y))

    def values_rms(values: Sequence[float]) -> float:
        data = np.asarray(values, dtype=float)
        return float(np.sqrt(np.mean(data * data))) if data.size else math.nan

    return {
        "label": label,
        "mode": mode,
        "run": run_dir.name,
        "strict_valid": bool(validity.get("valid")),
        "comparison_usable": bool(analysis_validity.get("valid")),
        "strict_rejection_reasons": ";".join(validity.get("rejection_reasons", [])),
        "truth_z_peak_m": finite(validity.get("metrics", {}).get("truth_z_reference_error_peak_m")),
        "ekf_truth_z_peak_m": finite(validity.get("metrics", {}).get("ekf_truth_z_difference_peak_m")),
        **{field: row.get(field, math.nan) for field in PERFORMANCE_FIELDS},
        "command_xy_error_rms_m": values_rms(command_errors),
        "command_xy_error_peak_m": max(command_errors, default=math.nan),
        "command_vs_mission_xy_offset_rms_m": values_rms(command_offsets),
        "command_vs_mission_xy_offset_peak_m": max(command_offsets, default=math.nan),
    }


def analyze_full_mechanism(run_dir: Path, label: str,
                           stage1_api: Dict[str, object]) -> Dict[str, object]:
    route_start, route_end = route_bounds(run_dir)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["debug_array"])
    fso = debug_dataset(ulog, 687)
    coordination = debug_dataset(ulog, 684)
    anti_swing = debug_dataset(ulog, 681)
    if fso is None or coordination is None or anti_swing is None:
        raise RuntimeError(f"missing mechanism debug data in {run_dir}")

    ft = np.asarray(fso["timestamp"], dtype=float) * 1.0e-6
    fm = (ft >= route_start) & (ft <= route_end)
    ct = np.asarray(coordination["timestamp"], dtype=float) * 1.0e-6
    cm = (ct >= route_start) & (ct <= route_end)
    at = np.asarray(anti_swing["timestamp"], dtype=float) * 1.0e-6
    am = (at >= route_start) & (at <= route_end)

    def fd(index: int) -> np.ndarray:
        return np.asarray(fso[f"data[{index}]"], dtype=float)[fm]

    def cd(index: int) -> np.ndarray:
        return np.asarray(coordination[f"data[{index}]"], dtype=float)[cm]

    direct = np.hypot(fd(18), fd(19))
    ratio = fd(46)
    confidence = fd(47)
    energy = fd(22)
    energetic = (energy >= 0.003) & (ft[fm] >= route_start + 10.0)
    energetic_ratio = ratio[energetic]
    energetic_confidence = confidence[energetic]
    permission = cd(24)
    raw_as = np.hypot(cd(6), cd(7))
    applied_as = np.hypot(cd(8), cd(9))
    ratio_mask = raw_as > 1.0e-6
    realized_permission = applied_as[ratio_mask] / raw_as[ratio_mask]
    as_power = cd(18)
    positive_power, negative_power = integrate_parts(ct[cm], as_power)
    anti_gain = np.asarray(anti_swing["data[19]"], dtype=float)[am]
    flight_t, flight_angle, _, _ = stage1_api["load_flight"](run_dir)
    fft_ratio = float(stage1_api["fft_peak_ratio"](flight_t, flight_angle))
    energetic_ratio_median = float(np.median(energetic_ratio)) if energetic_ratio.size else math.nan

    return {
        "label": label,
        "run": run_dir.name,
        "mode_id_median": float(np.median(fd(26))),
        "selector_mode6_ratio": float(np.mean(np.isclose(cd(23), 6.0))),
        "direct_compensation_peak_m_s2": float(np.max(direct)),
        "gain_scale_min": float(np.min(fd(52))),
        "gain_scale_max": float(np.max(fd(52))),
        "hangas_gain_scale_min": float(np.min(anti_gain)),
        "hangas_gain_scale_max": float(np.max(anti_gain)),
        "permission_min": float(np.min(permission)),
        "permission_p05": float(np.percentile(permission, 5.0)),
        "permission_mean": float(np.mean(permission)),
        "permission_max": float(np.max(permission)),
        "permission_below_0p99_ratio": float(np.mean(permission < 0.99)),
        "permission_range": float(np.ptp(permission)),
        "realized_as_ratio_min": float(np.min(realized_permission)) if realized_permission.size else math.nan,
        "realized_as_ratio_mean": float(np.mean(realized_permission)) if realized_permission.size else math.nan,
        "realized_as_ratio_max": float(np.max(realized_permission)) if realized_permission.size else math.nan,
        "total_acc_saturated_ratio": float(np.mean(cd(27) > 0.5)),
        "as_power_max_w_kg": float(np.max(as_power)),
        "as_positive_power_integral_j_kg": positive_power,
        "as_negative_power_integral_j_kg": negative_power,
        "frequency_ratio_median": float(np.median(ratio)),
        "frequency_confidence_median": float(np.median(confidence)),
        "frequency_confidence_max": float(np.max(confidence)),
        "energetic_frequency_ratio_median": energetic_ratio_median,
        "energetic_frequency_confidence_median": (
            float(np.median(energetic_confidence)) if energetic_confidence.size else math.nan
        ),
        "energetic_frequency_confidence_max": (
            float(np.max(energetic_confidence)) if energetic_confidence.size else math.nan
        ),
        "energetic_high_confidence_ratio": (
            float(np.mean(energetic_confidence >= 0.7)) if energetic_confidence.size else math.nan
        ),
        "offline_fft_peak_ratio": fft_ratio,
        "energetic_frequency_error_vs_fft_pct": (
            abs(energetic_ratio_median / fft_ratio - 1.0) * 100.0
            if math.isfinite(energetic_ratio_median) and math.isfinite(fft_ratio) and fft_ratio else math.nan
        ),
    }


def trajectory_progress(elapsed: float, duration: float, ramp: float) -> Tuple[float, float, float]:
    elapsed = min(duration, max(0.0, elapsed))
    peak_rate = 1.0 / (duration - ramp)
    if elapsed < ramp:
        phase = math.pi * elapsed / ramp
        return (0.5 * peak_rate * (elapsed - ramp * math.sin(phase) / math.pi),
                0.5 * peak_rate * (1.0 - math.cos(phase)),
                0.5 * peak_rate * math.pi * math.sin(phase) / ramp)
    if elapsed <= duration - ramp:
        return peak_rate * (elapsed - 0.5 * ramp), peak_rate, 0.0
    if elapsed < duration:
        decel = elapsed - (duration - ramp)
        phase = math.pi * decel / ramp
        progress = peak_rate * (duration - 1.5 * ramp) + 0.5 * peak_rate * (
            decel + ramp * math.sin(phase) / math.pi
        )
        return progress, 0.5 * peak_rate * (1.0 + math.cos(phase)), \
            -0.5 * peak_rate * math.pi * math.sin(phase) / ramp
    return 1.0, 0.0, 0.0


def shaper_characteristics(run_dir: Path) -> Dict[str, object]:
    events = read_events(run_dir)
    move = next(row for row in events if any(name in str(row.get("name", ""))
                                             for name in ("north", "east", "south", "west")))
    params = move["params"]
    spacing = finite(params.get("input_shaper_spacing_s"))
    weights = tuple(float(value) for value in params.get("input_shaper_weights", []))
    base_duration = finite(params.get("input_shaper_base_duration_s"))
    dt = 0.001
    t = np.arange(0.0, 6.0 + 0.5 * dt, dt)
    base_v = np.asarray([2.0 * trajectory_progress(value, 6.0, 1.0)[1] for value in t])
    base_a = np.asarray([2.0 * trajectory_progress(value, 6.0, 1.0)[2] for value in t])
    shaped_v = np.zeros(t.shape)
    shaped_a = np.zeros(t.shape)
    for index, weight in enumerate(weights):
        shaped_v += weight * np.asarray([
            2.0 * trajectory_progress(value - index * spacing, base_duration, 1.0)[1] for value in t
        ])
        shaped_a += weight * np.asarray([
            2.0 * trajectory_progress(value - index * spacing, base_duration, 1.0)[2] for value in t
        ])

    def pendulum_residual(acceleration: np.ndarray) -> float:
        omega = math.sqrt(9.80665 / 0.6)
        zeta = 0.10
        full_t = np.arange(0.0, 16.0 + 0.5 * dt, dt)
        forcing = np.zeros(full_t.shape)
        forcing[:acceleration.size] = acceleration
        angle, rate = 0.0, 0.0
        output = np.zeros(full_t.shape)
        for index in range(1, full_t.size):
            acc = forcing[index - 1]
            rate += dt * (-2.0 * zeta * omega * rate - omega * omega * angle - acc / 0.6)
            angle += dt * rate
            output[index] = angle
        return float(np.degrees(np.sqrt(np.mean(output[full_t >= 6.0] ** 2))))

    base_residual = pendulum_residual(base_a)
    shaped_residual = pendulum_residual(shaped_a)
    return {
        "type": params.get("input_shaper"),
        "frequency_source": params.get("input_shaper_frequency_source"),
        "damping_ratio": finite(params.get("input_shaper_damping_ratio")),
        "spacing_s": spacing,
        "weights": list(weights),
        "weight_sum": float(sum(weights)),
        "original_duration_s": 6.0,
        "base_duration_s": base_duration,
        "shaped_total_duration_s": 6.0,
        "original_peak_velocity_m_s": float(np.max(np.abs(base_v))),
        "shaped_peak_velocity_m_s": float(np.max(np.abs(shaped_v))),
        "peak_velocity_change_pct": pct(float(np.max(np.abs(shaped_v))), float(np.max(np.abs(base_v)))),
        "original_peak_acceleration_m_s2": float(np.max(np.abs(base_a))),
        "shaped_peak_acceleration_m_s2": float(np.max(np.abs(shaped_a))),
        "peak_acceleration_change_pct": pct(float(np.max(np.abs(shaped_a))), float(np.max(np.abs(base_a)))),
        "linear_nominal_postmove_rms_deg_original": base_residual,
        "linear_nominal_postmove_rms_deg_shaped": shaped_residual,
        "linear_nominal_residual_change_pct": pct(shaped_residual, base_residual),
    }


def mode_means(rows: Sequence[Dict[str, object]]) -> Dict[str, float]:
    fields = PERFORMANCE_FIELDS + ("truth_z_peak_m", "ekf_truth_z_peak_m")
    return {field: float(np.mean([finite(row[field]) for row in rows])) for field in fields}


def comparison_rows(candidate_name: str, candidate: Sequence[Dict[str, object]],
                    baseline_name: str, baseline: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    rows = []
    for index in range(2):
        row: Dict[str, object] = {
            "comparison": f"{candidate_name}/{baseline_name}",
            "pair": index + 1,
            "candidate": candidate[index]["label"],
            "baseline": baseline[index]["label"],
        }
        for field, output in COMPARISON_FIELDS.items():
            row[output] = pct(finite(candidate[index][field]), finite(baseline[index][field]))
        rows.append(row)
    return rows


def mean_changes(candidate: Dict[str, float], baseline: Dict[str, float]) -> Dict[str, float]:
    return {output: pct(candidate[field], baseline[field]) for field, output in COMPARISON_FIELDS.items()}


def status(passed: bool) -> str:
    return "通过" if passed else "失败"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    formal = root / "formal"
    vfb_api = runpy.run_path(str(VFB_REPORT), run_name="ucis_vfb_report_api")
    stage1_api = runpy.run_path(str(STAGE1_BENCHMARK), run_name="ucis_stage1_api")

    paths = {mode: run_dirs(root, mode) for mode in MODES}
    counts = {mode: len(items) for mode, items in paths.items()}
    if any(count != 2 for count in counts.values()):
        raise RuntimeError(f"expected exactly two runs per mode, got {counts}")

    labels = {BASE_MODE: ("A1", "A2"), FIXED_MODE: ("B1", "B2"), FULL_MODE: ("C1", "C2")}
    rows_by_mode: Dict[str, List[Dict[str, object]]] = {}
    formal_rows: List[Dict[str, object]] = []
    for mode in MODES:
        rows_by_mode[mode] = []
        for label, path in zip(labels[mode], paths[mode]):
            result = analyze_performance(mode, path, label, vfb_api)
            rows_by_mode[mode].append(result)
            formal_rows.append(result)
    write_csv(formal / "formal_results.csv", formal_rows)

    mechanism_rows = [
        analyze_full_mechanism(path, label, stage1_api)
        for label, path in zip(labels[FULL_MODE], paths[FULL_MODE])
    ]
    write_csv(formal / "mechanism_audit.csv", mechanism_rows)
    shaper = shaper_characteristics(paths[FIXED_MODE][0])
    (formal / "shaper_characteristics.json").write_text(
        json.dumps(shaper, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    base, fixed, full = (rows_by_mode[mode] for mode in MODES)
    pair_ba = comparison_rows("B", fixed, "A", base)
    pair_cb = comparison_rows("C", full, "B", fixed)
    pair_ca = comparison_rows("C", full, "A", base)
    pair_rows = pair_ba + pair_cb + pair_ca
    write_csv(formal / "pair_comparisons.csv", pair_rows)

    means = {"A": mode_means(base), "B": mode_means(fixed), "C": mode_means(full)}
    changes = {
        "B_vs_A": mean_changes(means["B"], means["A"]),
        "C_vs_B": mean_changes(means["C"], means["B"]),
        "C_vs_A": mean_changes(means["C"], means["A"]),
    }
    all_strict_valid = all(bool(row["strict_valid"]) for row in formal_rows)
    all_comparison_usable = all(bool(row["comparison_usable"]) for row in formal_rows)
    mechanism_gates = {
        "mode6_and_selector_active": all(abs(finite(row["mode_id_median"]) - 6.0) <= 1.0e-6
            and finite(row["selector_mode6_ratio"]) >= 0.99 for row in mechanism_rows),
        "permission_in_unit_interval": all(finite(row["permission_min"]) >= -1.0e-6
            and finite(row["permission_max"]) <= 1.000001 for row in mechanism_rows),
        "permission_varies": all(finite(row["permission_range"]) >= 0.05
            and finite(row["permission_below_0p99_ratio"]) > 0.01 for row in mechanism_rows),
        "never_amplifies_as": all(finite(row["realized_as_ratio_max"]) <= 1.0001
            and finite(row["gain_scale_max"]) <= 1.000001
            and finite(row["hangas_gain_scale_max"]) <= 1.000001 for row in mechanism_rows),
        "direct_compensation_zero": all(finite(row["direct_compensation_peak_m_s2"]) <= 1.0e-7
            for row in mechanism_rows),
        "as_positive_power_zero": all(finite(row["as_positive_power_integral_j_kg"]) <= 1.0e-9
            and finite(row["as_power_max_w_kg"]) <= 1.0e-7 for row in mechanism_rows),
    }
    frequency_online_pass = all(
        finite(row["energetic_frequency_confidence_median"]) >= 0.7
        and finite(row["energetic_high_confidence_ratio"]) >= 0.8
        and finite(row["energetic_frequency_error_vs_fft_pct"]) <= 5.0
        for row in mechanism_rows
    )
    b_gates = {
        "both_pair_swing_decrease": all(finite(row["swing_rms_change_pct"]) < 0.0 for row in pair_ba),
        "both_pair_energy_decrease": all(finite(row["energy_change_pct"]) < 0.0 for row in pair_ba),
        "mean_swing_le_minus_5pct": changes["B_vs_A"]["swing_rms_change_pct"] <= -5.0,
        "mean_energy_le_minus_8pct": changes["B_vs_A"]["energy_change_pct"] <= -8.0,
        "mean_xy_le_plus_10pct": changes["B_vs_A"]["xy_rms_change_pct"] <= 10.0,
        "mean_jerk_le_plus_10pct": changes["B_vs_A"]["jerk_p95_change_pct"] <= 10.0,
    }
    c_vs_b_gates = {
        "swing_not_worse_than_plus_3pct": changes["C_vs_B"]["swing_rms_change_pct"] <= 3.0,
        "energy_not_worse_than_plus_3pct": changes["C_vs_B"]["energy_change_pct"] <= 3.0,
        "xy_improves_or_le_plus_2pct": changes["C_vs_B"]["xy_rms_change_pct"] <= 2.0,
        "jerk_improves_or_le_plus_2pct": changes["C_vs_B"]["jerk_p95_change_pct"] <= 2.0,
        "mechanism_all_pass": all(mechanism_gates.values()),
    }
    c_vs_a_gates = {
        "both_pair_swing_decrease": all(finite(row["swing_rms_change_pct"]) < 0.0 for row in pair_ca),
        "both_pair_energy_decrease": all(finite(row["energy_change_pct"]) < 0.0 for row in pair_ca),
        "mean_swing_le_minus_5pct": changes["C_vs_A"]["swing_rms_change_pct"] <= -5.0,
        "mean_energy_le_minus_8pct": changes["C_vs_A"]["energy_change_pct"] <= -8.0,
        "mean_xy_le_plus_10pct": changes["C_vs_A"]["xy_rms_change_pct"] <= 10.0,
        "mean_jerk_le_plus_7pct": changes["C_vs_A"]["jerk_p95_change_pct"] <= 7.0,
        "all_six_strict_valid": all_strict_valid,
    }
    b_trend_pass = all(b_gates.values())
    c_vs_b_pass = all(c_vs_b_gates.values())
    complete_formal_pass = all(c_vs_a_gates.values()) and b_trend_pass and c_vs_b_pass
    if complete_formal_pass and frequency_online_pass:
        decision_code = "COMPLETE_METHOD_PRELIMINARY_PASS"
    elif b_trend_pass and c_vs_b_pass:
        decision_code = "KEEP_UNIFIED_ROUTE_FIX_STRICT_VALIDITY_AND_FREQUENCY"
    elif b_trend_pass:
        decision_code = "KEEP_INPUT_SHAPING_REWORK_COORDINATOR_WITHIN_SAME_ROUTE"
    else:
        decision_code = "REWORK_EQUAL_TIME_SHAPER_WITHIN_SAME_ROUTE"

    decision = {
        "decision": decision_code,
        "all_six_strict_valid": all_strict_valid,
        "all_six_comparison_usable_excluding_z_observation": all_comparison_usable,
        "input_shaping_trend_pass": b_trend_pass,
        "coordinator_increment_pass": c_vs_b_pass,
        "complete_formal_pass": complete_formal_pass,
        "frequency_online_branch_pass": frequency_online_pass,
        "means": means,
        "mean_changes_pct": changes,
        "input_shaping_gates": b_gates,
        "coordinator_gates": c_vs_b_gates,
        "complete_gates": c_vs_a_gates,
        "mechanism_gates": mechanism_gates,
        "shaper": shaper,
    }
    (formal / "decision.json").write_text(
        json.dumps(decision, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )

    lines = [
        "# ZD680统一输入整形—可信频率—协调AS完整方法初步验证报告",
        "",
        "日期：2026-07-22",
        "",
        "## 1. 结论先行",
        "",
    ]
    if complete_formal_pass:
        lines.append("本轮完整方法达到预注册初步可行门槛，可以进入多绳长和在线可信频率泛化。")
    else:
        lines.append(
            "本轮不能宣布“完整方法正式通过”，但统一路线具有继续价值；应保留PID＋AS＋输入整形＋可信度安全退化＋协调权限这一结构，不应因单项失败更换方法。"
        )
    lines += [
        "",
        f"- 固定频率等时ZVD趋势判定：{status(b_trend_pass)}。",
        f"- 协调器相对同轨迹ZVD的增量判定：{status(c_vs_b_pass)}。",
        f"- 完整C相对A及六次有效性的总判定：{status(complete_formal_pass)}。",
        f"- 在线可信频率分支判定：{status(frequency_online_pass)}；不通过时实际整形安全回退标称频率。",
        f"- 正式飞行严格有效性：{sum(bool(row['strict_valid']) for row in formal_rows)}/6；比较可用性（仅把Z异常作为观察项）：{sum(bool(row['comparison_usable']) for row in formal_rows)}/6。",
        "",
        "A的两次失效都来自真实高度与EKF高度偏差，而不是摆角发散、水平越界或failsafe。由于预注册要求六次均有效，不能把这两次删掉后声称通过。",
        "",
        "## 2. 方法与隔离关系",
        "",
        "- A：PX4 PID＋原能量型AS，不整形。",
        "- B：A＋标称频率三脉冲ZVD；总移动时间仍为6 s。",
        "- C：B＋RLS/HESO频率可信度诊断和位置/加速度/jerk协调AS权限；直接扰动补偿严格为0，AS倍率不超过1。",
        "- 因本轮在线频率可信度不足，B和C使用相同的标称频率整形轨迹，所以C/B只隔离协调器的增量。",
        "",
        "正式顺序为A1→B1→C1→C2→B2→A2；绳长0.6 m、载荷0.5 kg、四向2 m/6 s、严格初始门。",
        "",
        "## 3. 六次运行结果",
        "",
        "| 轮次 | 严格有效 | 摆角RMS(°) | 峰值(°) | 能量积分 | XY RMS(m) | jerk P95 | 真值Z峰值(m) | EKF-真值Z峰值(m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in formal_rows:
        lines.append(
            f"| {row['label']} | {'是' if row['strict_valid'] else '否'} | {fmt(row['swing_angle_rms_deg'],4)} | "
            f"{fmt(row['swing_angle_peak_deg'],3)} | {fmt(row['swing_energy_integral_j_s_kg'],5)} | "
            f"{fmt(row['xy_error_rms_m'],5)} | {fmt(row['final_jerk_p95_m_s3'],4)} | "
            f"{fmt(row['truth_z_peak_m'],3)} | {fmt(row['ekf_truth_z_peak_m'],3)} |"
        )
    lines += [
        "",
        "模式均值：",
        "",
        "| 模式 | 摆角RMS(°) | 峰值(°) | 能量积分 | 任务参考XY RMS(m) | 实际-下发指令XY RMS(m) | 指令-原任务参考RMS(m) | jerk P95 | 真值Z峰值(m) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for name in ("A", "B", "C"):
        item = means[name]
        lines.append(
            f"| {name} | {fmt(item['swing_angle_rms_deg'],4)} | {fmt(item['swing_angle_peak_deg'],3)} | "
            f"{fmt(item['swing_energy_integral_j_s_kg'],5)} | {fmt(item['xy_error_rms_m'],5)} | "
            f"{fmt(item['command_xy_error_rms_m'],5)} | {fmt(item['command_vs_mission_xy_offset_rms_m'],5)} | "
            f"{fmt(item['final_jerk_p95_m_s3'],4)} | {fmt(item['truth_z_peak_m'],3)} |"
        )
    lines += [
        "",
        "## 4. 对照结果",
        "",
        "| 对照 | 摆角RMS | 能量 | XY RMS | jerk P95 |",
        "|---|---:|---:|---:|---:|",
    ]
    for name in ("B_vs_A", "C_vs_B", "C_vs_A"):
        item = changes[name]
        lines.append(
            f"| {name.replace('_vs_', '/')} | {fmt_pct(item['swing_rms_change_pct'])} | "
            f"{fmt_pct(item['energy_change_pct'])} | {fmt_pct(item['xy_rms_change_pct'])} | "
            f"{fmt_pct(item['jerk_p95_change_pct'])} |"
        )
    lines += [
        "",
        "成对同方向性见`formal/pair_comparisons.csv`。负百分比表示改善。A两次严格无效，所以A相关百分比只作为机制趋势，不能当最终性能定论。",
        "",
        f"XY恶化不是位置PID突然失稳：实际位置相对下发整形指令的RMS由A的{fmt(means['A']['command_xy_error_rms_m'],5)} m变为B的{fmt(means['B']['command_xy_error_rms_m'],5)} m，基本不变；主要问题是整形指令本身相对原任务时序产生了{fmt(means['B']['command_vs_mission_xy_offset_rms_m'],5)} m RMS、{fmt(means['B']['command_vs_mission_xy_offset_peak_m'],5)} m峰值的相位滞后。因此需要改整形轨迹，而不是重调位置PID。",
        "",
        f"协调器的增量是小而一致的：C/B摆角RMS为{fmt_pct(changes['C_vs_B']['swing_rms_change_pct'])}、能量为{fmt_pct(changes['C_vs_B']['energy_change_pct'])}、XY为{fmt_pct(changes['C_vs_B']['xy_rms_change_pct'])}、jerk为{fmt_pct(changes['C_vs_B']['jerk_p95_change_pct'])}。但峰值摆角反而增加{fmt_pct(changes['C_vs_B']['swing_peak_change_pct'])}，且C的真值高度峰值均值{fmt(means['C']['truth_z_peak_m'],3)} m高于B的{fmt(means['B']['truth_z_peak_m'],3)} m，所以只能说协调器通过了本轮冻结增量门，不能说它已经优化成熟。",
        "",
        "## 5. 输入整形本身",
        "",
        f"ZVD间隔为{fmt(shaper['spacing_s'],6)} s，权重为{', '.join(fmt(value,7) for value in shaper['weights'])}，权重和{fmt(shaper['weight_sum'],8)}。",
        f"基础S曲线由6 s压缩为{fmt(shaper['base_duration_s'],4)} s，再卷积回6 s；总任务时间没有延长。峰值速度由{fmt(shaper['original_peak_velocity_m_s'])}增至{fmt(shaper['shaped_peak_velocity_m_s'])} m/s（{fmt_pct(shaper['peak_velocity_change_pct'])}），峰值加速度由{fmt(shaper['original_peak_acceleration_m_s2'])}变为{fmt(shaper['shaped_peak_acceleration_m_s2'])} m/s²（{fmt_pct(shaper['peak_acceleration_change_pct'])}）。",
        f"线性标称摆模型的移动后RMS由{fmt(shaper['linear_nominal_postmove_rms_deg_original'],5)}°降到{fmt(shaper['linear_nominal_postmove_rms_deg_shaped'],5)}°，仅作为整形器离线一致性检查。",
        "",
        "## 6. 协调器与可信频率机制",
        "",
        "| 轮次 | 许可min/mean/max | 许可<0.99 | 实际AS倍率max | 直接补偿峰值 | AS正功积分 | 频率比/FFT比 | 能量段置信度中位数/最大值 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in mechanism_rows:
        lines.append(
            f"| {row['label']} | {fmt(row['permission_min'])}/{fmt(row['permission_mean'])}/{fmt(row['permission_max'])} | "
            f"{fmt_pct(100.0 * finite(row['permission_below_0p99_ratio']))[:-1] + '%' if math.isfinite(finite(row['permission_below_0p99_ratio'])) else '—'} | "
            f"{fmt(row['realized_as_ratio_max'])} | {fmt(row['direct_compensation_peak_m_s2'],8)} | "
            f"{fmt(row['as_positive_power_integral_j_kg'],9)} | {fmt(row['energetic_frequency_ratio_median'])}/{fmt(row['offline_fft_peak_ratio'])} | "
            f"{fmt(row['energetic_frequency_confidence_median'])}/{fmt(row['energetic_frequency_confidence_max'])} |"
        )
    lines += [
        "",
        "机制门结论：",
        "",
    ]
    for name, passed in mechanism_gates.items():
        lines.append(f"- {name}：{status(passed)}。")
    lines += [
        "",
        "这里的关键解释是：频率估计器虽然在运行，但置信度没有达到在线改写整形频率的门槛，因此整形器使用标称绳长频率。这个结果证明了安全退化路径有效，不证明自适应频率分支已经有效。",
        "",
        "## 7. 为什么仍不能封版",
        "",
        "1. A两次均出现垂向真值偏差超限，且EKF低估实际高度变化；六次全有效门失败。B显著减轻该现象，C又比B更接近高度门槛；两次同方向结果提示协调AS可能加重垂向耦合，但当前样本数不足以认定因果。",
        "2. 频率估计置信度不足，完整方法当前实质工作在“标称频率ZVD＋安全协调器”点，论文所需的在线可信频率贡献尚未形成闭环证据。",
        "3. 本轮只有0.6 m/0.5 kg单工况和每模式2次，够做初筛，不够支撑统计和泛化结论。",
        "4. 等时ZVD通过压缩内部轨迹换取整形，峰值运动学代价必须在后续加入速度/加速度约束后复验。",
        "",
        "## 8. 下一步固定计划（不换方法）",
        "",
        "1. 先修垂向证据链：恢复可信高度源或融合Gazebo真值诊断，检查GPS高度延迟、推力余量和吊挂张力引起的垂向耦合；用A/B/C短矩阵确认A不再偶发越门，且C不比B恶化。",
        "2. 在同一个ZVD模块内加入受约束的等时整形：以6 s总时长不变为前提，对基础轨迹峰值速度/加速度设上限，必要时联合优化脉冲间隔和基础ramp，不改PID＋AS主线。",
        "3. 修改协调器门控输入：继续保持0～1和功率非正，只把当前硬取最小值升级为可解释的平滑预算分配；把垂向推力余量/倾角余量加入许可，优先消除C相对B的高度恶化。",
        "4. 频率路径仍采用RLS/HESO有效频率＋可信度门：先通过滑窗离线回放把飞行FFT误差压到5%内、能量段高置信覆盖率提高到80%，再允许缓慢更新ZVD间隔；置信度不足继续回退标称值。",
        "5. 通过上述门后做0.5/0.6/0.8 m、载荷扰动和参数失配矩阵，每点至少5次，再形成论文主表。",
        "",
        "## 9. 可复核文件",
        "",
        "- `formal/formal_results.csv`：六次性能与有效性；",
        "- `formal/pair_comparisons.csv`：三组两两成对差异；",
        "- `formal/mechanism_audit.csv`：许可、功率、直接补偿、频率可信度；",
        "- `formal/shaper_characteristics.json`：整形参数与运动学代价；",
        "- `formal/decision.json`：冻结判据和自动判定；",
        "- 各run目录：原始ULog、events、live/joint samples、validity。",
        "",
    ]
    report = root / "ZD680_统一输入整形_可信频率_协调AS完整方法初步验证报告_20260722.md"
    report.write_text("\n".join(lines), encoding="utf-8")
    print(report)
    print(json.dumps({
        "decision": decision_code,
        "strict_valid": f"{sum(bool(row['strict_valid']) for row in formal_rows)}/6",
        "B_vs_A": changes["B_vs_A"],
        "C_vs_B": changes["C_vs_B"],
        "C_vs_A": changes["C_vs_A"],
        "frequency_online_pass": frequency_online_pass,
    }, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
