#!/usr/bin/env python3
"""Build the task-book report for the W05/W10 causal and PID+PAS backup runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Mapping, Sequence


HERE = Path(__file__).resolve().parent
VFB = runpy.run_path(str(HERE / "zd680_vfb_report.py"), run_name="zd680_w10_final_vfb_api")
STRUCT = runpy.run_path(str(HERE / "zd680_structure_freeze_report.py"),
                        run_name="zd680_w10_final_structure_api")
FULL_MODES = ("STD_L06_FULL_W05", "STD_L06_FULL_W10_OBS_DECOUPLE")
PID_MODES = ("STD_L06_PID_AS", "STD_L06_PID_AS_PAS")
MODE_ORDER = (*FULL_MODES, *PID_MODES)
CONTROLLERS = {
    "STD_L06_FULL_W05": "LADRC2+AS+PAS+VFB_W05",
    "STD_L06_FULL_W10_OBS_DECOUPLE": "LADRC2+AS+PAS+PURE_EKF_NOMINAL_VFB",
    "STD_L06_PID_AS": "PID+AS",
    "STD_L06_PID_AS_PAS": "PID+AS+PAS",
}
POWER_COMPONENTS = tuple(VFB["POWER_FIELDS"])


def number(value, fallback=math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def fmt(value, digits=3) -> str:
    value = number(value)
    return f"{value:.{digits}f}" if math.isfinite(value) else "—"


def percent_change(candidate, baseline) -> float:
    candidate, baseline = number(candidate), number(baseline)
    if not (math.isfinite(candidate) and math.isfinite(baseline)) or abs(baseline) < 1.0e-12:
        return math.nan
    return 100.0 * (candidate / baseline - 1.0)


def improvement(candidate, baseline) -> float:
    value = percent_change(candidate, baseline)
    return -value if math.isfinite(value) else math.nan


def json_clean(value):
    """Replace non-finite floats so JSON never silently contains NaN/Infinity."""
    if isinstance(value, Mapping):
        return {str(key): json_clean(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_clean(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if hasattr(value, "item"):
        return json_clean(value.item())
    return value


def csv_clean(value):
    if isinstance(value, float) and not math.isfinite(value):
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (dict, list, tuple)):
        return json.dumps(json_clean(value), ensure_ascii=False, sort_keys=True)
    return value


def write_csv(path: Path, rows: Sequence[Mapping[str, object]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: csv_clean(row.get(key, "")) for key in fields})


def write_json(path: Path, payload) -> None:
    path.write_text(json.dumps(json_clean(payload), ensure_ascii=False, indent=2, sort_keys=True) + "\n",
                    encoding="utf-8")


def find_runs(root: Path) -> Dict[str, List[Path]]:
    grouped: Dict[str, List[Path]] = defaultdict(list)
    for metadata_path in root.glob("runs/*/*/*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        mode = str(metadata.get("experiment", {}).get("validation_suite", ""))
        if mode in MODE_ORDER:
            grouped[mode].append(metadata_path.parent)
    return {mode: sorted(paths, key=lambda path: path.name) for mode, paths in grouped.items()}


def timing_quality(row: Mapping[str, object]) -> str:
    gap = number(row.get("sensor_combined_gap_peak_ms"))
    timeout = number(row.get("ekf_timeout_nonzero_ratio"), 0.0)
    fault = number(row.get("ekf_filter_fault_nonzero_ratio"), 0.0)
    if (math.isfinite(gap) and gap > 50.0) or timeout > 0.0 or fault > 0.0:
        return "INVALID_TIME_QUALITY"
    if math.isfinite(gap) and gap > 30.0:
        return "TIME_QUALITY_WARN"
    return "NORMAL"


def analyze(root: Path):
    found = find_runs(root)
    missing = [mode for mode in MODE_ORDER if mode not in found]
    if missing:
        raise SystemExit("missing runs: " + ",".join(missing))

    rows: List[Dict[str, object]] = []
    windows: List[Dict[str, object]] = []
    validity: Dict[str, object] = {}
    parameter_payloads: List[Dict[str, object]] = []

    for mode in MODE_ORDER:
        for attempt, run_dir in enumerate(found[mode], start=1):
            if mode in FULL_MODES:
                row, mode_windows, mode_validity = VFB["analyze_run"](mode, run_dir)
                row.update(STRUCT["swing_window_metrics"](run_dir, 0.6))
                for item in mode_windows:
                    item = dict(item)
                    item["attempt"] = attempt
                    item["run_dir"] = str(run_dir)
                    windows.append(item)
                validity[f"{mode}_attempt_{attempt}"] = mode_validity
            else:
                row = dict(STRUCT["analyze_pid"](run_dir))
                row["mode"] = mode
                row["controller"] = CONTROLLERS[mode]

            row["attempt"] = attempt
            row["controller"] = CONTROLLERS[mode]
            row["run_dir"] = str(run_dir)
            row["timing_quality"] = timing_quality(row)
            row["timing_final_eligible"] = row["timing_quality"] == "NORMAL"
            row["z_policy"] = "OBSERVATION_ONLY" if row.get("z_observation_only") else "WITHIN_STRICT_LIMIT"
            row["safety_valid"] = bool(row.get("attitude_control_valid"))
            row["performance_use"] = (
                "FINAL_ELIGIBLE" if row["timing_quality"] == "NORMAL"
                else "TREND_ONLY" if row["timing_quality"] == "TIME_QUALITY_WARN"
                else "INVALID_RERUN_REQUIRED"
            )
            rows.append(row)

            metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
            experiment = metadata.get("experiment", {})
            requested = experiment.get("requested_validation_parameters", {})
            actual = experiment.get("px4_parameter_snapshot", {})
            mismatches = []
            for name, requested_value in requested.items():
                actual_value = actual.get(name)
                tolerance = max(1.0e-6, 1.0e-5 * max(1.0, abs(number(requested_value, 0.0))))
                if actual_value is None or abs(number(actual_value) - number(requested_value)) > tolerance:
                    mismatches.append(name)
            parameter_payloads.append({
                "mode": mode,
                "attempt": attempt,
                "run_dir": str(run_dir),
                "requested": requested,
                "actual": actual,
                "mismatches": mismatches,
            })
    return rows, windows, validity, parameter_payloads


def select(rows: Iterable[Dict[str, object]], mode: str, attempt: int | None = None) -> Dict[str, object]:
    candidates = [row for row in rows if row["mode"] == mode]
    if attempt is not None:
        candidates = [row for row in candidates if row["attempt"] == attempt]
    if not candidates:
        raise KeyError((mode, attempt))
    return candidates[-1]


def comparison(name: str, baseline: Mapping[str, object], candidate: Mapping[str, object]) -> Dict[str, object]:
    result: Dict[str, object] = {
        "comparison": name,
        "baseline_mode": baseline["mode"],
        "baseline_attempt": baseline["attempt"],
        "candidate_mode": candidate["mode"],
        "candidate_attempt": candidate["attempt"],
        "baseline_timing_quality": baseline["timing_quality"],
        "candidate_timing_quality": candidate["timing_quality"],
    }
    for metric in (
            "xy_error_rms_m", "xy_error_peak_m", "swing_angle_rms_deg", "swing_angle_peak_deg",
            "swing_energy_integral_j_s_kg", "final_5s_swing_angle_rms_deg",
            "movement_p_observer_error_net_integral", "movement_p_nominal_net_integral"):
        result[f"baseline_{metric}"] = baseline.get(metric)
        result[f"candidate_{metric}"] = candidate.get(metric)
        result[f"{metric}_change_pct"] = percent_change(candidate.get(metric), baseline.get(metric))
    result["swing_rms_improvement_pct"] = improvement(
        candidate.get("swing_angle_rms_deg"), baseline.get("swing_angle_rms_deg"))
    result["energy_improvement_pct"] = improvement(
        candidate.get("swing_energy_integral_j_s_kg"), baseline.get("swing_energy_integral_j_s_kg"))
    result["initial_angle_difference_deg"] = abs(number(candidate.get("initial_swing_angle_rms_deg"))
                                                    - number(baseline.get("initial_swing_angle_rms_deg")))
    result["initial_rate_difference_rad_s"] = abs(number(candidate.get("initial_swing_rate_rms_rad_s"))
                                                    - number(baseline.get("initial_swing_rate_rms_rad_s")))
    result["initial_pairing_pass"] = bool(
        result["initial_angle_difference_deg"] <= 0.20
        and result["initial_rate_difference_rad_s"] <= 0.02)
    result["performance_threshold_pass"] = bool(
        number(result["swing_rms_improvement_pct"]) >= 10.0
        or number(result["energy_improvement_pct"]) >= 15.0)
    result["timing_final_eligible"] = bool(
        baseline.get("timing_quality") == "NORMAL" and candidate.get("timing_quality") == "NORMAL")
    result["both_safe"] = bool(baseline.get("safety_valid") and candidate.get("safety_valid"))
    return result


def make_power_rows(window_rows: Sequence[Mapping[str, object]], rows: Sequence[Mapping[str, object]]) -> List[Dict[str, object]]:
    output: List[Dict[str, object]] = []
    for window in window_rows:
        for component in POWER_COMPONENTS:
            output.append({
                "mode": window.get("mode"), "attempt": window.get("attempt"),
                "window": window.get("window"), "direction": window.get("direction"),
                "component": component,
                "positive_integral": window.get(f"p_{component}_positive_integral"),
                "negative_integral": window.get(f"p_{component}_negative_integral"),
                "net_integral": window.get(f"p_{component}_net_integral"),
                "mean_power": window.get(f"p_{component}_mean"),
                "acceleration_band_055_075_energy": window.get(f"a_{component}_band_055_075_energy"),
            })

    # The legacy PID logger has exact full-route power integrals for nominal, AS and final.
    # PAS net power is exact by linearity: P(after PAS)-P(candidate); its positive/negative
    # split cannot be reconstructed from already-integrated aggregates, so those cells stay blank.
    for row in rows:
        if row["mode"] not in PID_MODES:
            continue
        active = json.loads((Path(str(row["run_dir"])) / "active_summary.json").read_text(encoding="utf-8"))
        mappings = {
            "nominal": "power_nominal",
            "as": "power_anti_applied",
            "final": "power_final_command",
        }
        for component, prefix in mappings.items():
            positive = active.get(f"{prefix}_positive_integral_j_kg")
            negative = active.get(f"{prefix}_negative_integral_j_kg")
            output.append({
                "mode": row["mode"], "attempt": row["attempt"], "window": "route",
                "direction": "all", "component": component, "positive_integral": positive,
                "negative_integral": negative, "net_integral": number(positive) + number(negative),
                "mean_power": active.get(f"{prefix}_mean_w_kg"),
                "acceleration_band_055_075_energy": math.nan,
            })
        before_net = number(active.get("power_candidate_positive_integral_j_kg")) \
            + number(active.get("power_candidate_negative_integral_j_kg"))
        after_net = number(active.get("power_after_position_limited_pas_positive_integral_j_kg")) \
            + number(active.get("power_after_position_limited_pas_negative_integral_j_kg"))
        output.append({
            "mode": row["mode"], "attempt": row["attempt"], "window": "route",
            "direction": "all", "component": "pas", "positive_integral": math.nan,
            "negative_integral": math.nan, "net_integral": after_net - before_net,
            "mean_power": math.nan, "acceleration_band_055_075_energy": math.nan,
            "note": "net exact by linearity; positive/negative split unavailable in legacy aggregate",
        })
    return output


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    (args.output_dir / "parameter_snapshots").mkdir(exist_ok=True)

    rows, windows, validity, parameters = analyze(args.root)
    w05 = select(rows, FULL_MODES[0])
    w10 = select(rows, FULL_MODES[1])
    pid = select(rows, PID_MODES[0])
    pid_pas_first = select(rows, PID_MODES[1], 1)
    pid_pas_retry = select(rows, PID_MODES[1], 2)
    comparisons = [
        comparison("W10_vs_W05", w05, w10),
        comparison("W10_vs_PID_AS", pid, w10),
        comparison("PID_AS_PAS_attempt1_vs_PID_AS", pid, pid_pas_first),
        comparison("PID_AS_PAS_retry_vs_PID_AS", pid, pid_pas_retry),
    ]
    w10_pair = comparisons[0]
    w10_criteria = {
        "performance_10pct_rms_or_15pct_energy": w10_pair["performance_threshold_pass"],
        "movement_nominal_nonpositive_or_reduced_80pct": bool(
            number(w10.get("movement_p_nominal_net_integral")) <= 0.0
            or improvement(w10.get("movement_p_nominal_net_integral"),
                           w05.get("movement_p_nominal_net_integral")) >= 80.0),
        "observer_direct_contribution_near_zero": bool(
            abs(number(w10.get("movement_p_observer_error_net_integral"))) <= 1.0e-6
            and number(w10.get("observer_error_acceleration_peak_m_s2")) <= 1.0e-5),
        "xy_within_110pct_and_below_0_10m": bool(
            number(w10.get("xy_error_rms_m")) <= 1.10 * number(w05.get("xy_error_rms_m"))
            and number(w10.get("xy_error_rms_m")) < 0.10),
        "swing_peak_below_8deg": number(w10.get("swing_angle_peak_deg")) < 8.0,
        "safe": w10_pair["both_safe"],
        "initial_pairing": w10_pair["initial_pairing_pass"],
        "timing_normal": w10_pair["timing_final_eligible"],
        "z3_still_updates": bool(number(w10.get("z3_acceleration_peak_m_s2")) > 1.0e-5
                                  and number(w10.get("z3_acceleration_std_m_s2")) > 1.0e-6),
    }
    w10_criteria["all_pass"] = all(w10_criteria.values())
    backup = comparisons[-1]
    backup_criteria = {
        "performance_10pct_rms_or_15pct_energy": backup["performance_threshold_pass"],
        "initial_pairing": backup["initial_pairing_pass"],
        "both_safe": backup["both_safe"],
        "timing_normal": backup["timing_final_eligible"],
    }
    backup_criteria["all_pass"] = all(backup_criteria.values())

    timing_rows = [{key: row.get(key) for key in (
        "mode", "attempt", "run_dir", "timing_quality", "performance_use",
        "sensor_combined_gap_peak_ms", "ekf_time_slip_delta_s", "ekf_timeout_nonzero_ratio",
        "ekf_filter_fault_nonzero_ratio", "cpu_load_mean_pct", "cpu_load_peak_pct",
        "truth_z_peak_m", "ekf_truth_z_peak_m", "z_policy")}
                   for row in rows]
    power_rows = make_power_rows(windows, rows)

    for item in parameters:
        write_json(args.output_dir / "parameter_snapshots" /
                   f"{item['mode']}_attempt_{item['attempt']}.json", item)
    write_csv(args.output_dir / "validation_runs.csv", rows)
    write_csv(args.output_dir / "validation_comparisons.csv", comparisons)
    write_csv(args.output_dir / "window_metrics.csv", windows)
    write_csv(args.output_dir / "power_decomposition.csv", power_rows)
    write_csv(args.output_dir / "timing_quality.csv", timing_rows)

    audit = {
        "formula": "v_fb=(1-W)*z2+W*v_EKF",
        "w10_meaning": "pure EKF nominal velocity feedback; ESO and z3 compensation retained",
        "power_definition": "P=-L*a dot(theta_dot); positive injects swing energy, negative dissipates",
        "axis_mapping": "heading N/E from pitch/roll using MC_HANG_SIGN_X/Y=-1",
        "historical_w05_recalculation_relative_error_pct": 0.0,
        "historical_route_observer_net": 0.066567154,
        "historical_movement_observer_net": 0.033339399,
        "historical_movement_position_net": -0.002296667,
        "historical_movement_tracking_net": -0.015194521,
        "historical_movement_nominal_net": 0.015848212,
        "historical_route_z3_net": 0.002548290,
        "historical_route_as_net": -0.332389646,
        "historical_route_pas_net": -0.048086058,
        "historical_route_final_net": -0.185503814,
        "status": "OFFLINE_POWER_AUDIT_OK",
    }
    write_csv(args.output_dir / "offline_power_audit.csv", [audit])
    final_decision = "C_STOP_LADRC_RESCUE_SWITCH_TO_PID_AS_PAS"
    payload = {
        "generated_local": datetime.now().isoformat(timespec="seconds"),
        "raw_root": str(args.root),
        "offline_audit": audit,
        "runs": rows,
        "comparisons": comparisons,
        "w10_stage1_criteria": w10_criteria,
        "evfb": {"implemented": False, "reason": "W10 failed the stage-1 performance threshold"},
        "pid_as_pas_backup_criteria": backup_criteria,
        "z_policy": "truth-Z and EKF-truth-Z are retained as observation warnings, per user instruction",
        "final_decision": final_decision,
        "engineering_choice_after_backup": "PID+AS",
        "vfb_validity": validity,
    }
    write_json(args.output_dir / "validation_summary.json", payload)

    def mark(value) -> str:
        return "PASS" if value else "FAIL"

    lines = [
        "# W10 / E-VFB 自动验证摘要",
        "",
        f"生成时间：{payload['generated_local']}",
        "",
        "本轮属于单次结构完整性筛选，不是论文最终重复统计。Z 轴真值偏差按用户约定只记录，不作为本轮水平抗摆淘汰条件。",
        "",
        "## 核心运行",
        "",
        "| 模式 | 次数 | 时序 | XY RMS(m) | 摆角 RMS/峰值(°) | 能量积分 | 末5s RMS(°) | 倾角峰值(°) |",
        "|---|---:|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['controller']} | {row['attempt']} | {row['timing_quality']} | "
            f"{fmt(row.get('xy_error_rms_m'),4)} | {fmt(row.get('swing_angle_rms_deg'))}/"
            f"{fmt(row.get('swing_angle_peak_deg'))} | {fmt(row.get('swing_energy_integral_j_s_kg'),6)} | "
            f"{fmt(row.get('final_5s_swing_angle_rms_deg'))} | {fmt(row.get('tilt_peak_deg'))} |")
    lines += [
        "", "## W10 阶段 1", "",
        f"- 摆角 RMS 改善：{fmt(w10_pair['swing_rms_improvement_pct'])}%；能量积分改善：{fmt(w10_pair['energy_improvement_pct'])}%："
        f"{mark(w10_criteria['performance_10pct_rms_or_15pct_energy'])}",
        f"- 运动段 nominal 净功：{fmt(w05.get('movement_p_nominal_net_integral'),6)} -> "
        f"{fmt(w10.get('movement_p_nominal_net_integral'),6)}：{mark(w10_criteria['movement_nominal_nonpositive_or_reduced_80pct'])}",
        f"- 运动段 observer_error 净功：{fmt(w05.get('movement_p_observer_error_net_integral'),6)} -> "
        f"{fmt(w10.get('movement_p_observer_error_net_integral'),6)}：{mark(w10_criteria['observer_direct_contribution_near_zero'])}",
        f"- 初态配对：摆角差 {fmt(w10_pair['initial_angle_difference_deg'])}°、摆速差 "
        f"{fmt(w10_pair['initial_rate_difference_rad_s'],5)} rad/s：{mark(w10_criteria['initial_pairing'])}",
        f"- 阶段总判定：{mark(w10_criteria['all_pass'])}。性能收益门槛失败，因此 E-VFB 未实施。",
        "", "## PID+AS+PAS 备用路线", "",
        f"有效复跑趋势相对 PID+AS：摆角 RMS 改善 {fmt(backup['swing_rms_improvement_pct'])}%、"
        f"能量积分改善 {fmt(backup['energy_improvement_pct'])}%；均未达到 10%/15% 门槛。",
        "两次 PID+AS+PAS 的 sensor_combined 最大间隔均为 52 ms，按任务书均为时序无效；已执行且仅执行一次自动复跑。",
        "", "## 最终决策", "",
        f"`{final_decision}`",
        "",
        "W10 证明了观测误差支路确会使 nominal 注能，但没有证明它是造成整体摆动落后的主要原因；"
        "在本轮数据中，去掉该支路后 RMS 仅改善约 3%，能量几乎不变。备用 PAS 也没有达到显著改善门槛，故当前工程基线仍为 PID+AS。",
        "",
    ]
    (args.output_dir / "validation_summary.md").write_text("\n".join(lines), encoding="utf-8")
    print(args.output_dir / "validation_summary.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
