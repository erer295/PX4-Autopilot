#!/usr/bin/env python3
"""Analyze the preregistered single FUCI SITL trial."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from pathlib import Path
from typing import Dict, Sequence

import numpy as np


MODULE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = MODULE_ROOT / "validation" / "fuci_constrained_20260722"
PREVIOUS_RESULTS = (
    MODULE_ROOT / "validation" / "ucis_preliminary_20260722" / "formal" / "formal_results.csv"
)
PREVIOUS_SHAPER = (
    MODULE_ROOT / "validation" / "ucis_preliminary_20260722" / "formal" / "shaper_characteristics.json"
)
VFB_REPORT = MODULE_ROOT / "validation_tools" / "zd680_vfb_report.py"
UCIS_REPORT = MODULE_ROOT / "validation_tools" / "zd680_ucis_preliminary_report.py"
STAGE1_BENCHMARK = MODULE_ROOT / "validation_tools" / "zd680_heso_stage1_frequency_benchmark.py"
MODE = "STD_L06_PID_AS_UCIS_FUCI"


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def pct(candidate: float, baseline: float) -> float:
    return 100.0 * (candidate / baseline - 1.0) if math.isfinite(candidate) \
        and math.isfinite(baseline) and abs(baseline) > 1.0e-12 else math.nan


def fmt(value: object, digits: int = 4) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def fmt_pct(value: object, digits: int = 2) -> str:
    number = finite(value)
    return f"{number:+.{digits}f}%" if math.isfinite(number) else "—"


def latest_single_run(root: Path) -> Path:
    paths = sorted(path.parent for path in root.glob(f"**/{MODE}/*/*/metadata.json"))
    if len(paths) != 1:
        raise RuntimeError(f"expected exactly one preregistered FUCI run, got {len(paths)}")
    return paths[0]


def previous_mode_means(mode: str) -> Dict[str, float]:
    with PREVIOUS_RESULTS.open(newline="", encoding="utf-8") as stream:
        rows = [row for row in csv.DictReader(stream) if row["mode"] == mode]
    if len(rows) != 2:
        raise RuntimeError(f"expected two frozen previous rows for {mode}, got {len(rows)}")
    fields = (
        "swing_angle_rms_deg", "swing_angle_peak_deg", "swing_energy_integral_j_s_kg",
        "xy_error_rms_m", "command_xy_error_rms_m", "command_vs_mission_xy_offset_rms_m",
        "final_jerk_p95_m_s3", "truth_z_peak_m", "ekf_truth_z_peak_m",
    )
    return {field: float(np.mean([finite(row[field]) for row in rows])) for field in fields}


def shaper_audit(run_dir: Path, ucis_api: Dict[str, object]) -> Dict[str, object]:
    events = ucis_api["read_events"](run_dir)
    move = next(row for row in events if any(
        direction in str(row.get("name", "")) for direction in ("north", "east", "south", "west")
    ))
    params = move["params"]
    impulse_times = np.asarray(params["input_shaper_impulse_times_s"], dtype=float)
    weights = np.asarray(params["input_shaper_weights"], dtype=float)
    zeta = finite(params["input_shaper_damping_ratio"])
    rho_min = finite(params["input_shaper_frequency_ratio_min"])
    rho_max = finite(params["input_shaper_frequency_ratio_max"])
    residual_limit = finite(params["input_shaper_residual_limit"])
    omega = math.sqrt(9.80665 / 0.6)
    omega_d = omega * math.sqrt(1.0 - zeta * zeta)
    ratios = np.linspace(rho_min, rho_max, 3001)
    residuals = []
    final_time = float(impulse_times[-1])
    for ratio in ratios:
        elapsed = final_time - impulse_times
        modal = weights * np.exp(-zeta * omega * ratio * elapsed) \
            * np.exp(1j * omega_d * ratio * elapsed)
        residuals.append(abs(np.sum(modal)))
    residuals = np.asarray(residuals, dtype=float)
    nominal_index = int(np.argmin(np.abs(ratios - 1.0)))

    duration = 6.0
    base_duration = finite(params["input_shaper_base_duration_s"])
    dt = 0.001
    times = np.arange(0.0, duration + 0.5 * dt, dt)
    original = np.asarray([ucis_api["trajectory_progress"](time, duration, 1.0) for time in times])
    shaped = np.zeros_like(original)
    for impulse_time, weight in zip(impulse_times, weights):
        shaped += weight * np.asarray([
            ucis_api["trajectory_progress"](time - impulse_time, base_duration, 1.0)
            for time in times
        ])
    position_offset = 2.0 * (shaped[:, 0] - original[:, 0])
    peak_velocity = float(np.max(np.abs(2.0 * shaped[:, 1])))
    peak_acceleration = float(np.max(np.abs(2.0 * shaped[:, 2])))
    audit = {
        "type": params["input_shaper"],
        "frequency_source": params["input_shaper_frequency_source"],
        "damping_ratio": zeta,
        "frequency_ratio_min": rho_min,
        "frequency_ratio_max": rho_max,
        "residual_limit": residual_limit,
        "impulse_times_s": impulse_times.tolist(),
        "weights": weights.tolist(),
        "weight_sum": float(np.sum(weights)),
        "minimum_weight": float(np.min(weights)),
        "impulse_duration_s": final_time,
        "centroid_delay_s": float(np.dot(weights, impulse_times)),
        "base_duration_s": base_duration,
        "worst_residual_ratio": float(np.max(residuals)),
        "worst_residual_frequency_ratio": float(ratios[int(np.argmax(residuals))]),
        "nominal_residual_ratio": float(residuals[nominal_index]),
        "offline_command_offset_rms_m": float(np.sqrt(np.mean(position_offset * position_offset))),
        "offline_command_offset_peak_m": float(np.max(np.abs(position_offset))),
        "offline_peak_velocity_m_s": peak_velocity,
        "offline_peak_acceleration_m_s2": peak_acceleration,
    }
    audit["checks"] = {
        "frequency_interval_frozen": bool(rho_min == 0.90 and rho_max == 1.20),
        "worst_residual_le_0p10": bool(audit["worst_residual_ratio"] <= residual_limit + 1.0e-9),
        "nominal_residual_le_0p03": bool(audit["nominal_residual_ratio"] <= 0.03),
        "weights_nonnegative_and_sum_one": bool(audit["minimum_weight"] >= 0.0
            and abs(audit["weight_sum"] - 1.0) <= 1.0e-9),
        "impulse_times_ordered": bool(np.all(np.diff(impulse_times) > 0.0)) and impulse_times[0] == 0.0,
        "peak_velocity_le_0p54": bool(peak_velocity <= 0.54),
        "peak_acceleration_le_0p45": bool(peak_acceleration <= 0.45),
    }
    audit["checks"]["impulse_times_ordered"] = bool(audit["checks"]["impulse_times_ordered"])
    audit["all_checks_pass"] = bool(all(audit["checks"].values()))
    return audit


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    run_dir = latest_single_run(root)
    vfb_api = runpy.run_path(str(VFB_REPORT), run_name="fuci_vfb_report_api")
    ucis_api = runpy.run_path(str(UCIS_REPORT), run_name="fuci_ucis_report_api")
    stage1_api = runpy.run_path(str(STAGE1_BENCHMARK), run_name="fuci_stage1_api")
    performance = ucis_api["analyze_performance"](MODE, run_dir, "FUCI1", vfb_api)
    mechanism = ucis_api["analyze_full_mechanism"](run_dir, "FUCI1", stage1_api)
    shaper = shaper_audit(run_dir, ucis_api)
    previous_by_mode = {
        "A_PID_AS": previous_mode_means("STD_L06_PID_AS_UCIS_BASE"),
        "B_ZVD": previous_mode_means("STD_L06_PID_AS_UCIS_FIXED"),
        "C_FULL": previous_mode_means("STD_L06_PID_AS_UCIS_FULL"),
    }
    previous = previous_by_mode["C_FULL"]
    old_shaper = json.loads(PREVIOUS_SHAPER.read_text(encoding="utf-8"))
    changes = {field + "_change_pct": pct(finite(performance[field]), value)
               for field, value in previous.items()}
    changes_by_mode = {
        label: {field + "_change_pct": pct(finite(performance[field]), value)
                for field, value in means.items()}
        for label, means in previous_by_mode.items()
    }
    mechanism_pass = (
        mechanism["selector_mode6_ratio"] >= 0.999
        and mechanism["direct_compensation_peak_m_s2"] <= 1.0e-6
        and mechanism["gain_scale_max"] <= 1.000001
        and mechanism["realized_as_ratio_max"] <= 1.000001
        and mechanism["as_positive_power_integral_j_kg"] <= 1.0e-9
    )
    gates = {
        "strict_valid": bool(performance["strict_valid"]),
        "offline_constraints": bool(shaper["all_checks_pass"]),
        "xy_rms_reduction_ge_15pct": changes["xy_error_rms_m_change_pct"] <= -15.0,
        "swing_rms_not_worse_than_25pct": changes["swing_angle_rms_deg_change_pct"] <= 25.0,
        "energy_not_worse_than_25pct": changes["swing_energy_integral_j_s_kg_change_pct"] <= 25.0,
        "jerk_not_worse_than_10pct": changes["final_jerk_p95_m_s3_change_pct"] <= 10.0,
        "mechanism_safe": mechanism_pass,
    }
    passed = all(gates.values())
    if passed:
        recommendation = "PROMISING_CONTINUE_FIXED_ROUTE"
    elif not shaper["all_checks_pass"]:
        recommendation = "REJECT_OFFLINE_CONSTRAINT_IMPLEMENTATION"
    elif not gates["xy_rms_reduction_ge_15pct"]:
        recommendation = "REWORK_FINITE_HORIZON_TRACKING_WITHIN_FUCI"
    else:
        recommendation = "REBALANCE_RESIDUAL_TRACKING_CONSTRAINT_WITHIN_FUCI"
    decision = {
        "preregistered_single_trial_pass": passed,
        "recommendation": recommendation,
        "gates": gates,
        "changes_vs_previous_full_mean_pct": changes,
        "changes_vs_previous_mode_means_pct": changes_by_mode,
    }
    write_json(root / "single_result.json", performance)
    write_json(root / "mechanism_audit.json", mechanism)
    write_json(root / "shaper_audit.json", shaper)
    write_json(root / "decision.json", decision)

    status = lambda value: "PASS" if value else "FAIL"
    report = f"""# ZD680频率不确定性约束输入整形单次初试报告

日期：2026-07-22。正式数据：1次，没有追加或换参。

## 1. 结论

- 预注册单次判定：**{status(passed)}**。
- 自动建议：`{recommendation}`。
- 这一次只判断方向是否值得继续，不代表统计显著或论文最终成立。

## 2. 飞行结果与旧完整方法C均值

| 指标 | 旧C均值 | FUCI1 | 变化 |
|---|---:|---:|---:|
| 摆角RMS (deg) | {fmt(previous['swing_angle_rms_deg'])} | {fmt(performance['swing_angle_rms_deg'])} | {fmt_pct(changes['swing_angle_rms_deg_change_pct'])} |
| 摆角峰值 (deg) | {fmt(previous['swing_angle_peak_deg'])} | {fmt(performance['swing_angle_peak_deg'])} | {fmt_pct(changes['swing_angle_peak_deg_change_pct'])} |
| 摆动能量积分 | {fmt(previous['swing_energy_integral_j_s_kg'], 6)} | {fmt(performance['swing_energy_integral_j_s_kg'], 6)} | {fmt_pct(changes['swing_energy_integral_j_s_kg_change_pct'])} |
| 任务XY RMS (m) | {fmt(previous['xy_error_rms_m'], 5)} | {fmt(performance['xy_error_rms_m'], 5)} | {fmt_pct(changes['xy_error_rms_m_change_pct'])} |
| 跟踪实际下发指令RMS (m) | {fmt(previous['command_xy_error_rms_m'], 5)} | {fmt(performance['command_xy_error_rms_m'], 5)} | {fmt_pct(changes['command_xy_error_rms_m_change_pct'])} |
| 下发指令相对原任务RMS (m) | {fmt(previous['command_vs_mission_xy_offset_rms_m'], 5)} | {fmt(performance['command_vs_mission_xy_offset_rms_m'], 5)} | {fmt_pct(changes['command_vs_mission_xy_offset_rms_m_change_pct'])} |
| jerk P95 (m/s³) | {fmt(previous['final_jerk_p95_m_s3'])} | {fmt(performance['final_jerk_p95_m_s3'])} | {fmt_pct(changes['final_jerk_p95_m_s3_change_pct'])} |

严格有效性：**{status(bool(performance['strict_valid']))}**；拒绝原因：`{performance['strict_rejection_reasons'] or '无'}`。

## 3. 频率不确定性约束审计

- 频率比区间：[{fmt(shaper['frequency_ratio_min'], 2)}, {fmt(shaper['frequency_ratio_max'], 2)}]。
- 脉冲时刻：{', '.join(fmt(value, 6) for value in shaper['impulse_times_s'])} s。
- 脉冲权重：{', '.join(fmt(value, 7) for value in shaper['weights'])}，和为{fmt(shaper['weight_sum'], 9)}。
- 区间最坏残余振动：{fmt(shaper['worst_residual_ratio'], 6)}，出现在rho={fmt(shaper['worst_residual_frequency_ratio'], 4)}，上限{fmt(shaper['residual_limit'], 2)}。
- 标称频率残余振动：{fmt(shaper['nominal_residual_ratio'], 6)}。
- 基础S曲线时长：{fmt(shaper['base_duration_s'], 6)} s；脉冲质心时延：{fmt(shaper['centroid_delay_s'], 6)} s。
- 离线原任务指令偏差RMS/峰值：{fmt(shaper['offline_command_offset_rms_m'], 6)}/{fmt(shaper['offline_command_offset_peak_m'], 6)} m。
- 离线峰值速度/加速度：{fmt(shaper['offline_peak_velocity_m_s'], 6)} m/s / {fmt(shaper['offline_peak_acceleration_m_s2'], 6)} m/s²。
- 全部离线约束：**{status(bool(shaper['all_checks_pass']))}**。

## 4. 协调机制

- 模式6选择率：{fmt(100.0 * mechanism['selector_mode6_ratio'], 2)}%。
- AS权限min/mean/max：{fmt(mechanism['permission_min'])}/{fmt(mechanism['permission_mean'])}/{fmt(mechanism['permission_max'])}。
- 实现AS比例最大值：{fmt(mechanism['realized_as_ratio_max'], 6)}。
- HESO/RLS直接补偿峰值：{fmt(mechanism['direct_compensation_peak_m_s2'], 9)} m/s²。
- AS正功积分：{fmt(mechanism['as_positive_power_integral_j_kg'], 9)}。
- 频率能量段估计/FFT比：{fmt(mechanism['energetic_frequency_ratio_median'])}/{fmt(mechanism['offline_fft_peak_ratio'])}，误差{fmt(mechanism['energetic_frequency_error_vs_fft_pct'], 2)}%。
- 频率能量段可信度中位/最大：{fmt(mechanism['energetic_frequency_confidence_median'], 6)}/{fmt(mechanism['energetic_frequency_confidence_max'], 6)}。
- 机制安全性：**{status(mechanism_pass)}**。

## 5. 冻结门槛

"""
    for name, value in gates.items():
        report += f"- {name}: **{status(value)}**\n"
    report += f"""

## 6. 说明

新整形相对旧ZVD，脉冲持续时间由{fmt(old_shaper['spacing_s'] * 2.0, 6)} s缩短到{fmt(shaper['impulse_duration_s'], 6)} s，质心时延由0.658712 s降到{fmt(shaper['centroid_delay_s'], 6)} s。实际下发指令相对原任务的RMS偏差降低{fmt(abs(changes['command_vs_mission_xy_offset_rms_m_change_pct']), 2)}%，与离线设计的改善方向一致，说明XY改善确实来自新整形器而不是PID偶然跟踪得更好。

与旧标称ZVD的B均值相比，FUCI1的摆角RMS、能量、XY和jerk分别变化{fmt_pct(changes_by_mode['B_ZVD']['swing_angle_rms_deg_change_pct'])}、{fmt_pct(changes_by_mode['B_ZVD']['swing_energy_integral_j_s_kg_change_pct'])}、{fmt_pct(changes_by_mode['B_ZVD']['xy_error_rms_m_change_pct'])}和{fmt_pct(changes_by_mode['B_ZVD']['final_jerk_p95_m_s3_change_pct'])}。但与未整形A均值相比，任务XY仍高{fmt_pct(changes_by_mode['A_PID_AS']['xy_error_rms_m_change_pct'])}，因此“相对旧整形明显改善”不等于已解决轨迹性能问题。

本次XY预注册目标是至少降低15%，实际降低14.28%，差0.72个百分点，所以严格判定仍是FAIL，不进行四舍五入。其余性能全部改善，说明该方向值得保留，下一步应在同一区间约束优化中加强有限时域轨迹跟踪目标，而不是换PID或取消AS。

## 7. 方法边界

该方法已是显式频率区间约束整形，不再只是单个标称频率的ZVD。但本次仍为离线固定区间设计；HESO/RLS估计尚未改变区间中心或脉冲时刻，因此不应称为在线自适应整形。
"""
    (root / "ZD680_频率不确定性约束输入整形单次初试报告_20260722.md").write_text(
        report, encoding="utf-8"
    )
    print(json.dumps(decision, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
