#!/usr/bin/env python3
"""Audit/report the preregistered dual-length finite-horizon FUCI trials."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np


MODULE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = MODULE_ROOT / "validation" / "fuci_finite_horizon_20260722"
PREVIOUS_FUCI = MODULE_ROOT / "validation" / "fuci_constrained_20260722" / "single_result.json"
VFB_REPORT = MODULE_ROOT / "validation_tools" / "zd680_vfb_report.py"
UCIS_REPORT = MODULE_ROOT / "validation_tools" / "zd680_ucis_preliminary_report.py"
STAGE1_BENCHMARK = MODULE_ROOT / "validation_tools" / "zd680_heso_stage1_frequency_benchmark.py"

MODES = (
    ("FH06", "STD_L06_PID_AS_UCIS_FH", 0.6),
    ("FH08", "STD_L08_PID_AS_UCIS_FH", 0.8),
)


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def pct(candidate: object, baseline: object) -> float:
    candidate_value, baseline_value = finite(candidate), finite(baseline)
    if not math.isfinite(candidate_value) or not math.isfinite(baseline_value) \
            or abs(baseline_value) <= 1.0e-12:
        return math.nan
    return 100.0 * (candidate_value / baseline_value - 1.0)


def fmt(value: object, digits: int = 4) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def fmt_pct(value: object, digits: int = 2) -> str:
    number = finite(value)
    return f"{number:+.{digits}f}%" if math.isfinite(number) else "—"


def status(value: bool) -> str:
    return "PASS" if value else "FAIL"


def write_json(path: Path, value: object) -> None:
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


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


def single_run(root: Path, mode: str) -> Path:
    paths = sorted(path.parent for path in (root / "runs" / mode).glob("*/*/metadata.json"))
    if len(paths) != 1:
        raise RuntimeError(f"expected exactly one preregistered run for {mode}, got {len(paths)}")
    return paths[0]


def rms(values: np.ndarray) -> float:
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values * values))) if values.size else math.nan


def direction_audit(run_dir: Path, ucis_api: Dict[str, object], label: str) -> List[Dict[str, object]]:
    events = [item for item in ucis_api["read_events"](run_dir)
              if str(item.get("name", "")).startswith("action_")]
    with (run_dir / "hang_joint_samples.csv").open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    times = np.asarray([finite(item.get("boot_s_est")) for item in samples])
    angles = np.asarray([finite(item.get("hang_angle_deg")) for item in samples])
    rows: List[Dict[str, object]] = []
    for index, event in enumerate(events):
        name = str(event.get("name", ""))
        direction = next((item for item in ("north", "east", "south", "west") if item in name), None)
        if direction is None:
            continue
        start, end = finite(event["start_boot_s"]), finite(event["end_boot_s"])
        move_mask = (times >= start) & (times <= end)
        hold_end = end
        if index + 1 < len(events) and "hold" in str(events[index + 1].get("name", "")):
            hold_end = finite(events[index + 1]["end_boot_s"])
        hold_mask = (times > end) & (times <= hold_end)
        move_values = angles[move_mask]
        hold_values = angles[hold_mask]
        peak_index = int(np.nanargmax(np.abs(move_values))) if np.any(np.isfinite(move_values)) else 0
        move_times = times[move_mask]
        rows.append({
            "label": label,
            "direction": direction,
            "move_swing_rms_deg": rms(move_values),
            "move_swing_peak_deg": float(np.nanmax(np.abs(move_values))),
            "move_peak_time_from_start_s": float(move_times[peak_index] - start),
            "post_hold_swing_rms_deg": rms(hold_values),
            "post_hold_swing_peak_deg": float(np.nanmax(np.abs(hold_values))),
        })
    return rows


def implementation_audit(run_dir: Path, expected: Dict[str, object],
                         ucis_api: Dict[str, object]) -> Dict[str, object]:
    events = ucis_api["read_events"](run_dir)
    moves = [item for item in events if any(
        direction in str(item.get("name", "")) for direction in ("north", "east", "south", "west")
    )]
    checks: Dict[str, bool] = {"four_move_events": len(moves) == 4}
    if moves:
        params = [item["params"] for item in moves]
        expected_coefficients = np.asarray(expected["coefficients"], dtype=float)
        checks.update({
            "finite_horizon_type_recorded": all(
                item.get("input_shaper") == "finite_horizon_frequency_uncertainty_constrained_trajectory"
                for item in params
            ),
            "full_six_second_horizon": all(
                abs(finite(item.get("input_shaper_base_duration_s")) - 6.0) <= 1.0e-9
                and abs(finite(item.get("duration_s")) - 6.0) <= 1.0e-9 for item in params
            ),
            "frequency_interval_recorded": all(
                finite(item.get("input_shaper_frequency_ratio_min")) == 0.90
                and finite(item.get("input_shaper_frequency_ratio_max")) == 1.20 for item in params
            ),
            "frozen_coefficients_match": all(
                np.allclose(np.asarray(item.get("finite_horizon_coefficients", []), dtype=float),
                            expected_coefficients, rtol=0.0, atol=1.0e-12)
                for item in params
            ),
            "no_causal_impulses": all(not item.get("input_shaper_impulse_times_s")
                                      and not item.get("input_shaper_weights") for item in params),
        })
    checks["all_checks_pass"] = all(checks.values())
    return {"run": run_dir.name, "checks": checks}


def offline_audit(item: Dict[str, object]) -> Dict[str, object]:
    overshoot_m = max(0.0, -2.0 * finite(item["position_min"]),
                      2.0 * (finite(item["position_max"]) - 1.0))
    checks = {
        "command_offset_rms_le_0p075_m": finite(item["command_offset_rms_m"]) <= 0.075,
        "peak_velocity_le_0p54_m_s": finite(item["peak_velocity_m_s"]) <= 0.54,
        "peak_acceleration_le_0p65_m_s2": finite(item["peak_acceleration_m_s2"]) <= 0.65,
        "peak_jerk_le_3p10_m_s3": finite(item["peak_jerk_m_s3"]) <= 3.10,
        "worst_terminal_mode_le_1p15_deg": finite(item["worst_terminal_modal_amplitude_deg"]) <= 1.15,
        "position_overshoot_le_0p005_m": overshoot_m <= 0.005,
    }
    return {**item, "position_overshoot_m": overshoot_m, "checks": checks,
            "all_checks_pass": all(checks.values())}


def flight_gates(performance: Dict[str, object], mechanism: Dict[str, object]) -> Dict[str, bool]:
    return {
        "strict_valid": bool(performance["strict_valid"]),
        "failsafe_zero": finite(performance["failsafe_ratio"]) == 0.0,
        "selector_mode6_100pct": finite(mechanism["selector_mode6_ratio"]) >= 0.999999,
        "swing_rms_le_1p0_deg": finite(performance["swing_angle_rms_deg"]) <= 1.0,
        "swing_peak_le_4p0_deg": finite(performance["swing_angle_peak_deg"]) <= 4.0,
        "xy_rms_le_0p12_m": finite(performance["xy_error_rms_m"]) <= 0.12,
        "jerk_p95_le_2p0_m_s3": finite(performance["final_jerk_p95_m_s3"]) <= 2.0,
        "direct_compensation_zero": finite(mechanism["direct_compensation_peak_m_s2"]) <= 1.0e-7,
        "as_positive_power_zero": finite(mechanism["as_positive_power_integral_j_kg"]) <= 1.0e-9,
        "as_never_amplified": finite(mechanism["realized_as_ratio_max"]) <= 1.000001,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    vfb_api = runpy.run_path(str(VFB_REPORT), run_name="fh_vfb_report_api")
    ucis_api = runpy.run_path(str(UCIS_REPORT), run_name="fh_ucis_report_api")
    stage1_api = runpy.run_path(str(STAGE1_BENCHMARK), run_name="fh_stage1_report_api")
    design_items = json.loads((root / "finite_horizon_design.json").read_text(encoding="utf-8"))
    designs = {round(finite(item["rope_length_m"]), 1): item for item in design_items}

    performances: Dict[str, Dict[str, object]] = {}
    mechanisms: Dict[str, Dict[str, object]] = {}
    offline: Dict[str, Dict[str, object]] = {}
    implementations: Dict[str, Dict[str, object]] = {}
    directions: List[Dict[str, object]] = []
    paths: Dict[str, Path] = {}
    for label, mode, length in MODES:
        run_dir = single_run(root, mode)
        paths[label] = run_dir
        performances[label] = ucis_api["analyze_performance"](
            mode, run_dir, label, vfb_api
        )
        mechanisms[label] = ucis_api["analyze_full_mechanism"](
            run_dir, label, stage1_api
        )
        offline[label] = offline_audit(designs[length])
        implementations[label] = implementation_audit(run_dir, designs[length], ucis_api)
        directions.extend(direction_audit(run_dir, ucis_api, label))

    previous = json.loads(PREVIOUS_FUCI.read_text(encoding="utf-8"))
    comparison_fields = (
        "swing_angle_rms_deg", "swing_angle_peak_deg", "swing_energy_integral_j_s_kg",
        "xy_error_rms_m", "command_xy_error_rms_m", "command_vs_mission_xy_offset_rms_m",
        "final_jerk_p95_m_s3", "truth_z_peak_m",
    )
    fh06_vs_fuci = {field + "_change_pct": pct(performances["FH06"][field], previous[field])
                    for field in comparison_fields}
    fh08_vs_fh06 = {field + "_change_pct": pct(performances["FH08"][field], performances["FH06"][field])
                    for field in comparison_fields}
    direction_map = {(str(item["label"]), str(item["direction"])): item for item in directions}
    gates_by_label = {label: flight_gates(performances[label], mechanisms[label])
                      for label, _, _ in MODES}
    flight_pass_by_label = {label: all(items.values()) for label, items in gates_by_label.items()}
    relative_gates = {
        "fh06_command_offset_reduction_ge_20pct":
            fh06_vs_fuci["command_vs_mission_xy_offset_rms_m_change_pct"] <= -20.0,
        "fh06_swing_rms_not_worse_than_50pct":
            fh06_vs_fuci["swing_angle_rms_deg_change_pct"] <= 50.0,
        "fh06_xy_rms_reduction_ge_15pct":
            fh06_vs_fuci["xy_error_rms_m_change_pct"] <= -15.0,
    }
    offline_all = all(bool(offline[label]["all_checks_pass"]) for label, _, _ in MODES)
    implementation_all = all(bool(implementations[label]["checks"]["all_checks_pass"])
                             for label, _, _ in MODES)
    dual_length_pass = offline_all and implementation_all \
        and all(flight_pass_by_label.values()) and all(relative_gates.values())
    decision_code = (
        "FINITE_HORIZON_DUAL_LENGTH_PRELIMINARY_PASS" if dual_length_pass
        else "FINITE_HORIZON_DUAL_LENGTH_FAIL_REWORK_SAME_ROUTE"
    )
    decision = {
        "decision": decision_code,
        "dual_length_preliminary_robustness_pass": dual_length_pass,
        "offline_all_pass": offline_all,
        "implementation_all_pass": implementation_all,
        "flight_pass_by_length": flight_pass_by_label,
        "flight_gates": gates_by_label,
        "fh06_relative_gates_vs_previous_fuci": relative_gates,
        "fh06_changes_vs_previous_fuci_pct": fh06_vs_fuci,
        "fh08_changes_vs_fh06_pct": fh08_vs_fh06,
        "sample_scope": "one preregistered valid SITL run per rope length; no retries",
    }

    write_csv(root / "formal_results.csv", list(performances.values()))
    write_csv(root / "mechanism_audit.csv", list(mechanisms.values()))
    write_csv(root / "direction_results.csv", directions)
    write_json(root / "finite_horizon_audit.json", {
        "offline": offline,
        "implementation": implementations,
    })
    write_json(root / "decision.json", decision)

    p06, p08 = performances["FH06"], performances["FH08"]
    m06, m08 = mechanisms["FH06"], mechanisms["FH08"]
    report = f"""# ZD680有限时域频率不确定性轨迹整形双绳长验证总结报告

日期：2026-07-22。正式数据严格按预注册顺序`FH06 -> FH08`各运行1次，没有补跑、删数或中途调参。

## 1. 结论先行

- 双绳长初步鲁棒性总判定：**{status(dual_length_pass)}**，自动结论`{decision_code}`。
- 有限时域升级在“减少任务时序失真和XY误差”上有效，但在“摆角峰值、闭环jerk和相对旧FUCI的摆角RMS”上失败，所以当前不能封版，也不能宣称已经获得双绳长鲁棒性。
- 方法不需要更换。保留`PID + 能量AS + 模式6协调器 + 频率不确定性有限时域整形`，下一轮应把优化目标从“只压6 s末端线性模态”改成“全时域摆角/峰值 + 末端残摆 + 闭环jerk + 非零初态”的统一约束。
- 两次都严格有效、failsafe为0、模式6选择率100%，因此失败是性能门失败，不是仿真数据无效。

## 2. 本次改了什么

旧FUCI采用“压缩基础S曲线 + 三个延迟脉冲”的因果卷积。新版保持完整6 s时域，直接构造

`s(u)=s0(u)+sum(c_k u^(k+3)(1-u)^(14-k)), u=t/6, k=0..11`。

基函数的位置、速度、加速度修正在两端都为0，所以2 m位移、6 s总时长和末端零速零加速度不变。0.6 m与0.8 m使用同一基函数数、权重、`rho in [0.90,1.20]`频率区间和门槛，只根据标称绳长离线重算一次冻结系数。实际事件中的系数、时域和频率区间与设计文件核对结果：**{status(implementation_all)}**。

## 3. 离线轨迹审计

| 绳长 | 任务偏差RMS(m) | 偏差峰值(m) | 速度峰值 | 加速度峰值 | jerk峰值 | 区间最坏末端模态(°) | 位置越界(m) | 结论 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 0.6 m | {fmt(offline['FH06']['command_offset_rms_m'],6)} | {fmt(offline['FH06']['command_offset_peak_m'],6)} | {fmt(offline['FH06']['peak_velocity_m_s'],4)} | {fmt(offline['FH06']['peak_acceleration_m_s2'],4)} | {fmt(offline['FH06']['peak_jerk_m_s3'],4)} | {fmt(offline['FH06']['worst_terminal_modal_amplitude_deg'],4)} | {fmt(offline['FH06']['position_overshoot_m'],6)} | {status(offline['FH06']['all_checks_pass'])} |
| 0.8 m | {fmt(offline['FH08']['command_offset_rms_m'],6)} | {fmt(offline['FH08']['command_offset_peak_m'],6)} | {fmt(offline['FH08']['peak_velocity_m_s'],4)} | {fmt(offline['FH08']['peak_acceleration_m_s2'],4)} | {fmt(offline['FH08']['peak_jerk_m_s3'],4)} | {fmt(offline['FH08']['worst_terminal_modal_amplitude_deg'],4)} | {fmt(offline['FH08']['position_overshoot_m'],6)} | {status(offline['FH08']['all_checks_pass'])} |

两个离线轨迹都通过冻结门。注意这个“末端模态”只检查6 s终点，不约束6 s过程中的最大摆角，这是本轮飞行失败的主要结构性漏洞。

## 4. 双绳长飞行结果

| 指标 | 0.6 m FH06 | 0.8 m FH08 | FH08/FH06变化 | 门槛 |
|---|---:|---:|---:|---:|
| 严格有效 | 是 | 是 | — | 必须有效 |
| 摆角RMS (°) | {fmt(p06['swing_angle_rms_deg'])} | {fmt(p08['swing_angle_rms_deg'])} | {fmt_pct(fh08_vs_fh06['swing_angle_rms_deg_change_pct'])} | <=1.0 |
| 摆角峰值 (°) | {fmt(p06['swing_angle_peak_deg'])} | {fmt(p08['swing_angle_peak_deg'])} | {fmt_pct(fh08_vs_fh06['swing_angle_peak_deg_change_pct'])} | <=4.0 |
| 摆动能量积分 | {fmt(p06['swing_energy_integral_j_s_kg'],6)} | {fmt(p08['swing_energy_integral_j_s_kg'],6)} | {fmt_pct(fh08_vs_fh06['swing_energy_integral_j_s_kg_change_pct'])} | 观察项 |
| 任务XY RMS (m) | {fmt(p06['xy_error_rms_m'],5)} | {fmt(p08['xy_error_rms_m'],5)} | {fmt_pct(fh08_vs_fh06['xy_error_rms_m_change_pct'])} | <=0.12 |
| 实际-下发指令RMS (m) | {fmt(p06['command_xy_error_rms_m'],5)} | {fmt(p08['command_xy_error_rms_m'],5)} | {fmt_pct(fh08_vs_fh06['command_xy_error_rms_m_change_pct'])} | 观察项 |
| 指令-原任务RMS (m) | {fmt(p06['command_vs_mission_xy_offset_rms_m'],5)} | {fmt(p08['command_vs_mission_xy_offset_rms_m'],5)} | {fmt_pct(fh08_vs_fh06['command_vs_mission_xy_offset_rms_m_change_pct'])} | 离线<=0.075 |
| 最终加速度jerk P95 | {fmt(p06['final_jerk_p95_m_s3'])} | {fmt(p08['final_jerk_p95_m_s3'])} | {fmt_pct(fh08_vs_fh06['final_jerk_p95_m_s3_change_pct'])} | <=2.0 |
| 真值高度误差峰值(m) | {fmt(p06['truth_z_peak_m'],3)} | {fmt(p08['truth_z_peak_m'],3)} | {fmt_pct(fh08_vs_fh06['truth_z_peak_m_change_pct'])} | 观察项 |

0.6 m失败项：摆角峰值{fmt(p06['swing_angle_peak_deg'])}°和jerk P95 {fmt(p06['final_jerk_p95_m_s3'])}。0.8 m失败项相同，分别为{fmt(p08['swing_angle_peak_deg'])}°和{fmt(p08['final_jerk_p95_m_s3'])}。两绳长的摆角RMS和XY都过门，但峰值与平滑性同时失败，所以不能只凭RMS接近就判鲁棒。

分方向真值进一步显示问题不是均匀发生：

| 绳长/方向 | 移动段RMS(°) | 移动段峰值(°) | 后续悬停RMS(°) | 后续悬停峰值(°) |
|---|---:|---:|---:|---:|
| FH06 north | {fmt(direction_map[('FH06','north')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH06','north')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH06','north')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH06','north')]['post_hold_swing_peak_deg'])} |
| FH06 east | {fmt(direction_map[('FH06','east')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH06','east')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH06','east')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH06','east')]['post_hold_swing_peak_deg'])} |
| FH06 south | {fmt(direction_map[('FH06','south')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH06','south')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH06','south')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH06','south')]['post_hold_swing_peak_deg'])} |
| FH06 west | {fmt(direction_map[('FH06','west')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH06','west')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH06','west')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH06','west')]['post_hold_swing_peak_deg'])} |
| FH08 north | {fmt(direction_map[('FH08','north')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH08','north')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH08','north')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH08','north')]['post_hold_swing_peak_deg'])} |
| FH08 east | {fmt(direction_map[('FH08','east')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH08','east')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH08','east')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH08','east')]['post_hold_swing_peak_deg'])} |
| FH08 south | {fmt(direction_map[('FH08','south')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH08','south')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH08','south')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH08','south')]['post_hold_swing_peak_deg'])} |
| FH08 west | {fmt(direction_map[('FH08','west')]['move_swing_rms_deg'])} | {fmt(direction_map[('FH08','west')]['move_swing_peak_deg'])} | {fmt(direction_map[('FH08','west')]['post_hold_swing_rms_deg'])} | {fmt(direction_map[('FH08','west')]['post_hold_swing_peak_deg'])} |

north段都很小，主要峰值集中在east/west以及转入下一段后的残摆；例如FH06的south移动本身峰值只有{fmt(direction_map[('FH06','south')]['move_swing_peak_deg'])}°，后续悬停却达到{fmt(direction_map[('FH06','south')]['post_hold_swing_peak_deg'])}°。这支持“非零初态、固定任务顺序和方向耦合破坏单段零初态相消”的解释，但单次固定顺序还不能把原因唯一归结为X/Y控制器不一致。

## 5. 相对上一版0.6 m FUCI

| 指标 | 旧FUCI1 | FH06 | 变化 | 冻结要求 |
|---|---:|---:|---:|---:|
| 摆角RMS (°) | {fmt(previous['swing_angle_rms_deg'])} | {fmt(p06['swing_angle_rms_deg'])} | {fmt_pct(fh06_vs_fuci['swing_angle_rms_deg_change_pct'])} | 不恶化超过50% |
| 摆角峰值 (°) | {fmt(previous['swing_angle_peak_deg'])} | {fmt(p06['swing_angle_peak_deg'])} | {fmt_pct(fh06_vs_fuci['swing_angle_peak_deg_change_pct'])} | 观察项 |
| 摆动能量积分 | {fmt(previous['swing_energy_integral_j_s_kg'],6)} | {fmt(p06['swing_energy_integral_j_s_kg'],6)} | {fmt_pct(fh06_vs_fuci['swing_energy_integral_j_s_kg_change_pct'])} | 观察项 |
| 任务XY RMS (m) | {fmt(previous['xy_error_rms_m'],5)} | {fmt(p06['xy_error_rms_m'],5)} | {fmt_pct(fh06_vs_fuci['xy_error_rms_m_change_pct'])} | 至少改善15% |
| 指令-原任务RMS (m) | {fmt(previous['command_vs_mission_xy_offset_rms_m'],5)} | {fmt(p06['command_vs_mission_xy_offset_rms_m'],5)} | {fmt_pct(fh06_vs_fuci['command_vs_mission_xy_offset_rms_m_change_pct'])} | 至少改善20% |
| jerk P95 | {fmt(previous['final_jerk_p95_m_s3'])} | {fmt(p06['final_jerk_p95_m_s3'])} | {fmt_pct(fh06_vs_fuci['final_jerk_p95_m_s3_change_pct'])} | 观察项 |

有限时域升级把任务XY降低{fmt(abs(fh06_vs_fuci['xy_error_rms_m_change_pct']),2)}%，把指令时序偏差降低{fmt(abs(fh06_vs_fuci['command_vs_mission_xy_offset_rms_m_change_pct']),2)}%，这正是升级想解决的问题；但摆角RMS恶化{fmt(fh06_vs_fuci['swing_angle_rms_deg_change_pct'],2)}%，超过允许的50%，jerk也恶化{fmt(fh06_vs_fuci['final_jerk_p95_m_s3_change_pct'],2)}%。因此不是“所有方面都变差”，而是把旧方案的时序滞后换成了更强的瞬态激励，权衡失衡。

## 6. 协调器、AS和频率支路

| 项目 | FH06 | FH08 |
|---|---:|---:|
| 模式6选择率 | {fmt(100*m06['selector_mode6_ratio'],2)}% | {fmt(100*m08['selector_mode6_ratio'],2)}% |
| AS权限min/mean/max | {fmt(m06['permission_min'])}/{fmt(m06['permission_mean'])}/{fmt(m06['permission_max'])} | {fmt(m08['permission_min'])}/{fmt(m08['permission_mean'])}/{fmt(m08['permission_max'])} |
| 实现AS比例max | {fmt(m06['realized_as_ratio_max'])} | {fmt(m08['realized_as_ratio_max'])} |
| AS正功积分 | {fmt(m06['as_positive_power_integral_j_kg'],9)} | {fmt(m08['as_positive_power_integral_j_kg'],9)} |
| AS负功积分 | {fmt(m06['as_negative_power_integral_j_kg'],6)} | {fmt(m08['as_negative_power_integral_j_kg'],6)} |
| HESO/RLS直接补偿峰值 | {fmt(m06['direct_compensation_peak_m_s2'],9)} | {fmt(m08['direct_compensation_peak_m_s2'],9)} |
| 能量段频率比/FFT比 | {fmt(m06['energetic_frequency_ratio_median'])}/{fmt(m06['offline_fft_peak_ratio'])} | {fmt(m08['energetic_frequency_ratio_median'])}/{fmt(m08['offline_fft_peak_ratio'])} |
| 频率误差 | {fmt(m06['energetic_frequency_error_vs_fft_pct'],2)}% | {fmt(m08['energetic_frequency_error_vs_fft_pct'],2)}% |
| 能量段可信度中位/最大 | {fmt(m06['energetic_frequency_confidence_median'],6)}/{fmt(m06['energetic_frequency_confidence_max'],6)} | {fmt(m08['energetic_frequency_confidence_median'],6)}/{fmt(m08['energetic_frequency_confidence_max'],6)} |

机制安全边界均成立：没有直接HESO补偿、AS不放大且不做正功。0.8 m的AS负功绝对值比0.6 m更大，权限最低值也更低，说明协调器确实在更强地抑制能量，而不是失效。另一方面，频率可信度仍接近0；特别是0.8 m在线估计相对FFT误差{fmt(m08['energetic_frequency_error_vs_fft_pct'],2)}%，所以本轮只验证了离线区间轨迹，不能宣称“HESO可信频率在线自适应”已经具有跨绳长鲁棒性。

## 7. 为什么离线通过、飞行仍失败

1. **目标错位。** 当前优化主要压低6 s终点的线性模态状态，允许中途先摆大再在终点相消；飞行门检查整段任务RMS和峰值，所以会出现“末端模型合格、途中峰值4.3~4.4°超限”。
2. **平滑门太松且不是闭环量。** 离线参考jerk已经达到{fmt(offline['FH06']['peak_jerk_m_s3'],2)}/{fmt(offline['FH08']['peak_jerk_m_s3'],2)} m/s³，接近3.10上限；位置PID和AS为了追踪这些弯折还会追加动作，最终闭环jerk P95升到{fmt(p06['final_jerk_p95_m_s3'],2)}/{fmt(p08['final_jerk_p95_m_s3'],2)}，超过2.0门。
3. **模型假设过理想。** 离线模型假设每一段从零摆角、零摆速开始，并把无人机指令加速度当成负载实际激励；真实四边任务有上一段残摆、位置闭环延迟、AS动作、三维转弯和垂向耦合，相消条件会偏移。
4. **优化系数大、局部曲率高。** 多项式修正虽然端点平坦，但系数相互抵消，仍能在中段形成高曲率。它减少了时序偏差，却把能量推到更短时间尺度，解释了XY变好而摆角峰值和jerk同时变差。
5. **跨绳长频率证据不足。** 0.8 m的FFT频率仍落在离线`[0.90,1.20]`区间内，但在线估计误差和低可信度说明目前不能依靠它实时校正；0.8 m摆角RMS与0.6 m接近，但能量积分增加{fmt(fh08_vs_fh06['swing_energy_integral_j_s_kg_change_pct'],2)}%，不能视为完整鲁棒。

真值高度误差峰值达到{fmt(p06['truth_z_peak_m'],3)}/{fmt(p08['truth_z_peak_m'],3)} m，虽然未触发本次严格有效性拒绝，仍提示较强水平动作伴随垂向耦合。它是观察到的相关现象，单次数据不足以认定因果。

## 8. 下一步固定计划

不换主线，按以下顺序修正：

1. 把离线目标升级为多频率、多初态的全时域目标：对`max|theta(t)|`、`RMS(theta(t))`和末端`theta/rate`同时设约束；初态覆盖本轮各段进入时观测到的摆角和摆速，不再只从零状态算。
2. 把参考轨迹jerk硬上限由3.10收紧到约1.4~1.6 m/s³，并加入加加速度连续性或三阶端点基函数；给闭环jerk 2.0留出PID/AS余量。
3. 不再用无约束的大系数最小二乘。改为带位置单调性、速度、加速度、jerk、系数范数和全时域摆角的凸二次/序列凸约束；目标优先级固定为安全峰值→平滑→末端残摆→任务时序。
4. 用本轮ULog离线回放实际无人机加速度和每段真实初态，先做“若采用新轨迹，模态状态是否下降”的反事实筛选。只有离线回放同时预计峰值<3.5°、jerk参考<1.6，才允许再做SITL。
5. 复验仍使用0.6/0.8 m同一规则，先各1次门槛筛选；通过后扩为0.5/0.6/0.8 m、载荷变化和频率失配，每点至少5次。在线HESO频率支路必须先把0.8 m误差压到5%内、能量段高可信覆盖率提高到80%，再参与轨迹更新。

## 9. 冻结门逐项结论

### FH06飞行门

"""
    for name, value in gates_by_label["FH06"].items():
        report += f"- `{name}`：**{status(value)}**。\n"
    report += "\n### FH08飞行门\n\n"
    for name, value in gates_by_label["FH08"].items():
        report += f"- `{name}`：**{status(value)}**。\n"
    report += "\n### FH06相对旧FUCI门\n\n"
    for name, value in relative_gates.items():
        report += f"- `{name}`：**{status(value)}**。\n"
    report += f"""

## 10. 证据文件

- `finite_horizon_design.json`：双绳长离线系数与稠密频率审计；
- `finite_horizon_audit.json`：离线门和实际事件系数一致性；
- `formal_results.csv`：两次飞行性能；
- `mechanism_audit.csv`：模式6、AS功率/权限、HESO/RLS频率诊断；
- `direction_results.csv`：四个方向的移动与后续悬停摆角；
- `decision.json`：全部冻结门和自动总判定；
- 两个run目录：原始ULog、真值关节、live samples、events和validity。

本轮只有每个绳长1次，结论是“已发现可重复到两个绳长的共同失败模式”，不是统计显著性结论，也不是实机结论。
"""
    report_path = root / "ZD680_有限时域频率不确定性轨迹整形双绳长验证总结报告_20260722.md"
    report_path.write_text(report, encoding="utf-8")
    print(report_path)
    print(json.dumps(decision, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
