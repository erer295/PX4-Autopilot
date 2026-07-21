#!/usr/bin/env python3
"""Compare standard-condition PID+AS with LADRC2+AS+PAS+VFB W05."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence

import numpy as np
from pyulog import ULog


REPO_ROOT = Path(__file__).resolve().parents[4]
VFB_REPORT = Path(__file__).with_name("zd680_vfb_report.py")
VFB = runpy.run_path(str(VFB_REPORT), run_name="zd680_structure_vfb_report_api")
GENERIC = VFB["GENERIC"]
PID_MODE = "STD_L06_PID_AS"
FULL_MODE = "STD_L06_FULL_W05"
MODES = (PID_MODE, FULL_MODE)
Z_OBSERVATION_ONLY_CHECKS = VFB["Z_OBSERVATION_ONLY_CHECKS"]


def finite(value, fallback=math.nan) -> float:
    return VFB["finite"](value, fallback)


def fmt(value, digits=3) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    VFB["write_csv"](path, rows)


def latest_runs(root: Path) -> Dict[str, Path]:
    grouped: Dict[str, List[Path]] = {}
    for metadata_path in root.glob("runs/*/*/*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            mode = str(metadata.get("experiment", {}).get("validation_suite", ""))
        except (OSError, json.JSONDecodeError):
            continue
        if mode in MODES:
            grouped.setdefault(mode, []).append(metadata_path.parent)
    return {mode: max(paths, key=lambda path: path.stat().st_mtime) for mode, paths in grouped.items()}


def route_bounds(run_dir: Path):
    events = GENERIC["read_events"](run_dir / "events.csv")
    route = next(window for window in VFB["action_windows"](events) if window[0] == "route")
    return events, float(route[2]), float(route[3])


def swing_window_metrics(run_dir: Path, rope_length_m: float) -> Dict[str, object]:
    events, route_start, route_end = route_bounds(run_dir)
    actions = [event for event in events if str(event.get("name", "")).startswith("action_")]
    moves = [event for event in actions if any(name in str(event.get("name", ""))
                                               for name in VFB["MOVE_NAMES"])]
    holds = [event for event in actions if event not in moves]
    with (run_dir / "hang_joint_samples.csv").open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    t = np.asarray([finite(sample.get("boot_s_est")) for sample in samples], dtype=float)
    angle = np.asarray([finite(sample.get("hang_angle_deg")) for sample in samples], dtype=float)
    rate = np.asarray([finite(sample.get("hang_velocity_rad_s")) for sample in samples], dtype=float)
    energy = 0.5 * np.square(rope_length_m * rate) \
        + 9.80665 * rope_length_m * (1.0 - np.cos(np.radians(angle)))

    def event_mask(selected) -> np.ndarray:
        mask = np.zeros(t.shape, dtype=bool)
        for event in selected:
            mask |= ((t >= finite(event.get("start_boot_s"))) & (t <= finite(event.get("end_boot_s"))))
        return mask

    first_move_start = min(finite(event.get("start_boot_s")) for event in moves)
    windows = {
        "route": (t >= route_start) & (t <= route_end),
        "first_hold": event_mask(holds[:1]),
        "pre_first_move_1s": (t >= first_move_start - 1.0) & (t <= first_move_start),
        "movement": event_mask(moves),
        "normal_hover": event_mask(holds[:-1] if len(holds) > 1 else holds),
        "final_10s": (t >= route_end - 10.0) & (t <= route_end),
    }
    result: Dict[str, object] = {}
    for name, mask in windows.items():
        window_t, window_angle, window_rate, window_energy = t[mask], angle[mask], rate[mask], energy[mask]
        result.update({
            f"{name}_swing_angle_rms_deg": VFB["rms"](window_angle),
            f"{name}_swing_angle_peak_deg": VFB["peak"](window_angle),
            f"{name}_swing_rate_rms_rad_s": VFB["rms"](window_rate),
            f"{name}_swing_rate_peak_rad_s": VFB["peak"](window_rate),
            f"{name}_swing_energy_mean_j_kg": float(np.nanmean(window_energy)) if window_energy.size else math.nan,
            f"{name}_swing_energy_peak_j_kg": VFB["peak"](window_energy),
            f"{name}_swing_energy_integral_j_s_kg": VFB["integrate"](window_t, window_energy),
        })
    return result


def reference_xy_metrics(run_dir: Path) -> Dict[str, object]:
    events, route_start, route_end = route_bounds(run_dir)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["vehicle_local_position"])
    local = ulog.get_dataset("vehicle_local_position").data
    local_t, xy_error = VFB["reference_xy_errors"](local, events)
    route_mask = (local_t >= route_start) & (local_t <= route_end)
    movement_mask = VFB["combined_window_mask"](local_t, "movement", route_start, route_end, events)
    hover_mask = VFB["combined_window_mask"](local_t, "normal_hover", route_start, route_end, events)
    final_mask = (local_t >= route_end - 10.0) & (local_t <= route_end)
    endpoint_error = math.nan
    route_indices = np.flatnonzero(route_mask)
    if route_indices.size:
        endpoint_index = int(route_indices[-1])
        final_event = next(event for event in reversed(events)
                           if str(event.get("name", "")).startswith("action_"))
        rx, ry, _, _, _ = VFB["reference_sample"](final_event, float(local_t[endpoint_index]))
        endpoint_error = math.hypot(float(local["x"][endpoint_index]) - rx,
                                    float(local["y"][endpoint_index]) - ry)
    return {
        "xy_error_rms_m": VFB["rms"](xy_error[route_mask]),
        "xy_error_peak_m": VFB["peak"](xy_error[route_mask]),
        "moving_xy_error_rms_m": VFB["rms"](xy_error[movement_mask]),
        "hover_xy_error_rms_m": VFB["rms"](xy_error[hover_mask]),
        "final_10s_xy_error_rms_m": VFB["rms"](xy_error[final_mask]),
        "endpoint_xy_error_m": endpoint_error,
    }


def attitude_and_timing_metrics(run_dir: Path) -> Dict[str, object]:
    _, start, end = route_bounds(run_dir)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), [
        "vehicle_attitude", "vehicle_angular_velocity", "vehicle_attitude_setpoint", "vehicle_status",
        "control_allocator_status", "actuator_motors", "estimator_status", "sensor_combined", "cpuload",
    ])
    attitude = ulog.get_dataset("vehicle_attitude").data
    attitude_t = np.asarray(attitude["timestamp"], dtype=float) * 1.0e-6
    attitude_mask = (attitude_t >= start) & (attitude_t <= end)
    roll_deg, pitch_deg, _ = VFB["quaternion_to_euler_deg"](attitude)
    tilt_deg = np.degrees(np.arccos(np.clip(
        1.0 - 2.0 * (np.square(np.asarray(attitude["q[1]"], dtype=float))
                     + np.square(np.asarray(attitude["q[2]"], dtype=float))), -1.0, 1.0)))

    setpoint = ulog.get_dataset("vehicle_attitude_setpoint").data
    attitude_error = VFB["quaternion_tracking_error_deg"](attitude, setpoint, start, end)

    angular = ulog.get_dataset("vehicle_angular_velocity").data
    angular_t = np.asarray(angular["timestamp"], dtype=float) * 1.0e-6
    angular_mask = (angular_t >= start) & (angular_t <= end)
    roll_rate = np.asarray(angular["xyz[0]"], dtype=float)[angular_mask]
    pitch_rate = np.asarray(angular["xyz[1]"], dtype=float)[angular_mask]
    horizontal_rate = np.hypot(roll_rate, pitch_rate)

    status = ulog.get_dataset("vehicle_status").data
    status_t = np.asarray(status["timestamp"], dtype=float) * 1.0e-6
    status_mask = (status_t >= start) & (status_t <= end)
    failure_detector = np.asarray(status["failure_detector_status"], dtype=np.uint32)[status_mask]
    failsafe = np.asarray(status["failsafe"], dtype=bool)[status_mask]

    allocator = ulog.get_dataset("control_allocator_status").data
    allocator_t = np.asarray(allocator["timestamp"], dtype=float) * 1.0e-6
    allocator_mask = (allocator_t >= start) & (allocator_t <= end)
    torque_achieved = np.asarray(allocator["torque_setpoint_achieved"], dtype=bool)[allocator_mask]
    thrust_achieved = np.asarray(allocator["thrust_setpoint_achieved"], dtype=bool)[allocator_mask]

    motors = ulog.get_dataset("actuator_motors").data
    motor_t = np.asarray(motors["timestamp"], dtype=float) * 1.0e-6
    motor_mask = (motor_t >= start) & (motor_t <= end)
    motor_values = np.column_stack([
        np.asarray(motors[f"control[{index}]"], dtype=float)[motor_mask] for index in range(4)
    ])
    finite_motors = motor_values[np.isfinite(motor_values)]

    estimator = ulog.get_dataset("estimator_status").data
    estimator_t = np.asarray(estimator["timestamp"], dtype=float) * 1.0e-6
    estimator_mask = (estimator_t >= start) & (estimator_t <= end)
    time_slip = np.asarray(estimator["time_slip"], dtype=float)[estimator_mask]
    timeout_flags = np.asarray(estimator["timeout_flags"], dtype=np.uint32)[estimator_mask]
    fault_flags = np.asarray(estimator["filter_fault_flags"], dtype=np.uint32)[estimator_mask]

    sensor = ulog.get_dataset("sensor_combined").data
    sensor_t = np.asarray(sensor["timestamp"], dtype=float) * 1.0e-6
    sensor_t = sensor_t[(sensor_t >= start) & (sensor_t <= end)]
    sensor_gaps = np.diff(sensor_t)

    cpu = ulog.get_dataset("cpuload").data
    cpu_t = np.asarray(cpu["timestamp"], dtype=float) * 1.0e-6
    cpu_load = np.asarray(cpu["load"], dtype=float)[(cpu_t >= start) & (cpu_t <= end)]

    result: Dict[str, object] = {
        "roll_rms_deg": VFB["rms"](roll_deg[attitude_mask]),
        "roll_peak_abs_deg": VFB["peak"](roll_deg[attitude_mask]),
        "pitch_rms_deg": VFB["rms"](pitch_deg[attitude_mask]),
        "pitch_peak_abs_deg": VFB["peak"](pitch_deg[attitude_mask]),
        "tilt_rms_deg": VFB["rms"](tilt_deg[attitude_mask]),
        "tilt_p95_deg": VFB["percentile"](tilt_deg[attitude_mask], 95.0),
        "tilt_peak_deg": VFB["peak"](tilt_deg[attitude_mask]),
        "attitude_tracking_error_rms_deg": VFB["rms"](attitude_error),
        "attitude_tracking_error_p95_deg": VFB["percentile"](attitude_error, 95.0),
        "attitude_tracking_error_peak_deg": VFB["peak"](attitude_error),
        "horizontal_body_rate_rms_rad_s": VFB["rms"](horizontal_rate),
        "horizontal_body_rate_p95_rad_s": VFB["percentile"](horizontal_rate, 95.0),
        "horizontal_body_rate_peak_rad_s": VFB["peak"](horizontal_rate),
        "attitude_quat_reset_count": int(np.ptp(np.asarray(attitude["quat_reset_counter"], dtype=np.int64)[attitude_mask])),
        "failure_detector_nonzero_ratio": float(np.mean(failure_detector != 0)) if failure_detector.size else math.nan,
        "vehicle_status_failsafe_ratio": float(np.mean(failsafe)) if failsafe.size else math.nan,
        "allocator_torque_not_achieved_ratio": float(np.mean(~torque_achieved)) if torque_achieved.size else math.nan,
        "allocator_thrust_not_achieved_ratio": float(np.mean(~thrust_achieved)) if thrust_achieved.size else math.nan,
        "motor_output_min": float(np.min(finite_motors)) if finite_motors.size else math.nan,
        "motor_output_max": float(np.max(finite_motors)) if finite_motors.size else math.nan,
        "motor_high_saturation_sample_ratio": float(np.mean(np.any(motor_values >= 0.99, axis=1))),
        "ekf_time_slip_delta_s": float(time_slip[-1] - time_slip[0]) if time_slip.size >= 2 else math.nan,
        "ekf_timeout_nonzero_ratio": float(np.mean(timeout_flags != 0)) if timeout_flags.size else math.nan,
        "ekf_filter_fault_nonzero_ratio": float(np.mean(fault_flags != 0)) if fault_flags.size else math.nan,
        "sensor_combined_gap_peak_ms": 1000.0 * float(np.max(sensor_gaps)) if sensor_gaps.size else math.nan,
        "cpu_load_mean_pct": 100.0 * float(np.mean(cpu_load)) if cpu_load.size else math.nan,
        "cpu_load_peak_pct": 100.0 * float(np.max(cpu_load)) if cpu_load.size else math.nan,
    }
    limits = {
        "tilt_peak_deg": 20.0,
        "attitude_tracking_error_peak_deg": 10.0,
        "horizontal_body_rate_peak_rad_s": 2.0,
        "attitude_quat_reset_count": 0.0,
        "failure_detector_nonzero_ratio": 0.0,
        "vehicle_status_failsafe_ratio": 0.0,
        "allocator_torque_not_achieved_ratio": 0.01,
        "motor_high_saturation_sample_ratio": 0.05,
    }
    rejections = [name for name, limit in limits.items()
                  if not math.isfinite(finite(result.get(name))) or finite(result.get(name)) > limit]
    result["attitude_control_valid"] = not rejections
    result["attitude_control_rejection_reasons"] = ";".join(rejections)
    return result


def base_validity(run_dir: Path):
    validity = json.loads((run_dir / "validity.json").read_text(encoding="utf-8"))
    strict_rejections = [str(item) for item in validity.get("rejection_reasons", [])]
    z_rejections = [item for item in strict_rejections if item in Z_OBSERVATION_ONLY_CHECKS]
    non_z_rejections = [item for item in strict_rejections if item not in Z_OBSERVATION_ONLY_CHECKS]
    analysis_valid = bool(validity.get("completed", False)) and not non_z_rejections
    return validity, z_rejections, non_z_rejections, analysis_valid


def analyze_pid(run_dir: Path):
    row = dict(GENERIC["analyze_run"](run_dir))
    validity, z_rejections, non_z_rejections, analysis_valid = base_validity(run_dir)
    row.update(attitude_and_timing_metrics(run_dir))
    row.update(swing_window_metrics(run_dir, 0.6))
    row.update(reference_xy_metrics(run_dir))
    row.update({
        "mode": PID_MODE,
        "controller": "PID+AS",
        "strict_base_valid": bool(validity.get("valid", False)),
        "analysis_base_valid": analysis_valid,
        "z_observation_only": bool(z_rejections),
        "z_observation_reasons": ";".join(z_rejections),
        "non_z_rejection_reasons": ";".join(non_z_rejections),
        "requested_weight_median": 0.0,
        "effective_weight_median": 0.0,
        "fallback_ratio": 0.0,
    })
    row["valid"] = bool(analysis_valid and row.get("attitude_control_valid"))
    row["analysis_rejection_reasons"] = ";".join(non_z_rejections + (
        [] if row.get("attitude_control_valid") else [str(row.get("attitude_control_rejection_reasons"))]
    ))
    return row


def analyze_full(run_dir: Path):
    row, windows, validity = VFB["analyze_run"](FULL_MODE, run_dir)
    row.update(swing_window_metrics(run_dir, 0.6))
    row["controller"] = "LADRC2+AS+PAS+VFB_W05"
    return row, windows, validity


def pct_change(test, baseline) -> float:
    test, baseline = finite(test), finite(baseline)
    return 100.0 * (test / baseline - 1.0) if math.isfinite(test) and math.isfinite(baseline) and abs(baseline) > 1.0e-12 else math.nan


def compare(pid: Dict[str, object], full: Dict[str, object]) -> Dict[str, object]:
    result: Dict[str, object] = {}
    metrics = (
        "xy_error_rms_m", "xy_error_peak_m", "moving_xy_error_rms_m", "hover_xy_error_rms_m",
        "along_path_speed_rmse_m_s", "cross_path_velocity_rms_m_s", "endpoint_xy_error_m",
        "swing_angle_rms_deg", "swing_angle_peak_deg", "swing_rate_rms_rad_s", "swing_rate_peak_rad_s",
        "swing_energy_mean_j_kg", "swing_energy_peak_j_kg", "swing_energy_integral_j_s_kg",
        "final_5s_swing_angle_rms_deg", "final_acc_rms_m_s2", "final_acc_p95_m_s2",
        "final_jerk_rms_m_s3", "final_jerk_p95_m_s3", "tilt_rms_deg", "tilt_peak_deg",
        "attitude_tracking_error_rms_deg", "attitude_tracking_error_peak_deg",
        "first_hold_swing_angle_rms_deg", "pre_first_move_1s_swing_angle_rms_deg",
        "movement_swing_angle_rms_deg", "movement_swing_angle_peak_deg",
        "movement_swing_energy_integral_j_s_kg", "normal_hover_swing_angle_rms_deg",
        "final_10s_swing_angle_rms_deg",
    )
    for metric in metrics:
        result[f"{metric}_change_pct"] = pct_change(full.get(metric), pid.get(metric))
    result["swing_rms_improvement_pct"] = -finite(result.get("swing_angle_rms_deg_change_pct"))
    result["energy_integral_improvement_pct"] = -finite(result.get("swing_energy_integral_j_s_kg_change_pct"))
    result["initial_angle_difference_deg"] = abs(finite(full.get("initial_swing_angle_rms_deg"))
                                                  - finite(pid.get("initial_swing_angle_rms_deg")))
    result["initial_rate_difference_rad_s"] = abs(finite(full.get("initial_swing_rate_rms_rad_s"))
                                                   - finite(pid.get("initial_swing_rate_rms_rad_s")))
    result["performance_threshold_pass"] = bool(
        finite(result["swing_rms_improvement_pct"]) >= 10.0
        or finite(result["energy_integral_improvement_pct"]) >= 15.0
    )
    result["position_threshold_pass"] = bool(
        finite(full.get("xy_error_rms_m")) < 0.10
        and finite(full.get("xy_error_rms_m")) <= 1.35 * finite(pid.get("xy_error_rms_m"))
    )
    result["peak_threshold_pass"] = bool(finite(full.get("swing_angle_peak_deg")) < 8.0)
    result["safety_threshold_pass"] = bool(
        pid.get("valid") and full.get("valid")
        and pid.get("attitude_control_valid") and full.get("attitude_control_valid")
        and finite(full.get("total_envelope_trigger_ratio")) < 0.05
    )
    result["initial_pairing_pass"] = bool(
        finite(result["initial_angle_difference_deg"]) <= 0.20
        and finite(result["initial_rate_difference_rad_s"]) <= 0.02
    )
    result["freeze_structure"] = bool(
        result["performance_threshold_pass"] and result["position_threshold_pass"]
        and result["peak_threshold_pass"] and result["safety_threshold_pass"]
        and result["initial_pairing_pass"]
    )
    result["decision"] = "FREEZE_FOR_FORMAL_VALIDATION" if result["freeze_structure"] else "DO_NOT_FREEZE_AS_SUPERIOR_STRUCTURE"
    return result


def markdown(rows, comparison) -> str:
    by_mode = {str(row["mode"]): row for row in rows}
    pid, full = by_mode[PID_MODE], by_mode[FULL_MODE]
    lines = [
        "# ZD680 标准工况 PID+AS 与 W05 完整方案自动对比",
        "",
        f"生成时间：{datetime.now().isoformat(timespec='seconds')}",
        "",
        "> 单次结构冻结验证，不是论文最终重复统计数据；Z 超限按约定只作观察警告。",
        "",
        "## 运行结果",
        "",
        "| 模式 | 分析有效 | 严格Z有效 | 初始摆角/摆速 | XY RMS/峰值(m) | 摆角 RMS/峰值(°) | 能量积分 | 末5s RMS(°) |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['controller']} | {'是' if row.get('valid') else '否'} | {'是' if row.get('strict_base_valid') else '否'} | "
            f"{fmt(row.get('initial_swing_angle_rms_deg'))}°/{fmt(row.get('initial_swing_rate_rms_rad_s'),4)} | "
            f"{fmt(row.get('xy_error_rms_m'),4)}/{fmt(row.get('xy_error_peak_m'),4)} | "
            f"{fmt(row.get('swing_angle_rms_deg'))}/{fmt(row.get('swing_angle_peak_deg'))} | "
            f"{fmt(row.get('swing_energy_integral_j_s_kg'),4)} | {fmt(row.get('final_5s_swing_angle_rms_deg'))} |"
        )
    lines += [
        "",
        "## FULL 相对 PID+AS",
        "",
        "| 指标 | 变化 |",
        "|---|---:|",
        f"| XY RMS | {fmt(comparison.get('xy_error_rms_m_change_pct'),1)}% |",
        f"| 摆角 RMS | {fmt(comparison.get('swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 摆角峰值 | {fmt(comparison.get('swing_angle_peak_deg_change_pct'),1)}% |",
        f"| 能量积分 | {fmt(comparison.get('swing_energy_integral_j_s_kg_change_pct'),1)}% |",
        f"| 末5s摆角 RMS | {fmt(comparison.get('final_5s_swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 最终加速度 RMS | {fmt(comparison.get('final_acc_rms_m_s2_change_pct'),1)}% |",
        f"| jerk P95 | {fmt(comparison.get('final_jerk_p95_m_s3_change_pct'),1)}% |",
        "",
        "## 分窗口摆动（排查初态影响）",
        "",
        "| 窗口 | PID+AS 摆角RMS(°) | FULL 摆角RMS(°) | FULL变化 |",
        "|---|---:|---:|---:|",
        f"| 首个5秒保持 | {fmt(pid.get('first_hold_swing_angle_rms_deg'))} | {fmt(full.get('first_hold_swing_angle_rms_deg'))} | {fmt(comparison.get('first_hold_swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 首次运动前1秒 | {fmt(pid.get('pre_first_move_1s_swing_angle_rms_deg'))} | {fmt(full.get('pre_first_move_1s_swing_angle_rms_deg'))} | {fmt(comparison.get('pre_first_move_1s_swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 四个纯运动段 | {fmt(pid.get('movement_swing_angle_rms_deg'))} | {fmt(full.get('movement_swing_angle_rms_deg'))} | {fmt(comparison.get('movement_swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 普通悬停段 | {fmt(pid.get('normal_hover_swing_angle_rms_deg'))} | {fmt(full.get('normal_hover_swing_angle_rms_deg'))} | {fmt(comparison.get('normal_hover_swing_angle_rms_deg_change_pct'),1)}% |",
        f"| 末端10秒 | {fmt(pid.get('final_10s_swing_angle_rms_deg'))} | {fmt(full.get('final_10s_swing_angle_rms_deg'))} | {fmt(comparison.get('final_10s_swing_angle_rms_deg_change_pct'),1)}% |",
        "",
        "## 姿态与安全",
        "",
        "| 模式 | 倾角 RMS/P95/峰值(°) | 姿态误差 RMS/峰值(°) | 水平角速度 RMS/峰值(rad/s) | 电机范围/高饱和率 | failsafe/故障检测 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['controller']} | {fmt(row.get('tilt_rms_deg'))}/{fmt(row.get('tilt_p95_deg'))}/{fmt(row.get('tilt_peak_deg'))} | "
            f"{fmt(row.get('attitude_tracking_error_rms_deg'))}/{fmt(row.get('attitude_tracking_error_peak_deg'))} | "
            f"{fmt(row.get('horizontal_body_rate_rms_rad_s'))}/{fmt(row.get('horizontal_body_rate_peak_rad_s'))} | "
            f"{fmt(row.get('motor_output_min'))}～{fmt(row.get('motor_output_max'))}/"
            f"{fmt(100.0 * finite(row.get('motor_high_saturation_sample_ratio')),3)}% | "
            f"{fmt(100.0 * finite(row.get('vehicle_status_failsafe_ratio')),1)}%/"
            f"{fmt(100.0 * finite(row.get('failure_detector_nonzero_ratio')),1)}% |"
        )
    lines += [
        "",
        "## 冻结门槛",
        "",
        f"- 摆角 RMS 改善 >=10% 或能量积分改善 >=15%：{'通过' if comparison['performance_threshold_pass'] else '不通过'}。",
        f"- XY RMS <0.10 m 且不超过 PID+AS 的 1.35 倍：{'通过' if comparison['position_threshold_pass'] else '不通过'}。",
        f"- FULL 摆角峰值 <8°：{'通过' if comparison['peak_threshold_pass'] else '不通过'}。",
        f"- 两组安全、姿态和饱和检查：{'通过' if comparison['safety_threshold_pass'] else '不通过'}。",
        f"- 初态相对配对（摆角差 <=0.20°、摆速差 <=0.02 rad/s）：{'通过' if comparison['initial_pairing_pass'] else '不通过'}。",
        "",
        f"自动结论：`{comparison['decision']}`。",
        "",
        "完整的 FULL 分窗口功率和频带数据见 `full_window_metrics.csv`。",
        "",
    ]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    args = parser.parse_args()
    output = args.output_dir or args.root / "report"
    output.mkdir(parents=True, exist_ok=True)
    found = latest_runs(args.root)
    if any(mode not in found for mode in MODES):
        missing = [mode for mode in MODES if mode not in found]
        raise SystemExit("missing runs: " + ",".join(missing))
    pid = analyze_pid(found[PID_MODE])
    full, windows, full_validity = analyze_full(found[FULL_MODE])
    rows = [pid, full]
    comparison = compare(pid, full)
    write_csv(output / "validation_runs.csv", rows)
    write_csv(output / "validation_comparison.csv", [comparison])
    write_csv(output / "full_window_metrics.csv", windows)
    payload = {
        "generated_local": datetime.now().isoformat(timespec="seconds"),
        "runs": rows,
        "comparison": comparison,
        "full_vfb_validity": full_validity,
    }
    (output / "validation_summary.json").write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "validation_summary.md").write_text(markdown(rows, comparison), encoding="utf-8")
    print(output / "validation_summary.md")
    return 0 if all(bool(row.get("valid")) for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
