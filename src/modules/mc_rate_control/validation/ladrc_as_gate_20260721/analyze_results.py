#!/usr/bin/env python3
"""Analyze the bounded ZD680 rate-LADRC + anti-swing gate experiment.

The script deliberately keeps gate-invalid screening runs in the manifest and
diagnostic metrics.  They are never promoted into formal paired statistics.
It depends only on Python 3, NumPy, and pyulog; plots are written as SVG so the
analysis does not depend on the locally broken matplotlib binary package.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
from typing import Any, Iterable

import numpy as np
from pyulog import ULog


ROOT = Path(__file__).resolve().parent
RUNS = ROOT / "runs"
REPORT = ROOT / "report"
PLOTS = ROOT / "plots"
ROPE_LENGTH_M = 0.60
G = 9.80665
TIMING_TOPICS = (
    "vehicle_angular_velocity",
    "vehicle_rates_setpoint",
    "vehicle_attitude",
    "vehicle_attitude_setpoint",
    "vehicle_torque_setpoint",
    "vehicle_local_position",
    "vehicle_local_position_setpoint",
    "control_allocator_status",
    "actuator_motors",
)
LOAD_TOPICS = list(TIMING_TOPICS) + [
    "trajectory_setpoint", "debug_array", "vehicle_status", "failure_detector_status",
]


def finite(value: Any, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def rms(values: Iterable[float]) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(np.square(values)))) if values.size else math.nan


def peak(values: Iterable[float]) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.max(np.abs(values))) if values.size else math.nan


def p95(values: Iterable[float]) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(np.abs(values), 95.0)) if values.size else math.nan


def iae(t: np.ndarray, values: np.ndarray) -> float:
    good = np.isfinite(t) & np.isfinite(values)
    return float(np.trapezoid(np.abs(values[good]), t[good])) if np.count_nonzero(good) >= 2 else math.nan


def write_csv(path: Path, rows: list[dict[str, Any]], fields: list[str] | None = None) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if fields is None:
        fields = []
        for row in rows:
            for key in row:
                if key not in fields:
                    fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dataset(ulog: ULog, name: str, multi_id: int = 0) -> dict[str, np.ndarray] | None:
    try:
        return ulog.get_dataset(name, multi_instance=multi_id).data
    except (KeyError, IndexError, ValueError):
        return None


def debug_dataset(ulog: ULog, debug_id: int) -> dict[str, np.ndarray] | None:
    for item in ulog.data_list:
        if item.name == "debug_array" and len(item.data.get("id", [])) and int(item.data["id"][0]) == debug_id:
            return item.data
    return None


def timestamps(data: dict[str, np.ndarray], sample: bool = False) -> np.ndarray:
    key = "timestamp_sample" if sample and "timestamp_sample" in data else "timestamp"
    return np.asarray(data[key], dtype=float) * 1.0e-6


def window_values(data: dict[str, np.ndarray], field: str, start: float, end: float,
                  sample: bool = False) -> tuple[np.ndarray, np.ndarray]:
    t = timestamps(data, sample)
    mask = (t >= start) & (t <= end)
    return t[mask], np.asarray(data[field], dtype=float)[mask]


def interp(t_source: np.ndarray, value: np.ndarray, t_target: np.ndarray) -> np.ndarray:
    good = np.isfinite(t_source) & np.isfinite(value)
    if np.count_nonzero(good) < 2:
        return np.full(t_target.shape, math.nan)
    ts, vs = t_source[good], value[good]
    order = np.argsort(ts)
    result = np.interp(t_target, ts[order], vs[order], left=np.nan, right=np.nan)
    return result


def quaternion_euler(data: dict[str, np.ndarray], prefix: str) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    w = np.asarray(data[f"{prefix}[0]"], dtype=float)
    x = np.asarray(data[f"{prefix}[1]"], dtype=float)
    y = np.asarray(data[f"{prefix}[2]"], dtype=float)
    z = np.asarray(data[f"{prefix}[3]"], dtype=float)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def wrap_pi(value: np.ndarray) -> np.ndarray:
    return (value + np.pi) % (2.0 * np.pi) - np.pi


def choose_window(events: list[dict[str, str]]) -> tuple[str, float, float, bool, str, dict[str, Any]]:
    actions = [event for event in events if event["name"].startswith("action_") and event["success"] == "1"]
    failed = [event for event in events if event["success"] == "0"]
    initial = next((event for event in events if event["name"] == "hang_initial_condition"), None)
    observed: dict[str, Any] = {}
    if initial:
        try:
            observed = json.loads(initial["params_json"]).get("observed", {})
        except json.JSONDecodeError:
            pass
    if actions:
        return "actions", finite(actions[0]["start_boot_s"]), finite(actions[-1]["end_boot_s"]), True, "", observed
    if failed:
        event = failed[-1]
        reason = "gate_failed:" + event["name"]
        return event["name"], finite(event["start_boot_s"]), finite(event["end_boot_s"]), False, reason, observed
    phases = [event for event in events if event["event_type"] == "flight_phase"]
    if phases:
        return "last_phase", finite(phases[-1]["start_boot_s"]), finite(phases[-1]["end_boot_s"]), False, "no_action_window", observed
    return "missing", math.nan, math.nan, False, "missing_events", observed


def timing_row(tag: str, signal: str, t_all: np.ndarray, start: float, end: float) -> dict[str, Any]:
    t = np.asarray(t_all, dtype=float)
    t = t[np.isfinite(t)]
    t = t[(t >= start) & (t <= end)]
    diffs = np.diff(t)
    positive = diffs[diffs > 0.0]
    median = float(np.median(positive)) if positive.size else math.nan
    threshold = max(3.0 * median, 0.020) if math.isfinite(median) else math.nan
    max_gap = float(np.max(diffs)) if diffs.size else math.nan
    bad_gaps = int(np.count_nonzero(diffs > threshold)) if math.isfinite(threshold) else -1
    nonpositive = int(np.count_nonzero(diffs <= 0.0))
    duration = max(0.0, end - start)
    # Coverage is the fraction not lost to an unexplained long gap. Ordinary
    # sample spacing is covered by the sample stream itself; only the part of
    # a gap beyond the declared hard threshold counts as missing. This avoids
    # penalizing legitimate 5 Hz status topics solely because an event edge is
    # not phase-aligned with a publication, while a historical 52 ms sensor
    # gap still fails independently through long_gap_count.
    if len(t) and math.isfinite(threshold) and duration > 0.0:
        missing = float(np.sum(np.maximum(diffs - threshold, 0.0)))
        missing += max(0.0, t[0] - start - threshold)
        missing += max(0.0, end - t[-1] - threshold)
        coverage = max(0.0, 100.0 * (duration - missing) / duration)
    else:
        coverage = math.nan
    boundary_ok = bool(len(t) and t[0] - start <= threshold and end - t[-1] <= threshold) if math.isfinite(threshold) else False
    valid = bool(len(t) >= 2 and nonpositive == 0 and bad_gaps == 0 and coverage >= 99.0 and boundary_ok)
    return {
        "run_tag": tag, "signal": signal, "window_start_s": start, "window_end_s": end,
        "sample_count": len(t), "median_period_ms": 1000.0 * median,
        "max_gap_ms": 1000.0 * max_gap, "gap_limit_ms": 1000.0 * threshold,
        "long_gap_count": bad_gaps, "nonpositive_interval_count": nonpositive,
        "coverage_pct": coverage, "boundary_covered": int(boundary_ok), "timing_valid": int(valid),
    }


def signal_metrics(row: dict[str, Any], prefix: str, t: np.ndarray, value: np.ndarray) -> None:
    row[f"{prefix}_rms"] = rms(value)
    row[f"{prefix}_peak"] = peak(value)
    row[f"{prefix}_p95"] = p95(value)
    row[f"{prefix}_iae"] = iae(t, value)


def longest_true_duration(t: np.ndarray, condition: np.ndarray) -> float:
    best = current = 0.0
    for index in range(1, len(t)):
        if condition[index - 1] and condition[index]:
            current += max(0.0, float(t[index] - t[index - 1]))
            best = max(best, current)
        else:
            current = 0.0
    return best


def analyze_run(gate_path: Path) -> tuple[dict[str, Any], list[dict[str, Any]], dict[str, tuple[np.ndarray, np.ndarray]]]:
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    run_dir = Path(gate["run_dir"])
    events = read_csv(run_dir / "events.csv")
    window_name, start, end, gate_pass, gate_reason, observed = choose_window(events)
    ulog_path = run_dir / "position_offboard.ulg"
    ulog = ULog(str(ulog_path), LOAD_TOPICS)
    tag = gate["tag"]
    row: dict[str, Any] = {
        "run_tag": tag, "stage": gate["stage"], "group": gate["group"], "condition": gate["condition"],
        "parameter_set": gate["parameter_set"], "repetition": gate["repetition"],
        "analysis_window": window_name, "window_start_s": start, "window_end_s": end,
        "window_duration_s": end - start, "initial_gate_pass": int(gate_pass),
        "initial_gate_reason": gate_reason,
        "initial_swing_angle_rms_deg": finite(observed.get("angle_rms_deg")),
        "initial_swing_rate_rms_rad_s": finite(observed.get("rate_rms_rad_s")),
        "initial_xy_error_m": finite(observed.get("xy_error_m")),
        "initial_horizontal_speed_m_s": finite(observed.get("horizontal_speed_m_s")),
    }
    timing: list[dict[str, Any]] = []
    for name in TIMING_TOPICS:
        data = dataset(ulog, name)
        timing.append(timing_row(tag, name, timestamps(data, sample=True) if data else np.array([]), start, end))

    joint_path = run_dir / "hang_joint_samples.csv"
    joints = read_csv(joint_path)
    joint_t_all = np.asarray([finite(item["boot_s_est"]) for item in joints])
    timing.append(timing_row(tag, "hang_joint_samples", joint_t_all, start, end))
    row["timing_valid"] = int(all(item["timing_valid"] for item in timing))
    row["timing_invalid_signals"] = ";".join(item["signal"] for item in timing if not item["timing_valid"])

    series: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    angular = dataset(ulog, "vehicle_angular_velocity")
    rate_sp = dataset(ulog, "vehicle_rates_setpoint")
    if angular and rate_sp:
        ta = timestamps(angular, sample=True)
        mask = (ta >= start) & (ta <= end)
        ta = ta[mask]
        tsp = timestamps(rate_sp)
        for axis, actual_field, sp_field in (("roll", "xyz[0]", "roll"), ("pitch", "xyz[1]", "pitch"), ("yaw", "xyz[2]", "yaw")):
            actual = np.asarray(angular[actual_field], dtype=float)[mask]
            reference = interp(tsp, np.asarray(rate_sp[sp_field], dtype=float), ta)
            error = actual - reference
            signal_metrics(row, f"{axis}_rate_error_rad_s", ta, error)
            row[f"{axis}_rate_actual_rms_rad_s"] = rms(actual)
            series[f"{axis}_rate_error"] = (ta, error)
            series[f"{axis}_rate_actual"] = (ta, actual)
            series[f"{axis}_rate_setpoint"] = (ta, reference)

    attitude = dataset(ulog, "vehicle_attitude")
    attitude_sp = dataset(ulog, "vehicle_attitude_setpoint")
    if attitude and attitude_sp:
        ta = timestamps(attitude, sample=True)
        mask = (ta >= start) & (ta <= end)
        actual_angles = quaternion_euler(attitude, "q")
        sp_angles = quaternion_euler(attitude_sp, "q_d")
        tsp = timestamps(attitude_sp)
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            actual = actual_angles[index][mask]
            reference = interp(tsp, sp_angles[index], ta[mask])
            error_deg = np.degrees(wrap_pi(actual - reference))
            signal_metrics(row, f"{axis}_attitude_error_deg", ta[mask], error_deg)
            series[f"{axis}_attitude_actual_deg"] = (ta[mask], np.degrees(actual))
            series[f"{axis}_attitude_setpoint_deg"] = (ta[mask], np.degrees(reference))

    torque = dataset(ulog, "vehicle_torque_setpoint")
    if torque:
        tt = timestamps(torque, sample=True)
        mask = (tt >= start) & (tt <= end)
        tt = tt[mask]
        for index, axis in enumerate(("roll", "pitch", "yaw")):
            values = np.asarray(torque[f"xyz[{index}]"], dtype=float)[mask]
            signal_metrics(row, f"{axis}_normalized_torque", tt, values)
            row[f"{axis}_normalized_torque_diff_rms"] = rms(np.diff(values))
            row[f"{axis}_normalized_torque_total_variation"] = float(np.sum(np.abs(np.diff(values))))
            series[f"{axis}_torque"] = (tt, values)

    local = dataset(ulog, "vehicle_local_position")
    if local:
        tl = timestamps(local)
        mask = (tl >= start) & (tl <= end)
        tl = tl[mask]
        target_event = next((event for event in events if event["name"] in ("hold_current_position_check", "action_01_hold")), None)
        params: dict[str, Any] = {}
        if target_event:
            try:
                params = json.loads(target_event["params_json"])
            except json.JSONDecodeError:
                pass
        ex = np.asarray(local["x"], dtype=float)[mask] - finite(params.get("x_m"), 0.0)
        ey = np.asarray(local["y"], dtype=float)[mask] - finite(params.get("y_m"), 0.0)
        for axis, error in (("x", ex), ("y", ey)):
            signal_metrics(row, f"{axis}_position_error_m", tl, error)
        eres = np.hypot(ex, ey)
        signal_metrics(row, "xy_position_error_m", tl, eres)
        vxy = np.hypot(np.asarray(local["vx"], dtype=float)[mask], np.asarray(local["vy"], dtype=float)[mask])
        row["horizontal_speed_rms_m_s"] = rms(vxy)
        row["horizontal_speed_peak_m_s"] = peak(vxy)
        instant_gate = (eres <= 0.10) & (vxy <= 0.05)
        row["position_gate_sample_pass_ratio"] = float(np.mean(instant_gate)) if instant_gate.size else math.nan
        row["position_gate_longest_continuous_s"] = longest_true_duration(tl, instant_gate)
        series["xy_error"] = (tl, eres)
        series["x_error"] = (tl, ex)
        series["y_error"] = (tl, ey)

    jt = joint_t_all
    jmask = (jt >= start) & (jt <= end)
    jt = jt[jmask]
    jr = np.asarray([finite(item["roll_position_rad"]) for item in joints])[jmask]
    jp = np.asarray([finite(item["pitch_position_rad"]) for item in joints])[jmask]
    jrv = np.asarray([finite(item["roll_velocity_rad_s"]) for item in joints])[jmask]
    jpv = np.asarray([finite(item["pitch_velocity_rad_s"]) for item in joints])[jmask]
    jres = np.hypot(jr, jp)
    jrate = np.hypot(jrv, jpv)
    energy = 0.5 * np.square(ROPE_LENGTH_M * jrate) + G * ROPE_LENGTH_M * (1.0 - np.cos(jres))
    signal_metrics(row, "swing_roll_angle_deg", jt, np.degrees(jr))
    signal_metrics(row, "swing_pitch_angle_deg", jt, np.degrees(jp))
    signal_metrics(row, "swing_resultant_angle_deg", jt, np.degrees(jres))
    row["swing_resultant_tail5_rms_deg"] = rms(np.degrees(jres[jt >= end - 5.0]))
    row["swing_resultant_rate_rms_rad_s"] = rms(jrate)
    row["swing_specific_energy_mean_j_kg"] = float(np.mean(energy)) if energy.size else math.nan
    row["swing_specific_energy_peak_j_kg"] = peak(energy)
    row["swing_specific_energy_integral_j_s_kg"] = float(np.trapezoid(energy, jt)) if energy.size >= 2 else math.nan
    series["swing_angle"] = (jt, np.degrees(jres))
    series["swing_roll_deg"] = (jt, np.degrees(jr))
    series["swing_pitch_deg"] = (jt, np.degrees(jp))
    series["swing_energy"] = (jt, energy)

    position_sp = dataset(ulog, "vehicle_local_position_setpoint")
    if position_sp:
        tp = timestamps(position_sp)
        mask = (tp >= start) & (tp <= end)
        tp = tp[mask]
        accel = np.column_stack([np.asarray(position_sp[f"acceleration[{i}]"], dtype=float)[mask] for i in range(2)])
        accel_norm = np.linalg.norm(accel, axis=1)
        row["horizontal_acceleration_command_rms_m_s2"] = rms(accel_norm)
        if len(tp) >= 2:
            dt = np.diff(tp)
            jerk = np.linalg.norm(np.diff(accel, axis=0), axis=1) / dt
            row["horizontal_acceleration_jerk_p95_m_s3"] = p95(jerk)

    allocator = dataset(ulog, "control_allocator_status")
    if allocator:
        _, achieved = window_values(allocator, "torque_setpoint_achieved", start, end)
        row["allocator_torque_not_achieved_ratio"] = float(np.mean(achieved < 0.5)) if achieved.size else math.nan
    motors = dataset(ulog, "actuator_motors")
    if motors:
        tm = timestamps(motors)
        mask = (tm >= start) & (tm <= end)
        values = np.column_stack([np.asarray(motors[f"control[{i}]"], dtype=float)[mask] for i in range(4)])
        values = np.where(np.isfinite(values), values, np.nan)
        row["motor_output_peak"] = float(np.nanmax(values)) if values.size else math.nan
        row["motor_high_saturation_ratio"] = float(np.mean(np.any(values >= 0.99, axis=1))) if values.size else math.nan
        row["motor_low_saturation_ratio"] = float(np.mean(np.any(values <= 0.01, axis=1))) if values.size else math.nan

    ladrc = debug_dataset(ulog, 682)
    if ladrc:
        td = timestamps(ladrc)
        mask = (td >= start) & (td <= end)
        td = td[mask]
        mapping = {
            "ladrc_z1_roll_rad_s": 21, "ladrc_z1_pitch_rad_s": 22, "ladrc_z1_yaw_rad_s": 23,
            "ladrc_z2_roll_rad_s2": 24, "ladrc_z2_pitch_rad_s2": 25, "ladrc_z2_yaw_rad_s2": 26,
            "ladrc_raw_roll": 27, "ladrc_raw_pitch": 28, "ladrc_raw_yaw": 29,
        }
        for key, index in mapping.items():
            values = np.asarray(ladrc[f"data[{index}]"], dtype=float)[mask]
            row[f"{key}_rms"] = rms(values)
            row[f"{key}_peak"] = peak(values)
            if key.startswith("ladrc_z2") or key.startswith("ladrc_raw"):
                series[key] = (td, values)
        for axis, index in (("roll", 30), ("pitch", 31), ("yaw", 32)):
            limited = np.asarray(ladrc[f"data[{index}]"], dtype=float)[mask]
            row[f"ladrc_local_limit_{axis}_ratio"] = float(np.mean(limited > 0.5)) if limited.size else math.nan
            series[f"ladrc_limit_{axis}"] = (td, limited)

    hangas = debug_dataset(ulog, 681)
    if hangas:
        th = timestamps(hangas)
        mask = (th >= start) & (th <= end)
        ax = np.asarray(hangas["data[16]"], dtype=float)[mask]
        ay = np.asarray(hangas["data[17]"], dtype=float)[mask]
        row["as_applied_acceleration_rms_m_s2"] = rms(np.hypot(ax, ay))
    hangcoord = debug_dataset(ulog, 684)
    if hangcoord:
        th = timestamps(hangcoord)
        mask = (th >= start) & (th <= end)
        sat = np.asarray(hangcoord["data[27]"], dtype=float)[mask]
        row["horizontal_acceleration_saturation_ratio"] = float(np.mean(sat > 0.5)) if sat.size else math.nan

    status = dataset(ulog, "vehicle_status")
    if status:
        _, failsafe = window_values(status, "failsafe", start, end)
        row["vehicle_failsafe_ratio"] = float(np.mean(failsafe > 0.5)) if failsafe.size else math.nan
    failure = dataset(ulog, "failure_detector_status")
    if failure:
        tf = timestamps(failure)
        mask = (tf >= start) & (tf <= end)
        flags = [field for field in failure if field.startswith("fd_")]
        any_failure = np.zeros(np.count_nonzero(mask), dtype=bool)
        for field in flags:
            any_failure |= np.asarray(failure[field], dtype=bool)[mask]
        row["failure_detector_ratio"] = float(np.mean(any_failure)) if any_failure.size else math.nan

    safety_ok = finite(row.get("vehicle_failsafe_ratio"), 0.0) == 0.0 and finite(row.get("failure_detector_ratio"), 0.0) == 0.0
    row["safety_valid"] = int(safety_ok)
    row["formal_eligible"] = int(gate_pass and bool(row["timing_valid"]) and safety_ok)
    rejection = []
    if not gate_pass:
        rejection.append(gate_reason)
    if not row["timing_valid"]:
        rejection.append("timing:" + row["timing_invalid_signals"])
    if not safety_ok:
        rejection.append("safety_flag")
    row["formal_rejection_reasons"] = ";".join(rejection)
    return row, timing, series


def parameter_rows(metrics: list[dict[str, Any]]) -> list[dict[str, Any]]:
    configured = {
        "T0": (120.0, 5.5, 16.0), "T1": (120.0, 5.5, 13.5),
        "T2": (140.0, 5.5, 16.5), "T3": (120.0, 4.5, 13.5),
        "T4": (140.0, 4.5, 13.5), "T5": (160.0, 5.0, 15.0),
    }
    rows = []
    for name, (b0, wc, wo) in configured.items():
        candidates = [row for row in metrics if row["group"] == "C1" and row["parameter_set"] == name]
        row = candidates[-1] if candidates else {}
        rows.append({
            "parameter_set": name, "b0_roll": b0, "b0_pitch": b0, "wc_roll_rad_s": wc,
            "wc_pitch_rad_s": wc, "wo_roll_rad_s": wo, "wo_pitch_rad_s": wo,
            "wo_wc_ratio": wo / wc, "d_roll": 0.003, "d_pitch": 0.003,
            "local_limit_roll": 0.30, "local_limit_pitch": 0.30, "td_enabled": 0,
            "rbf_enabled": 0, "screen_run_tag": row.get("run_tag", ""),
            "initial_gate_pass": row.get("initial_gate_pass", ""),
            "formal_eligible": row.get("formal_eligible", ""),
            "failure_reason": row.get("formal_rejection_reasons", "not_run"),
            "roll_rate_error_rms_rad_s": row.get("roll_rate_error_rad_s_rms", ""),
            "pitch_rate_error_rms_rad_s": row.get("pitch_rate_error_rad_s_rms", ""),
            "roll_torque_rms": row.get("roll_normalized_torque_rms", ""),
            "pitch_torque_rms": row.get("pitch_normalized_torque_rms", ""),
            "xy_error_rms_m": row.get("xy_position_error_m_rms", ""),
            "horizontal_speed_rms_m_s": row.get("horizontal_speed_rms_m_s", ""),
            "position_gate_sample_pass_ratio": row.get("position_gate_sample_pass_ratio", ""),
            "position_gate_longest_continuous_s": row.get("position_gate_longest_continuous_s", ""),
            "swing_rms_deg": row.get("swing_resultant_angle_deg_rms", ""),
            "screening_note": "D0 hover gate only; not a formal route pair",
        })
    return rows


def manifest_row(gate_path: Path, metric: dict[str, Any]) -> dict[str, Any]:
    gate = json.loads(gate_path.read_text(encoding="utf-8"))
    run_dir = Path(gate["run_dir"])
    ulog = run_dir / "position_offboard.ulg"
    params = gate.get("requested_parameters", {})
    return {
        "run_tag": gate["tag"], "stage": gate["stage"], "group": gate["group"],
        "condition": gate["condition"], "axis": "none_D0", "seed": "not_exposed_by_harness",
        "parameter_set": gate["parameter_set"], "ladrc_enabled": params.get("MC_LADRC_EN"),
        "as_enabled": params.get("MC_HANG_AS_EN"), "disturbance": "none",
        "disturbance_frame": "not_applicable", "disturbance_application_point": "not_applicable",
        "run_dir": str(run_dir), "ulog_path": str(ulog), "ulog_size_bytes": ulog.stat().st_size,
        "ulog_sha256": sha256(ulog), "window_start_s": metric["window_start_s"],
        "window_end_s": metric["window_end_s"], "initial_gate_pass": metric["initial_gate_pass"],
        "timing_valid": metric["timing_valid"], "safety_valid": metric["safety_valid"],
        "formal_eligible": metric["formal_eligible"],
        "invalid_reason": metric["formal_rejection_reasons"],
        "code_revision": "b9671fe69a+diagnostics_worktree",
    }


def escape(text: Any) -> str:
    return str(text).replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def screening_svg(rows: list[dict[str, Any]], path: Path) -> None:
    width, height = 1000, 560
    margin_left, margin_top, plot_w, plot_h = 85, 70, 840, 390
    values = [finite(row.get("xy_error_rms_m"), 0.0) for row in rows]
    maximum = max(values + [0.1]) * 1.15
    bars = []
    bar_w = plot_w / max(1, len(rows)) * 0.55
    for index, (row, value) in enumerate(zip(rows, values)):
        x = margin_left + (index + 0.5) * plot_w / len(rows) - bar_w / 2
        h = plot_h * value / maximum
        y = margin_top + plot_h - h
        color = "#238636" if int(finite(row.get("initial_gate_pass"), 0.0)) else "#d73a49"
        bars.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" fill="{color}"/>')
        bars.append(f'<text x="{x + bar_w/2:.1f}" y="{margin_top + plot_h + 27}" text-anchor="middle">{escape(row["parameter_set"])}</text>')
        bars.append(f'<text x="{x + bar_w/2:.1f}" y="{max(20, y - 7):.1f}" text-anchor="middle" font-size="12">{value:.3f}</text>')
    threshold_y = margin_top + plot_h - plot_h * 0.1 / maximum
    content = "\n".join(bars)
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" fill="#24292f">
<text x="{width/2}" y="32" text-anchor="middle" font-size="21">LADRC D0 screening: horizontal position error RMS</text>
<line x1="{margin_left}" y1="{margin_top}" x2="{margin_left}" y2="{margin_top+plot_h}" stroke="#24292f"/>
<line x1="{margin_left}" y1="{margin_top+plot_h}" x2="{margin_left+plot_w}" y2="{margin_top+plot_h}" stroke="#24292f"/>
<line x1="{margin_left}" y1="{threshold_y:.1f}" x2="{margin_left+plot_w}" y2="{threshold_y:.1f}" stroke="#0969da" stroke-dasharray="7,5"/>
<text x="{margin_left+plot_w-4}" y="{threshold_y-7:.1f}" text-anchor="end" font-size="12" fill="#0969da">instantaneous position-error bound 0.10 m (not RMS criterion)</text>
<text transform="translate(23 {margin_top+plot_h/2}) rotate(-90)" text-anchor="middle">RMS (m), diagnostic gate window</text>
{content}
<text x="{width/2}" y="535" text-anchor="middle" font-size="13">Red = strict initial-condition gate failed; all six candidates are retained.</text>
</g></svg>'''
    path.write_text(svg, encoding="utf-8")


def metric_bar_svg(rows: list[dict[str, Any]], field: str, title: str, ylabel: str, path: Path) -> None:
    width, height = 1000, 560
    left, top, plot_w, plot_h = 90, 70, 835, 390
    values = [finite(row.get(field), 0.0) for row in rows]
    maximum = max(values + [1.0e-9]) * 1.15
    elements = []
    bar_w = plot_w / max(1, len(rows)) * 0.55
    for index, (row, value) in enumerate(zip(rows, values)):
        x = left + (index + 0.5) * plot_w / len(rows) - bar_w / 2.0
        h = plot_h * value / maximum
        y = top + plot_h - h
        elements.append(f'<rect x="{x:.1f}" y="{y:.1f}" width="{bar_w:.1f}" height="{h:.1f}" fill="#d73a49"/>')
        elements.append(f'<text x="{x+bar_w/2:.1f}" y="{top+plot_h+27}" text-anchor="middle">{escape(row["parameter_set"])}</text>')
        elements.append(f'<text x="{x+bar_w/2:.1f}" y="{max(18, y-7):.1f}" text-anchor="middle" font-size="12">{value:.3g}</text>')
    svg = f'''<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">
<rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" fill="#24292f">
<text x="{width/2}" y="32" text-anchor="middle" font-size="21">{escape(title)}</text>
<line x1="{left}" y1="{top}" x2="{left}" y2="{top+plot_h}" stroke="#24292f"/>
<line x1="{left}" y1="{top+plot_h}" x2="{left+plot_w}" y2="{top+plot_h}" stroke="#24292f"/>
<text transform="translate(24 {top+plot_h/2}) rotate(-90)" text-anchor="middle">{escape(ylabel)}</text>
{''.join(elements)}
<text x="{width/2}" y="535" text-anchor="middle" font-size="13">All bars are gate-invalid screening diagnostics, not formal paired results.</text>
</g></svg>'''
    path.write_text(svg, encoding="utf-8")


def line_panels_svg(
    panels: list[tuple[str, str, list[tuple[str, np.ndarray, np.ndarray, str, str]]]],
    title: str,
    path: Path,
) -> None:
    width, panel_height = 1200, 270
    height = 70 + panel_height * len(panels) + 40
    left, right = 95, 30
    plot_w = width - left - right
    chunks = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<g font-family="sans-serif" fill="#24292f">',
        f'<text x="{width/2}" y="30" text-anchor="middle" font-size="21">{escape(title)}</text>',
    ]
    for panel_index, (panel_title, ylabel, lines) in enumerate(panels):
        y_top = 58 + panel_index * panel_height
        plot_h = panel_height - 62
        prepared = []
        all_x, all_y = [], []
        for label, t, value, color, dash in lines:
            t = np.asarray(t, dtype=float)
            value = np.asarray(value, dtype=float)
            good = np.isfinite(t) & np.isfinite(value)
            if np.count_nonzero(good) < 2:
                continue
            x = t[good] - t[good][0]
            y = value[good]
            if len(x) > 1200:
                indices = np.linspace(0, len(x) - 1, 1200, dtype=int)
                x, y = x[indices], y[indices]
            prepared.append((label, x, y, color, dash))
            all_x.extend(x.tolist())
            all_y.extend(y.tolist())
        if not prepared:
            continue
        xmin, xmax = 0.0, max(all_x)
        ymin, ymax = min(all_y), max(all_y)
        if ymin == ymax:
            ymin, ymax = ymin - 1.0, ymax + 1.0
        pad = 0.08 * (ymax - ymin)
        ymin, ymax = ymin - pad, ymax + pad
        chunks.extend([
            f'<text x="{left}" y="{y_top+14}" font-size="15">{escape(panel_title)}</text>',
            f'<line x1="{left}" y1="{y_top+28}" x2="{left}" y2="{y_top+28+plot_h}" stroke="#57606a"/>',
            f'<line x1="{left}" y1="{y_top+28+plot_h}" x2="{left+plot_w}" y2="{y_top+28+plot_h}" stroke="#57606a"/>',
            f'<text transform="translate(22 {y_top+28+plot_h/2}) rotate(-90)" text-anchor="middle" font-size="12">{escape(ylabel)}</text>',
            f'<text x="{left+plot_w}" y="{y_top+28+plot_h+24}" text-anchor="end" font-size="12">relative time (s), each trace starts at 0</text>',
            f'<text x="{left-8}" y="{y_top+35}" text-anchor="end" font-size="11">{ymax:.3g}</text>',
            f'<text x="{left-8}" y="{y_top+28+plot_h}" text-anchor="end" font-size="11">{ymin:.3g}</text>',
        ])
        legend_x = left + 12
        for line_index, (label, x, y, color, dash) in enumerate(prepared):
            sx = left + plot_w * (x - xmin) / max(1.0e-12, xmax - xmin)
            sy = y_top + 28 + plot_h * (ymax - y) / (ymax - ymin)
            points = " ".join(f"{xx:.1f},{yy:.1f}" for xx, yy in zip(sx, sy))
            dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
            chunks.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.3"{dash_attr}/>' )
            lx = legend_x + (line_index % 4) * 255
            ly = y_top + 13 + (line_index // 4) * 14
            chunks.append(f'<line x1="{lx}" y1="{ly}" x2="{lx+22}" y2="{ly}" stroke="{color}"{dash_attr}/><text x="{lx+27}" y="{ly+4}" font-size="10">{escape(label)}</text>')
    chunks.append("</g></svg>")
    path.write_text("\n".join(chunks), encoding="utf-8")


def placeholder_svg(title: str, lines: list[str], path: Path) -> None:
    content = "".join(f'<text x="600" y="{135 + 34*i}" text-anchor="middle" font-size="18">{escape(line)}</text>' for i, line in enumerate(lines))
    path.write_text(f'''<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="420" viewBox="0 0 1200 420">
<rect width="100%" height="100%" fill="white"/><g font-family="sans-serif" fill="#24292f">
<text x="600" y="54" text-anchor="middle" font-size="24">{escape(title)}</text>{content}</g></svg>''', encoding="utf-8")


def main() -> int:
    global ROOT, RUNS, REPORT, PLOTS
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path, default=ROOT)
    args = parser.parse_args()
    ROOT = args.root.resolve()
    RUNS, REPORT, PLOTS = ROOT / "runs", ROOT / "report", ROOT / "plots"
    REPORT.mkdir(parents=True, exist_ok=True)
    PLOTS.mkdir(parents=True, exist_ok=True)
    gates = sorted(RUNS.glob("**/gate_run.json"))
    metrics: list[dict[str, Any]] = []
    timings: list[dict[str, Any]] = []
    manifests: list[dict[str, Any]] = []
    series_by_tag: dict[str, dict[str, tuple[np.ndarray, np.ndarray]]] = {}
    for gate in gates:
        metric, timing, series = analyze_run(gate)
        metrics.append(metric)
        timings.extend(timing)
        manifests.append(manifest_row(gate, metric))
        series_by_tag[str(metric["run_tag"])] = series
        print(f"analyzed {metric['run_tag']}: eligible={metric['formal_eligible']} reason={metric['formal_rejection_reasons']}")
    params = parameter_rows(metrics)
    outputs = {
        "run_manifest.csv": (manifests, None),
        "metrics_raw.csv": (metrics, None),
        "timing_quality.csv": (timings, None),
        "parameter_sets.csv": (params, None),
    }
    for filename, (rows, fields) in outputs.items():
        write_csv(ROOT / filename, rows, fields)
        write_csv(REPORT / filename, rows, fields)
    pair_fields = [
        "pair_id", "condition", "axis", "pid_run_tag", "ladrc_run_tag", "metric",
        "pid_value", "ladrc_value", "improvement_pct", "pair_valid",
    ]
    write_csv(ROOT / "paired_comparison.csv", [], pair_fields)
    write_csv(REPORT / "paired_comparison.csv", [], pair_fields)
    screening_svg(params, PLOTS / "parameter_screening_xy_rms.svg")
    metric_bar_svg(params, "roll_rate_error_rms_rad_s", "LADRC D0 screening: roll rate-error RMS",
                   "rad/s", PLOTS / "parameter_screening_roll_rate_rms.svg")
    metric_bar_svg(params, "swing_rms_deg", "LADRC D0 screening: resultant swing-angle RMS",
                   "deg", PLOTS / "parameter_screening_swing_rms.svg")

    colors = {"C0": "#0969da", "C1": "#d1242f", "C2": "#1a7f37", "C3": "#8250df"}
    t0_rows = [row for row in metrics if row["parameter_set"] == "T0"]

    def selected_lines(key: str, suffix: str = "") -> list[tuple[str, np.ndarray, np.ndarray, str, str]]:
        result = []
        for row in t0_rows:
            series = series_by_tag[row["run_tag"]]
            if key not in series:
                continue
            t, value = series[key]
            result.append((f"{row['group']}{suffix}", t, value, colors[row["group"]], "5,4" if "setpoint" in key else ""))
        return result

    line_panels_svg([
        ("Roll body-rate tracking", "rad/s", selected_lines("roll_rate_actual", " actual") + selected_lines("roll_rate_setpoint", " setpoint")),
        ("Pitch body-rate tracking", "rad/s", selected_lines("pitch_rate_actual", " actual") + selected_lines("pitch_rate_setpoint", " setpoint")),
        ("Roll/Pitch attitude tracking error", "deg", selected_lines("roll_attitude_actual_deg", " roll actual")
         + selected_lines("roll_attitude_setpoint_deg", " roll setpoint")
         + selected_lines("pitch_attitude_actual_deg", " pitch actual")
         + selected_lines("pitch_attitude_setpoint_deg", " pitch setpoint")),
    ], "Stage 0 T0 tracking traces (different gate windows; diagnostic only)", PLOTS / "stage0_t0_attitude_rate.svg")

    line_panels_svg([
        ("Resultant suspended-load angle", "deg", selected_lines("swing_angle")),
        ("Horizontal position error", "m", selected_lines("xy_error")),
        ("Swing specific energy", "J/kg", selected_lines("swing_energy")),
    ], "Stage 0 T0 position and suspended-load response", PLOTS / "stage0_t0_swing_position_energy.svg")

    observer_panels = []
    for group in ("C1", "C3"):
        row = next((item for item in t0_rows if item["group"] == group), None)
        if not row:
            continue
        series = series_by_tag[row["run_tag"]]
        observer_panels.append((f"{group} roll normalized control", "normalized torque",
                                [(f"{group} torque", *series["roll_torque"], colors[group], ""),
                                 (f"{group} raw LADRC", *series["ladrc_raw_roll"], "#bf8700", "5,4")]))
        observer_panels.append((f"{group} roll LESO total disturbance (D0: no injected torque)", "rad/s²",
                                [(f"{group} z2", *series["ladrc_z2_roll_rad_s2"], colors[group], "")]))
    line_panels_svg(observer_panels, "LADRC T0 observer/control diagnostics", PLOTS / "stage0_ladrc_t0_observer_control.svg")
    placeholder_svg("Stage 2 / Stage 3 plots intentionally absent", [
        "All six LADRC candidates failed the Stage 1 initial-condition gate.",
        "No D-T, D-F, or D-FT disturbance was injected.",
        "No valid C2/C3 paired improvement statistic exists.",
    ], PLOTS / "stage2_stage3_not_run.svg")
    summary = {
        "run_count": len(metrics), "formal_eligible_count": sum(int(row["formal_eligible"]) for row in metrics),
        "timing_valid_count": sum(int(row["timing_valid"]) for row in metrics),
        "ladrc_parameter_count": len(params),
        "stage1_formal_pair_count": 0,
        "note": "Formal paired comparison is empty unless both members passed the strict initial gate.",
    }
    for path in (ROOT / "analysis_summary.json", REPORT / "analysis_summary.json"):
        path.write_text(json.dumps(summary, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(summary, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
