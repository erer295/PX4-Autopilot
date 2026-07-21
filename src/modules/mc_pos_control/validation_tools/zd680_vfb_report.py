#!/usr/bin/env python3
"""Analyze the four ZD680 EKF-ESO velocity-feedback task-book runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import numpy as np
from pyulog import ULog


REPO_ROOT = Path(__file__).resolve().parents[4]
GENERIC_REPORT = REPO_ROOT / "Tools" / "simulation" / "gz" / "tools" / "zd680_validation_report.py"
GENERIC = runpy.run_path(str(GENERIC_REPORT), run_name="zd680_vfb_generic_report_api")
MODES = ("VFB_L06_W00", "VFB_L06_W05", "VFB_L08_W00", "VFB_L08_W05")
SUPPORTED_MODES = (
    "VFB_L06_W00", "VFB_L06_W05", "VFB_L08_W00", "VFB_L08_W25", "VFB_L08_W05",
    "STD_L06_PID_AS", "STD_L06_PID_AS_PAS", "STD_L06_FULL_W05", "STD_L06_FULL_W10_OBS_DECOUPLE",
)
MOVE_NAMES = ("north", "east", "south", "west")
Z_OBSERVATION_ONLY_CHECKS = {
    "truth_z_peak_le_0_5m",
    "ekf_truth_z_peak_le_0_3m",
}
POWER_FIELDS = {
    "position": 36,
    "tracking": 37,
    "observer_error": 38,
    "velocity_total": 39,
    "nominal": 40,
    "z3": 41,
    "as": 42,
    "pas": 43,
    "final": 44,
}
ACCEL_FIELDS = {
    "position": (14, 15),
    "tracking": (16, 17),
    "observer_error": (18, 19),
    "velocity_total": (20, 21),
    "nominal": (22, 23),
    "z3": (24, 25),
    "as": (26, 27),
    "pas": (28, 29),
    "final": (30, 31),
}


def finite(value, fallback=math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def rms(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.sqrt(np.mean(values * values))) if values.size else math.nan


def percentile(values: np.ndarray, p: float) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.percentile(values, p)) if values.size else math.nan


def peak(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    values = values[np.isfinite(values)]
    return float(np.max(np.abs(values))) if values.size else math.nan


def dataset_optional(ulog: ULog, name: str):
    try:
        return ulog.get_dataset(name).data
    except (KeyError, IndexError):
        return None


def quaternion_to_euler_deg(data: Dict[str, np.ndarray], prefix: str = "q") -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Convert PX4 scalar-first quaternions to roll, pitch and yaw in degrees."""
    w = np.asarray(data[f"{prefix}[0]"], dtype=float)
    x = np.asarray(data[f"{prefix}[1]"], dtype=float)
    y = np.asarray(data[f"{prefix}[2]"], dtype=float)
    z = np.asarray(data[f"{prefix}[3]"], dtype=float)
    norm = np.sqrt(w * w + x * x + y * y + z * z)
    norm = np.where(norm > 1.0e-9, norm, np.nan)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def quaternion_tracking_error_deg(attitude: Dict[str, np.ndarray], setpoint: Dict[str, np.ndarray],
                                  start: float, end: float) -> np.ndarray:
    """Return shortest 3-D quaternion angle between attitude and setpoint."""
    actual_t = np.asarray(attitude["timestamp"], dtype=float) * 1.0e-6
    desired_t = np.asarray(setpoint["timestamp"], dtype=float) * 1.0e-6
    mask = (desired_t >= start) & (desired_t <= end)
    desired_t = desired_t[mask]
    if desired_t.size == 0 or actual_t.size < 2:
        return np.asarray([], dtype=float)
    actual = np.column_stack([np.interp(desired_t, actual_t, np.asarray(attitude[f"q[{i}]"], dtype=float))
                              for i in range(4)])
    desired = np.column_stack([np.asarray(setpoint[f"q_d[{i}]"], dtype=float)[mask] for i in range(4)])
    actual_norm = np.linalg.norm(actual, axis=1)
    desired_norm = np.linalg.norm(desired, axis=1)
    valid = np.isfinite(actual).all(axis=1) & np.isfinite(desired).all(axis=1) \
        & (actual_norm > 1.0e-9) & (desired_norm > 1.0e-9)
    if not np.any(valid):
        return np.asarray([], dtype=float)
    actual = actual[valid] / actual_norm[valid, None]
    desired = desired[valid] / desired_norm[valid, None]
    dot = np.clip(np.abs(np.sum(actual * desired, axis=1)), 0.0, 1.0)
    return np.degrees(2.0 * np.arccos(dot))


def contiguous_slices(t: np.ndarray) -> List[slice]:
    """Split a monotonic sample stream at gaps (including omitted hold phases)."""
    if t.size == 0:
        return []
    differences = np.diff(t)
    positive = differences[differences > 1.0e-6]
    nominal_dt = float(np.median(positive)) if positive.size else math.nan
    gap_limit = max(0.05, 5.0 * nominal_dt) if math.isfinite(nominal_dt) else 0.05
    breaks = np.flatnonzero(differences > gap_limit) + 1
    boundaries = np.r_[0, breaks, t.size]
    return [slice(int(start), int(end)) for start, end in zip(boundaries[:-1], boundaries[1:])]


def jerk_magnitude(t: np.ndarray, north: np.ndarray, east: np.ndarray) -> np.ndarray:
    """Return horizontal jerk after dropping invalid/duplicate timestamps."""
    t = np.asarray(t, dtype=float)
    north = np.asarray(north, dtype=float)
    east = np.asarray(east, dtype=float)
    finite_mask = np.isfinite(t) & np.isfinite(north) & np.isfinite(east)
    t, north, east = t[finite_mask], north[finite_mask], east[finite_mask]
    if t.size < 3:
        return np.asarray([], dtype=float)
    strictly_increasing = np.r_[True, np.diff(t) > 1.0e-6]
    t, north, east = t[strictly_increasing], north[strictly_increasing], east[strictly_increasing]
    if t.size < 3:
        return np.asarray([], dtype=float)
    segments = []
    for selection in contiguous_slices(t):
        if t[selection].size >= 3:
            segments.append(np.hypot(np.gradient(north[selection], t[selection]),
                                     np.gradient(east[selection], t[selection])))
    return np.concatenate(segments) if segments else np.asarray([], dtype=float)


def integrate(t: np.ndarray, values: np.ndarray) -> float:
    mask = np.isfinite(t) & np.isfinite(values)
    t, values = np.asarray(t, dtype=float)[mask], np.asarray(values, dtype=float)[mask]
    if t.size < 2:
        return math.nan
    strictly_increasing = np.r_[True, np.diff(t) > 1.0e-6]
    t, values = t[strictly_increasing], values[strictly_increasing]
    integrals = [float(np.trapezoid(values[selection], t[selection]))
                 for selection in contiguous_slices(t) if t[selection].size >= 2]
    return float(sum(integrals)) if integrals else math.nan


def band_energy(t: np.ndarray, north: np.ndarray, east: np.ndarray, low=0.55, high=0.75) -> float:
    mask = np.isfinite(t) & np.isfinite(north) & np.isfinite(east)
    t, north, east = t[mask], north[mask], east[mask]
    strictly_increasing = np.r_[True, np.diff(t) > 1.0e-6]
    t, north, east = t[strictly_increasing], north[strictly_increasing], east[strictly_increasing]
    if t.size < 32:
        return math.nan
    energies = []
    for selection in contiguous_slices(t):
        value = band_energy_contiguous(t[selection], north[selection], east[selection], low, high)
        if math.isfinite(value):
            energies.append(value)
    return float(sum(energies)) if energies else math.nan


def band_energy_contiguous(t: np.ndarray, north: np.ndarray, east: np.ndarray,
                           low: float, high: float) -> float:
    if t.size < 32 or t[-1] - t[0] < 5.0:
        return math.nan
    dt = float(np.median(np.diff(t)))
    if not math.isfinite(dt) or dt <= 0.0:
        return math.nan
    grid = np.arange(t[0], t[-1], dt)
    if grid.size < 32:
        return math.nan
    total = 0.0
    window = np.hanning(grid.size)
    denominator = (1.0 / dt) * np.sum(window * window)
    frequencies = np.fft.rfftfreq(grid.size, dt)
    band = (frequencies >= low) & (frequencies <= high)
    for values in (north, east):
        signal = np.interp(grid, t, values)
        signal = signal - np.mean(signal)
        spectrum = np.abs(np.fft.rfft(signal * window)) ** 2 / denominator
        if spectrum.size > 2:
            spectrum[1:-1] *= 2.0
        total += float(np.trapezoid(spectrum[band], frequencies[band])) if band.sum() >= 2 else 0.0
    return total


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


def expected_weight(mode: str) -> float:
    if mode in ("STD_L06_PID_AS", "STD_L06_PID_AS_PAS"):
        return 0.0
    if mode == "STD_L06_FULL_W05":
        return 0.5
    if mode == "STD_L06_FULL_W10_OBS_DECOUPLE":
        return 1.0
    if mode.endswith("W00"):
        return 0.0
    if mode.endswith("W25"):
        return 0.25
    if mode.endswith("W05"):
        return 0.5
    return math.nan


def latest_runs(root: Path, modes: Sequence[str] = MODES) -> Dict[str, Path]:
    grouped: Dict[str, List[Path]] = {}
    for metadata_path in root.glob("runs/*/*/*/metadata.json"):
        try:
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            mode = str(metadata.get("experiment", {}).get("validation_suite", ""))
        except (OSError, json.JSONDecodeError):
            continue
        if mode in modes:
            grouped.setdefault(mode, []).append(metadata_path.parent)
    return {mode: max(paths, key=lambda path: path.stat().st_mtime) for mode, paths in grouped.items()}


def action_windows(events: List[Dict[str, object]]) -> List[Tuple[str, str, float, float]]:
    actions = [event for event in events if str(event.get("name", "")).startswith("action_")]
    route_start = min(finite(event["start_boot_s"]) for event in actions)
    route_end = max(finite(event["end_boot_s"]) for event in actions)
    moves = [event for event in actions if any(name in str(event.get("name", "")) for name in MOVE_NAMES)]
    holds = [event for event in actions if event not in moves]
    windows = [("route", "all", route_start, route_end)]
    for event in moves:
        name = str(event["name"])
        direction = next(item for item in MOVE_NAMES if item in name)
        start, end = finite(event["start_boot_s"]), finite(event["end_boot_s"])
        windows.extend([
            (f"{direction}_move", direction, start, end),
            (f"{direction}_accel", direction, start, min(start + 1.0, end)),
            (f"{direction}_cruise", direction, min(start + 1.0, end), max(start + 1.0, end - 1.0)),
            (f"{direction}_decel", direction, max(start, end - 1.0), end),
        ])
    if moves:
        windows.append(("movement", "all", min(finite(e["start_boot_s"]) for e in moves),
                        max(finite(e["end_boot_s"]) for e in moves)))
        windows.extend([
            ("movement_accel", "all", min(finite(e["start_boot_s"]) for e in moves),
             max(finite(e["end_boot_s"]) for e in moves)),
            ("movement_cruise", "all", min(finite(e["start_boot_s"]) for e in moves),
             max(finite(e["end_boot_s"]) for e in moves)),
            ("movement_decel", "all", min(finite(e["start_boot_s"]) for e in moves),
             max(finite(e["end_boot_s"]) for e in moves)),
        ])
    if holds:
        normal_holds = holds[:-1] if len(holds) > 1 else holds
        windows.append(("normal_hover", "all", min(finite(e["start_boot_s"]) for e in normal_holds),
                        max(finite(e["end_boot_s"]) for e in normal_holds)))
    windows.append(("final_10s", "all", route_end - 10.0, route_end))
    return windows


def combined_window_mask(t: np.ndarray, name: str, start: float, end: float,
                         events: List[Dict[str, object]]) -> np.ndarray:
    combined_names = ("movement", "movement_accel", "movement_cruise", "movement_decel", "normal_hover")
    if name not in combined_names:
        return (t >= start) & (t <= end)
    actions = [event for event in events if str(event.get("name", "")).startswith("action_")]
    if name.startswith("movement"):
        selected = [e for e in actions if any(item in str(e.get("name", "")) for item in MOVE_NAMES)]
    else:
        holds = [e for e in actions if not any(item in str(e.get("name", "")) for item in MOVE_NAMES)]
        selected = holds[:-1] if len(holds) > 1 else holds
    mask = np.zeros(t.shape, dtype=bool)
    for event in selected:
        event_start, event_end = finite(event["start_boot_s"]), finite(event["end_boot_s"])
        if name == "movement_accel":
            event_end = min(event_start + 1.0, event_end)
        elif name == "movement_cruise":
            event_start, event_end = min(event_start + 1.0, event_end), max(event_start + 1.0, event_end - 1.0)
        elif name == "movement_decel":
            event_start = max(event_start, event_end - 1.0)
        mask |= (t >= event_start) & (t <= event_end)
    return mask


def reference_sample(event: Dict[str, object], sample_s: float) -> Tuple[float, float, float, float, float]:
    """Handle both S-curve move records and direct-position hold records."""
    params = event.get("params", {})
    if isinstance(params, dict) and "start" not in params and "position" not in params:
        x, y = finite(params.get("x_m")), finite(params.get("y_m"))
        if math.isfinite(x) and math.isfinite(y):
            return x, y, 0.0, 0.0, 0.0
    return GENERIC["ref_sample"](event, sample_s)


def reference_xy_errors(local: Dict[str, np.ndarray], events: List[Dict[str, object]]) -> Tuple[np.ndarray, np.ndarray]:
    t = np.asarray(local["timestamp"], dtype=float) * 1.0e-6
    x, y = np.asarray(local["x"], dtype=float), np.asarray(local["y"], dtype=float)
    errors = np.full(t.shape, np.nan)
    for event in events:
        if not str(event.get("name", "")).startswith("action_"):
            continue
        mask = (t >= finite(event["start_boot_s"])) & (t <= finite(event["end_boot_s"]))
        for index in np.flatnonzero(mask):
            rx, ry, _, _, _ = reference_sample(event, float(t[index]))
            errors[index] = math.hypot(float(x[index]) - rx, float(y[index]) - ry)
    return t, errors


def analyze_run(mode: str, run_dir: Path):
    base = dict(GENERIC["analyze_run"](run_dir))
    metadata = json.loads((run_dir / "metadata.json").read_text(encoding="utf-8"))
    validity = json.loads((run_dir / "validity.json").read_text(encoding="utf-8"))
    events = GENERIC["read_events"](run_dir / "events.csv")
    windows = action_windows(events)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), [
        "vehicle_local_position", "debug_array", "estimator_status", "sensor_combined", "cpuload",
        "vehicle_attitude", "vehicle_angular_velocity", "vehicle_attitude_setpoint", "vehicle_status",
        "control_allocator_status", "actuator_motors",
    ])
    vfb = GENERIC["find_debug"](ulog, 686)
    expected_vfb_weight = expected_weight(mode)
    checks: Dict[str, Dict[str, object]] = {}
    warnings: List[str] = []

    def check(name: str, passed: bool, detail) -> None:
        checks[name] = {"passed": bool(passed), "detail": detail}

    strict_rejections = [str(item) for item in validity.get("rejection_reasons", [])]
    z_observations = [item for item in strict_rejections if item in Z_OBSERVATION_ONLY_CHECKS]
    non_z_rejections = [item for item in strict_rejections if item not in Z_OBSERVATION_ONLY_CHECKS]
    analysis_base_valid = bool(validity.get("completed", False)) and not non_z_rejections
    check("base_validity_excluding_z", analysis_base_valid, non_z_rejections)
    if z_observations:
        warnings.append("z_observation_only:" + ",".join(z_observations))
    check("hangvfb_debug_present", vfb is not None, "debug_array ID 686")
    window_rows: List[Dict[str, object]] = []
    if vfb is None:
        return base, window_rows, {"valid": False, "checks": checks, "warnings": warnings}

    t = np.asarray(vfb["timestamp"], dtype=float) * 1.0e-6
    route = next(window for window in windows if window[0] == "route")
    route_mask = (t >= route[2]) & (t <= route[3])
    requested = np.asarray(vfb["data[32]"], dtype=float)[route_mask]
    effective = np.asarray(vfb["data[33]"], dtype=float)[route_mask]
    fallback = np.asarray(vfb["data[34]"], dtype=float)[route_mask]
    velocity_valid = np.asarray(vfb["data[35]"], dtype=float)[route_mask]
    requested_median = float(np.nanmedian(requested)) if requested.size else math.nan
    effective_median = float(np.nanmedian(effective)) if effective.size else math.nan
    fallback_ratio = float(np.mean(fallback > 0.5)) if fallback.size else math.nan
    velocity_valid_ratio = float(np.mean(velocity_valid > 0.5)) if velocity_valid.size else math.nan
    identity_nominal = np.hypot(np.asarray(vfb["data[47]"], dtype=float)[route_mask],
                                np.asarray(vfb["data[48]"], dtype=float)[route_mask])
    identity_velocity = np.hypot(np.asarray(vfb["data[49]"], dtype=float)[route_mask],
                                 np.asarray(vfb["data[50]"], dtype=float)[route_mask])
    controls = np.column_stack([
        np.asarray(vfb[f"data[{index}]"], dtype=float)[route_mask]
        for pair in ACCEL_FIELDS.values() for index in pair
    ])
    check("requested_weight_matches_label", math.isfinite(requested_median)
          and abs(requested_median - expected_vfb_weight) <= 0.002,
          {"expected": expected_vfb_weight, "observed": requested_median})
    check("effective_weight_matches_request", math.isfinite(effective_median)
          and abs(effective_median - expected_vfb_weight) <= 0.002,
          {"expected": expected_vfb_weight, "observed": effective_median})
    check("control_diagnostics_finite", controls.size > 0 and np.all(np.isfinite(controls)),
          {"samples": int(controls.shape[0]), "nonfinite": int(np.size(controls) - np.isfinite(controls).sum())})
    check("velocity_valid_not_persistent", math.isfinite(velocity_valid_ratio) and velocity_valid_ratio >= 0.99,
          velocity_valid_ratio)
    check("nominal_identity", identity_nominal.size > 0 and float(np.nanmax(identity_nominal)) <= 1.0e-4,
          float(np.nanmax(identity_nominal)) if identity_nominal.size else math.nan)
    check("velocity_identity", identity_velocity.size > 0 and float(np.nanmax(identity_velocity)) <= 1.0e-4,
          float(np.nanmax(identity_velocity)) if identity_velocity.size else math.nan)
    if math.isfinite(fallback_ratio) and fallback_ratio > 0.01:
        warnings.append(f"fallback_ratio_gt_1pct:{fallback_ratio:.6f}")

    local = ulog.get_dataset("vehicle_local_position").data
    local_t, xy_error = reference_xy_errors(local, events)
    route_mask_local = (local_t >= route[2]) & (local_t <= route[3])
    movement_mask_local = combined_window_mask(local_t, "movement", route[2], route[3], events)
    hover_mask_local = combined_window_mask(local_t, "normal_hover", route[2], route[3], events)
    final_mask_local = (local_t >= route[3] - 10.0) & (local_t <= route[3])
    final_local_indices = np.flatnonzero(route_mask_local)
    endpoint_error = math.nan
    if final_local_indices.size:
        endpoint_index = int(final_local_indices[-1])
        rx, ry, _, _, _ = reference_sample(
            next(event for event in reversed(events) if str(event.get("name", "")).startswith("action_")),
            float(local_t[endpoint_index]))
        endpoint_error = math.hypot(float(local["x"][endpoint_index]) - rx,
                                    float(local["y"][endpoint_index]) - ry)

    estimator = ulog.get_dataset("estimator_status").data
    estimator_t = np.asarray(estimator["timestamp"], dtype=float) * 1.0e-6
    estimator_mask = (estimator_t >= route[2]) & (estimator_t <= route[3])
    time_slip = np.asarray(estimator["time_slip"], dtype=float)[estimator_mask]
    timeout_flags = np.asarray(estimator["timeout_flags"], dtype=np.uint32)[estimator_mask]
    filter_fault_flags = np.asarray(estimator["filter_fault_flags"], dtype=np.uint32)[estimator_mask]
    vertical_accuracy = np.asarray(estimator["pos_vert_accuracy"], dtype=float)[estimator_mask]

    sensor = ulog.get_dataset("sensor_combined").data
    sensor_t = np.asarray(sensor["timestamp"], dtype=float) * 1.0e-6
    sensor_route_t = sensor_t[(sensor_t >= route[2]) & (sensor_t <= route[3])]
    sensor_gaps = np.diff(sensor_route_t)

    cpu = ulog.get_dataset("cpuload").data
    cpu_t = np.asarray(cpu["timestamp"], dtype=float) * 1.0e-6
    cpu_load = np.asarray(cpu["load"], dtype=float)[(cpu_t >= route[2]) & (cpu_t <= route[3])]

    attitude = dataset_optional(ulog, "vehicle_attitude")
    angular_velocity = dataset_optional(ulog, "vehicle_angular_velocity")
    attitude_setpoint = dataset_optional(ulog, "vehicle_attitude_setpoint")
    vehicle_status = dataset_optional(ulog, "vehicle_status")
    allocator = dataset_optional(ulog, "control_allocator_status")
    motors = dataset_optional(ulog, "actuator_motors")
    attitude_metrics: Dict[str, float] = {}
    if attitude is not None:
        attitude_t = np.asarray(attitude["timestamp"], dtype=float) * 1.0e-6
        attitude_mask = (attitude_t >= route[2]) & (attitude_t <= route[3])
        roll_deg, pitch_deg, _ = quaternion_to_euler_deg(attitude)
        tilt_deg = np.degrees(np.arccos(np.clip(
            1.0 - 2.0 * (np.square(np.asarray(attitude["q[1]"], dtype=float))
                         + np.square(np.asarray(attitude["q[2]"], dtype=float))), -1.0, 1.0)))
        attitude_metrics.update({
            "attitude_sample_count": int(attitude_mask.sum()),
            "roll_rms_deg": rms(roll_deg[attitude_mask]),
            "roll_peak_abs_deg": peak(roll_deg[attitude_mask]),
            "pitch_rms_deg": rms(pitch_deg[attitude_mask]),
            "pitch_peak_abs_deg": peak(pitch_deg[attitude_mask]),
            "tilt_rms_deg": rms(tilt_deg[attitude_mask]),
            "tilt_p95_deg": percentile(tilt_deg[attitude_mask], 95.0),
            "tilt_peak_deg": peak(tilt_deg[attitude_mask]),
            "attitude_quat_reset_count": int(np.ptp(np.asarray(attitude["quat_reset_counter"], dtype=np.int64)[attitude_mask]))
                if attitude_mask.any() else 0,
        })
        if attitude_setpoint is not None:
            tracking_error = quaternion_tracking_error_deg(attitude, attitude_setpoint, route[2], route[3])
            attitude_metrics.update({
                "attitude_tracking_error_rms_deg": rms(tracking_error),
                "attitude_tracking_error_p95_deg": percentile(tracking_error, 95.0),
                "attitude_tracking_error_peak_deg": peak(tracking_error),
            })
    if angular_velocity is not None:
        angular_t = np.asarray(angular_velocity["timestamp"], dtype=float) * 1.0e-6
        angular_mask = (angular_t >= route[2]) & (angular_t <= route[3])
        roll_rate = np.asarray(angular_velocity["xyz[0]"], dtype=float)[angular_mask]
        pitch_rate = np.asarray(angular_velocity["xyz[1]"], dtype=float)[angular_mask]
        yaw_rate = np.asarray(angular_velocity["xyz[2]"], dtype=float)[angular_mask]
        horizontal_rate = np.hypot(roll_rate, pitch_rate)
        attitude_metrics.update({
            "horizontal_body_rate_rms_rad_s": rms(horizontal_rate),
            "horizontal_body_rate_p95_rad_s": percentile(horizontal_rate, 95.0),
            "horizontal_body_rate_peak_rad_s": peak(horizontal_rate),
            "yaw_rate_rms_rad_s": rms(yaw_rate),
            "yaw_rate_peak_abs_rad_s": peak(yaw_rate),
        })
    if vehicle_status is not None:
        status_t = np.asarray(vehicle_status["timestamp"], dtype=float) * 1.0e-6
        status_mask = (status_t >= route[2]) & (status_t <= route[3])
        failure_detector = np.asarray(vehicle_status["failure_detector_status"], dtype=np.uint32)[status_mask]
        status_failsafe = np.asarray(vehicle_status["failsafe"], dtype=bool)[status_mask]
        attitude_metrics.update({
            "failure_detector_nonzero_ratio": float(np.mean(failure_detector != 0)) if failure_detector.size else math.nan,
            "vehicle_status_failsafe_ratio": float(np.mean(status_failsafe)) if status_failsafe.size else math.nan,
        })
    if allocator is not None:
        allocator_t = np.asarray(allocator["timestamp"], dtype=float) * 1.0e-6
        allocator_mask = (allocator_t >= route[2]) & (allocator_t <= route[3])
        torque_achieved = np.asarray(allocator["torque_setpoint_achieved"], dtype=bool)[allocator_mask]
        thrust_achieved = np.asarray(allocator["thrust_setpoint_achieved"], dtype=bool)[allocator_mask]
        unallocated_torque = np.column_stack([
            np.asarray(allocator[f"unallocated_torque[{i}]"], dtype=float)[allocator_mask] for i in range(3)
        ])
        attitude_metrics.update({
            "allocator_torque_not_achieved_ratio": float(np.mean(~torque_achieved)) if torque_achieved.size else math.nan,
            "allocator_thrust_not_achieved_ratio": float(np.mean(~thrust_achieved)) if thrust_achieved.size else math.nan,
            "unallocated_torque_peak": peak(np.linalg.norm(unallocated_torque, axis=1)),
        })
    if motors is not None:
        motor_t = np.asarray(motors["timestamp"], dtype=float) * 1.0e-6
        motor_mask = (motor_t >= route[2]) & (motor_t <= route[3])
        motor_values = np.column_stack([
            np.asarray(motors[f"control[{i}]"], dtype=float)[motor_mask] for i in range(4)
        ])
        finite_motor = motor_values[np.isfinite(motor_values)]
        attitude_metrics.update({
            "motor_output_min": float(np.min(finite_motor)) if finite_motor.size else math.nan,
            "motor_output_max": float(np.max(finite_motor)) if finite_motor.size else math.nan,
            "motor_high_saturation_sample_ratio": float(np.mean(np.any(motor_values >= 0.99, axis=1)))
                if motor_values.size else math.nan,
        })

    ekf_minus_z2 = np.hypot(np.asarray(vfb["data[10]"], dtype=float)[route_mask],
                            np.asarray(vfb["data[11]"], dtype=float)[route_mask])
    feedback_minus_z2 = np.hypot(
        np.asarray(vfb["data[6]"], dtype=float)[route_mask] - np.asarray(vfb["data[4]"], dtype=float)[route_mask],
        np.asarray(vfb["data[7]"], dtype=float)[route_mask] - np.asarray(vfb["data[5]"], dtype=float)[route_mask])
    feedback_minus_ekf = np.hypot(
        np.asarray(vfb["data[6]"], dtype=float)[route_mask] - np.asarray(vfb["data[2]"], dtype=float)[route_mask],
        np.asarray(vfb["data[7]"], dtype=float)[route_mask] - np.asarray(vfb["data[3]"], dtype=float)[route_mask])
    observer_acceleration = np.hypot(
        np.asarray(vfb["data[18]"], dtype=float)[route_mask],
        np.asarray(vfb["data[19]"], dtype=float)[route_mask])
    z3_acceleration = np.hypot(
        np.asarray(vfb["data[24]"], dtype=float)[route_mask],
        np.asarray(vfb["data[25]"], dtype=float)[route_mask])
    if mode == "STD_L06_FULL_W10_OBS_DECOUPLE":
        feedback_ekf_rms = rms(feedback_minus_ekf)
        observer_acceleration_peak = peak(observer_acceleration)
        z3_acceleration_peak = peak(z3_acceleration)
        z3_acceleration_std = float(np.nanstd(z3_acceleration)) if z3_acceleration.size else math.nan
        check("w10_feedback_equals_ekf", math.isfinite(feedback_ekf_rms) and feedback_ekf_rms <= 1.0e-5,
              feedback_ekf_rms)
        check("w10_observer_error_direct_control_zero",
              math.isfinite(observer_acceleration_peak) and observer_acceleration_peak <= 1.0e-5,
              observer_acceleration_peak)
        check("w10_z3_still_updates", math.isfinite(z3_acceleration_peak) and math.isfinite(z3_acceleration_std)
              and z3_acceleration_peak > 1.0e-5 and z3_acceleration_std > 1.0e-6,
              {"peak": z3_acceleration_peak, "std": z3_acceleration_std})
    base.update({
        "strict_base_valid": bool(validity.get("valid", False)),
        "analysis_base_valid": analysis_base_valid,
        "z_observation_only": bool(z_observations),
        "z_observation_reasons": ";".join(z_observations),
        "non_z_rejection_reasons": ";".join(non_z_rejections),
        "expected_vfb_weight": expected_vfb_weight,
        "requested_weight_median": requested_median,
        "requested_weight_mean": float(np.nanmean(requested)) if requested.size else math.nan,
        "requested_weight_min": float(np.nanmin(requested)) if requested.size else math.nan,
        "requested_weight_max": float(np.nanmax(requested)) if requested.size else math.nan,
        "effective_weight_median": effective_median,
        "effective_weight_mean": float(np.nanmean(effective)) if effective.size else math.nan,
        "effective_weight_min": float(np.nanmin(effective)) if effective.size else math.nan,
        "effective_weight_max": float(np.nanmax(effective)) if effective.size else math.nan,
        "fallback_ratio": fallback_ratio,
        "velocity_valid_ratio": velocity_valid_ratio,
        "ekf_minus_z2_rms_m_s": rms(ekf_minus_z2),
        "ekf_minus_z2_p95_m_s": percentile(ekf_minus_z2, 95.0),
        "ekf_minus_z2_peak_m_s": peak(ekf_minus_z2),
        "feedback_minus_z2_rms_m_s": rms(feedback_minus_z2),
        "feedback_minus_ekf_rms_m_s": rms(feedback_minus_ekf),
        "observer_error_acceleration_peak_m_s2": peak(observer_acceleration),
        "z3_acceleration_peak_m_s2": peak(z3_acceleration),
        "z3_acceleration_std_m_s2": float(np.nanstd(z3_acceleration)) if z3_acceleration.size else math.nan,
        "nominal_identity_max_abs_m_s2": float(np.nanmax(identity_nominal)) if identity_nominal.size else math.nan,
        "velocity_identity_max_abs_m_s2": float(np.nanmax(identity_velocity)) if identity_velocity.size else math.nan,
        "xy_error_rms_m": rms(xy_error[route_mask_local]),
        "xy_error_peak_m": peak(xy_error[route_mask_local]),
        "moving_xy_error_rms_m": rms(xy_error[movement_mask_local]),
        "hover_xy_error_rms_m": rms(xy_error[hover_mask_local]),
        "final_10s_xy_error_rms_m": rms(xy_error[final_mask_local]),
        "endpoint_xy_error_m": endpoint_error,
        "ekf_time_slip_delta_s": (float(time_slip[-1] - time_slip[0]) if time_slip.size >= 2 else math.nan),
        "ekf_time_slip_span_s": (float(np.nanmax(time_slip) - np.nanmin(time_slip)) if time_slip.size else math.nan),
        "ekf_timeout_nonzero_ratio": (float(np.mean(timeout_flags != 0)) if timeout_flags.size else math.nan),
        "ekf_filter_fault_nonzero_ratio": (float(np.mean(filter_fault_flags != 0)) if filter_fault_flags.size else math.nan),
        "ekf_vertical_accuracy_peak_m": peak(vertical_accuracy),
        "sensor_combined_gap_peak_ms": (1000.0 * float(np.nanmax(sensor_gaps)) if sensor_gaps.size else math.nan),
        "cpu_load_mean_pct": (100.0 * float(np.nanmean(cpu_load)) if cpu_load.size else math.nan),
        "cpu_load_peak_pct": (100.0 * float(np.nanmax(cpu_load)) if cpu_load.size else math.nan),
        **attitude_metrics,
    })
    attitude_limits = {
        "tilt_peak_deg": 20.0,
        "horizontal_body_rate_peak_rad_s": 2.0,
        "attitude_tracking_error_peak_deg": 10.0,
        "failure_detector_nonzero_ratio": 0.0,
        "vehicle_status_failsafe_ratio": 0.0,
        "attitude_quat_reset_count": 0.0,
        "allocator_torque_not_achieved_ratio": 0.01,
        "motor_high_saturation_sample_ratio": 0.05,
    }
    attitude_rejections = [name for name, limit in attitude_limits.items()
                           if not math.isfinite(finite(base.get(name))) or finite(base.get(name)) > limit]
    base["attitude_control_valid"] = not attitude_rejections
    base["attitude_control_rejection_reasons"] = ";".join(attitude_rejections)
    check("attitude_control_stable", not attitude_rejections,
          {name: {"observed": finite(base.get(name)), "limit": limit}
           for name, limit in attitude_limits.items()})

    for window_name, direction, start, end in windows:
        mask = combined_window_mask(t, window_name, start, end, events)
        if mask.sum() < 2:
            continue
        row: Dict[str, object] = {
            "mode": mode,
            "rope_length_m": base.get("rope_length_m"),
            "window": window_name,
            "direction": direction,
            "start_boot_s": start,
            "end_boot_s": end,
            "sample_count": int(mask.sum()),
        }
        window_t = t[mask]
        for component, (north_index, east_index) in ACCEL_FIELDS.items():
            north = np.asarray(vfb[f"data[{north_index}]"], dtype=float)[mask]
            east = np.asarray(vfb[f"data[{east_index}]"], dtype=float)[mask]
            magnitude = np.hypot(north, east)
            row[f"a_{component}_rms_m_s2"] = rms(magnitude)
            row[f"a_{component}_p95_m_s2"] = percentile(magnitude, 95.0)
            row[f"a_{component}_peak_m_s2"] = peak(magnitude)
            row[f"a_{component}_band_055_075_energy"] = band_energy(window_t, north, east)
            if component in ("velocity_total", "nominal", "final") and window_t.size >= 3:
                jerk = jerk_magnitude(window_t, north, east)
                row[f"a_{component}_jerk_rms_m_s3"] = rms(jerk)
                row[f"a_{component}_jerk_p95_m_s3"] = percentile(jerk, 95.0)
                row[f"a_{component}_jerk_peak_m_s3"] = peak(jerk)
        for component, index in POWER_FIELDS.items():
            values = np.asarray(vfb[f"data[{index}]"], dtype=float)[mask]
            row[f"p_{component}_positive_integral"] = integrate(window_t, np.maximum(values, 0.0))
            row[f"p_{component}_negative_integral"] = integrate(window_t, np.minimum(values, 0.0))
            row[f"p_{component}_net_integral"] = integrate(window_t, values)
            row[f"p_{component}_mean"] = float(np.nanmean(values)) if values.size else math.nan
        window_rows.append(row)

    route_row = next(row for row in window_rows if row["window"] == "route")
    movement_row = next(row for row in window_rows if row["window"] == "movement")
    final_row = next(row for row in window_rows if row["window"] == "final_10s")
    for component in POWER_FIELDS:
        base[f"route_p_{component}_net_integral"] = route_row[f"p_{component}_net_integral"]
        base[f"movement_p_{component}_net_integral"] = movement_row[f"p_{component}_net_integral"]
        base[f"final10_p_{component}_net_integral"] = final_row[f"p_{component}_net_integral"]
    base["final_jerk_p95_m_s3"] = route_row["a_final_jerk_p95_m_s3"]
    base["final_jerk_rms_m_s3"] = route_row["a_final_jerk_rms_m_s3"]
    base["final_jerk_peak_m_s3"] = route_row["a_final_jerk_peak_m_s3"]
    base["final_acc_rms_m_s2"] = route_row["a_final_rms_m_s2"]
    base["final_acc_p95_m_s2"] = route_row["a_final_p95_m_s2"]
    base["final_acc_peak_m_s2"] = route_row["a_final_peak_m_s2"]
    base["velocity_band_055_075_energy"] = route_row["a_velocity_total_band_055_075_energy"]
    base["nominal_band_055_075_energy"] = route_row["a_nominal_band_055_075_energy"]
    base["final_band_055_075_energy"] = route_row["a_final_band_055_075_energy"]
    overall_valid = all(bool(item["passed"]) for item in checks.values())
    base["base_valid"] = analysis_base_valid
    base["vfb_valid"] = overall_valid
    base["valid"] = overall_valid
    base["vfb_rejection_reasons"] = ";".join(name for name, item in checks.items() if not item["passed"])
    base["vfb_warnings"] = ";".join(warnings)
    return base, window_rows, {"valid": overall_valid, "checks": checks, "warnings": warnings}


def reduction_percent(test: float, baseline: float) -> float:
    return 100.0 * (baseline - test) / abs(baseline) if math.isfinite(test) and math.isfinite(baseline) and abs(baseline) > 1e-9 else math.nan


def compare(rows: Sequence[Dict[str, object]]) -> List[Dict[str, object]]:
    by_mode = {str(row["mode"]): row for row in rows}
    comparisons = []
    for rope, baseline_mode, candidate_modes in (
        (0.6, "VFB_L06_W00", ("VFB_L06_W05",)),
        (0.8, "VFB_L08_W00", ("VFB_L08_W25", "VFB_L08_W05")),
    ):
        for test_mode in candidate_modes:
            if baseline_mode not in by_mode or test_mode not in by_mode:
                continue
            baseline, test = by_mode[baseline_mode], by_mode[test_mode]
            result: Dict[str, object] = {
                "rope_length_m": rope,
                "baseline_mode": baseline_mode,
                "test_mode": test_mode,
                "test_weight": expected_weight(test_mode),
                "both_valid": bool(baseline.get("valid") and test.get("valid")),
            }
            for component in ("observer_error", "velocity_total", "nominal"):
                key = f"movement_p_{component}_net_integral"
                result[f"{component}_net_baseline"] = finite(baseline.get(key))
                result[f"{component}_net_test"] = finite(test.get(key))
                result[f"{component}_net_reduction_pct"] = reduction_percent(finite(test.get(key)), finite(baseline.get(key)))
            for metric in ("swing_angle_rms_deg", "swing_angle_peak_deg", "swing_energy_integral_j_s_kg",
                           "final_5s_swing_angle_rms_deg"):
                result[f"{metric}_change_pct"] = -reduction_percent(finite(test.get(metric)), finite(baseline.get(metric)))
            result["xy_rms_change_pct"] = -reduction_percent(finite(test.get("xy_error_rms_m")), finite(baseline.get("xy_error_rms_m")))
            result["final10_xy_rms_change_pct"] = -reduction_percent(finite(test.get("final_10s_xy_error_rms_m")),
                                                                      finite(baseline.get("final_10s_xy_error_rms_m")))
            result["jerk_p95_change_pct"] = -reduction_percent(finite(test.get("final_jerk_p95_m_s3")),
                                                                finite(baseline.get("final_jerk_p95_m_s3")))
            result["final_acc_rms_change_pct"] = -reduction_percent(finite(test.get("final_acc_rms_m_s2")),
                                                                     finite(baseline.get("final_acc_rms_m_s2")))
            result["test_saturation_ratio"] = finite(test.get("total_envelope_trigger_ratio"))
            result["test_fallback_ratio"] = finite(test.get("fallback_ratio"))
            result["test_as_net"] = finite(test.get("movement_p_as_net_integral"))
            result["test_pas_net"] = finite(test.get("movement_p_pas_net_integral"))
            mechanism = all(finite(result.get(f"{component}_net_reduction_pct")) >= 40.0
                            for component in ("observer_error", "velocity_total", "nominal"))
            swing_ok = all(finite(result.get(f"{metric}_change_pct")) <= 0.0
                           for metric in ("swing_angle_rms_deg", "swing_angle_peak_deg",
                                          "swing_energy_integral_j_s_kg", "final_5s_swing_angle_rms_deg"))
            costs_ok = finite(result["xy_rms_change_pct"]) <= 10.0 \
                and finite(result["final10_xy_rms_change_pct"]) <= 20.0 \
                and finite(result["final_acc_rms_change_pct"]) <= 20.0 \
                and finite(result["jerk_p95_change_pct"]) <= 30.0 \
                and finite(result["test_saturation_ratio"]) < 0.05 \
                and finite(result["test_fallback_ratio"]) < 0.01
            result["mechanism_threshold_pass"] = mechanism
            result["swing_no_worse_pass"] = swing_ok
            result["cost_threshold_pass"] = costs_ok
            if mechanism and swing_ok and costs_ok:
                result["decision"] = "A_success"
            elif mechanism and not swing_ok:
                result["decision"] = "B_power_better_swing_not_better"
            elif swing_ok and not costs_ok:
                result["decision"] = "C_swing_better_but_command_cost"
            else:
                result["decision"] = "E_overall_not_proven"
            comparisons.append(result)
    return comparisons


def fmt(value, digits=3) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def markdown(rows, comparisons) -> str:
    lines = [
        "# ZD680 吊挂无人机 EKF–ESO 融合速度反馈自动验证报告",
        "",
        f"生成时间：{datetime.now().isoformat(timespec='seconds')}",
        "",
        "> 本轮是方案完整性自动验证，不是论文最终重复统计数据。",
        "> 按当前阶段约定，Z 真值/EKF 超限保留原始数值和严格判定，但只作观测警告，不阻止 VFB 水平控制机理比较。",
        "",
        f"## {len(rows)} 次运行",
        "",
        "| 模式 | 分析有效 | 严格Z有效 | 真值Z/EKF-Z峰值(m) | 请求/实际权重 | 回退率 | XY RMS(m) | 摆角 RMS/峰值(°) | 末5s RMS(°) | 饱和率 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in rows:
        lines.append(
            f"| {row['mode']} | {'是' if row.get('valid') else '否'} | {'是' if row.get('strict_base_valid') else '否'} | "
            f"{fmt(row.get('truth_z_peak_m'))}/{fmt(row.get('ekf_truth_z_peak_m'))} | "
            f"{fmt(row.get('requested_weight_median'),2)}/{fmt(row.get('effective_weight_median'),2)} | "
            f"{fmt(100*finite(row.get('fallback_ratio')),3)}% | {fmt(row.get('xy_error_rms_m'))} | "
            f"{fmt(row.get('swing_angle_rms_deg'))}/{fmt(row.get('swing_angle_peak_deg'))} | "
            f"{fmt(row.get('final_5s_swing_angle_rms_deg'))} | {fmt(100*finite(row.get('total_envelope_trigger_ratio')),2)}% |"
        )
    lines += ["", "## 同绳长候选权重相对 W00", "",
              "| 绳长 | 权重 | observer/velocity/nominal 净功降低 | 摆角RMS变化 | 能量积分变化 | XY变化 | a_final RMS变化 | jerk变化 | 判定 |",
              "|---:|---:|---:|---:|---:|---:|---:|---:|---|"]
    for item in comparisons:
        lines.append(
            f"| {item['rope_length_m']:.1f} m | {fmt(item.get('test_weight'),2)} | {fmt(item.get('observer_error_net_reduction_pct'),1)}%/"
            f"{fmt(item.get('velocity_total_net_reduction_pct'),1)}%/{fmt(item.get('nominal_net_reduction_pct'),1)}% | "
            f"{fmt(item.get('swing_angle_rms_deg_change_pct'),1)}% | {fmt(item.get('swing_energy_integral_j_s_kg_change_pct'),1)}% | "
            f"{fmt(item.get('xy_rms_change_pct'),1)}% | {fmt(item.get('final_acc_rms_change_pct'),1)}% | "
            f"{fmt(item.get('jerk_p95_change_pct'),1)}% | {item['decision']} |"
        )
    lines += ["", "## 无人机本体姿态与执行器检查", "",
              "| 模式 | 姿态有效 | 滚转 RMS/峰值(°) | 俯仰 RMS/峰值(°) | 倾角 RMS/P95/峰值(°) | 姿态跟踪误差 RMS/P95/峰值(°) | 水平角速度 RMS/P95/峰值(rad/s) | 电机范围/高饱和率 |",
              "|---|---:|---:|---:|---:|---:|---:|---:|"]
    for row in rows:
        lines.append(
            f"| {row['mode']} | {'是' if row.get('attitude_control_valid') else '否'} | "
            f"{fmt(row.get('roll_rms_deg'))}/{fmt(row.get('roll_peak_abs_deg'))} | "
            f"{fmt(row.get('pitch_rms_deg'))}/{fmt(row.get('pitch_peak_abs_deg'))} | "
            f"{fmt(row.get('tilt_rms_deg'))}/{fmt(row.get('tilt_p95_deg'))}/{fmt(row.get('tilt_peak_deg'))} | "
            f"{fmt(row.get('attitude_tracking_error_rms_deg'))}/{fmt(row.get('attitude_tracking_error_p95_deg'))}/"
            f"{fmt(row.get('attitude_tracking_error_peak_deg'))} | "
            f"{fmt(row.get('horizontal_body_rate_rms_rad_s'))}/{fmt(row.get('horizontal_body_rate_p95_rad_s'))}/"
            f"{fmt(row.get('horizontal_body_rate_peak_rad_s'))} | "
            f"{fmt(row.get('motor_output_min'))}～{fmt(row.get('motor_output_max'))}/"
            f"{fmt(100.0 * finite(row.get('motor_high_saturation_sample_ratio')),3)}% |"
        )
    lines += ["", "## 自动有效性", ""]
    for row in rows:
        if row.get("valid"):
            lines.append(f"- `{row['mode']}`：非 Z 基础飞行有效性和 VFB 专项有效性均通过。")
        else:
            lines.append(f"- `{row['mode']}`：无效；{row.get('rejection_reasons','')} {row.get('vfb_rejection_reasons','')}")
        if row.get("vfb_warnings"):
            lines.append(f"  - 质量警告：{row['vfb_warnings']}")
    lines += ["", "完整的分窗口九类功率、0.55–0.75 Hz 频带能量和四方向加速/巡航/减速数据见 `window_metrics.csv`。", ""]
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument("--mode", action="append", choices=SUPPORTED_MODES,
                        help="analyze one mode; repeat for a selected comparison matrix")
    args = parser.parse_args()
    output = args.output_dir or args.root / "report"
    output.mkdir(parents=True, exist_ok=True)
    selected_modes = tuple(args.mode) if args.mode else MODES
    found = latest_runs(args.root, selected_modes)
    rows, window_rows, validity_payload = [], [], {}
    for mode in selected_modes:
        if mode not in found:
            rows.append({"mode": mode, "valid": False, "vfb_rejection_reasons": "run_missing"})
            continue
        try:
            row, windows, validity = analyze_run(mode, found[mode])
            rows.append(row)
            window_rows.extend(windows)
            validity_payload[mode] = validity
            (found[mode] / "vfb_validity.json").write_text(json.dumps(validity, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        except Exception as exc:
            rows.append({"mode": mode, "valid": False, "vfb_rejection_reasons": f"analysis_error:{exc}",
                         "run_dir": str(found[mode])})
    comparisons = compare(rows)
    write_csv(output / "validation_runs.csv", rows)
    write_csv(output / "window_metrics.csv", window_rows)
    write_csv(output / "validation_comparisons.csv", comparisons)
    summary = {"generated_local": datetime.now().isoformat(timespec="seconds"), "runs": rows,
               "comparisons": comparisons, "vfb_validity": validity_payload}
    (output / "validation_summary.json").write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    (output / "validation_summary.md").write_text(markdown(rows, comparisons), encoding="utf-8")
    print(output / "validation_summary.md")
    return 0 if len(rows) == len(selected_modes) and all(bool(row.get("valid")) for row in rows) else 1


if __name__ == "__main__":
    raise SystemExit(main())
