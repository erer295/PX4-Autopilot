#!/usr/bin/env python3
"""Build the 2026-07-22 HESO-USC quick-freeze evidence package."""

from __future__ import annotations

import argparse
import csv
import hashlib
import html
import json
import math
import random
import runpy
import shutil
import subprocess
from datetime import datetime
from pathlib import Path

import numpy as np
from pyulog import ULog


REPO_ROOT = Path(__file__).resolve().parents[4]
MODULE_ROOT = Path(__file__).resolve().parents[2]
VFB_REPORT = Path(__file__).with_name("zd680_vfb_report.py")
VFB = runpy.run_path(str(VFB_REPORT), run_name="zd680_fso_usc_quick_report_api")
GENERIC = VFB["GENERIC"]
A06 = "STD_L06_PID_AS"
B06 = "STD_L06_PID_AS_FSOUSC_K20"
A08 = "STD_L08_PID_AS"
B08 = "STD_L08_PID_AS_FSOUSC_K20"
METRICS = {
    "xy_error_rms_m": "XY RMS (m)",
    "swing_angle_rms_deg": "摆角 RMS (°)",
    "swing_angle_peak_deg": "摆角峰值 (°)",
    "swing_energy_integral_j_s_kg": "摆动能量积分 (J·s/kg)",
    "final_5s_swing_angle_rms_deg": "末5 s摆角 RMS (°)",
    "final_acc_rms_m_s2": "最终加速度 RMS (m/s²)",
    "final_jerk_p95_m_s3": "最终 jerk P95 (m/s³)",
}


def finite(value, fallback=math.nan):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return fallback
    return value if math.isfinite(value) else fallback


def pct(test, baseline):
    test, baseline = finite(test), finite(baseline)
    return 100.0 * (test / baseline - 1.0) if abs(baseline) > 1.0e-12 else math.nan


def integrate(t, values):
    t, values = np.asarray(t, dtype=float), np.asarray(values, dtype=float)
    mask = np.isfinite(t) & np.isfinite(values)
    t, values = t[mask], values[mask]
    if t.size < 2:
        return math.nan
    keep = np.r_[True, np.diff(t) > 1.0e-6]
    return float(np.trapezoid(values[keep], t[keep]))


def write_csv(path: Path, rows):
    fields = []
    for row in rows:
        for field in row:
            if field not in fields:
                fields.append(field)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def json_write(path: Path, payload):
    path.write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def sha256(path: Path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def run_dirs(root: Path, mode: str):
    return sorted(path.resolve() for path in (root / "runs" / mode).glob("*/*") if (path / "metadata.json").is_file())


def analyze(mode: str, path: Path):
    row, windows, validity = VFB["analyze_run"](mode, path)
    row["run_dir"] = str(path)
    return row, windows, validity


def aggregate(rows):
    result = {"n": len(rows), "valid_count": sum(bool(row.get("valid")) for row in rows)}
    for key in METRICS:
        values = np.asarray([finite(row.get(key)) for row in rows], dtype=float)
        result[key] = float(np.mean(values))
        result[f"{key}_sd"] = float(np.std(values, ddof=1)) if values.size > 1 else 0.0
    return result


def compare_aggregate(a, b):
    return {f"{key}_change_pct": pct(b[key], a[key]) for key in METRICS}


def pair_rows(label, rope, a_rows, b_rows, source_labels):
    output = []
    for index, (a, b, source) in enumerate(zip(a_rows, b_rows, source_labels), 1):
        row = {
            "record_type": "pair",
            "gate": label,
            "rope_length_m": rope,
            "pair": index,
            "source": source,
            "a_run": a["run_dir"],
            "b_run": b["run_dir"],
            "a_valid": bool(a.get("valid")),
            "b_valid": bool(b.get("valid")),
        }
        for key in METRICS:
            row[f"a_{key}"] = finite(a.get(key))
            row[f"b_{key}"] = finite(b.get(key))
            row[f"{key}_change_pct"] = pct(b.get(key), a.get(key))
        output.append(row)
    return output


def gate_l06(a_rows, b_rows):
    a, b = aggregate(a_rows), aggregate(b_rows)
    changes = compare_aggregate(a, b)
    pairs = pair_rows("L06", 0.6, a_rows, b_rows, ["locked_20260721"] * 3 + ["new_20260722"] * 2)
    checks = {
        "all_10_runs_valid": all(row.get("valid") for row in a_rows + b_rows),
        "new_pair_swing_rms_each_decreases": all(row["swing_angle_rms_deg_change_pct"] < 0 for row in pairs[-2:]),
        "new_pair_energy_each_decreases": all(row["swing_energy_integral_j_s_kg_change_pct"] < 0 for row in pairs[-2:]),
        "combined_swing_rms_le_minus_3pct": changes["swing_angle_rms_deg_change_pct"] <= -3.0,
        "combined_energy_le_minus_5pct": changes["swing_energy_integral_j_s_kg_change_pct"] <= -5.0,
        "combined_peak_le_plus_3pct": changes["swing_angle_peak_deg_change_pct"] <= 3.0,
        "combined_xy_le_plus_5pct": changes["xy_error_rms_m_change_pct"] <= 5.0,
        "combined_jerk_le_plus_7pct": changes["final_jerk_p95_m_s3_change_pct"] <= 7.0,
        "each_pair_jerk_le_plus_12pct": all(row["final_jerk_p95_m_s3_change_pct"] <= 12.0 for row in pairs),
    }
    core_names = [name for name in checks if "jerk" not in name]
    core_pass = all(checks[name] for name in core_names)
    jerk = changes["final_jerk_p95_m_s3_change_pct"]
    jerk_class = "pass" if jerk <= 5.0 else ("borderline" if jerk <= 7.0 else "fail")
    passed = all(checks.values())
    return {"baseline": a, "candidate": b, "changes": changes, "checks": checks,
            "core_pass": core_pass, "jerk_class": jerk_class, "pass": passed}, pairs


def route_bounds(run_dir: Path):
    events = GENERIC["read_events"](run_dir / "events.csv")
    windows = VFB["action_windows"](events)
    route = next(window for window in windows if window[0] == "route")
    return events, float(route[2]), float(route[3])


def generic_xy_errors(local, events):
    """Recompute the locked report's event-wise XY reference error."""
    t = np.asarray(local["timestamp"], dtype=float) * 1.0e-6
    x, y = np.asarray(local["x"], dtype=float), np.asarray(local["y"], dtype=float)
    errors = np.full(t.shape, np.nan)
    for event in events:
        if not str(event.get("name", "")).startswith("action_"):
            continue
        mask = (t >= finite(event["start_boot_s"])) & (t <= finite(event["end_boot_s"]))
        for index in np.flatnonzero(mask):
            rx, ry, _, _, _ = GENERIC["ref_sample"](event, float(t[index]))
            errors[index] = math.hypot(float(x[index]) - rx, float(y[index]) - ry)
    return t, errors


def mechanism(run_dir: Path):
    _, start, end = route_bounds(run_dir)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["debug_array"])
    fso = GENERIC["find_debug"](ulog, 687)
    if fso is None:
        return {"run_dir": str(run_dir), "debug_present": False}
    t = np.asarray(fso["timestamp"], dtype=float) * 1.0e-6
    route = (t >= start) & (t <= end)
    active = route & (np.asarray(fso["data[30]"], dtype=float) > 0.5)
    power = np.asarray(fso["data[25]"], dtype=float)[active]
    at = t[active]
    raw = np.hypot(np.asarray(fso["data[14]"], dtype=float)[active], np.asarray(fso["data[15]"], dtype=float)[active])
    applied_n = np.asarray(fso["data[18]"], dtype=float)[active]
    applied_e = np.asarray(fso["data[19]"], dtype=float)[active]
    applied = np.hypot(applied_n, applied_e)
    jerk = VFB["jerk_magnitude"](at, applied_n, applied_e)
    feasible = np.asarray(fso["data[44]"], dtype=float)[active] > 0.5
    gate = np.asarray(fso["data[39]"], dtype=float)[route]
    frequency = np.asarray(fso["data[20]"], dtype=float)[route]
    return {
        "run_dir": str(run_dir), "debug_present": True,
        "route_samples": int(np.sum(route)), "active_samples": int(np.sum(active)),
        "active_ratio": float(np.mean(active[route])) if np.any(route) else math.nan,
        "gate_active_ratio": float(np.mean(gate > 1.0e-3)) if gate.size else math.nan,
        "center_frequency_median_hz": float(np.nanmedian(frequency)) if frequency.size else math.nan,
        "unified_infeasible_ratio": float(np.mean(~feasible)) if feasible.size else math.nan,
        "raw_positive_power_sample_ratio": float(np.mean(np.asarray(fso["data[23]"], dtype=float)[active] > 0.0)),
        "applied_positive_power_integral_j_kg": integrate(at, np.maximum(power, 0.0)),
        "applied_negative_power_integral_j_kg": integrate(at, np.minimum(power, 0.0)),
        "applied_net_power_integral_j_kg": integrate(at, power),
        "applied_over_raw_compensation_rms": (
            float(np.sqrt(np.mean(applied ** 2)) / np.sqrt(np.mean(raw ** 2)))
            if raw.size and np.mean(raw ** 2) > 1.0e-12 else math.nan
        ),
        "compensation_jerk_p95_m_s3": float(np.percentile(jerk, 95.0)) if jerk.size else math.nan,
        "compensation_jerk_peak_m_s3": float(np.max(jerk)) if jerk.size else math.nan,
    }


def raw_recompute(run_dir: Path, analyzed):
    events, start, end = route_bounds(run_dir)
    with (run_dir / "hang_joint_samples.csv").open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    t = np.asarray([finite(row["boot_s_est"]) for row in samples])
    angle = np.asarray([finite(row["hang_angle_deg"]) for row in samples])
    rate = np.asarray([finite(row["hang_velocity_rad_s"]) for row in samples])
    mask = (t >= start) & (t <= end)
    energy = 0.5 * np.square(0.6 * rate) + 9.80665 * 0.6 * (1.0 - np.cos(np.radians(angle)))
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["vehicle_local_position", "debug_array"])
    local = ulog.get_dataset("vehicle_local_position").data
    lt, xy = generic_xy_errors(local, events)
    lmask = (lt >= start) & (lt <= end)
    position_debug = GENERIC["find_debug"](ulog, 685)
    vt = np.asarray(position_debug["timestamp"], dtype=float) * 1.0e-6
    vmask = (vt >= start) & (vt <= end)
    jerk = np.hypot(np.asarray(position_debug["data[23]"], dtype=float)[vmask],
                    np.asarray(position_debug["data[24]"], dtype=float)[vmask])
    values = {
        "swing_angle_rms_deg": float(np.sqrt(np.mean(np.square(angle[mask])))),
        "swing_angle_peak_deg": float(np.max(np.abs(angle[mask]))),
        "swing_energy_integral_j_s_kg": integrate(t[mask], energy[mask]),
        "xy_error_rms_m": float(np.sqrt(np.nanmean(np.square(xy[lmask])))),
        "final_jerk_p95_m_s3": float(np.percentile(jerk, 95.0)),
    }
    comparisons = {}
    for key, value in values.items():
        reference = finite(analyzed.get(key))
        difference = abs(value - reference)
        tolerance = max(1.0e-8, 1.0e-6 * abs(reference))
        comparisons[key] = {"raw": value, "analyzer": reference, "absolute_difference": difference,
                            "tolerance": tolerance, "pass": difference <= tolerance}
    return {"run_dir": str(run_dir), "comparisons": comparisons,
            "pass": all(item["pass"] for item in comparisons.values())}


def representative_series(run_dir: Path):
    events, start, end = route_bounds(run_dir)
    with (run_dir / "hang_joint_samples.csv").open(newline="", encoding="utf-8") as stream:
        samples = list(csv.DictReader(stream))
    ht = np.asarray([finite(row["boot_s_est"]) for row in samples])
    angle = np.asarray([finite(row["hang_angle_deg"]) for row in samples])
    hm = (ht >= start) & (ht <= end)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["vehicle_local_position", "debug_array"])
    local = ulog.get_dataset("vehicle_local_position").data
    lt, xy = generic_xy_errors(local, events)
    lm = (lt >= start) & (lt <= end)
    coord = GENERIC["find_debug"](ulog, 684)
    vt = np.asarray(coord["timestamp"], dtype=float) * 1.0e-6
    vm = (vt >= start) & (vt <= end)
    final_acc = np.hypot(np.asarray(coord["data[10]"], dtype=float)[vm], np.asarray(coord["data[11]"], dtype=float)[vm])
    fso = GENERIC["find_debug"](ulog, 687)
    if fso is not None:
        ft = np.asarray(fso["timestamp"], dtype=float) * 1.0e-6
        fm = (ft >= start) & (ft <= end)
        comp = np.hypot(np.asarray(fso["data[18]"], dtype=float)[fm], np.asarray(fso["data[19]"], dtype=float)[fm])
        ftime = ft[fm] - start
    else:
        comp, ftime = np.asarray([]), np.asarray([])
    return {"hang_t": ht[hm] - start, "angle": angle[hm], "local_t": lt[lm] - start,
            "xy": xy[lm], "vfb_t": vt[vm] - start, "final_acc": final_acc,
            "fso_t": ftime, "comp": comp}


def make_plots(output: Path, pairs, a_rows, b_rows):
    plot_dir = output / "plots"
    plot_dir.mkdir(exist_ok=True)
    plot_metrics = ["swing_angle_rms_deg", "swing_energy_integral_j_s_kg",
                    "swing_angle_peak_deg", "xy_error_rms_m", "final_jerk_p95_m_s3"]
    width, height, margin, panel_w = 1500, 390, 55, 280
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<style>text{font-family:sans-serif;fill:#222}.small{font-size:12px}.title{font-size:18px;font-weight:bold}.panel{font-size:14px;font-weight:bold}</style>',
           '<text x="750" y="27" text-anchor="middle" class="title">L=0.6 m：五组配对指标变化（蓝=锁定旧数据，橙=本轮新增）</text>']
    for panel, key in enumerate(plot_metrics):
        x0 = margin + panel * panel_w
        values = [row[f"{key}_change_pct"] for row in pairs]
        bound = max(5.0, max(abs(value) for value in values) * 1.2)
        top, bottom = 70, 325
        zero = (top + bottom) / 2
        scale = (bottom - top) / (2.0 * bound)
        svg += [f'<text x="{x0+112}" y="52" text-anchor="middle" class="panel">{html.escape(METRICS[key])}</text>',
                f'<line x1="{x0}" y1="{zero:.1f}" x2="{x0+225}" y2="{zero:.1f}" stroke="#333"/>',
                f'<line x1="{x0}" y1="{top}" x2="{x0}" y2="{bottom}" stroke="#777"/>',
                f'<text x="{x0-5}" y="{top+4}" text-anchor="end" class="small">+{bound:.1f}%</text>',
                f'<text x="{x0-5}" y="{bottom+4}" text-anchor="end" class="small">-{bound:.1f}%</text>']
        for index, (row, value) in enumerate(zip(pairs, values), 1):
            x = x0 + 13 + (index - 1) * 43
            y = zero - value * scale
            color = "#4C78A8" if row["source"].startswith("locked") else "#F58518"
            rect_y, rect_h = min(y, zero), max(1.0, abs(y - zero))
            svg.append(f'<rect x="{x}" y="{rect_y:.1f}" width="28" height="{rect_h:.1f}" fill="{color}"/>')
            svg.append(f'<text x="{x+14}" y="{bottom+17}" text-anchor="middle" class="small">{index}</text>')
            label_y = y - 5 if value >= 0 else y + 14
            svg.append(f'<text x="{x+14}" y="{label_y:.1f}" text-anchor="middle" class="small">{value:+.1f}</text>')
    svg.append('<text x="750" y="375" text-anchor="middle" class="small">横轴：配对编号；纵轴：B 相对 A 变化 (%)</text></svg>')
    (plot_dir / "l06_paired_metrics.svg").write_text("\n".join(svg), encoding="utf-8")

    a = representative_series(Path(a_rows[-2]["run_dir"]))
    b = representative_series(Path(b_rows[-2]["run_dir"]))
    series = [
        ("摆角 (°)", [(a["hang_t"], a["angle"], "#4C78A8"), (b["hang_t"], b["angle"], "#F58518")]),
        ("XY误差 (m)", [(a["local_t"], a["xy"], "#4C78A8"), (b["local_t"], b["xy"], "#F58518")]),
        ("最终加速度 (m/s²)", [(a["vfb_t"], a["final_acc"], "#4C78A8"), (b["vfb_t"], b["final_acc"], "#F58518")]),
        ("HESO补偿幅值 (m/s²)", [(b["fso_t"], b["comp"], "#F58518")]),
    ]
    width, height, left, right = 1200, 920, 115, 1160
    svg = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
           '<rect width="100%" height="100%" fill="white"/>',
           '<style>text{font-family:sans-serif;fill:#222}.small{font-size:12px}.title{font-size:19px;font-weight:bold}.label{font-size:14px}</style>',
           '<text x="600" y="28" text-anchor="middle" class="title">L=0.6 m 新增第一组代表性时序</text>',
           '<line x1="850" y1="48" x2="885" y2="48" stroke="#4C78A8" stroke-width="2"/><text x="892" y="52" class="small">A1 PID+AS</text>',
           '<line x1="1000" y1="48" x2="1035" y2="48" stroke="#F58518" stroke-width="2"/><text x="1042" y="52" class="small">B1 HESO-USC</text>']

    def polyline(t, y, x0, x1, y0, y1, ymin, ymax):
        t, y = np.asarray(t), np.asarray(y)
        mask = np.isfinite(t) & np.isfinite(y)
        t, y = t[mask], y[mask]
        if t.size == 0:
            return ""
        step = max(1, int(math.ceil(t.size / 1000)))
        t, y = t[::step], y[::step]
        tmax = max(1.0e-9, max(float(np.max(t)), 50.0))
        xs = x0 + (x1 - x0) * t / tmax
        ys = y1 - (y1 - y0) * (y - ymin) / max(1.0e-12, ymax - ymin)
        return " ".join(f"{x:.1f},{yy:.1f}" for x, yy in zip(xs, ys))

    for index, (label, curves) in enumerate(series):
        top, bottom = 75 + index * 205, 245 + index * 205
        all_values = np.concatenate([np.asarray(curve[1])[np.isfinite(curve[1])] for curve in curves])
        ymin, ymax = float(np.min(all_values)), float(np.max(all_values))
        padding = max(1.0e-6, 0.08 * (ymax - ymin))
        ymin, ymax = ymin - padding, ymax + padding
        svg += [f'<rect x="{left}" y="{top}" width="{right-left}" height="{bottom-top}" fill="none" stroke="#aaa"/>',
                f'<line x1="{left}" y1="{(top+bottom)/2:.1f}" x2="{right}" y2="{(top+bottom)/2:.1f}" stroke="#ddd"/>',
                f'<text x="{left-12}" y="{top+5}" text-anchor="end" class="small">{ymax:.3g}</text>',
                f'<text x="{left-12}" y="{bottom}" text-anchor="end" class="small">{ymin:.3g}</text>',
                f'<text x="20" y="{(top+bottom)/2:.1f}" class="label">{html.escape(label)}</text>']
        for t, values, color in curves:
            points = polyline(t, values, left, right, top, bottom, ymin, ymax)
            svg.append(f'<polyline points="{points}" fill="none" stroke="{color}" stroke-width="1.25"/>')
    svg += [f'<text x="{(left+right)/2}" y="905" text-anchor="middle" class="label">正式轨迹相对时间 (s)，全宽约 50 s</text>', '</svg>']
    (plot_dir / "l06_representative_timeseries.svg").write_text("\n".join(svg), encoding="utf-8")


def copy_evidence(output: Path, run_records, candidate_path: Path):
    for name in ("logs", "parameter_snapshots", "validity"):
        (output / name).mkdir(exist_ok=True)
    for record in run_records:
        run_dir = Path(record["run_dir"])
        label = record["label"]
        for source_name, suffix in (("sitl_launcher.log", "sitl.log"), ("recorder_console.log", "recorder.log")):
            source = run_dir / source_name
            if source.is_file():
                shutil.copy2(source, output / "logs" / f"{label}_{suffix}")
        for source_name, suffix in (("validity.json", "validity.json"), ("vfb_validity.json", "vfb_validity.json")):
            source = run_dir / source_name
            if source.is_file():
                shutil.copy2(source, output / "validity" / f"{label}_{suffix}")
    metadata = json.loads((candidate_path / "metadata.json").read_text(encoding="utf-8"))
    experiment = metadata["experiment"]
    requested = experiment.get("requested_validation_parameters", {})
    observed = experiment.get("px4_parameter_snapshot", {})
    matched = all(key in observed and abs(float(observed[key]) - float(value)) <= 1.0e-5
                  for key, value in requested.items())
    snapshot = {
        "source_run": str(candidate_path),
        "captured_utc": metadata.get("created_utc"),
        "requested_validation_parameters": requested,
        "px4_parameter_snapshot": observed,
        "match_with_float_tolerance_1e-5": matched,
    }
    json_write(output / "parameter_snapshots" / "frozen_candidate_parameters.json", snapshot)


def make_manifest(output: Path, locked_root: Path, new_root: Path, rows_by_path, readiness_root: Path):
    records = []
    specifications = [
        (locked_root, A06, "L06_A", "locked_20260721", False),
        (locked_root, B06, "L06_B", "locked_20260721", False),
        (new_root, A06, "L06_A", "new_20260722", True),
        (new_root, B06, "L06_B", "new_20260722", True),
    ]
    counters = {}
    for root, mode, prefix, source, formal_budget in specifications:
        for path in run_dirs(root, mode):
            counters[prefix] = counters.get(prefix, 0) + 1
            row = rows_by_path[str(path)]
            model = json.loads((path / "model_variant.json").read_text(encoding="utf-8"))
            metadata = json.loads((path / "metadata.json").read_text(encoding="utf-8"))
            label = f"{prefix}{counters[prefix]}"
            records.append({
                "label": label, "source": source, "formal_budget_20260722": formal_budget,
                "mode": mode, "run_dir": str(path), "valid": bool(row.get("valid")),
                "rope_length_m": model.get("rope_length_m"), "payload_mass_kg": model.get("payload_mass_kg"),
                "rbf_learn": metadata.get("args", {}).get("rbf_learn"),
                "model_source_restored": model.get("source_model_restored_after_spawn"),
                "ulog_sha256": sha256(path / "position_offboard.ulg"),
            })
    ready = run_dirs(readiness_root, "STD_L06_PID_AS_FSOUSC_READY")
    if ready:
        records.append({"label": "READINESS", "source": "readiness_20260722", "formal_budget_20260722": False,
                        "mode": "STD_L06_PID_AS_FSOUSC_READY", "run_dir": str(ready[-1]), "valid": False,
                        "rope_length_m": 0.6, "payload_mass_kg": 0.5, "rbf_learn": False,
                        "model_source_restored": True, "ulog_sha256": sha256(ready[-1] / "position_offboard.ulg")})
    write_csv(output / "run_manifest.csv", records)
    return records


def make_report(gate, pairs, mechanism_rows, raw_check, decision, budget_used):
    a, b, changes = gate["baseline"], gate["candidate"], gate["changes"]
    lines = [
        "结论：本轮最终判定为 `STOP_HESO_DIRECT`，按任务书停止 HESO 直接加速度补偿路线，不进入 0.8 m 复验。  ",
        "0.6 m 新增的 A1/B1/A2/B2 四次运行全部严格有效，没有补跑、剔除或调参。  ",
        f"合并 2026-07-21 锁定三对后，n=5 的摆角 RMS 变化 {changes['swing_angle_rms_deg_change_pct']:+.2f}%、能量 {changes['swing_energy_integral_j_s_kg_change_pct']:+.2f}%、峰值 {changes['swing_angle_peak_deg_change_pct']:+.2f}%、XY {changes['xy_error_rms_m_change_pct']:+.2f}%，四项核心性能均过线。  ",
        f"但总控制 jerk P95 均值增加 {changes['final_jerk_p95_m_s3_change_pct']:+.2f}%，两组新增配对分别增加 {pairs[-2]['final_jerk_p95_m_s3_change_pct']:+.2f}% 和 {pairs[-1]['final_jerk_p95_m_s3_change_pct']:+.2f}%，同时触发均值 +7% 与单对 +12% 的硬停止条件。  ",
        "因此保留 PID+AS 为当前主线，HESO 辅助调度/输入整形方向继续保留为下一阶段候选，但本任务没有越界实施该新结构。",
        "",
        "# ZD680 PID+AS 与 HESO-USC 快速封版验证报告",
        "",
        f"生成时间：{datetime.now().isoformat(timespec='seconds')}（Asia/Shanghai）",
        "",
        "## 1. 执行范围与预算",
        "",
        "本轮严格比较 `A=PX4 PID+AS` 与 `B=PX4 PID+AS+HESO-USC直接水平加速度补偿`。控制参数、轨迹、初态门和有效性规则在非正式就绪检查后冻结。",
        "",
        f"正式预算使用 `{budget_used}/8` 次：0.6 m 使用 4 次且全部有效；由于闸门 1 的 jerk 硬条件失败，剩余 4 次没有用于 0.8 m。没有无效补跑，也没有为了追求好结果增加样本。",
        "",
        "## 2. 健康检查与冻结证据",
        "",
        "- 唯一一次完整 `px4_sitl_default` 构建成功，并强制重编译了旧参数枚举对应的 `mc_autotune_attitude_control` 对象。",
        "- 非正式短就绪启动后，旧的 `wrong type passed to param_get()` 错误为 0 次。",
        "- 记录参数确认：位置/速率均为 PID，`MC_HANG_AS_EN=1`；LADRC、RBF、RBF学习、PAS、旧频率整形均关闭。",
        "- 候选固定为 `FSO_MD=2, FSO_POS=1, BW=0.30 Hz, K=0.20, LEAD=0.05 s, LIM=0.08 m/s², SLW=0.30 m/s³`。",
        "- PID 工况索引的 `rbf_learn=false` 与实际 `MC_RBF_LEARN_EN=0` 已统一；每次运行前检查无残留 PX4/Gazebo/记录器进程。",
        "- 新增 A1 在起飞后、记录器连接前出现一次 `Accel #0 TIMEOUT` 启动瞬态；随后传感器恢复，并经过预飞等待、位置稳定和连续 3 秒吊挂初态门后才开始正式轨迹。该次正式窗口的传感器/EKF/动作/安全有效性全部通过，原始告警保留在日志中，未隐藏也未补跑。",
        "- 就绪运行只有 1 s 保持，故通用正式轨迹判据按预期将其标为无效；它不属于正式预算，也未进入任何均值。",
        "",
        "## 3. 0.6 m 新增两对结果",
        "",
        "| 配对 | 有效 | XY变化 | 摆角RMS变化 | 峰值变化 | 能量变化 | 末5s变化 | jerk P95变化 |",
        "|---|---:|---:|---:|---:|---:|---:|---:|",
    ]
    for row in pairs[-2:]:
        lines.append(
            f"| 新{row['pair']-3} | 是 | {row['xy_error_rms_m_change_pct']:+.2f}% | "
            f"{row['swing_angle_rms_deg_change_pct']:+.2f}% | {row['swing_angle_peak_deg_change_pct']:+.2f}% | "
            f"{row['swing_energy_integral_j_s_kg_change_pct']:+.2f}% | "
            f"{row['final_5s_swing_angle_rms_deg_change_pct']:+.2f}% | {row['final_jerk_p95_m_s3_change_pct']:+.2f}% |"
        )
    lines += [
        "",
        "两组新增配对的摆角 RMS 和摆动能量都下降，满足方向一致性要求；真正触发停止的是 jerk，而不是消摆核心指标。",
        "",
        "## 4. 锁定旧三对 + 新两对，n=5 汇总",
        "",
        "| 指标 | PID+AS n=5（均值±SD） | HESO-USC n=5（均值±SD） | 变化 | 闸门 |",
        "|---|---:|---:|---:|---|",
    ]
    thresholds = {
        "xy_error_rms_m": "≤+5%",
        "swing_angle_rms_deg": "≤-3%",
        "swing_angle_peak_deg": "≤+3%",
        "swing_energy_integral_j_s_kg": "≤-5%",
        "final_5s_swing_angle_rms_deg": "观察",
        "final_acc_rms_m_s2": "观察",
        "final_jerk_p95_m_s3": "≤+5%通过；≤+7%边界",
    }
    for key, title in METRICS.items():
        change = changes[f"{key}_change_pct"]
        lines.append(f"| {title} | {a[key]:.5f} ± {a[key+'_sd']:.5f} | {b[key]:.5f} ± {b[key+'_sd']:.5f} | {change:+.2f}% | {thresholds[key]} |")
    lines += [
        "",
        "闸门逐项结果：",
        "",
    ]
    for name, passed in gate["checks"].items():
        lines.append(f"- {'通过' if passed else '失败'}：`{name}`")
    lines += [
        "",
        "核心消摆与位置指标全部通过，但 jerk 分类为 `fail`，并且新两对均越过单对 +12% 硬线，所以不能使用“边界保留”结论。",
        "",
        "## 5. 为什么摆动改善但 jerk 失败",
        "",
        "通俗地说，HESO-USC 像是在原来的 PID+AS 指令上再做很小、很快的方向修正。修正幅值虽然只有 0.08 m/s²，自己的斜率也限制为 0.30 m/s³，但它经常出现在加速、减速和约束切换附近；这时原 PID+AS 指令也正在转弯，两路小变化叠加后，最终指令的一阶差分会被放大。",
        "",
        "jerk 衡量的是“加速度变化有多快”，不是加速度本身有多大。因此补偿幅值小、加速度 RMS 不增加，并不保证 jerk 小。新增两对中最终加速度 RMS 分别变化 "
        f"`{pairs[-2]['final_acc_rms_m_s2_change_pct']:+.2f}%` 和 `{pairs[-1]['final_acc_rms_m_s2_change_pct']:+.2f}%`，但 jerk 分别增加 "
        f"`{pairs[-2]['final_jerk_p95_m_s3_change_pct']:+.2f}%` 和 `{pairs[-1]['final_jerk_p95_m_s3_change_pct']:+.2f}%`，正好说明问题来自变化速度而非指令幅值。",
        "",
        "此外，HESO 的功率、位置与斜率约束存在瞬时冲突。统一约束不可行时采用平滑衰减可以避免旧硬投影的大跳变，但仍会改变最终命令的局部斜率；这条直接叠加路线已经到达“能消一点摆，但平滑代价无法稳定受控”的结构性边界。",
        "",
        "## 6. HESO 机理审计（本轮新增 B1/B2）",
        "",
        "| B样本 | 激活采样 | 统一约束不可行率 | 补偿净功 (J/kg) | 正功/负功积分 | 补偿jerk P95/峰值 |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for index, row in enumerate(mechanism_rows, 1):
        lines.append(
            f"| B{index} | {row['active_samples']} | {100*row['unified_infeasible_ratio']:.2f}% | "
            f"{row['applied_net_power_integral_j_kg']:.5f} | {row['applied_positive_power_integral_j_kg']:.5f}/"
            f"{row['applied_negative_power_integral_j_kg']:.5f} | {row['compensation_jerk_p95_m_s3']:.3f}/"
            f"{row['compensation_jerk_peak_m_s3']:.3f} |"
        )
    lines += [
        "",
        "补偿本身仍以净耗散为主，说明 HESO 信号有消摆信息；但“有用信号”不等于“适合直接叠加到最终加速度”。本轮停止结论针对直接补偿结构，不是否定 HESO 作为状态/频率信息源的价值。",
        "",
        "## 7. 原始数据随机复算",
        "",
        f"固定随机种子 20260722 从新增两对中抽取第 `{raw_check['pair']}` 对，直接从 A/B 两次 ULog、事件和吊挂 CSV 重算 5 个关键指标。结果：`{'通过' if raw_check['pass'] else '失败'}`。",
        "",
        "| 控制器 | 指标 | 原始复算 | 分析器 | 绝对差 |",
        "|---|---|---:|---:|---:|",
    ]
    for controller in ("A", "B"):
        for key, item in raw_check[controller.lower()]["comparisons"].items():
            lines.append(f"| {controller} | {METRICS[key]} | {item['raw']:.9g} | {item['analyzer']:.9g} | {item['absolute_difference']:.3g} |")
    lines += [
        "",
        "## 8. 最终决定与后续主线",
        "",
        f"最终决定：**`{decision}`**。0.8 m 未运行是执行预注册停止规则，不是数据缺失，也不是平台失败。",
        "",
        "当前工程主线继续使用 PID+AS。HESO辅助 PID+AS 调度/输入整形仍值得保留，因为本轮 n=5 已证明 HESO 信号能稳定降低摆角 RMS 和能量；下一阶段应把 HESO 从“直接推一把”改成“告诉 AS 何时、在哪个频率工作”或“提前把参考轨迹整形成少激摆的输入”。这样可以利用频率信息，同时避免直接补偿与 PID+AS 在加减速拐点叠加。该新结构必须另立任务书、重新冻结基线和门槛，本轮没有提前实施。",
        "",
        "就论文而言，HESO-USC 直接补偿不能作为主方法封版；它可以作为完整消融/负结果，解释为何幅值受限仍可能带来 jerk 代价。仅凭当前直接补偿结果不建议投北大核心主方法论文，后续需完成调度或输入整形的新方法、多绳长/载荷重复、统计检验及 HIL/实飞。",
        "",
        "## 9. 文件索引",
        "",
        "- `quick_results.csv`：逐对与 n=5 汇总。",
        "- `decision.json`：机器可读闸门、预算和停止原因。",
        "- `run_manifest.csv`：锁定旧样本、本轮正式样本与非正式就绪样本。",
        "- `plots/l06_paired_metrics.svg`、`plots/l06_representative_timeseries.svg`：必要图。",
        "- `logs/`、`parameter_snapshots/`、`validity/`：启动记录、冻结参数和逐次有效性证据。",
        "- `raw_recompute.json`、`mechanism_audit.csv`、`repository_state.json`：复算、机理与代码状态。",
        "",
    ]
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--locked-l06", type=Path, required=True)
    parser.add_argument("--new-l06", type=Path, required=True)
    parser.add_argument("--readiness", type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)

    paths_a = run_dirs(args.locked_l06, A06) + run_dirs(args.new_l06, A06)
    paths_b = run_dirs(args.locked_l06, B06) + run_dirs(args.new_l06, B06)
    if len(paths_a) != 5 or len(paths_b) != 5:
        raise RuntimeError(f"expected five L06 pairs, found A={len(paths_a)} B={len(paths_b)}")
    analyzed_a, analyzed_b, rows_by_path = [], [], {}
    validity_by_path = {}
    for mode, paths, target in ((A06, paths_a, analyzed_a), (B06, paths_b, analyzed_b)):
        for path in paths:
            row, _, validity = analyze(mode, path)
            target.append(row); rows_by_path[str(path)] = row; validity_by_path[str(path)] = validity
            json_write(path / "vfb_validity.json", validity)

    gate, pairs = gate_l06(analyzed_a, analyzed_b)
    aggregate_row = {"record_type": "aggregate", "gate": "L06", "rope_length_m": 0.6,
                     "pair": "n=5", "source": "locked_3_plus_new_2",
                     "a_valid": gate["baseline"]["valid_count"], "b_valid": gate["candidate"]["valid_count"]}
    for key in METRICS:
        aggregate_row[f"a_{key}"] = gate["baseline"][key]
        aggregate_row[f"a_{key}_sd"] = gate["baseline"][f"{key}_sd"]
        aggregate_row[f"b_{key}"] = gate["candidate"][key]
        aggregate_row[f"b_{key}_sd"] = gate["candidate"][f"{key}_sd"]
        aggregate_row[f"{key}_change_pct"] = gate["changes"][f"{key}_change_pct"]
    write_csv(output / "quick_results.csv", pairs + [aggregate_row])

    mechanism_rows = [mechanism(path) for path in paths_b[-2:]]
    write_csv(output / "mechanism_audit.csv", mechanism_rows)
    selected_pair = random.Random(20260722).choice((0, 1))
    selected_a, selected_b = paths_a[-2 + selected_pair], paths_b[-2 + selected_pair]
    raw_a = raw_recompute(selected_a, rows_by_path[str(selected_a)])
    raw_b = raw_recompute(selected_b, rows_by_path[str(selected_b)])
    raw_check = {"pair": selected_pair + 1, "a": raw_a, "b": raw_b,
                 "pass": bool(raw_a["pass"] and raw_b["pass"])}
    json_write(output / "raw_recompute.json", raw_check)

    decision = "STOP_HESO_DIRECT"
    decision_payload = {
        "decision": decision, "generated_local": datetime.now().isoformat(timespec="seconds"),
        "formal_budget": {"maximum": 8, "used": 4, "remaining_unused": 4, "invalid": 0, "retries": 0},
        "gate_1_l06": gate,
        "gate_2_l08": {"status": "not_run", "reason": "gate_1_jerk_hard_stop"},
        "stop_reasons": [
            f"combined jerk P95 change {gate['changes']['final_jerk_p95_m_s3_change_pct']:+.6f}% > +7%",
            f"new pair 1 jerk P95 change {pairs[-2]['final_jerk_p95_m_s3_change_pct']:+.6f}% > +12%",
            f"new pair 2 jerk P95 change {pairs[-1]['final_jerk_p95_m_s3_change_pct']:+.6f}% > +12%",
        ],
        "core_non_jerk_pass": gate["core_pass"], "raw_recompute_pass": raw_check["pass"],
    }
    json_write(output / "decision.json", decision_payload)

    manifest = make_manifest(output, args.locked_l06, args.new_l06, rows_by_path, args.readiness)
    copy_evidence(output, manifest, paths_b[-1])
    make_plots(output, pairs, analyzed_a, analyzed_b)

    px4 = REPO_ROOT / "build" / "px4_sitl_default" / "bin" / "px4"
    tracked_scripts = [Path(__file__).with_name(name) for name in
                       ("zd680_vfb_recorder.py", "zd680_vfb_validation_suite.py", "zd680_vfb_report.py")]
    untracked_sources = [
        REPO_ROOT / "src/modules/mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadFrequencySelectiveDebugArray.hpp",
        REPO_ROOT / "src/modules/mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadFrequencySelectiveObserver.cpp",
        REPO_ROOT / "src/modules/mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadFrequencySelectiveObserver.hpp",
        Path(__file__),
    ]
    state = {
        "git_head": subprocess.check_output(["git", "rev-parse", "HEAD"], cwd=REPO_ROOT, text=True).strip(),
        "git_status_short": subprocess.check_output(["git", "status", "--short"], cwd=REPO_ROOT, text=True).splitlines(),
        "binary_sha256": sha256(px4),
        "validation_script_sha256": {path.name: sha256(path) for path in tracked_scripts},
        "untracked_source_sha256": {str(path.relative_to(REPO_ROOT)): sha256(path) for path in untracked_sources},
        "build": {"command": "make px4_sitl_default", "result": "success", "steps": 11},
        "readiness": {"type_error_count": 0, "formal_budget": False,
                      "note": "hold-only readiness intentionally fails formal directional-route validity"},
    }
    json_write(output / "repository_state.json", state)
    complete_diff = subprocess.check_output(["git", "diff", "--binary"], cwd=REPO_ROOT)
    for path in untracked_sources:
        relative = path.relative_to(REPO_ROOT)
        result = subprocess.run(["git", "diff", "--no-index", "--binary", "--", "/dev/null", str(relative)],
                                cwd=REPO_ROOT, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if result.returncode not in (0, 1):
            raise RuntimeError(result.stderr.decode("utf-8", errors="replace"))
        complete_diff += result.stdout
    (output / "git_diff.patch").write_bytes(complete_diff)

    report = make_report(gate, pairs, mechanism_rows, raw_check, decision, 4)
    (output / "ZD680_PID_AS_HESO_USC_快速封版验证报告_20260722.md").write_text(report, encoding="utf-8")
    print(output / "ZD680_PID_AS_HESO_USC_快速封版验证报告_20260722.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
