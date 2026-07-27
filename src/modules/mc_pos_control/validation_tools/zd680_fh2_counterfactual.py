#!/usr/bin/env python3
"""Chained-reference counterfactual screen for the FHv2 trajectory.

The earlier per-segment replay driven by ACTUAL acceleration (savgol
derivative of velocity) proved unreliable: clock skew between live and joint
sample streams plus differentiation noise produced phantom swing (model peak
4 deg where truth was 0.9 deg on calm segments).  This screen instead uses
chained REFERENCE-driven simulation over the whole four-direction route:

1. Validation -- run the exact machinery on the FAILED FHv1 trajectory
   (reference acceleration from its frozen coefficients) and compare the
   chained prediction against the measured joint truth, segment by segment.
   If the chain reproduces FHv1's measured peaks, the same machinery is
   trustworthy for FHv2.
2. Counterfactual -- chain the FHv2 reference over the same route from the
   same strict initial state.  Preregistered screen gate: worst predicted
   segment peak < 3.5 deg and reference jerk < 1.6 m/s^3 (the jerk part is
   checked from the design audit).
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

import importlib.util

_DESIGN_SPEC = importlib.util.spec_from_file_location(
    "fh2_design", str(Path(__file__).with_name("zd680_fuci_fh2_design.py")))
fh2_design = importlib.util.module_from_spec(_DESIGN_SPEC)
_DESIGN_SPEC.loader.exec_module(fh2_design)

DIRECTION_UNIT = {
    "north": (1.0, 0.0), "south": (-1.0, 0.0), "east": (0.0, 1.0), "west": (0.0, -1.0),
}
ROUTE = (
    ("north", 6.0), ("hold", 5.0), ("east", 6.0), ("hold", 5.0),
    ("south", 6.0), ("hold", 5.0), ("west", 6.0), ("hold", 10.0),
)
SCREEN_PEAK_DEG = 3.5
SCREEN_JERK_M_S3 = 1.6
DT = 0.01


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="", encoding="utf-8") as stream:
        return list(csv.DictReader(stream))


def load_joints(run_dir: Path) -> dict[str, np.ndarray]:
    rows = read_csv(run_dir / "hang_joint_samples.csv")
    return {
        "t": np.asarray([finite(r.get("boot_s_est")) for r in rows]),
        "roll": np.asarray([finite(r.get("roll_position_rad")) for r in rows]),
        "pitch": np.asarray([finite(r.get("pitch_position_rad")) for r in rows]),
    }


def route_start(run_dir: Path) -> float:
    """Absolute boot time of the first move (north) from events.csv."""
    for row in read_csv(run_dir / "events.csv"):
        if "north" in str(row.get("name", "")):
            return finite(row.get("start_boot_s"))
    raise RuntimeError(f"no north move event in {run_dir}")


def shaped_acceleration(order: int, coefficients: np.ndarray,
                        duration: float = 6.0) -> tuple[np.ndarray, np.ndarray]:
    """Reference acceleration (m/s^2) over one move for a coefficient set."""
    time = np.arange(0.0, duration + 0.5 * DT, DT)
    u = time / duration
    _, _, base_acceleration = fh2_design.trajectory_progress(time, duration)
    _, _, basis_acceleration_u = fh2_design.finite_horizon_basis(order, u)
    acceleration = base_acceleration + basis_acceleration_u @ coefficients / (duration ** 2)
    return time, fh2_design.DISTANCE_M * acceleration


def chained_prediction(order: int, coefficients: np.ndarray, rope_length_m: float,
                       init_state: tuple[float, float, float, float]
                       ) -> dict[str, object]:
    """Chain the reference over ROUTE; return per-segment swing magnitudes."""
    move_time, move_acc = shaped_acceleration(order, coefficients)
    forcing = []
    times = []
    t_cursor = 0.0
    for name, duration in ROUTE:
        count = int(round(duration / DT))
        seg_t = t_cursor + np.arange(count) * DT
        times.append(seg_t)
        if name in DIRECTION_UNIT:
            ux, uy = DIRECTION_UNIT[name]
            acc = np.interp(seg_t - t_cursor, move_time, move_acc)
            forcing.append(np.stack((acc * ux, acc * uy), axis=1))
        else:
            forcing.append(np.zeros((count, 2)))
        t_cursor += duration
    t_all = np.concatenate(times)
    forcing_all = np.concatenate(forcing, axis=0)
    angles, _ = fh2_design.simulate_pendulum(
        forcing_all, DT, rope_length_m, 1.0,
        np.array([init_state[0], init_state[2]]), np.array([init_state[1], init_state[3]]),
    )
    magnitude = np.sqrt(angles[:, 0] ** 2 + angles[:, 1] ** 2)
    segments = []
    t_cursor = 0.0
    for name, duration in ROUTE:
        mask = (t_all >= t_cursor) & (t_all < t_cursor + duration)
        segments.append({
            "name": name, "start_s": t_cursor, "duration_s": duration,
            "peak_deg": float(np.max(magnitude[mask]) * 180.0 / math.pi),
            "rms_deg": float(np.sqrt(np.mean(magnitude[mask] ** 2)) * 180.0 / math.pi),
            "entry_deg": float(magnitude[int(mask.argmax())] * 180.0 / math.pi),
        })
        t_cursor += duration
    return {"t": t_all, "magnitude_deg": magnitude * 180.0 / math.pi, "segments": segments}


def truth_segments(run_dir: Path) -> list[dict[str, object]]:
    """Measured per-segment swing peaks aligned to the route clock."""
    joints = load_joints(run_dir)
    start = route_start(run_dir)
    magnitude = np.sqrt(joints["roll"] ** 2 + joints["pitch"] ** 2)
    rel = joints["t"] - start
    segments = []
    t_cursor = 0.0
    for name, duration in ROUTE:
        mask = (rel >= t_cursor) & (rel < t_cursor + duration)
        values = magnitude[mask]
        segments.append({
            "name": name,
            "peak_deg": float(np.nanmax(values) * 180.0 / math.pi) if values.size else math.nan,
            "rms_deg": float(np.sqrt(np.nanmean(values ** 2)) * 180.0 / math.pi)
            if values.size else math.nan,
        })
        t_cursor += duration
    # True initial state at route entry.
    entry = int(np.nanargmin(np.abs(joints["t"] - start)))
    init = (float(joints["roll"][entry]), 0.0, float(joints["pitch"][entry]), 0.0)
    return segments, init


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--design", type=Path, required=True, help="FHv2 design JSON")
    parser.add_argument("--fh1-design", type=Path, required=True,
                        help="FHv1 finite_horizon_design.json (validation target)")
    parser.add_argument("--run", type=Path, action="append", required=True)
    parser.add_argument("--rope-length", type=float, action="append", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    fh2_items = {round(float(item["rope_length_m"]), 2): item
                 for item in json.loads(args.design.read_text(encoding="utf-8"))}
    fh1_items = {round(float(item["rope_length_m"]), 2): item
                 for item in json.loads(args.fh1_design.read_text(encoding="utf-8"))}

    all_results = []
    for run_dir, rope_length in zip(args.run, args.rope_length):
        run_dir = run_dir.resolve()
        L = round(rope_length, 2)
        truth, init = truth_segments(run_dir)
        fh1 = chained_prediction(
            int(fh1_items[L]["order"]), np.asarray(fh1_items[L]["coefficients"], dtype=float),
            rope_length, init)
        fh2 = chained_prediction(
            int(fh2_items[L]["order"]), np.asarray(fh2_items[L]["coefficients"], dtype=float),
            rope_length, init)
        # Validation of the chain on FHv1: per-segment peak error vs truth.
        validation = []
        for pred, meas in zip(fh1["segments"], truth):
            validation.append({
                "name": pred["name"],
                "truth_peak_deg": meas["peak_deg"],
                "fh1_pred_peak_deg": pred["peak_deg"],
                "peak_error_deg": pred["peak_deg"] - meas["peak_deg"],
            })
        all_results.append({
            "run": run_dir.name,
            "rope_length_m": rope_length,
            "validation": validation,
            "validation_peak_abs_error_max_deg": max(
                abs(v["peak_error_deg"]) for v in validation),
            "fh2_segments": fh2["segments"],
            "fh2_worst_peak_deg": max(s["peak_deg"] for s in fh2["segments"]),
            "fh2_worst_move_peak_deg": max(
                s["peak_deg"] for s in fh2["segments"] if s["name"] in DIRECTION_UNIT),
        })

    screen = {
        "results": all_results,
        "gates": {
            "fh2_predicted_peak_le_3p5_deg": all(
                r["fh2_worst_peak_deg"] <= SCREEN_PEAK_DEG for r in all_results),
            "fh2_reference_jerk_le_1p6_m_s3": all(
                float(fh2_items[round(L, 2)]["peak_jerk_m_s3"]) <= SCREEN_JERK_M_S3
                for L in args.rope_length),
        },
    }
    screen["screen_pass"] = all(screen["gates"].values())
    text = json.dumps(screen, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")

    for result in all_results:
        print(f"== {result['run']} (L={result['rope_length_m']} m) ==")
        print(f"{'segment':<8}{'truth_peak':>11}{'fh1_pred':>10}{'err':>8}"
              f"{'fh2_peak':>10}{'fh2_rms':>9}{'fh2_entry':>10}")
        truth_by_name = {v["name"]: v for v in result["validation"]}
        for seg in result["fh2_segments"]:
            v = truth_by_name[seg["name"]]
            print(f"{seg['name']:<8}{v['truth_peak_deg']:>11.3f}{v['fh1_pred_peak_deg']:>10.3f}"
                  f"{v['peak_error_deg']:>8.3f}{seg['peak_deg']:>10.3f}{seg['rms_deg']:>9.3f}"
                  f"{seg['entry_deg']:>10.3f}")
        print(f"validation max|err|={result['validation_peak_abs_error_max_deg']:.3f} deg, "
              f"FHv2 worst peak={result['fh2_worst_peak_deg']:.3f} deg")
    print(f"screen_pass={screen['screen_pass']} gates={screen['gates']}")
    return 0 if screen["screen_pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
