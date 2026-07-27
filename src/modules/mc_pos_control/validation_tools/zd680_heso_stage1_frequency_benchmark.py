#!/usr/bin/env python3
"""Benchmark suspended-load effective-frequency estimators for stage 1."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import deque
from pathlib import Path

import numpy as np
from pyulog import ULog


G = 9.80665
ROPE_LENGTH = 0.6
NOMINAL_OMEGA = math.sqrt(G / ROPE_LENGTH)
FREQUENCY_RATIOS = np.asarray([0.70, 0.80, 0.90, 1.00, 1.10, 1.20, 1.30])
REPO_ROOT = Path(__file__).resolve().parents[4]
DEFAULT_DATA_ROOT = Path(__file__).resolve().parents[1] / "validation" / "heso_gain_schedule_validation_20260722" / "l06" / "runs"
DEFAULT_OUTPUT = Path(__file__).resolve().parents[1] / "validation" / "heso_stage12_validation_20260722" / "stage1"


def smooth_step(value, lower, upper):
    value = np.asarray(value, dtype=float)
    if upper <= lower:
        return np.ones_like(value)
    x = np.clip((value - lower) / (upper - lower), 0.0, 1.0)
    return x * x * (3.0 - 2.0 * x)


def scalar_smooth_step(value: float, lower: float, upper: float) -> float:
    return float(smooth_step(np.asarray([value]), lower, upper)[0])


def lowpass(previous, value, dt, time_constant):
    alpha = math.exp(-dt / time_constant)
    return alpha * previous + (1.0 - alpha) * value


def interpolated_ratio(scores):
    best = int(np.argmin(scores))
    offset = 0.0
    if 0 < best < len(scores) - 1:
        left, center, right = scores[best - 1], scores[best], scores[best + 1]
        curvature = left - 2.0 * center + right
        if curvature > 1.0e-12:
            offset = float(np.clip(0.5 * (left - right) / curvature, -0.5, 0.5))
    return float(np.clip(FREQUENCY_RATIOS[best] + 0.1 * offset, 0.7, 1.3)), best


class LegacyHesoBank:
    def __init__(self, dt, nominal_omega=NOMINAL_OMEGA, bandwidth_hz=0.30):
        self.dt = dt
        self.wn = nominal_omega
        self.wo = 2.0 * math.pi * bandwidth_hz
        self.angle = np.zeros((7, 2))
        self.rate = np.zeros((7, 2))
        self.disturbance = np.zeros((7, 2))
        self.score = np.full(7, 1.0e-4)
        self.ratio = 1.0
        self.variance = 0.0
        self.confidence = 0.0
        self.initialized = False
        self.elapsed = 0.0

    def update(self, angle, rate, forcing, energy):
        if not self.initialized:
            self.angle[:] = angle
            self.rate[:] = rate
            self.initialized = True
        l1 = 3.0 * self.wo
        l3 = self.wo ** 3
        alpha = math.exp(-self.dt / 3.0)
        for index, ratio in enumerate(FREQUENCY_RATIOS):
            candidate = self.wn * ratio
            l2 = 3.0 * self.wo ** 2 - candidate ** 2
            innovation = angle - self.angle[index]
            previous_angle = self.angle[index].copy()
            previous_rate = self.rate[index].copy()
            self.angle[index] += self.dt * (previous_rate + l1 * innovation)
            self.rate[index] += self.dt * (-candidate ** 2 * previous_angle + forcing
                                           + self.disturbance[index] + l2 * innovation)
            self.disturbance[index] += self.dt * l3 * innovation
            angle_residual = angle - self.angle[index]
            rate_residual = rate - self.rate[index]
            sample = float(angle_residual @ angle_residual
                           + rate_residual @ rate_residual / self.wn ** 2)
            self.score[index] = alpha * self.score[index] + (1.0 - alpha) * sample
        instantaneous, _ = interpolated_ratio(self.score)
        residual = instantaneous - self.ratio
        self.ratio = lowpass(self.ratio, instantaneous, self.dt, 1.5)
        self.variance = lowpass(self.variance, residual * residual, self.dt, 1.5)
        self.elapsed += self.dt
        contrast = np.clip((np.max(self.score) - np.min(self.score))
                           / (np.max(self.score) + np.min(self.score) + 1.0e-12), 0.0, 1.0)
        stability = 1.0 - scalar_smooth_step(math.sqrt(max(self.variance, 0.0)), 0.03, 0.15)
        signal = max(scalar_smooth_step(energy, 0.003, 0.009),
                     scalar_smooth_step(float(np.linalg.norm(rate)), 0.015, 0.045))
        settled = scalar_smooth_step(self.elapsed, 2.0, 2.5)
        target = math.sqrt(contrast) * stability * signal * settled
        self.confidence = lowpass(self.confidence, target, self.dt, 1.0)
        return self.ratio, self.confidence


class PredictiveBank:
    """Candidate bank scored on correction-free one-step physical prediction."""

    def __init__(self, dt, nominal_omega=NOMINAL_OMEGA):
        self.dt = dt
        self.wn = nominal_omega
        self.score = np.full(7, 1.0e-5)
        self.ratio = 1.0
        self.variance = 0.0
        self.confidence = 0.0
        self.previous_angle = np.zeros(2)
        self.previous_rate = np.zeros(2)
        self.previous_forcing = np.zeros(2)
        self.initialized = False
        self.elapsed = 0.0

    def update(self, angle, rate, forcing, energy):
        if not self.initialized:
            self.previous_angle = angle.copy()
            self.previous_rate = rate.copy()
            self.previous_forcing = forcing.copy()
            self.initialized = True
            return self.ratio, self.confidence
        alpha = math.exp(-self.dt / 3.0)
        for index, ratio in enumerate(FREQUENCY_RATIOS):
            candidate_squared = (self.wn * ratio) ** 2
            acceleration = -candidate_squared * self.previous_angle + self.previous_forcing
            predicted_angle = self.previous_angle + self.dt * self.previous_rate + 0.5 * self.dt ** 2 * acceleration
            predicted_rate = self.previous_rate + self.dt * acceleration
            angle_residual = angle - predicted_angle
            rate_residual = rate - predicted_rate
            sample = float(angle_residual @ angle_residual
                           + rate_residual @ rate_residual / self.wn ** 2)
            self.score[index] = alpha * self.score[index] + (1.0 - alpha) * sample
        instantaneous, best = interpolated_ratio(self.score)
        residual = instantaneous - self.ratio
        self.ratio = lowpass(self.ratio, instantaneous, self.dt, 1.5)
        self.variance = lowpass(self.variance, residual * residual, self.dt, 1.5)
        sorted_scores = np.sort(self.score)
        separation = np.clip((sorted_scores[1] - sorted_scores[0])
                             / (sorted_scores[1] + sorted_scores[0] + 1.0e-12), 0.0, 1.0)
        curvature = 0.0
        if 0 < best < 6:
            curvature = max(self.score[best - 1] - 2.0 * self.score[best] + self.score[best + 1], 0.0)
        curvature_scale = np.clip(curvature / (np.median(self.score) + 1.0e-12), 0.0, 1.0)
        stability = 1.0 - scalar_smooth_step(math.sqrt(max(self.variance, 0.0)), 0.03, 0.12)
        signal = max(scalar_smooth_step(energy, 0.003, 0.009),
                     scalar_smooth_step(float(np.linalg.norm(rate)), 0.015, 0.045))
        self.elapsed += self.dt
        settled = scalar_smooth_step(self.elapsed, 2.0, 2.5)
        target = math.sqrt(separation) * math.sqrt(curvature_scale) * stability * signal * settled
        self.confidence = lowpass(self.confidence, target, self.dt, 1.0)
        self.previous_angle = angle.copy()
        self.previous_rate = rate.copy()
        self.previous_forcing = forcing.copy()
        return self.ratio, self.confidence


class FilteredRls:
    """Window-integral two-parameter RLS for omega^2 and viscous damping."""

    def __init__(self, dt, nominal_omega=NOMINAL_OMEGA, window_s=0.40):
        self.dt = dt
        self.wn = nominal_omega
        self.window = max(int(round(window_s / dt)), 5)
        self.angles = deque(maxlen=self.window + 1)
        self.rates = deque(maxlen=self.window + 1)
        self.forcings = deque(maxlen=self.window + 1)
        self.theta = np.asarray([nominal_omega ** 2, 0.10 * nominal_omega])
        self.covariance = np.diag([30.0, 10.0])
        self.forgetting = math.exp(-dt / 15.0)
        self.ratio = 1.0
        self.variance = 0.0
        self.confidence = 0.0
        self.information = 0.0
        self.residual_variance = 1.0e-4
        self.elapsed = 0.0

    def update(self, angle, rate, forcing, energy):
        self.angles.append(angle.copy())
        self.rates.append(rate.copy())
        self.forcings.append(forcing.copy())
        self.elapsed += self.dt
        if len(self.angles) <= self.window:
            return self.ratio, self.confidence
        angles = np.asarray(self.angles)
        rates = np.asarray(self.rates)
        forcings = np.asarray(self.forcings)
        angle_integral = np.trapezoid(angles, dx=self.dt, axis=0)
        rate_integral = np.trapezoid(rates, dx=self.dt, axis=0)
        forcing_integral = np.trapezoid(forcings, dx=self.dt, axis=0)
        response = rates[-1] - rates[0] - forcing_integral
        residual_sum = 0.0
        excitation_sum = 0.0
        for axis in range(2):
            phi = np.asarray([-angle_integral[axis], -rate_integral[axis]])
            excitation = float(phi @ phi)
            if excitation < 1.0e-10:
                continue
            p_phi = self.covariance @ phi
            denominator = self.forgetting + float(phi @ p_phi)
            gain = p_phi / denominator
            innovation = float(response[axis] - phi @ self.theta)
            self.theta += gain * innovation
            self.covariance = (self.covariance - np.outer(gain, phi) @ self.covariance) / self.forgetting
            residual_sum += innovation * innovation
            excitation_sum += excitation
        minimum_q = (0.70 * self.wn) ** 2
        maximum_q = (1.30 * self.wn) ** 2
        self.theta[0] = float(np.clip(self.theta[0], minimum_q, maximum_q))
        self.theta[1] = float(np.clip(self.theta[1], 0.0, 1.5 * self.wn))
        raw_ratio = math.sqrt(self.theta[0]) / self.wn
        frequency_residual = raw_ratio - self.ratio
        self.ratio = lowpass(self.ratio, raw_ratio, self.dt, 1.5)
        self.variance = lowpass(self.variance, frequency_residual ** 2, self.dt, 1.5)
        self.information = lowpass(self.information, excitation_sum / max(self.window * self.dt, 1.0e-6), self.dt, 2.0)
        self.residual_variance = lowpass(self.residual_variance, residual_sum, self.dt, 2.0)
        information_gate = scalar_smooth_step(self.information, 1.0e-5, 8.0e-4)
        stability = 1.0 - scalar_smooth_step(math.sqrt(max(self.variance, 0.0)), 0.02, 0.08)
        residual_gate = 1.0 - scalar_smooth_step(self.residual_variance, 2.0e-4, 4.0e-3)
        signal = max(scalar_smooth_step(energy, 0.003, 0.009),
                     scalar_smooth_step(float(np.linalg.norm(rate)), 0.015, 0.045))
        target = information_gate * stability * residual_gate * signal
        self.confidence = lowpass(self.confidence, target, self.dt, 1.0)
        return self.ratio, self.confidence


def run_estimators(t, angle, rate, forcing, rope_length=ROPE_LENGTH):
    dt = float(np.median(np.diff(t)))
    nominal_omega = math.sqrt(G / rope_length)
    estimators = {
        "legacy_heso_bank": LegacyHesoBank(dt, nominal_omega),
        "predictive_bank": PredictiveBank(dt, nominal_omega),
        "filtered_rls": FilteredRls(dt, nominal_omega),
    }
    output = {name: {"ratio": np.ones(len(t)), "confidence": np.zeros(len(t))} for name in estimators}
    energy = 0.5 * (rope_length * np.linalg.norm(rate, axis=1)) ** 2 + G * rope_length * (
        1.0 - np.cos(np.linalg.norm(angle, axis=1)))
    for sample in range(len(t)):
        for name, estimator in estimators.items():
            ratio, confidence = estimator.update(angle[sample], rate[sample], forcing[sample], energy[sample])
            output[name]["ratio"][sample] = ratio
            output[name]["confidence"][sample] = confidence
    output["energy"] = energy
    return output


def lowpass_measurements(values, dt, cutoff_hz=4.0):
    output = np.empty_like(values)
    output[0] = values[0]
    alpha = math.exp(-2.0 * math.pi * cutoff_hz * dt)
    for index in range(1, len(values)):
        output[index] = alpha * output[index - 1] + (1.0 - alpha) * values[index]
    return output


def synthetic_case(true_ratio, noise_scale, seed, duration=60.0, dt=0.01, low_excitation=False):
    rng = np.random.default_rng(seed)
    t = np.arange(0.0, duration, dt)
    omega = NOMINAL_OMEGA * true_ratio
    damping_ratio = 0.04 + 0.025 * ((seed % 3) / 2.0)
    angle = np.zeros((len(t), 2))
    rate = np.zeros((len(t), 2))
    angle[0] = np.radians([0.05, -0.04] if low_excitation else [3.0, -2.0])
    forcing = np.zeros_like(angle)
    if not low_excitation:
        forcing[:, 0] = 0.32 * np.sin(2.0 * math.pi * 0.17 * t) + 0.09 * np.sin(2.0 * math.pi * 0.41 * t + 0.3)
        forcing[:, 1] = 0.27 * np.sin(2.0 * math.pi * 0.13 * t + 0.7) + 0.07 * np.sin(2.0 * math.pi * 0.37 * t)
    disturbance = np.column_stack([
        0.025 * np.sin(2.0 * math.pi * 0.035 * t + 0.2),
        0.020 * np.sin(2.0 * math.pi * 0.045 * t + 0.8),
    ]) if not low_excitation else np.zeros_like(angle)
    for index in range(1, len(t)):
        acceleration = (-omega ** 2 * angle[index - 1]
                        - 2.0 * damping_ratio * omega * rate[index - 1]
                        + forcing[index - 1] + disturbance[index - 1])
        rate[index] = rate[index - 1] + dt * acceleration
        angle[index] = angle[index - 1] + dt * rate[index]
    angle_noise = math.radians(0.02 if noise_scale == "low" else 0.08)
    rate_noise = 0.0015 if noise_scale == "low" else 0.006
    measured_angle = angle + rng.normal(0.0, angle_noise, angle.shape)
    measured_rate = rate + rng.normal(0.0, rate_noise, rate.shape)
    measured_angle = lowpass_measurements(measured_angle, dt)
    measured_rate = lowpass_measurements(measured_rate, dt)
    return t, measured_angle, measured_rate, forcing


def debug_dataset(ulog, debug_id):
    datasets = []
    for dataset in ulog.data_list:
        if dataset.name == "debug_array":
            data = dataset.data
            if "id" in data and np.any(np.asarray(data["id"]) == debug_id):
                mask = np.asarray(data["id"]) == debug_id
                datasets.append({key: np.asarray(value)[mask] for key, value in data.items()})
    if not datasets:
        return None
    return max(datasets, key=lambda item: len(item["timestamp"]))


def read_route_bounds(path):
    with (path / "events.csv").open(newline="", encoding="utf-8") as stream:
        events = list(csv.DictReader(stream))
    actions = [row for row in events if row["name"].startswith("action_")]
    return min(float(row["start_boot_s"]) for row in actions), max(float(row["end_boot_s"]) for row in actions)


def quaternion_yaw(attitude):
    q0 = np.asarray(attitude["q[0]"], dtype=float)
    q1 = np.asarray(attitude["q[1]"], dtype=float)
    q2 = np.asarray(attitude["q[2]"], dtype=float)
    q3 = np.asarray(attitude["q[3]"], dtype=float)
    return np.unwrap(np.arctan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)))


def load_flight(path, rope_length=ROPE_LENGTH):
    start, end = read_route_bounds(path)
    ulog = ULog(str(path / "position_offboard.ulg"), ["debug_array", "vehicle_attitude"])
    anti_swing = debug_dataset(ulog, 681)
    fso = debug_dataset(ulog, 687)
    attitude = ulog.get_dataset("vehicle_attitude").data
    if anti_swing is None or fso is None:
        raise RuntimeError(f"missing debug 681/687 in {path}")
    t_all = np.asarray(anti_swing["timestamp"], dtype=float) * 1.0e-6
    mask = (t_all >= start) & (t_all <= end)
    t = t_all[mask]
    angle = np.column_stack([anti_swing["data[0]"][mask], anti_swing["data[1]"][mask]])
    rate = np.column_stack([anti_swing["data[2]"][mask], anti_swing["data[3]"][mask]])
    fso_t = np.asarray(fso["timestamp"], dtype=float) * 1.0e-6
    acceleration_ned = np.column_stack([
        np.interp(t, fso_t, fso["data[2]"]),
        np.interp(t, fso_t, fso["data[3]"]),
    ])
    attitude_t = np.asarray(attitude["timestamp"], dtype=float) * 1.0e-6
    yaw = np.interp(t, attitude_t, quaternion_yaw(attitude))
    cosine, sine = np.cos(yaw), np.sin(yaw)
    acceleration_heading = np.column_stack([
        cosine * acceleration_ned[:, 0] + sine * acceleration_ned[:, 1],
        -sine * acceleration_ned[:, 0] + cosine * acceleration_ned[:, 1],
    ])
    forcing = -acceleration_heading / rope_length
    return t, angle, rate, forcing


def fft_peak_ratio(t, angle, nominal_omega=NOMINAL_OMEGA):
    dt = float(np.median(np.diff(t)))
    window = np.hanning(len(t))
    centered = (angle - np.mean(angle, axis=0)) * window[:, None]
    frequencies = np.fft.rfftfreq(len(t), dt)
    spectrum = np.sum(np.abs(np.fft.rfft(centered, axis=0)) ** 2, axis=1)
    nominal_hz = nominal_omega / (2.0 * math.pi)
    mask = (frequencies >= 0.65 * nominal_hz) & (frequencies <= 1.35 * nominal_hz)
    return float(frequencies[mask][np.argmax(spectrum[mask])] / nominal_hz)


def metrics(ratio, confidence, true_ratio, time_mask):
    error = np.abs(ratio[time_mask] / true_ratio - 1.0) * 100.0
    if error.size == 0:
        raise ValueError("frequency benchmark metric mask selected no samples")
    high_confidence = confidence[time_mask] > 0.7
    return {
        "median_abs_error_pct": float(np.median(error)),
        "p95_abs_error_pct": float(np.percentile(error, 95)),
        "mean_ratio": float(np.mean(ratio[time_mask])),
        "median_ratio": float(np.median(ratio[time_mask])),
        "confidence_median": float(np.median(confidence[time_mask])),
        "confidence_max": float(np.max(confidence[time_mask])),
        "high_confidence_ratio": float(np.mean(high_confidence)),
        "high_confidence_error_gt5_ratio": float(np.mean(error[high_confidence] > 5.0)) if np.any(high_confidence) else math.nan,
    }


def write_csv(path, rows):
    fields = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    synthetic_rows = []
    aggregate_samples = {name: [] for name in ("legacy_heso_bank", "predictive_bank", "filtered_rls")}
    for ratio_index, true_ratio in enumerate((0.75, 0.85, 0.95, 1.00, 1.05, 1.15, 1.25)):
        for noise_index, noise in enumerate(("low", "medium")):
            seed = 1722 + 10 * ratio_index + noise_index
            t, angle, rate, forcing = synthetic_case(true_ratio, noise, seed)
            result = run_estimators(t, angle, rate, forcing)
            # The synthetic cases remain persistently excited by a known base
            # acceleration after their free response decays. Energy-only
            # selection would incorrectly discard the higher-frequency forced
            # steady states; low/no-excitation is audited in a separate case.
            mask = t >= 15.0
            for estimator in aggregate_samples:
                row = {
                    "dataset": "synthetic",
                    "true_ratio": true_ratio,
                    "noise": noise,
                    "seed": seed,
                    "estimator": estimator,
                    **metrics(result[estimator]["ratio"], result[estimator]["confidence"], true_ratio, mask),
                }
                synthetic_rows.append(row)
                aggregate_samples[estimator].extend(
                    np.abs(result[estimator]["ratio"][mask] / true_ratio - 1.0).tolist())

    low_excitation_rows = []
    t, angle, rate, forcing = synthetic_case(1.0, "medium", 9917, low_excitation=True)
    low_result = run_estimators(t, angle, rate, forcing)
    mask = t >= 15.0
    for estimator in aggregate_samples:
        low_excitation_rows.append({
            "dataset": "synthetic_low_excitation",
            "estimator": estimator,
            "confidence_median": float(np.median(low_result[estimator]["confidence"][mask])),
            "confidence_max": float(np.max(low_result[estimator]["confidence"][mask])),
            "high_confidence_ratio": float(np.mean(low_result[estimator]["confidence"][mask] > 0.7)),
            "energy_max_j_kg": float(np.max(low_result["energy"][mask])),
        })

    flight_rows = []
    for mode_path in sorted(path for path in args.data_root.iterdir() if path.is_dir()):
        for run_path in sorted(path for path in mode_path.glob("*/*") if (path / "metadata.json").is_file()):
            t, angle, rate, forcing = load_flight(run_path)
            result = run_estimators(t, angle, rate, forcing)
            reference = fft_peak_ratio(t, angle)
            mask = (t >= t[0] + 10.0) & (result["energy"] >= 0.003)
            for estimator in aggregate_samples:
                row = {
                    "dataset": "flight",
                    "mode": mode_path.name,
                    "run": run_path.name,
                    "fft_peak_ratio": reference,
                    "estimator": estimator,
                    **metrics(result[estimator]["ratio"], result[estimator]["confidence"], reference, mask),
                }
                row["median_difference_from_nominal_pct"] = abs(row["median_ratio"] - 1.0) * 100.0
                flight_rows.append(row)

    aggregate_rows = []
    for estimator, samples in aggregate_samples.items():
        errors = np.asarray(samples) * 100.0
        matching_flights = [row for row in flight_rows if row["estimator"] == estimator]
        low = next(row for row in low_excitation_rows if row["estimator"] == estimator)
        aggregate_rows.append({
            "estimator": estimator,
            "synthetic_median_abs_error_pct": float(np.median(errors)),
            "synthetic_p95_abs_error_pct": float(np.percentile(errors, 95)),
            "flight_median_abs_error_vs_fft_pct": float(np.median([row["median_abs_error_pct"] for row in matching_flights])),
            "flight_max_abs_error_vs_fft_pct": float(np.max([row["median_abs_error_pct"] for row in matching_flights])),
            "low_excitation_confidence_max": low["confidence_max"],
            "synthetic_median_target_pass": bool(np.median(errors) <= 3.0),
            "synthetic_p95_target_pass": bool(np.percentile(errors, 95) <= 5.0),
            "flight_fft_target_pass": bool(np.median([row["median_abs_error_pct"] for row in matching_flights]) <= 5.0),
            "low_excitation_target_pass": bool(low["high_confidence_ratio"] == 0.0),
        })
    eligible = [row for row in aggregate_rows if row["synthetic_median_target_pass"]
                and row["low_excitation_target_pass"]]
    ranking_pool = eligible if eligible else aggregate_rows
    winner = min(ranking_pool, key=lambda row: (
        row["synthetic_median_abs_error_pct"], row["synthetic_p95_abs_error_pct"],
        row["flight_median_abs_error_vs_fft_pct"]))
    decision = {
        "winner": winner["estimator"],
        "winner_meets_all_targets": bool(winner["synthetic_median_target_pass"]
                                          and winner["synthetic_p95_target_pass"]
                                          and winner["flight_fft_target_pass"]
                                          and winner["low_excitation_target_pass"]),
        "aggregate": aggregate_rows,
        "selection_rule": "eligible minimum synthetic median error, then P95, then flight FFT cross-check",
    }
    write_csv(args.output_dir / "synthetic_cases.csv", synthetic_rows)
    write_csv(args.output_dir / "low_excitation.csv", low_excitation_rows)
    write_csv(args.output_dir / "flight_crosscheck.csv", flight_rows)
    write_csv(args.output_dir / "estimator_summary.csv", aggregate_rows)
    (args.output_dir / "stage1_decision.json").write_text(
        json.dumps(decision, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(decision, indent=2, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
