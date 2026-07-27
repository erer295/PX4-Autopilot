#!/usr/bin/env python3
"""Design/audit a fixed-horizon trajectory under suspended-load frequency uncertainty."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
from numpy.polynomial import Polynomial


G = 9.80665
ZETA = 0.10
DISTANCE_M = 2.0
DURATION_S = 6.0
RAMP_S = 1.0


def trajectory_progress(elapsed: np.ndarray, duration: float = DURATION_S) -> tuple[np.ndarray, ...]:
    t = np.clip(np.asarray(elapsed, dtype=float), 0.0, duration)
    peak = 1.0 / (duration - RAMP_S)
    position = np.zeros_like(t)
    velocity = np.zeros_like(t)
    acceleration = np.zeros_like(t)
    first = t < RAMP_S
    phase = math.pi * t[first] / RAMP_S
    position[first] = 0.5 * peak * (t[first] - RAMP_S * np.sin(phase) / math.pi)
    velocity[first] = 0.5 * peak * (1.0 - np.cos(phase))
    acceleration[first] = 0.5 * peak * math.pi * np.sin(phase) / RAMP_S
    cruise = (t >= RAMP_S) & (t <= duration - RAMP_S)
    position[cruise] = peak * (t[cruise] - 0.5 * RAMP_S)
    velocity[cruise] = peak
    last = (t > duration - RAMP_S) & (t < duration)
    decel = t[last] - (duration - RAMP_S)
    phase = math.pi * decel / RAMP_S
    position[last] = peak * (duration - 1.5 * RAMP_S) + 0.5 * peak * (
        decel + RAMP_S * np.sin(phase) / math.pi
    )
    velocity[last] = 0.5 * peak * (1.0 + np.cos(phase))
    acceleration[last] = -0.5 * peak * math.pi * np.sin(phase) / RAMP_S
    position[t >= duration] = 1.0
    return position, velocity, acceleration


def finite_horizon_basis(order: int, u: np.ndarray) -> tuple[np.ndarray, ...]:
    """Return endpoint-flat Bernstein-like position bases and derivatives in u."""
    envelope = Polynomial([0.0, 0.0, 0.0, 1.0]) * Polynomial([1.0, -3.0, 3.0, -1.0])
    position, velocity, acceleration = [], [], []
    for index in range(order):
        polynomial = envelope * Polynomial([0.0] * index + [1.0]) \
            * Polynomial([1.0, -1.0]) ** (order - 1 - index)
        position.append(polynomial(u))
        velocity.append(polynomial.deriv(1)(u))
        acceleration.append(polynomial.deriv(2)(u))
    return np.asarray(position).T, np.asarray(velocity).T, np.asarray(acceleration).T


def modal_state(acceleration: np.ndarray, dt: float, rope_length_m: float, ratio: float) -> complex:
    omega = math.sqrt(G / rope_length_m) * ratio
    omega_d = omega * math.sqrt(1.0 - ZETA * ZETA)
    angle = 0.0
    rate = 0.0
    for forcing in acceleration[:-1]:
        def derivative(theta: float, theta_rate: float) -> tuple[float, float]:
            return theta_rate, -2.0 * ZETA * omega * theta_rate - omega * omega * theta \
                - float(forcing) / rope_length_m
        k1 = derivative(angle, rate)
        k2 = derivative(angle + 0.5 * dt * k1[0], rate + 0.5 * dt * k1[1])
        k3 = derivative(angle + 0.5 * dt * k2[0], rate + 0.5 * dt * k2[1])
        k4 = derivative(angle + dt * k3[0], rate + dt * k3[1])
        angle += dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        rate += dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
    return complex(angle, rate / omega_d)


def design(rope_length_m: float, order: int, design_ratios: tuple[float, ...], regularization: float,
           residual_weight: float, acceleration_weight: float, jerk_weight: float,
           dt: float = 0.002) -> dict[str, object]:
    time = np.arange(0.0, DURATION_S + 0.5 * dt, dt)
    u = time / DURATION_S
    base_position, base_velocity, base_acceleration = trajectory_progress(time)
    basis_position, basis_velocity_u, basis_acceleration_u = finite_horizon_basis(order, u)
    basis_velocity = basis_velocity_u / DURATION_S
    basis_acceleration = basis_acceleration_u / (DURATION_S * DURATION_S)
    base_acceleration_m_s2 = DISTANCE_M * base_acceleration
    response_base = np.asarray([
        modal_state(base_acceleration_m_s2, dt, rope_length_m, ratio) for ratio in design_ratios
    ])
    response_basis = np.empty((len(design_ratios), order), dtype=complex)
    for ratio_index, ratio in enumerate(design_ratios):
        for basis_index in range(order):
            response_basis[ratio_index, basis_index] = modal_state(
                DISTANCE_M * basis_acceleration[:, basis_index], dt, rope_length_m, ratio
            )
    degree_scale = 180.0 / math.pi
    tracking = DISTANCE_M * basis_position / math.sqrt(time.size)
    acceleration_penalty = DISTANCE_M * basis_acceleration / math.sqrt(time.size)
    jerk_penalty = np.gradient(DISTANCE_M * basis_acceleration, dt, axis=0) / math.sqrt(time.size)
    modal_matrix = degree_scale * np.vstack((response_basis.real, response_basis.imag)) \
        / math.sqrt(2.0 * len(design_ratios))
    modal_target = -degree_scale * np.concatenate((response_base.real, response_base.imag)) \
        / math.sqrt(2.0 * len(design_ratios))
    matrix = np.vstack((
        tracking,
        math.sqrt(residual_weight) * modal_matrix,
        math.sqrt(acceleration_weight) * acceleration_penalty,
        math.sqrt(jerk_weight) * jerk_penalty,
        math.sqrt(regularization) * np.eye(order),
    ))
    target = np.concatenate((
        np.zeros(time.size),
        math.sqrt(residual_weight) * modal_target,
        np.zeros(time.size),
        np.zeros(time.size),
        np.zeros(order),
    ))
    coefficients = np.linalg.lstsq(matrix, target, rcond=None)[0]
    position = base_position + basis_position @ coefficients
    velocity = base_velocity + basis_velocity @ coefficients
    acceleration = base_acceleration + basis_acceleration @ coefficients
    jerk = np.gradient(DISTANCE_M * acceleration, dt)
    offset = DISTANCE_M * (position - base_position)
    audit_ratios = np.linspace(0.90, 1.20, 121)
    residual_deg = np.asarray([
        abs(modal_state(DISTANCE_M * acceleration, dt, rope_length_m, ratio)) * 180.0 / math.pi
        for ratio in audit_ratios
    ])
    return {
        "rope_length_m": rope_length_m,
        "order": order,
        "design_ratios": list(design_ratios),
        "regularization": regularization,
        "residual_weight": residual_weight,
        "acceleration_weight": acceleration_weight,
        "jerk_weight": jerk_weight,
        "coefficients": coefficients.tolist(),
        "position_min": float(np.min(position)),
        "position_max": float(np.max(position)),
        "command_offset_rms_m": float(np.sqrt(np.mean(offset * offset))),
        "command_offset_peak_m": float(np.max(np.abs(offset))),
        "peak_velocity_m_s": float(np.max(np.abs(DISTANCE_M * velocity))),
        "peak_acceleration_m_s2": float(np.max(np.abs(DISTANCE_M * acceleration))),
        "peak_jerk_m_s3": float(np.max(np.abs(jerk))),
        "worst_terminal_modal_amplitude_deg": float(np.max(residual_deg)),
        "worst_terminal_modal_frequency_ratio": float(audit_ratios[int(np.argmax(residual_deg))]),
        "nominal_terminal_modal_amplitude_deg": float(residual_deg[40]),
        "lower_terminal_modal_amplitude_deg": float(residual_deg[0]),
        "upper_terminal_modal_amplitude_deg": float(residual_deg[-1]),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rope-length", type=float, action="append", default=[])
    parser.add_argument("--order", type=int, default=12)
    parser.add_argument("--ratios", default="0.90,1.00,1.10,1.20")
    parser.add_argument("--regularization", type=float, default=1.0e-8)
    parser.add_argument("--residual-weight", type=float, default=1.0)
    parser.add_argument("--acceleration-weight", type=float, default=0.01)
    parser.add_argument("--jerk-weight", type=float, default=0.001)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    lengths = args.rope_length or [0.6, 0.8]
    ratios = tuple(float(value) for value in args.ratios.split(","))
    result = [design(length, args.order, ratios, args.regularization, args.residual_weight,
                     args.acceleration_weight, args.jerk_weight) for length in lengths]
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.write_text(text, encoding="utf-8")
    print(text, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
