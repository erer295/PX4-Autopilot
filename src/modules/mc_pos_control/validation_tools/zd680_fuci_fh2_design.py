#!/usr/bin/env python3
"""Design/audit the FHv2 fixed-horizon trajectory with full-horizon convex constraints.

FHv1 failed flight gates because the offline objective only suppressed the
six-second terminal linear mode: mid-window swing peaks and reference jerk were
unconstrained.  FHv2 keeps the same endpoint-flat basis and the same frequency
ratio interval, but solves a convex quadratic program with hard constraints on

- the full-horizon swing angle |theta(t)| over rho in [0.90, 1.20], both from
  zero initial state and from corner initial states observed in the failed run;
- reference jerk (hard cap so the closed-loop jerk P95 keeps margin to 2.0);
- velocity / acceleration / position overshoot / coefficient magnitude.

The online frequency branch is explicitly out of scope: the design uses the
offline bounded ratio interval only (safe-degradation architecture).

Only NumPy is available, so the QP is solved with a small ADMM solver that
exploits the shared row structure of the swing constraints (the same basis
response matrix appears once per initial-condition corner; only the offset
changes).
"""

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
ORDER = 16
RATIO_MIN = 0.90
RATIO_MAX = 1.20
DESIGN_RATIOS = (0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20)

# Corner initial states taken from the failed FHv1 direction audit: entering a
# move with up to ~0.5 deg residual angle and ~0.02 rad/s residual rate covers
# every observed segment entry of the four-direction route.
INIT_ANGLE_DEG = 0.5
INIT_RATE_RAD_S = 0.02

# Hard design bounds (the offline audit gates live in offline_gates() below).
BOUNDS = {
    # Frozen 2026-07-23 after the feasibility map: order-16 basis, swing
    # (2.7/3.7 deg), terminal box 0.80 deg, reference jerk 1.59 (audit grid
    # shows <=1.60), offset peak 0.15 m.  Offset RMS ~0.09 is the Pareto
    # floor under these safety bounds; the 0.075 gate from FHv1 is not
    # jointly feasible and is re-justified in the FHv2 preregistration.
    "swing_zero_init_deg": 2.7,
    "swing_init_deg": 3.7,
    "jerk_m_s3": 1.59,
    "velocity_min_m_s": -0.01,
    "velocity_max_m_s": 0.54,
    "acceleration_m_s2": 0.65,
    "progress_min": -0.0025,
    "progress_max": 1.0025,
    "coefficient_m": 0.20,
    # Terminal modal |Re|/|Im| bound per design ratio (deg); the amplitude is
    # then bounded by sqrt(2) times this value.  Convex box instead of the
    # nonlinear amplitude ball.
    "terminal_component_deg": 0.80,
    # Pointwise cap on the command offset |s(u)-s0(u)|*DISTANCE (m); keeps the
    # offset RMS small without a quadratic constraint.
    "offset_peak_m": 0.15,
}

# Frozen per-length swing bounds (2026-07-23): identical design rules for
# every length; only the swing bounds adapt to feasibility.  The slower 0.8 m
# pendulum cannot meet (2.7, 3.7) jointly with the other frozen bounds.
PER_LENGTH_SWING_BOUNDS = {
    # Amendment v2 (2026-07-23, before the FH2_06 rerun): 0.6 m moved to
    # (3.0, 4.0) after the first FH2_06 flight failed only the XY gate
    # (0.1282 > 0.12); (2.7, 3.7) pinned the offset RMS at 0.090.
    # Amendment v3 (2026-07-23, before any L05 flight): 0.5 m moved to
    # (3.0, 4.0) as well; the (2.7, 3.7) offset floor (0.0849) mapped to a
    # borderline flight XY, same failure mode as the 0.6 m v1 gate run.
    # Amendment v5 (2026-07-24, before the L08 v5 reruns): 0.8 m moved to
    # (3.5, 4.5) after 2/5 L08 flights failed only the XY gate (0.124/0.121);
    # offset RMS drops 0.0880 -> 0.0699, chained worst peak 2.909 deg.
    0.5: (3.0, 4.0),
    0.6: (3.0, 4.0),
    0.8: (3.5, 4.5),
}

DT_SIM = 0.002
CONSTRAINT_STRIDE = 5  # constraint grid dt = 0.01 s
JERK_CONSTRAINT_STRIDE = 2  # jerk grid dt = 0.004 s


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
    """Return endpoint-flat position bases and derivatives in u."""
    envelope = Polynomial([0.0, 0.0, 0.0, 1.0]) * Polynomial([1.0, -3.0, 3.0, -1.0])
    position, velocity, acceleration = [], [], []
    for index in range(order):
        polynomial = envelope * Polynomial([0.0] * index + [1.0]) \
            * Polynomial([1.0, -1.0]) ** (order - 1 - index)
        position.append(polynomial(u))
        velocity.append(polynomial.deriv(1)(u))
        acceleration.append(polynomial.deriv(2)(u))
    return np.asarray(position).T, np.asarray(velocity).T, np.asarray(acceleration).T


def simulate_pendulum(acceleration: np.ndarray, dt: float, rope_length_m: float,
                      ratio, theta0=0.0, rate0=0.0) -> tuple[np.ndarray, np.ndarray]:
    """Forced linear suspended-load response; theta in rad, positive against accel.

    `acceleration` is (T,) or (T, M); `ratio`, `theta0` and `rate0` are scalars
    or length-M arrays.  Returns (angles, rates) with the same leading shape.
    """
    forcing_all = np.atleast_2d(acceleration.T).T  # (T, M)
    columns = forcing_all.shape[1]
    omega = np.broadcast_to(np.asarray(ratio, dtype=float), (columns,)) \
        * math.sqrt(G / rope_length_m)
    theta = np.broadcast_to(np.asarray(theta0, dtype=float), (columns,)).astype(float).copy()
    rate = np.broadcast_to(np.asarray(rate0, dtype=float), (columns,)).astype(float).copy()
    angles = np.empty_like(forcing_all)
    rates = np.empty_like(forcing_all)
    angles[0] = theta
    rates[0] = rate

    def derivative(angle, angle_rate, forcing):
        return angle_rate, -2.0 * ZETA * omega * angle_rate - omega * omega * angle \
            - forcing / rope_length_m

    for index in range(forcing_all.shape[0] - 1):
        forcing = forcing_all[index]
        k1 = derivative(theta, rate, forcing)
        k2 = derivative(theta + 0.5 * dt * k1[0], rate + 0.5 * dt * k1[1], forcing)
        k3 = derivative(theta + 0.5 * dt * k2[0], rate + 0.5 * dt * k2[1], forcing)
        k4 = derivative(theta + dt * k3[0], rate + dt * k3[1], forcing)
        theta += dt * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]) / 6.0
        rate += dt * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]) / 6.0
        angles[index + 1] = theta
        rates[index + 1] = rate
    if np.ndim(acceleration) == 1:
        return angles[:, 0], rates[:, 0]
    return angles, rates


def terminal_modal_batch(acceleration: np.ndarray, dt: float, rope_length_m: float,
                         ratio) -> np.ndarray:
    """Complex terminal modal amplitude per column of the (T, M) acceleration."""
    ratio_array = np.atleast_1d(np.asarray(ratio, dtype=float))
    omega = ratio_array * math.sqrt(G / rope_length_m)
    omega_d = omega * math.sqrt(1.0 - ZETA * ZETA)
    angles, rates = simulate_pendulum(acceleration, dt, rope_length_m, ratio_array)
    return angles[-1] + 1j * (rates[-1] / omega_d)


def solve_qp(matrix: np.ndarray, target: np.ndarray,
             row_blocks: list[tuple[np.ndarray, np.ndarray, float]],
             penalty: float = 1.0, iterations: int = 20000,
             tolerance: float = 1.0e-9) -> tuple[np.ndarray, float, int]:
    """Solve min ||A c - b||^2 s.t. G_i c <= h_i[copy, :] for every block.

    Each block is (rows, h, scale) with h shaped (copies, n_rows).  Constraint
    generation with OSQP as the exact sub-solver: solve a small active subset,
    dense-check the full ~50k-row set, add the most violated rows, repeat.
    Exact at convergence and avoids both the dense 50k-constraint solve and
    the slow tail of plain ADMM.  Falls back to scaled ADMM when neither
    osqp nor scipy is available.
    """
    try:
        import osqp
        import scipy.sparse as sparse
    except ImportError:
        return admm_constrained_lstsq(matrix, target, row_blocks, penalty,
                                      iterations, tolerance)

    rows_all = []
    h_all = []
    for rows, h, scale in row_blocks:
        for copy in range(h.shape[0]):
            rows_all.append(rows / scale)
            h_all.append(h[copy] / scale)
    constraint_matrix = np.vstack(rows_all)
    constraint_bound = np.concatenate(h_all)
    gram = matrix.T @ matrix
    rhs = matrix.T @ target
    objective_scale = max(1.0, float(np.max(np.abs(np.diag(gram)))))
    hessian = sparse.csc_matrix((gram + 1.0e-12 * np.eye(gram.shape[0])) / objective_scale)
    gradient = -rhs / objective_scale

    def osqp_solve(active: list[int], warm=None):
        sub = sparse.csc_matrix(constraint_matrix[active])
        solver = osqp.OSQP()
        solver.setup(P=hessian, q=gradient, A=sub,
                     l=-np.inf * np.ones(len(active)), u=constraint_bound[active],
                     eps_abs=1.0e-10, eps_rel=1.0e-10, max_iter=100000,
                     polish=True, verbose=False)
        result = solver.solve()
        if result.x is None or "infeasible" in result.info.status:
            return None
        return result.x

    # Constraint generation: exact OSQP on a small active subset, dense check
    # of the full set, add the most violated rows, repeat.
    coefficients = np.zeros(matrix.shape[1])
    slack_all = -(constraint_bound)  # slack of c=0
    active = sorted(int(i) for i in np.argsort(slack_all)[:200])
    for _ in range(40):
        coefficients = osqp_solve(active, coefficients)
        if coefficients is None:
            return None, math.inf, -1
        slack_all = constraint_matrix @ coefficients - constraint_bound
        max_violation = float(np.max(slack_all))
        if max_violation <= tolerance:
            break
        active_set = set(active)
        added = 0
        for index in np.argsort(-slack_all):
            index = int(index)
            if slack_all[index] <= max(tolerance, 0.02 * max_violation) or added >= 200:
                break
            if index not in active_set:
                active_set.add(index)
                added += 1
        if added == 0:
            break
        active = sorted(active_set)
    # Physical-unit violation for the gate.
    rows_phys = []
    h_phys = []
    for rows, h, _ in row_blocks:
        for copy in range(h.shape[0]):
            rows_phys.append(rows)
            h_phys.append(h[copy])
    if coefficients is None:
        return None, math.inf, -1
    slack_phys = np.vstack(rows_phys) @ coefficients - np.concatenate(h_phys)
    return coefficients, float(np.max(np.maximum(0.0, slack_phys))), len(active)
def admm_constrained_lstsq(matrix: np.ndarray, target: np.ndarray,
                           row_blocks: list[tuple[np.ndarray, np.ndarray, float]],
                           penalty: float = 1.0, iterations: int = 20000,
                           tolerance: float = 1.0e-9) -> tuple[np.ndarray, float, int]:
    """Solve min ||A c - b||^2 s.t. G_i c <= h_i[copy, :] for every block.

    Each block is (rows, h, scale) with h shaped (copies, n_rows): copies share
    the row matrix and differ only in offset.  Rows are scaled to O(1) and an
    adaptive penalty (Boyd's residual balancing) is used, which matters here
    because degree-scale swing rows and SI kinematic rows differ by ~100x.
    """
    scaled_blocks = [(rows / scale, h / scale) for rows, h, scale in row_blocks]
    gram = matrix.T @ matrix
    rhs_objective = matrix.T @ target
    block_grams = [rows.T @ rows for rows, _ in scaled_blocks]

    mu = penalty
    coefficients = np.zeros(matrix.shape[1])
    duals = [np.zeros_like(h) for _, h in scaled_blocks]
    slacks = [np.zeros_like(h) for _, h in scaled_blocks]
    max_violation = math.inf
    iteration = 0
    for iteration in range(iterations):
        normal = gram.copy()
        for block_index, (_, h) in enumerate(scaled_blocks):
            normal += mu * h.shape[0] * block_grams[block_index]
        chol = np.linalg.cholesky(normal + 1.0e-12 * np.eye(normal.shape[0]))
        # A few inner sweeps per penalty update keep the duals warm.
        for _ in range(20):
            rhs = rhs_objective.copy()
            for block_index, (rows, h) in enumerate(scaled_blocks):
                rows_c = (rows @ coefficients)[None, :]
                slacks[block_index] = np.minimum(h, rows_c + duals[block_index])
                duals[block_index] += rows_c - slacks[block_index]
                rhs += mu * (rows.T @ (slacks[block_index] - duals[block_index]).sum(axis=0))
            new_coefficients = np.linalg.solve(chol.T, np.linalg.solve(chol, rhs))
            if np.max(np.abs(new_coefficients - coefficients)) < 1.0e-12:
                coefficients = new_coefficients
                break
            coefficients = new_coefficients
        primal_sq = 0.0
        for block_index, (rows, h) in enumerate(scaled_blocks):
            rows_c = (rows @ coefficients)[None, :]
            primal_sq += float(np.sum(np.maximum(0.0, rows_c - h) ** 2))
        # Penalty adaptation on the scaled primal feasibility only.
        primal_norm = math.sqrt(primal_sq)
        if primal_norm > 1.0e-3:
            mu = min(mu * 1.5, 1.0e4)
        elif primal_norm < 1.0e-6:
            mu = max(mu / 1.5, 1.0e-2)
        # Report the violation in physical units for the gate.
        max_violation = max(
            float(np.max(np.maximum(0.0, (rows @ coefficients)[None, :] - h))) * scale
            for (rows, h), (_, _, scale) in zip(scaled_blocks, row_blocks)
        )
        if max_violation < tolerance:
            break
    return coefficients, max_violation, iteration + 1


def design(rope_length_m: float, tracking_weight: float = 20.0, terminal_weight: float = 5.0,
           acceleration_weight: float = 0.02, regularization: float = 1.0e-4,
           penalty: float = 1.0, iterations: int = 4000) -> dict[str, object]:
    swing_bounds = PER_LENGTH_SWING_BOUNDS.get(round(rope_length_m, 2))
    if swing_bounds is not None:
        BOUNDS["swing_zero_init_deg"], BOUNDS["swing_init_deg"] = swing_bounds
    time = np.arange(0.0, DURATION_S + 0.5 * DT_SIM, DT_SIM)
    u = time / DURATION_S
    base_position, base_velocity, base_acceleration = trajectory_progress(time)
    basis_position, basis_velocity_u, basis_acceleration_u = finite_horizon_basis(ORDER, u)
    basis_velocity = basis_velocity_u / DURATION_S
    basis_acceleration = basis_acceleration_u / (DURATION_S * DURATION_S)

    # Normalize each basis by its position sup-norm so coefficients read as
    # meters of trajectory offset; recorder coefficients undo this scaling.
    basis_scale = np.max(np.abs(basis_position), axis=0)
    norm_position = basis_position / basis_scale
    norm_velocity = basis_velocity / basis_scale
    norm_acceleration = basis_acceleration / basis_scale

    grid = np.arange(0, time.size, CONSTRAINT_STRIDE)
    base_acceleration_m_s2 = DISTANCE_M * base_acceleration
    design_ratios = np.asarray(DESIGN_RATIOS)

    # Forced swing responses (degrees) for base and basis, per design ratio.
    base_responses, _ = simulate_pendulum(
        np.tile(base_acceleration_m_s2[:, None], (1, design_ratios.size)),
        DT_SIM, rope_length_m, design_ratios,
    )
    basis_responses, _ = simulate_pendulum(
        np.repeat((DISTANCE_M * norm_acceleration)[:, :, None], design_ratios.size, axis=2
                  ).reshape(time.size, -1),
        DT_SIM, rope_length_m,
        np.tile(design_ratios, ORDER),
    )
    # basis_responses columns are ordered basis-major, ratio-minor.
    swing_base_deg = base_responses * 180.0 / math.pi                     # (T, R)
    swing_basis_deg = (basis_responses * 180.0 / math.pi)                 # (T, ORDER*R)

    # Homogeneous corner responses (degrees).
    corner_signs = [(sa, sr) for sa in (-1.0, 1.0) for sr in (-1.0, 1.0)]
    corner_theta0 = np.array([math.radians(sa * INIT_ANGLE_DEG) for sa, _ in corner_signs])
    corner_rate0 = np.array([sr * INIT_RATE_RAD_S for _, sr in corner_signs])
    n_corners = len(corner_signs)
    hom_angles, _ = simulate_pendulum(
        np.zeros((time.size, n_corners * design_ratios.size)),
        DT_SIM, rope_length_m,
        np.tile(design_ratios, n_corners),
        np.repeat(corner_theta0, design_ratios.size),
        np.repeat(corner_rate0, design_ratios.size),
    )
    hom_deg = hom_angles * 180.0 / math.pi  # (T, C*R), corner-major ratio-minor

    # Swing constraint block (ratio-major, sign-minor rows; copies = 1 zero
    # init + 4 corners share these rows).
    swing_rows_list = []
    swing_zero_list = []
    for r_index in range(design_ratios.size):
        responses = swing_basis_deg[grid][:, r_index::design_ratios.size]  # (N, ORDER)
        swing_rows_list.extend((responses, -responses))
        swing_zero_list.extend((swing_base_deg[grid, r_index], -swing_base_deg[grid, r_index]))
    swing_rows = np.vstack(swing_rows_list)
    swing_zero = np.concatenate(swing_zero_list)
    corner_h = []
    for corner in range(n_corners):
        rows = []
        for r_index in range(design_ratios.size):
            hom = hom_deg[grid, corner * design_ratios.size + r_index]
            total = swing_base_deg[grid, r_index] + hom
            rows.extend((total, -total))
        corner_h.append(BOUNDS["swing_init_deg"] - np.concatenate(rows))
    h_swing = np.stack([BOUNDS["swing_zero_init_deg"] - swing_zero] + corner_h, axis=0)

    # Kinematic constraint rows on the same grid (SI units), one block per
    # quantity so the ADMM scaling stays meaningful.  Jerk uses a denser grid:
    # as the third derivative it peaks between the coarse swing-grid points.
    grid_jerk = np.arange(0, time.size, JERK_CONSTRAINT_STRIDE)
    jerk_rows = np.vstack((DISTANCE_M * np.gradient(norm_acceleration, DT_SIM, axis=0)[grid_jerk],
                           -DISTANCE_M * np.gradient(norm_acceleration, DT_SIM, axis=0)[grid_jerk]))
    jerk_base = DISTANCE_M * np.gradient(base_acceleration, DT_SIM)[grid_jerk]
    jerk_h = np.concatenate((BOUNDS["jerk_m_s3"] - jerk_base, BOUNDS["jerk_m_s3"] + jerk_base))[None, :]
    velocity_rows = np.vstack((DISTANCE_M * norm_velocity[grid], -DISTANCE_M * norm_velocity[grid]))
    velocity_base = DISTANCE_M * base_velocity[grid]
    velocity_h = np.concatenate((BOUNDS["velocity_max_m_s"] - velocity_base,
                                 -BOUNDS["velocity_min_m_s"] + velocity_base))[None, :]
    acceleration_rows = np.vstack((DISTANCE_M * norm_acceleration[grid],
                                   -DISTANCE_M * norm_acceleration[grid]))
    acceleration_base = base_acceleration_m_s2[grid]
    acceleration_h = np.concatenate((BOUNDS["acceleration_m_s2"] - acceleration_base,
                                     BOUNDS["acceleration_m_s2"] + acceleration_base))[None, :]
    position_rows = np.vstack((norm_position[grid], -norm_position[grid]))
    position_base = base_position[grid]
    position_h = np.concatenate((BOUNDS["progress_max"] - position_base,
                                 -BOUNDS["progress_min"] + position_base))[None, :]
    coefficient_rows = np.vstack((np.eye(ORDER), -np.eye(ORDER)))
    coefficient_h = np.full((1, 2 * ORDER), BOUNDS["coefficient_m"])

    # Terminal modal responses (deg), used by both the hard convex box
    # constraint and the quadratic objective.
    modal_basis = terminal_modal_batch(
        np.repeat((DISTANCE_M * norm_acceleration)[:, :, None], design_ratios.size, axis=2
                  ).reshape(time.size, -1),
        DT_SIM, rope_length_m, np.tile(design_ratios, ORDER),
    ) * 180.0 / math.pi  # (ORDER*R,) basis-major
    modal_base = terminal_modal_batch(
        np.tile(base_acceleration_m_s2[:, None], (1, design_ratios.size)),
        DT_SIM, rope_length_m, design_ratios,
    ) * 180.0 / math.pi  # (R,)

    # Terminal modal as a hard convex box: |Re|, |Im| <= terminal_component_deg
    # per design ratio.  Basis terminal responses are linear in the
    # coefficients, so these are plain linear rows.
    term_rows = []
    term_h = []
    term_bound = BOUNDS["terminal_component_deg"]
    for r_index in range(design_ratios.size):
        modal = modal_basis[r_index::design_ratios.size]
        base_modal = modal_base[r_index]
        term_rows.extend((modal.real[None, :], -modal.real[None, :],
                          modal.imag[None, :], -modal.imag[None, :]))
        term_h.extend((term_bound - base_modal.real, term_bound + base_modal.real,
                       term_bound - base_modal.imag, term_bound + base_modal.imag))
    terminal_rows_c = np.vstack(term_rows)
    terminal_h_c = np.asarray(term_h)[None, :]

    # Pointwise command-offset cap (linear rows).
    offset_rows = np.vstack((DISTANCE_M * norm_position[grid],
                             -DISTANCE_M * norm_position[grid]))
    offset_h = np.full((1, 2 * grid.size), BOUNDS["offset_peak_m"])

    constraint_blocks = [
        (swing_rows, h_swing, BOUNDS["swing_init_deg"]),
        (terminal_rows_c, terminal_h_c, term_bound),
        (offset_rows, offset_h, BOUNDS["offset_peak_m"]),
        (jerk_rows, jerk_h, BOUNDS["jerk_m_s3"]),
        (velocity_rows, velocity_h, BOUNDS["velocity_max_m_s"]),
        (acceleration_rows, acceleration_h, BOUNDS["acceleration_m_s2"]),
        (position_rows, position_h, 1.0),
        (coefficient_rows, coefficient_h, BOUNDS["coefficient_m"]),
    ]

    # Objective: tracking offset + terminal modal over design ratios +
    # acceleration smoothness + regularization.  Safety peak and jerk are hard
    # constraints, matching the frozen priority order.
    terminal_rows = []
    terminal_target = []
    for r_index in range(design_ratios.size):
        modal = modal_basis[r_index::design_ratios.size]  # (ORDER,)
        terminal_rows.extend((modal.real, modal.imag))
        terminal_target.extend((-modal_base[r_index].real, -modal_base[r_index].imag))
    matrix = np.vstack((
        math.sqrt(tracking_weight) * DISTANCE_M * norm_position[grid],
        math.sqrt(terminal_weight) * np.asarray(terminal_rows),
        math.sqrt(acceleration_weight) * DISTANCE_M * norm_acceleration[grid],
        math.sqrt(regularization) * np.eye(ORDER),
    ))
    target = np.concatenate((
        np.zeros(grid.size),
        math.sqrt(terminal_weight) * np.asarray(terminal_target),
        np.zeros(grid.size),
        np.zeros(ORDER),
    ))

    coefficients, max_violation, used_iterations = solve_qp(
        matrix, target, constraint_blocks, penalty=penalty, iterations=iterations,
    )
    if coefficients is None:
        return {
            "rope_length_m": rope_length_m,
            "version": "FHv2_full_horizon_convex",
            "bounds": dict(BOUNDS),
            "solver": {"method": "OSQP", "status": "primal_infeasible",
                       "iterations_used": used_iterations,
                       "max_constraint_violation": math.inf},
            "offline_gates": {"feasible": False},
            "offline_all_pass": False,
        }

    # Dense independent audit (independent ratio grid, full dt).
    shaped_position = base_position + norm_position @ coefficients
    shaped_velocity = base_velocity + norm_velocity @ coefficients
    shaped_acceleration = base_acceleration + norm_acceleration @ coefficients
    acceleration_m_s2 = DISTANCE_M * shaped_acceleration
    jerk = np.gradient(acceleration_m_s2, DT_SIM)
    offset = DISTANCE_M * (shaped_position - base_position)
    audit_ratios = np.linspace(RATIO_MIN, RATIO_MAX, 121)
    terminal_deg = np.abs(terminal_modal_batch(
        np.tile(acceleration_m_s2[:, None], (1, audit_ratios.size)),
        DT_SIM, rope_length_m, audit_ratios,
    )) * 180.0 / math.pi
    audit_angles, _ = simulate_pendulum(
        np.tile(acceleration_m_s2[:, None], (1, audit_ratios.size)),
        DT_SIM, rope_length_m, audit_ratios,
    )
    transient_zero_deg = np.max(np.abs(audit_angles), axis=0) * 180.0 / math.pi
    audit_init_angles, _ = simulate_pendulum(
        np.tile(acceleration_m_s2[:, None], (1, n_corners * audit_ratios.size)),
        DT_SIM, rope_length_m,
        np.tile(audit_ratios, n_corners),
        np.repeat(corner_theta0, audit_ratios.size),
        np.repeat(corner_rate0, audit_ratios.size),
    )
    transient_init_deg = np.max(np.abs(audit_init_angles), axis=0).reshape(
        n_corners, audit_ratios.size).max(axis=0) * 180.0 / math.pi

    base_nominal_angles, _ = simulate_pendulum(
        base_acceleration_m_s2, DT_SIM, rope_length_m, 1.0)
    base_transient_deg = float(np.max(np.abs(base_nominal_angles)) * 180.0 / math.pi)
    base_terminal_deg = float(np.abs(terminal_modal_batch(
        base_acceleration_m_s2, DT_SIM, rope_length_m, 1.0))[0] * 180.0 / math.pi)

    recorder_coefficients = (coefficients / basis_scale).tolist()
    result = {
        "rope_length_m": rope_length_m,
        "version": "FHv2_full_horizon_convex",
        "order": ORDER,
        "basis": "u^(k+3)*(1-u)^(14-k), k=0..11 (recorder coefficients are unnormalized)",
        "design_ratios": list(DESIGN_RATIOS),
        "ratio_interval": [RATIO_MIN, RATIO_MAX],
        "init_corners": {"angle_deg": INIT_ANGLE_DEG, "rate_rad_s": INIT_RATE_RAD_S},
        "bounds": {
            "swing_max_zero_init_deg": BOUNDS["swing_zero_init_deg"],
            "swing_max_init_deg": BOUNDS["swing_init_deg"],
            "jerk_max_m_s3": BOUNDS["jerk_m_s3"],
            "velocity_range_m_s": [BOUNDS["velocity_min_m_s"], BOUNDS["velocity_max_m_s"]],
            "acceleration_max_m_s2": BOUNDS["acceleration_m_s2"],
            "progress_range": [BOUNDS["progress_min"], BOUNDS["progress_max"]],
            "coefficient_max_m": BOUNDS["coefficient_m"],
        },
        "objective_weights": {
            "tracking": tracking_weight, "terminal": terminal_weight,
            "acceleration": acceleration_weight, "regularization": regularization,
        },
        "solver": {"method": "ADMM_QP", "penalty": penalty,
                   "iterations_requested": iterations, "iterations_used": used_iterations,
                   "max_constraint_violation": float(max_violation)},
        "basis_scale": basis_scale.tolist(),
        "coefficients": recorder_coefficients,
        "coefficients_offset_m": coefficients.tolist(),
        "command_offset_rms_m": float(np.sqrt(np.mean(offset * offset))),
        "command_offset_peak_m": float(np.max(np.abs(offset))),
        "position_min": float(np.min(shaped_position)),
        "position_max": float(np.max(shaped_position)),
        "velocity_min_m_s": float(np.min(DISTANCE_M * shaped_velocity)),
        "peak_velocity_m_s": float(np.max(np.abs(DISTANCE_M * shaped_velocity))),
        "peak_acceleration_m_s2": float(np.max(np.abs(acceleration_m_s2))),
        "peak_jerk_m_s3": float(np.max(np.abs(jerk))),
        "worst_terminal_modal_amplitude_deg": float(np.max(terminal_deg)),
        "worst_terminal_modal_frequency_ratio": float(audit_ratios[int(np.argmax(terminal_deg))]),
        "nominal_terminal_modal_amplitude_deg": float(terminal_deg[40]),
        "worst_transient_swing_zero_init_deg": float(np.max(transient_zero_deg)),
        "worst_transient_swing_frequency_ratio": float(
            audit_ratios[int(np.argmax(transient_zero_deg))]),
        "worst_transient_swing_with_init_deg": float(np.max(transient_init_deg)),
        "base_nominal_transient_swing_deg": base_transient_deg,
        "base_nominal_terminal_modal_deg": base_terminal_deg,
    }
    result["offline_gates"] = offline_gates(result)
    result["offline_all_pass"] = all(result["offline_gates"].values())
    return result


def offline_gates(item: dict[str, object]) -> dict[str, bool]:
    overshoot_m = max(0.0, -DISTANCE_M * float(item["position_min"]),
                      DISTANCE_M * (float(item["position_max"]) - 1.0))
    return {
        "command_offset_rms_le_0p095_m": float(item["command_offset_rms_m"]) <= 0.095,
        "peak_velocity_le_0p55_m_s": float(item["peak_velocity_m_s"]) <= 0.55,
        "peak_acceleration_le_0p65_m_s2": float(item["peak_acceleration_m_s2"]) <= 0.65,
        "peak_jerk_le_1p61_m_s3": float(item["peak_jerk_m_s3"]) <= 1.61,
        "worst_terminal_mode_le_1p15_deg": float(item["worst_terminal_modal_amplitude_deg"]) <= 1.15,
        "worst_transient_zero_init_within_bound":
            float(item["worst_transient_swing_zero_init_deg"])
            <= PER_LENGTH_SWING_BOUNDS.get(round(float(item["rope_length_m"]), 2),
                                           (BOUNDS["swing_zero_init_deg"], 0.0))[0] + 0.05,
        "worst_transient_with_init_within_bound":
            float(item["worst_transient_swing_with_init_deg"])
            <= PER_LENGTH_SWING_BOUNDS.get(round(float(item["rope_length_m"]), 2),
                                           (0.0, BOUNDS["swing_init_deg"]))[1] + 0.05,
        "position_overshoot_le_0p005_m": overshoot_m <= 0.005,
        "velocity_never_reverses_past_0p02": float(item["velocity_min_m_s"]) >= -0.02,
        "solver_violation_le_5e-3": float(item["solver"]["max_constraint_violation"]) <= 5.0e-3,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rope-length", type=float, action="append", default=[])
    parser.add_argument("--tracking-weight", type=float, default=20.0)
    parser.add_argument("--terminal-weight", type=float, default=5.0)
    parser.add_argument("--acceleration-weight", type=float, default=0.02)
    parser.add_argument("--regularization", type=float, default=1.0e-4)
    parser.add_argument("--penalty", type=float, default=1.0)
    parser.add_argument("--iterations", type=int, default=4000)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    lengths = args.rope_length or [0.5, 0.6, 0.8]
    result = [
        design(length, args.tracking_weight, args.terminal_weight, args.acceleration_weight,
               args.regularization, args.penalty, args.iterations)
        for length in lengths
    ]
    for item in result:
        if "command_offset_rms_m" not in item:
            print(f"L={item['rope_length_m']:.1f} m: INFEASIBLE "
                  f"({item['solver']['status']})")
            continue
        print(f"L={item['rope_length_m']:.1f} m: all_pass={item['offline_all_pass']} "
              f"offset_rms={item['command_offset_rms_m']:.4f} m, "
              f"jerk={item['peak_jerk_m_s3']:.3f} m/s^3, "
              f"terminal={item['worst_terminal_modal_amplitude_deg']:.3f} deg, "
              f"transient0={item['worst_transient_swing_zero_init_deg']:.3f} deg, "
              f"transient_init={item['worst_transient_swing_with_init_deg']:.3f} deg, "
              f"violation={item['solver']['max_constraint_violation']:.2e}, "
              f"iters={item['solver']['iterations_used']}")
        for name, passed in item["offline_gates"].items():
            if not passed:
                print(f"  FAIL {name}")
    text = json.dumps(result, ensure_ascii=False, indent=2) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(text, encoding="utf-8")
    return 0 if all(item["offline_all_pass"] for item in result) else 1


if __name__ == "__main__":
    raise SystemExit(main())
