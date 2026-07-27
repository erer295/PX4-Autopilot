#!/usr/bin/env python3
"""Task-book recorder adapter for ZD680 EKF-ESO VFB validation runs."""

from __future__ import annotations

import math
import runpy
import sys
from dataclasses import asdict
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
SOURCE = REPO_ROOT / "Tools" / "simulation" / "gz" / "tools" / "position_offboard_flight_recorder.py"
API = runpy.run_path(str(SOURCE), run_name="zd680_vfb_recorder_api")

MODES = (
    "NONE",
    "VFB_L06_W00",
    "VFB_L06_W05",
    "VFB_L08_W00",
    "VFB_L08_W25",
    "VFB_L08_W05",
    "STD_L06_PID_AS",
    "STD_L06_PID_AS_PAS",
    "STD_L06_PID_AS_FSO_SHADOW",
    "STD_L06_PID_AS_FSOPC",
    "STD_L06_PID_AS_FSOPC_K20",
    "STD_L06_PID_AS_FSOPC_K40",
    "STD_L06_PID_AS_FSOPC_K60",
    "STD_L06_PID_AS_FSOJPC_K05",
    "STD_L06_PID_AS_FSOJPC_K10",
    "STD_L06_PID_AS_FSOJPC_K15",
    "STD_L06_PID_AS_FSOJPC_K20",
    "STD_L06_PID_AS_FSOUSC_K20",
    "STD_L06_PID_AS_FSOUSC_READY",
    "STD_L08_PID_AS",
    "STD_L08_PID_AS_FSOUSC_K20",
    "STD_L06_PID_AS_HGS_K40",
    "STD_L06_PID_AS_HGS_READY",
    "STD_L08_PID_AS_HGS_K40",
    "STD_L06_PID_AS_HGS2_ORACLE_K40",
    "STD_L06_PID_AS_HGS2_ORACLE_READY",
    "STD_L06_PID_AS_HGS2_RLS_SHADOW",
    "STD_L06_PID_AS_HGS2_RLS_READY",
    "STD_L06_PID_AS_HGS2_BASE",
    "STD_L06_PID_AS_UCIS_BASE",
    "STD_L06_PID_AS_UCIS_FIXED",
    "STD_L06_PID_AS_UCIS_FULL",
    "STD_L06_PID_AS_UCIS_FUCI",
    "STD_L06_PID_AS_UCIS_FH",
    "STD_L08_PID_AS_UCIS_FH",
    "STD_L06_PID_AS_UCIS_READY",
    "STD_L06_FULL_W05",
    "STD_L06_FULL_W10_OBS_DECOUPLE",
    "STD_L05_PID_AS_UCIS_FH2",
    "STD_L06_PID_AS_UCIS_FH2",
    "STD_L08_PID_AS_UCIS_FH2",
    "STD_L06P75_PID_AS_UCIS_FH2",
    "STD_MM070_PID_AS_UCIS_FH2",
    "STD_L06_PID_AS_UCIS_EI",
    "STD_MM070_PID_AS_UCIS_EI",
)


def velocity_feedback_weight(mode: str) -> float:
    if "PID_AS" in mode:
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
    raise ValueError(f"unsupported VFB weight label: {mode}")


def profile(mode: str, best_ramp_s: float = 1.0):
    if mode not in MODES or mode == "NONE":
        raise ValueError(f"unsupported VFB validation mode: {mode}")
    if "MM070" in mode:
        rope_length_m = 0.7
    elif "L08" in mode:
        rope_length_m = 0.8
    elif "L05" in mode:
        rope_length_m = 0.5
    else:
        rope_length_m = 0.6
    return {
        "mode": mode,
        "controller": "PID_AS" if "PID_AS" in mode else "FULL",
        "rope_length_m": rope_length_m,
        # Rope length the coefficients and MC_HANG_LEN assume; differs from
        # the physical rope for frequency-mismatch (MM) modes.
        "design_rope_length_m": 0.6 if "MM070" in mode else rope_length_m,
        "payload_mass_kg": 0.75 if "P75" in mode else 0.5,
        "actions": (
            "hold:1" if mode.endswith("READY")
            else "hold:5;north:2:6;hold:5;east:2:6;hold:5;south:2:6;hold:5;west:2:6;hold:10"
        ),
        "motion_profile": "s_curve",
        "ramp_time_s": 1.0,
        "wrench_schedule": "",
        "velocity_feedback_weight": velocity_feedback_weight(mode),
        "initial_condition_profile": "strict_matched_taskbook_v1",
        "initial_angle_rms_max_deg": 0.25 if ("HGS2" in mode or "UCIS" in mode) else 0.75,
        "initial_rate_rms_max_rad_s": 0.020 if ("HGS2" in mode or "UCIS" in mode) else 0.06,
        "initial_current_angle_max_deg": 0.40 if ("HGS2" in mode or "UCIS" in mode) else 0.75,
        "initial_current_rate_max_rad_s": 0.035 if ("HGS2" in mode or "UCIS" in mode) else 0.06,
        "initial_xy_error_max_m": 0.08 if ("HGS2" in mode or "UCIS" in mode) else 0.10,
        "initial_horizontal_speed_max_m_s": 0.10,
        "initial_stable_time_s": 3.0,
    }


def controller_parameters(controller: str, rope_length_m: float):
    int_params, float_params = API["validation_controller_parameters"](controller, rope_length_m)
    int_params = dict(int_params)
    float_params = dict(float_params)
    float_params["MC_HANG_WO_R"] = 0.60
    return int_params, float_params


def estimator_parameters():
    """Freeze the existing GPS-only EKF setup used by the VFB baseline.

    Z truth disagreement is retained as an observational limitation for this
    controller-structure test.  Keeping these values fixed prevents a stale
    parameters.bson from silently changing the estimator between VFB modes.
    """
    int_params = {
        "EKF2_EV_CTRL": 0,
        "EKF2_GPS_CTRL": 7,
        "EKF2_HGT_REF": 1,
        "EKF2_BARO_CTRL": 0,
        "EKF2_RNG_CTRL": 0,
    }
    float_params = {
        "EKF2_BARO_NOISE": 3.50,
        "EKF2_ACC_B_NOISE": 0.0,
    }
    return int_params, float_params


def parameters_for_mode(mode: str):
    selected = profile(mode)
    int_params, float_params = controller_parameters(
        str(selected["controller"]), float(selected.get("design_rope_length_m",
                                                        selected["rope_length_m"]))
    )
    if mode == "STD_L06_PID_AS_PAS":
        # Stage-4 isolation: this mode differs from PID+AS only by enabling
        # the already frozen ACTIVE PAS path.
        int_params["MC_HANG_PAS_MD"] = 2
        int_params["MC_HANG_PAS_POS"] = 1
    fso_shadow = mode == "STD_L06_PID_AS_FSO_SHADOW"
    fso_active = "FSOPC" in mode or "FSOJPC" in mode or "FSOUSC" in mode
    heso_gain_schedule = "HGS_K40" in mode or "HGS_READY" in mode
    hgs2_oracle = "HGS2_ORACLE" in mode
    hgs2_rls_shadow = "HGS2_RLS" in mode
    unified_coordinator = (
        "UCIS_FULL" in mode or "UCIS_FUCI" in mode or "UCIS_FH" in mode or "UCIS_READY" in mode
        or "UCIS_EI" in mode
    )
    int_params["MC_HANG_FSO_MD"] = (
        6 if unified_coordinator else
        (5 if hgs2_oracle else
        (4 if hgs2_rls_shadow else
         (3 if heso_gain_schedule else (1 if fso_shadow else (2 if fso_active else 0)))))
    )
    int_params["MC_HANG_FSO_POS"] = 1 if (
        "FSOJPC" in mode or "FSOUSC" in mode or heso_gain_schedule or hgs2_oracle or hgs2_rls_shadow
        or unified_coordinator
    ) else 0
    compensation_gain = {
        "STD_L06_PID_AS_FSOPC_K20": 0.20,
        "STD_L06_PID_AS_FSOPC_K40": 0.40,
        "STD_L06_PID_AS_FSOPC_K60": 0.60,
        "STD_L06_PID_AS_FSOJPC_K05": 0.05,
        "STD_L06_PID_AS_FSOJPC_K10": 0.10,
        "STD_L06_PID_AS_FSOJPC_K15": 0.15,
        "STD_L06_PID_AS_FSOJPC_K20": 0.20,
        "STD_L06_PID_AS_FSOUSC_K20": 0.20,
        "STD_L06_PID_AS_FSOUSC_READY": 0.20,
        "STD_L08_PID_AS_FSOUSC_K20": 0.20,
        "STD_L06_PID_AS_HGS_K40": 0.40,
        "STD_L06_PID_AS_HGS_READY": 0.40,
        "STD_L08_PID_AS_HGS_K40": 0.40,
        "STD_L06_PID_AS_HGS2_ORACLE_K40": 0.40,
        "STD_L06_PID_AS_HGS2_ORACLE_READY": 0.40,
        "STD_L06_PID_AS_HGS2_RLS_SHADOW": 0.40,
        "STD_L06_PID_AS_HGS2_RLS_READY": 0.40,
        "STD_L06_PID_AS_HGS2_BASE": 0.40,
    }.get(mode, 0.20)
    compensation_limit = {
        "STD_L06_PID_AS_FSOJPC_K05": 0.04,
        "STD_L06_PID_AS_FSOJPC_K10": 0.04,
        "STD_L06_PID_AS_FSOJPC_K15": 0.06,
        "STD_L06_PID_AS_FSOJPC_K20": 0.08,
        "STD_L06_PID_AS_FSOUSC_K20": 0.08,
        "STD_L06_PID_AS_FSOUSC_READY": 0.08,
        "STD_L08_PID_AS_FSOUSC_K20": 0.08,
    }.get(mode, 0.08)
    float_params.update({
        "MC_HANG_FSO_BW": 0.30,
        "MC_HANG_FSO_K": compensation_gain,
        "MC_HANG_FSO_LEAD": 0.05,
        "MC_HANG_FSO_LIM": compensation_limit,
        "MC_HANG_FSO_SLW": 0.30,
        "MC_HANG_FSO_E": 0.003,
        "MC_HANG_FSO_R": 0.03,
        "MC_HANG_FSO_T": 2.0,
        "MC_HANG_FSO_CF": 0.20,
        "MC_HANG_FSO_PE": 0.05,
    })
    float_params["MC_PLADRC_VFB_W"] = float(selected["velocity_feedback_weight"])
    estimator_int, estimator_float = estimator_parameters()
    int_params.update(estimator_int)
    float_params.update(estimator_float)
    return int_params, float_params


def apply_params(master, args) -> None:
    if args.validation_suite == "NONE":
        return
    selected = profile(args.validation_suite, args.validation_best_ramp)
    int_params, float_params = parameters_for_mode(args.validation_suite)
    print(
        "applying task-book VFB parameter profile: "
        f"{args.validation_suite} (L={selected['rope_length_m']}, W={selected['velocity_feedback_weight']})"
    )
    for name, value in int_params.items():
        API["set_int_param"](master, name, value)
    for name, value in float_params.items():
        API["set_float_param"](master, name, value)
    args.requested_validation_parameters = {**int_params, **float_params}
    args.requested_active_parameters = dict(args.requested_validation_parameters)


GLOBALS = API["main"].__globals__
GLOBALS["VALIDATION_SUITE_MODES"] = MODES
GLOBALS["ACTIVE_SNAPSHOT_PARAMS"] = tuple(GLOBALS["ACTIVE_SNAPSHOT_PARAMS"]) + (
    "MC_PLADRC_VFB_W", "EKF2_BARO_NOISE", "EKF2_ACC_B_NOISE",
    "MC_HANG_FSO_MD", "MC_HANG_FSO_BW", "MC_HANG_FSO_K", "MC_HANG_FSO_LEAD", "MC_HANG_FSO_LIM",
    "MC_HANG_FSO_SLW", "MC_HANG_FSO_E", "MC_HANG_FSO_R", "MC_HANG_FSO_T", "MC_HANG_FSO_CF",
    "MC_HANG_FSO_POS", "MC_HANG_FSO_PE",
)
GLOBALS["validation_suite_profile"] = profile
GLOBALS["validation_controller_parameters"] = controller_parameters
GLOBALS["validation_mode_parameters"] = parameters_for_mode
GLOBALS["apply_validation_params"] = apply_params

ORIGINAL_APPLY_EXPERIMENT_PRESET = GLOBALS["apply_experiment_preset"]


def apply_experiment_preset_with_strict_initial_pairing(args, parser) -> None:
    ORIGINAL_APPLY_EXPERIMENT_PRESET(args, parser)
    if args.validation_suite in MODES and args.validation_suite != "NONE":
        # PID-family runs always command MC_RBF_LEARN_EN=0.  Keep the run
        # index/metadata label consistent with that commanded parameter.
        args.rbf_learn = False
        selected = profile(args.validation_suite, args.validation_best_ramp)
        args.hang_initial_timeout = 60.0
        args.hang_initial_window = 1.0
        args.hang_initial_stable_time = float(selected["initial_stable_time_s"])
        args.hang_initial_angle_rms_deg = float(selected["initial_angle_rms_max_deg"])
        args.hang_initial_rate_rms = float(selected["initial_rate_rms_max_rad_s"])
        args.hang_initial_current_angle_deg = float(selected["initial_current_angle_max_deg"])
        args.hang_initial_current_rate = float(selected["initial_current_rate_max_rad_s"])
        args.xy_tolerance = float(selected["initial_xy_error_max_m"])
        args.velocity_tolerance = float(selected["initial_horizontal_speed_max_m_s"])


GLOBALS["apply_experiment_preset"] = apply_experiment_preset_with_strict_initial_pairing


ORIGINAL_RUN_ACTION_TRANSITION = GLOBALS["run_action_transition"]


def zvd_impulses(rope_length_m: float, damping_ratio: float = 0.10):
    """Return the standard damped-ZVD impulse spacing and normalized weights."""
    omega = math.sqrt(9.80665 / max(0.05, rope_length_m))
    damping_ratio = min(0.30, max(0.0, damping_ratio))
    root = math.sqrt(max(1.0e-6, 1.0 - damping_ratio * damping_ratio))
    spacing_s = math.pi / (omega * root)
    decay = math.exp(-damping_ratio * math.pi / root)
    denominator = (1.0 + decay) ** 2
    return spacing_s, (1.0 / denominator, 2.0 * decay / denominator, decay * decay / denominator)


def frequency_uncertainty_constrained_impulses(rope_length_m: float, damping_ratio: float = 0.10):
    """Return the frozen positive three-impulse specified-insensitivity design.

    This is the selected solution of the preregistered offline problem: reduce
    six-second mission-reference distortion subject to V(rho) <= 0.10 for
    rho in [0.90, 1.20], V(1) <= 0.03, and frozen kinematic limits.  Times are
    normalized by the nominal damped half-period so the same dimensionless
    design scales with rope length.
    """
    nominal_spacing_s, _ = zvd_impulses(rope_length_m, damping_ratio)
    time_ratios = (0.0, 0.7298599938329202, 1.4980561058033302)
    weights = (0.3295785796978337, 0.4230609449850559, 0.24736047531711053)
    impulse_times_s = tuple(ratio * nominal_spacing_s for ratio in time_ratios)
    return impulse_times_s, weights


def extra_insensitive_impulses(rope_length_m: float, damping_ratio: float = 0.10):
    """Classic three-impulse Extra-Insensitive (EI) shaper (Singhose et al.).

    Same damped half-period spacing as ZVD; weights trade nominal residual
    for a wide low-sensitivity notch.  Used as the literature baseline for
    the frequency-uncertainty comparison (amendment v4).
    """
    spacing_s, weights = zvd_impulses(rope_length_m, damping_ratio)
    # Recover K = exp(-zeta*pi/sqrt(1-zeta^2)) from the ZVD weights: with
    # denominator (1+K)^2 the weights are (1, 2K, K^2)/(1+K)^2.
    k_from_w = math.sqrt(weights[2] / weights[0])
    ei_weights = ((1.0 + k_from_w) / 4.0, (1.0 - k_from_w) / 2.0, (1.0 + k_from_w) / 4.0)
    return (0.0, spacing_s, 2.0 * spacing_s), ei_weights


# Frozen by finite_horizon_design.json before either formal flight.  Both
# lengths use the same basis, weights and frequency-ratio grid; only the
# nominal rope length supplied to the offline design differs.
FINITE_HORIZON_COEFFICIENTS = {
    0.6: (
        -15.155016422082781, -231.6804620134673, -35.85732223475387,
        13.323052997616841, 7.413256947975582, -2.6982596301418713,
        -6.406220997443135, -6.1846955886894115, 9.239843683212403,
        145.24700266677814, 739.6481077800605, -7.120894787342365,
    ),
    0.8: (
        -71.71611144534991, -240.97082458970436, -51.39435328086336,
        15.054903171357005, 17.164710107937754, -0.4036925586772697,
        -18.420236886409302, -21.35477711872853, 27.361739063282727,
        229.60829272510506, 738.5579092225387, -19.71188414793457,
    ),
}


def finite_horizon_coefficients(rope_length_m: float):
    for frozen_length_m, coefficients in FINITE_HORIZON_COEFFICIENTS.items():
        if abs(rope_length_m - frozen_length_m) <= 1.0e-6:
            return coefficients
    raise ValueError(f"no frozen finite-horizon trajectory for rope length {rope_length_m:g} m")


# FHv2 (2026-07-23): order-16 endpoint-flat basis u^(k+3)(1-u)^(18-k), frozen
# by fh2_design.json.  Full-horizon convex design: swing/jerk/terminal/offset
# hard constraints over rho in [0.90, 1.20] with corner initial states.
FINITE_HORIZON2_ORDER = 16
FINITE_HORIZON2_COEFFICIENTS = {
    0.5: (
        -14.736558789, 189.204778807, -6457.023589192,
        -22244.952498153, 127689.800890561, -229959.809095296,
        13125.578530116, 409558.676279616, -354009.367125382,
        -119225.80888729, 229959.737170802, -64524.26621463,
        -9754.819624679, 11226.643280762, -336.966124253,
        16.905690769,
    ),
    0.6: (
        -13.016965883, 111.224303323, -4483.272369254,
        -31961.47175277, 127689.858435071, -151211.730015788,
        -115843.580695291, 409558.573268479, -325496.610273216,
        -72609.341840612, 229959.710375061, -99789.500832742,
        8988.968878596, 8198.538966351, -229.980854347,
        14.849441044,
    ),
    0.8: (
        -8.230390369, -96.506782899, 472.044335249,
        -49485.763703918, 127689.858435071, -2632.107287509,
        -338281.164109529, 303979.267665194, -47831.270183484,
        -95067.143607111, 182594.864439616, -127689.858434928,
        38793.310149997, 2102.950331704, 1.428027305,
        10.058221118,
    ),
}


def finite_horizon2_coefficients(rope_length_m: float):
    for frozen_length_m, coefficients in FINITE_HORIZON2_COEFFICIENTS.items():
        if abs(rope_length_m - frozen_length_m) <= 1.0e-6:
            return coefficients
    raise ValueError(f"no frozen FHv2 trajectory for rope length {rope_length_m:g} m")


def finite_horizon2_basis(progress_u: float, index: int):
    """Return b, db/du and d2b/du2 for u^(k+3)(1-u)^(18-k)."""
    p = index + 3
    q = FINITE_HORIZON2_ORDER + 2 - index
    u = min(1.0, max(0.0, progress_u))
    one_minus_u = 1.0 - u
    position = u ** p * one_minus_u ** q
    velocity_u = (
        p * u ** (p - 1) * one_minus_u ** q
        - q * u ** p * one_minus_u ** (q - 1)
    )
    acceleration_u = (
        p * (p - 1) * u ** (p - 2) * one_minus_u ** q
        - 2 * p * q * u ** (p - 1) * one_minus_u ** (q - 1)
        + q * (q - 1) * u ** p * one_minus_u ** (q - 2)
    )
    return position, velocity_u, acceleration_u


def finite_horizon2_progress(elapsed_s: float, duration_s: float, motion_profile: str,
                             ramp_time_s: float, coefficients):
    base_position, base_velocity, base_acceleration = GLOBALS["trajectory_progress"](
        elapsed_s, duration_s, motion_profile, ramp_time_s
    )
    duration_s = max(0.001, duration_s)
    progress_u = min(1.0, max(0.0, elapsed_s / duration_s))
    position = base_position
    velocity = base_velocity
    acceleration = base_acceleration
    for index, coefficient in enumerate(coefficients):
        basis_position, basis_velocity_u, basis_acceleration_u = finite_horizon2_basis(
            progress_u, index
        )
        position += coefficient * basis_position
        velocity += coefficient * basis_velocity_u / duration_s
        acceleration += coefficient * basis_acceleration_u / (duration_s * duration_s)
    return position, velocity, acceleration


def finite_horizon2_transition_setpoint(start, end, elapsed_s, duration_s, motion_profile,
                                        ramp_time_s, coefficients):
    progress, progress_rate, progress_acceleration = finite_horizon2_progress(
        elapsed_s, duration_s, motion_profile, ramp_time_s, coefficients
    )
    dx_m = end.x_m - start.x_m
    dy_m = end.y_m - start.y_m
    dz_m = end.z_m - start.z_m
    yaw_delta = GLOBALS["wrap_pi"](end.yaw_rad - start.yaw_rad)
    PositionSetpoint = API["PositionSetpoint"]
    return PositionSetpoint(
        x_m=start.x_m + dx_m * progress,
        y_m=start.y_m + dy_m * progress,
        z_m=start.z_m + dz_m * progress,
        yaw_rad=GLOBALS["wrap_pi"](start.yaw_rad + yaw_delta * progress),
        vx_m_s=dx_m * progress_rate,
        vy_m_s=dy_m * progress_rate,
        vz_m_s=dz_m * progress_rate,
        ax_m_s2=dx_m * progress_acceleration,
        ay_m_s2=dy_m * progress_acceleration,
        az_m_s2=dz_m * progress_acceleration,
    )


def finite_horizon_basis(progress_u: float, index: int):
    """Return b, db/du and d2b/du2 for u^(k+3)(1-u)^(14-k)."""
    p = index + 3
    q = 14 - index
    u = min(1.0, max(0.0, progress_u))
    one_minus_u = 1.0 - u
    position = u ** p * one_minus_u ** q
    velocity_u = (
        p * u ** (p - 1) * one_minus_u ** q
        - q * u ** p * one_minus_u ** (q - 1)
    )
    acceleration_u = (
        p * (p - 1) * u ** (p - 2) * one_minus_u ** q
        - 2 * p * q * u ** (p - 1) * one_minus_u ** (q - 1)
        + q * (q - 1) * u ** p * one_minus_u ** (q - 2)
    )
    return position, velocity_u, acceleration_u


def finite_horizon_progress(elapsed_s: float, duration_s: float, motion_profile: str,
                            ramp_time_s: float, coefficients):
    base_position, base_velocity, base_acceleration = GLOBALS["trajectory_progress"](
        elapsed_s, duration_s, motion_profile, ramp_time_s
    )
    duration_s = max(0.001, duration_s)
    progress_u = min(1.0, max(0.0, elapsed_s / duration_s))
    position = base_position
    velocity = base_velocity
    acceleration = base_acceleration
    for index, coefficient in enumerate(coefficients):
        basis_position, basis_velocity_u, basis_acceleration_u = finite_horizon_basis(
            progress_u, index
        )
        position += coefficient * basis_position
        velocity += coefficient * basis_velocity_u / duration_s
        acceleration += coefficient * basis_acceleration_u / (duration_s * duration_s)
    return position, velocity, acceleration


def finite_horizon_transition_setpoint(start, end, elapsed_s, duration_s, motion_profile,
                                       ramp_time_s, coefficients):
    progress, progress_rate, progress_acceleration = finite_horizon_progress(
        elapsed_s, duration_s, motion_profile, ramp_time_s, coefficients
    )
    dx_m = end.x_m - start.x_m
    dy_m = end.y_m - start.y_m
    dz_m = end.z_m - start.z_m
    yaw_delta = GLOBALS["wrap_pi"](end.yaw_rad - start.yaw_rad)
    PositionSetpoint = API["PositionSetpoint"]
    return PositionSetpoint(
        x_m=start.x_m + dx_m * progress,
        y_m=start.y_m + dy_m * progress,
        z_m=start.z_m + dz_m * progress,
        yaw_rad=GLOBALS["wrap_pi"](start.yaw_rad + yaw_delta * progress),
        vx_m_s=dx_m * progress_rate,
        vy_m_s=dy_m * progress_rate,
        vz_m_s=dz_m * progress_rate,
        ax_m_s2=dx_m * progress_acceleration,
        ay_m_s2=dy_m * progress_acceleration,
        az_m_s2=dz_m * progress_acceleration,
    )


def finite_or_zero(value: float) -> float:
    return value if math.isfinite(value) else 0.0


def input_shaped_transition_setpoint(start, end, elapsed_s, total_duration_s, base_duration_s,
                                     motion_profile, ramp_time_s, impulse_times_s, weights):
    samples = [
        GLOBALS["shaped_trajectory_setpoint"](
            start, end, elapsed_s - impulse_time_s, base_duration_s, motion_profile, ramp_time_s
        )
        for impulse_time_s in impulse_times_s
    ]

    def weighted(field: str, nan_as_zero: bool = False) -> float:
        values = [float(getattr(sample, field)) for sample in samples]
        if nan_as_zero:
            values = [finite_or_zero(value) for value in values]
        return sum(weight * value for weight, value in zip(weights, values))

    PositionSetpoint = API["PositionSetpoint"]
    return PositionSetpoint(
        x_m=weighted("x_m"),
        y_m=weighted("y_m"),
        z_m=weighted("z_m"),
        yaw_rad=samples[0].yaw_rad,
        vx_m_s=weighted("vx_m_s", True),
        vy_m_s=weighted("vy_m_s", True),
        vz_m_s=weighted("vz_m_s", True),
        ax_m_s2=weighted("ax_m_s2", True),
        ay_m_s2=weighted("ay_m_s2", True),
        az_m_s2=weighted("az_m_s2", True),
    )


def run_action_transition_with_zvd(master, state, args, events, name, duration_s,
                                   target_setpoint, spec, index, extra_params):
    if args.validation_suite not in (
        "STD_L06_PID_AS_UCIS_FIXED", "STD_L06_PID_AS_UCIS_FULL", "STD_L06_PID_AS_UCIS_FUCI",
        "STD_L06_PID_AS_UCIS_FH", "STD_L08_PID_AS_UCIS_FH",
        "STD_L05_PID_AS_UCIS_FH2", "STD_L06_PID_AS_UCIS_FH2", "STD_L08_PID_AS_UCIS_FH2",
        "STD_L06P75_PID_AS_UCIS_FH2", "STD_MM070_PID_AS_UCIS_FH2",
        "STD_L06_PID_AS_UCIS_EI", "STD_MM070_PID_AS_UCIS_EI",
    ):
        return ORIGINAL_RUN_ACTION_TRANSITION(
            master, state, args, events, name, duration_s, target_setpoint, spec, index, extra_params
        )

    start_setpoint = state.position_setpoint
    motion_profile = args.action_motion_profile
    ramp_time_s = args.action_ramp_time if motion_profile == "s_curve" else 0.0
    selected = profile(args.validation_suite, args.validation_best_ramp)
    if args.validation_suite.endswith("UCIS_FH2"):
        selected = profile(args.validation_suite, args.validation_best_ramp)
        design_rope_length_m = float(selected["design_rope_length_m"])
        coefficients = finite_horizon2_coefficients(design_rope_length_m)
        GLOBALS["trajectory_progress"](0.0, duration_s, motion_profile, ramp_time_s)
        params = {
            "index": index,
            "action": spec.name,
            "raw": spec.raw,
            "start": asdict(start_setpoint),
            "end": asdict(target_setpoint),
            "duration_s": duration_s,
            "trajectory_profile": motion_profile,
            "ramp_time_s": ramp_time_s,
            "input_shaper": "fhv2_full_horizon_convex_frequency_uncertainty_trajectory",
            "input_shaper_frequency_source": "bounded_ratio_interval_offline_safe_degradation",
            "input_shaper_damping_ratio": 0.10,
            "input_shaper_frequency_ratio_min": 0.90,
            "input_shaper_frequency_ratio_max": 1.20,
            "input_shaper_design_frequency_ratios": [0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20],
            "input_shaper_base_duration_s": duration_s,
            "input_shaper_impulse_times_s": [],
            "input_shaper_weights": [],
            "finite_horizon_basis": "u^(k+3)*(1-u)^(18-k), k=0..15",
            "finite_horizon_order": 16,
            "finite_horizon_coefficients": list(coefficients),
            "finite_horizon_tracking_weight": 20.0,
            "finite_horizon_terminal_weight": 5.0,
            "design_rope_length_m": design_rope_length_m,
            "physical_rope_length_m": float(selected["rope_length_m"]),
            "payload_mass_kg": float(selected["payload_mass_kg"]),
        }
        params.update(extra_params)
        GLOBALS["run_setpoint_phase"](
            master,
            state,
            events,
            name,
            duration_s,
            lambda elapsed_s: finite_horizon2_transition_setpoint(
                start_setpoint, target_setpoint, elapsed_s, duration_s,
                motion_profile, ramp_time_s, coefficients,
            ),
            params,
        )
        return target_setpoint

    if args.validation_suite.endswith("UCIS_FH"):
        coefficients = finite_horizon_coefficients(float(selected["rope_length_m"]))
        # The direct trajectory keeps the full six-second horizon.  Unlike the
        # earlier causal shaper, it does not compress the base move to make
        # room for delayed impulses.
        GLOBALS["trajectory_progress"](0.0, duration_s, motion_profile, ramp_time_s)
        params = {
            "index": index,
            "action": spec.name,
            "raw": spec.raw,
            "start": asdict(start_setpoint),
            "end": asdict(target_setpoint),
            "duration_s": duration_s,
            "trajectory_profile": motion_profile,
            "ramp_time_s": ramp_time_s,
            "input_shaper": "finite_horizon_frequency_uncertainty_constrained_trajectory",
            "input_shaper_frequency_source": "bounded_ratio_interval_offline_finite_horizon",
            "input_shaper_damping_ratio": 0.10,
            "input_shaper_frequency_ratio_min": 0.90,
            "input_shaper_frequency_ratio_max": 1.20,
            "input_shaper_design_frequency_ratios": [0.90, 0.95, 1.00, 1.05, 1.10, 1.15, 1.20],
            "input_shaper_base_duration_s": duration_s,
            "input_shaper_impulse_times_s": [],
            "input_shaper_weights": [],
            "finite_horizon_basis": "u^(k+3)*(1-u)^(14-k), k=0..11",
            "finite_horizon_coefficients": list(coefficients),
            "finite_horizon_residual_weight": 1.0,
            "finite_horizon_acceleration_weight": 0.01,
            "finite_horizon_jerk_weight": 0.001,
            "finite_horizon_regularization": 1.0e-6,
        }
        params.update(extra_params)
        GLOBALS["run_setpoint_phase"](
            master,
            state,
            events,
            name,
            duration_s,
            lambda elapsed_s: finite_horizon_transition_setpoint(
                start_setpoint, target_setpoint, elapsed_s, duration_s,
                motion_profile, ramp_time_s, coefficients,
            ),
            params,
        )
        return target_setpoint

    spacing_s, zvd_weights = zvd_impulses(float(selected["rope_length_m"]))
    if args.validation_suite.endswith("UCIS_EI"):
        design_rope_length_m = float(selected["design_rope_length_m"])
        impulse_times_s, weights = extra_insensitive_impulses(design_rope_length_m)
        shaper_type = "extra_insensitive_three_impulse"
        frequency_source = "nominal_rope_length_classic_ei"
    elif args.validation_suite == "STD_L06_PID_AS_UCIS_FUCI":
        impulse_times_s, weights = frequency_uncertainty_constrained_impulses(
            float(selected["rope_length_m"])
        )
        shaper_type = "positive_three_impulse_frequency_uncertainty_constrained"
        frequency_source = "bounded_ratio_interval_offline_constraint"
    else:
        impulse_times_s = (0.0, spacing_s, 2.0 * spacing_s)
        weights = zvd_weights
        shaper_type = "damped_zvd_equal_total_time"
        frequency_source = "confidence_fallback_nominal"
    if args.validation_suite.endswith("UCIS_EI"):
        spacing_s, _ = zvd_impulses(float(selected["design_rope_length_m"]))
    base_duration_s = duration_s - impulse_times_s[-1]

    if base_duration_s <= 2.0 * ramp_time_s + 0.05:
        raise RuntimeError(
            f"equal-duration ZVD has insufficient base trajectory time: {base_duration_s:.3f}s"
        )

    GLOBALS["trajectory_progress"](0.0, base_duration_s, motion_profile, ramp_time_s)
    params = {
        "index": index,
        "action": spec.name,
        "raw": spec.raw,
        "start": asdict(start_setpoint),
        "end": asdict(target_setpoint),
        "duration_s": duration_s,
        "trajectory_profile": motion_profile,
        "ramp_time_s": ramp_time_s,
        "input_shaper": shaper_type,
        "input_shaper_frequency_source": frequency_source,
        "input_shaper_damping_ratio": 0.10,
        "input_shaper_spacing_s": spacing_s,
        "input_shaper_impulse_times_s": list(impulse_times_s),
        "input_shaper_weights": list(weights),
        "input_shaper_base_duration_s": base_duration_s,
        "input_shaper_frequency_ratio_min": 0.90 if args.validation_suite.endswith("FUCI") else 1.0,
        "input_shaper_frequency_ratio_max": 1.20 if args.validation_suite.endswith("FUCI") else 1.0,
        "input_shaper_residual_limit": 0.10 if args.validation_suite.endswith("FUCI") else 0.0,
    }
    params.update(extra_params)
    GLOBALS["run_setpoint_phase"](
        master,
        state,
        events,
        name,
        duration_s,
        lambda elapsed_s: input_shaped_transition_setpoint(
            start_setpoint, target_setpoint, elapsed_s, duration_s, base_duration_s,
            motion_profile, ramp_time_s, impulse_times_s, weights,
        ),
        params,
    )
    return target_setpoint


GLOBALS["run_action_transition"] = run_action_transition_with_zvd


if __name__ == "__main__":
    raise SystemExit(GLOBALS["main"]())
