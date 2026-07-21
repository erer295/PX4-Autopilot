#!/usr/bin/env python3
"""Task-book recorder adapter for ZD680 EKF-ESO VFB validation runs."""

from __future__ import annotations

import runpy
import sys
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
    "STD_L06_FULL_W05",
    "STD_L06_FULL_W10_OBS_DECOUPLE",
)


def velocity_feedback_weight(mode: str) -> float:
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
    raise ValueError(f"unsupported VFB weight label: {mode}")


def profile(mode: str, best_ramp_s: float = 1.0):
    if mode not in MODES or mode == "NONE":
        raise ValueError(f"unsupported VFB validation mode: {mode}")
    return {
        "mode": mode,
        "controller": "PID_AS" if mode in ("STD_L06_PID_AS", "STD_L06_PID_AS_PAS") else "FULL",
        "rope_length_m": 0.8 if "L08" in mode else 0.6,
        "payload_mass_kg": 0.5,
        "actions": "hold:5;north:2:6;hold:5;east:2:6;hold:5;south:2:6;hold:5;west:2:6;hold:10",
        "motion_profile": "s_curve",
        "ramp_time_s": 1.0,
        "wrench_schedule": "",
        "velocity_feedback_weight": velocity_feedback_weight(mode),
        "initial_condition_profile": "strict_matched_taskbook_v1",
        "initial_angle_rms_max_deg": 0.75,
        "initial_rate_rms_max_rad_s": 0.06,
        "initial_current_angle_max_deg": 0.75,
        "initial_current_rate_max_rad_s": 0.06,
        "initial_xy_error_max_m": 0.10,
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
        str(selected["controller"]), float(selected["rope_length_m"])
    )
    if mode == "STD_L06_PID_AS_PAS":
        # Stage-4 isolation: this mode differs from PID+AS only by enabling
        # the already frozen ACTIVE PAS path.
        int_params["MC_HANG_PAS_MD"] = 2
        int_params["MC_HANG_PAS_POS"] = 1
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
)
GLOBALS["validation_suite_profile"] = profile
GLOBALS["validation_controller_parameters"] = controller_parameters
GLOBALS["validation_mode_parameters"] = parameters_for_mode
GLOBALS["apply_validation_params"] = apply_params

ORIGINAL_APPLY_EXPERIMENT_PRESET = GLOBALS["apply_experiment_preset"]


def apply_experiment_preset_with_strict_initial_pairing(args, parser) -> None:
    ORIGINAL_APPLY_EXPERIMENT_PRESET(args, parser)
    if args.validation_suite in MODES and args.validation_suite != "NONE":
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


if __name__ == "__main__":
    raise SystemExit(GLOBALS["main"]())
