#!/usr/bin/env python3
"""Launch the four fresh-instance ZD680 VFB task-book SITL runs in order."""

from __future__ import annotations

import runpy
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
SOURCE = REPO_ROOT / "Tools" / "simulation" / "gz" / "tools" / "zd680_validation_suite.py"
RECORDER = Path(__file__).with_name("zd680_vfb_recorder.py")
API = runpy.run_path(str(SOURCE), run_name="zd680_vfb_suite_api")

MODES = ("VFB_L06_W00", "VFB_L06_W05", "VFB_L08_W00", "VFB_L08_W05")
CURRENT_MODE = ""


def profile(mode: str, best_ramp_s: float = 1.0):
    recorder_api = runpy.run_path(str(RECORDER), run_name="zd680_vfb_profile_api")
    return recorder_api["profile"](mode, best_ramp_s)


def frozen_params(mode: str):
    selected = profile(mode)
    recorder_api = runpy.run_path(str(RECORDER), run_name="zd680_vfb_params_api")
    int_params, float_params = recorder_api["controller_parameters"]("FULL", float(selected["rope_length_m"]))
    float_params["MC_PLADRC_VFB_W"] = float(selected["velocity_feedback_weight"])
    estimator_int, estimator_float = recorder_api["estimator_parameters"]()
    int_params.update(estimator_int)
    float_params.update(estimator_float)
    return int_params, float_params


original_send = API["PtyProcess"].send


def send_with_pre_takeoff_freeze(self, command: str) -> None:
    if command == "commander takeoff":
        int_params, float_params = frozen_params(CURRENT_MODE)
        print(f"[task-book] freezing {CURRENT_MODE} controller parameters before takeoff")
        for name, value in int_params.items():
            original_send(self, f"param set {name} {int(value)}")
        for name, value in float_params.items():
            original_send(self, f"param set {name} {float(value):.9g}")
    original_send(self, command)


original_run_mode = API["run_mode"]


def run_mode(mode: str, args):
    global CURRENT_MODE
    CURRENT_MODE = mode
    return original_run_mode(mode, args)


GLOBALS = API["main"].__globals__
GLOBALS["RECORDER"] = RECORDER
GLOBALS["VALIDATION_MODES"] = MODES
GLOBALS["validation_suite_profile"] = profile
GLOBALS["PtyProcess"].send = send_with_pre_takeoff_freeze
GLOBALS["run_mode"] = run_mode


if __name__ == "__main__":
    raise SystemExit(GLOBALS["main"]())
