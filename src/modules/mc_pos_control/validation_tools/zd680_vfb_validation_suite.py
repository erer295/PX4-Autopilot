#!/usr/bin/env python3
"""Launch fresh-instance ZD680 VFB task-book SITL runs in order."""

from __future__ import annotations

import runpy
import os
import subprocess
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[4]
SOURCE = REPO_ROOT / "Tools" / "simulation" / "gz" / "tools" / "zd680_validation_suite.py"
RECORDER = Path(__file__).with_name("zd680_vfb_recorder.py")
API = runpy.run_path(str(SOURCE), run_name="zd680_vfb_suite_api")

MODES = (
    "VFB_L06_W00", "VFB_L06_W05", "VFB_L08_W00", "VFB_L08_W25", "VFB_L08_W05",
    "STD_L06_PID_AS", "STD_L06_PID_AS_PAS", "STD_L06_PID_AS_FSO_SHADOW",
    "STD_L06_PID_AS_FSOPC", "STD_L06_PID_AS_FSOPC_K20", "STD_L06_PID_AS_FSOPC_K40",
    "STD_L06_PID_AS_FSOPC_K60",
    "STD_L06_PID_AS_FSOJPC_K05", "STD_L06_PID_AS_FSOJPC_K10", "STD_L06_PID_AS_FSOJPC_K15",
    "STD_L06_PID_AS_FSOJPC_K20",
    "STD_L06_PID_AS_FSOUSC_K20",
    "STD_L06_PID_AS_FSOUSC_READY",
    "STD_L08_PID_AS", "STD_L08_PID_AS_FSOUSC_K20",
    "STD_L06_PID_AS_HGS_K40", "STD_L06_PID_AS_HGS_READY", "STD_L08_PID_AS_HGS_K40",
    "STD_L06_PID_AS_HGS2_ORACLE_K40", "STD_L06_PID_AS_HGS2_ORACLE_READY",
    "STD_L06_PID_AS_HGS2_RLS_SHADOW", "STD_L06_PID_AS_HGS2_RLS_READY",
    "STD_L06_PID_AS_HGS2_BASE",
    "STD_L06_PID_AS_UCIS_BASE", "STD_L06_PID_AS_UCIS_FIXED",
    "STD_L06_PID_AS_UCIS_FULL", "STD_L06_PID_AS_UCIS_FUCI",
    "STD_L06_PID_AS_UCIS_FH", "STD_L08_PID_AS_UCIS_FH", "STD_L06_PID_AS_UCIS_READY",
    "STD_L06_FULL_W05", "STD_L06_FULL_W10_OBS_DECOUPLE",
    "STD_L05_PID_AS_UCIS_FH2", "STD_L06_PID_AS_UCIS_FH2", "STD_L08_PID_AS_UCIS_FH2",
    "STD_L06P75_PID_AS_UCIS_FH2", "STD_MM070_PID_AS_UCIS_FH2",
    "STD_L06_PID_AS_UCIS_EI", "STD_MM070_PID_AS_UCIS_EI",
)
CURRENT_MODE = ""


def profile(mode: str, best_ramp_s: float = 1.0):
    recorder_api = runpy.run_path(str(RECORDER), run_name="zd680_vfb_profile_api")
    return recorder_api["profile"](mode, best_ramp_s)


def frozen_params(mode: str):
    recorder_api = runpy.run_path(str(RECORDER), run_name="zd680_vfb_params_api")
    return recorder_api["parameters_for_mode"](mode)


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


def residual_sim_processes():
    """Return only PX4/Gazebo/recorder processes that can contaminate a run."""
    result = subprocess.run(
        ["ps", "-eo", "pid=,comm=,args="], check=True, text=True,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    )
    ancestors = {os.getpid(), os.getppid()}
    residuals = []
    for line in result.stdout.splitlines():
        fields = line.strip().split(None, 2)
        if len(fields) < 3:
            continue
        pid, command, arguments = int(fields[0]), fields[1], fields[2]
        if pid in ancestors:
            continue
        is_px4 = command == "px4"
        is_gazebo = command in {"gzserver", "gzclient"} or "gz sim -r" in arguments
        is_recorder = "position_offboard_flight_recorder.py" in arguments
        if is_px4 or is_gazebo or is_recorder:
            residuals.append(line.strip())
    return residuals


def run_mode(mode: str, args):
    global CURRENT_MODE
    residuals = residual_sim_processes()
    if residuals:
        raise RuntimeError("residual PX4/Gazebo/recorder process before run: " + " | ".join(residuals))
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
