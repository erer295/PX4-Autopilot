#!/usr/bin/env python3
"""Launch fresh, reproducible ZD680_HANG PID/LADRC gate runs.

This is deliberately a thin task-specific launcher around the project's existing
position_offboard_flight_recorder.py.  It freezes the outer position controller,
energy anti-swing parameters, disabled experimental branches, and the inner-loop
parameter set before takeoff.  Every run gets a fresh PX4/Gazebo process.
"""

from __future__ import annotations

import argparse
import json
import os
import pty
import queue
import shutil
import signal
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional


REPO_ROOT = Path(__file__).resolve().parents[5]
RESULT_ROOT = Path(__file__).resolve().parent
RECORDER = REPO_ROOT / "Tools/simulation/gz/tools/position_offboard_flight_recorder.py"
PX4_PYMAVLINK = REPO_ROOT / "src/modules/mavlink/mavlink"
if str(PX4_PYMAVLINK) not in sys.path:
    sys.path.insert(0, str(PX4_PYMAVLINK))
from pymavlink import mavutil  # noqa: E402

D0_ACTIONS = "hold:5;north:2:6;hold:5;east:2:6;hold:5;south:2:6;hold:5;west:2:6;hold:10"
SMOKE_ACTIONS = "hold:8"

PARAMETER_SETS: Dict[str, Dict[str, float]] = {
    # T0 is the actual pre-test parameters.bson configuration read back on the
    # first Stage-0 launch, and matches the repository's prior safe TD-LADRC test.
    "T0": {"b0_r": 120.0, "b0_p": 120.0, "wc_r": 5.5, "wc_p": 5.5, "wo_r": 16.0, "wo_p": 16.0, "lim_r": 0.30, "lim_p": 0.30},
    # T1-T5 are the complete, bounded D0-only screening set.  The existing
    # D=0.003 damping and 0.30 authority remain frozen, so only b0/wc/wo vary.
    "T1": {"b0_r": 120.0, "b0_p": 120.0, "wc_r": 5.5, "wc_p": 5.5, "wo_r": 13.5, "wo_p": 13.5, "lim_r": 0.30, "lim_p": 0.30},
    "T2": {"b0_r": 140.0, "b0_p": 140.0, "wc_r": 5.5, "wc_p": 5.5, "wo_r": 16.5, "wo_p": 16.5, "lim_r": 0.30, "lim_p": 0.30},
    "T3": {"b0_r": 120.0, "b0_p": 120.0, "wc_r": 4.5, "wc_p": 4.5, "wo_r": 13.5, "wo_p": 13.5, "lim_r": 0.30, "lim_p": 0.30},
    "T4": {"b0_r": 140.0, "b0_p": 140.0, "wc_r": 4.5, "wc_p": 4.5, "wo_r": 13.5, "wo_p": 13.5, "lim_r": 0.30, "lim_p": 0.30},
    "T5": {"b0_r": 160.0, "b0_p": 160.0, "wc_r": 5.0, "wc_p": 5.0, "wo_r": 15.0, "wo_p": 15.0, "lim_r": 0.30, "lim_p": 0.30},
}

GROUPS = {
    "C0": {"ladrc": 0, "as": 0},
    "C1": {"ladrc": 1, "as": 0},
    "C2": {"ladrc": 0, "as": 1},
    "C3": {"ladrc": 1, "as": 1},
}

SAFE_POST_RUN_PARAMETERS = {
    "MC_LADRC_EN": 0,
    "MC_HANG_AS_EN": 1,
    "MC_LADRC_TD_MODE": 0,
    "MC_RBF_EN": 0,
    "MC_RBF_INJECT_EN": 0,
    "MC_RBF_LEARN_EN": 0,
    "MC_HANG_PAS_MD": 0,
    "MC_HANG_PAS_POS": 0,
    "MC_PLADRC_EN": 0,
    "MC_PLADRC_TD_EN": 0,
}


class PtyProcess:
    def __init__(self, command: List[str], cwd: Path, env: Dict[str, str], log_path: Path):
        self.command = command
        self.cwd = cwd
        self.env = env
        self.log_path = log_path
        self.master_fd: Optional[int] = None
        self.process: Optional[subprocess.Popen] = None
        self.messages: "queue.Queue[str]" = queue.Queue()
        self.tail = ""

    def start(self) -> None:
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        master_fd, slave_fd = pty.openpty()
        self.master_fd = master_fd
        self.process = subprocess.Popen(
            self.command,
            cwd=self.cwd,
            env=self.env,
            stdin=slave_fd,
            stdout=slave_fd,
            stderr=slave_fd,
            start_new_session=True,
            close_fds=True,
        )
        os.close(slave_fd)

        def read_output() -> None:
            assert self.master_fd is not None
            with self.log_path.open("ab") as output:
                while True:
                    try:
                        payload = os.read(self.master_fd, 4096)
                    except OSError:
                        break
                    if not payload:
                        break
                    output.write(payload)
                    output.flush()
                    decoded = payload.decode("utf-8", errors="replace")
                    sys.stdout.write(decoded)
                    sys.stdout.flush()
                    self.messages.put(decoded)

        threading.Thread(target=read_output, name="gate-px4-reader", daemon=True).start()

    def send(self, command: str) -> None:
        if self.master_fd is None:
            raise RuntimeError("PX4 process is not running")
        print(f"[PX4] {command}")
        os.write(self.master_fd, (command + "\n").encode())

    def wait_for(self, marker: str, timeout_s: float) -> bool:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if marker in self.tail:
                return True
            if self.process is not None and self.process.poll() is not None:
                return False
            try:
                chunk = self.messages.get(timeout=min(0.5, max(0.01, deadline - time.monotonic())))
            except queue.Empty:
                continue
            self.tail = (self.tail + chunk)[-30000:]
        return marker in self.tail

    def stop(self) -> None:
        if self.process is None:
            return
        if self.process.poll() is None:
            try:
                self.send("shutdown")
                self.process.wait(timeout=12)
            except (OSError, subprocess.TimeoutExpired):
                try:
                    os.killpg(self.process.pid, signal.SIGTERM)
                except ProcessLookupError:
                    pass
                try:
                    self.process.wait(timeout=8)
                except subprocess.TimeoutExpired:
                    try:
                        os.killpg(self.process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    self.process.wait(timeout=5)
        if self.master_fd is not None:
            try:
                os.close(self.master_fd)
            except OSError:
                pass


class GcsHeartbeat:
    """Provide the local GCS heartbeat normally supplied by QGroundControl."""

    def __init__(self) -> None:
        self.stop_event = threading.Event()
        self.thread: Optional[threading.Thread] = None

    def start(self) -> None:
        def publish() -> None:
            connection = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255)
            while not self.stop_event.is_set():
                connection.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                self.stop_event.wait(0.5)
            connection.close()

        self.thread = threading.Thread(target=publish, name="gate-gcs-heartbeat", daemon=True)
        self.thread.start()

    def stop(self) -> None:
        self.stop_event.set()
        if self.thread is not None:
            self.thread.join(timeout=2)


def newest_run(root: Path, started_wall_s: float) -> Optional[Path]:
    candidates = [p for p in root.glob("*/*") if p.is_dir() and p.stat().st_mtime >= started_wall_s - 2]
    return max(candidates, key=lambda p: p.stat().st_mtime) if candidates else None


def px4_parameters(group: str, parameter_set: str) -> Dict[str, float]:
    selection = GROUPS[group]
    tuning = PARAMETER_SETS[parameter_set]
    values: Dict[str, float] = {
        # Only the inner rate controller varies between each matched pair.
        "MC_LADRC_EN": selection["ladrc"],
        "MC_LADRC_B0_R": tuning["b0_r"],
        "MC_LADRC_B0_P": tuning["b0_p"],
        "MC_LADRC_B0_Y": 20.0,
        "MC_LADRC_WC_R": tuning["wc_r"],
        "MC_LADRC_WC_P": tuning["wc_p"],
        "MC_LADRC_WC_Y": 4.0,
        "MC_LADRC_WO_R": tuning["wo_r"],
        "MC_LADRC_WO_P": tuning["wo_p"],
        "MC_LADRC_WO_Y": 15.0,
        "MC_LADRC_LIM_R": tuning["lim_r"],
        "MC_LADRC_LIM_P": tuning["lim_p"],
        "MC_LADRC_LIM_Y": 0.20,
        "MC_LADRC_D_R": 0.003,
        "MC_LADRC_D_P": 0.003,
        "MC_LADRC_D_Y": 0.0,
        "MC_LADRC_TD_MODE": 0,
        "MC_RBF_EN": 0,
        "MC_RBF_INJECT_EN": 0,
        "MC_RBF_LEARN_EN": 0,
        # Mature PX4 position PID on X/Y/Z; all former outer LADRC/TD branches off.
        "MC_PLADRC_EN": 0,
        "MC_PLADRC_TD_EN": 0,
        # Frozen accepted energy-AS structure; PAS/VFB-related injection is off.
        "MC_HANG_AS_EN": selection["as"],
        "MC_HANG_MODE": 2,
        "MC_HANG_FRQ_EN": 0,
        "MC_HANG_SIGN_X": -1,
        "MC_HANG_SIGN_Y": -1,
        "MC_HANG_PAS_MD": 0,
        "MC_HANG_PAS_POS": 0,
        "MC_HANG_LEN": 0.60,
        "MC_HANG_WC_R": 0.25,
        "MC_HANG_WO_R": 0.50,
        "MC_HANG_WO_MIN": 2.0,
        "MC_HANG_K_ANG": 0.0,
        "MC_HANG_K_RATE": 0.6,
        "MC_HANG_ZETA": 0.10,
        "MC_HANG_E_MIN": 0.0,
        "MC_HANG_E_FULL": 0.01,
        "MC_HANG_ACC_LIM": 0.20,
        "MC_HANG_ACC_SLW": 2.0,
        "MC_HANG_LPF_HZ": 4.0,
        "MC_HANG_TOT_A": 0.8,
        # Freeze the simulated supply so run order does not bias authority.
        "SIM_BAT_DRAIN": 3600.0,
        "SIM_BAT_MIN_PCT": 100.0,
        "MIS_TAKEOFF_ALT": 10.0,
    }
    return values


def run_one(args, group: str, parameter_set: str, repetition: int) -> Dict[str, object]:
    started = datetime.now()
    tag = f"{args.stage}_{args.condition}_{group}_{parameter_set}_r{repetition:02d}_{started:%Y%m%d_%H%M%S}"
    launch_dir = args.output_dir / "launcher"
    run_output = args.output_dir / "runs" / tag
    summary_output = args.output_dir / "summaries"
    launcher_log = launch_dir / f"{tag}.log"
    recorder_log = launch_dir / f"{tag}_recorder.log"
    requested = px4_parameters(group, parameter_set)
    process = PtyProcess(["make", "px4_sitl", "gz_zd680_hang"], REPO_ROOT, {**os.environ, "HEADLESS": "1", "PX4_MAV_BROADCAST": "1"}, launcher_log)
    heartbeat = GcsHeartbeat()
    run_dir: Optional[Path] = None
    error = ""
    recorder_rc: Optional[int] = None

    try:
        heartbeat.start()
        process.start()
        if not process.wait_for("Ready for takeoff!", args.startup_timeout):
            raise RuntimeError("PX4/Gazebo startup did not reach Ready for takeoff")

        for name, value in requested.items():
            rendered = str(int(value)) if float(value).is_integer() else f"{value:.9g}"
            process.send(f"param set {name} {rendered}")
        process.send("param show MC_LADRC*")
        process.send("param show MC_HANG*")
        process.send("param show MC_PLADRC*")
        time.sleep(2)
        process.send("commander takeoff")
        time.sleep(args.takeoff_wait)

        actions = SMOKE_ACTIONS if args.stage == "smoke" else D0_ACTIONS
        recorder_command = [
            sys.executable,
            str(RECORDER),
            "--output-dir", str(run_output),
            "--summary-dir", str(summary_output),
            "--profile", "actions",
            "--actions", actions,
            "--action-motion-profile", "s_curve",
            "--action-ramp-time", "1.0",
            "--world", "default",
            "--model-name", "zd680_hang_0",
            "--link-name", "base_link",
            "--hang-joint-log", "on",
            "--no-face-north-on-entry",
            "--hang-initial-gate",
            "--hang-initial-window", "3.0",
            "--hang-initial-stable-time", "3.0",
            "--hang-initial-angle-rms-deg", "0.3",
            "--hang-initial-rate-rms", "0.015",
            "--hang-initial-current-angle-deg", "1.0",
            "--hang-initial-current-rate", "0.06",
            "--xy-tolerance", "0.10",
            "--velocity-tolerance", "0.05",
            "--position-stable-time", "3.0",
            "--require-position",
            "--log-flush-wait", "3.0",
        ]
        print("[recorder] " + " ".join(recorder_command))
        recorder_started = time.time()
        completed = subprocess.run(recorder_command, cwd=REPO_ROOT, text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
        recorder_rc = completed.returncode
        recorder_log.parent.mkdir(parents=True, exist_ok=True)
        recorder_log.write_text(completed.stdout, encoding="utf-8")
        print(completed.stdout, end="")
        run_dir = newest_run(run_output, recorder_started)

        process.send("commander land")
        time.sleep(args.land_wait)
    except Exception as exc:
        error = f"{type(exc).__name__}: {exc}"
        print(f"ERROR {tag}: {error}", file=sys.stderr)
    finally:
        # Never leave a developer's next SITL launch in the last experimental
        # controller mode. This happens after recording/landing and therefore
        # cannot alter the archived run.
        if process.process is not None and process.process.poll() is None and "Ready for takeoff!" in process.tail:
            for name, value in SAFE_POST_RUN_PARAMETERS.items():
                process.send(f"param set {name} {value}")
            process.send("param save")
            time.sleep(1)
        process.stop()
        heartbeat.stop()
        time.sleep(args.cooldown)

    record: Dict[str, object] = {
        "tag": tag,
        "stage": args.stage,
        "condition": args.condition,
        "group": group,
        "parameter_set": parameter_set,
        "repetition": repetition,
        "started_local": started.isoformat(timespec="seconds"),
        "requested_parameters": requested,
        "recorder_returncode": recorder_rc,
        "run_dir": str(run_dir) if run_dir else "",
        "launcher_log": str(launcher_log),
        "recorder_log": str(recorder_log),
        "error": error,
    }
    if run_dir is not None:
        shutil.copy2(launcher_log, run_dir / "sitl_launcher.log")
        shutil.copy2(recorder_log, run_dir / "recorder_console.log")
        (run_dir / "gate_run.json").write_text(json.dumps(record, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    launch_dir.mkdir(parents=True, exist_ok=True)
    with (launch_dir / "run_results.jsonl").open("a", encoding="utf-8") as output:
        output.write(json.dumps(record, sort_keys=True) + "\n")
    return record


def parse_csv_list(text: str, allowed: Dict[str, object]) -> List[str]:
    values = [item.strip().upper() for item in text.split(",") if item.strip()]
    unknown = [item for item in values if item not in allowed]
    if unknown:
        raise argparse.ArgumentTypeError(f"unknown values: {','.join(unknown)}")
    return values


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--stage", choices=("smoke", "screen", "stage1"), required=True)
    parser.add_argument("--condition", default="D0")
    parser.add_argument("--groups", default="C0,C1,C2,C3")
    parser.add_argument("--parameter-sets", default="T0")
    parser.add_argument("--repetitions", type=int, default=1)
    parser.add_argument("--output-dir", type=Path, default=RESULT_ROOT)
    parser.add_argument("--startup-timeout", type=float, default=150.0)
    parser.add_argument("--takeoff-wait", type=float, default=22.0)
    parser.add_argument("--land-wait", type=float, default=18.0)
    parser.add_argument("--cooldown", type=float, default=4.0)
    parser.add_argument("--stop-on-error", action="store_true")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("--repetitions must be positive")
    groups = parse_csv_list(args.groups, GROUPS)
    parameter_sets = parse_csv_list(args.parameter_sets, PARAMETER_SETS)
    args.output_dir = args.output_dir.resolve()
    args.output_dir.mkdir(parents=True, exist_ok=True)

    results = []
    for repetition in range(1, args.repetitions + 1):
        for group in groups:
            selected_sets = parameter_sets if GROUPS[group]["ladrc"] else ["T0"]
            for parameter_set in selected_sets:
                result = run_one(args, group, parameter_set, repetition)
                results.append(result)
                if args.stop_on_error and (result["error"] or result["recorder_returncode"] != 0):
                    return 1
    return 0 if results and all(not r["error"] and r["recorder_returncode"] == 0 for r in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
