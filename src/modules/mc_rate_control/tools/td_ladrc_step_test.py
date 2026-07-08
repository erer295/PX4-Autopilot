#!/usr/bin/env python3
"""
Run a repeatable TD-LADRC position-step test in PX4 SITL.

The script sets the LADRC/TD parameters through MAVSDK, takes off, switches to
Offboard position control, then commands X/Y position steps that exercise the
inner-loop rate setpoint transition. PX4 ULog records the TD debug_array with
name "ladrc_td" and id 682.
"""

import argparse
import asyncio
from dataclasses import dataclass

from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw


@dataclass
class Step:
    name: str
    north_m: float
    east_m: float
    down_m: float
    hold_s: float


@dataclass
class Origin:
    north_m: float
    east_m: float
    down_m: float


async def wait_connected(drone: System) -> None:
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("[INFO] PX4 connected")
            return


async def wait_health(drone: System, timeout_s: float) -> None:
    async def _wait() -> None:
        async for health in drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                print("[INFO] global position and home position are OK")
                return

    try:
        await asyncio.wait_for(_wait(), timeout=timeout_s)
    except asyncio.TimeoutError:
        print("[WARN] health wait timed out; continuing for SITL")


async def get_position_ned(drone: System, timeout_s: float) -> Origin:
    async def _wait() -> Origin:
        async for position_velocity in drone.telemetry.position_velocity_ned():
            position = position_velocity.position
            return Origin(position.north_m, position.east_m, position.down_m)

        raise RuntimeError("position_velocity_ned stream ended")

    return await asyncio.wait_for(_wait(), timeout=timeout_s)


async def set_int_param(drone: System, name: str, value: int, optional: bool = False) -> None:
    try:
        await drone.param.set_param_int(name, int(value))
        print(f"[PARAM] {name} = {int(value)}")
    except Exception as exc:
        if optional:
            print(f"[WARN] optional param {name} not set: {exc}")
            return
        raise


async def get_int_param(drone: System, name: str, optional: bool = False) -> int | None:
    try:
        value = await drone.param.get_param_int(name)
        print(f"[PARAM] readback {name} = {int(value)}")
        return int(value)
    except Exception as exc:
        if optional:
            print(f"[WARN] optional param {name} not read: {exc}")
            return None
        raise


async def set_float_param(drone: System, name: str, value: float, optional: bool = False) -> None:
    try:
        await drone.param.set_param_float(name, float(value))
        print(f"[PARAM] {name} = {float(value):.6g}")
    except Exception as exc:
        if optional:
            print(f"[WARN] optional param {name} not set: {exc}")
            return
        raise


async def apply_params(drone: System, args) -> None:
    await set_int_param(drone, "MC_LADRC_EN", 1)
    await set_int_param(drone, "MC_LADRC_TD_MODE", args.td_mode)
    await get_int_param(drone, "MC_LADRC_TD_MODE")
    await set_int_param(drone, "MC_RBF_INJECT_EN", 0, optional=True)
    await set_int_param(drone, "MC_RBF_LEARN_EN", 0, optional=True)
    await set_int_param(drone, "MC_HANG_AS_EN", 1 if args.hang_as_en else 0, optional=True)

    await set_float_param(drone, "MC_LADRC_TD_FW_R", args.td_fw_r)
    await set_float_param(drone, "MC_LADRC_TD_FW_P", args.td_fw_p)
    await set_float_param(drone, "MC_LADRC_TD_FA_R", args.td_fa_r)
    await set_float_param(drone, "MC_LADRC_TD_FA_P", args.td_fa_p)
    await set_float_param(drone, "MC_LADRC_TD_SW_R", args.td_sw_r)
    await set_float_param(drone, "MC_LADRC_TD_SW_P", args.td_sw_p)
    await set_float_param(drone, "MC_LADRC_TD_SA_R", args.td_sa_r)
    await set_float_param(drone, "MC_LADRC_TD_SA_P", args.td_sa_p)
    await set_float_param(drone, "MC_LADRC_TD_W_Y", args.td_w_y)
    await set_float_param(drone, "MC_LADRC_TD_ZETA", args.td_zeta)
    await set_float_param(drone, "MC_LADRC_TD_A_Y", args.td_a_y)
    await set_float_param(drone, "MC_LADRC_TD_DLY", args.td_delay)
    await set_float_param(drone, "MC_LADRC_TD_RAMP", args.td_ramp)
    await set_float_param(drone, "MC_LADRC_TD_ATT", args.td_att)
    await set_float_param(drone, "MC_LADRC_TD_RATE", args.td_rate)
    await set_float_param(drone, "MC_LADRC_TD_ERR", args.td_err)
    await set_float_param(drone, "MC_LADRC_TD_FF", 0.0)

    if args.conservative_ladrc:
        await set_float_param(drone, "MC_LADRC_WC_R", 5.5)
        await set_float_param(drone, "MC_LADRC_WC_P", 5.5)
        await set_float_param(drone, "MC_LADRC_WO_R", 16.0)
        await set_float_param(drone, "MC_LADRC_WO_P", 16.0)
        await set_float_param(drone, "MC_LADRC_D_R", 0.003)
        await set_float_param(drone, "MC_LADRC_D_P", 0.003)
        await set_float_param(drone, "MC_LADRC_LIM_R", 0.30)
        await set_float_param(drone, "MC_LADRC_LIM_P", 0.30)


async def hold_position(drone: System, step: Step, rate_hz: float) -> None:
    print(
        f"[STEP] {step.name}: north={step.north_m:.2f} east={step.east_m:.2f} "
        f"alt={-step.down_m:.2f} hold={step.hold_s:.1f}s"
    )

    period_s = 1.0 / max(rate_hz, 1.0)
    deadline = asyncio.get_running_loop().time() + max(step.hold_s, 0.0)

    while asyncio.get_running_loop().time() < deadline:
        await drone.offboard.set_position_ned(
            PositionNedYaw(step.north_m, step.east_m, step.down_m, 0.0)
        )
        await asyncio.sleep(period_s)


def build_steps(args, origin: Origin) -> list[Step]:
    amp = abs(args.step_m)

    return [
        Step("settle_origin", origin.north_m, origin.east_m, origin.down_m, args.settle_s),
        Step("x_step_positive", origin.north_m + amp, origin.east_m, origin.down_m, args.hold_s),
        Step("x_return_brake", origin.north_m, origin.east_m, origin.down_m, args.hold_s),
        Step("x_step_negative", origin.north_m - amp, origin.east_m, origin.down_m, args.hold_s),
        Step("x_return_brake_2", origin.north_m, origin.east_m, origin.down_m, args.hold_s),
        Step("y_step_positive", origin.north_m, origin.east_m + amp, origin.down_m, args.hold_s),
        Step("y_return_brake", origin.north_m, origin.east_m, origin.down_m, args.hold_s),
        Step("diagonal_step", origin.north_m + amp, origin.east_m + amp, origin.down_m, args.hold_s),
        Step("diagonal_return", origin.north_m, origin.east_m, origin.down_m, args.post_s),
    ]


async def main() -> None:
    parser = argparse.ArgumentParser(description="TD-LADRC position-step tuner")
    parser.add_argument("--connection", default="udpin://0.0.0.0:14540")
    parser.add_argument("--td-en", type=int, default=1, help="legacy alias: 1 maps to TD mode 2, 0 maps to mode 0")
    parser.add_argument("--td-mode", type=int, choices=[0, 1, 2, 3], default=None)
    parser.add_argument("--td-fw-r", type=float, default=15.0)
    parser.add_argument("--td-fw-p", type=float, default=15.0)
    parser.add_argument("--td-fa-r", type=float, default=80.0)
    parser.add_argument("--td-fa-p", type=float, default=80.0)
    parser.add_argument("--td-sw-r", type=float, default=8.0)
    parser.add_argument("--td-sw-p", type=float, default=8.0)
    parser.add_argument("--td-sa-r", type=float, default=30.0)
    parser.add_argument("--td-sa-p", type=float, default=30.0)
    parser.add_argument("--td-w-r", type=float, default=None, help="legacy alias for --td-fw-r")
    parser.add_argument("--td-w-p", type=float, default=None, help="legacy alias for --td-fw-p")
    parser.add_argument("--td-a-r", type=float, default=None, help="legacy alias for --td-fa-r")
    parser.add_argument("--td-a-p", type=float, default=None, help="legacy alias for --td-fa-p")
    parser.add_argument("--td-w-y", type=float, default=5.0)
    parser.add_argument("--td-zeta", type=float, default=1.0)
    parser.add_argument("--td-a-y", type=float, default=8.0)
    parser.add_argument("--td-delay", type=float, default=4.0)
    parser.add_argument("--td-ramp", type=float, default=4.0)
    parser.add_argument("--td-att", type=float, default=0.15)
    parser.add_argument("--td-rate", type=float, default=0.5)
    parser.add_argument("--td-err", type=float, default=0.5)
    parser.add_argument("--hang-as-en", type=int, default=0, help="keep 0 for TD-only tests")
    parser.add_argument("--conservative-ladrc", action="store_true")
    parser.add_argument("--altitude", type=float, default=5.0)
    parser.add_argument(
        "--fixed-origin",
        action="store_true",
        help="use NED origin 0/0 and --altitude instead of the current vehicle position",
    )
    parser.add_argument("--takeoff-wait-s", type=float, default=10.0)
    parser.add_argument("--settle-s", type=float, default=6.0)
    parser.add_argument("--hold-s", type=float, default=8.0)
    parser.add_argument("--post-s", type=float, default=10.0)
    parser.add_argument("--step-m", type=float, default=1.5)
    parser.add_argument("--setpoint-rate", type=float, default=20.0)
    args = parser.parse_args()
    args.td_mode = args.td_mode if args.td_mode is not None else (2 if args.td_en else 0)

    if args.td_w_r is not None:
        args.td_fw_r = args.td_w_r

    if args.td_w_p is not None:
        args.td_fw_p = args.td_w_p

    if args.td_a_r is not None:
        args.td_fa_r = args.td_a_r

    if args.td_a_p is not None:
        args.td_fa_p = args.td_a_p

    drone = System()
    print(f"[INFO] connecting to {args.connection}")
    await drone.connect(system_address=args.connection)
    await wait_connected(drone)
    await wait_health(drone, timeout_s=30.0)
    await apply_params(drone, args)

    await drone.action.set_takeoff_altitude(abs(args.altitude))
    print("[INFO] arming")
    await drone.action.arm()
    print(f"[INFO] takeoff to {abs(args.altitude):.1f} m")
    await drone.action.takeoff()
    await asyncio.sleep(args.takeoff_wait_s)

    if args.fixed_origin:
        origin = Origin(0.0, 0.0, -abs(args.altitude))

    else:
        origin = await get_position_ned(drone, timeout_s=5.0)

    print(
        f"[INFO] offboard origin: north={origin.north_m:.2f} east={origin.east_m:.2f} "
        f"alt={-origin.down_m:.2f}"
    )

    start_sp = PositionNedYaw(origin.north_m, origin.east_m, origin.down_m, 0.0)
    await drone.offboard.set_position_ned(start_sp)

    try:
        print("[INFO] starting offboard")
        await drone.offboard.start()
    except OffboardError as exc:
        print(f"[ERROR] offboard start failed: {exc._result.result}")
        await drone.action.land()
        raise

    try:
        for step in build_steps(args, origin):
            await hold_position(drone, step, args.setpoint_rate)
    finally:
        print("[INFO] stopping offboard and landing")
        try:
            await drone.offboard.stop()
        except Exception as exc:
            print(f"[WARN] offboard stop failed: {exc}")

        await drone.action.land()
        await asyncio.sleep(8.0)

    print("[INFO] test complete; inspect ULog debug_array name=ladrc_td id=682")


if __name__ == "__main__":
    asyncio.run(main())
