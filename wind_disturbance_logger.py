#!/usr/bin/env python3

import asyncio
import csv
import math
import subprocess
import time
from datetime import datetime
from mavsdk import System


# ============================================================
# 基本配置
# ============================================================

PX4_UDP_ADDRESS = "udpin://0.0.0.0:14540"

OUTPUT_CSV = f"px4_rbf_ladrc_disturbance_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"

# 记录频率，PlotJuggler 推荐 20~50 Hz
RECORD_HZ = 50.0

# ============================================================
# Gazebo wrench 外扰配置
# ============================================================
#
# 你的 Gazebo topic 列表中已经出现：
#   /world/windy/wrench
#   /world/windy/wrench/clear
#   /world/windy/wrench/persistent
#
# 所以这里使用 windy world 的 persistent wrench 接口。
#
# 注意：
# 1. force / torque 是 world 坐标系下的量。
# 2. 对 x500_0::base_link 施加 wrench。
# 3. 如果你的模型名字不是 x500_0，请改 GZ_TARGET_ENTITY_NAME。
#

GZ_WORLD_NAME = "windy"

GZ_WRENCH_TOPIC = f"/world/{GZ_WORLD_NAME}/wrench/persistent"
GZ_WRENCH_CLEAR_TOPIC = f"/world/{GZ_WORLD_NAME}/wrench/clear"

GZ_WRENCH_MSGTYPE = "gz.msgs.EntityWrench"
GZ_ENTITY_MSGTYPE = "gz.msgs.Entity"

# 推荐先用 LINK。你已经验证外力能让 QGC 有响应，说明这个写法在你的环境中可用。
GZ_TARGET_ENTITY_NAME = "x500_0::base_link"
GZ_TARGET_ENTITY_TYPE = "LINK"

# 如果你发现 LINK 施加力矩行为不理想，可以尝试：
# GZ_TARGET_ENTITY_NAME = "x500_0"
# GZ_TARGET_ENTITY_TYPE = "MODEL"


# ============================================================
# 等效风扰参数
# ============================================================
#
# 等效风力：
#   F = 0.5 * rho * Cd * A * V^2
#
# 对 500 mm 轴距四旋翼，A 可以先取 0.02 ~ 0.06 m^2。
# 这里默认 0.03 m^2。
#

AIR_DENSITY = 1.225
DEFAULT_CD = 1.0
DEFAULT_REF_AREA_M2 = 0.03


# ============================================================
# 工具函数
# ============================================================

def is_bad_number(x):
    try:
        return x is None or math.isnan(float(x)) or math.isinf(float(x))
    except Exception:
        return True


def safe_number(x, fallback=0.0):
    if is_bad_number(x):
        return fallback
    return float(x)


def get_nested_attr(obj, paths, default=math.nan):
    """
    兼容不同 MAVSDK 版本的字段名。
    paths 示例：
      ["position.north_m", "north_m"]
    """
    for path in paths:
        current = obj
        ok = True

        for name in path.split("."):
            if hasattr(current, name):
                current = getattr(current, name)
            else:
                ok = False
                break

        if ok and current is not None:
            return current

    return default


async def ainput(prompt):
    """
    非阻塞 input，避免用户输入时暂停数据采集。
    """
    return await asyncio.to_thread(input, prompt)


async def read_float(prompt, default=None):
    """
    读取浮点数。
    如果 default 不为 None，用户直接回车则使用默认值。
    """
    while True:
        text = (await ainput(prompt)).strip()

        if text == "" and default is not None:
            return float(default)

        try:
            return float(text)
        except ValueError:
            print("[ERROR] 请输入数字。")


def run_gz_topic_command(topic, msgtype, payload, timeout=5):
    """
    调用 gz topic 发布消息。
    """
    cmd = [
        "gz", "topic",
        "-t", topic,
        "-m", msgtype,
        "-p", payload
    ]

    try:
        result = subprocess.run(
            cmd,
            capture_output=True,
            text=True,
            timeout=timeout
        )

        if result.returncode != 0:
            print("\n[WARN] gz topic 发布失败。")
            print("[WARN] 命令:", " ".join(cmd))
            print("[WARN] stderr:", result.stderr.strip())
            return 0

        return 1

    except FileNotFoundError:
        print("\n[WARN] 找不到 gz 命令，请确认 Gazebo 已安装且 gz 在 PATH 中。")
        return 0

    except subprocess.TimeoutExpired:
        print("\n[WARN] gz topic 命令超时。")
        return 0


def check_gazebo_topics():
    """
    检查 Gazebo 中是否存在 wrench topic。
    """
    try:
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True,
            text=True,
            timeout=5
        )

        if result.returncode != 0:
            print("[WARN] 无法获取 gz topic 列表。")
            print(result.stderr.strip())
            return

        topics = result.stdout.splitlines()

        print("\n[INFO] 检查 Gazebo wrench topic:")

        if GZ_WRENCH_TOPIC in topics:
            print(f"[OK] 找到 {GZ_WRENCH_TOPIC}")
        else:
            print(f"[WARN] 没找到 {GZ_WRENCH_TOPIC}")

        if GZ_WRENCH_CLEAR_TOPIC in topics:
            print(f"[OK] 找到 {GZ_WRENCH_CLEAR_TOPIC}")
        else:
            print(f"[WARN] 没找到 {GZ_WRENCH_CLEAR_TOPIC}")

        if GZ_WRENCH_TOPIC not in topics or GZ_WRENCH_CLEAR_TOPIC not in topics:
            print("[WARN] 你的 Gazebo world 可能不是 windy，或者没有加载 ApplyLinkWrench。")
            print("[WARN] 请先确认：")
            print("       gz topic -l | grep wrench")
            print("       make px4_sitl gz_x500_windy")

    except Exception as e:
        print(f"[WARN] 检查 Gazebo topic 时出错: {e}")


def publish_wrench_to_gazebo(fx, fy, fz, tx, ty, tz):
    """
    向 Gazebo 施加持续外力 / 外力矩。
    force 单位：N
    torque 单位：N*m

    注意：
    Gazebo wrench 默认按 world 坐标系解释 force / torque。
    """
    payload = (
        f'entity {{ name: "{GZ_TARGET_ENTITY_NAME}" type: {GZ_TARGET_ENTITY_TYPE} }} '
        f'wrench {{ '
        f'force {{ x: {fx} y: {fy} z: {fz} }} '
        f'torque {{ x: {tx} y: {ty} z: {tz} }} '
        f'}}'
    )

    return run_gz_topic_command(
        topic=GZ_WRENCH_TOPIC,
        msgtype=GZ_WRENCH_MSGTYPE,
        payload=payload,
        timeout=5
    )


def clear_wrench_in_gazebo():
    """
    清除施加在目标 link / model 上的 persistent wrench。
    """
    payload = f'name: "{GZ_TARGET_ENTITY_NAME}" type: {GZ_TARGET_ENTITY_TYPE}'

    return run_gz_topic_command(
        topic=GZ_WRENCH_CLEAR_TOPIC,
        msgtype=GZ_ENTITY_MSGTYPE,
        payload=payload,
        timeout=5
    )


def equivalent_wind_force(
    wind_speed_m_s,
    direction_x,
    direction_y,
    direction_z,
    cd=DEFAULT_CD,
    area_m2=DEFAULT_REF_AREA_M2,
    rho=AIR_DENSITY
):
    """
    将风速换算为等效外力。

    F = 0.5 * rho * Cd * A * V^2

    direction_x/y/z 表示风力方向，不要求单位化。
    例如：
      x 方向风扰：direction = (1, 0, 0)
      y 方向风扰：direction = (0, 1, 0)

    返回：
      fx, fy, fz, force_magnitude
    """
    dx = float(direction_x)
    dy = float(direction_y)
    dz = float(direction_z)

    norm = math.sqrt(dx * dx + dy * dy + dz * dz)

    if norm < 1e-9:
        dx, dy, dz = 1.0, 0.0, 0.0
        norm = 1.0

    dx /= norm
    dy /= norm
    dz /= norm

    v = abs(float(wind_speed_m_s))
    force_mag = 0.5 * rho * cd * area_m2 * v * v

    return force_mag * dx, force_mag * dy, force_mag * dz, force_mag


# ============================================================
# 主程序
# ============================================================

async def main():
    drone = System()

    print("Connecting to PX4...")
    await drone.connect(system_address=PX4_UDP_ADDRESS)

    async for connection_state in drone.core.connection_state():
        if connection_state.is_connected:
            print("Connected to PX4!")
            break

    try:
        await drone.telemetry.set_rate_attitude_euler(RECORD_HZ)
        await drone.telemetry.set_rate_position_velocity_ned(RECORD_HZ)
        await drone.telemetry.set_rate_imu(RECORD_HZ)
        print(f"Telemetry rate set to {RECORD_HZ} Hz.")
    except Exception as e:
        print(f"[WARN] 设置遥测频率失败，继续使用默认频率: {e}")

    print(f"CSV output: {OUTPUT_CSV}")

    check_gazebo_topics()

    print("\n================ 命令说明 ================")
    print("wind   : 输入风速 m/s，脚本换算为等效外力 N")
    print("force  : 直接输入外力 N")
    print("torque : 直接输入外力矩 N*m，推荐用于姿态抗扰实验")
    print("wrench : 同时输入外力 N 和外力矩 N*m")
    print("clear  : 清除当前持续外扰")
    print("exit   : 结束记录")
    print("==========================================\n")

    start_time = time.monotonic()
    stop_event = asyncio.Event()

    have_attitude = asyncio.Event()
    have_position = asyncio.Event()
    have_imu = asyncio.Event()

    # 最新无人机状态
    state = {
        "roll_deg": math.nan,
        "pitch_deg": math.nan,
        "yaw_deg": math.nan,

        "x_m": math.nan,
        "y_m": math.nan,
        "z_m": math.nan,

        "vx_m_s": math.nan,
        "vy_m_s": math.nan,
        "vz_m_s": math.nan,

        "ax_m_s2": math.nan,
        "ay_m_s2": math.nan,
        "az_m_s2": math.nan,

        "gyro_x_rad_s": math.nan,
        "gyro_y_rad_s": math.nan,
        "gyro_z_rad_s": math.nan,
    }

    # 上一次有效值，用来避免 CSV 里出现 nan
    last_valid_state = {
        key: 0.0 for key in state.keys()
    }

    # 外扰状态，全部用数字，方便 PlotJuggler 画图
    disturbance_state = {
        "active": 0,

        # 0 none, 1 wind_equivalent_force, 2 force, 3 torque, 4 wrench
        "mode_id": 0,

        "event_marker": 0,
        "event_id": 0,
        "cmd_ok": 1,

        # 等效风扰记录
        "wind_equiv_speed_m_s": 0.0,
        "wind_dir_x": 0.0,
        "wind_dir_y": 0.0,
        "wind_dir_z": 0.0,
        "wind_cd": DEFAULT_CD,
        "wind_area_m2": DEFAULT_REF_AREA_M2,

        # 实际发给 Gazebo 的 wrench
        "force_x_N": 0.0,
        "force_y_N": 0.0,
        "force_z_N": 0.0,

        "torque_x_Nm": 0.0,
        "torque_y_Nm": 0.0,
        "torque_z_Nm": 0.0,

        "force_magnitude_N": 0.0,
        "torque_magnitude_Nm": 0.0,
    }

    def set_disturbance_state(
        active,
        mode_id,
        fx=0.0,
        fy=0.0,
        fz=0.0,
        tx=0.0,
        ty=0.0,
        tz=0.0,
        wind_speed=0.0,
        wind_dir_x=0.0,
        wind_dir_y=0.0,
        wind_dir_z=0.0,
        wind_cd=DEFAULT_CD,
        wind_area_m2=DEFAULT_REF_AREA_M2,
        event_marker=0,
        cmd_ok=1
    ):
        disturbance_state["active"] = int(active)
        disturbance_state["mode_id"] = int(mode_id)
        disturbance_state["event_marker"] = int(event_marker)
        disturbance_state["cmd_ok"] = int(cmd_ok)

        disturbance_state["wind_equiv_speed_m_s"] = float(wind_speed)
        disturbance_state["wind_dir_x"] = float(wind_dir_x)
        disturbance_state["wind_dir_y"] = float(wind_dir_y)
        disturbance_state["wind_dir_z"] = float(wind_dir_z)
        disturbance_state["wind_cd"] = float(wind_cd)
        disturbance_state["wind_area_m2"] = float(wind_area_m2)

        disturbance_state["force_x_N"] = float(fx)
        disturbance_state["force_y_N"] = float(fy)
        disturbance_state["force_z_N"] = float(fz)

        disturbance_state["torque_x_Nm"] = float(tx)
        disturbance_state["torque_y_Nm"] = float(ty)
        disturbance_state["torque_z_Nm"] = float(tz)

        disturbance_state["force_magnitude_N"] = math.sqrt(fx * fx + fy * fy + fz * fz)
        disturbance_state["torque_magnitude_Nm"] = math.sqrt(tx * tx + ty * ty + tz * tz)

    async def attitude_task():
        async for attitude in drone.telemetry.attitude_euler():
            state["roll_deg"] = attitude.roll_deg
            state["pitch_deg"] = attitude.pitch_deg
            state["yaw_deg"] = attitude.yaw_deg

            have_attitude.set()

            if stop_event.is_set():
                break

    async def position_task():
        async for posvel in drone.telemetry.position_velocity_ned():
            north = get_nested_attr(posvel, ["position.north_m", "north_m"])
            east = get_nested_attr(posvel, ["position.east_m", "east_m"])
            down = get_nested_attr(posvel, ["position.down_m", "down_m"])

            vn = get_nested_attr(posvel, ["velocity.north_m_s", "vel_n_m_s"])
            ve = get_nested_attr(posvel, ["velocity.east_m_s", "vel_e_m_s"])
            vd = get_nested_attr(posvel, ["velocity.down_m_s", "vel_d_m_s"])

            state["x_m"] = north
            state["y_m"] = east
            state["z_m"] = -down if not is_bad_number(down) else math.nan

            state["vx_m_s"] = vn
            state["vy_m_s"] = ve
            state["vz_m_s"] = vd

            have_position.set()

            if stop_event.is_set():
                break

    async def imu_task():
        async for imu in drone.telemetry.imu():
            ax = get_nested_attr(
                imu,
                [
                    "acceleration_frd.forward_m_s2",
                    "accelerometer.x_m_s2",
                    "accelerometer_x_m_s2",
                ]
            )

            ay = get_nested_attr(
                imu,
                [
                    "acceleration_frd.right_m_s2",
                    "accelerometer.y_m_s2",
                    "accelerometer_y_m_s2",
                ]
            )

            az = get_nested_attr(
                imu,
                [
                    "acceleration_frd.down_m_s2",
                    "accelerometer.z_m_s2",
                    "accelerometer_z_m_s2",
                ]
            )

            gx = get_nested_attr(
                imu,
                [
                    "angular_velocity_frd.forward_rad_s",
                    "gyroscope.x_rad_s",
                    "gyro_x_rad_s",
                ]
            )

            gy = get_nested_attr(
                imu,
                [
                    "angular_velocity_frd.right_rad_s",
                    "gyroscope.y_rad_s",
                    "gyro_y_rad_s",
                ]
            )

            gz = get_nested_attr(
                imu,
                [
                    "angular_velocity_frd.down_rad_s",
                    "gyroscope.z_rad_s",
                    "gyro_z_rad_s",
                ]
            )

            state["ax_m_s2"] = ax
            state["ay_m_s2"] = ay
            state["az_m_s2"] = az

            state["gyro_x_rad_s"] = gx
            state["gyro_y_rad_s"] = gy
            state["gyro_z_rad_s"] = gz

            have_imu.set()

            if stop_event.is_set():
                break

    async def csv_writer_task():
        # 等待关键遥测先有数据，避免 CSV 前几行全是 nan
        print("[INFO] 等待 attitude / position 遥测数据...")

        await have_attitude.wait()
        await have_position.wait()

        try:
            await asyncio.wait_for(have_imu.wait(), timeout=3.0)
        except asyncio.TimeoutError:
            print("[WARN] 3 秒内没有收到 IMU 数据，先开始记录。")

        print("[INFO] 遥测数据已就绪，开始写 CSV。")

        fieldnames = [
            "time_s",
            "sample_index",

            "roll_deg",
            "pitch_deg",
            "yaw_deg",

            "x_m",
            "y_m",
            "z_m",

            "vx_m_s",
            "vy_m_s",
            "vz_m_s",

            "ax_m_s2",
            "ay_m_s2",
            "az_m_s2",

            "gyro_x_rad_s",
            "gyro_y_rad_s",
            "gyro_z_rad_s",

            "dist_active",
            "dist_mode_id",

            "dist_force_x_N",
            "dist_force_y_N",
            "dist_force_z_N",
            "dist_force_magnitude_N",

            "dist_torque_x_Nm",
            "dist_torque_y_Nm",
            "dist_torque_z_Nm",
            "dist_torque_magnitude_Nm",

            "wind_equiv_speed_m_s",
            "wind_dir_x",
            "wind_dir_y",
            "wind_dir_z",
            "wind_cd",
            "wind_area_m2",

            "dist_event_marker",
            "dist_event_id",
            "dist_cmd_ok",

            # 兼容你之前 PlotJuggler 的变量名
            "wind_active",
            "wind_x_m_s",
            "wind_y_m_s",
            "wind_z_m_s",
            "wind_speed_m_s",
            "wind_event_marker",
            "wind_event_id",
            "wind_cmd_ok",
        ]

        sample_index = 0
        dt = 1.0 / RECORD_HZ
        last_flush_time = time.monotonic()

        with open(OUTPUT_CSV, mode="w", newline="") as csvfile:
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()

            while not stop_event.is_set():
                now = time.monotonic()
                time_s = now - start_time

                row = {
                    "time_s": time_s,
                    "sample_index": sample_index,
                }

                # 状态量：如果当前值是 nan，就使用上一时刻有效值
                for key in state.keys():
                    value = state[key]
                    if is_bad_number(value):
                        value = last_valid_state[key]
                    else:
                        value = float(value)
                        last_valid_state[key] = value

                    row[key] = value

                row.update({
                    "dist_active": disturbance_state["active"],
                    "dist_mode_id": disturbance_state["mode_id"],

                    "dist_force_x_N": disturbance_state["force_x_N"],
                    "dist_force_y_N": disturbance_state["force_y_N"],
                    "dist_force_z_N": disturbance_state["force_z_N"],
                    "dist_force_magnitude_N": disturbance_state["force_magnitude_N"],

                    "dist_torque_x_Nm": disturbance_state["torque_x_Nm"],
                    "dist_torque_y_Nm": disturbance_state["torque_y_Nm"],
                    "dist_torque_z_Nm": disturbance_state["torque_z_Nm"],
                    "dist_torque_magnitude_Nm": disturbance_state["torque_magnitude_Nm"],

                    "wind_equiv_speed_m_s": disturbance_state["wind_equiv_speed_m_s"],
                    "wind_dir_x": disturbance_state["wind_dir_x"],
                    "wind_dir_y": disturbance_state["wind_dir_y"],
                    "wind_dir_z": disturbance_state["wind_dir_z"],
                    "wind_cd": disturbance_state["wind_cd"],
                    "wind_area_m2": disturbance_state["wind_area_m2"],

                    "dist_event_marker": disturbance_state["event_marker"],
                    "dist_event_id": disturbance_state["event_id"],
                    "dist_cmd_ok": disturbance_state["cmd_ok"],

                    # 兼容旧变量名
                    "wind_active": disturbance_state["active"] if disturbance_state["mode_id"] == 1 else 0,
                    "wind_x_m_s": disturbance_state["wind_equiv_speed_m_s"] * disturbance_state["wind_dir_x"],
                    "wind_y_m_s": disturbance_state["wind_equiv_speed_m_s"] * disturbance_state["wind_dir_y"],
                    "wind_z_m_s": disturbance_state["wind_equiv_speed_m_s"] * disturbance_state["wind_dir_z"],
                    "wind_speed_m_s": disturbance_state["wind_equiv_speed_m_s"],
                    "wind_event_marker": disturbance_state["event_marker"],
                    "wind_event_id": disturbance_state["event_id"],
                    "wind_cmd_ok": disturbance_state["cmd_ok"],
                })

                writer.writerow(row)

                # event_marker 只保持一个采样点，方便 PlotJuggler 看到尖峰
                disturbance_state["event_marker"] = 0

                sample_index += 1

                if now - last_flush_time > 1.0:
                    csvfile.flush()
                    last_flush_time = now

                await asyncio.sleep(dt)

            csvfile.flush()

    async def apply_disturbance_for_duration(
        duration,
        mode_id,
        fx=0.0,
        fy=0.0,
        fz=0.0,
        tx=0.0,
        ty=0.0,
        tz=0.0,
        wind_speed=0.0,
        wind_dir_x=0.0,
        wind_dir_y=0.0,
        wind_dir_z=0.0,
        wind_cd=DEFAULT_CD,
        wind_area_m2=DEFAULT_REF_AREA_M2
    ):
        """
        施加指定持续时间的扰动。
        使用 persistent wrench，同时周期性重复发布，增强可靠性。
        """
        disturbance_state["event_id"] += 1

        ok = await asyncio.to_thread(
            publish_wrench_to_gazebo,
            fx, fy, fz, tx, ty, tz
        )

        set_disturbance_state(
            active=1,
            mode_id=mode_id,
            fx=fx,
            fy=fy,
            fz=fz,
            tx=tx,
            ty=ty,
            tz=tz,
            wind_speed=wind_speed,
            wind_dir_x=wind_dir_x,
            wind_dir_y=wind_dir_y,
            wind_dir_z=wind_dir_z,
            wind_cd=wind_cd,
            wind_area_m2=wind_area_m2,
            event_marker=1,
            cmd_ok=ok
        )

        print("\n[INFO] 外扰已开始:")
        print(f"       duration = {duration:.3f} s")
        print(f"       force    = ({fx:.4f}, {fy:.4f}, {fz:.4f}) N")
        print(f"       torque   = ({tx:.4f}, {ty:.4f}, {tz:.4f}) N*m")
        print(f"       cmd_ok   = {ok}")

        publish_rate_hz = 10.0
        publish_dt = 1.0 / publish_rate_hz
        end_time = time.monotonic() + duration

        while time.monotonic() < end_time and not stop_event.is_set():
            ok = await asyncio.to_thread(
                publish_wrench_to_gazebo,
                fx, fy, fz, tx, ty, tz
            )
            disturbance_state["cmd_ok"] = ok
            await asyncio.sleep(publish_dt)

        print("[INFO] 外扰结束，清除 persistent wrench。")

        ok_clear = await asyncio.to_thread(clear_wrench_in_gazebo)

        set_disturbance_state(
            active=0,
            mode_id=0,
            fx=0.0,
            fy=0.0,
            fz=0.0,
            tx=0.0,
            ty=0.0,
            tz=0.0,
            wind_speed=0.0,
            wind_dir_x=0.0,
            wind_dir_y=0.0,
            wind_dir_z=0.0,
            wind_cd=wind_cd,
            wind_area_m2=wind_area_m2,
            event_marker=-1,
            cmd_ok=ok_clear
        )

        print(f"[INFO] clear_cmd_ok = {ok_clear}")

    async def disturbance_input_task():
        while not stop_event.is_set():
            cmd = (await ainput(
                "\n输入命令 wind / force / torque / wrench / clear / exit: "
            )).strip().lower()

            if cmd == "exit":
                print("Stopping logger...")
                stop_event.set()
                break

            if cmd == "clear":
                ok_clear = await asyncio.to_thread(clear_wrench_in_gazebo)

                set_disturbance_state(
                    active=0,
                    mode_id=0,
                    event_marker=-1,
                    cmd_ok=ok_clear
                )

                print(f"[INFO] 已清除 persistent wrench, cmd_ok={ok_clear}")
                continue

            if cmd == "wind":
                print("\n[wind] 等效风扰模式")
                print("说明：输入风速，脚本根据 F=0.5*rho*Cd*A*V^2 换算为等效外力。")
                print("推荐：15 m/s, direction=(1,0,0), Cd=1.0, A=0.03")
                duration = await read_float("输入扰动持续时间 s，例如 5: ")
                wind_speed = await read_float("输入等效风速 m/s，例如 15: ")
                dx = await read_float("输入风力方向 x，例如 1: ", default=1.0)
                dy = await read_float("输入风力方向 y，例如 0: ", default=0.0)
                dz = await read_float("输入风力方向 z，例如 0: ", default=0.0)
                cd = await read_float(f"输入阻力系数 Cd，默认 {DEFAULT_CD}: ", default=DEFAULT_CD)
                area = await read_float(f"输入迎风面积 A m^2，默认 {DEFAULT_REF_AREA_M2}: ", default=DEFAULT_REF_AREA_M2)

                fx, fy, fz, force_mag = equivalent_wind_force(
                    wind_speed_m_s=wind_speed,
                    direction_x=dx,
                    direction_y=dy,
                    direction_z=dz,
                    cd=cd,
                    area_m2=area
                )

                print(f"[INFO] 等效风力大小 = {force_mag:.4f} N")
                print(f"[INFO] 等效风力向量 = ({fx:.4f}, {fy:.4f}, {fz:.4f}) N")

                await apply_disturbance_for_duration(
                    duration=duration,
                    mode_id=1,
                    fx=fx,
                    fy=fy,
                    fz=fz,
                    tx=0.0,
                    ty=0.0,
                    tz=0.0,
                    wind_speed=wind_speed,
                    wind_dir_x=dx / max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-9),
                    wind_dir_y=dy / max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-9),
                    wind_dir_z=dz / max(math.sqrt(dx * dx + dy * dy + dz * dz), 1e-9),
                    wind_cd=cd,
                    wind_area_m2=area
                )
                continue

            if cmd == "force":
                print("\n[force] 直接外力模式")
                print("说明：输入单位 N。推荐先试 3~10 N。")
                duration = await read_float("输入扰动持续时间 s，例如 5: ")
                fx = await read_float("输入 force_x_N，例如 5: ", default=5.0)
                fy = await read_float("输入 force_y_N，例如 0: ", default=0.0)
                fz = await read_float("输入 force_z_N，例如 0: ", default=0.0)

                await apply_disturbance_for_duration(
                    duration=duration,
                    mode_id=2,
                    fx=fx,
                    fy=fy,
                    fz=fz,
                    tx=0.0,
                    ty=0.0,
                    tz=0.0
                )
                continue

            if cmd == "torque":
                print("\n[torque] 直接外力矩模式")
                print("说明：输入单位 N*m。推荐姿态抗扰实验先试 0.02~0.20 N*m。")
                print("例如 roll 扰动用 torque_x_Nm，pitch 扰动用 torque_y_Nm，yaw 扰动用 torque_z_Nm。")
                duration = await read_float("输入扰动持续时间 s，例如 3: ")
                tx = await read_float("输入 torque_x_Nm，例如 0.05: ", default=0.05)
                ty = await read_float("输入 torque_y_Nm，例如 0: ", default=0.0)
                tz = await read_float("输入 torque_z_Nm，例如 0: ", default=0.0)

                await apply_disturbance_for_duration(
                    duration=duration,
                    mode_id=3,
                    fx=0.0,
                    fy=0.0,
                    fz=0.0,
                    tx=tx,
                    ty=ty,
                    tz=tz
                )
                continue

            if cmd == "wrench":
                print("\n[wrench] 外力 + 外力矩模式")
                duration = await read_float("输入扰动持续时间 s，例如 5: ")

                fx = await read_float("输入 force_x_N，例如 5: ", default=5.0)
                fy = await read_float("输入 force_y_N，例如 0: ", default=0.0)
                fz = await read_float("输入 force_z_N，例如 0: ", default=0.0)

                tx = await read_float("输入 torque_x_Nm，例如 0.05: ", default=0.05)
                ty = await read_float("输入 torque_y_Nm，例如 0: ", default=0.0)
                tz = await read_float("输入 torque_z_Nm，例如 0: ", default=0.0)

                await apply_disturbance_for_duration(
                    duration=duration,
                    mode_id=4,
                    fx=fx,
                    fy=fy,
                    fz=fz,
                    tx=tx,
                    ty=ty,
                    tz=tz
                )
                continue

            print("[WARN] 未知命令。请输入 wind / force / torque / wrench / clear / exit。")

    tasks = [
        asyncio.create_task(attitude_task()),
        asyncio.create_task(position_task()),
        asyncio.create_task(imu_task()),
        asyncio.create_task(csv_writer_task()),
        asyncio.create_task(disturbance_input_task()),
    ]

    try:
        await tasks[-1]
    except KeyboardInterrupt:
        print("KeyboardInterrupt received.")
        stop_event.set()
    finally:
        stop_event.set()

        print("[INFO] 程序退出前清除 Gazebo persistent wrench。")
        await asyncio.to_thread(clear_wrench_in_gazebo)

        for task in tasks:
            task.cancel()

        await asyncio.gather(*tasks, return_exceptions=True)

    print(f"CSV saved: {OUTPUT_CSV}")


if __name__ == "__main__":
    asyncio.run(main())
