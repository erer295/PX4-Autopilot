#!/usr/bin/env python3
"""Generate the paper figures for the FHv2 draft."""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import numpy as np

import importlib.util

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent / "validation" / "fh2_robustness_20260723"
FIG = ROOT / "figures"
FIG.mkdir(exist_ok=True)

for candidate in ("/usr/share/fonts/opentype/noto/NotoSerifCJK-Regular.ttc",
                  "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"):
    if Path(candidate).is_file():
        fm.fontManager.addfont(candidate)
CJK = next((name for name in ("Noto Serif CJK SC", "Noto Sans CJK SC", "Noto Sans CJK JP")
            if name in {f.name for f in fm.fontManager.ttflist}), None)
if CJK:
    plt.rcParams["font.family"] = CJK
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 150

spec = importlib.util.spec_from_file_location("fh2", str(HERE / "zd680_fuci_fh2_design.py"))
fh2 = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fh2)

design = {round(i["rope_length_m"], 2): i
          for i in json.loads((ROOT / "fh2_design.json").read_text(encoding="utf-8"))}
fh1_design = {round(i["rope_length_m"], 2): i for i in json.loads(
    (HERE.parent / "validation" / "fuci_finite_horizon_20260722" / "finite_horizon_design.json")
    .read_text(encoding="utf-8"))}


def shaped_profiles(item, dt=0.002):
    time = np.arange(0.0, 6.0 + 0.5 * dt, dt)
    u = time / 6.0
    bp, bv, ba = fh2.trajectory_progress(time)
    order = int(item["order"])
    bpos, bvel_u, bacc_u = fh2.finite_horizon_basis(order, u)
    c = np.asarray(item["coefficients"], dtype=float)
    pos = bp + bpos @ c
    vel = bv + bvel_u @ c / 6.0
    acc = ba + bacc_u @ c / 36.0
    jerk = np.gradient(2.0 * acc, dt)
    return time, 2.0 * pos, 2.0 * vel, 2.0 * acc, jerk


def fig1():
    item = design[0.6]
    time, pos, vel, acc, jerk = shaped_profiles(item)
    _, pos0, vel0, acc0, jerk0 = shaped_profiles(
        {"order": 1, "coefficients": [0.0]})  # base S-curve
    axes = plt.subplots(4, 1, figsize=(7, 8), sharex=True)[1]
    series = ((pos, pos0, "位置 (m)"), (vel, vel0, "速度 (m/s)"),
              (acc, acc0, "加速度 (m/s²)"), (jerk, jerk0, "加加速度 (m/s³)"))
    for ax, (y, y0, label) in zip(axes, series):
        ax.plot(time, y0, "--", color="0.5", label="原始 S 曲线")
        ax.plot(time, y, "b-", label="FHv2 整形轨迹")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[3].axhline(1.6, color="r", ls=":", lw=1, label="jerk 约束 1.6")
    axes[0].legend(loc="lower right", fontsize=8)
    axes[3].legend(loc="upper right", fontsize=8)
    axes[3].set_xlabel("时间 (s)")
    axes[0].set_title("FHv2 整形轨迹（0.6 m 绳长，2 m/6 s 任务）")
    plt.tight_layout()
    plt.savefig(FIG / "fig1_trajectory.png")
    plt.close()


def swing_curves(item, rope_length):
    dt = 0.002
    time, _, _, acc, _ = shaped_profiles(item, dt)
    ratios = np.linspace(0.90, 1.20, 61)
    ang, _ = fh2.simulate_pendulum(np.tile(acc[:, None], (1, ratios.size)), dt,
                                   rope_length, ratios)
    transient = np.max(np.abs(ang), axis=0) * 180.0 / math.pi
    terminal = np.abs(fh2.terminal_modal_batch(
        np.tile(acc[:, None], (1, ratios.size)), dt, rope_length, ratios)) * 180.0 / math.pi
    return ratios, transient, terminal


def fig2():
    ratios, tr2, te2 = swing_curves(design[0.6], 0.6)
    _, tr1, te1 = swing_curves(fh1_design[0.6], 0.6)
    base_item = {"order": 1, "coefficients": [0.0]}
    _, tr0, te0 = swing_curves(base_item, 0.6)
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7, 6), sharex=True)
    a1.plot(ratios, tr0, "--", color="0.5", label="不整形")
    a1.plot(ratios, tr1, "-.", color="tab:red", label="FHv1（仅末端约束）")
    a1.plot(ratios, tr2, "b-", label="FHv2（全时域约束）")
    a1.axhline(3.0, color="r", ls=":", lw=1)
    a1.set_ylabel("最坏瞬态摆角 (°)")
    a1.set_title("频率比区间内摆角保证（0.6 m 标称，零初态）")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3)
    a2.plot(ratios, te0, "--", color="0.5", label="不整形")
    a2.plot(ratios, te1, "-.", color="tab:red", label="FHv1")
    a2.plot(ratios, te2, "b-", label="FHv2")
    a2.axhline(1.15, color="r", ls=":", lw=1)
    a2.set_ylabel("末端残余模态 (°)")
    a2.set_xlabel("频率比 ρ = ω实际/ω标称")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(FIG / "fig2_interval_guarantee.png")
    plt.close()


def load_truth(run_dir: Path):
    rows = list(csv.DictReader((run_dir / "hang_joint_samples.csv").open()))
    t = np.asarray([float(r["boot_s_est"]) for r in rows])
    roll = np.asarray([float(r["roll_position_rad"]) for r in rows])
    pitch = np.asarray([float(r["pitch_position_rad"]) for r in rows])
    return t, np.sqrt(roll ** 2 + pitch ** 2) * 180.0 / math.pi


def route_start(run_dir: Path):
    with (run_dir / "events.csv").open() as f:
        for row in csv.DictReader(f):
            if "north" in row["name"]:
                return float(row["start_boot_s"])
    raise RuntimeError("no north event")


def fig3():
    base = HERE.parent / "validation"
    runs = {
        "PID+AS 不整形（基线）": base / "fso_usc_quick_freeze_20260722",
        "FHv1（失败版）": base / "fuci_finite_horizon_20260722" / "runs" /
            "STD_L06_PID_AS_UCIS_FH" / "2026-07-22" / "position_offboard_20260722_213647",
        "FHv2": ROOT / "gate_runs_v2" / "runs" / "STD_L06_PID_AS_UCIS_FH2",
    }
    plt.figure(figsize=(8, 4.5))
    # baseline: pick one valid run from the quick-freeze A group
    base_runs = sorted((base / "fso_usc_quick_freeze_20260722").glob(
        "l06/runs/STD_L06_PID_AS/*/*/hang_joint_samples.csv"))
    pairs = []
    if base_runs:
        pairs.append(("PID+AS 不整形（基线）", base_runs[0].parent))
    pairs.append(("FHv1（仅末端约束）", runs["FHv1（失败版）"]))
    fh2_runs = sorted(runs["FHv2"].glob("*/*/hang_joint_samples.csv"))
    pairs.append(("FHv2", fh2_runs[-1].parent))
    styles = {"PID+AS 不整形（基线）": ("--", "0.4"), "FHv1（仅末端约束）": ("-.", "tab:red"),
              "FHv2": ("-", "b")}
    for label, run_dir in pairs:
        t, mag = load_truth(run_dir)
        start = route_start(run_dir)
        mask = (t >= start) & (t <= start + 49)
        plt.plot(t[mask] - start, mag[mask], styles[label][0], color=styles[label][1],
                 label=label, lw=1.2)
    for seg_start in (0, 11, 22, 33):
        plt.axvspan(seg_start, seg_start + 6, color="0.9", alpha=0.4)
    plt.axhline(4.0, color="r", ls=":", lw=1, label="峰值门 4.0°")
    plt.xlabel("任务时间 (s)")
    plt.ylabel("摆角幅值 (°)")
    plt.title("0.6 m 绳长四方向任务真值摆角对比（灰带为移动段）")
    plt.legend(fontsize=8, loc="upper right")
    plt.grid(alpha=0.3)
    plt.tight_layout()
    plt.savefig(FIG / "fig3_flight_swing.png")
    plt.close()


def fig4():
    decision = json.loads((ROOT / "matrix_decision.json").read_text(encoding="utf-8"))
    points = decision["points"]
    order = ["L05", "L06", "L08", "P75", "MM070", "EI_L06", "EI_MM070", "BASE_L08"]
    labels = [p for p in order if p in points and points[p]["n_runs"] > 0]
    mean = [points[p]["swing_angle_rms_deg_mean"] for p in labels]
    sd = [points[p]["swing_angle_rms_deg_sd"] for p in labels]
    peak = [points[p]["swing_angle_peak_deg_mean"] for p in labels]
    x = np.arange(len(labels))
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    a1.bar(x, mean, yerr=sd, capsize=3, color="tab:blue", alpha=0.8)
    a1.axhline(1.0, color="r", ls=":", lw=1, label="RMS 门 1.0°")
    a1.set_ylabel("摆角 RMS (°)")
    a1.legend(fontsize=8)
    a1.grid(alpha=0.3, axis="y")
    a2.bar(x, peak, color="tab:orange", alpha=0.8)
    a2.axhline(4.0, color="r", ls=":", lw=1, label="峰值门 4.0°")
    a2.set_ylabel("摆角峰值 (°)")
    a2.set_xticks(x, labels, rotation=20)
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3, axis="y")
    a1.set_title("鲁棒性矩阵摆角结果（均值±SD）")
    plt.tight_layout()
    plt.savefig(FIG / "fig4_matrix.png")
    plt.close()


if __name__ == "__main__":
    fig1()
    print("fig1 done")
    fig2()
    print("fig2 done")
    fig3()
    print("fig3 done")
    try:
        fig4()
        print("fig4 done")
    except FileNotFoundError:
        print("fig4 skipped (matrix not finished)")
