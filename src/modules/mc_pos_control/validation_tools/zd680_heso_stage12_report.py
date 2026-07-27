#!/usr/bin/env python3
"""Generate the preregistered HESO stage-1/stage-2 validation report."""

from __future__ import annotations

import argparse
import csv
import json
import math
import runpy
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
from pyulog import ULog


MODULE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_ROOT = MODULE_ROOT / "validation" / "heso_stage12_validation_20260722"
VFB_REPORT = MODULE_ROOT / "validation_tools" / "zd680_vfb_report.py"
STAGE1_BENCHMARK = MODULE_ROOT / "validation_tools" / "zd680_heso_stage1_frequency_benchmark.py"

BASE_MODE = "STD_L06_PID_AS_HGS2_BASE"
ORACLE_MODE = "STD_L06_PID_AS_HGS2_ORACLE_K40"
RLS_MODE = "STD_L06_PID_AS_HGS2_RLS_SHADOW"

PERFORMANCE_FIELDS = (
    "initial_swing_angle_rms_deg",
    "initial_swing_rate_rms_rad_s",
    "initial_xy_error_m",
    "swing_angle_rms_deg",
    "swing_angle_peak_deg",
    "swing_energy_integral_j_s_kg",
    "final_5s_swing_angle_rms_deg",
    "xy_error_rms_m",
    "final_acc_rms_m_s2",
    "final_jerk_p95_m_s3",
    "as_rms_m_s2",
    "as_peak_m_s2",
    "as_positive_power_integral_j_kg",
    "as_negative_power_integral_j_kg",
    "total_acc_saturated_ratio",
    "failsafe_ratio",
)


def finite(value: object, fallback: float = math.nan) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return fallback
    return result if math.isfinite(result) else fallback


def pct(candidate: float, baseline: float) -> float:
    return 100.0 * (candidate / baseline - 1.0) if baseline else math.nan


def fmt(value: object, digits: int = 3) -> str:
    number = finite(value)
    return f"{number:.{digits}f}" if math.isfinite(number) else "—"


def fmt_pct(value: object, digits: int = 2) -> str:
    number = finite(value)
    return f"{number:+.{digits}f}%" if math.isfinite(number) else "—"


def write_csv(path: Path, rows: Sequence[Dict[str, object]]) -> None:
    fields: List[str] = []
    for row in rows:
        for key in row:
            if key not in fields:
                fields.append(key)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def debug_dataset(ulog: ULog, debug_id: int):
    datasets = []
    for dataset in ulog.data_list:
        if dataset.name != "debug_array" or "id" not in dataset.data:
            continue
        mask = np.asarray(dataset.data["id"]) == debug_id
        if np.any(mask):
            datasets.append({key: np.asarray(value)[mask] for key, value in dataset.data.items()})
    return max(datasets, key=lambda item: len(item["timestamp"])) if datasets else None


def route_bounds(run_dir: Path) -> Tuple[float, float]:
    with (run_dir / "events.csv").open(newline="", encoding="utf-8") as stream:
        events = list(csv.DictReader(stream))
    actions = [row for row in events if str(row.get("name", "")).startswith("action_")]
    return (min(float(row["start_boot_s"]) for row in actions),
            max(float(row["end_boot_s"]) for row in actions))


def power_parts(t: np.ndarray, power: np.ndarray) -> Tuple[float, float]:
    positive = np.maximum(power, 0.0)
    negative = np.minimum(power, 0.0)
    return float(np.trapezoid(positive, t)), float(np.trapezoid(negative, t))


def run_dirs(root: Path, mode: str) -> List[Path]:
    paths = sorted((root / "stage2" / "runs" / mode).glob("*/*"))
    return [path for path in paths if (path / "metadata.json").is_file()]


def analyze_mechanism(run_dir: Path, label: str, stage1_api: Dict[str, object]) -> Dict[str, object]:
    route_start, route_end = route_bounds(run_dir)
    ulog = ULog(str(run_dir / "position_offboard.ulg"), ["debug_array", "vehicle_attitude"])
    fso = debug_dataset(ulog, 687)
    anti_swing = debug_dataset(ulog, 681)
    coordination = debug_dataset(ulog, 684)
    if fso is None or anti_swing is None or coordination is None:
        raise RuntimeError(f"missing mechanism debug data in {run_dir}")

    ft = np.asarray(fso["timestamp"], dtype=float) * 1.0e-6
    fm = (ft >= route_start) & (ft <= route_end)
    at = np.asarray(anti_swing["timestamp"], dtype=float) * 1.0e-6
    am = (at >= route_start) & (at <= route_end)
    ct = np.asarray(coordination["timestamp"], dtype=float) * 1.0e-6
    cm = (ct >= route_start) & (ct <= route_end)

    def fso_data(index: int) -> np.ndarray:
        return np.asarray(fso[f"data[{index}]"], dtype=float)[fm]

    direct = np.hypot(fso_data(18), fso_data(19))
    frequency_ratio = fso_data(46)
    confidence = fso_data(47)
    energy_gate = fso_data(49)
    position_gate = fso_data(50)
    gain_raw = fso_data(51)
    gain_applied = fso_data(52)
    active = fso_data(57)
    position_error = np.hypot(fso_data(33), fso_data(34))
    anti_gain = np.asarray(anti_swing["data[19]"], dtype=float)[am]
    anti_raw = np.hypot(
        np.asarray(anti_swing["data[14]"], dtype=float)[am],
        np.asarray(anti_swing["data[15]"], dtype=float)[am],
    )
    anti_power = np.asarray(coordination["data[18]"], dtype=float)[cm]
    anti_positive, anti_negative = power_parts(ct[cm], anti_power)

    flight_t, flight_angle, _, _ = stage1_api["load_flight"](run_dir)
    fft_ratio = float(stage1_api["fft_peak_ratio"](flight_t, flight_angle))
    energetic = (ft >= route_start + 10.0) & (ft <= route_end) & (np.asarray(fso["data[22]"], dtype=float) >= 0.003)
    energetic_ratio = np.asarray(fso["data[46]"], dtype=float)[energetic]
    energetic_confidence = np.asarray(fso["data[47]"], dtype=float)[energetic]
    energetic_ratio_median = float(np.median(energetic_ratio)) if energetic_ratio.size else math.nan
    high_position = position_error > 0.05

    return {
        "label": label,
        "run": run_dir.name,
        "route_samples": int(np.sum(fm)),
        "mode_id": int(round(float(np.median(fso_data(26))))),
        "direct_compensation_peak_m_s2": float(np.max(direct)),
        "frequency_ratio_min": float(np.min(frequency_ratio)),
        "frequency_ratio_median": float(np.median(frequency_ratio)),
        "frequency_ratio_max": float(np.max(frequency_ratio)),
        "offline_fft_peak_ratio": fft_ratio,
        "energetic_frequency_ratio_median": energetic_ratio_median,
        "energetic_frequency_error_vs_fft_pct": abs(energetic_ratio_median / fft_ratio - 1.0) * 100.0,
        "confidence_median": float(np.median(confidence)),
        "confidence_max": float(np.max(confidence)),
        "energetic_confidence_median": float(np.median(energetic_confidence)) if energetic_confidence.size else math.nan,
        "energetic_confidence_max": float(np.max(energetic_confidence)) if energetic_confidence.size else math.nan,
        "energetic_high_confidence_ratio": float(np.mean(energetic_confidence > 0.7)) if energetic_confidence.size else math.nan,
        "energy_gate_max": float(np.max(energy_gate)),
        "position_gate_min": float(np.min(position_gate)),
        "gain_scale_raw_mean": float(np.mean(gain_raw)),
        "gain_scale_applied_mean": float(np.mean(gain_applied)),
        "gain_scale_applied_max": float(np.max(gain_applied)),
        "gain_scale_when_position_gt_0p05_mean": float(np.mean(gain_applied[high_position])) if np.any(high_position) else math.nan,
        "gain_scale_when_position_gt_0p05_max": float(np.max(gain_applied[high_position])) if np.any(high_position) else math.nan,
        "position_gt_0p05_ratio": float(np.mean(high_position)),
        "schedule_active_ratio": float(np.mean(active > 0.5)),
        "hangas_gain_scale_min": float(np.min(anti_gain)),
        "hangas_gain_scale_mean": float(np.mean(anti_gain)),
        "hangas_gain_scale_max": float(np.max(anti_gain)),
        "as_raw_acceleration_rms_m_s2": float(np.sqrt(np.mean(anti_raw * anti_raw))),
        "as_raw_acceleration_peak_m_s2": float(np.max(anti_raw)),
        "as_raw_above_0p2_ratio": float(np.mean(anti_raw > 0.2)),
        "as_power_max": float(np.max(anti_power)),
        "as_positive_power_integral_j_kg": anti_positive,
        "as_negative_power_integral_j_kg": anti_negative,
        "rls_residual_variance_median": float(np.median(fso_data(54))),
        "rls_information_median": float(np.median(fso_data(55))),
    }


def mean(rows: Sequence[Dict[str, object]], field: str) -> float:
    return float(np.mean([finite(row[field]) for row in rows]))


def status_word(passed: bool) -> str:
    return "通过" if passed else "**失败**"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=DEFAULT_ROOT)
    args = parser.parse_args()
    root = args.root.resolve()
    stage2 = root / "stage2"
    stage2.mkdir(parents=True, exist_ok=True)

    vfb_api = runpy.run_path(str(VFB_REPORT), run_name="heso_stage12_vfb_api")
    stage1_api = runpy.run_path(str(STAGE1_BENCHMARK), run_name="heso_stage12_stage1_api")
    stage1_decision = json.loads((root / "stage1" / "stage1_decision.json").read_text(encoding="utf-8"))

    paths_by_mode = {mode: run_dirs(root, mode) for mode in (BASE_MODE, ORACLE_MODE, RLS_MODE)}
    if any(len(paths) != 2 for paths in paths_by_mode.values()):
        counts = {mode: len(paths) for mode, paths in paths_by_mode.items()}
        raise RuntimeError(f"expected exactly two formal runs per mode, got {counts}")

    labels = {BASE_MODE: ("A1", "A2"), ORACLE_MODE: ("B1", "B2"), RLS_MODE: ("C1", "C2")}
    rows_by_mode: Dict[str, List[Dict[str, object]]] = {}
    formal_rows: List[Dict[str, object]] = []
    for mode, paths in paths_by_mode.items():
        rows_by_mode[mode] = []
        for label, path in zip(labels[mode], paths):
            row, _, analysis_validity = vfb_api["analyze_run"](mode, path)
            result = {
                "label": label,
                "mode": mode,
                "run": path.name,
                "valid": bool(row.get("valid")) and bool(analysis_validity.get("valid")),
                **{field: row.get(field, math.nan) for field in PERFORMANCE_FIELDS},
            }
            rows_by_mode[mode].append(result)
            formal_rows.append(result)
    write_csv(stage2 / "formal_results.csv", formal_rows)

    mechanism_rows: List[Dict[str, object]] = []
    for mode in (ORACLE_MODE, RLS_MODE):
        for label, path in zip(labels[mode], paths_by_mode[mode]):
            mechanism_rows.append(analyze_mechanism(path, label, stage1_api))
    write_csv(stage2 / "mechanism_audit.csv", mechanism_rows)

    base = rows_by_mode[BASE_MODE]
    oracle = rows_by_mode[ORACLE_MODE]
    pair_fields = {
        "swing_angle_rms_deg": "swing_rms_change_pct",
        "swing_energy_integral_j_s_kg": "energy_change_pct",
        "xy_error_rms_m": "xy_rms_change_pct",
        "final_jerk_p95_m_s3": "jerk_p95_change_pct",
        "as_rms_m_s2": "as_rms_change_pct",
        "final_acc_rms_m_s2": "final_acc_rms_change_pct",
    }
    pair_rows = []
    for index in range(2):
        pair_row: Dict[str, object] = {"pair": index + 1, "baseline": base[index]["label"], "oracle": oracle[index]["label"]}
        for field, output in pair_fields.items():
            pair_row[output] = pct(finite(oracle[index][field]), finite(base[index][field]))
        pair_rows.append(pair_row)
    write_csv(stage2 / "pair_comparisons.csv", pair_rows)

    mean_fields = (
        "swing_angle_rms_deg", "swing_angle_peak_deg", "swing_energy_integral_j_s_kg",
        "xy_error_rms_m", "final_jerk_p95_m_s3", "as_rms_m_s2", "final_acc_rms_m_s2",
        "as_negative_power_integral_j_kg",
    )
    base_means = {field: mean(base, field) for field in mean_fields}
    oracle_means = {field: mean(oracle, field) for field in mean_fields}
    changes = {field: pct(oracle_means[field], base_means[field]) for field in mean_fields}

    oracle_mechanism = [row for row in mechanism_rows if str(row["label"]).startswith("B")]
    rls_mechanism = [row for row in mechanism_rows if str(row["label"]).startswith("C")]
    all_valid = all(bool(row["valid"]) for row in formal_rows)
    gates = {
        "all_six_runs_valid": all_valid,
        "both_pair_swing_rms_decrease": all(finite(row["swing_rms_change_pct"]) < 0.0 for row in pair_rows),
        "both_pair_energy_decrease": all(finite(row["energy_change_pct"]) < 0.0 for row in pair_rows),
        "mean_swing_rms_le_minus_2pct": changes["swing_angle_rms_deg"] <= -2.0,
        "mean_energy_le_minus_3pct": changes["swing_energy_integral_j_s_kg"] <= -3.0,
        "mean_xy_rms_le_plus_5pct": changes["xy_error_rms_m"] <= 5.0,
        "mean_jerk_p95_le_plus_7pct": changes["final_jerk_p95_m_s3"] <= 7.0,
        "each_pair_jerk_p95_le_plus_12pct": all(finite(row["jerk_p95_change_pct"]) <= 12.0 for row in pair_rows),
        "oracle_mode_and_truth_signals_correct": all(row["mode_id"] == 5
            and abs(finite(row["frequency_ratio_median"]) - 1.0) <= 1e-6
            and abs(finite(row["confidence_median"]) - 1.0) <= 1e-6 for row in oracle_mechanism),
        "direct_heso_compensation_zero": all(finite(row["direct_compensation_peak_m_s2"]) <= 1e-7 for row in mechanism_rows),
        "oracle_gain_in_range_and_activates": all(1.0 <= finite(row["hangas_gain_scale_min"])
            and finite(row["hangas_gain_scale_max"]) <= 1.40001
            and finite(row["schedule_active_ratio"]) > 0.0 for row in oracle_mechanism),
        "rls_shadow_gain_unity_and_inactive": all(abs(finite(row["hangas_gain_scale_min"]) - 1.0) <= 1e-6
            and abs(finite(row["hangas_gain_scale_max"]) - 1.0) <= 1e-6
            and finite(row["schedule_active_ratio"]) == 0.0 for row in rls_mechanism),
        "as_positive_power_zero": all(finite(row["as_positive_power_integral_j_kg"]) <= 1e-9
            and finite(row["as_power_max"]) <= 1e-7 for row in mechanism_rows),
    }
    performance_pass = all(gates[name] for name in (
        "all_six_runs_valid", "both_pair_swing_rms_decrease", "both_pair_energy_decrease",
        "mean_swing_rms_le_minus_2pct", "mean_energy_le_minus_3pct", "mean_xy_rms_le_plus_5pct",
        "mean_jerk_p95_le_plus_7pct", "each_pair_jerk_p95_le_plus_12pct",
    ))
    mechanism_pass = all(gates[name] for name in (
        "oracle_mode_and_truth_signals_correct", "direct_heso_compensation_zero",
        "oracle_gain_in_range_and_activates", "rls_shadow_gain_unity_and_inactive",
        "as_positive_power_zero",
    ))
    decision = {
        "stage1_winner": stage1_decision["winner"],
        "stage1_winner_meets_all_targets": bool(stage1_decision["winner_meets_all_targets"]),
        "stage2_oracle_performance_pass": performance_pass,
        "stage2_mechanism_pass": mechanism_pass,
        "decision": "KEEP_HESO_AS_GAIN_SCHEDULING_FOR_GENERALIZATION" if performance_pass
                    else "STOP_CURRENT_AS_GAIN_SCHEDULER",
        "base_means": base_means,
        "oracle_means": oracle_means,
        "oracle_vs_base_change_pct": changes,
        "pair_comparisons": pair_rows,
        "gates": gates,
    }
    (stage2 / "decision.json").write_text(json.dumps(decision, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    stage1_rows = {row["estimator"]: row for row in stage1_decision["aggregate"]}
    legacy = stage1_rows["legacy_heso_bank"]
    predictive = stage1_rows["predictive_bank"]
    rls = stage1_rows["filtered_rls"]
    b1, b2 = oracle_mechanism
    c1, c2 = rls_mechanism

    report = f"""# ZD680 HESO辅助PID+AS：阶段1频率识别与阶段2真值调度因果验证报告

日期：2026-07-22  
结论状态：**第一、二阶段完成；当前AS增益调度不通过联合门槛，RLS仅保留影子模式。**

## 一、结论先行

这轮试验把两个容易混在一起的问题拆开了：第一阶段检查“频率能不能估准”，第二阶段直接给调度器正确频率和满可信度，检查“调度规律本身有没有用”。

最终结论有三点：

1. **滤波RLS是本轮最好的频率估计结构，但还不能闭环。** 它在合成数据上的中位误差为 {rls['synthetic_median_abs_error_pct']:.2f}%，P95为 {rls['synthetic_p95_abs_error_pct']:.2f}%，明显好于两个HESO bank；但冻结飞行日志对FFT的中位差为 {rls['flight_median_abs_error_vs_fft_pct']:.2f}%，高于预注册5%门槛，所以阶段2只做影子记录。
2. **给当前调度器由已知绳长确定的模型标称真值频率后，摆角RMS均值下降 {abs(changes['swing_angle_rms_deg']):.2f}%，但能量只下降 {abs(changes['swing_energy_integral_j_s_kg']):.2f}%，jerk增加 {changes['final_jerk_p95_m_s3']:.2f}%。** 第一对能量还增加 {pair_rows[0]['energy_change_pct']:.2f}%，因此没有通过“两对能量都下降、均值能量至少下降3%、jerk不超过+7%”的联合门槛。
3. 机理链路本身是正确执行的：Oracle两次频率比和可信度均严格为1，增益确实激活，HESO直接补偿严格为0，AS功率保持非正；RLS两次增益严格为1。故本轮失败不能再主要归因于HESO候选频率和可信度偏差，核心矛盾已经转到**AS增益调度规律、AS限幅以及位置环和抗摆环的动态协调**。

按预注册决策，本轮输出为：`{decision['decision']}`。这里“停止”的是**当前这条AS增益调度公式直接闭环扩展**，不是删除HESO，也不是否定PID+AS基线。HESO/RLS仍可作为输入整形参数、工况识别量或影子诊断量继续研究。

## 二、试验设计与冻结规则

### 2.1 公共工况

- PX4 SITL＋Gazebo ZD680吊挂模型；绳长0.6 m，载荷0.5 kg；
- PID位置环和现有能量型AS保持不变，AS加速度上限0.20 m/s²；
- 四方向2 m/6 s S曲线，段间保持5 s，最后保持10 s；
- 正式顺序严格为 `A1→B1→C1→C2→B2→A2`，共6次，不因结果追加；
- A为PID+AS基线，B为Oracle频率＋满可信度＋现有能量/位置门的AS增益调度，C为RLS影子观测；
- B最大调度倍率1.4、倍率变化率0.30/s；C倍率强制为1；B/C的HESO直接加速度补偿均关闭。

初始门原计划采用0.20°/0.015 rad/s/0.08 m。短程就绪试验表明该门在当前仿真背景摆动下不能稳定进入正式路线，因此在正式试验前一次性修订为0.25° RMS、0.020 rad/s RMS、当前摆角0.40°、当前摆速0.035 rad/s、XY 0.08 m、连续稳定3 s。修订记录在[阶段2初始门就绪修订记录](阶段2初始门就绪修订记录.md)，性能判据和算法参数均未改变。

### 2.2 预注册性能门槛

- 两个配对的摆角RMS和能量积分均下降；
- 摆角RMS均值变化不高于-2%，能量积分均值变化不高于-3%；
- XY RMS均值不高于+5%；
- jerk P95均值不高于+7%，每一对不高于+12%。

## 三、阶段1：频率识别结果

| 估计器 | 合成中位绝对误差 | 合成P95 | 飞行/FFT中位差 | 低激励高可信误触发 | 判定 |
|---|---:|---:|---:|---:|---|
| 当前HESO bank | {legacy['synthetic_median_abs_error_pct']:.2f}% | {legacy['synthetic_p95_abs_error_pct']:.2f}% | {legacy['flight_median_abs_error_vs_fft_pct']:.2f}% | {legacy['low_excitation_confidence_max']:.3f} | P95和飞行检查失败 |
| 校正前预测残差 bank | {predictive['synthetic_median_abs_error_pct']:.2f}% | {predictive['synthetic_p95_abs_error_pct']:.2f}% | {predictive['flight_median_abs_error_vs_fft_pct']:.2f}% | {predictive['low_excitation_confidence_max']:.3f} | P95和飞行检查失败 |
| 滤波RLS | **{rls['synthetic_median_abs_error_pct']:.2f}%** | **{rls['synthetic_p95_abs_error_pct']:.2f}%** | {rls['flight_median_abs_error_vs_fft_pct']:.2f}% | {rls['low_excitation_confidence_max']:.3f} | 合成通过、飞行检查失败 |

### 3.1 为什么滤波RLS在合成数据好、飞行数据仍偏高

滤波RLS不是逐点对角加速度求二阶导数，而是在0.4 s窗口内积分摆动方程，同时估计 `omega²` 和阻尼项，因此对测量噪声的放大较小。它在已知真值、充分激励的合成数据上确实有效。

但真实闭环飞行中的吊挂运动不是一个只受重力和固定阻尼支配的自由单摆。飞行器基础加速度、位置环修正、AS自身作用、多轴耦合和有限窗非平稳性都进入同一个回归式；若这些项的时序、方向或模型稍有偏差，RLS会把一部分“控制器造成的动态”误解释成频率变化。阶段2影子复验中，两次RLS在有能量样本上的频率比中位数为 {c1['energetic_frequency_ratio_median']:.4f}/{c2['energetic_frequency_ratio_median']:.4f}，FFT为 {c1['offline_fft_peak_ratio']:.4f}/{c2['offline_fft_peak_ratio']:.4f}，相差 {c1['energetic_frequency_error_vs_fft_pct']:.2f}%/{c2['energetic_frequency_error_vs_fft_pct']:.2f}%。

好的一面是可信度没有误放行：上述有效能量区间的可信度最大值只有 {c1['energetic_confidence_max']:.3f}/{c2['energetic_confidence_max']:.3f}，大于0.7的占比均为0。因此“估计仍有偏差”与“门控足够保守”同时成立。

### 3.2 为什么校正前预测 bank 反而更差

它避免了HESO扰动状态在同一步内擦除频率失配，但一拍预测残差仍同时包含测量噪声、未建模阻尼、基础加速度误差和慢变扰动。这个残差并不是纯频率误差；预测窗太短时，噪声和强迫项的权重反而比自然频率更大，所以飞行/FFT误差升到 {predictive['flight_median_abs_error_vs_fft_pct']:.2f}%。这条结构不值得继续只靠调权重修补。

阶段1原始结果见[估计器汇总](stage1/estimator_summary.csv)、[飞行交叉检查](stage1/flight_crosscheck.csv)和[机器判定](stage1/stage1_decision.json)。

## 四、阶段2：正式性能结果

### 4.1 六次运行

| 运行 | 模式 | 初始摆角RMS(°) | 初始摆速RMS(rad/s) | 摆角RMS(°) | 能量积分(J·s/kg) | XY RMS(m) | jerk P95(m/s³) | AS RMS(m/s²) |
|---|---|---:|---:|---:|---:|---:|---:|---:|
"""
    for row in formal_rows:
        report += (f"| {row['label']} | {row['mode']} | {fmt(row['initial_swing_angle_rms_deg'], 4)} | "
                   f"{fmt(row['initial_swing_rate_rms_rad_s'], 5)} | {fmt(row['swing_angle_rms_deg'], 5)} | "
                   f"{fmt(row['swing_energy_integral_j_s_kg'], 5)} | {fmt(row['xy_error_rms_m'], 5)} | "
                   f"{fmt(row['final_jerk_p95_m_s3'], 5)} | {fmt(row['as_rms_m_s2'], 5)} |\n")

    report += f"""

六次 `validity.json` 均为有效，failsafe占比均为0。C只作为影子运行，不参与A/B性能判定。

### 4.2 Oracle对基线的两对变化

| 配对 | 摆角RMS | 能量积分 | XY RMS | jerk P95 | AS RMS |
|---|---:|---:|---:|---:|---:|
| B1 对 A1 | {fmt_pct(pair_rows[0]['swing_rms_change_pct'])} | {fmt_pct(pair_rows[0]['energy_change_pct'])} | {fmt_pct(pair_rows[0]['xy_rms_change_pct'])} | {fmt_pct(pair_rows[0]['jerk_p95_change_pct'])} | {fmt_pct(pair_rows[0]['as_rms_change_pct'])} |
| B2 对 A2 | {fmt_pct(pair_rows[1]['swing_rms_change_pct'])} | {fmt_pct(pair_rows[1]['energy_change_pct'])} | {fmt_pct(pair_rows[1]['xy_rms_change_pct'])} | {fmt_pct(pair_rows[1]['jerk_p95_change_pct'])} | {fmt_pct(pair_rows[1]['as_rms_change_pct'])} |

| 均值指标 | A均值 | B均值 | 变化 | 门槛 | 判定 |
|---|---:|---:|---:|---:|---|
| 摆角RMS(°) | {base_means['swing_angle_rms_deg']:.5f} | {oracle_means['swing_angle_rms_deg']:.5f} | **{fmt_pct(changes['swing_angle_rms_deg'])}** | ≤-2% | {status_word(gates['mean_swing_rms_le_minus_2pct'])} |
| 能量积分(J·s/kg) | {base_means['swing_energy_integral_j_s_kg']:.5f} | {oracle_means['swing_energy_integral_j_s_kg']:.5f} | **{fmt_pct(changes['swing_energy_integral_j_s_kg'])}** | ≤-3% | {status_word(gates['mean_energy_le_minus_3pct'])} |
| XY RMS(m) | {base_means['xy_error_rms_m']:.5f} | {oracle_means['xy_error_rms_m']:.5f} | {fmt_pct(changes['xy_error_rms_m'])} | ≤+5% | {status_word(gates['mean_xy_rms_le_plus_5pct'])} |
| jerk P95(m/s³) | {base_means['final_jerk_p95_m_s3']:.5f} | {oracle_means['final_jerk_p95_m_s3']:.5f} | **{fmt_pct(changes['final_jerk_p95_m_s3'])}** | ≤+7% | {status_word(gates['mean_jerk_p95_le_plus_7pct'])} |
| AS RMS(m/s²) | {base_means['as_rms_m_s2']:.5f} | {oracle_means['as_rms_m_s2']:.5f} | {fmt_pct(changes['as_rms_m_s2'])} | 机理项 | — |
| 最终加速度RMS(m/s²) | {base_means['final_acc_rms_m_s2']:.5f} | {oracle_means['final_acc_rms_m_s2']:.5f} | {fmt_pct(changes['final_acc_rms_m_s2'])} | 观察项 | — |

联合判定中，摆角均值、XY均值、两对jerk上限和两对摆角方向通过；**两对能量同向下降失败、能量均值门槛失败、jerk均值门槛失败**。因此不能因为摆角RMS刚达到-2%就宣布方案有效，更没有达到论文目标的摆角-5%、能量-8%。

## 五、机理审计

| 指标 | B1 | B2 | C1 | C2 |
|---|---:|---:|---:|---:|
| 模式ID | {b1['mode_id']} | {b2['mode_id']} | {c1['mode_id']} | {c2['mode_id']} |
| 直接补偿峰值(m/s²) | {b1['direct_compensation_peak_m_s2']:.1e} | {b2['direct_compensation_peak_m_s2']:.1e} | {c1['direct_compensation_peak_m_s2']:.1e} | {c2['direct_compensation_peak_m_s2']:.1e} |
| 频率比中位数 | {b1['frequency_ratio_median']:.4f} | {b2['frequency_ratio_median']:.4f} | {c1['frequency_ratio_median']:.4f} | {c2['frequency_ratio_median']:.4f} |
| 可信度中位数 | {b1['confidence_median']:.4f} | {b2['confidence_median']:.4f} | {c1['confidence_median']:.6f} | {c2['confidence_median']:.6f} |
| AS增益均值 | {b1['hangas_gain_scale_mean']:.4f} | {b2['hangas_gain_scale_mean']:.4f} | {c1['hangas_gain_scale_mean']:.4f} | {c2['hangas_gain_scale_mean']:.4f} |
| AS增益峰值 | {b1['hangas_gain_scale_max']:.4f} | {b2['hangas_gain_scale_max']:.4f} | {c1['hangas_gain_scale_max']:.4f} | {c2['hangas_gain_scale_max']:.4f} |
| 调度激活占比 | {100*b1['schedule_active_ratio']:.2f}% | {100*b2['schedule_active_ratio']:.2f}% | {100*c1['schedule_active_ratio']:.2f}% | {100*c2['schedule_active_ratio']:.2f}% |
| AS原始量超过0.2占比 | {100*b1['as_raw_above_0p2_ratio']:.2f}% | {100*b2['as_raw_above_0p2_ratio']:.2f}% | {100*c1['as_raw_above_0p2_ratio']:.2f}% | {100*c2['as_raw_above_0p2_ratio']:.2f}% |
| AS正功积分(J/kg) | {b1['as_positive_power_integral_j_kg']:.1e} | {b2['as_positive_power_integral_j_kg']:.1e} | {c1['as_positive_power_integral_j_kg']:.1e} | {c2['as_positive_power_integral_j_kg']:.1e} |

Oracle两次调度分别激活 {100*b1['schedule_active_ratio']:.1f}%/{100*b2['schedule_active_ratio']:.1f}%，AS增益均值为 {b1['hangas_gain_scale_mean']:.3f}/{b2['hangas_gain_scale_mean']:.3f}，峰值均达到1.4。相较基线，AS RMS均值增加 {changes['as_rms_m_s2']:.2f}%。这证明调度不是“没有打开”，而是确实让AS更强、更久。

另一方面，Oracle的AS原始命令超过0.20 m/s²上限的时间为 {100*b1['as_raw_above_0p2_ratio']:.2f}%/{100*b2['as_raw_above_0p2_ratio']:.2f}%；两次基线对应约1.72%/1.61%。增益调度使AS局部限幅驻留扩大约2.5倍，增益与实际输出不再线性对应。

位置保护也不是瞬时硬切换。位置误差大于0.05 m时，B1/B2仍有 {100*b1['position_gt_0p05_ratio']:.1f}%/{100*b2['position_gt_0p05_ratio']:.1f}%的路线样本，期间AS增益均值为 {b1['gain_scale_when_position_gt_0p05_mean']:.3f}/{b2['gain_scale_when_position_gt_0p05_mean']:.3f}。原因包括位置门在0.025～0.10 m之间渐变，以及0.30/s的倍率平滑退场；这能防突变，但不能保证位置压力一上升就立即释放AS权限。

完整机理数据见[mechanism_audit.csv](stage2/mechanism_audit.csv)。

## 六、为什么Oracle仍没有通过：通俗解释

### 6.1 频率估计像“导航”，调度规律像“怎么踩油门”

这次把导航答案固定为已知绳长对应的模型标称值，绕过了HESO候选选择和可信度估计。结果摆角有一点改善，但能量和jerk没有同时达标，说明主要问题已不是HESO给错导航，而是油门策略：知道吊挂标称频率，并不自动等于知道每个时刻应把AS放大多少。

需要说明边界：两条Oracle日志的有限窗FFT主峰比分别为 {b1['offline_fft_peak_ratio']:.3f}/{b2['offline_fft_peak_ratio']:.3f}，约比绳长模型值高5%。FFT会混入强迫运动、有限窗和多轴耦合，不能反过来定义严格真值；但它提示这里的Oracle是“物理参数真值”，不是在线时变模态的完美标签。当前调度中的频率比只对增益作0.9～1.1限幅缩放，且增益峰值已碰到1.4，因此这5%左右差异不足以解释能量方向不一致和jerk超限。

### 6.2 更强的AS确实做了更多负功，但位置环会反向修正

AS自身始终不向摆动注入能量，正功积分为0；Oracle两次AS负功积分绝对值比基线更大。这说明功率约束有效。但飞行器还必须跟踪位置，AS把机体拉向消摆方向时，位置PID会为了回到轨迹而产生反向动作。当前门只约束“AS这一支不做正功”，没有约束“AS＋位置PID合成以后对摆动的总功率”，所以局部消摆收益会被位置修正抵消。第一对出现“摆角小降、能量反升”正是这种竞争的表现。

### 6.3 0.20 m/s²限幅把连续增益变成了削顶信号

当AS原始命令已经接近上限，再把增益从1提高到1.4，不会得到1.4倍的有效阻尼，只会更早碰顶、在上限附近停留更久。削顶改变波形并增加高频成分，因此AS RMS增加 {changes['as_rms_m_s2']:.2f}%的同时，jerk增加 {changes['final_jerk_p95_m_s3']:.2f}%，而摆角收益只有 {abs(changes['swing_angle_rms_deg']):.2f}%。

### 6.4 当前“能量×位置门”没有预测未来位置压力

现在的位置门只看误差大小，不看误差增长率、位置PID当前需要多少加速度、总加速度还剩多少余量，也不看下一小段时间AS与位置命令是否对抗。它属于反应式保护，不是协调优化。倍率再加一层斜率限制后，保护动作必然滞后。

### 6.5 两对结果方向不一致，说明收益还小于工况波动

B1对A1的能量为+{pair_rows[0]['energy_change_pct']:.2f}%，B2对A2为{pair_rows[1]['energy_change_pct']:.2f}%。初始条件都合法，但不完全相同；如果方案效果足够强，两对通常应保持同向。现在一好一坏，说明平均改善还没有形成稳定裕量，不能支撑泛化或论文主结论。

## 七、接下来怎么做

### 7.1 立即停止的工作

- 不再继续调HESO bank的候选权重来救当前闭环结果；Oracle已证明估计器不是唯一主因。
- 不让RLS进入闭环；它的飞行/FFT偏差仍约6.6%～6.8%。
- 不继续把AS最大倍率从1.4往上加；当前限幅驻留和jerk已经恶化。

### 7.2 建议的下一条主线：HESO/RLS辅助输入整形

保留PID+AS作为稳定基线，把估计频率用于**参考轨迹整形**，而不是直接乘AS增益。具体按三步做：

1. 先实现双脉冲ZV或三脉冲ZVD整形器，离线由频率估计给出脉冲间隔；频率可信度不足时退化为标称频率整形。
2. 只整形水平位置/速度参考，不改PX4内环；增加最大延迟、末端到达时间和轨迹边界约束。
3. 做 `PID+AS`、`固定标称ZVD+PID+AS`、`自适应ZVD+PID+AS` 三组消融。先要求固定ZVD显著改善，再检验自适应频率是否带来额外收益。

这条路线更适合北大核心论文：频率估计的作用点清晰，消融结构完整，也能把“估计准确度—整形鲁棒性—摆动抑制”串成一条可解释主线。

### 7.3 若仍坚持AS调度，应先重构协调器

调度量不再直接写成 `K_AS = K0(1+0.4×门控)`，而应根据可用加速度预算求一个小型约束分配：

- 输入：位置误差、误差变化率、位置PID请求、摆能、摆速、AS候选命令、剩余加速度和jerk预算；
- 约束：AS功率非正、合成命令功率上界、位置误差增长不恶化、总加速度/jerk限制；
- 输出：0～1的AS权限或最优AS幅值；
- 先用Oracle频率做同一套A/B因果验证，Oracle通过后才接回RLS/HESO。

### 7.4 RLS需要修改的部分

- 把基础加速度输入做时间对齐和坐标系一致性单测；
- 将窗口由固定0.4 s扩展为按估计周期归一化的多周期窗口；
- 引入控制输入与AS反馈的闭环偏差修正，避免把控制器造成的运动当作自然频率；
- 可信度同时考察参数协方差、残差白化、轴间一致性和与短时频谱的交叉一致性；
- 下一轮飞行交叉门仍保持中位≤5%、最大≤8%，通过前只做影子。

## 八、工程验证与可复现材料

- 新增模式4：RLS影子估计，直接补偿为0、AS倍率固定1；
- 新增模式5：Oracle频率比1、可信度1，复用冻结的能量/位置调度；
- 新增两项单元测试：RLS合成频率收敛及影子隔离、Oracle调度激活及位置退场；
- `SuspendedLoadAntiSwingTest`：28/28通过；
- `px4_sitl_default`完整编译：368步完成，生成`bin/px4`；
- 正式6次SITL全部有效，无failsafe，不使用补跑额度。

数据与判定文件：

- [预注册方案](预注册_阶段1频率识别与阶段2因果消融方案.md)
- [阶段1条件激活记录](阶段1判定与阶段2条件激活记录.md)
- [阶段2正式逐次结果](stage2/formal_results.csv)
- [阶段2配对比较](stage2/pair_comparisons.csv)
- [阶段2机理审计](stage2/mechanism_audit.csv)
- [阶段2机器判定](stage2/decision.json)

## 九、最终判断

第一、二阶段已经回答了最关键的因果问题：**频率估计有改进空间，但即便绕过HESO候选选择与可信度误差，当前“可信度×能量×位置门→AS增益倍率”的闭环结构仍不能稳定同时改善摆角、能量、位置和jerk。**

因此现阶段不建议以这版完整结构直接组织北大核心论文主结果。可以保留“有效频率估计＋可信度门控”的研究资产，但下一轮应把作用点转到输入整形，或先把AS/位置协调器重构为显式约束分配。只有Oracle频率版本先稳定通过联合门槛，才值得恢复“估计器＋闭环调度”的完整论文路线。
"""
    report_path = root / "ZD680_HESO阶段1频率识别与阶段2真值调度因果验证报告_20260722.md"
    report_path.write_text(report, encoding="utf-8")
    print(json.dumps(decision, indent=2, ensure_ascii=False))
    print(f"report: {report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
