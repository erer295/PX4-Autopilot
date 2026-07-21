# ZD680 LADRC + AS 有界验证归档（2026-07-21）

本目录是任务书 `ZD680_LADRC_AS_Codex验证实验任务书_v1.0.md` 的一次性 go/no-go 验证归档。

最终结果：六组 LADRC 参数均未通过严格 D0 初态门禁，路线按规则在 Stage 1 前置门槛停止；未运行 D-T、D-F、D-FT。唯一决策见 `decision.txt`，完整解释见 `final_report.md`。

## 目录

- `run_experiments.py`：启动全新 PX4/Gazebo、冻结组别/参数、起飞、调用既有 recorder、降落并恢复 PID+AS 安全运行态。
- `analyze_results.py`：读取 ULog 和关节 CSV，审计时序、初态与安全状态，计算指标、校验值并生成 SVG。
- `runs/`：9 次有数据飞行；每次包含 ULog、关节/事件/live CSV、参数请求、PX4/recorder 控制台日志。
- `launcher/`、`summaries/`：运行器总日志与既有 recorder 摘要。
- 根目录的 `run_manifest.csv`、`metrics_raw.csv`、`timing_quality.csv`、`parameter_sets.csv`、`paired_comparison.csv` 是任务书要求的机器可读交付；`report/` 保留同内容镜像和分析摘要。
- `plots/`：Stage 0/T0 时序图和六组筛选图；另有说明 Stage 2/3 未运行的占位图，防止把缺失误当成遗漏。
- `code_audit.md`：控制链、公式、单位、切换、限幅、几何、扰动通道与日志审计。
- `final_report.md`：按任务书第 15 节固定的 14 段结构编写。

## 环境与依赖

- PX4 仓库：`/home/lst/PX4-Autopilot`
- 基线 revision：`b9671fe69a`，另含本目录说明的诊断 worktree 改动
- 仿真目标：`px4_sitl_default` + `gz_zd680_hang`
- Python：3.x
- Python 包：`numpy`、`pyulog`、PX4 自带 `pymavlink`
- 图形输出不依赖 matplotlib；本机 matplotlib 与 NumPy 2 ABI 不兼容，所以脚本直接输出标准 SVG。

## 构建

从仓库根目录执行：

```bash
make px4_sitl_default -j2
```

本轮首次全量链接成功；报告收尾时再次增量构建，输出 `ninja: no work to do.`，退出码 0。

## 复现实验

下列命令会真实启动 SITL 并产生新日志。由于本轮已经达到六组调参上限，只有在“复现实验”而非继续调参时才应重新运行。

四组 T0 冒烟：

```bash
python3 src/modules/mc_rate_control/validation/ladrc_as_gate_20260721/run_experiments.py \
  --stage smoke --groups C0,C1,C2,C3 --parameter-sets T0 --repetitions 1
```

只用 C1/D0 复现 T1～T5 有界筛选：

```bash
python3 src/modules/mc_rate_control/validation/ladrc_as_gate_20260721/run_experiments.py \
  --stage smoke --groups C1 --parameter-sets T1,T2,T3,T4,T5 --repetitions 1
```

门禁固定为：位置误差 ≤0.10 m、水平速度 ≤0.05 m/s 且连续 3 s；最近 3 s 合成摆角 RMS ≤0.3°、摆角速度 RMS ≤0.015 rad/s；当前摆角 ≤1°、当前摆速 ≤0.06 rad/s。失败后 recorder 切 LOITER，运行器随后执行 LAND。

`run_experiments.py` 支持 `--stage stage1` 的固定 D0 S 曲线运输路线，但本轮不得执行，因为所有 LADRC 参数均未通过前置门槛。脚本不实现 Stage 2/3 扰动批量运行，避免误跳过硬停止条件。

## 重新生成分析

```bash
python3 src/modules/mc_rate_control/validation/ladrc_as_gate_20260721/analyze_results.py
```

脚本会覆盖生成：

- `run_manifest.csv`（并镜像到 `report/`）
- `metrics_raw.csv`（并镜像到 `report/`）
- `timing_quality.csv`（并镜像到 `report/`）
- `parameter_sets.csv`（并镜像到 `report/`）
- `paired_comparison.csv`（并镜像到 `report/`）
- `report/analysis_summary.json`
- `plots/*.svg`

`paired_comparison.csv` 只有表头是预期行为：没有任何 LADRC 运行通过初态门禁，不能形成有效 PID/LADRC 正式配对。

## 指标口径

- 分析窗口：门禁通过则取 `action_*`；门禁失败则取失败的完整 `hold_current_position_check` 或 `hang_initial_condition`。两类窗口不可互称正式配对。
- rate/attitude error：实际值减插值对齐后的 setpoint；插值只在无长缺口的原始有效信号上做。
- 合成摆角：`sqrt(theta_roll² + theta_pitch²)`。
- 单位质量摆动能量：`0.5*(L*theta_dot_resultant)² + g*L*(1-cos(theta_resultant))`，单位 J/kg；积分单位 J·s/kg。
- normalized torque 是 PX4 归一化命令，不是 N·m。
- 时序长缺口阈值：`max(3*median_period, 20 ms)`；出现长缺口、非单调采样时间、窗口覆盖不足 99% 时标无效。
- 改善率口径虽然在任务书中定义，但本轮没有有效配对，故不计算改善率，不做显著性声明。

## 数据追溯

`run_manifest.csv` 为每个 ULog 保存绝对路径、字节数和 SHA-256。原始数据总量较大，分析时不要移动单个 run 内的文件；如整体搬迁本目录，请重新运行分析以刷新绝对路径与校验值。

## 安全和边界

- 所有 9 次有数据飞行均自动降落并解除武装，无 failsafe、故障检测或电机高饱和。
- 最后一次筛选后已把持久 SITL 参数恢复并保存为 PID+AS（LADRC 0、AS 1）。运行器也包含同样的飞后恢复逻辑。
- 本结果仅是 SITL 仿真，不是实机验证。
- 不应继续添加 RBF、TD 或复杂观测器来“救援”本路线；下一步按任务书只能冻结 PID+AS 工程基线。
