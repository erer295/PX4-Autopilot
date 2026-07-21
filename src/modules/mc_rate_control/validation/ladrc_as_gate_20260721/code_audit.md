# ZD680 内环 LADRC + 外环 AS：代码、控制链与几何审计

审计日期：2026-07-21（Asia/Shanghai）  
代码基线：`b9671fe69a`，分支 `rbf-ladrc-work`，外加本目录所列的只读诊断改动。  
范围：角速度内环、位置/AS 外环、Gazebo 吊挂几何、扰动工具、ULog 时序与信号可观测性。

## 1. 控制组可复现性

| 组 | `MC_LADRC_EN` | `MC_HANG_AS_EN` | 其余公共控制结构 |
|---|---:|---:|---|
| C0 | 0 | 0 | PX4 位置/速度 PID，外环 LADRC/TD、RBF、PAS、VFB 均关闭 |
| C1 | 1 | 0 | 同上 |
| C2 | 0 | 1 | 同上；冻结既有能量 AS 参数 |
| C3 | 1 | 1 | 同上；冻结既有能量 AS 参数 |

`run_experiments.py` 在每次起飞前显式写入并回读相关参数；每次运行使用新的 PX4/Gazebo 进程。现有开关是三轴统一开关，因此 C1/C3 的 yaw 也切到 LADRC。本轮没有向 yaw 注入扰动，分析单列 yaw，研究结论只允许覆盖 roll/pitch。

脚本在每次运行结束后把持久运行态恢复为 PID+AS 安全基线：`MC_LADRC_EN=0`、`MC_HANG_AS_EN=1`，同时继续关闭 TD、RBF、PAS 和位置外环 LADRC。本轮结束时也已人工执行一次 `param save` 并回读确认这两个关键值为 0/1。

## 2. 实际 LADRC 控制链

实现位于 `mc_rate_control/ladrc_rate_control/LadrcRateControl.*`，控制对象按一阶角速度模型处理：

\[
\dot\omega_i=b_{0,i}u_i+f_i.
\]

实际前向 Euler 离散 LESO 为：

\[
e_k=\omega_k-z_{1,k},
\]

\[
z_{1,k+1}=z_{1,k}+\Delta t\,[z_{2,k}+b_0u_{applied,k-1}+2\omega_o e_k],
\]

\[
z_{2,k+1}=z_{2,k}+\Delta t\,[\omega_o^2e_k].
\]

控制律为：

\[
u_{raw}=\frac{\omega_c(\omega_{sp}-z_1)-z_2}{b_0}-D\dot\omega,
\qquad
u=\operatorname{clip}(u_{raw},-u_{lim},u_{lim}).
\]

- `u` 是 PX4 `vehicle_torque_setpoint` 的归一化力矩命令，不是 N·m。
- `b0` 的单位是 `(rad/s²)/normalized_torque`；`z2` 是总扰动加速度量（rad/s²）。本轮没有完成物理输入增益标定，禁止把 `z2` 称为精确 N·m 外力矩估计。
- `beta1=2*wo`、`beta2=wo²`，与一阶对象的二阶 LESO 匹配。
- 本轮保留已有的小型角加速度阻尼 `D_R=D_P=0.003`。它是 T0 已使用的稳定配置组成部分；所有 T0～T5 均固定不变，未作为扫参变量。

## 3. 时间戳、实际输入与切换

- 控制循环由 `vehicle_angular_velocity` 更新驱动，`dt` 取 `timestamp_sample` 差值，并限在 0.125 ms～20 ms。
- LESO 不使用未经后处理的理论输入。控制器发布完 yaw 低通及可选电池缩放后的最终归一化力矩后，通过 `setAppliedTorque(_last_published_torque)` 把实际发布值作为下一拍 `u_applied`。
- 控制分配器的正/负饱和标志同时馈给 PID 与 LADRC。分配器饱和时 LADRC 冻结 `z2_dot`，防止把执行器权限不足错误学习为外扰。
- PID→LADRC 空中切换时，`z1` 初始化为实测速率，`z2` 依据当前设定值和上拍已发布力矩求取，输出以当前力矩为起点；LADRC→PID 时清零 PID 积分器。地面状态会重置 LESO，避免学习地面接触。
- 现有 `MC_LADRC_EN` 是三轴统一开关。为避免明显重构，本轮没有改成分轴开关。

## 4. 限幅与公平性

- PID 与 LADRC 最终都经过相同 PX4 控制分配器和电机物理约束。
- 本轮 LADRC roll/pitch 本地限幅为 ±0.30，未高于 PID 的执行器权限，因此候选组没有获得额外控制权；严格意义上它比 PID 多一道保守本地限制。
- 诊断结果显示 T0～T5 的 LADRC 本地 roll/pitch 限幅占比均为 0；因此本轮失败不是 ±0.30 本地限幅造成的。
- 所有运行的 `allocator_torque_not_achieved_ratio=0`，电机高饱和占比为 0。T4/T5 出现的是水平加速度总预算饱和（分别约 44.2%/59.5%），属于姿态/摆动恶化后外环追赶的结果。
- 仿真电池冻结：`SIM_BAT_DRAIN=3600`、`SIM_BAT_MIN_PCT=100`。

## 5. 禁用支路确认

本轮所有运行均显式设置：

- `MC_LADRC_TD_MODE=0`；
- `MC_RBF_EN=MC_RBF_INJECT_EN=MC_RBF_LEARN_EN=0`；
- `MC_PLADRC_EN=MC_PLADRC_TD_EN=0`；
- `MC_HANG_PAS_MD=MC_HANG_PAS_POS=0`；
- 没有启用新的观测器、RBF、TD、PAS、VFB、MPC 或神经网络。

## 6. 日志补充

已有 ULog 能记录姿态、角速度及设定值、归一化力矩、控制分配状态、电机、位置/速度、AS 分解、吊挂关节、failsafe 和故障标志，但原 `rate_ctrl_status` 只复用了积分字段保存 `-z2/b0`，缺少直接的 `z1/z2/u_raw/local_limit`。

为满足任务书可审计性，仅增加诊断，不改变控制律：

- `debug_array` 名称 `ladrc_td`、ID 682 的索引 21～23：`z1` roll/pitch/yaw；
- 索引 24～26：`z2` roll/pitch/yaw；
- 索引 27～29：本地限幅前 `u_raw`；
- 索引 30～32：本地限幅标志。

所有 7 个 LADRC 归档 ULog 均成功读取这些信号；2 个 PID 组按设计不发布 LADRC 专属项。

## 7. 吊挂几何

模型采用 Gazebo SDF 的 FLU/ENU 约定；PX4 机体系为 FRD、局部位置为 NED。

- `zd680_base/base_link` 主体惯性：质量 1.72 kg，惯性中心位于 link 原点，`Ixx=Iyy=0.0105 kg·m²`、`Izz=0.0200 kg·m²`。
- 四个 rotor link 各 0.12 kg，关于 X/Y 对称，均位于 base 原点上方 0.06 m。只计无人机主体与四个 rotor link，合计 2.20 kg，聚合质心约为 `[0,0,+0.0131] m`（base-link FLU）。
- 万向节挂点为 `[0,0,-0.20] m`（相对 base_link），故相对上述无人机聚合质心的向量约为

  \[
  r_h=[0,0,-0.2131]\ \mathrm{m}\quad\text{(FLU)}.
  \]

- 绳长 0.60 m；payload 质量 0.50 kg；绳 link 质量 0.010 kg；gimbal link 质量 0.005 kg。payload/绳/gimbal 不计入“无人机自身质心”计算。
- 吊绳竖直时，张力与 `r_h` 平行，`r_h×T_L≈0`；存在水平张力分量时会直接形成 roll/pitch 力矩。因此不能说挂点与质心重合，也不能把全部吊载效应都等价成纯力矩。
- 仿真挂点/惯量在本轮未改动；没有真实平台测量文件可用于核对，故这里只确认仿真几何，不外推为实机几何。

## 8. 扰动注入通道审计

现有 recorder 向 `/world/<world>/wrench/persistent` 发布 `gz.msgs.EntityWrench`，默认实体是 `zd680_hang_0::base_link`、类型 `LINK`：

- 力单位 N、力矩单位 N·m；消息只含 force/torque，不含独立作用点偏置。
- `world` 输入直接按 Gazebo 世界 ENU 发布；`local_ned` 使用 `(N,E,D)→(E,N,-D)`；`body` 先按当前 yaw 将 PX4 body FRD 投影到 NED，再转换为 ENU。
- 纯力和纯力矩是独立字段。对 link 的无偏置力不会由工具额外添加人为力矩；若要验证偏心力，必须显式设计力矩或换用支持作用点的接口。
- 调度器支持确定时刻、梯形/正弦波和结束时零 wrench 覆盖及 clear 重试，并在事件与 live CSV 中保存请求/发布的 ENU/NED 值。

本轮因 Stage 1 失败，按硬停止规则没有实际注入 D-T、D-F 或 D-FT；因此这里是通道审计，不是扰动幅值实测标定。

## 9. 发现的问题与结论边界

1. 六组 LADRC 都无法建立正式 D0 初态，是清晰性能失败，不是日志或环境阻塞。
2. Gazebo 启动器没有暴露显式随机种子参数；每次使用相同新世界/模型初态，但 manifest 将 seed 标为 `not_exposed_by_harness`，不伪称已固定数值种子。
3. 三轴统一开关使 yaw 也切换；yaw 数据已保留，但主结论仅针对 roll/pitch。
4. `z2` 未做 N·m 标定；只能称“总扰动估计”。
5. C0 首次冒烟在发现持久 T0 值前启动，其 LADRC 参数字段曾写为源码默认 50/8/30/0/0.35；C0 实际使用 PID，所以这些禁用字段不影响其控制。之后所有 LADRC/C2/C3 运行使用回读的 T0=120/5.5/16/0.003/0.30。
6. 运行前工作树已有与本任务无关的 Gazebo 子模块、`mc_pos_control` 工具和结果改动；本轮未覆盖、清理、提交或推送它们。

## 10. 本轮修改文件与性质

- `MulticopterRateControl.cpp`：把现有 LADRC 状态写入未占用 debug_array 槽位；诊断性修改。
- `ladrc_rate_control/LadrcRateControl.hpp/.cpp`：保存并暴露限幅前输出与限幅标志；诊断性修改。
- `validation/ladrc_as_gate_20260721/run_experiments.py`：任务专用的有界运行器、参数冻结、严格初态门禁、飞后安全基线恢复。
- `validation/ladrc_as_gate_20260721/analyze_results.py`：ULog/CSV 时序与指标分析、SHA-256 manifest、无 matplotlib 的 SVG 图生成。
- 本目录其余 CSV/JSON/Markdown/SVG/ULog：实验原始数据和报告，不参与飞控控制。

没有修改控制方程、轨迹、位置 PID、能量 AS 参数、模型几何或执行器配置。
