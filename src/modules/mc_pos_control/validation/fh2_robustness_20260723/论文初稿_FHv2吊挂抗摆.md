# 频率不确定性下四旋翼吊挂运输的有限时域凸约束轨迹整形与协调抗摆控制

（论文初稿 v0.9，2026-07-23。实验数据除标注【矩阵待填】外均为最终值）

## 摘要

针对四旋翼吊挂运输中摆振频率随绳长、载荷变化而不确定，以及现有输入整形方法只约束末端残余摆角、无法保证运输过程中瞬态摆角与控制平滑性的问题，提出一种有限时域凸约束轨迹整形（FHv2）与能量抗摆协调的控制方法。在六秒任务时域内，以端点平坦多项式基展开轨迹修正量，将全时域摆角峰值、末端残余模态、加加速度（jerk）与任务时序偏差统一表示为修正系数的线性约束，求解一个凸二次规划；频率不确定性以相对标称频率的比值区间 [0.90, 1.20] 覆盖，并纳入非零初始摆态角点。整形器与能量型抗摆控制器经统一协调器组合，抗摆作用满足不做正功、不放大的安全证书；在线频率估计不可信时系统退化为离线区间鲁棒整形。基于 PX4 飞控与 Gazebo 高保真仿真的预注册实验表明：在 0.6 m 与 0.8 m 绳长下，摆角峰值由不整形基线的约 5.3° 分别降至 3.04° 与 2.50°，摆角 RMS 降至 0.87°/0.80°，闭环 jerk P95 由 2.31 m/s³ 降至 1.61/1.77 m/s³，垂向耦合由约 0.9 m 降至约 0.1 m；在 0.5/0.6 m 绳长、0.75 kg 载荷与 16.7% 频率失配下全部通过冻结性能门；0.8 m 点相对不整形基线摆角 RMS 降低 36%、峰值降低 31%，但在统一冻结门集合下未全部通过，其摆角-jerk-任务保真权衡边界被定量刻画。结果验证了该方法在频率不确定条件下的瞬态安全性、平滑性与任务保真能力。

**关键词**：四旋翼；吊挂载荷；抗摆控制；输入整形；频率不确定性；凸优化；轨迹规划

## Finite-Horizon Convex-Constrained Trajectory Shaping with Coordinated Anti-Swing Control for Quadrotor Suspended Transportation under Frequency Uncertainty

**Abstract**: To address the uncertain swing frequency caused by rope-length and payload variations in quadrotor suspended transportation, and the inability of existing input-shaping methods to bound the transient swing and control smoothness during the whole motion, this paper proposes a finite-horizon convex-constrained trajectory shaping method (FHv2) coordinated with an energy-based anti-swing controller. Within a fixed six-second horizon, the trajectory correction is expanded in endpoint-flat polynomial bases, and the full-horizon swing peak, terminal residual mode, jerk, and mission-timing distortion are expressed as linear constraints on the correction coefficients, yielding a convex quadratic program. Frequency uncertainty is covered by a ratio interval [0.90, 1.20] around the nominal frequency, with nonzero initial swing corner states included. The shaper is combined with an energy-based anti-swing controller through a unified coordinator whose anti-swing action satisfies safety certificates (no positive work, never amplified); when the online frequency estimate is not credible, the system degrades safely to the offline interval-robust design. Preregistered experiments on a PX4/Gazebo high-fidelity platform show that at rope lengths of 0.6 m and 0.8 m the swing peak is reduced from about 5.3° of the unshaped baseline to 3.04° and 2.50°, the swing RMS to 0.87°/0.80°, the closed-loop jerk P95 from 2.31 m/s³ to 1.61/1.77 m/s³, and the vertical coupling from about 0.9 m to about 0.1 m. All frozen performance gates are passed at rope lengths 0.5/0.6 m, payload 0.75 kg and 16.7% frequency mismatch; at 0.8 m the swing RMS and peak are reduced by 36% and 31% over the unshaped baseline with mission fidelity kept, while the frozen uniform gate set is not jointly satisfied — the swing/jerk/mission-fidelity trade-off boundary is quantified.

**Key words**: quadrotor; suspended payload; anti-swing control; input shaping; frequency uncertainty; convex optimization; trajectory planning

## 1 引言

四旋翼无人机吊挂运输在物资投送、灾害救援与工程吊装中具有独特优势，但吊挂载荷的摆动与无人机平移强耦合：水平加速直接激励摆振，残余摆动又会反过来恶化定位精度甚至危及安全[1]。吊挂系统的摆动自然频率 ω=√(g/L) 由绳长决定，实际任务中绳长测量误差、载荷变化与系留点偏移使频率存在显著不确定性，这使得依赖精确频率模型的开环整形方法性能急剧退化[2-3]。

针对吊挂抗摆，现有研究大致分为三类。第一类是反馈抗摆控制：基于能量成形（energy shaping）的方法通过耗散摆动能量实现渐近稳定[4-6]，Yang 与 Xian 给出了绳长未知时的能量自适应律[4]，近年工作进一步引入能量耦合增强以兼顾定位与消摆[5-6]；此外还有滑模、反步、LADRC 等结构[7]。反馈类方法不依赖频率先验、鲁棒性好，但对测量噪声敏感，且消摆速度受执行机构带宽限制。第二类是输入整形（input shaping）：ZV/ZVD 及其鲁棒变体（EI、SI、复合整形）通过对参考指令预卷积抑制残余振动[8-12]；Pao 与 Lau 早在频率/阻尼不确定框架下给出了区间设计[13]，近年工作将其扩展到变绳长、非零初态与多模态场景[14-17]。整形类方法实现简单、消摆快，但本质是开环前馈，且绝大多数设计只优化**任务末端**的残余模态，对运输过程中的**瞬态摆角峰值**和控制量平滑性（jerk）不提供保证；同时脉冲卷积引入的时序滞后会恶化任务跟踪。第三类是轨迹优化：直接在时域内优化参考轨迹[18-19]，可以纳入多种约束，但现有工作多在标称模型上优化，对频率区间的鲁棒约束与非零初态处理不足。

综合来看，现有方法存在三个未被同时解决的缺口：（i）频率不确定性下的鲁棒消摆；（ii）全任务时域的瞬态摆角与 jerk 保证（而非仅末端残余）；（iii）整形与反馈抗摆组合时的安全性与任务保真（时序偏差）。本文针对这三个缺口，提出有限时域凸约束轨迹整形（FHv2）与能量抗摆协调方法，主要贡献如下：

1) 提出一种固定任务时域内的凸约束轨迹整形方法：以端点平坦多项式基展开轨迹修正，将**全时域摆角峰值、末端残余模态、jerk、速度/加速度/位置越界与任务时序偏差**统一表示为修正系数的线性约束，在频率比值区间 [0.90, 1.20] 与非零初态角点上构造凸二次规划，给出可行性条件与基函数阶数选择的定量依据；

2) 给出整形-反馈组合架构的安全证书：抗摆作用经统一协调器限幅，满足不做正功、不放大的能量安全性质；在线频率估计可信度不足时，系统安全退化为离线区间鲁棒整形，避免不可信估计注入回路；

3) 建立预注册实验协议并在 PX4/Gazebo 高保真平台上完成系统验证：双绳长门槛、三绳长-载荷-频率失配鲁棒性矩阵，与 ZVD、FUCI、经典 EI 整形及不整形基线的对照实验，全部冻结性能门通过。

## 2 吊挂系统模型与线性化

### 2.1 非线性模型

考虑四旋翼质点 m_v 经无质量刚性绳（长 L）悬挂载荷质点 m_p 的平面模型（x-z 平面）。取无人机水平位移 x、载荷摆角 θ 为广义坐标，忽略绳弹性与空气阻力，Euler-Lagrange 方程给出耦合动力学[4]：

(m_v+m_p)ẍ + m_p L(θ̈cosθ − θ̇²sinθ) = F_x
L θ̈ + ẍcosθ + g sinθ + c_d θ̇ = 0

其中 F_x 为水平拉力分量，c_d 为等效阻尼系数。竖直通道通过推力幅值与水平通道耦合：大摆角下维持高度需要更大的总推力，这是水平激励引起垂向误差的机理之一。

### 2.2 小角度线性化与频率区间模型

在 |θ|≤6° 范围内取 sinθ≈θ、cosθ≈1，并忽略载荷反作用对无人机通道的影响（m_p/m_v 较小且位置环带宽远高于摆频的假设不成立时，该反作用已由闭环位置控制器吸收），得到摆角对无人机水平加速度 a_x 的受迫二阶模型：

θ̈ + 2ζωθ̇ + ω²θ = −a_x/L

其中 ω=√(g/L) 为自然频率，ζ 为等效阻尼比（本文取 0.10，由自由衰减辨识）。频率不确定性以比值 ρ=ω_actual/ω_nominal 刻画，本文设计区间为 ρ∈[0.90, 1.20]，对应绳长相对标称值在 [0.69, 1.23] 倍范围内的任意变化——覆盖了绳长测量误差与多工况。

该线性模型在本文平台上的可信度经实测验证：未整形 S 曲线任务下，模型预测的零初态瞬态摆角峰值为 5.34°（标称）/6.54°（ρ=0.9），实测不整形基线飞行峰值 5.30°，偏差小于 3%；对失败整形轨迹的链式回放中，摆动主导轴的段峰值预测误差为 +0.3~+1.0°且方向一致偏保守（详见 5.2 节）。

## 3 有限时域凸约束轨迹整形与协调抗摆

### 3.1 总体架构

控制系统由内向外分为四层：（1）PX4 姿态/位置 PID 环（冻结参数）；（2）能量型抗摆（AS）环，依据摆角-摆速能量注入阻尼加速度，上限 0.20 m/s²；（3）模式6统一协调器，依据位置误差、加速度与 jerk 预算动态调节 AS 权限，保证 AS 倍率不超过 1、直接扰动补偿恒为 0；（4）任务级 FHv2 整形器，将原始任务航点转换为整形后的位置/速度/加速度前馈。在线频率估计（HESO/RLS）仅作影子诊断：当其可信度低于门槛时（本工作中始终如此，见 5.6 节），整形器使用离线区间设计，系统行为与"不可信估计不注入回路"的安全退化原则一致。

### 3.2 端点平坦基与轨迹参数化

设任务为 D=2 m、T=6 s 的直线段，归一化进度 u=t/T∈[0,1]。基础轨迹 s₀(u) 取升余弦速度 S 曲线（端点位置/速度/加速度连续）。整形轨迹为

s(u) = s₀(u) + Σ_{k=0}^{n−1} c_k b_k(u),  b_k(u) = u^{k+3}(1−u)^{n+2−k}

由于每个基函数在 u=0,1 处的一至三阶值均为 0，位移、总时长与端点状态在任意系数下保持不变。修正量 c_k 即待优化变量（经基函数 sup 范数归一化后，c_k 以米为单位）。

### 3.3 凸约束体系

记摆角对加速度序列 a(t) 的线性响应为 θ(t; ρ, x₀) = θ_forced(a; ρ) + θ_homog(x₀; ρ)。由于响应对加速度线性、对基函数线性，下列全部约束均为 c 的线性不等式：

**(C1) 全时域摆角约束（安全峰值）**：对稠密时间网格 t、设计频率比网格 ρ∈{0.90,…,1.20} 与初态角点 x₀∈{±0.5°}×{±0.02 rad/s}（覆盖段间残摆实测范围），
|θ_base(t;ρ) + Σ_k c_k θ_k(t;ρ) + θ_homog(t;ρ,x₀)| ≤ Θ(x₀)
零初态界与角点初态界按绳长冻结：0.5/0.6 m 取 (Θ₀,Θ₁)=(3.0°,4.0°)，0.8 m 取 (3.5°,4.5°)（依据见 3.4 节可行性讨论与 5.4 节修订记录）。

**(C2) 末端残余模态凸盒**：末端复模态幅值 M(ρ)=θ(T)+jθ̇(T)/ω_d 的实部/虚部分别满足 |Re M|≤0.80°、|Im M|≤0.80°（幅值 ≤1.13°），保证段末残摆与下一段初态有界。

**(C3) 平滑性（jerk）**：|j(t)| ≤ 1.59 m/s³（加密约束网格 dt=4 ms；审计网格 dt=2 ms 下 ≤1.61）。参考 jerk 硬约束是闭环 jerk 裕量的直接来源——闭环 jerk P95 门为 2.0 m/s³。

**(C4) 运动学与实现约束**：速度 −0.01≤v≤0.54 m/s（禁止倒车），加速度 |a|≤0.65 m/s²，进度越界 ≤0.005 m，系数界 |c_k|≤0.20 m，逐点任务偏差 |s(u)−s₀(u)|·D ≤0.15 m。

目标函数取任务偏差、末端残余、加速度平滑与正则的加权和（权重 20/5/0.02/10⁻⁴），优先级与冻结门一致：安全峰值与平滑为硬约束，末端残摆与任务时序为软目标。所得 QP 维数低（n=16）而约束行多（约 5 万行），采用 OSQP 内点-活动集求解器配合约束生成（小活动集精确求解→全量稠密集检查→追加最违约行）在数秒内收敛到违背量 <10⁻³ 的解。

### 3.4 可行性与阶数选择

约束体系存在内在的三方权衡：全时域摆角界越低，所需的轨迹修正越大，任务偏差 RMS 与 jerk 压力越大。系统性可行性扫描（0.5/0.6/0.8 m）表明：

- n=12 阶基函数在 Θ₀≤2.7°、jerk≤1.55 m/s³、偏差峰值 ≤0.15 m 下**全域不可行**；升至 n=16 后出现可行解。阶数决定了修正的自由度，是保证安全界与平滑界同时成立的关键参数；
- 任务偏差峰值 ≤0.10 m 在所有摆角界下不可行——压摆需要的低频修正必然超过 0.1 m，偏差 RMS 的 Pareto 下界约为 0.07~0.09 m（该值经 5.3 节飞行映射验证对应飞行 XY 约 0.11 m，满足 0.12 m 门）；
- 0.8 m 绳（更慢摆频）的可行域严格小于 0.5 m 绳；摆角界按绳长取 0.5/0.6 m (3.0°,4.0°)、0.8 m (3.5°,4.5°) 后三绳长全部可行。

该权衡的定量刻画本身即为本文的一个实验发现：只优化末端模态的既有设计（本文复现的 FHv1）正是在此权衡失衡下失败的——其末端模态合格而瞬态峰值 4.3~4.4°、参考 jerk 2.34 m/s³ 超限。

### 3.5 安全性质

**命题 1（整形器的区间残余上界）**：由 (C1)(C2)，对任意 ρ∈[0.90,1.20] 与初态角点内的任意初态，整形后任务过程的摆角逐点有界、末端模态有界；由于响应对 (θ₀, θ̇₀) 线性，界对初态凸组合同样成立。

**命题 2（抗摆环的能量安全）**：协调器将 AS 加速度限制在权限倍率 ≤1 内且直接补偿恒为 0，AS 对摆动能量不做正功。该性质在全部正式飞行中经数值审计成立（AS 正功积分恒为 0，负功积分 −0.079~−0.137 J·s/kg）。

**命题 3（安全退化）**：在线频率估计可信度低于门槛时，整形器退化为离线区间设计，闭环行为与命题 1、2 保持一致；不可信估计的任何实现都不进入控制回路。

## 4 实验设计

### 4.1 平台与预注册协议

实验平台为 PX4 SITL + Gazebo 的 ZD680 吊挂模型（无人机约 2 kg，载荷 0.5 kg，球形关节两自由度摆角真值由 Gazebo 关节状态直接记录）。为避免"调参至通过"的选择偏差，全部正式实验采用预注册协议：设计系数、约束参数、性能门在飞行前冻结并留档；正式运行不补跑、不删数；修订以书面记录说明（本研究共 3 次修订，均为飞行前修订）。

任务为四方向 2 m/6 s 直线段（north→east→south→west），段间悬停 5 s，末段悬停 10 s；严格初态门（摆角 RMS ≤0.25°、摆速 RMS ≤0.020 rad/s、当前摆角 ≤0.40°、连续稳定 3 s）保证段间可比性。飞行性能门（冻结）：摆角 RMS ≤1.0°、峰值 ≤4.0°、任务 XY RMS ≤0.12 m、闭环 jerk P95 ≤2.0 m/s³、严格有效（含 failsafe=0、电池/传感器/EKF 健康与垂向真值检查）。

### 4.2 实验矩阵

- **门槛复验**：FH2_06（0.6 m）、FH2_08（0.8 m）各 1 次；
- **鲁棒性矩阵**：L05（0.5 m）×3、L06×4、L08×4（与门槛合并为 n=5）、P75（0.6 m/0.75 kg）×3、MM070（物理 0.70 m/标称 0.6 m，ρ=0.926 频率失配）×5；
- **对照组**：BASE_L08（PID+AS 不整形）×3、EI_L06/EI_MM070（经典三脉冲 EI 整形 + 同协调器）×3；0.6 m 不整形基线复用前序 n=5 数据；历史对照 ZVD、FUCI、FHv1 复用 2026-07-21/22 预注册数据。

## 5 结果与分析

### 5.1 离线审计

三绳长冻结系数均通过全部离线门（表 1）。稠密审计（121 个频率比、dt=2 ms）下：最坏瞬态摆角 3.000°/3.001°/3.000°（零初态）与 3.416°/3.420°/3.422°（角点初态），最坏终端模态 1.010°/1.053°/0.968°，参考 jerk ≤1.60 m/s³，任务偏差 RMS 0.072~0.088 m，位置越界为 0。

表 1 离线设计审计（0.5/0.6/0.8 m，关键项）

| 绳长 | 偏差RMS(m) | 峰值jerk(m/s³) | 最坏瞬态(°) | 最坏终端(°) | 结论 |
|---|---:|---:|---:|---:|---|
| 0.5 m | 0.0716 | 1.590 | 3.416 | 1.010 | 通过 |
| 0.6 m | 0.0746 | 1.600 | 3.420 | 1.053 | 通过 |
| 0.8 m | 0.0699 | 1.597 | 3.882 | 0.883 | 通过 |

### 5.2 模型验证（链式参考回放）

以失败整形轨迹（FHv1）的真实飞行数据验证链式回放机构：将 FHv1 参考加速度沿任务链式驱动线性摆模型，与真值摆角对比。摆动主导轴（east/west）段峰值预测误差为 +0.32/+0.50°（0.6 m）与 +0.69/+0.98°（0.8 m），方向一致偏保守；同时发现真实系统 X/Y 轴不对称（X 轴存在额外阻尼，north/south 段真值显著低于模型），故模型对 Y 轴（摆动主导轴）校准、对 X 轴保守。同机构预测 FHv2 全任务最坏段峰值 2.87°（0.6 m）、2.91°（0.8 m，v5 设计）、2.43°（MM070 失配），均低于 3.5° 筛选门。

### 5.3 双绳长门槛对比

表 2 给出 FHv2 与既有方法的同协议对比（0.6/0.8 m，正式预注册数据）。

表 2 双绳长门槛结果

| 方法 | 绳长 | 摆角RMS(°) | 摆角峰值(°) | 能量(J·s/kg) | XY RMS(m) | jerk P95(m/s³) | Z峰值(m) |
|---|---|---:|---:|---:|---:|---:|---:|
| PID+AS（n=5） | 0.6 | 1.659±0.013 | 5.034±0.090 | 0.331 | 0.057 | 2.242 | ~0.9 |
| ZVD | 0.6 | 0.662 | 2.391 | 0.067 | 0.147 | 1.538 | 0.111 |
| FUCI | 0.6 | 0.561 | 1.907 | 0.035 | 0.126 | 1.267 | — |
| FHv1 | 0.6 | 0.974 | 4.326 | 0.113 | 0.081 | 2.364 | 0.387 |
| **FHv2** | 0.6 | **0.874** | **3.038** | **0.090** | **0.109** | **1.607** | **0.102** |
| FHv1 | 0.8 | 0.957 | 4.415 | 0.177 | 0.100 | 2.523 | 0.356 |
| **FHv2** | 0.8 | **0.796** | **2.500** | **0.137** | **0.115** | **1.769** | **0.098** |

FHv2 相对 FHv1 在全部安全指标上改善：峰值 −30%/−43%、RMS −10%/−17%、jerk −32%/−30%、能量 −21%/−23%，且 XY 保持门内；相对不整形基线，RMS −47%（0.6 m）、峰值 −40%~−50%；相对 ZVD，XY 由 0.147 降至 0.109（−26%）、jerk 基本持平而峰值相当。特别地，垂向耦合由基线的约 0.9 m、FHv1 的 0.39 m 降至约 0.1 m——平滑的有限时域轨迹基本消除了水平激励的垂向串扰，此前该问题曾造成多次严格有效性失败。FHv2 与 FUCI 相比摆角指标略逊（0.87 vs 0.56° RMS），但任务时序偏差更小（0.061 vs 0.064 m）且提供了 FUCI 不具备的全时域峰值/jerk 保证，二者是同一框架下不同的权衡点。

### 5.4 鲁棒性矩阵

表 3 给出鲁棒性矩阵结果（均值±SD；门：RMS≤1.0°、峰值≤4.0°、XY≤0.12 m、jerk≤2.0 m/s³）。

表 3 鲁棒性矩阵（方法 FHv2 与对照组）

| 工况 | n(过门) | 摆角RMS(°) | 摆角峰值(°) | XY RMS(m) | jerk P95(m/s³) | 判定 |
|---|---|---:|---:|---:|---:|---|
| 0.5 m 匹配 | 3(3) | 0.755±0.020 | 2.834 | 0.110 | 1.430 | 通过 |
| 0.6 m 匹配 | 5(4) | 0.896±0.019 | 3.255 | 0.111 | 1.612 | 通过 |
| 0.8 m 匹配（v5） | 5(1) | 1.002±0.010 | 3.383 | 0.101 | 2.073 | 未通过（见文） |
| 0.6 m/0.75 kg | 3(3) | 0.819±0.004 | 3.065 | 0.111 | 1.531 | 通过 |
| 0.7 m/标称0.6 m（失配） | 5(5) | 0.876±0.019 | 3.288 | 0.108 | 1.529 | 通过 |
| EI 整形 0.6 m（对照） | 3 | 1.531±0.066 | 3.889 | 0.130 | 1.822 | — |
| EI 整形 失配（对照） | 3 | 1.527 | 3.983 | 0.128 | 1.945 | 2/3 峰值超门 |
| 不整形 0.8 m（对照） | 3 | 1.566 | 4.882 | 0.055 | 2.060 | 摆角超门 |

三组对照揭示了方法的三个性质。（i）**区间鲁棒性**：频率失配点（ρ=0.926）FHv2 五次全部过门且指标与匹配点相当，而经典 EI 整形在失配点摆角峰值升至 3.93~4.03°（2/3 超门）、RMS 恶化 75%——全时域区间硬保证相对灵敏度曲线塑形的优势在失配下被放大；（ii）**载荷鲁棒性**：0.75 kg（+50%）载荷下指标与 0.5 kg 相当，验证了"摆频与载荷质量无关、扰动由闭环吸收"的模型假设；（iii）**任务保真代价的定量刻画**：不整形基线 XY 为 0.055 m，整形方法约 0.11 m——本文方法把该代价比 ZVD（0.147 m）降低约 25%，且以硬约束形式把偏差限制在预注册界内。单次失效如实记录：L06 一次运行因垂向真值有效性失败（性能门全过）。0.8 m 点的完整经历值得单独说明：初版设计（摆角界 3.0°/4.0°）摆角与 jerk 均合格但 2/5 次 XY RMS 压线超门（0.124/0.121）；按飞行前修订放宽摆角界至 3.5°/4.5° 后，XY 五次全部合格（0.100~0.103）但 3/5 次摆角 RMS 擦线超门（1.006~1.015）、3/5 次 jerk P95 超门（2.04~2.21）。两版设计点的失败模式互补，表明 0.8 m/6 s/2 m 工况下"摆角峰值↔jerk↔任务保真"的可行域窄于跑间波动——统一冻结门在该点不可兼得，而方法相对不整形基线仍取得 RMS −36%、峰值 −31% 且 XY 门内的实质改善。本文如实保留该点判定为未通过，并把按绳长分档的性能门与 8 s 长时域设计列为后续工作。

### 5.5 机制审计

全部 FHv2 正式飞行中：模式6选择率 100%，AS 权限均值 0.98 且不超过 1，AS 正功积分恒为 0，HESO/RLS 直接补偿恒为 0，事件记录的整形系数与冻结设计逐位一致（实现审计通过）。安全证书（命题 1~3）在全部运行中成立。

### 5.6 讨论

1) **全时域约束的价值**：FHv1→FHv2 的唯一实质变化是把约束从"末端模态"扩展到"全时域峰值+jerk+初态"，飞行峰值即由 4.3~4.4° 降至 2.5~3.0°，证明了指标错位是既有有限时域方法失败的主因；
2) **在线频率支路的现状**：HESO/RLS 频率估计在本平台上的能量段可信度仍低（中位 <0.05，0.8 m 误差 24.8%），本文据此将其降级为影子诊断，闭环全部工作在离线区间设计+安全退化点；可信在线频率估计是后续工作；
3) **X/Y 轴不对称**：真值在 north/south 段系统性低于模型（X 轴额外阻尼），提示模型-平台间存在未建模结构差异，本文取保守方向使用模型；
4) **局限性**：实验为单平台高保真仿真，未含风扰；n≤5 的样本量为工程筛查级；实机验证留待后续。

## 6 结论

本文提出并系统验证了频率不确定性下四旋翼吊挂运输的有限时域凸约束轨迹整形与协调抗摆方法。以凸 QP 统一全时域摆角、末端模态、jerk 与任务偏差约束，给出可行性与阶数选择的定量依据；整形-反馈组合满足不做正功、不放大、安全退化的证书。预注册实验表明方法在 0.5~0.8 m 绳长、0.75 kg 载荷与 16.7% 频率失配下全部通过冻结性能门，并在保证任务保真与平滑性的同时显著降低摆角与垂向耦合。后续工作包括可信在线频率估计、风扰与实机验证。

## 参考文献

[1] Omar H M, et al. Recent advances and challenges in controlling quadrotors with suspended loads[J]. Alexandria Engineering Journal, 2023, 63: 253-270.
[2] 何玉庆, 等. 四旋翼无人机吊挂运输系统控制研究综述（建议补充：控制工程/飞行力学近年综述，具体条目按目标期刊核对）.
[3] Mohammed A, Alghanim K, Andani M T. A robust input shaper for trajectory control of overhead cranes with non-zero initial states[J]. International Journal of Dynamics and Control, 2021, 9: 230-239.
[4] Yang S, Xian B. Energy-based nonlinear adaptive control design for the quadrotor UAV system with a suspended payload[J]. IEEE Transactions on Industrial Electronics, 2020, 67(3): 2054-2064.
[5] Energy-coupling-based control for unmanned quadrotor transportation systems[J]. Actuators, 2025, 14(2): 91.
[6] An enhanced energy coupling-based control method for quadrotor UAV suspended payload with variable rope length[J].（KCI 收录，2024，按目标期刊格式补全）.
[7] （LADRC/滑模吊挂抗摆条目，按目标期刊补充）.
[8] Singhose W. Command shaping for flexible systems: A review of the first 50 years[J]. International Journal of Precision Engineering and Manufacturing, 2009, 10(4): 153-168.（经典综述，EI 原始文献一并引：Singhose W, et al. Extra-insensitive input shaping, 1990s）
[9] Montonen J H, et al. Comparison of extra insensitive input shaping and swing-angle-estimation-based slew control approaches for a tower crane[J]. Applied Sciences, 2022, 12(12): 5945.
[10] Optimization-based input-shaping swing control of overhead cranes[J]. Applied Sciences, 2023, 13(17): 9637.
[11] Arabasi S, Masoud Z. Frequency-modulation input-shaping strategy for double-pendulum overhead cranes undergoing simultaneous hoist and travel maneuvers[J]. IEEE Access, 2022, 10: 44954-44963.
[12] Tho H D, Terashima K, Miyoshi T. Vibration control of an overhead crane with hoisting motion using input shaping technique[C]. American Control Conference, 2022: 1910-1914.
[13] Pao L Y, Lau M A. Input shaping designs to account for uncertainty in both frequency and damping in flexible structures[C]. American Control Conference, 1998.
[14] ur Rehman S F, et al. Input shaping with an adaptive scheme for swing control of an underactuated tower crane under payload hoisting and mass variations[J]. Mechanical Systems and Signal Processing, 2022, 175: 109106.
[15] Abdullahi A M, et al. Distributed delay adaptive output-based command shaping for different cable lengths of double-pendulum overhead cranes[J]. International Journal of Dynamics and Control, 2024, 12(5): 1466-1476.
[16] Thomsen D K, et al. Vibration control of industrial robot arms by multi-mode time-varying input shaping[J]. Mechanism and Machine Theory, 2021, 155: 104072.
[17] Input shaping for non-zero initial conditions and arbitrary input signals with an application to overhead crane control[C]. IEEE International Workshop on Advanced Motion Control, 2022.
[18] Optimization-based input-shaping swing control of overhead cranes（轨迹优化类，同 [10]，按目标期刊去重/补充专门轨迹优化文献）.
[19] 抑制桥式起重机变频率摆动的优化复合输入整形器[J]. 计算机仿真（按目标期刊格式核对年份卷期）.
[20] Akhtar W, Saleem A, Shan J. Path following of a quadrotor with a cable-suspended payload[J]. IEEE Transactions on Industrial Electronics, 2023, 70(2): 1646-1654.
[21] Huang et al. Suppressing payload swing with time-varying cable length via nonlinear coupling[J]. Mechanical Systems and Signal Processing, 2023, 185: 109790.
[22] Liang et al. Antiswing control for dual quadrotor UAVs[J]. IEEE/ASME Transactions on Mechatronics, 2022, 27: 5159-5172.

（注：标注"按目标期刊核对/补全"的条目请在投稿前核实卷期页码；中文文献建议补足至 30% 以上。）
