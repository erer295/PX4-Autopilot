# ZD680_HANG 控制律、输入输出与问题评估文档

生成日期：2026-07-06

## 1. 文档目的

本文档总结当前代码中三部分控制器的控制律、输入输出、核心公式和控制目标：

1. `LADRC` 多旋翼角速度内环。
2. `RBF-LADRC` 残差补偿模块。
3. `ZD680_HANG` 吊挂抗摆外环。

目标是评估三者之间是否存在控制目标冲突、信号耦合问题、公式方向问题或实验解释风险。

相关代码位置：

```text
mc_rate_control/ladrc_rate_control/LadrcRateControl.cpp
mc_rate_control/ladrc_rate_control/LadrcRateControl.hpp
mc_rate_control/rbf_residual_compensation/RbfResidualCompensation.cpp
mc_rate_control/rbf_residual_compensation/RbfResidualCompensation.hpp
mc_rate_control/MulticopterRateControl.cpp
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.cpp
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.hpp
mc_pos_control/PositionControl/PositionControl.cpp
```

## 2. 总体控制链路

当前系统的控制链路可以概括为：

```text
trajectory_setpoint
    |
    v
mc_pos_control, PositionControl
    |
    | 生成位置/速度控制加速度 a_pos
    |
    | 叠加吊挂抗摆外环 a_hang
    v
最终水平加速度设定值 a_cmd_xy = a_pos_xy + a_hang_xy
    |
    v
_accelerationControl()
    |
    v
vehicle_attitude_setpoint
    |
    v
mc_att_control
    |
    v
vehicle_rates_setpoint
    |
    v
mc_rate_control
    |
    | PID 或 LADRC 或 RBF-LADRC
    v
vehicle_torque_setpoint
    |
    v
control allocator / motors
```

三部分控制器的层级关系：

```text
吊挂抗摆外环：
    位置控制层，输出水平加速度修正 a_hang_xy

LADRC：
    角速度内环，输出归一化力矩 u_ladrc

RBF-LADRC：
    LADRC 后级残差补偿，输出额外归一化力矩 u_rbf
```

最终组合关系：

```text
a_cmd_xy = a_pos_xy + a_hang_xy

u_rate = u_ladrc                         , LADRC 模式
u_rate = u_ladrc + G_inj * u_rbf          , RBF-LADRC 注入模式
```

其中 `a_hang_xy` 会改变姿态设定值和角速度设定值，随后由 LADRC 或 RBF-LADRC 去执行。因此吊挂外环虽然不直接输出力矩，但它会间接改变内环负担。

## 3. LADRC 角速度内环

### 3.1 控制对象

当前 LADRC 是多旋翼机体系角速度控制器，对 roll、pitch、yaw 三个轴分别独立计算。

每个轴的简化对象为：

```text
omega_dot = f_total + b0 * u
```

符号含义：

| 符号 | 含义 |
|---|---|
| `omega` | 当前机体系角速度，roll/pitch/yaw rate |
| `omega_sp` | 角速度设定值 |
| `omega_dot` | 角加速度，来自 gyro derivative |
| `u` | 归一化力矩命令，PX4 torque setpoint |
| `b0` | 等效输入增益 |
| `f_total` | 总扰动，包括模型误差、惯量耦合、气动、吊挂耦合、执行器滞后等 |

这个模型的关键假设是：

```text
所有未知项都可以被等效成一个总扰动 f_total。
```

吊挂负载对机体角速度的影响也会被 LADRC 看成 `f_total` 的一部分。

### 3.2 LADRC 输入

代码中 `LadrcRateControl::update()` 的输入为：

```cpp
update(rate, rate_sp, angular_accel, dt, landed)
```

对应信号：

| 输入 | 来源 | 作用 |
|---|---|---|
| `rate` | `vehicle_angular_velocity.xyz` | 当前机体系角速度 |
| `rate_sp` | `vehicle_rates_setpoint` | 目标角速度 |
| `angular_accel` | `vehicle_angular_velocity.xyz_derivative` | 可选角加速度阻尼 |
| `dt` | 控制周期 | 离散更新 |
| `landed` | land detector | 落地时重置/冻结学习 |
| `u_observer` | 上一周期实际发布力矩 | LESO 预测输入 |
| saturation flags | allocator 状态 | 饱和时冻结扰动学习 |

主要参数：

| 参数 | 含义 |
|---|---|
| `MC_LADRC_EN` | 是否启用 LADRC |
| `MC_LADRC_B0_R/P/Y` | 每轴等效输入增益 `b0` |
| `MC_LADRC_WC_R/P/Y` | 控制器带宽 `wc` |
| `MC_LADRC_WO_R/P/Y` | 观测器带宽 `wo` |
| `MC_LADRC_LIM_R/P/Y` | 每轴归一化力矩限幅 |
| `MC_LADRC_D_R/P/Y` | 角加速度阻尼项 |

当前默认值：

| 参数 | 默认值 |
|---|---:|
| `B0_R/P/Y` | `50, 50, 20` |
| `WC_R/P/Y` | `8, 8, 4 rad/s` |
| `WO_R/P/Y` | `30, 30, 15 rad/s` |
| `LIM_R/P/Y` | `0.35, 0.35, 0.20` |
| `D_R/P/Y` | `0, 0, 0` |

吊挂实验中曾使用过更保守的 roll/pitch：

```text
B0_R/P = 120
WC_R/P = 8
WO_R/P = 18
D_R/P  = 0.003
```

### 3.3 LESO 观测器

LADRC 内部状态：

| 状态 | 含义 |
|---|---|
| `z1` | 角速度估计 |
| `z2` | 总扰动估计，单位近似为角加速度 |

观测误差：

```text
e_obs = omega - z1
```

LESO：

```text
z1_dot = z2 + b0 * u_last + beta1 * e_obs
z2_dot = beta2 * e_obs
```

带宽参数化：

```text
beta1 = 2 * wo
beta2 = wo^2
```

离散更新：

```text
z1(k+1) = z1(k) + dt * z1_dot
z2(k+1) = z2(k) + dt * z2_dot
```

保护逻辑：

```text
如果 landed 或 allocator 饱和：
    z2_dot = 0
```

原因是执行器无法实现力矩时，观测器不应该把执行器饱和误认为外部扰动。

`z2` 限幅：

```text
|z2| <= |b0| * torque_limit
```

### 3.4 LADRC 控制律

角速度跟踪误差使用估计状态：

```text
e_rate = omega_sp - z1
```

基础 LADRC 输出：

```text
u_ladrc = (wc * e_rate - z2) / b0
```

如果启用角加速度阻尼：

```text
u_ladrc = (wc * e_rate - z2) / b0 - D_accel * omega_dot
```

最终限幅：

```text
u_ladrc = sat(u_ladrc, -MC_LADRC_LIM, MC_LADRC_LIM)
```

扰动补偿项可写为：

```text
u_dist = -z2 / b0
```

代码中把 `u_dist` 复用记录到：

```text
rate_ctrl_status.rollspeed_integ
rate_ctrl_status.pitchspeed_integ
rate_ctrl_status.yawspeed_integ
```

注意：在 LADRC 模式下，这三个字段不是 PID 积分项，而是 LADRC 的扰动力矩补偿项。

### 3.5 LADRC 输出

输出为：

```text
u_ladrc = [u_roll, u_pitch, u_yaw]
```

类型为 PX4 归一化力矩命令：

```text
vehicle_torque_setpoint.xyz
```

在最终发布前，yaw 轴还会经过：

```text
MC_YAW_TQ_CUTOFF
```

低通滤波。

### 3.6 LADRC 控制目标

LADRC 的直接目标：

```text
让机体角速度 omega 快速跟踪 omega_sp。
估计并补偿总扰动 f_total。
```

对普通多旋翼，这是合理目标。但对吊挂系统，要注意：

```text
吊挂负载摆动不是单纯要被内环快速消除的扰动；
吊挂摆动是一个欠驱动柔性模态；
机体角速度跟踪越快，不一定吊挂越稳。
```

这就是 LADRC 在吊挂模型下可能不如 PID 的根本原因之一。

### 3.7 LADRC 可能存在的问题

#### 问题 1：LADRC 的控制目标不包含吊挂摆角

LADRC 只看到：

```text
rate
rate_sp
angular_accel
```

它不知道：

```text
hang_roll_joint
hang_pitch_joint
theta_dot_hang
```

所以 LADRC 会把吊挂引起的机体扰动当成普通 `f_total` 去补偿。这个补偿可能减小机体角速度误差，但也可能通过吊点加速度继续激励负载摆动。

#### 问题 2：高 `wo` 可能放大吊挂敏感频段

`wo` 越大，扰动估计越快，但对噪声和柔性模态越敏感。吊挂绳长 `0.6 m` 时固有频率约：

```text
f_n = 0.643 Hz
omega_n = 4.04 rad/s
```

如果 LADRC 在这个频段附近积极补偿，就可能把吊挂模态当成要立即消除的扰动，导致相位上补能。

#### 问题 3：角加速度 D 阻尼是机体阻尼，不是吊挂阻尼

`MC_LADRC_D_R/P` 的作用是：

```text
u -= D_accel * omega_dot
```

它类似 PX4 PID 的 D 项，可以降低机体角速度高频振荡。但它不等价于吊挂摆角阻尼，因为吊挂的状态是 `theta` 和 `theta_dot`，不是机体 `omega` 和 `omega_dot`。

#### 问题 4：LADRC 与抗摆外环可能抢控制权

抗摆外环改变的是水平加速度设定值，进而改变姿态和角速度设定值。LADRC 会努力快速执行这个变化。

如果抗摆外环本身相位不准，LADRC 反而会更快、更准确地执行一个可能补能的指令。

这会出现：

```text
位置误差下降；
机体控制更积极；
吊挂摆角上升。
```

这和最近 `114816` 的现象一致。

## 4. RBF-LADRC 残差补偿

### 4.1 RBF 模块定位

当前 RBF 不是独立飞控外环，也不是吊挂抗摆控制器。它是接在 LADRC 后面的角速度内环残差补偿模块。

路径：

```text
LADRC 输出 u_ladrc
    |
    v
RBF 计算残差 u_rbf
    |
    v
u_final = u_ladrc + G_inj * u_rbf
```

如果：

```text
MC_RBF_INJECT_EN = 0
```

则 RBF 只计算、学习、记录，不改变最终力矩。

如果：

```text
MC_RBF_INJECT_EN = 1
```

则 RBF 残差会注入最终力矩。

### 4.2 RBF 输入

RBF 的宿主是 `MulticopterRateControl.cpp`。每个控制周期中，先运行 LADRC，再准备 RBF 输入。

原始输入包括：

| 输入 | 含义 |
|---|---|
| `rate` | 当前角速度 |
| `rate_sp` | 角速度设定值 |
| `angular_accel` | 当前角加速度 |
| `ladrc_torque` | LADRC 输出力矩 |
| `ladrc_disturbance_compensation` | LADRC 估计扰动补偿 `-z2/b0` |
| `applied_torque` | 上一周期实际发布力矩 |

但实际 RBF 特征向量不是直接用这些原始量，而是构造每轴 5 维归一化特征。

### 4.3 RBF 特征向量

每个轴的输入维度固定为：

```text
kPerAxisInputDimension = 5
```

每个轴的特征为：

```text
x_axis = [
    1,
    e_rate_filtered / e_scale,
    u_ladrc / ladrc_limit,
    u_dist / ladrc_limit,
    residual_accel_filtered / accel_scale
]
```

其中：

```text
e_rate = rate_sp - rate
u_dist = -z2 / b0
```

代码中的归一化形式为：

```text
x1 = constrainUnitByScale(rate_error_filtered, error_scale)
x2 = constrainUnitByScale(torque_setpoint, ladrc_limit)
x3 = constrainUnitByScale(disturbance_compensation, ladrc_limit)
x4 = constrainUnitByScale(residual_accel_filtered, accel_scale)
```

特征中没有：

```text
hang_roll_joint
hang_pitch_joint
hang_roll_rate
hang_pitch_rate
rope_length
payload mass
```

所以当前 RBF 学习的是机体角速度内环残差，不是直接学习吊挂抗摆策略。

### 4.4 残差角加速度

代码先估计 LADRC 模型预测的角加速度：

```text
predicted_accel = b0 * (u_last - u_dist)
```

由于：

```text
u_dist = -z2 / b0
```

所以：

```text
predicted_accel = b0 * u_last + z2
```

这和 LADRC 观测器中的对象模型一致：

```text
omega_dot = z2 + b0 * u
```

残差角加速度：

```text
residual_accel = angular_accel - predicted_accel
```

再经过低通滤波：

```text
residual_accel_filtered = LPF(residual_accel, MC_RBF_RES_HZ)
```

### 4.5 RBF 基函数

对每个轴独立计算径向基激活值。

高斯基函数：

```text
phi_j = exp(-0.5 * ||x - c_j||^2 / sigma_j^2)
```

代码中距离计算忽略第 0 个 bias 特征，只使用：

```text
x[1], x[2], x[3], x[4]
```

也就是：

```text
filtered rate error
LADRC torque
LADRC disturbance compensation
filtered residual angular acceleration
```

如果启用激活归一化：

```text
phi_j = phi_j / sum(phi_j)
```

默认基函数布置：

```text
basis 0: center = 0
basis 1/2: +/- filtered rate error
basis 3/4: +/- residual angular acceleration
basis 5/6: +/- LADRC disturbance compensation torque
extra: +/- LADRC torque
```

相关参数：

| 参数 | 含义 | 默认 |
|---|---|---:|
| `MC_RBF_BASIS` | 基函数数量 | `7` |
| `MC_RBF_WIDTH` | 基函数宽度 | `0.3` |
| `MC_RBF_SPACING` | 中心间距 | `0.3` |
| `MC_RBF_FEAT_LIM` | 特征限幅 | `100` |

### 4.6 RBF 输出

每个轴原始输出：

```text
u_rbf_raw = sum_j w_j * phi_j
```

先限幅：

```text
u_rbf_sat = sat(u_rbf_raw, -MC_RBF_LIM, MC_RBF_LIM)
```

再低通：

```text
u_rbf_lpf(k) = alpha * u_rbf_lpf(k-1) + (1 - alpha) * u_rbf_sat(k)
```

其中：

```text
alpha = MC_RBF_LPF_ALPHA
```

然后做变化率限制：

```text
|u_rbf(k) - u_rbf(k-1)| <= MC_RBF_DU_MAX * dt
```

最终输出：

```text
u_rbf = sat(u_rbf_lpf_slew, -MC_RBF_LIM, MC_RBF_LIM)
```

默认残差力矩限幅：

| 参数 | 默认 |
|---|---:|
| `MC_RBF_LIM_R` | `0.015` |
| `MC_RBF_LIM_P` | `0.015` |
| `MC_RBF_LIM_Y` | `0.0` |

所以当前 RBF 输出本身非常保守，roll/pitch 只允许输出 `0.015` 的归一化残差。

### 4.7 RBF 学习目标

残差力矩学习目标：

```text
target = -residual_accel_filtered / b0 * MC_RBF_ERR_GAIN
```

如果启用额外 rate error 项：

```text
target += MC_RBF_ERR_WC * rate_error_filtered / b0
```

最终限幅：

```text
target = sat(target, -MC_RBF_LIM, MC_RBF_LIM)
```

再经过目标低通：

```text
target_filtered = LPF(target, MC_RBF_TGT_HZ)
```

当前默认：

```text
MC_RBF_ERR_GAIN = 0.05
MC_RBF_ERR_WC   = 0
MC_RBF_TGT_HZ   = 2.0 Hz
```

因此默认主要学习角加速度残差，并且只学习很保守的一小部分：

```text
target ≈ -0.05 * residual_accel_filtered / b0
```

### 4.8 RBF 权重更新律

RBF 使用类似 NLMS 的预测误差更新。

预测：

```text
prediction = sum_j w_j * phi_j
```

拟合误差：

```text
fit_error = target - prediction
```

激活范数：

```text
phi_norm_sq = epsilon + sum_j phi_j^2
```

权重微分：

```text
w_dot_j = learning_rate * fit_error * phi_j / phi_norm_sq - leakage * w_j
```

离散更新：

```text
w_j(k+1) = w_j(k) + dt * w_dot_j
```

权重限幅：

```text
w_j in [-MC_RBF_LIM, MC_RBF_LIM]
```

### 4.9 RBF 学习门控

RBF 的学习不是一直开启，必须满足大量条件。

全局门控：

```text
armed
not landed
not maybe_landed
roll/pitch attitude < 20 deg
allocator torque/thrust achieved
motor output between 0.05 and 0.95
dt valid
```

每轴门控：

```text
MC_RBF_E_MIN < |filtered rate error| < MC_RBF_E_MAX
|rate_sp_dot| < MC_RBF_SPD_MAX
|residual_accel_filtered| < MC_RBF_ACC_MAX
torque not saturated
LADRC torque < 0.85 * LADRC limit
injected torque < 0.85 * final limit
no sign-change freeze
no target-jump freeze
```

冻结原因包括：

| 冻结原因 | 含义 |
|---|---|
| sign change | rate error 变号 |
| target jump | 学习目标突变 |
| accel residual | 残差角加速度过大 |
| saturation | 力矩接近饱和 |
| setpoint jump | 角速度设定值变化太快 |
| gate | 全局门控不满足 |

冻结时，输出会按：

```text
u_rbf *= MC_RBF_FRZ_DEC
```

逐渐衰减。

### 4.10 RBF 最终注入

如果：

```text
MC_RBF_INJECT_EN = 1
```

则每轴最终力矩为：

```text
u_final = sat(u_ladrc + G_inj * u_rbf, -u_final_lim, u_final_lim)
```

其中：

```text
G_inj = MC_RBF_INJ_R/P/Y
u_final_lim = LADRC_limit * max(MC_RBF_FIN_GAIN, 1)
```

注意当前代码中使用：

```text
final_limit = ladrc_limit * fmaxf(MC_RBF_FIN_GAIN, 1)
```

所以 `MC_RBF_FIN_GAIN` 小于 `1` 时，实际不会把最终限幅降到 LADRC limit 以下。它只能保持或略微放大最终限幅。

### 4.11 RBF 控制目标

RBF 的直接目标：

```text
学习 LADRC 模型没有解释掉的角加速度残差，
输出一个额外的归一化残差力矩，
改善机体角速度跟踪。
```

它不是直接目标：

```text
降低吊挂摆角；
降低吊挂摆速；
降低负载相对位移。
```

### 4.12 RBF 可能存在的问题

#### 问题 1：RBF 输入没有吊挂状态

当前 RBF 特征没有：

```text
theta_hang
theta_dot_hang
rope_length
payload mass
anti_swing_acc
```

所以它无法知道当前残差力矩对吊挂摆动是耗能还是补能。

#### 问题 2：学习目标以机体角加速度残差为主

RBF 学习目标是：

```text
target ≈ -residual_accel / b0
```

这对机体内环是合理的，但对吊挂系统不一定合理。因为有些角加速度残差来自吊挂负载摆动，如果 RBF 快速抵消这些残差，可能让机体更稳，但负载摆动更大。

#### 问题 3：RBF 输出很小，可能难以体现效果

当前默认：

```text
MC_RBF_LIM_R/P = 0.015
MC_RBF_ERR_GAIN = 0.05
MC_RBF_LPF_ALPHA = 0.95
```

这意味着 RBF 很保守。它可能不足以明显改变吊挂实验结果，尤其在 `0.6-0.8 Hz` 的强摆动工况下。

#### 问题 4：RBF 学习经常被冻结

吊挂实验中扰动会导致：

```text
姿态变化
rate error 变号
target jump
residual accel 偏大
力矩接近限幅
```

这些都会触发冻结。结果是 RBF 可能大部分时间只在输出衰减，而不是有效学习。

#### 问题 5：RBF 权重 disarm 后 reset

当前 RBF 权重在重置时清零，没有跨实验保持学习结果。因此每次仿真基本都是重新学习，难以表现出“训练后控制器”的优势。

#### 问题 6：RBF 可能与抗摆外环目标冲突

抗摆外环想通过改变机体水平加速度来降低摆角。RBF 想通过补偿力矩来减小机体角速度残差。

如果抗摆外环为了耗散吊挂能量故意让机体做某些较慢或较柔和的动作，RBF 可能把这些动作产生的 rate error 或 residual accel 当成需要补偿的误差，从而削弱抗摆效果。

## 5. 吊挂抗摆外环

### 5.1 控制对象

吊挂负载小角度模型可近似为：

```text
theta_ddot + 2*zeta*omega_n*theta_dot + omega_n^2*theta
    = F_payload / (m*L) - a_base / L
```

其中：

| 符号 | 含义 |
|---|---|
| `theta` | 负载摆角 |
| `theta_dot` | 负载摆角速度 |
| `L` | 绳长 |
| `m` | 负载质量 |
| `a_base` | 无人机吊点水平加速度 |
| `F_payload` | 负载受到的水平外力 |
| `omega_n` | 吊挂固有角频率 |

固有频率：

```text
omega_n = sqrt(g / L)
f_n = 1 / (2*pi) * sqrt(g / L)
```

对当前 `L = 0.60 m`：

```text
omega_n = 4.04 rad/s
f_n = 0.643 Hz
```

这说明 `0.6 Hz` 和 `0.8 Hz` 扰动都在吊挂敏感区域附近。

### 5.2 抗摆外环位置

抗摆外环位于 PX4 位置控制的加速度层。

原始位置控制生成：

```text
a_pos_xy
```

抗摆外环生成：

```text
a_hang_xy
```

最终水平加速度：

```text
a_cmd_xy = a_pos_xy + a_hang_xy
```

代码位置：

```text
PositionControl::_velocityControl()
```

当前代码：

```cpp
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

const Vector2f anti_swing_acceleration =
	_suspended_load_anti_swing.update(dt, hrt_absolute_time(), _yaw, _suspended_load_anti_swing_flying);
_acc_sp.xy() += anti_swing_acceleration;

_accelerationControl();
```

### 5.3 抗摆外环输入

主要输入：

| 输入 | 来源 | 作用 |
|---|---|---|
| `hang_roll_joint angle` | Gazebo joint state -> debug_array | 吊挂 roll 摆角 |
| `hang_pitch_joint angle` | Gazebo joint state -> debug_array | 吊挂 pitch 摆角 |
| `hang_roll_joint rate` | Gazebo joint state -> debug_array | 吊挂 roll 摆速 |
| `hang_pitch_joint rate` | Gazebo joint state -> debug_array | 吊挂 pitch 摆速 |
| `yaw` | 位置控制器 | body 到 NED 旋转 |
| `dt` | 控制周期 | 滤波/斜率限制 |
| `flying` | takeoff state + OFFBOARD gate | 控制启停 |

桥接数据：

```text
debug_array id = 680
name = hangjoint
data[0] = roll_angle
data[1] = pitch_angle
data[2] = roll_rate
data[3] = pitch_rate
```

主要参数：

| 参数 | 含义 |
|---|---|
| `MC_HANG_AS_EN` | 抗摆开关 |
| `MC_HANG_OFFB` | 是否仅 OFFBOARD 启用 |
| `MC_HANG_LEN` | 绳长 |
| `MC_HANG_K_ANG` | 摆角反馈增益 |
| `MC_HANG_K_RATE` | 摆速阻尼增益 |
| `MC_HANG_ACC_LIM` | 抗摆加速度限幅 |
| `MC_HANG_ACC_SLW` | 抗摆加速度斜率限制 |
| `MC_HANG_LPF_HZ` | 摆角/摆速低通 |
| `MC_HANG_SIGN_X/Y` | 轴向符号 |
| `MC_HANG_MAX_ANG` | 小角度模型有效角 |
| `MC_HANG_TIMEOUT` | 测量超时 |
| `MC_HANG_ACT_DLY` | 起飞后激活延时 |
| `MC_HANG_ACT_ANG` | 激活前最大摆角 |
| `MC_HANG_ACT_R` | 激活前最大摆速 |
| `MC_HANG_ACT_T` | 激活稳定时间 |
| `MC_HANG_RAMP_T` | 激活渐入时间 |
| `MC_HANG_SAFE_A` | 安全退出角 |

### 5.4 关节到机体系摆角映射

代码中采用：

```text
theta_body_x = sign_x * hang_pitch_joint
theta_body_y = sign_y * hang_roll_joint
```

摆角速度：

```text
theta_dot_body_x = sign_x * hang_pitch_joint_velocity
theta_dot_body_y = sign_y * hang_roll_joint_velocity
```

当前实验建议值：

```text
MC_HANG_SIGN_X = -1
MC_HANG_SIGN_Y = -1
```

这个符号是经验验证得出的。它不是由公式自动保证的，后续如果模型坐标或 joint 定义变化，必须重新确认符号。

### 5.5 摆角与摆速滤波

一阶低通：

```text
alpha = exp(-2*pi*f_c*dt)
```

滤波：

```text
theta_f(k) = alpha * theta_f(k-1) + (1 - alpha) * theta(k)
theta_dot_f(k) = alpha * theta_dot_f(k-1) + (1 - alpha) * theta_dot(k)
```

其中：

```text
f_c = MC_HANG_LPF_HZ
```

默认：

```text
MC_HANG_LPF_HZ = 4 Hz
```

注意：滤波会引入相位滞后。在 `0.6-0.8 Hz` 吊挂敏感频段附近，即使 4 Hz 低通不算很低，也仍然会带来一定相位延迟。

### 5.6 激活逻辑

测量可用条件：

```text
enabled == true
flying == true
joint_state.valid == true
measurement fresh
angle/rate finite
```

激活前必须满足：

```text
elapsed_since_flying >= MC_HANG_ACT_DLY
|theta_f| <= MC_HANG_ACT_ANG
|theta_dot_f| <= MC_HANG_ACT_R
上述条件持续 MC_HANG_ACT_T
```

激活后渐入：

```text
ramp = clamp((t - engaged_since) / MC_HANG_RAMP_T, 0, 1)
```

当前常用安全参数：

```text
ACT_DLY = 8 s
ACT_ANG = 0.05 rad
ACT_R   = 0.08 rad/s
ACT_T   = 2 s
RAMP_T  = 5 s
```

### 5.7 抗摆控制律

小角度模型有效判断：

```text
if |theta_f| <= MC_HANG_MAX_ANG:
    theta_feedback = theta_f
else:
    theta_feedback = 0
```

当前抗摆加速度在机体系下为：

```text
a_hang_body = L * (K_ang * theta_feedback + K_rate * theta_dot_f)
```

其中：

```text
L      = MC_HANG_LEN
K_ang  = MC_HANG_K_ANG
K_rate = MC_HANG_K_RATE
```

当前实验主要使用：

```text
K_ang = 0
K_rate = 0.6
```

也就是：

```text
a_hang_body = L * K_rate * theta_dot_f
```

这本质上是吊挂摆速阻尼项。

机体系到 NED 水平面旋转：

```text
a_north = cos(yaw) * a_body_x - sin(yaw) * a_body_y
a_east  = sin(yaw) * a_body_x + cos(yaw) * a_body_y
```

限幅：

```text
|a_hang_body| <= MC_HANG_ACC_LIM
```

渐入：

```text
a_hang_body *= ramp
```

斜率限制：

```text
|a_hang_ned(k) - a_hang_ned(k-1)| <= MC_HANG_ACC_SLW * dt
```

最终再次限幅：

```text
|a_hang_ned| <= MC_HANG_ACC_LIM
```

### 5.8 安全退出逻辑

当前安全逻辑：

```text
if engaged and |theta_f| > MC_HANG_SAFE_A:
    resetActivation()
    output = 0
```

这意味着它是硬切换：

```text
正常输出 -> 瞬间退出 -> 等待再次满足激活条件
```

实验中已经看到：

```text
SAFE_A = 0.12 rad：
    扰动期间较保守，但容易过早退出，扰动后恢复差。

SAFE_A = 0.35 rad：
    不容易退出，扰动后位置恢复改善，但扰动期间摆角可能变大。
```

### 5.9 抗摆外环输出

输出为：

```text
a_hang_ned = [a_north, a_east]
```

单位：

```text
m/s^2
```

叠加位置：

```text
_acc_sp.xy() += a_hang_ned
```

最终通过 `_accelerationControl()` 转换成姿态和推力设定值。

### 5.10 抗摆外环控制目标

直接目标：

```text
通过调整无人机吊点水平加速度，降低吊挂负载摆角和摆速。
```

间接目标：

```text
在外部扰动下尽量同时保持飞机位置稳定。
```

理想情况下应满足：

```text
位置误差下降；
吊挂摆角 RMS 下降；
吊挂摆角 Peak 下降；
扰动后摆角衰减更快。
```

当前问题是实验中出现：

```text
位置误差下降；
吊挂摆角反而上升。
```

这说明当前抗摆外环可能在部分工况下更像“位置抗扰增强项”，还不是稳定可靠的“吊挂抗摆项”。

### 5.11 抗摆外环可能存在的问题

#### 问题 1：当前控制律只有简单 D 阻尼

当前主要公式是：

```text
a_hang = L * K_rate * theta_dot
```

它没有显式检查当前输出是否真的让吊挂能量下降。

吊挂能量近似：

```text
E = 0.5 * L^2 * |theta_dot|^2 + 0.5 * g * L * |theta|^2
```

理想抗摆应该尽量满足：

```text
dE/dt < 0
```

当前代码没有这个判断，所以在相位延迟、坐标符号、内环响应影响下，可能出现补能。

#### 问题 2：硬 SAFE_A 会造成两难

硬阈值的结果是：

```text
阈值小：过早退出，后期不帮忙。
阈值大：大摆角仍然输出，可能补能。
```

这正是 `SAFE_A = 0.12` 和 `SAFE_A = 0.35` 的实验差异。

#### 问题 3：缺少质量参数

当前抗摆律使用：

```text
L
theta
theta_dot
```

没有显式使用：

```text
payload mass m
```

如果控制输出是加速度修正，理论上质量可以不直接进入控制律。但当扰动直接作用在负载上时：

```text
theta_static = atan(F / (m*g))
```

不同质量会显著影响摆角响应。因此如果后续要做多载荷鲁棒性，不能只说控制器与质量无关，必须用实验验证。

#### 问题 4：滤波和执行链路带来相位延迟

链路中存在：

```text
Gazebo joint_state
debug_array 桥接
位置控制周期
低通滤波
加速度到姿态转换
姿态环
角速度内环
电机响应
```

这些都会引入相位延迟。在 `0.6-0.8 Hz` 附近，延迟可能让阻尼项从耗能变成补能。

#### 问题 5：没有内部状态日志

目前缺少：

```text
engaged
active
ramp
safety reset
a_hang_north
a_hang_east
theta_filtered
theta_dot_filtered
```

所以很多判断只能从摆角和位置结果反推。后续必须新增抗摆外环内部日志。

## 6. 三者之间的耦合关系

### 6.1 抗摆外环和 LADRC 的耦合

抗摆外环输出：

```text
a_hang_xy
```

它会改变姿态设定值，进而改变角速度设定值：

```text
a_hang_xy -> attitude_sp -> rate_sp
```

LADRC 的任务是：

```text
让 rate 快速跟踪 rate_sp
```

因此：

```text
如果 a_hang_xy 正确耗能，LADRC 越好，抗摆越有效；
如果 a_hang_xy 相位错误，LADRC 越好，补能越明显。
```

这解释了为什么 LADRC 不一定比 PID 更适合吊挂：PID 可能因为响应更钝，反而没有把错误相位的加速度指令执行得那么激进。

### 6.2 RBF 和 LADRC 的耦合

RBF 学习的是：

```text
LADRC 预测角加速度和实际角加速度的残差
```

它试图输出：

```text
u_rbf
```

使机体角速度模型误差变小。

但吊挂摆动会导致机体角加速度残差。RBF 可能学习到：

```text
如何让机体角速度更稳
```

而不是：

```text
如何让吊挂摆角更小
```

这两个目标在吊挂系统中可能冲突。

### 6.3 RBF 和抗摆外环的耦合

抗摆外环可能故意让机体产生某种水平加速度来消耗吊挂能量。这个动作可能引起短时 rate error 或 residual accel。

RBF 如果把这些误差当成需要补偿的残差，就可能削弱抗摆动作。

因此后续如果要做真正的 `RBF-LADRC + 抗摆`，RBF 应该知道吊挂状态，或者至少在吊挂大摆动时冻结学习。

### 6.4 三者控制目标对比

| 模块 | 输入状态 | 输出 | 直接目标 | 是否直接关心吊挂摆角 |
|---|---|---|---|---|
| LADRC | 机体角速度、角速度设定值、角加速度 | 归一化力矩 | 角速度跟踪、扰动补偿 | 否 |
| RBF | rate error、LADRC 输出、扰动补偿、残差角加速度 | 残差力矩 | 减小机体角加速度残差 | 否 |
| 抗摆外环 | 吊挂摆角、摆速、yaw | 水平加速度修正 | 减小吊挂摆动 | 是 |

目前真正直接使用吊挂状态的只有抗摆外环。

## 7. 当前最值得怀疑的问题点

### 7.1 核心问题判断

现在最值得怀疑的不是代码架构，而是控制目标不统一：

```text
LADRC/RBF 追求机体角速度误差小；
抗摆外环追求吊挂摆角小；
位置外环追求位置误差小。
```

在刚性无人机中这些目标通常一致。但在吊挂无人机中，它们可能互相冲突。

### 7.2 具体风险排序

#### 风险 1：抗摆外环相位不准

表现：

```text
位置误差变小；
吊挂摆角变大。
```

对应最近实验：

```text
114816
```

可能原因：

```text
简单 theta_dot 阻尼没有能量判断；
滤波和执行链路导致相位滞后；
SAFE_A = 0.35 允许大摆角时继续输出。
```

#### 风险 2：LADRC 执行抗摆指令过于积极

如果抗摆外环输出方向稍有问题，LADRC 会快速把这个输出落实到机体运动上。

这会让：

```text
PID 看起来更稳；
LADRC 看起来更激进；
RBF-LADRC 未必改善。
```

#### 风险 3：RBF 学习目标与抗摆目标不一致

RBF 没有吊挂状态，学习目标是机体角加速度残差。它可能学习到对机体有利、对吊挂不利的补偿。

#### 风险 4：硬安全阈值导致控制不连续

`SAFE_A` 突然 reset 会导致：

```text
抗摆输出瞬间变 0；
之后等待重新满足激活条件；
扰动后可能没有足够阻尼。
```

#### 风险 5：实验扰动目标容易混淆

最近若命令使用：

```text
--link-name base_link
```

且没有显式 `--entity`，则扰动可能实际加在：

```text
zd680_hang_0::base_link
```

这代表机体受扰，不代表负载直接受扰。

论文和评估中必须区分：

```text
base_link 扰动
hang_payload_link 扰动
```

## 8. 建议修改方向

### 8.1 先新增抗摆内部日志

建议新增：

```text
debug_array id = 681
name = hangas
```

记录：

```text
data[0] theta_body_x_filtered
data[1] theta_body_y_filtered
data[2] theta_dot_body_x_filtered
data[3] theta_dot_body_y_filtered
data[4] a_hang_north
data[5] a_hang_east
data[6] active
data[7] engaged
data[8] ramp
data[9] safety_state 或 safety_scale
```

没有这些日志，很难判断抗摆外环到底是退出太早、参与太久，还是方向错误。

### 8.2 把 SAFE_A 硬退出改成平滑缩放

建议由：

```text
if |theta| > SAFE_A:
    reset
```

改成：

```text
if |theta| <= SAFE_LO:
    scale = 1
elif SAFE_LO < |theta| < SAFE_HI:
    scale = smooth decreasing from 1 to 0
else:
    scale = 0
```

建议初值：

```text
SAFE_LO = 0.18 rad
SAFE_HI = 0.30 rad
```

最终：

```text
a_hang = scale * ramp * a_hang_raw
```

这样比 `0.12` 和 `0.35` 的单一硬阈值更合理。

### 8.3 给抗摆输出加入能量判断

建议把吊挂能量写入控制设计：

```text
E = 0.5 * L^2 * |theta_dot|^2 + 0.5 * g * L * |theta|^2
```

控制目标从：

```text
a_hang = L * K_rate * theta_dot
```

升级为：

```text
只允许或优先允许让 dE/dt < 0 的 a_hang
```

工程上可以先做简化判据：

```text
如果当前 a_hang 与吊挂状态组合显示可能补能：
    降低 a_hang 输出比例
```

### 8.4 给 LADRC/RBF 加吊挂大摆动保护

当：

```text
|theta| > 某阈值
或 |theta_dot| > 某阈值
```

建议：

```text
RBF 暂停学习；
RBF 输出逐渐衰减；
LADRC 不继续提高带宽；
抗摆外环进入平滑保护区。
```

理由是大摆角阶段的数据不适合作为 RBF 机体残差学习样本，容易把吊挂模态学成错误补偿。

### 8.5 RBF 后续应加入吊挂状态

如果论文要强调 `RBF-LADRC` 对吊挂负载有效，建议把 RBF 从“内环残差补偿”升级为“抗摆残差补偿”。

可选输入：

```text
theta_x
theta_y
theta_dot_x
theta_dot_y
a_hang_x
a_hang_y
rate_error
u_ladrc
u_dist
```

可选输出：

```text
方案 A：输出残差力矩 u_rbf，继续作用在内环
方案 B：输出抗摆残差加速度 a_rbf_hang，作用在外环
```

从吊挂控制角度，方案 B 更直观：

```text
a_cmd_xy = a_pos_xy + a_passive_hang_xy + a_rbf_hang_xy
```

这样 RBF 的目标可以直接定义为减小吊挂摆角，而不是只减小机体角速度残差。

## 9. 推荐评估清单

后续每次分析这三套控制器，建议按下面清单判断。

### 9.1 LADRC 检查项

```text
rate error RMS 是否下降？
u_dist = -z2/b0 是否过大？
z2 是否频繁顶到 |b0|*limit？
roll/pitch torque 是否接近 limit？
angular_accel damping 是否过大导致高频动作？
WO 是否太高导致对吊挂频段过敏？
```

### 9.2 RBF 检查项

```text
MC_RBF_INJECT_EN 是否开启？
rbf_output_filtered 是否接近 0？
rbf_target 是否频繁跳变？
rbf_freeze_flag 是否长期为 1？
rbf_weight_norm 是否增长？
rbf_saturation_flag 是否触发？
学习发生时吊挂摆角是否已经很大？
```

### 9.3 抗摆外环检查项

```text
engaged 何时变 1？
active 何时变 1？
是否触发 SAFE_A？
a_hang_north/east 与 theta_dot 的相位关系是否耗能？
扰动前是否自激？
扰动期间 swing RMS 是否低于无抗摆？
扰动后 last 5 s swing RMS 是否低于无抗摆？
```

### 9.4 总体指标

不能只看位置误差。必须同时看：

```text
XY RMS
XY Peak
swing RMS
swing Peak
swing rate RMS
last 5 s swing RMS
settling time
rate error RMS
motor saturation
torque saturation
```

如果出现：

```text
XY RMS 下降，但 swing RMS 上升
```

则不能认为抗摆成功。

## 10. 当前阶段结论

当前三部分控制器的关系可以总结为：

```text
LADRC：
    优化机体角速度跟踪，不直接知道吊挂。

RBF：
    学习 LADRC 剩余角加速度残差，不直接知道吊挂。

抗摆外环：
    唯一直接使用吊挂摆角的模块，但当前控制律只是简单阻尼项。
```

因此，当前最大问题不是“代码接错了”，而是：

```text
内环控制目标和吊挂抗摆目标还没有统一。
```

下一阶段最合理的方向是：

```text
1. 先给抗摆外环加内部日志；
2. 把 SAFE_A 硬退出改成平滑缩放；
3. 给抗摆输出加能量耗散判断；
4. 大摆角时冻结或衰减 RBF；
5. 如果继续做 RBF-LADRC 论文贡献，让 RBF 显式使用吊挂状态。
```

只有当实验结果同时满足：

```text
位置误差下降；
吊挂摆角 RMS 下降；
吊挂摆角 Peak 下降；
扰动后衰减更快；
控制输出没有明显饱和；
多频率和多绳长下均有效；
```

才能说明 LADRC、RBF 和抗摆外环形成了真正一致的吊挂负载控制方案。
