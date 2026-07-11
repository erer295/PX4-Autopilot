# ZD680_HANG 二阶外环 LADRC、能量抗摆与频率约束现状总结

生成日期：2026-07-11  
状态：当前代码与截至 `20260711_090716` 仿真数据的研究基线  
范围：只记录已经实现的结构、控制公式、参数、试验事实、已确认问题和尚未证明的边界，不记录计划采用的解决方法。

## 1. 文档目的

当前研究对象是带两自由度吊挂负载的 ZD680 四旋翼。当前研究控制结构可概括为：

```text
二阶位置外环 LADRC（水平 X/Y）
+ PX4 原生位置/速度 PID（垂直 Z）
+ 绳长相关的 LADRC 带宽约束
+ 基于摆动能量的水平抗摆加速度
+ PX4 原生姿态环和角速度 PID 内环
+ Gazebo / PX4 SITL 实现与 ULog 记录
```

本文档用于形成一个不带后续方案倾向的“现状快照”，避免后续分析混用旧的一阶 LADRC、角速度 LADRC、RBF 或早期 LegacyPD 结果。

当前主研究路径不包含：

- 已删除/退役的一阶位置外环 LADRC；
- 角速度内环 LADRC；
- RBF 残差补偿；
- 全轴二阶 LADRC 作为当前主方案；
- 早期仅使用摆角/摆速 PD 的抗摆作为当前主算法。

这些模块仍可能作为历史代码或对照项存在，但当前完整方案使用：

```text
MC_PLADRC_EN = 3       二阶 LADRC-X/Y + 原生 PID-Z
MC_PLADRC_TD_EN = 0    当前试验关闭 TD
MC_LADRC_EN = 0        原生角速度 PID
MC_RBF_EN = 0          关闭 RBF
MC_HANG_MODE = 2       能量阻尼抗摆
```

## 2. 仿真对象与坐标约定

### 2.1 Gazebo 模型

当前模型：

```text
Tools/simulation/gz/models/zd680_hang/model.sdf
```

Gazebo 实例：

```text
zd680_hang_0
```

主要结构：

```text
base_link
  |
  +-- hang_roll_joint
        |
        +-- hang_gimbal_link
              |
              +-- hang_pitch_joint
                    |
                    +-- hang_rope_link
                          |
                          +-- hang_payload_joint
                                |
                                +-- hang_payload_link
```

当前模型参数：

| 项目 | 当前值 |
|---|---:|
| 负载质量 | `0.50 kg` |
| 绳长 | `0.60 m` |
| 负载外扰实体 | `zd680_hang_0::hang_payload_link` |
| 摆角关节 | `hang_roll_joint`、`hang_pitch_joint` |

### 2.2 PX4 与机体系坐标

PX4 位置控制采用 NED：

```text
X: North
Y: East
Z: Down
```

抗摆控制内部先使用机体系水平摆角，再根据航向角旋转到 NED。

当前已验证的关节到机体系映射为：

```text
theta_body_x = sign_x * theta_pitch_joint
theta_body_y = sign_y * theta_roll_joint

theta_dot_body_x = sign_x * theta_dot_pitch_joint
theta_dot_body_y = sign_y * theta_dot_roll_joint
```

当前参数：

```text
MC_HANG_SIGN_X = -1
MC_HANG_SIGN_Y = -1
```

机体系抗摆加速度到 NED 的旋转为：

```text
[a_N]   [ cos(psi)  -sin(psi)] [a_body_x]
[a_E] = [ sin(psi)   cos(psi)] [a_body_y]
```

反向、Y 轴、非零航向等先前试验没有显示系统性轴交换或符号反向。最近完整方案的能量衰减也与当前符号相符。

### 2.3 吊挂固有频率

小角度单摆固有角频率和频率为：

```text
omega_n = sqrt(g / L)
f_n     = omega_n / (2*pi)
```

当 `L=0.60 m` 时：

```text
omega_n ≈ 4.043 rad/s
f_n     ≈ 0.643 Hz
```

最近外扰恢复段的实测主振荡频率约为 `0.68~0.71 Hz`。该值是闭环机体—吊挂耦合系统的有限时间拟合频率，不等同于独立理想单摆的精确固有频率。

## 3. 完整系统结构

### 3.1 信号流

```text
OFFBOARD trajectory_setpoint
          |
          v
MulticopterPositionControl / PositionControl
          |
          +-- 水平 X/Y: 二阶位置 LADRC
          |
          +-- 垂直 Z: PX4 原生位置 P + 速度 PID
          |
          +-- 吊挂状态: theta, theta_dot
          |       |
          |       v
          |   能量抗摆加速度 a_AS,NED
          |
          v
水平加速度合成与总包络限制
          |
          v
最终加速度 a_cmd,NED
          |
          v
PX4 accelerationControl()
          |
          v
vehicle_attitude_setpoint / thrust setpoint
          |
          v
PX4 原生姿态控制 + 原生角速度 PID
          |
          v
control allocator / motors
          |
          v
Gazebo ZD680 + suspended payload
          |
          +-- vehicle position / velocity / attitude feedback
          |
          +-- joint_state -> hangjoint -> anti-swing feedback
```

### 3.2 当前控制层级

| 层级 | 当前控制器 | 输出 |
|---|---|---|
| 水平位置外环 | 二阶 LADRC | NED X/Y 加速度 |
| 垂直位置外环 | PX4 原生位置 P + 速度 PID | NED Z 加速度 |
| 吊挂抗摆外环 | 能量阻尼 | NED X/Y 加速度修正 |
| 姿态环 | PX4 原生姿态控制 | 角速度设定值 |
| 角速度环 | PX4 原生 PID | 力矩设定值 |
| 执行器分配 | PX4 control allocator | 电机输出 |

### 3.3 位置控制器模式

代码中的选择关系为：

| `MC_PLADRC_EN` | 实际模式 | 说明 |
|---:|---|---|
| `0` | PID | 原生位置 P + 速度 PID，X/Y/Z 均使用原生路径 |
| `1` | PID | 一阶位置 LADRC 已退役，该值安全回退 PID |
| `2` | LADRC2 | X/Y/Z 全轴二阶 LADRC，仍保留为可选对照模式 |
| `3` | LADRC2-XY + PID-Z | 当前主研究模式 |

原生 PID 基线的基本形式为：

```text
v_sp = Kp_pos * (x_sp - x) + v_feedforward

a_PID = Kp_vel * (v_sp - v)
      + I_vel
      - Kd_vel * v_dot
      + a_feedforward
```

随后同样经过水平加速度合成、姿态生成、推力限制和原生姿态/角速度内环。PID 对比组关闭抗摆时，合成器中的抗摆请求为零。

## 4. 吊挂状态获取模块

### 4.1 Gazebo 到 PX4 桥接

Gazebo 中：

```text
/world/default/model/zd680_hang_0/joint_state
```

通过：

```text
simulation/gz_bridge/GZSuspendedLoadJointBridge.cpp
simulation/gz_bridge/GZSuspendedLoadJointBridge.hpp
```

转换为 PX4 `debug_array`：

```text
id   = 680
name = hangjoint
```

字段：

| 索引 | 含义 | 单位 |
|---:|---|---|
| `0` | roll joint angle | rad |
| `1` | pitch joint angle | rad |
| `2` | roll joint rate | rad/s |
| `3` | pitch joint rate | rad/s |

### 4.2 有效性条件

抗摆模块只在以下条件同时成立时使用吊挂测量：

- `MC_HANG_AS_EN=1`；
- `MC_HANG_MODE` 不是 Off；
- 飞行状态有效；
- 若 `MC_HANG_OFFB=1`，当前处于 OFFBOARD；
- 关节状态有效且四个角度/角速度字段均有限；
- 测量时间戳未超过 `MC_HANG_TIMEOUT`。

任一条件不满足时，模块复位激活状态并将请求加速度收回。

## 5. 二阶位置外环 LADRC

### 5.1 控制对象

每个位置轴使用二阶扩张状态模型：

```text
x_dot  = v
v_dot  = f_total + b0 * u
```

其中：

| 符号 | 含义 |
|---|---|
| `x` | 位置 |
| `v` | 速度 |
| `u` | 加速度控制输入 |
| `b0` | 名义输入增益 |
| `f_total` | 未建模动力学、外扰、耦合及输入模型误差的总和 |

当前水平轴 `b0=1`，输入 `u` 的单位直接对应 `m/s^2`。

### 5.2 三阶 LESO

二阶对象使用三个观测状态：

```text
z1: 位置估计
z2: 速度估计
z3: 总扰动估计
```

观测误差：

```text
e_o = x - z1
```

连续形式：

```text
z1_dot = z2 + beta1 * e_o
z2_dot = z3 + b0 * u_obs + beta2 * e_o
z3_dot = beta3 * e_o
```

带宽参数化：

```text
beta1 = 3 * wo
beta2 = 3 * wo^2
beta3 = wo^3
```

源码采用显式离散积分：

```text
z_i(k+1) = z_i(k) + dt * z_i_dot(k)
```

保护限制：

```text
|z2| <= 100 m/s
|z3| <= |b0| * acceleration_limit
```

### 5.3 二阶 LADRC 控制律

控制带宽参数化：

```text
kp = wc^2
kd = 2 * wc
```

控制律：

```text
u_LADRC = [
    wc^2 * (x_sp - z1)
  + 2*wc * (v_sp - z2)
  - z3
] / b0
```

若启用加速度阻尼：

```text
u_LADRC = u_LADRC - D_acc * v_dot_measured
```

当前：

```text
MC_PLADRC_D_XY = 0
```

单独的扰动补偿分量为：

```text
u_dist = -z3 / b0
```

水平 LADRC 输出先经过水平加速度范数限制。

### 5.4 混合 X/Y-LADRC、Z-PID 模式

`MC_PLADRC_EN=3` 时：

- X/Y 使用上述二阶 LADRC；
- Z 轴保留 PX4 原生位置到速度、速度到加速度控制链；
- X/Y 速度积分项保持为零；
- Z 轴积分与 PX4 原生抗饱和逻辑保留。

Z 轴核心形式为：

```text
v_sp,z = Kp_pos,z * (z_sp - z) + velocity_feedforward

a_z = Kp_vel,z * (v_sp,z - v_z)
    + I_z
    - Kd_vel,z * v_dot_z
```

该结构用于避免二阶 LADRC 对高度大阶跃的早期过冲问题，并保留 PX4 已验证的高度控制行为。

### 5.5 跟踪微分器 TD

代码保留可选二阶 TD：

```text
v1_dot = v2
v2_dot = -2*zeta_td*w_td*v2 + w_td^2*(v_sp - v1)
```

并限制：

```text
|v2| <= a_td_limit
```

当前试验：

```text
MC_PLADRC_TD_EN = 0
```

因此最近数据不包含 TD 对设定值的整形作用。

### 5.6 无扰切换初始化

控制器进入二阶 LADRC 时：

```text
z1 = measured_position
z2 = measured_velocity
```

根据当前参考加速度和上一周期已施加加速度计算候选 `z3`。只有候选值位于物理扰动限幅范围内时才使用，否则将 `z3` 置零。该逻辑用于避免大位置阶跃在初始化时被错误写入扰动状态。

### 5.7 LESO 已知输入

LESO 的 `u_obs` 来自最终推力设定值反算的加速度：

```text
a_applied,xy = thrust_sp,xy * g / hover_thrust
```

它包含最终组合与推力饱和后的命令信息，但不是 Gazebo 真实机体加速度测量。`posladrc` 日志中称为 applied/observer input 的量均属于该重构输入。

## 6. 绳长相关的频率约束

### 6.1 当前公式

打开 `MC_HANG_FRQ_EN` 后，根据已知绳长计算：

```text
omega_n = sqrt(g / L)
```

水平有效带宽：

```text
wc_eff = min(
    MC_PLADRC_WC_XY,
    MC_HANG_WC_R * omega_n
)

wo_eff = max(
    MC_HANG_WO_MIN * wc_eff,
    min(
        MC_PLADRC_WO_XY,
        MC_HANG_WO_R * omega_n
    )
)
```

### 6.2 当前含义边界

当前实现是根据绳长改变 X/Y 的两个标量带宽。它没有对不同频率分量进行独立滤波，也没有在线估计绳长或实测摆动频率。

### 6.3 最近两种有效参数

基础参数：

```text
MC_PLADRC_WC_XY = 1.8 rad/s
MC_PLADRC_WO_XY = 6.0 rad/s
MC_HANG_WC_R    = 0.25
```

高观察器带宽组：

```text
MC_HANG_WO_R   = 0.80
MC_HANG_WO_MIN = 2.50

wc_eff ≈ 1.0107 rad/s
wo_eff ≈ 3.2343 rad/s
```

低观察器带宽组：

```text
MC_HANG_WO_R   = 0.50
MC_HANG_WO_MIN = 2.00

wc_eff ≈ 1.0107 rad/s
wo_eff ≈ 2.0214 rad/s
```

## 7. 能量抗摆模块

### 7.1 摆角与摆速滤波

摆角和摆速使用一阶低通：

```text
alpha = exp(-2*pi*f_c*dt)
y_f(k) = alpha*y_f(k-1) + (1-alpha)*y(k)
```

当前：

```text
f_c = MC_HANG_LPF_HZ = 4 Hz
```

### 7.2 单位质量摆动能量

代码使用二维精确余弦势能形式：

```text
E = 0.5 * L^2 * (theta_dot_x^2 + theta_dot_y^2)
  + g*L * [2 - cos(theta_x) - cos(theta_y)]
```

单位为：

```text
J/kg = m^2/s^2
```

小角度近似为：

```text
E ≈ 0.5*L^2*||theta_dot||^2
  + 0.5*g*L*||theta||^2
```

### 7.3 能量门控

门控系数：

```text
sigma_E = 0,                            E <= E_min

sigma_E = (E-E_min)/(E_full-E_min),     E_min < E < E_full

sigma_E = 1,                            E >= E_full
```

当前：

```text
E_min  = 0
E_full = 0.02 J/kg
```

### 7.4 阻尼增益

能量模式的等效阻尼增益：

```text
k_d = 2 * zeta * sqrt(g*L)
```

当前：

```text
zeta = 0.10
L    = 0.60 m
k_d  ≈ 0.485 m/s
```

### 7.5 能量抗摆控制律

机体系原始加速度：

```text
a_AS,body,raw = k_d * sigma_E * theta_dot_body
```

在当前关节符号约定下，机体水平加速度对摆动能量的控制贡献写为：

```text
E_dot_control = -L * a_body · theta_dot_body
```

代入原始能量抗摆控制律：

```text
E_dot_AS,raw
  = -L * k_d * sigma_E * ||theta_dot_body||^2
  <= 0
```

因此，未经过斜率限制和系统组合的原始抗摆项在理论上不增加摆动能量。

### 7.6 LegacyPD 模式

代码仍保留早期模式：

```text
a_AS,body = L * (K_angle*theta + K_rate*theta_dot)
```

当前主方案使用 `MC_HANG_MODE=2`，该公式只作为代码中仍存在的对照模式记录。

### 7.7 限幅、渐入与斜率限制

能量抗摆输出依次经过：

1. 二维范数限幅；
2. 激活渐入系数；
3. body 到 NED 旋转；
4. NED 加速度斜率限制；
5. 再次范数限幅。

公式表示：

```text
a_limited = sat_norm(a_AS,body,raw, a_AS,max)

ramp = clamp((t-t_engaged)/T_ramp, 0, 1)

a_target,NED = R_yaw * (ramp * a_limited)

Delta_a = sat_norm(
    a_target,NED - a_AS,NED(k-1),
    slew_rate * dt
)

a_AS,NED(k) = sat_norm(
    a_AS,NED(k-1) + Delta_a,
    a_AS,max
)
```

当前：

```text
a_AS,max = 0.20 m/s^2
slew     = 2.0 m/s^3
ramp     = 2.0 s
```

### 7.8 激活和安全状态

当前有效参数：

```text
MC_HANG_ACT_DLY = 2.0 s
MC_HANG_ACT_ANG = 0
MC_HANG_ACT_R   = 0
MC_HANG_ACT_T   = 0
MC_HANG_SAFE_A  = 0.35 rad
MC_HANG_REARM   = 1.0 s
```

当激活角/速阈值为零时，不以初始摆角和摆速阻止激活；仍需满足飞行状态、模式、测量有效和延时条件。

已 engaged 后，若滤波摆角范数超过安全角：

- 抗摆退出 engaged；
- 请求加速度按斜率限制收回零；
- 等待重新满足延时与激活条件；
- 状态日志标记 safety/rearming。

## 8. 基础控制与抗摆加速度合成

### 8.1 总水平加速度包络

有效总包络：

```text
a_total,max = min(MPC_ACC_HOR_MAX, MC_HANG_TOT_A)
```

当前：

```text
MC_HANG_TOT_A = 0.80 m/s^2
MC_PLADRC_LIM_XY = 0.80 m/s^2
```

LADRC 自身水平限幅还会取：

```text
min(MC_PLADRC_LIM_XY, a_total,max)
```

### 8.2 实际合成顺序

代码首先限制基础位置控制加速度：

```text
a_base = sat_norm(a_base_raw, a_total,max)
```

然后计算抗摆请求 `a_AS`。

当前实现为抗摆请求预留控制权：

```text
如果 ||a_AS|| >= a_total,max:
    a_AS,applied = sat_norm(a_AS, a_total,max)
    a_final = a_AS,applied

否则:
    a_final = constrainXY(a_AS, a_base, a_total,max)
```

`constrainXY(primary, secondary, max)` 保留 primary，并在剩余二维范数空间内加入 secondary。因此当前代码的 primary 是抗摆请求，基础位置控制使用剩余空间。

最终满足：

```text
||a_final,xy|| <= a_total,max
```

### 8.3 最终姿态与推力

最终 NED 加速度经过 PX4 `_accelerationControl()`：

```text
body_z_desired ∝ [-a_x, -a_y, g-a_z]
```

随后经过倾角、总推力和水平推力裕量限制，生成姿态设定值和推力设定值。

最终饱和推力再被反算为 LESO 下一周期已知输入。

### 8.4 PX4 原生姿态与角速度内环

当前研究没有修改姿态和角速度控制律。其功能可用下列框架表示：

```text
姿态误差 q_error
    -> PX4 四元数姿态控制
    -> body rate setpoint omega_sp

omega_error = omega_sp - omega

torque_sp = Kp_rate * omega_error
          + Ki_rate * integral(omega_error)
          - Kd_rate * omega_dot
          + rate feedforward
```

实际 PX4 姿态控制还包含四元数误差构造、yaw 权重、角速度限制；角速度 PID 包含积分限幅、饱和状态处理和执行器分配反馈。最近试验始终使用：

```text
MC_LADRC_EN = 0
MC_RBF_EN = 0
```

## 9. 当前参数基线

以下参数对应最近有效完整方案 `20260711_090716`。

### 9.1 控制器选择

| 参数 | 值 | 含义 |
|---|---:|---|
| `MC_PLADRC_EN` | `3` | LADRC2-X/Y + PID-Z |
| `MC_PLADRC_TD_EN` | `0` | 关闭位置 TD |
| `MC_LADRC_EN` | `0` | 原生角速度 PID |
| `MC_RBF_EN` | `0` | 关闭 RBF |

### 9.2 水平 LADRC

| 参数 | 值 |
|---|---:|
| `MC_PLADRC_B0_XY` | `1.0` |
| `MC_PLADRC_WC_XY` | `1.8 rad/s` |
| `MC_PLADRC_WO_XY` | `6.0 rad/s` |
| `MC_PLADRC_LIM_XY` | `0.8 m/s^2` |
| `MC_PLADRC_D_XY` | `0` |

经频率约束后：

| 量 | 值 |
|---|---:|
| `wc_eff` | `1.0107 rad/s` |
| `wo_eff` | `2.0214 rad/s` |

### 9.3 能量抗摆

| 参数 | 值 |
|---|---:|
| `MC_HANG_AS_EN` | `1` |
| `MC_HANG_MODE` | `2` |
| `MC_HANG_OFFB` | `1` |
| `MC_HANG_LEN` | `0.60 m` |
| `MC_HANG_ZETA` | `0.10` |
| `MC_HANG_E_MIN` | `0` |
| `MC_HANG_E_FULL` | `0.02 J/kg` |
| `MC_HANG_ACC_LIM` | `0.20 m/s^2` |
| `MC_HANG_ACC_SLW` | `2.0 m/s^3` |
| `MC_HANG_LPF_HZ` | `4.0 Hz` |
| `MC_HANG_SIGN_X/Y` | `-1 / -1` |
| `MC_HANG_SAFE_A` | `0.35 rad` |
| `MC_HANG_TOT_A` | `0.80 m/s^2` |

### 9.4 频率约束

| 参数 | 值 |
|---|---:|
| `MC_HANG_FRQ_EN` | `1` |
| `MC_HANG_WC_R` | `0.25` |
| `MC_HANG_WO_R` | `0.50` |
| `MC_HANG_WO_MIN` | `2.0` |

## 10. 日志与试验记录结构

### 10.1 PX4 内部日志

#### `hangjoint`, ID 680

原始关节状态，见第 4 节。

#### `hangas`, ID 681

| 索引 | 含义 |
|---:|---|
| `0,1` | 滤波 body X/Y 摆角 |
| `2,3` | 滤波 body X/Y 摆速 |
| `4,5` | 抗摆请求 N/E 加速度 |
| `6` | active |
| `7` | engaged |
| `8` | ramp scale |
| `9` | safety state：0正常、1重激活、2安全退出 |
| `10` | `omega_n` |
| `11` | 单位质量摆动能量 |
| `12` | 能量门控系数 |
| `13` | 能量阻尼增益 `k_d` |
| `14,15` | 抗摆原始 N/E 加速度 |
| `16,17` | 合成器记录的已应用抗摆 N/E 加速度 |
| `18` | 抗摆模式 |

#### `posladrc`, ID 683

| 索引 | 含义 |
|---:|---|
| `0:2` | TD 速度状态 |
| `3:5` | TD 速度导数 |
| `6:8` | LADRC 输出加速度 |
| `9:11` | 扰动补偿 `-z3/b0` |
| `12` | LADRC enabled |
| `13` | TD enabled |
| `14` | controller mode |
| `15,16` | LESO observer input X/Y |
| `17,18` | 基础控制器 raw X/Y |
| `19,20` | 抗摆合成后的 final X/Y |
| `21,22` | 推力反算 applied X/Y |
| `23,24` | 有效 `wc_xy / wo_xy` |

### 10.2 记录器输出

当前脚本：

```text
Tools/simulation/gz/tools/position_offboard_flight_recorder.py
```

每组通常生成：

```text
metadata.json
events.csv
summary.csv
live_samples.csv
hang_joint_samples.csv
position_offboard.ulg
px4_console.log
```

### 10.3 最近统一外扰场景

```text
profile: actions
actions: hold:20;hold:15
wrench: sine:20:6:0.5:west:0:none:0.0833333333
frame: local_ned
entity: zd680_hang_0::hang_payload_link
```

含义：

- OFFBOARD 进入后先悬停；
- 在动作开始 20 s 时对负载施加西向力；
- 力幅值 `0.5 N`；
- 频率 `0.08333 Hz`，完整周期 `12 s`；
- 实际只施加 `6 s`，因此是一段半正弦推力；
- 外扰结束后继续悬停约 `9.5 s`。

外力时域表达为：

```text
F_west(t) = 0.5 * sin(2*pi*0.083333*(t-t0)) N,
            0 <= t-t0 <= 6 s
```

因此该输入主要是一段平滑推开负载再释放的瞬态，不是持续多个周期的稳态正弦扫频。

### 10.4 指标定义

摆角范数：

```text
theta_norm = sqrt(theta_roll^2 + theta_pitch^2)
```

摆角 RMS：

```text
theta_RMS = sqrt(mean(theta_norm^2))
```

位置 XY 误差：

```text
e_xy = sqrt((x-x_sp)^2 + (y-y_sp)^2)
```

能量功率诊断：

```text
P_specific = -L * a_body · theta_dot_body
```

其中 `P<0` 表示该加速度分量在当前符号约定下消耗摆动能量，`P>0` 表示向摆动注入能量。外力直接作用于负载时，控制功率并不等于总能量变化率，因为外力本身也做功。

## 11. 最近飞行数据

### 11.1 数据组说明

| 记录 | 控制配置 | `wo_eff` | 抗摆 | 备注 |
|---|---|---:|---:|---|
| `225339` | 原生 PID | 不适用 | 关闭 | 同包络 PID 基线 |
| `225058` | LADRC2-XY + PID-Z | `3.234` | 开启 | 高观察器带宽完整方案 |
| `231033` | LADRC2-XY + PID-Z | `3.234` | 关闭 | 高观察器带宽消融 |
| `082745` | LADRC2-XY + PID-Z | `2.021` | 关闭 | 低观察器带宽消融 |
| `083254` | LADRC2-XY + PID-Z | `2.021` | 开启 | 仅默认 hold，无外扰，不属于同场景对比 |
| `090716` | LADRC2-XY + PID-Z | `2.021` | 开启 | 低观察器带宽完整方案 |

### 11.2 同场景主要指标

| 指标 | PID `225339` | 完整高 `225058` | 无抗摆高 `231033` | 无抗摆低 `082745` | 完整低 `090716` |
|---|---:|---:|---:|---:|---:|
| 外扰前摆角 RMS/deg | `0.186` | `1.431` | `4.945` | `3.521` | `0.696` |
| 外扰前平均能量 | `0.000059` | `0.004035` | `0.048163` | `0.023625` | `0.000937` |
| 外扰段摆角 RMS/deg | `5.514` | `8.415` | `8.746` | `8.109` | `8.555` |
| 外扰段摆角峰值/deg | `11.493` | `16.398` | `17.001` | `16.141` | `16.553` |
| 外扰段能量均值 | `0.02936` | `0.06831` | `0.10431` | `0.07264` | `0.06787` |
| 外扰结束能量 | `0.09271` | `0.17246` | `0.25496` | `0.17399` | `0.22504` |
| 外扰净能量增量* | `0.09264` | `0.16735` | `0.20269` | `0.15209` | `0.22416` |
| 外扰段 XY RMS/m | `0.0794` | `0.1194` | `0.0921` | `0.1462` | `0.1794` |
| 外扰段 XY 峰值/m | `0.1717` | `0.2349` | `0.1687` | `0.3412` | `0.3969` |
| 恢复段 XY RMS/m | `0.0944` | `0.1537` | `0.1291` | `0.2386` | `0.3048` |
| 恢复段 XY 峰值/m | `0.1472` | `0.2866` | `0.2773` | `0.3652` | `0.4599` |
| 恢复最后 3 s摆角 RMS/deg | `1.619` | `1.795` | `12.109` | `8.978` | `2.495` |
| 恢复最后 3 s能量 | `0.00488` | `0.00659` | `0.27842` | `0.15266` | `0.01112` |
| 最终能量 | `0.00219` | `0.00508` | `0.28021` | `0.14514` | `0.00439` |
| 外扰段加速度 P95/(m/s²) | `0.449` | `0.710` | `0.534` | `0.599` | `0.666` |
| 恢复段加速度 P95/(m/s²) | `0.352` | `0.756` | `0.703` | `0.678` | `0.800` |

`* 外扰净能量增量 = 外扰结束能量 - 外扰前最后样本能量`。

表中的摆角和能量来自 Gazebo 关节 CSV，并使用第 7.2 节的二维能量公式；位置和加速度指标来自对应 ULog。不同记录的窗口均按照各自 `events.csv/metadata.json` 中的实际 boot 时间对齐。

### 11.3 抗摆消融结果

在相同低观察器带宽下，`082745 -> 090716`：

```text
恢复最后 3 s 摆角 RMS: 8.978 deg -> 2.495 deg
恢复最后 3 s 能量:     0.15266 -> 0.01112
最终能量:              0.14514 -> 0.00439
```

与此同时：

```text
外扰段 XY RMS: 0.1462 m -> 0.1794 m
恢复段 XY RMS: 0.2386 m -> 0.3048 m
```

该数据表明抗摆开启后，恢复段负载能量显著降低，同时位置误差增大。

### 11.4 观察器带宽消融结果

在均关闭抗摆时，`231033 -> 082745`：

```text
wo_eff:                  3.234 -> 2.021 rad/s
外扰结束能量:           0.2550 -> 0.1740
恢复最后 3 s摆角 RMS:  12.109 -> 8.978 deg
恢复最后 3 s能量:       0.2784 -> 0.1527
```

同时：

```text
外扰段 XY RMS: 0.0921 -> 0.1462 m
恢复段 XY RMS: 0.1291 -> 0.2386 m
```

该数据表明观察器带宽降低后，摆动能量和持续摆角降低，但位置抗扰与恢复误差增大。

### 11.5 090716 控制权与饱和

`090716` 外扰结束后的前 3 s：

```text
最终水平加速度 P95 = 0.8 m/s^2
最终水平加速度最大值 = 0.8 m/s^2
||a_final|| >= 0.79 m/s^2 的样本比例 ≈ 49.9%
抗摆 applied P95 = 0.2 m/s^2
```

因此该段同时触及总加速度包络和抗摆加速度上限。

### 11.6 控制能量功率分解

以下为由命令加速度和滤波摆速计算的平均单位质量功率。正值表示该控制分量向摆动输入能量，负值表示消耗能量。

| 记录与阶段 | LADRC基础项 | 抗摆项 | 最终合成项 |
|---|---:|---:|---:|
| `225058` 外扰段 | `+0.00714` | `-0.00631` | `+0.00082` |
| `225058` 恢复段 | `-0.01374` | `-0.02311` | `-0.03665` |
| `090716` 外扰段 | `+0.00854` | `-0.00335` | `+0.00519` |
| `090716` 恢复段 | `-0.01283` | `-0.03707` | `-0.04476` |

两个完整方案在外扰段均表现为：

```text
基础位置控制项平均注入能量；
能量抗摆项平均消耗能量；
两者合成后仍为小幅正功率。
```

外扰结束后，基础位置控制与抗摆平均功率均转为负值，最终合成项快速消能。

### 11.7 摆动频段相位诊断

有限恢复窗口内的正弦拟合结果：

| 记录 | 主摆频/Hz | 命令加速度相对摆速 | 实际机体加速度相对摆速 | LESO补偿频段幅值 |
|---|---:|---:|---:|---:|
| `231033` 无抗摆高 `wo` | `0.714` | `-63.9 deg` | `-96.4 deg` | `0.0502 m/s²` |
| `082745` 无抗摆低 `wo` | `0.684` | `-55.1 deg` | `-72.8 deg` | `0.0202 m/s²` |
| `090716` 完整低 `wo` | `0.695` | `-7.1 deg` | `-64.1 deg` | `0.0146 m/s²` |

该拟合用于解释最近数据中的相位关系，不是完整的系统辨识结果。

### 11.8 083254 无扰悬停记录

`083254` 实际采用默认 `profile=hold`，没有外扰计划，且记录器未显式指定吊挂模型实体。PX4 内部 `hangjoint/hangas` 仍有数据，但没有生成 Gazebo `hang_joint_samples.csv`。

有效 hold 段：

```text
摆角 RMS = 1.261 deg
摆角峰值 = 2.159 deg
位置 XY RMS = 0.0228 m
位置峰值 = 0.0509 m
抗摆 applied P95 = 0.0137 m/s^2
safety state = 0
```

该记录只表明低 `wo` 完整配置在无扰 OFFBOARD 悬停中未出现持续发散或安全退出，不能作为统一外扰场景的对比组。

## 12. 当前已经确认的事实

1. `MC_PLADRC_EN=3` 的高度路径是 PX4 原生 PID，最近试验未再出现早期高度大阶跃持续冲高问题。
2. 一阶位置 LADRC 已不在可选择的主控制路径中，`MC_PLADRC_EN=1` 会退回 PID。
3. 关节到 body X/Y 的 `-1/-1` 符号在反向、Y 轴、航向变化和能量衰减数据中保持一致。
4. 能量抗摆原始控制律理论上具有非正能量导数。
5. 最近有效完整方案中，抗摆状态正常 engaged，未发生 safety abort。
6. 低 `wo` 无抗摆组相较高 `wo` 无抗摆组，恢复段摆动能量较低，但位置误差较高。
7. 低 `wo` 完整方案相较低 `wo` 无抗摆组，恢复末端摆角和能量显著降低，但位置误差进一步增大。
8. 当前 PID 基线在统一 `0.5 N / 6 s` 负载外扰场景中，位置、摆角峰值、末端能量和控制量均优于已测试完整方案。
9. 完整方案的抗摆项在平均意义上消耗能量，但外扰段基础 LADRC 项的平均能量输入更大，最终合成项仍为正。
10. 外扰结束后完整方案具有较强消能能力，但其恢复开始时负载已经具有高于 PID 的摆动能量。

## 13. 当前面临的问题与不确定边界

本节只记录问题，不记录拟采用的处理方式。

### 13.1 完整方案尚未优于 PID

在最近统一场景中，完整方案的：

- 外扰段位置 RMS 更大；
- 位置峰值更大；
- 摆角峰值更大；
- 外扰净能量增量更大；
- 加速度使用更大；
- 恢复末端能量仍高于 PID。

因此当前数据不能支持“完整方案整体性能优于原生 PID”的结论。

### 13.2 外扰段与恢复段行为不同

外扰段基础 LADRC 平均向吊挂输入能量，抗摆平均消耗能量，但不足以使最终控制功率转负。外扰结束后，两项均表现为平均耗能。当前系统存在明显的分阶段行为。

### 13.3 观察器带宽存在多指标权衡

高 `wo` 组的位置抗扰较好，但持续摆动和能量较大；低 `wo` 组的摆动频段相位和能量表现改善，但位置抗扰和恢复误差增大。当前标量带宽同时影响多个频率范围和多个指标。

### 13.4 基础位置控制与抗摆共享有限控制权

两者都输出水平加速度并受同一个 `0.8 m/s^2` 总包络约束。090716 恢复初期约一半样本接近总上限，抗摆项同时到达 `0.2 m/s^2` 上限。此时两个目标不能独立获得所请求的加速度。

### 13.5 理论被动性不等于完整闭环被动性

能量抗摆的原始输出满足：

```text
E_dot_AS,raw <= 0
```

但完整系统还包含：

- 输出斜率限制；
- 激活渐入；
- 总包络合成；
- 基础 LADRC 加速度；
- 姿态/角速度闭环动态；
- 推力饱和；
- 机体—吊挂耦合；
- 直接施加于负载的外力。

因此原始抗摆项的符号证明不能直接推出最终真实闭环能量单调下降。

### 13.6 LESO 已知输入与真实加速度不相同

当前 LESO 使用推力设定值反算加速度作为已知输入。日志中的真实 Gazebo 机体加速度在摆动频段相对命令存在明显幅值和相位差。该差异属于当前模型—执行链边界。

### 13.7 初始条件不完全一致

近期不同控制器和抗摆配置在外扰前的摆角 RMS 从 `0.186 deg` 到 `4.945 deg` 不等。绝对摆角和能量比较受到初始状态影响。文档同时记录了外扰净能量增量，但净增量也不能完全消除相位和初态差异。

### 13.8 当前外扰场景覆盖有限

最近统一数据集中于：

```text
L = 0.60 m
m_payload = 0.50 kg
force = 0.50 N
duration = 6 s
force frequency = 0.08333 Hz
direction = west
entity = hang_payload_link
```

这些数据尚不能代表其他绳长、质量、方向、扰动频率、轨迹和随机外扰下的行为。

### 13.9 当前频率约束是参数调度，不是在线频率识别

`omega_n` 完全由参数 `MC_HANG_LEN` 计算。模型绳长、有效摆长、闭环耦合频率和参数值之间的偏差不会被在线校正。

### 13.10 当前吊挂状态接口是 debug_array

关节状态和控制状态均通过通用 `debug_array` 传递或记录。它满足当前 SITL 验证，但不具备专用 uORB 消息的强类型语义和接口约束。

### 13.11 参数说明与实际优先级文字存在不一致

`MC_HANG_TOT_A` 参数注释中仍有“trajectory control has priority”的描述；当前 `PositionControl.cpp` 实际组合代码将抗摆请求作为 `constrainXY` 的 primary，并为抗摆优先保留加速度。现状判断以实际代码为准。

### 13.12 安全角裕量有限

当前 `MC_HANG_SAFE_A=0.35 rad`，约为 `20.1 deg`。090716 摆角范数峰值为 `18.46 deg`，接近但未超过安全阈值。日志中 safety state 保持正常。

### 13.13 当前门控在主要外扰段很快进入满权

`E_FULL=0.02 J/kg`，而近期外扰段能量均值通常为 `0.03~0.10 J/kg`。因此主要外扰阶段能量门控大部分时间接近或等于 1，能量大小主要通过摆速、阻尼增益和加速度限幅影响输出。

## 14. 当前结论边界

截至 2026-07-11，可以陈述：

```text
二阶 LADRC-X/Y + PID-Z + 绳长相关带宽约束 + 能量抗摆
已经在 PX4 SITL 中形成可运行、可关闭、可记录、可消融的完整闭环实现。
```

还可以陈述：

```text
能量抗摆模块能够显著降低外扰后的残余摆角和摆动能量；
观察器带宽会显著改变摆动频段相位、能量和位置抗扰表现；
当前完整方案在统一名义场景下尚未超过原生 PID。
```

当前不能陈述：

```text
完整方案已经在总体性能上优于 PID；
当前频率约束已经实现独立频带分离；
原始抗摆项的被动性等价于完整闭环严格被动；
现有单一绳长、质量和外扰场景足以证明鲁棒性；
当前性能差异只由某一个参数或某一个模块造成。
```

## 15. 主要代码位置

```text
mc_pos_control/MulticopterPositionControl.cpp
mc_pos_control/MulticopterPositionControl.hpp

mc_pos_control/PositionControl/PositionControl.cpp
mc_pos_control/PositionControl/PositionControl.hpp
mc_pos_control/PositionControl/LadrcPositionControl.cpp
mc_pos_control/PositionControl/LadrcPositionControl.hpp

mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.cpp
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.hpp
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadJointStateDebugArray.hpp

mc_pos_control/multicopter_position_ladrc_params.c
mc_pos_control/suspended_load_anti_swing_params.c

simulation/gz_bridge/GZSuspendedLoadJointBridge.cpp
simulation/gz_bridge/GZSuspendedLoadJointBridge.hpp

Tools/simulation/gz/models/zd680_hang/model.sdf
Tools/simulation/gz/tools/position_offboard_flight_recorder.py
```

## 16. 近期关键数据目录

```text
build/quick_eval/pid_equal_envelope_05n/2026-07-10/
  position_offboard_20260710_225339

build/quick_eval/full_disturbance_05n/2026-07-10/
  position_offboard_20260710_225058

build/quick_eval/ladrc_frq_no_as_05n/2026-07-10/
  position_offboard_20260710_231033

build/quick_eval/ladrc_no_as_wo202_05n/2026-07-11/
  position_offboard_20260711_082745

build/quick_eval/full_wo202_05n/2026-07-11/
  position_offboard_20260711_083254

build/quick_eval/full_wo202_05n_retry/2026-07-11/
  position_offboard_20260711_090716
```
