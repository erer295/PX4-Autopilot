# ZD680_HANG 吊挂负载控制实验总结

生成日期：2026-07-04

## 1. 当前结论

目前的 `ZD680_HANG` 模型、PID、LADRC、RBF-LADRC 三套控制和扰动记录链路已经基本打通。最近两轮有效对比的核心结论如下：

1. 三组控制器在 `ZD680_HANG` 上都能完成飞行，没有发现电机饱和、allocator 无法分配、failure detector 触发等硬故障。
2. 吊挂负载参数为质量 `0.50 kg`、绳长 `0.60 m`。对应小角度固有频率约 `0.643 Hz`。
3. 之前 `0.6 Hz` 扰动接近吊挂固有频率，容易激发明显摆动。
4. 后来改成 `0.35 Hz` 后，频率上避开了直接共振，但实际摆角没有减小，反而出现更大的低频摆幅。
5. PID 在吊挂摆动衰减和最终悬停恢复上仍然最好。
6. LADRC 的角速度内环更激进，容易把机体修正动作继续注入吊挂系统，导致摆动峰值偏大。
7. RBF-LADRC 在 `0.35 Hz` 扰动下位置误差最好，但当前 RBF 输出仍然很小，还不能认为它已经学到了有效的抗摆控制。

## 2. 代码架构

### 2.1 吊挂模型

模型文件：

```text
/home/lst/PX4-Autopilot/Tools/simulation/gz/models/zd680_hang/model.sdf
/home/lst/PX4-Autopilot/Tools/simulation/gz/models/zd680_hang/model.config
```

模型名称：

```text
zd680_hang
```

Gazebo 运行实例：

```text
zd680_hang_0
```

主要结构：

```text
base_link
  |
  |-- hang_roll_joint, revolute, X轴
      |
      hang_gimbal_link
        |
        |-- hang_pitch_joint, revolute, Y轴
            |
            hang_rope_link
              |
              |-- hang_payload_joint, fixed
                  |
                  hang_payload_link
```

关键参数：

| 项目 | 数值 |
|---|---:|
| 负载质量 `hang_payload_link/inertial/mass` | `0.50 kg` |
| 绳长 | `0.60 m` |
| 绳子质量 | `0.010 kg` |
| 吊点小球质量 | `0.005 kg` |
| 吊点相对 `base_link` 位置 | `z = -0.20 m` |
| `hang_roll_joint` 阻尼 | `0.02` |
| `hang_pitch_joint` 阻尼 | `0.02` |
| 摆角限位 | `±1.3962634 rad`, 约 `±80 deg` |

摆角记录 topic：

```text
/world/default/model/zd680_hang_0/joint_state
```

外力扰动目标：

```text
zd680_hang_0::hang_payload_link
```

注意：如果扰动目标写成 `zd680_0::base_link` 或 `zd680_0`，就不是当前吊挂负载实验。

### 2.2 扰动和轨迹记录脚本

脚本：

```text
/home/lst/PX4-Autopilot/Tools/simulation/gz/tools/position_offboard_flight_recorder.py
```

关键路径：

1. `GazeboWrenchClient` 通过 `/world/default/wrench/persistent` 发布持续外力。
2. `--entity` 可以覆盖默认目标实体。
3. 默认 `--model-name` 是 `zd680_0`，吊挂模型必须显式指定 `zd680_hang_0`。
4. `--wrench-schedule` 用于配置扰动开始时间、结束时间、幅值、方向、频率。

吊挂实验必须使用类似下面的实体参数：

```bash
--model-name zd680_hang_0 \
--link-name hang_payload_link \
--entity 'zd680_hang_0::hang_payload_link' \
--entity-type LINK
```

### 2.3 PID 内环

PID 仍使用 PX4 原始多旋翼角速度控制器：

```text
src/modules/mc_rate_control/RateControl
```

在 `MulticopterRateControl.cpp` 中，当 `MC_LADRC_EN=0` 时走 PID 路径：

```cpp
torque_setpoint = _rate_control.update(rates, _rates_setpoint, angular_accel, dt, on_ground);
```

PID 的优势是本身带有成熟的角加速度 D 阻尼路径，对吊挂负载这类柔性/欠驱动扰动更不容易过度激励摆动。

### 2.4 LADRC 内环

LADRC 文件：

```text
src/modules/mc_rate_control/ladrc_rate_control/LadrcRateControl.cpp
src/modules/mc_rate_control/ladrc_rate_control/LadrcRateControl.hpp
```

启用参数：

```text
MC_LADRC_EN = 1
MC_RBF_EN   = 0
```

主要参数：

| 参数 | 含义 |
|---|---|
| `MC_LADRC_B0_R/P/Y` | roll/pitch/yaw 等效输入增益 `b0` |
| `MC_LADRC_WC_R/P/Y` | 控制器带宽 `wc` |
| `MC_LADRC_WO_R/P/Y` | 观测器带宽 `wo` |
| `MC_LADRC_LIM_R/P/Y` | LADRC 归一化力矩限幅 |
| `MC_LADRC_D_R/P/Y` | 角加速度阻尼，类似 PID 的 D 路径 |

当前常用吊挂测试参数：

```text
MC_LADRC_B0_R = 120
MC_LADRC_B0_P = 120
MC_LADRC_WC_R = 8
MC_LADRC_WC_P = 8
MC_LADRC_WO_R = 18
MC_LADRC_WO_P = 18
MC_LADRC_D_R  = 0.003
MC_LADRC_D_P  = 0.003
MC_LADRC_D_Y  = 0
```

### 2.5 RBF-LADRC 内环

RBF 文件：

```text
src/modules/mc_rate_control/rbf_residual_compensation/RbfResidualCompensation.cpp
src/modules/mc_rate_control/rbf_residual_compensation/RbfResidualCompensation.hpp
```

启用参数：

```text
MC_LADRC_EN       = 1
MC_RBF_EN         = 1
MC_RBF_INJECT_EN  = 1
MC_RBF_LEARN_EN   = 1
```

RBF 接在 LADRC 后面：

```text
u_ladrc + u_rbf = u_final
```

当前常用参数：

```text
MC_RBF_LIM_R       = 0.015
MC_RBF_LIM_P       = 0.015
MC_RBF_LIM_Y       = 0
MC_RBF_ERR_GAIN    = 0.05
MC_RBF_ERR_WC      = 0
MC_RBF_LR          = 0.05
MC_RBF_LEAK        = 0.05
MC_RBF_LPF_ALPHA   = 0.95
```

当前限制：

1. RBF 输入没有直接使用 `hang_roll_joint`、`hang_pitch_joint` 摆角。
2. RBF 权重在退出 RBF-LADRC 或 disarm 后会 reset。
3. 目前 RBF 输出量级很小，对实际抗摆的贡献有限。

## 3. 主要公式

### 3.1 吊挂负载固有频率

小角度单摆近似：

```text
omega_n = sqrt(g / L)
f_n     = omega_n / (2*pi)
```

代入 `L = 0.60 m`：

```text
omega_n = sqrt(9.81 / 0.60) = 4.04 rad/s
f_n     = 4.04 / (2*pi) = 0.643 Hz
```

所以 `0.6 Hz` 扰动接近吊挂固有频率，容易激发共振。

### 3.2 负载水平外力静态摆角

外力作用在负载上时，静态摆角近似：

```text
theta_0 = atan(F / (m*g))
```

代入 `F = 1 N`、`m = 0.50 kg`：

```text
theta_0 = atan(1 / (0.5*9.81)) = 11.5 deg
```

因此如果实验中看到 `30 deg` 到 `45 deg` 的摆角，说明已经不是简单静态偏转，而是明显动态摆动。

### 3.3 简化吊挂摆动方程

小角度近似下，若扰动直接作用于负载，单方向摆动可写成：

```text
theta_ddot + 2*zeta*omega_n*theta_dot + omega_n^2*theta
    = F_payload / (m*L) - a_base / L
```

含义：

1. `F_payload` 是直接加在负载上的水平扰动力。
2. `a_base` 是无人机机体平动加速度。
3. 即使外力只加在负载上，飞控为了修正位置产生的机体加速度也会反过来激励吊挂。

这也是为什么 LADRC 机体修正越激进，负载摆动可能越大。

### 3.4 LADRC 一阶扩张状态观测器

代码中的等效角速度模型：

```text
rate_dot = f + b0*u
```

其中：

```text
z1 ≈ rate
z2 ≈ total_disturbance
```

LESO：

```text
e      = rate - z1
z1_dot = z2 + b0*u_last + beta1*e
z2_dot = beta2*e
```

观测器带宽整定：

```text
beta1 = 2*wo
beta2 = wo^2
```

### 3.5 LADRC 控制律

代码中的控制律：

```text
u_ladrc = (wc*(rate_sp - z1) - z2) / b0
```

加入角加速度阻尼后：

```text
u = u_ladrc - D_accel*angular_accel
```

再经过 LADRC 限幅：

```text
u = constrain(u, -MC_LADRC_LIM, +MC_LADRC_LIM)
```

LADRC 记录到 `rate_ctrl_status` 的扰动补偿量：

```text
disturbance_compensation = -z2 / b0
```

### 3.6 RBF-LADRC 残差补偿

每个轴的 RBF 输入特征为：

```text
x = [
  1,
  filtered_rate_error,
  u_ladrc,
  ladrc_disturbance_compensation,
  filtered_residual_accel
]
```

残差角加速度：

```text
predicted_accel = b0*(u_last - disturbance_compensation)
residual_accel  = angular_accel - predicted_accel
```

RBF 残差目标：

```text
u_target = -residual_accel_filtered / b0 * MC_RBF_ERR_GAIN
```

如果 `MC_RBF_ERR_WC > 0`，还会加入低频 rate error 项：

```text
u_target += MC_RBF_ERR_WC * filtered_rate_error / b0
```

RBF 高斯基函数：

```text
phi_j = exp(-0.5 * ||x - c_j||^2 / sigma_j^2)
```

输出：

```text
u_rbf_raw = sum(w_j * phi_j)
u_rbf     = LPF(constrain(u_rbf_raw, -MC_RBF_LIM, +MC_RBF_LIM))
```

最终注入：

```text
u_final = constrain(u_ladrc + MC_RBF_INJ*u_rbf, -final_limit, +final_limit)
```

在线学习使用带泄漏的 NLMS 形式：

```text
prediction = sum(w_j * phi_j)
fit_error  = u_target - prediction
weight_dot = learning_rate * fit_error * phi_j / (||phi||^2 + eps)
             - leakage * w_j
```

## 4. 实验设置

### 4.1 模型和扰动确认

最近有效实验均满足：

```text
SYS_AUTOSTART = 22002
model instance = zd680_hang_0
disturbance entity = zd680_hang_0::hang_payload_link
entity type = LINK
joint state topic = /world/default/model/zd680_hang_0/joint_state
```

也就是说，扰动是加在吊挂负载上，不是直接加在整个无人机机体上。

### 4.2 对比指标

主要看以下指标：

1. `XY RMS`：水平位置跟踪误差。
2. `Z RMS`：高度误差。
3. `rate_norm`：机体角速度误差强度。
4. `torque_rms`：控制力矩输出强度。
5. `tilt_rms / tilt_peak`：机体倾角。
6. `hang angle RMS / peak`：吊挂摆角。
7. `hang velocity RMS / peak`：吊挂摆动速度。
8. motor saturation、allocator unallocated torque/thrust、failure detector 状态。

## 5. 0.6 Hz 扰动实验结果

实验 ID：

| 控制器 | 日志 ID |
|---|---|
| LADRC | `162514` |
| PID | `163259` |
| RBF-LADRC | `165218` |

扰动：

```text
sine:25:45:1:west:0:none:0.6
```

即 `25 s` 到 `45 s`，`1 N`，向西，正弦扰动，频率 `0.6 Hz`。

核心结果：

| 控制器 | 扰动段 XY RMS | 扰动段 Z RMS | 扰动段 rate error | 摆角 RMS | 摆角峰值 | 摆速 RMS | 最后悬停摆角 RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| LADRC | `0.145 m` | `0.120 m` | `9.31 deg/s` | `14.48 deg` | `36.28 deg` | `45.49 deg/s` | `3.31 deg` |
| PID | `0.168 m` | `0.115 m` | `5.39 deg/s` | `11.32 deg` | `29.67 deg` | `25.29 deg/s` | `11.18 deg` |
| RBF-LADRC | `0.194 m` | `0.066 m` | `6.98 deg/s` | `16.25 deg` | `36.00 deg` | `30.70 deg/s` | `3.47 deg` |

结论：

1. `0.6 Hz` 接近 `0.643 Hz` 固有频率，确实容易激发吊挂。
2. PID 在扰动段摆角最小，说明 PID 的阻尼特性更适合当前吊挂模型。
3. LADRC 和 RBF-LADRC 在最后恢复悬停时摆角比 PID 小，但扰动段摆动更大。
4. RBF 输出量级很小，不能把 RBF-LADRC 和 LADRC 的差异完全解释为有效学习补偿。

## 6. 0.35 Hz 扰动实验结果

实验 ID：

| 控制器 | 日志 ID |
|---|---|
| LADRC | `173125` |
| PID | `173901` |
| RBF-LADRC | `174613` |

扰动：

```text
sine:25:45:1:west:0:none:0.35
```

即 `25 s` 到 `45 s`，`1 N`，向西，正弦扰动，频率 `0.35 Hz`。

### 6.1 有效性检查

三组均满足：

1. 使用 `ZD680_HANG` 吊挂模型。
2. 扰动加在 `zd680_hang_0::hang_payload_link`。
3. 记录了 `hang_roll_joint` 和 `hang_pitch_joint`。
4. 无 failure detector 触发。
5. 无明显 motor saturation。
6. allocator unallocated torque/thrust 接近零。

### 6.2 主要位置误差

| 控制器 | 全动作 XY RMS | 全动作 Z RMS | 扰动段 XY RMS | 扰动段 Z RMS | 线性段 XY RMS | 线性段 Z RMS | 圆轨迹 XY RMS | 圆轨迹 Z RMS | 悬停 XY RMS | 悬停 Z RMS |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| LADRC | `0.110 m` | `0.122 m` | `0.154 m` | `0.113 m` | `0.152 m` | `0.111 m` | `0.129 m` | `0.081 m` | `0.040 m` | `0.151 m` |
| PID | `0.130 m` | `0.047 m` | `0.151 m` | `0.061 m` | `0.139 m` | `0.061 m` | `0.185 m` | `0.039 m` | `0.031 m` | `0.040 m` |
| RBF-LADRC | `0.115 m` | `0.049 m` | `0.133 m` | `0.032 m` | `0.124 m` | `0.031 m` | `0.165 m` | `0.063 m` | `0.025 m` | `0.047 m` |

位置误差结论：

1. 扰动段 `RBF-LADRC` 最好，尤其 Z 方向明显最好。
2. LADRC 最后的悬停 Z 误差偏大，存在高度恢复或低频偏差问题。
3. PID 在 Z 方向和最后悬停恢复上仍然很稳。

### 6.3 角速度和控制输出

| 控制器 | 全动作 rate_norm | 扰动段 rate_norm | 悬停 rate_norm | 全动作 torque RMS | 扰动段 torque RMS | 悬停 torque RMS |
|---|---:|---:|---:|---:|---:|---:|
| LADRC | `8.78 deg/s` | `15.20 deg/s` | `2.55 deg/s` | `0.0183` | `0.0319` | `0.0053` |
| PID | `3.97 deg/s` | `6.29 deg/s` | `0.48 deg/s` | `0.0193` | `0.0350` | `0.0013` |
| RBF-LADRC | `7.77 deg/s` | `12.89 deg/s` | `2.38 deg/s` | `0.0166` | `0.0277` | `0.0050` |

控制输出结论：

1. LADRC 和 RBF-LADRC 的角速度误差明显高于 PID。
2. PID 的 rate_norm 最小，说明内环更平顺。
3. LADRC 不是力矩不够，而是响应方式更容易激励吊挂。

### 6.4 吊挂摆角

| 控制器 | 全动作摆角 RMS | 全动作摆角峰值 | 扰动段摆角 RMS | 扰动段摆角峰值 | 扰动段摆速 RMS | 扰动段摆速峰值 | 最后悬停摆角 RMS |
|---|---:|---:|---:|---:|---:|---:|---:|
| LADRC | `11.19 deg` | `47.03 deg` | `19.81 deg` | `47.03 deg` | `81.61 deg/s` | `164.38 deg/s` | `3.27 deg` |
| PID | `10.41 deg` | `41.19 deg` | `19.52 deg` | `41.19 deg` | `32.86 deg/s` | `81.79 deg/s` | `0.70 deg` |
| RBF-LADRC | `10.26 deg` | `36.95 deg` | `17.35 deg` | `36.95 deg` | `70.04 deg/s` | `139.25 deg/s` | `3.04 deg` |

摆动结论：

1. 扰动段 `RBF-LADRC` 摆角 RMS 和峰值最低。
2. PID 最后悬停恢复最好，最终摆角 RMS 只有 `0.70 deg`。
3. LADRC 摆角峰值最大，达到 `47.03 deg`。
4. LADRC 和 RBF-LADRC 摆速明显大于 PID，说明它们对吊挂系统注入的动态能量更多。

## 7. 0.35 Hz 与 0.6 Hz 的对比

| 控制器 | 指标 | 0.6 Hz | 0.35 Hz | 变化 |
|---|---|---:|---:|---|
| LADRC | 扰动段摆角 RMS | `14.48 deg` | `19.81 deg` | 变差 |
| PID | 扰动段摆角 RMS | `11.32 deg` | `19.52 deg` | 变差 |
| RBF-LADRC | 扰动段摆角 RMS | `16.25 deg` | `17.35 deg` | 略变差 |
| LADRC | 扰动段摆角峰值 | `36.28 deg` | `47.03 deg` | 变差 |
| PID | 扰动段摆角峰值 | `29.67 deg` | `41.19 deg` | 变差 |
| RBF-LADRC | 扰动段摆角峰值 | `36.00 deg` | `36.95 deg` | 基本相近 |
| LADRC | 扰动段 XY RMS | `0.145 m` | `0.154 m` | 略变差 |
| PID | 扰动段 XY RMS | `0.168 m` | `0.151 m` | 略变好 |
| RBF-LADRC | 扰动段 XY RMS | `0.194 m` | `0.133 m` | 明显变好 |
| LADRC | 最后悬停摆角 RMS | `3.31 deg` | `3.27 deg` | 基本相同 |
| PID | 最后悬停摆角 RMS | `11.18 deg` | `0.70 deg` | 明显变好 |
| RBF-LADRC | 最后悬停摆角 RMS | `3.47 deg` | `3.04 deg` | 略变好 |

结论：

1. `0.35 Hz` 避开了 `0.6 m` 绳长对应的 `0.643 Hz` 直接共振点。
2. 但 `0.35 Hz` 的半周期更长，外力在同一方向持续作用时间更久，会产生更大的低频位移。
3. 因此共振问题在频率意义上有所缓解，但实际吊挂摆动没有解决。
4. `0.35 Hz` 不适合作为“更容易”的主对比频率，它更像低频大位移扰动测试。

## 8. 为什么 LADRC 目前没有优于 PID

### 8.1 LADRC 对吊挂负载过于激进

LADRC 的设计目标是快速估计并补偿总扰动，但吊挂负载不是刚体扰动，而是带固有频率的欠驱动系统。机体快速修正位置时，会通过吊点把能量输入负载摆动。

现象：

```text
0.35 Hz 扰动段 rate_norm:
LADRC      15.20 deg/s
PID         6.29 deg/s
RBF-LADRC  12.89 deg/s
```

LADRC rate error 更大，摆速也更大，说明它在不断快速修正，负载则被进一步激励。

### 8.2 PID 的 D 阻尼对吊挂更友好

PID 的角加速度 D 路径天然抑制高频姿态变化。吊挂负载最怕机体急加速、急修正。PID 虽然不是专门为吊挂设计，但它的阻尼特性正好减少了对负载的激励。

### 8.3 LADRC 的 LESO 把吊挂动态当作扰动补偿

LADRC 的 `z2` 估计的是总扰动，但吊挂摆动会随时间滞后、换向、储能。若 `WO` 偏高，LESO 可能追着吊挂动态快速补偿，反而形成新的激励闭环。

### 8.4 当前 RBF 没有直接感知摆角

RBF 当前输入是角速度误差、LADRC 力矩、LADRC 扰动补偿和残差角加速度，没有直接输入：

```text
hang_roll_joint
hang_pitch_joint
hang_roll_velocity
hang_pitch_velocity
```

因此它无法明确知道负载在往哪里摆，也无法形成真正意义上的 anti-swing 控制。

## 9. 下一步建议

### 9.1 扰动频率扫描

不要只用 `0.35 Hz` 和 `0.6 Hz`。建议增加：

```text
0.2 Hz   准静态低频拖拽
0.45 Hz  接近但低于固有频率
0.8 Hz   避开固有频率，较适合作为下一组主测试
1.0 Hz   高频扰动测试
```

下一组优先跑：

```text
0.8 Hz
```

理由：

1. 避开 `0.643 Hz` 固有频率。
2. 不像 `0.35 Hz` 那样长时间单向拉负载。
3. 更适合比较控制器的非共振抗扰性能。

### 9.2 LADRC 参数方向

当前不建议增大 `WC`。问题不是响应太慢，而是响应太急。下一步建议只降低 `WO`：

第一组：

```text
MC_LADRC_B0_R = 120
MC_LADRC_B0_P = 120
MC_LADRC_WC_R = 8
MC_LADRC_WC_P = 8
MC_LADRC_WO_R = 14
MC_LADRC_WO_P = 14
MC_LADRC_D_R  = 0.003
MC_LADRC_D_P  = 0.003
```

如果仍然摆动明显，再试：

```text
MC_LADRC_WO_R = 12
MC_LADRC_WO_P = 12
```

目标是降低 LESO 对吊挂动态的追随速度，让机体动作更柔和。

### 9.3 RBF-LADRC 改进方向

短期：

1. 保持 `MC_RBF_LIM_R/P = 0.010` 到 `0.015`。
2. 可以尝试把 `MC_RBF_ERR_GAIN` 从 `0.05` 提到 `0.10`，但要先在 `0.8 Hz` 小心验证。
3. 暂时不要大幅提高 RBF 输出限幅，避免它把 LADRC 的激进问题放大。

中期：

把吊挂状态加入 RBF 特征：

```text
hang_roll_angle
hang_pitch_angle
hang_roll_velocity
hang_pitch_velocity
```

这样 RBF 才能从“角速度残差补偿”变成“吊挂感知补偿”。

### 9.4 需要继续关注的问题

1. LADRC 在 `0.35 Hz` 最后悬停 Z RMS 为 `0.151 m`，明显高于 PID 和 RBF-LADRC，需要重复确认是否为单次偏差。
2. LADRC 摆角峰值达到 `47.03 deg`，接近大角度摆动，已经明显超出小角度线性近似范围。
3. RBF 当前输出很小，若后续论文或报告中声称 RBF 明显改善，需要补充 RBF 输出、权重范数和学习有效时间比例作为证据。

## 10. 推荐下一轮实验矩阵

建议下一轮先固定扰动：

```text
sine:25:45:1:west:0:none:0.8
```

然后跑四组：

| 组别 | 控制器 | 参数重点 |
|---|---|---|
| A | PID | 基准 |
| B | LADRC | `WO=18`, `D=0.003` |
| C | LADRC | `WO=14`, `D=0.003` |
| D | RBF-LADRC | 基于 `WO=14` 或 `WO=18`，保持 RBF 小限幅 |

判据：

1. 如果 `WO=14` 明显降低摆角和摆速，说明 LADRC 主要问题是观测器过快。
2. 如果 `0.8 Hz` 下 LADRC 接近或超过 PID，说明之前主要是频率和吊挂耦合问题。
3. 如果 `0.8 Hz` 下 PID 仍然最好，说明当前 LADRC 结构还需要显式 anti-swing 信息。
4. 如果 RBF-LADRC 的输出仍然很小，则下一步不应继续只调 RBF 学习率，而应加入吊挂角特征。

## 11. 当前最重要的判断

现在不是动力不足，也不是模型没有加载吊挂，而是控制器和吊挂负载之间的动态耦合问题。

PID 目前更稳，是因为它的内环阻尼和响应速度对吊挂更温和。LADRC 理论抗扰能力强，但在吊挂负载上，如果没有摆角反馈或更柔和的观测器/控制器带宽，它会把吊挂运动当作普通扰动快速补偿，反而可能激发摆动。

因此，下一阶段的重点不是继续盲目增强 LADRC 或 RBF，而是：

```text
先避开共振频率做公平对比，再降低 LADRC 观测器带宽，最后把吊挂摆角引入 RBF/LADRC。
```
