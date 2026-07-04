# ZD680_HANG 吊挂抗摆外环设计方案

生成日期：2026-07-04

## 1. 为什么现在需要改控制器结构

目前 PID、LADRC、RBF-LADRC 在 `ZD680_HANG` 吊挂模型上的对比已经暴露出一个比较明确的问题：

1. LADRC 的位置跟踪能力不差，经常比 PID 的 XY 误差更小。
2. 但 LADRC 的吊挂摆角、摆速、机体 rate error 明显比 PID 更大。
3. 单独降低 `MC_LADRC_WO_R/P` 到 `14` 后，摆动没有减小，反而加重。
4. 当前 LADRC 和 RBF-LADRC 都没有直接使用 `hang_roll_joint`、`hang_pitch_joint`，所以控制器不知道负载正在向哪里摆。

因此，现在的问题不是单纯“LADRC 参数没调好”，而是控制器缺少吊挂负载状态反馈。继续只调 `WO/WC/D`，很容易出现下面这种情况：

```text
位置误差变小，但吊挂摆动变大。
```

更合理的方向是在位置/加速度外环加入吊挂抗摆控制，让无人机的水平加速度指令主动避开或抑制吊挂摆动。

## 2. 当前 PX4 控制链路位置

PX4 多旋翼位置控制链路大致为：

```text
trajectory_setpoint
    |
    v
PositionControl::_positionControl()
    |
    v
PositionControl::_velocityControl()
    |
    | 生成 _acc_sp
    v
PositionControl::_accelerationControl()
    |
    | 将加速度指令转换为 thrust vector
    v
vehicle_attitude_setpoint
    |
    v
mc_att_control
    |
    v
mc_rate_control, PID / LADRC / RBF-LADRC
```

关键文件：

```text
src/modules/mc_pos_control/PositionControl/PositionControl.cpp
src/modules/mc_pos_control/PositionControl/PositionControl.hpp
src/modules/mc_pos_control/MulticopterPositionControl.cpp
src/modules/mc_pos_control/MulticopterPositionControl.hpp
```

当前 `PositionControl::_velocityControl()` 中的核心逻辑是：

```cpp
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

_accelerationControl();
```

这说明最适合加入抗摆外环的位置是：

```text
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

// 在这里加入吊挂抗摆补偿：
_acc_sp.xy() += anti_swing_acc_xy;

_accelerationControl();
```

这样做的好处：

1. 抗摆补偿直接作用在水平加速度指令上，物理意义清楚。
2. 不破坏 attitude/rate 内环。
3. 可以同时适用于 PID、LADRC、RBF-LADRC。
4. 便于观察：`vehicle_local_position_setpoint.acceleration` 和姿态 setpoint 都会体现抗摆修正。

## 3. 吊挂动力学基础

### 3.1 小角度单摆固有频率

吊挂负载可近似为单摆：

```text
omega_n = sqrt(g / L)
f_n     = omega_n / (2*pi)
```

当前模型：

```text
L = 0.60 m
g = 9.81 m/s^2
```

所以：

```text
omega_n = sqrt(9.81 / 0.60) = 4.04 rad/s
f_n     = 4.04 / (2*pi) = 0.643 Hz
```

这解释了为什么 `0.6 Hz` 附近非常容易激发摆动。

### 3.2 吊点水平加速度对摆动的影响

小角度下，单方向吊挂摆动可以写成：

```text
theta_ddot + 2*zeta*omega_n*theta_dot + omega_n^2*theta
    = -a_base / L + F_payload / (m*L)
```

其中：

| 符号 | 含义 |
|---|---|
| `theta` | 负载相对竖直方向的摆角 |
| `theta_dot` | 摆角速度 |
| `a_base` | 无人机吊点水平加速度 |
| `F_payload` | 直接作用在负载上的水平外力 |
| `m` | 负载质量 |
| `L` | 绳长 |

这条式子说明：

1. 即使扰动加在负载上，无人机自己的水平加速度也会影响吊挂摆动。
2. 位置控制越激进，吊点加速度越急，越容易继续激励负载。
3. 抗摆控制最自然的输入就是 `theta` 和 `theta_dot`，最自然的输出就是水平加速度修正。

## 4. 推荐控制结构

推荐结构：

```text
位置/速度控制器输出：a_pos_xy
吊挂抗摆外环输出：a_swing_xy

最终水平加速度：

a_cmd_xy = a_pos_xy + a_swing_xy
```

其中：

```text
a_swing_xy = sat( L * (K_theta * theta_xy + K_rate * theta_dot_xy), acc_limit )
```

参数含义：

| 参数 | 含义 |
|---|---|
| `L` | 绳长，默认 `0.60 m` |
| `K_theta` | 摆角反馈增益 |
| `K_rate` | 摆角速度阻尼增益 |
| `acc_limit` | 抗摆加速度限幅 |

第一版建议先只做阻尼：

```text
K_theta = 0
K_rate  = 1.5
acc_limit = 0.6 m/s^2
```

如果摆速下降，再加入摆角反馈：

```text
K_theta = 2.0
K_rate  = 2.5
acc_limit = 0.8 m/s^2
```

## 5. 轴向映射

模型中的吊挂关节为：

```text
hang_roll_joint   绕 X 轴
hang_pitch_joint  绕 Y 轴
```

对于小角度，可以先按如下思路映射到机体系水平摆角：

```text
theta_body_x ≈ sign_pitch * hang_pitch_joint
theta_body_y ≈ sign_roll  * hang_roll_joint
```

摆角速度同理：

```text
theta_dot_body_x ≈ sign_pitch * hang_pitch_joint_velocity
theta_dot_body_y ≈ sign_roll  * hang_roll_joint_velocity
```

但是符号一定要做成参数，因为 Gazebo SDF 关节方向、PX4 NED、机体系 FRD 之间可能存在符号差异。建议参数：

```text
MC_HANG_SIGN_X = 1 或 -1
MC_HANG_SIGN_Y = 1 或 -1
```

调试符号的方法：

1. 悬停时手动施加一个短暂 west/east 扰动。
2. 看 `hang_roll_joint`、`hang_pitch_joint` 的响应方向。
3. 开启很小的 `K_rate`，如果摆速变大，说明符号错了。
4. 符号正确时，抗摆补偿应该让摆速 RMS 下降。

## 6. 信号来源方案

这是整个方案里最关键的一点。

目前 recorder 脚本已经能记录：

```text
/world/default/model/zd680_hang_0/joint_state
```

并写入：

```text
hang_joint_samples.csv
```

但是 PX4 飞控内部当前并不知道这些 Gazebo joint states。因此，如果要在 PX4 控制器里真正使用摆角，需要把 Gazebo joint state 引入 PX4。

### 6.1 方案 A：脚本级抗摆，最快验证

最快的验证方法是在 `position_offboard_flight_recorder.py` 里直接做抗摆：

```text
gz joint_state -> recorder script -> 修改 offboard setpoint -> PX4
```

优点：

1. 不需要改 PX4 uORB 消息。
2. 可以很快验证 `K_rate/K_theta/acc_limit/sign` 是否有效。
3. 适合先确认抗摆控制方向。

缺点：

1. 这不是 PX4 内部控制器。
2. 受 MAVLink setpoint 频率和延迟影响。
3. 论文或最终算法实现时说服力不如 PX4 内部闭环。

脚本级可行做法：

1. 复用现有 `HangJointRecorder`，实时得到 `roll_position/pitch_position/velocity`。
2. 计算 `a_swing_xy`。
3. 如果 MAVLink setpoint 支持 acceleration feed-forward，则把 `a_swing_xy` 加到 setpoint acceleration。
4. 如果当前 setpoint 发送路径主要是 position setpoint，也可以先把 `a_swing_xy` 等效为一个小的速度/位置偏置，但这不如 acceleration feed-forward 干净。

适合用于第一轮符号和增益验证。

### 6.2 方案 B：PX4 内部抗摆，推荐最终方案

推荐最终结构：

```text
Gazebo joint_state
    |
    v
PX4 SITL bridge / MAVLink injection
    |
    v
uORB: suspended_load_status
    |
    v
MulticopterPositionControl
    |
    v
PositionControl::_acc_sp.xy()
```

建议新建一个 uORB topic：

```text
suspended_load_status
```

建议字段：

```text
uint64 timestamp
uint64 timestamp_sample
float32 roll_angle_rad
float32 pitch_angle_rad
float32 roll_rate_rad_s
float32 pitch_rate_rad_s
float32 angle_norm_rad
float32 rate_norm_rad_s
bool valid
```

然后 `MulticopterPositionControl` 订阅这个 topic，把吊挂状态传给 `PositionControl`。

优点：

1. 控制器闭环完整。
2. 抗摆逻辑可以和 PID/LADRC/RBF-LADRC 公平对比。
3. 可以记录 uORB 日志，分析更方便。

缺点：

1. 需要增加 uORB 消息或复用已有消息。
2. 需要写 Gazebo joint state 到 PX4 的桥接。
3. 实现周期比脚本级方案长。

## 7. 推荐参数设计

建议新增参数名称：

```text
MC_HANG_AS_EN
MC_HANG_LEN
MC_HANG_K_ANG
MC_HANG_K_RATE
MC_HANG_ACC_LIM
MC_HANG_LPF_HZ
MC_HANG_SIGN_X
MC_HANG_SIGN_Y
MC_HANG_MAX_ANG
MC_HANG_TIMEOUT
```

参数表：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `MC_HANG_AS_EN` | `0` | 是否启用吊挂抗摆外环 |
| `MC_HANG_LEN` | `0.60` | 绳长，单位 m |
| `MC_HANG_K_ANG` | `0.0` | 摆角反馈增益 |
| `MC_HANG_K_RATE` | `1.5` | 摆角速度阻尼增益 |
| `MC_HANG_ACC_LIM` | `0.6` | 抗摆水平加速度限幅，单位 m/s^2 |
| `MC_HANG_LPF_HZ` | `4.0` | 摆角/摆速低通滤波截止频率 |
| `MC_HANG_SIGN_X` | `1` | X 方向符号 |
| `MC_HANG_SIGN_Y` | `1` | Y 方向符号 |
| `MC_HANG_MAX_ANG` | `0.8` | 最大有效摆角，单位 rad，约 45.8 deg |
| `MC_HANG_TIMEOUT` | `0.2` | 吊挂状态超时时间，单位 s |

建议第一轮测试：

```text
MC_HANG_AS_EN   = 1
MC_HANG_LEN     = 0.60
MC_HANG_K_ANG   = 0.0
MC_HANG_K_RATE  = 1.5
MC_HANG_ACC_LIM = 0.6
MC_HANG_LPF_HZ  = 4.0
```

第二轮测试：

```text
MC_HANG_K_ANG   = 1.0
MC_HANG_K_RATE  = 2.0
MC_HANG_ACC_LIM = 0.8
```

第三轮测试：

```text
MC_HANG_K_ANG   = 2.0
MC_HANG_K_RATE  = 2.5
MC_HANG_ACC_LIM = 0.8
```

不建议一开始超过：

```text
MC_HANG_ACC_LIM > 1.0 m/s^2
```

因为当前扰动是 `1 N` 作用在 `0.5 kg` 负载上，静态等效加速度已经不小。抗摆外环过强会和位置控制抢控制权。

## 8. 控制律细节

### 8.1 原始抗摆输出

机体系下：

```text
a_as_body_x = L * (K_theta * theta_x + K_rate * theta_dot_x)
a_as_body_y = L * (K_theta * theta_y + K_rate * theta_dot_y)
```

向量形式：

```text
a_as_body = L * (K_theta * theta_body + K_rate * theta_dot_body)
```

### 8.2 限幅

水平抗摆加速度必须限幅：

```text
if norm(a_as_body) > acc_limit:
    a_as_body = a_as_body / norm(a_as_body) * acc_limit
```

### 8.3 机体系转 NED 水平系

如果 `PositionControl` 内部使用 NED 的 `_acc_sp.xy()`，而吊挂角被整理成 body frame，则需要用当前 yaw 做旋转：

```text
a_north = cos(yaw) * a_body_x - sin(yaw) * a_body_y
a_east  = sin(yaw) * a_body_x + cos(yaw) * a_body_y
```

即：

```text
a_as_ned_xy = R_yaw * a_as_body_xy
```

然后：

```text
_acc_sp(0) += a_as_ned_x
_acc_sp(1) += a_as_ned_y
```

### 8.4 低通滤波

对角度和角速度做一阶低通：

```text
alpha = exp(-2*pi*f_cut*dt)
x_filt = alpha*x_filt + (1-alpha)*x
```

建议：

```text
f_cut = 3 ~ 5 Hz
```

因为吊挂主频约 `0.64 Hz`，抗摆外环不需要高频噪声。

### 8.5 死区

可以设置小死区，避免悬停时微小噪声引入晃动：

```text
if abs(theta) < 0.5 deg and abs(theta_dot) < 1 deg/s:
    a_as = 0
```

第一版可以不加死区，只做低通和限幅。

## 9. 安全保护

必须加以下保护：

### 9.1 数据超时

如果吊挂状态超过 `MC_HANG_TIMEOUT` 没更新：

```text
a_as = 0
```

不要使用旧的摆角数据。

### 9.2 未飞行关闭

以下状态关闭抗摆：

```text
not armed
landed
maybe_landed
takeoff rampup before flight
```

原因是地面时吊挂可能与仿真初始化、起飞瞬间接触状态有关，不适合学习或补偿。

### 9.3 大角度保护

如果：

```text
sqrt(theta_x^2 + theta_y^2) > MC_HANG_MAX_ANG
```

建议第一版直接关闭角度反馈，只保留限幅后的速度阻尼，或者完全关闭抗摆：

```text
a_as = constrain(L*K_rate*theta_dot, acc_limit)
```

原因是大角度下小角度模型不再准确。

### 9.4 加速度限幅

无论任何情况：

```text
norm(a_as_xy) <= MC_HANG_ACC_LIM
```

### 9.5 输出变化率限制

建议增加 slew rate：

```text
|a_as[k] - a_as[k-1]| / dt <= MC_HANG_ACC_SLEW
```

建议默认：

```text
MC_HANG_ACC_SLEW = 2.0 m/s^3
```

如果第一版想简单，可以先不做 slew rate，但至少要做低通和限幅。

## 10. 推荐代码落点

### 10.1 PositionControl 新增接口

在 `PositionControl.hpp` 中增加：

```cpp
void setSuspendedLoadAntiSwing(
    bool enabled,
    const matrix::Vector2f &angle_body,
    const matrix::Vector2f &rate_body,
    float rope_length,
    float k_angle,
    float k_rate,
    float acc_limit,
    float yaw);
```

或者更干净一点，新建结构体：

```cpp
struct SuspendedLoadAntiSwing {
    bool enabled;
    matrix::Vector2f angle_body;
    matrix::Vector2f rate_body;
    float rope_length;
    float k_angle;
    float k_rate;
    float acc_limit;
    float yaw;
};
```

然后：

```cpp
void setSuspendedLoadAntiSwing(const SuspendedLoadAntiSwing &anti_swing);
```

### 10.2 PositionControl 中应用补偿

在 `PositionControl::_velocityControl()` 中：

```cpp
ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

applySuspendedLoadAntiSwing(dt);

_accelerationControl();
```

`applySuspendedLoadAntiSwing()` 做：

```cpp
if (!enabled) {
    return;
}

Vector2f a_body = rope_length * (k_angle * angle_body + k_rate * rate_body);
a_body = constrainNorm(a_body, acc_limit);

Vector2f a_ned;
a_ned(0) = cosf(yaw) * a_body(0) - sinf(yaw) * a_body(1);
a_ned(1) = sinf(yaw) * a_body(0) + cosf(yaw) * a_body(1);

_acc_sp(0) += a_ned(0);
_acc_sp(1) += a_ned(1);
```

### 10.3 MulticopterPositionControl 中传入吊挂状态

在 `MulticopterPositionControl` 中：

1. 订阅吊挂状态 topic。
2. 在每次 `Run()` 中更新吊挂状态。
3. 根据参数和状态有效性调用 `_control.setSuspendedLoadAntiSwing(...)`。
4. 如果状态无效，传入 `enabled=false`。

位置建议在：

```text
_control.setInputSetpoint(_setpoint);
...
_control.setState(states);
...
_control.update(dt);
```

之间设置 anti-swing 状态。

## 11. 和 LADRC/RBF-LADRC 的关系

抗摆外环和 LADRC/RBF-LADRC 不冲突。

推荐结构：

```text
吊挂抗摆外环：修正位置控制器水平加速度
LADRC：执行姿态/rate 内环抗扰
RBF：补偿 LADRC 剩余角速度残差
```

也就是：

```text
anti-swing outer loop
    -> position acceleration setpoint
        -> attitude setpoint
            -> LADRC / RBF-LADRC rate controller
```

不要把吊挂抗摆直接塞进 LADRC 的 `z2` 或 `u` 里作为第一版，因为：

1. LADRC 工作在角速度层，不直接控制吊点水平加速度。
2. 吊挂摆动是低频欠驱动模态，直接在 rate 层补偿容易引入更急的姿态动作。
3. 当前数据已经说明 LADRC 内环更激进会放大摆速。

## 12. RBF-LADRC 后续扩展

等抗摆外环稳定后，再改 RBF 输入。

当前 RBF 特征大致为：

```text
[
  filtered_rate_error,
  u_ladrc,
  ladrc_disturbance_compensation,
  filtered_residual_accel
]
```

建议扩展为：

```text
[
  filtered_rate_error,
  u_ladrc,
  ladrc_disturbance_compensation,
  filtered_residual_accel,
  hang_roll_angle,
  hang_pitch_angle,
  hang_roll_rate,
  hang_pitch_rate,
  anti_swing_acc_x,
  anti_swing_acc_y
]
```

这样 RBF 才能区分：

```text
这是普通角速度误差，
还是吊挂负载引起的周期性摆动。
```

但是不要第一步就这么做。先把确定性的抗摆外环做稳定，再加入 RBF 学习，实验更好解释。

## 13. 实验验证计划

### 13.1 基准组

保持现有轨迹：

```text
hold:6;north:4:8;east:4:8;south:4:8;west:4:8;orbit:3:24:2;hold:16
```

扰动：

```text
sine:25:45:1:west:0:none:0.8
```

基准组：

| 组别 | 控制器 | 抗摆 |
|---|---|---|
| A | PID | off |
| B | LADRC, `WC=8, WO=18, D=0.003` | off |
| C | LADRC, `WC=8, WO=18, D=0.003` | on, only rate |
| D | LADRC, `WC=8, WO=18, D=0.003` | on, angle + rate |

### 13.2 第一轮参数

只开速度阻尼：

```text
MC_HANG_K_ANG   = 0.0
MC_HANG_K_RATE  = 1.5
MC_HANG_ACC_LIM = 0.6
```

目标：

```text
摆速 RMS 明显下降
摆角 RMS 不明显上升
XY RMS 不恶化超过 20%
```

### 13.3 第二轮参数

加入小角度反馈：

```text
MC_HANG_K_ANG   = 1.0
MC_HANG_K_RATE  = 2.0
MC_HANG_ACC_LIM = 0.8
```

目标：

```text
扰动结束后 10 s 摆角 RMS 明显下降
最后 3 s 摆角 RMS 明显下降
```

### 13.4 第三轮参数

增强抗摆：

```text
MC_HANG_K_ANG   = 2.0
MC_HANG_K_RATE  = 2.5
MC_HANG_ACC_LIM = 0.8
```

如果这组导致 XY RMS 明显变差或摆速上升，说明过强，应回退。

## 14. 判断指标

每组实验至少统计：

### 14.1 位置指标

```text
disturbance XY RMS
disturbance Z RMS
final_hold_clean_recovery XY RMS
final_hold_clean_recovery Z RMS
```

### 14.2 吊挂指标

```text
disturbance hang angle RMS
disturbance hang angle peak
disturbance hang velocity RMS
disturbance hang velocity peak
post_disturbance hang angle RMS
post_disturbance hang velocity RMS
last 3 s hang angle RMS
```

### 14.3 内环指标

```text
rate error RMS
torque RMS
tilt RMS / peak
motor min/max
allocator unallocated torque/thrust
failure detector
```

### 14.4 成功标准

抗摆外环有效的最低标准：

```text
扰动段 hang velocity RMS 下降 > 25%
扰动后 10 s hang angle RMS 下降 > 30%
XY RMS 增加 < 20%
无 motor saturation
无 failure detector
```

更理想的目标：

```text
LADRC + anti-swing 的 XY RMS 仍优于 PID
同时吊挂摆角/摆速接近或优于 PID
```

## 15. 可能出现的问题和处理

### 15.1 开启后摆动更大

最可能是符号错。

处理：

```text
MC_HANG_SIGN_X *= -1
或
MC_HANG_SIGN_Y *= -1
```

先只开一个方向调试，不要两个方向一起调。

### 15.2 XY 跟踪明显变差

说明抗摆加速度和位置控制抢控制权。

处理：

```text
降低 MC_HANG_ACC_LIM
降低 MC_HANG_K_ANG
保留或小幅降低 MC_HANG_K_RATE
```

优先降低角度项，不要先砍速度阻尼。

### 15.3 摆速下降但摆角不下降

说明阻尼有效，但缺少位置恢复。

处理：

```text
逐步增加 MC_HANG_K_ANG
```

例如：

```text
0.0 -> 0.5 -> 1.0 -> 1.5 -> 2.0
```

### 15.4 摆角下降但机体动作很急

说明抗摆过强或滤波太高。

处理：

```text
降低 MC_HANG_ACC_LIM
降低 MC_HANG_LPF_HZ
增加输出 slew rate 限制
```

### 15.5 高度误差变大

虽然抗摆只加水平加速度，但倾角变大会影响垂向推力分配。

处理：

```text
降低 MC_HANG_ACC_LIM
确认 MPC_THR_XY_MARG 足够
检查 motor saturation
```

## 16. 推荐开发路线

### 阶段 1：脚本级验证

目标：

```text
快速确认符号和增益是否有效。
```

做法：

1. 复用 recorder 中的 `HangJointRecorder`。
2. 实时计算 `a_swing_xy`。
3. 注入到 OFFBOARD acceleration feed-forward 或等效 setpoint。
4. 对比 PID、LADRC、LADRC+anti-swing。

### 阶段 2：PX4 内部实现

目标：

```text
形成真正的控制器闭环。
```

做法：

1. 增加或复用 uORB topic 传入吊挂状态。
2. `MulticopterPositionControl` 订阅吊挂状态。
3. `PositionControl` 在 `_acc_sp.xy()` 上加入抗摆修正。
4. 增加参数和日志字段。

### 阶段 3：RBF-LADRC 扩展

目标：

```text
让 RBF 学习吊挂相关残差，而不是只学角速度残差。
```

做法：

1. RBF 输入加入吊挂角/角速度。
2. RBF 输出仍然限幅很小。
3. 先 bypass 学习记录，再开启 injection。

## 17. 当前最推荐的下一步

最推荐先实现一个最小抗摆外环：

```text
只用 theta_dot
只修正水平加速度
限幅 0.6 m/s^2
低通 4 Hz
```

控制律：

```text
a_as_xy = sat(L * K_rate * theta_dot_xy, 0.6)
```

初始参数：

```text
L = 0.60
K_rate = 1.5
K_theta = 0
```

用它先跑：

```text
0.8 Hz, 1 N west, hold:16
```

如果摆速 RMS 能显著下降，再加入角度项。这样风险最低，实验结果也最容易解释。

## 18. 总结

当前 LADRC 不如 PID 的核心原因不是内环没有抗扰能力，而是它没有吊挂状态意识。吊挂负载是一个低频欠驱动系统，不能简单当作刚体扰动处理。

抗摆外环应该做在位置/加速度层：

```text
用吊挂摆角和摆角速度，修正水平加速度指令。
```

推荐先做小而稳的版本：

```text
a_cmd_xy = a_pos_xy + sat(L*K_rate*theta_dot_xy, 0.6)
```

等这个版本证明能降低摆速，再加入摆角反馈和 RBF-LADRC 扩展。
