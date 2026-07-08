# ZD680_HANG 当前阶段总结、问题诊断与后续方案

生成日期：2026-07-06

## 1. 总体结论

目前 `ZD680_HANG` 吊挂负载模型、Gazebo 摆角桥接、PX4 内部抗摆外环、PID/LADRC/RBF-LADRC 对比实验链路已经基本打通。当前系统已经不是单纯的离线数据分析，而是形成了一个可以在 PX4 位置控制链路中实时使用吊挂摆角反馈的闭环实验平台。

但是从最近几组实验结果看，当前抗摆外环还没有达到最终论文级效果。它在一些工况下能改善飞机位置误差，但对吊挂摆角的抑制还不稳定，尤其在 `0.8 Hz` 扰动下出现了“位置误差变小、吊挂摆角变大”的现象。因此现在的判断是：

```text
代码架构方向是对的；
实验平台方向是对的；
问题主要在当前抗摆控制律太简单；
下一步应从单纯调参转向改控制律、加内部日志、完善实验矩阵。
```

如果目标是形成北大核心级别论文，目前材料还不够。当前结果可以作为很好的前期基础，但还需要补充明确的算法创新、完整对比实验、消融实验、频率/绳长/载荷鲁棒性实验，以及对抗摆机理的定量证明。

## 2. 当前代码架构

### 2.1 ZD680_HANG 吊挂模型

当前吊挂实验模型为：

```text
zd680_hang
```

Gazebo 运行实例通常为：

```text
zd680_hang_0
```

主要吊挂结构为：

```text
base_link
  |
  |-- hang_roll_joint
      |
      hang_gimbal_link
        |
        |-- hang_pitch_joint
            |
            hang_rope_link
              |
              |-- hang_payload_joint
                  |
                  hang_payload_link
```

当前默认物理参数：

| 项目 | 数值 |
|---|---:|
| 负载质量 | `0.50 kg` |
| 绳长 | `0.60 m` |
| 吊挂摆角记录 | `hang_roll_joint`、`hang_pitch_joint` |
| Gazebo joint topic | `/world/default/model/zd680_hang_0/joint_state` |

小角度单摆固有频率为：

```text
omega_n = sqrt(g / L)
f_n     = omega_n / (2*pi)
```

代入 `L = 0.60 m`：

```text
omega_n = sqrt(9.81 / 0.60) = 4.04 rad/s
f_n     = 4.04 / (2*pi) = 0.643 Hz
```

所以 `0.6 Hz` 扰动几乎位于吊挂固有频率附近，`0.8 Hz` 也仍然处在比较敏感的频带内。

### 2.2 Gazebo 到 PX4 的吊挂摆角桥接

新增桥接文件：

```text
simulation/gz_bridge/GZSuspendedLoadJointBridge.cpp
simulation/gz_bridge/GZSuspendedLoadJointBridge.hpp
```

桥接逻辑：

```text
Gazebo joint_state
    |
    v
GZSuspendedLoadJointBridge
    |
    v
uORB debug_array, id = 680, name = hangjoint
```

桥接订阅的 Gazebo topic：

```text
/world/default/model/<model_name>/joint_state
```

对 `zd680_hang_0` 来说就是：

```text
/world/default/model/zd680_hang_0/joint_state
```

发布到 PX4 内部的 `debug_array` 数据格式定义在：

```text
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadJointStateDebugArray.hpp
```

当前数据约定：

| `debug_array.data[]` | 含义 |
|---:|---|
| `data[0]` | `hang_roll_joint` angle |
| `data[1]` | `hang_pitch_joint` angle |
| `data[2]` | `hang_roll_joint` angular rate |
| `data[3]` | `hang_pitch_joint` angular rate |

标识：

```text
id   = 680
name = hangjoint
```

这个桥接方案的优点是没有新增 uORB 消息类型，改动范围较小，适合快速验证。缺点是 `debug_array` 语义较弱，后续如果要做更正式的工程化实现，可以考虑新增专门的 `suspended_load_state` uORB 消息。

### 2.3 PX4 位置控制中的抗摆外环

新增抗摆模块：

```text
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.cpp
mc_pos_control/SuspendedLoadAntiSwing/SuspendedLoadAntiSwing.hpp
mc_pos_control/SuspendedLoadAntiSwing/CMakeLists.txt
```

PX4 位置控制接入点：

```text
mc_pos_control/PositionControl/PositionControl.cpp
```

当前接入位置在 `PositionControl::_velocityControl()` 中：

```cpp
Vector3f vel_error = _vel_sp - _vel;
Vector3f acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);

ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

const Vector2f anti_swing_acceleration =
	_suspended_load_anti_swing.update(dt, hrt_absolute_time(), _yaw, _suspended_load_anti_swing_flying);
_acc_sp.xy() += anti_swing_acceleration;

_accelerationControl();
```

也就是说，抗摆外环不直接修改姿态环和角速度环，而是在原始位置控制器生成水平加速度指令后，额外叠加一个水平抗摆加速度：

```text
a_cmd_xy = a_pos_xy + a_hang_xy
```

这套结构有几个重要优点：

1. 原 PX4 位置外环仍然保留。
2. PID、LADRC、RBF-LADRC 内环都可以共用同一个抗摆外环。
3. 抗摆功能可以通过参数关闭，关闭后应退化为原始 PX4 行为。
4. 抗摆输出作用在加速度层，物理意义清楚。

### 2.4 当前抗摆控制律

当前代码将 Gazebo 关节角映射到机体系水平摆角：

```text
theta_body_x = sign_x * hang_pitch_joint
theta_body_y = sign_y * hang_roll_joint
```

角速度同理：

```text
theta_dot_body_x = sign_x * hang_pitch_joint_velocity
theta_dot_body_y = sign_y * hang_roll_joint_velocity
```

当前抗摆加速度在机体系下为：

```text
a_hang_body = L * (K_theta * theta_body + K_rate * theta_dot_body)
```

再根据当前 yaw 旋转到 NED 水平面：

```text
a_hang_north = cos(yaw) * a_body_x - sin(yaw) * a_body_y
a_hang_east  = sin(yaw) * a_body_x + cos(yaw) * a_body_y
```

之后经过限幅和斜率限制：

```text
|a_hang_xy| <= MC_HANG_ACC_LIM
|d a_hang_xy / dt| <= MC_HANG_ACC_SLW
```

当前主要是阻尼型控制：

```text
K_theta = 0
K_rate  > 0
```

这意味着当前抗摆器主要根据吊挂角速度给飞机一个水平加速度修正。它在小摆角、小扰动时有机会消耗摆动能量，但在接近固有频率的大周期扰动下，如果相位不准，也可能反过来给吊挂系统补能。

### 2.5 参数开关和保护机制

新增参数文件：

```text
mc_pos_control/suspended_load_anti_swing_params.c
```

当前主要参数如下：

| 参数 | 作用 | 当前默认/常用值 |
|---|---|---:|
| `MC_HANG_AS_EN` | 抗摆外环总开关 | `0` |
| `MC_HANG_OFFB` | 是否仅 OFFBOARD 允许抗摆 | `1` |
| `MC_HANG_LEN` | 绳长 | `0.60` |
| `MC_HANG_K_ANG` | 摆角反馈增益 | `0` |
| `MC_HANG_K_RATE` | 摆角速度阻尼增益 | 常用 `0.6` |
| `MC_HANG_ACC_LIM` | 抗摆加速度限幅 | 常用 `0.25` |
| `MC_HANG_ACC_SLW` | 抗摆加速度斜率限制 | 常用 `1.0` |
| `MC_HANG_LPF_HZ` | 摆角/摆速低通截止频率 | `4.0` |
| `MC_HANG_SIGN_X` | body X 方向符号 | 当前建议 `-1` |
| `MC_HANG_SIGN_Y` | body Y 方向符号 | 当前建议 `-1` |
| `MC_HANG_TIMEOUT` | 摆角测量超时 | `0.2 s` |
| `MC_HANG_ACT_DLY` | 起飞后延时激活 | 常用 `8 s` |
| `MC_HANG_ACT_ANG` | 激活前最大允许摆角 | `0.05 rad` |
| `MC_HANG_ACT_R` | 激活前最大允许摆速 | `0.08 rad/s` |
| `MC_HANG_ACT_T` | 稳定保持时间 | `2 s` |
| `MC_HANG_RAMP_T` | 激活后渐入时间 | 常用 `5 s` |
| `MC_HANG_SAFE_A` | 安全退出摆角 | 测过 `0.12`、`0.35` |

`MC_HANG_OFFB=1` 是一个很重要的工程保护。它保证即使上一次实验忘记关闭 `MC_HANG_AS_EN`，普通 QGC 起飞、悬停、Position 模式也不会被抗摆外环影响。

## 3. 日志链路总结

### 3.1 已确认的日志来源

Gazebo 中已能看到吊挂关节 topic：

```text
/world/default/model/zd680_hang_0/joint_state
```

PX4 内部通过 `debug_array` 接收吊挂状态：

```text
debug_array id = 680
name = hangjoint
```

飞行记录脚本当前也能保存：

```text
hang_joint_samples.csv
summary.csv
metadata.json
events.csv
position_offboard.ulg
```

### 3.2 当前日志中发现的关键边界

最近几组 0.8 Hz 抗摆对比实验使用了：

```text
--model-name zd680_hang_0
--link-name base_link
--wrench-schedule 'sine:20:12:3:west:0:none:0.8'
```

metadata 显示扰动实体为：

```text
zd680_hang_0::base_link
```

因此，这些实验主要代表：

```text
扰动作用在飞机机体 base_link，
机体运动再通过吊挂系统激发负载摆动。
```

它不是：

```text
扰动直接作用在 hang_payload_link 负载上。
```

这个结论很重要。当前数据仍然有效，但它回答的是“机体受扰时抗摆外环表现如何”。如果论文要强调吊挂负载抗扰，还必须补充一组扰动直接作用在 `hang_payload_link` 上的实验。

建议后续明确区分两类扰动：

```text
机体扰动：zd680_hang_0::base_link
负载扰动：zd680_hang_0::hang_payload_link
```

## 4. 关键实验结果总结

### 4.1 无抗摆基准：095740

实验条件：

```text
rate_controller = ladrc
MC_HANG_AS_EN = 0
扰动 = 0.8 Hz, 3 N, west, 12 s
扰动实体 = zd680_hang_0::base_link
```

主要结果：

| 阶段 | XY RMS | XY Peak | 摆角 RMS | 摆角 Peak |
|---|---:|---:|---:|---:|
| 扰动前 hold | `0.0293 m` | `0.0606 m` | `0.38 deg` | `0.80 deg` |
| 扰动期间 | `0.3545 m` | `0.6934 m` | `7.74 deg` | `17.62 deg` |
| 扰动后 hold | `0.3166 m` | `0.6934 m` | `7.13 deg` | `13.60 deg` |
| 最后 5 s | - | - | `4.52 deg` | `7.62 deg` |

这组是目前最重要的 0.8 Hz 无抗摆基准。

### 4.2 错误符号/不稳定抗摆：100047、100354

`100047`：

```text
SIGN_X = 1
SIGN_Y = 1
```

扰动前摆角 RMS 达到约 `34.90 deg`，说明抗摆外环在没有扰动前已经自激，符号或相位明显错误。

`100354`：

```text
SIGN_X = 1
SIGN_Y = -1
```

扰动前摆角 RMS 仍然约 `18.87 deg`，相比 `100047` 有改善，但仍然不正常。

结论：

```text
SIGN_Y = -1 方向基本正确；
SIGN_X = 1 仍可能导致 body X 方向补能；
后续采用 SIGN_X = -1, SIGN_Y = -1。
```

### 4.3 无扰动安全验证：113305

实验条件：

```text
MC_HANG_AS_EN = 1
SIGN_X = -1
SIGN_Y = -1
K_RATE = 0.6
ACC_LIM = 0.25
ACC_SLW = 1.0
ACT_DLY = 8
RAMP_T = 5
无扰动
```

结果：

| 指标 | 数值 |
|---|---:|
| action hold 摆角 RMS | `0.152 deg` |
| action hold 摆角 Peak | `0.339 deg` |
| action hold XY RMS | `0.0249 m` |

结论：

```text
启动保护、符号选择和小增益组合可以避免无扰动自激。
```

这说明当前架构本身不是必然不稳定，问题主要出现在扰动频带和抗摆控制律的相位/能量特性上。

### 4.4 SAFE_A = 0.12：113923

实验条件：

```text
MC_HANG_AS_EN = 1
SIGN_X = -1
SIGN_Y = -1
K_RATE = 0.6
ACC_LIM = 0.25
SAFE_A = 0.12 rad
扰动 = 0.8 Hz, 3 N, west, 12 s
扰动实体 = zd680_hang_0::base_link
```

位置结果：

| 阶段 | 无抗摆 095740 | 抗摆 113923 | 变化 |
|---|---:|---:|---:|
| 扰动期间 XY RMS | `0.3545 m` | `0.2784 m` | 改善约 `21.5%` |
| 扰动期间 XY Peak | `0.6934 m` | `0.6006 m` | 改善约 `13.4%` |
| 扰动后 XY RMS | `0.3166 m` | `0.2778 m` | 改善约 `12.3%` |

吊挂结果：

| 阶段 | 无抗摆 095740 | 抗摆 113923 |
|---|---:|---:|
| 扰动前摆角 RMS | `0.38 deg` | `0.17 deg` |
| 扰动期间摆角 RMS | `7.74 deg` | `6.03 deg` |
| 扰动期间摆角 Peak | `17.62 deg` | `17.37 deg` |
| 扰动后摆角 RMS | `7.13 deg` | `8.70 deg` |
| 最后 5 s 摆角 RMS | `4.52 deg` | `7.47 deg` |

结论：

```text
SAFE_A = 0.12 能降低扰动期间摆角 RMS，
但可能太保守，摆角超过约 6.9 deg 后抗摆外环退出，
导致扰动后恢复不如无抗摆。
```

### 4.5 SAFE_A = 0.35：114816

实验条件：

```text
MC_HANG_AS_EN = 1
SIGN_X = -1
SIGN_Y = -1
K_RATE = 0.6
ACC_LIM = 0.25
SAFE_A = 0.35 rad
扰动 = 0.8 Hz, 3 N, west, 12 s
扰动实体 = zd680_hang_0::base_link
```

位置结果：

| 阶段 | 无抗摆 095740 | 抗摆 114816 | 变化 |
|---|---:|---:|---:|
| 扰动前 XY RMS | `0.0293 m` | `0.0259 m` | 改善约 `11.6%` |
| 扰动期间 XY RMS | `0.3545 m` | `0.2899 m` | 改善约 `18.2%` |
| 扰动期间 XY Peak | `0.6934 m` | `0.5482 m` | 改善约 `20.9%` |
| 扰动后 XY RMS | `0.3166 m` | `0.2664 m` | 改善约 `15.9%` |

吊挂结果：

| 阶段 | 无抗摆 095740 | 抗摆 113923 | 抗摆 114816 |
|---|---:|---:|---:|
| 扰动前摆角 RMS | `0.38 deg` | `0.17 deg` | `0.17 deg` |
| 扰动期间摆角 RMS | `7.74 deg` | `6.03 deg` | `9.52 deg` |
| 扰动期间摆角 Peak | `17.62 deg` | `17.37 deg` | `21.77 deg` |
| 扰动后摆角 RMS | `7.13 deg` | `8.70 deg` | `7.85 deg` |
| 最后 5 s 摆角 RMS | `4.52 deg` | `7.47 deg` | `5.33 deg` |

结论：

```text
SAFE_A = 0.35 没有扰动前自激，启动安全性是好的；
它让抗摆外环在更大摆角下继续工作，扰动后恢复比 SAFE_A = 0.12 好；
但扰动期间摆角 RMS 和 Peak 均大于无抗摆基准，说明大摆角时继续输出可能在补能。
```

这组暴露了当前控制律的核心问题：

```text
抗摆外环可以改善机体位置误差，
但不一定真正降低吊挂负载摆角。
```

## 5. 当前存在的主要问题

### 5.1 当前控制律太简单

当前抗摆律本质上是：

```text
a_hang = L * K_rate * theta_dot
```

它没有显式判断吊挂能量是否增加，也没有针对 `0.6 m` 绳长的固有频率做相位补偿或频率整形。在接近 `0.643 Hz` 固有频率的扰动下，这种简单阻尼可能因为延迟、滤波、内环响应和坐标映射相位误差而变成补能。

### 5.2 位置改善和抗摆改善没有统一

目前最明显的问题是：

```text
114816 中 XY RMS 改善约 18.2%，
但扰动期间吊挂摆角 RMS 从 7.74 deg 增加到 9.52 deg。
```

论文和工程目标应该优先保证吊挂负载摆角下降，而不是只看飞机位置误差。否则抗摆外环可能只是让飞机更努力地抗扰，反而把吊挂摆得更大。

### 5.3 SAFE_A 是硬切换，容易两难

目前安全角逻辑是：

```text
摆角超过 MC_HANG_SAFE_A -> reset / 退出 / 等待重新激活
```

实验显示：

```text
SAFE_A = 0.12 rad：扰动期间较稳，但扰动后恢复差；
SAFE_A = 0.35 rad：扰动后恢复改善，但扰动期间摆角被放大。
```

这说明硬阈值不是最终方案。更合理的方式应是随摆角逐渐降低抗摆输出，而不是突然退出或一直工作。

### 5.4 LADRC 对吊挂系统可能过于积极

LADRC 的优势是快速估计并补偿扰动，但吊挂负载是欠驱动柔性系统。机体本体补偿越快，吊点加速度越明显，越可能激励吊挂摆动。

因此 LADRC 在普通多旋翼上比 PID 更强，不代表在吊挂负载上必然更好。当前数据已经多次显示 PID 在摆角衰减上反而更稳，这不是反常现象，而是吊挂系统和激进内环之间的耦合结果。

### 5.5 RBF-LADRC 当前还没有真正利用吊挂状态

目前 RBF-LADRC 的 RBF 残差补偿主要还是角速度内环残差学习，并没有直接把 `hang_roll_joint`、`hang_pitch_joint` 作为学习输入。因此它最多是学习机体角速度层面的残差，不能保证学到吊挂抗摆策略。

如果后续要把 RBF-LADRC 做成论文贡献，建议让 RBF 至少显式接触以下状态：

```text
theta_roll
theta_pitch
theta_dot_roll
theta_dot_pitch
a_hang_x
a_hang_y
```

或者让 RBF 学习的是抗摆外环残差，而不是只修正内环力矩。

### 5.6 抗摆外环内部可观测性不足

现在能记录：

```text
飞机位置
速度
姿态
电机
吊挂摆角
吊挂摆速
```

但还看不到：

```text
抗摆外环是否 engaged
抗摆外环是否 active
是否触发 SAFE_A 退出
ramp scale 是多少
最终输出的 a_hang_north / a_hang_east 是多少
```

这导致很多分析只能从外部结果反推，定位效率较低。下一步必须补充抗摆外环内部日志。

### 5.7 扰动目标需要更严格区分

当前 0.8 Hz 数据主要是 `base_link` 受扰。后续必须补充：

```text
base_link 扰动
hang_payload_link 扰动
base_link + hang_payload_link 组合扰动
```

否则论文中不能笼统说“吊挂负载扰动实验”，只能说“吊挂模型下机体外扰实验”。

## 6. 计划解决方案

### 6.1 第一阶段：先补内部日志

建议新增第二个 `debug_array`，专门记录抗摆外环内部状态：

```text
id   = 681
name = hangas
```

建议数据格式：

| `data[]` | 含义 |
|---:|---|
| `data[0]` | filtered body X swing angle |
| `data[1]` | filtered body Y swing angle |
| `data[2]` | filtered body X swing rate |
| `data[3]` | filtered body Y swing rate |
| `data[4]` | anti-swing acceleration north |
| `data[5]` | anti-swing acceleration east |
| `data[6]` | active |
| `data[7]` | engaged |
| `data[8]` | ramp scale |
| `data[9]` | safety scale 或 safety reset flag |

完成后，每次实验都能明确回答：

```text
抗摆到底有没有参与？
什么时候参与？
输出方向和摆速方向是否符合耗能逻辑？
是退出太早，还是参与太久？
```

### 6.2 第二阶段：把硬 SAFE_A 改成平滑衰减

建议新增两个安全角参数：

```text
MC_HANG_SAFE_LO
MC_HANG_SAFE_HI
```

控制逻辑从硬退出改成平滑缩放：

```text
angle_norm <= SAFE_LO:
    safety_scale = 1

SAFE_LO < angle_norm < SAFE_HI:
    safety_scale = smoothstep(SAFE_HI, SAFE_LO, angle_norm)

angle_norm >= SAFE_HI:
    safety_scale = 0
```

最终输出：

```text
a_hang = safety_scale * ramp_scale * a_hang_raw
```

建议初始参数：

```text
SAFE_LO = 0.18 rad 约 10.3 deg
SAFE_HI = 0.30 rad 约 17.2 deg
```

这样可以避免：

```text
0.12 rad 过早退出；
0.35 rad 大摆角继续补能。
```

### 6.3 第三阶段：加入能量耗散判断

吊挂小角度能量可近似写为：

```text
E = 0.5 * L^2 * |theta_dot|^2 + 0.5 * g * L * |theta|^2
```

从控制目标看，抗摆输出应该尽量满足：

```text
dE/dt < 0
```

工程上可以先做一个简单判据：如果当前抗摆加速度方向预计会增加摆动能量，就降低或禁止输出。

示意逻辑：

```text
raw = L * (K_theta * theta + K_rate * theta_dot)

if energy_injection_indicator(raw, theta, theta_dot) > 0:
    raw *= energy_reduction_scale
```

这个阶段的目标不是一开始就做复杂最优控制，而是避免当前简单 D 项在某些相位下补能。

### 6.4 第四阶段：加入频率感知

由于绳长可变，吊挂固有频率也会变化：

```text
f_n = 1 / (2*pi) * sqrt(g / L)
```

因此可以根据 `MC_HANG_LEN` 自动计算敏感频带：

```text
f_n(L)
```

然后对水平加速度指令或抗摆输出做频率整形：

```text
在 f_n 附近降低机体加速度激励；
或者对位置外环输出加入 notch / input shaping；
或者让抗摆外环避开错误相位输出。
```

这比固定调 `K_RATE` 更有理论说服力，也更适合论文表达。

### 6.5 第五阶段：RBF 从内环残差升级到抗摆残差

如果要保留 RBF-LADRC 作为主要创新点，建议将 RBF 的角色调整为：

```text
LADRC 保证机体角速度内环稳定；
抗摆外环根据吊挂状态给出基础抗摆加速度；
RBF 学习抗摆外环残差或参数修正。
```

可选结构：

```text
a_hang = a_passive(theta, theta_dot)
       + a_rbf(theta, theta_dot, vel_error, acc_sp)
```

同时必须加约束：

```text
大摆角冻结学习
输出限幅
权重泄漏
projection
能量增加时禁止学习或降低学习率
```

否则 RBF 可能为了减小位置误差而学习到会放大吊挂摆动的补偿。

## 7. 推荐实验规划

### 7.1 实验目标

后续实验不应只比较位置误差，而应同时比较：

```text
飞机位置保持能力
吊挂摆角抑制能力
扰动后摆动衰减能力
控制输出平滑性
电机/姿态是否饱和
不同频率、绳长、载荷下的鲁棒性
```

核心评价指标：

| 指标 | 含义 |
|---|---|
| `XY RMS` | 飞机水平位置误差 |
| `XY Peak` | 飞机最大水平偏差 |
| `swing RMS` | 吊挂摆角均方根 |
| `swing Peak` | 最大摆角 |
| `swing rate RMS` | 吊挂摆速 |
| `last 5 s swing RMS` | 扰动后恢复能力 |
| `settling time` | 摆角回到阈值内的时间 |
| `motor saturation ratio` | 电机是否接近饱和 |
| `attitude RMS/Peak` | 机体姿态激烈程度 |

### 7.2 第一批实验：当前控制律收尾验证

目的：确认简单阻尼型抗摆是否还有调参空间。

固定：

```text
SIGN_X = -1
SIGN_Y = -1
K_RATE = 0.6
ACC_LIM = 0.25
ACC_SLW = 1.0
ACT_DLY = 8
RAMP_T = 5
```

只扫：

```text
SAFE_A = 0.18, 0.22, 0.26
```

扰动：

```text
0.8 Hz, 3 N, west, 12 s
```

判断标准：

```text
扰动前摆角 RMS < 1 deg
扰动期间摆角 RMS < 7.74 deg
扰动期间摆角 Peak < 17.62 deg
最后 5 s 摆角 RMS <= 4.52 deg
XY RMS 不显著劣于无抗摆
```

如果这三个 `SAFE_A` 都不能同时改善位置和摆角，就停止继续调简单 D 阻尼，进入控制律升级。

### 7.3 第二批实验：扰动目标区分

每种控制器都要分别测试：

```text
机体扰动：zd680_hang_0::base_link
负载扰动：zd680_hang_0::hang_payload_link
```

建议命令中显式写 `--entity`，避免歧义。

机体扰动：

```bash
--model-name zd680_hang_0 \
--link-name base_link \
--entity 'zd680_hang_0::base_link' \
--entity-type LINK
```

负载扰动：

```bash
--model-name zd680_hang_0 \
--link-name hang_payload_link \
--entity 'zd680_hang_0::hang_payload_link' \
--entity-type LINK
```

论文中应分别报告两类结果。

### 7.4 第三批实验：控制器横向对比

控制器组：

```text
PID
LADRC
RBF-LADRC
PID + anti-swing
LADRC + anti-swing
RBF-LADRC + anti-swing
```

每组使用同一轨迹、同一扰动、同一初始高度和同一记录脚本。

建议频率：

| 频率 | 意义 |
|---:|---|
| `0.35 Hz` | 低于固有频率，验证普通低频扰动 |
| `0.50 Hz` | 接近固有频率下侧 |
| `0.64 Hz` | 接近 `0.6 m` 绳长固有频率 |
| `0.80 Hz` | 固有频率上侧敏感区 |
| `1.00 Hz` | 高频扰动 |

### 7.5 第四批实验：绳长和负载质量鲁棒性

绳长建议：

```text
L = 0.4 m, 0.6 m, 0.8 m
```

对应固有频率：

| 绳长 | 固有频率 |
|---:|---:|
| `0.4 m` | 约 `0.79 Hz` |
| `0.6 m` | 约 `0.64 Hz` |
| `0.8 m` | 约 `0.56 Hz` |

负载质量建议：

```text
m = 0.3 kg, 0.5 kg, 0.8 kg
```

注意：单摆小角度固有频率与质量无关，但负载质量会改变飞控补偿、推力裕度和扰动力造成的静态偏角。

### 7.6 第五批实验：统计重复性

每个关键工况至少重复：

```text
3 次，最好 5 次
```

报告：

```text
mean
standard deviation
best / worst
```

北大核心论文通常不能只给单次曲线，至少要有重复性或多工况覆盖来增强说服力。

## 8. 当前方案可行性分析

### 8.1 工程可行性

工程上是可行的，原因如下：

1. 已经完成 Gazebo joint state 到 PX4 内部的实时桥接。
2. 抗摆外环已经成功接入 PX4 位置控制加速度层。
3. 参数开关已经实现，关闭后可回到原始 PX4 外环。
4. `MC_HANG_OFFB=1` 能防止普通 QGC 起飞被抗摆影响。
5. 无扰动测试 `113305` 已证明当前结构不会必然自激。
6. `113923` 已证明在某些参数下吊挂摆角 RMS 可以降低。

因此，继续沿这个方向做是合理的。

### 8.2 算法可行性

算法上当前版本还偏初级。简单的 `theta_dot` 阻尼项可以作为 baseline，但不适合作为最终核心创新。

要形成更强方案，建议最终算法至少包含：

```text
吊挂状态反馈
平滑安全缩放
能量耗散判断
绳长自适应固有频率计算
频率整形或输入整形
可选 RBF 残差补偿
```

这样可以从“经验调参”上升到“有动力学依据的抗摆控制”。

### 8.3 实验可行性

实验平台已经可行，但实验设计需要更严格：

1. 明确扰动目标是机体还是负载。
2. 每组参数必须保存 metadata。
3. 必须同时比较位置误差和吊挂摆角。
4. 不同控制器必须使用完全相同轨迹和扰动。
5. 每组关键实验要重复。
6. 需要增加抗摆外环内部状态日志。

这些补齐后，实验部分可以支撑比较完整的论文。

## 9. 是否足够北大核心

### 9.1 当前状态

以目前结果来看，还不够稳妥。

主要原因：

1. 当前抗摆外环还没有稳定优于无抗摆基准。
2. 最近 `114816` 出现位置误差改善但吊挂摆角变大的现象。
3. 当前控制律主要是简单 D 阻尼，理论创新不足。
4. 0.8 Hz 实验扰动目标是 `base_link`，还不能完全代表负载直接扰动。
5. 实验重复次数不足，统计性不足。
6. RBF-LADRC 还没有直接利用吊挂摆角状态，创新链条不够闭合。

因此，如果现在直接写成论文，容易被质疑：

```text
为什么 LADRC/RBF 没明显优于 PID？
抗摆外环到底降低了负载摆角还是只降低了机体位置误差？
扰动是否真正作用在吊挂负载上？
算法创新是否只是工程加了一个反馈项？
结果是否具有重复性？
```

### 9.2 达到北大核心的可能方向

如果按下面方向补强，是有希望形成北大核心级别工作的：

```text
题目方向：
面向吊挂负载多旋翼的频率感知 LADRC-RBF 抗摆控制方法
```

建议论文贡献点：

1. 构建 `ZD680_HANG` 吊挂负载仿真平台，支持质量、绳长、扰动目标可配置。
2. 建立 Gazebo 吊挂关节状态到 PX4 控制器的实时桥接方法。
3. 提出附加式抗摆外环，不破坏原 PX4 位置外环，可通过参数开关启停。
4. 基于绳长计算固有频率，设计频率感知的抗摆/输入整形策略。
5. 引入能量耗散约束，避免简单阻尼项在共振附近补能。
6. 将 RBF 用于抗摆残差补偿或参数自适应，而不是只补偿内环力矩。
7. 在 PID、LADRC、RBF-LADRC、多频率、多绳长、多扰动目标下进行系统对比。

这个版本的创新链条会更完整：

```text
模型构建 -> 状态桥接 -> 抗摆外环 -> 频率/能量约束 -> RBF 自适应 -> 多工况验证
```

### 9.3 如果只做仿真，风险在哪里

如果没有实机实验，只做 SITL/Gazebo，也不是完全不能投，但要求会更高：

1. 理论推导要更完整。
2. 工况覆盖要更充分。
3. 和 baseline 的差距要明显。
4. 消融实验要清楚证明每个模块有效。
5. 要说明模型参数和扰动设置的物理合理性。

如果能补充 HIL 或小规模实机悬挂实验，说服力会明显提高。

## 10. 建议的近期执行顺序

### Step 1：完成当前简单阻尼的最后一轮确认

跑一组中间安全角：

```text
MC_HANG_SAFE_A = 0.22
```

其他保持：

```text
MC_HANG_K_RATE = 0.6
MC_HANG_ACC_LIM = 0.25
MC_HANG_ACC_SLW = 1.0
MC_HANG_SIGN_X = -1
MC_HANG_SIGN_Y = -1
MC_HANG_ACT_DLY = 8
MC_HANG_RAMP_T = 5
```

目的：

```text
判断 0.12 和 0.35 中间是否存在可用窗口。
```

### Step 2：新增抗摆外环内部日志

实现 `debug_array id=681`，记录：

```text
filtered theta
filtered theta_dot
anti-swing acceleration
active / engaged
ramp scale
safety scale
```

这是后续所有调试的基础。

### Step 3：实现平滑安全缩放

将 `MC_HANG_SAFE_A` 的硬退出改成：

```text
SAFE_LO / SAFE_HI 平滑衰减
```

优先验证：

```text
SAFE_LO = 0.18
SAFE_HI = 0.30
```

### Step 4：加入能量耗散约束

先做轻量级版本：

```text
当抗摆输出可能增加摆动能量时，降低输出比例。
```

目标是解决 `114816` 中大摆角时继续补能的问题。

### Step 5：做严格对比实验

至少完成：

```text
PID
LADRC
RBF-LADRC
PID + anti-swing
LADRC + anti-swing
RBF-LADRC + anti-swing
```

频率至少包含：

```text
0.35 Hz
0.64 Hz
0.8 Hz
```

扰动目标至少包含：

```text
base_link
hang_payload_link
```

每组重复至少 3 次。

## 11. 当前阶段一句话判断

当前方向不是错了，而是已经从“搭平台、调参数”进入了“必须改控制律”的阶段。

下一步最关键的不是继续盲目调大 LADRC、RBF 或抗摆增益，而是：

```text
先让抗摆外环可观测，
再让抗摆输出具备平滑安全缩放和能量耗散特性，
最后用严格实验矩阵证明它在位置误差和吊挂摆角上同时优于 baseline。
```

做到这一步后，这个方向才真正具备整理成北大核心论文的基础。
