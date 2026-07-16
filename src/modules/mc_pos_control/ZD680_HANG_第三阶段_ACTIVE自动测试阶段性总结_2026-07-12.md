# ZD680_HANG 第三阶段 ACTIVE 自动测试阶段性总结

生成日期：2026-07-12  
代码分支：`rbf-ladrc-work`  
对象：ZD680 吊挂模型，绳长 0.6 m，负载质量 0.5 kg  
当前方案：二阶位置外环 LADRC（XY）＋PX4 原生 Z 轴 PID＋能量抗摆＋频率约束＋ACTIVE 命令级能量协调＋总水平加速度包络

---

## 1. 本阶段目标

本阶段不继续增加主动陷波、CESO、LESO 执行模型、RBF、TD、内环 LADRC 或 MPC/QP，而是验证以下最小闭环：

```text
二阶位置 LADRC nominal + disturbance
                ↓
        原能量抗摆 AS
                ↓
     ACTIVE 能量协调修正
                ↓
  MC_HANG_TOT_A 总水平包络
                ↓
         姿态与推力控制
```

验证重点：

1. ACTIVE 是否真实进入最终加速度命令；
2. 修正方向是否降低预测正摆动功率；
3. 修正限幅、slew 和总包络是否有效；
4. FAST-1 与 FAST-2 是否安全；
5. 相对 PID 是否能降低摆角和摆动能量；
6. 当前实验条件是否足以公平比较 ACTIVE、SHADOW 和 PID。

---

## 2. 代码与测试工具状态

### 2.1 ACTIVE 控制代码

已经实现：

```text
MC_HANG_PAS_MD=0：OFF
MC_HANG_PAS_MD=1：SHADOW
MC_HANG_PAS_MD=2：ACTIVE
```

SHADOW 与 ACTIVE 共用同一计算函数。ACTIVE 只决定是否把同一修正加入总水平加速度包络之前，OFF 和 SHADOW 不改变旧闭环。

ACTIVE 计算为：

\[
P_{candidate}=-L\boldsymbol a_{candidate}^{T}\dot{\boldsymbol\theta}_h
\]

\[
\lambda=\max\left(0,
\frac{-\boldsymbol a_{candidate}^{T}\dot{\boldsymbol\theta}_h}
{\|\dot{\boldsymbol\theta}_h\|^2+\varepsilon}\right)
\]

\[
\Delta\boldsymbol a=k_{pas}\operatorname{sat}
(\lambda\dot{\boldsymbol\theta}_h,a_{pas,max})
\]

### 2.2 自动记录与统计

`position_offboard_flight_recorder.py` 已增加：

- `ACTIVE_NOWRENCH_FAST1`；
- `ACTIVE_FAST1`；
- `ACTIVE_FAST2`；
- `SHADOW_MATCHED`；
- `PID_FAIR`；
- 参数快照、Git 状态和初始状态元数据；
- candidate/projected/final 功率统计；
- ACTIVE 修正 RMS、P95、峰值、介入率、限幅率、slew 率；
- 摆角 RMS、峰值和恢复时间；
- wrench 同步清除与有效性检查。

### 2.3 动态初态门控

固定等待 8 s 或 20 s 不能保证吊摆相位一致，因此新增动态门控。门控同时检查：

```text
最近窗口摆角 RMS
最近窗口摆速 RMS
当前摆角
当前摆速
水平位置误差
水平速度
连续满足时间
```

不满足时不会启动 wrench，超时后整组在外扰前终止。

第一轮安全性验证采用了保守配置：

```text
窗口长度：1.0 s
连续满足：1.0 s
摆角 RMS：<= 0.50 deg
摆速 RMS：<= 0.03 rad/s
当前摆角：<= 0.50 deg
当前摆速：<= 0.03 rad/s
水平位置误差：<= 0.10 m
水平速度：<= 0.10 m/s
最长等待：40 s
门控通过后固定等待：2 s
```

实际运行表明当前模型的稳态底限约为：

```text
摆角 RMS 最低约 0.59 deg
摆速 RMS 最低约 0.043 rad/s
```

因此 0.50 deg / 0.03 rad/s 低于该模型可重复达到的稳态底限，适合验证“条件不满足时禁止施加外扰”，不适合作为批量对比试验门槛。根据实测，已经把批处理默认值校准为：

```text
摆角 RMS <= 0.75 deg
摆速 RMS <= 0.06 rad/s
当前摆角 <= 0.75 deg
当前摆速 <= 0.06 rad/s
最长等待 60 s
```

动态门控逻辑及上述实用阈值均已写入 `position_offboard_flight_recorder.py`。保守门槛已经通过“超时不施加 wrench”运行验证；校准门槛已完成语法和命令行接口检查，尚需用一组正式飞行确认能在 60 s 内稳定放行。

### 2.4 编译和单元测试

结果：

```text
SuspendedLoadAntiSwing：全部通过
PositionControl：30/30 通过
PX4 SITL：编译通过
记录脚本完整 AST 解析：通过
记录脚本 --help 接口检查：通过
```

测试覆盖 OFF 零输出、SHADOW 不改变最终命令、ACTIVE 正功率修正、功率方向、NaN/无效状态、总包络和 `hangcoord` 映射。

本次门控阈值调整只修改自动试验的放行条件，没有改动 PX4 控制律、FAST-1/FAST-2 参数、外扰幅值或数据统计口径。

---

## 3. 统一试验条件

### 3.1 控制和模型参数

```text
MC_PLADRC_EN=3
MC_PLADRC_TD_EN=0
MC_LADRC_EN=0
MC_RBF_EN=0
MC_RBF_LEARN_EN=0

MC_HANG_LEN=0.6
MC_HANG_ACC_LIM=0.2
MC_HANG_TOT_A=0.8
```

ACTIVE/SHADOW 使用：

```text
MC_HANG_AS_EN=1
MC_HANG_MODE=2
MC_HANG_FRQ_EN=1
MC_HANG_PAS_E=0.005
MC_HANG_PAS_R=0.05
MC_HANG_PAS_K=1.0
MC_HANG_PAS_LPF=1.0
MC_HANG_PAS_P=0.0005
MC_HANG_PAS_DLY=0.10
```

### 3.2 外扰

```text
实体：zd680_hang_0::hang_payload_link
方向：PX4 local NED west
形式：半正弦
幅值：0.5 N
持续时间：6 s
频率：0.083333 Hz
```

### 3.3 两档 ACTIVE

```text
FAST-1：PAS_LIM=0.05 m/s²，PAS_SLW=1.0 m/s³
FAST-2：PAS_LIM=0.08 m/s²，PAS_SLW=1.5 m/s³
```

---

## 4. 最近试验及有效性分类

| 模式 | 时间标识 | 外扰前摆角 RMS | wrench 状态 | 用途 |
|---|---:|---:|---|---|
| FAST-1 无扰 | 20260711_221325 | 无外扰 | 不适用 | 安全性有效 |
| FAST-1 | 20260711_221456 | 0.991° | 有效 | 有效快速组 |
| FAST-2 | 20260712_104011 | 0.917° | 有效 | 有效快速组 |
| FAST-2 | 20260712_104609 | 1.362° | 有效 | 初态不合格，仅安全参考 |
| FAST-2 | 20260712_104800 | 1.099° | 清除报告无效 | 剔除 |
| SHADOW | 20260712_105046 | 1.045° | 有效 | 接近门槛，诊断参考 |
| FAST-2，预悬停 20 s | 20260712_105419 | 0.418° | 有效 | 严格有效 |
| SHADOW，预悬停 20 s | 20260712_105658 | 1.050° | 有效 | 略超门槛，诊断参考 |
| PID，预悬停 20 s | 20260712_105935 | 0.181° | 有效 | 严格有效 |
| SHADOW，预悬停 20 s | 20260712_110143 | 1.104° | 有效 | 初态不合格，诊断参考 |
| FAST-2，动态门控 | 20260712_111257 | 未达到 0.50°/0.03 | 未启动 | 门控安全验证有效 |

---

## 5. FAST-1 无扰检查

| 指标 | 结果 |
|---|---:|
| 水平位置误差 RMS | 0.022 m |
| 水平位置误差峰值 | 0.054 m |
| 摆角 RMS | 0.701° |
| 摆角峰值 | 1.232° |
| ACTIVE 修正 | 0 |
| 总包络触发率 | 0 |

结论：低摆动状态下 ACTIVE 不误介入，没有引起高频振荡或持续包络触发。

---

## 6. FAST-1 与 FAST-2 快速结果

| 指标 | FAST-1 | FAST-2（104011） |
|---|---:|---:|
| ACTIVE 峰值 | 0.0500 | 0.0800 m/s² |
| candidate 正功率积分 | 0.18375 | 0.11256 J/kg |
| final 正功率积分 | 0.15622 | 0.08166 J/kg |
| 本组正功率降低 | 14.98% | 27.45% |
| 摆角 RMS | 4.451° | 1.628° |
| 摆角峰值 | 13.343° | 4.887° |
| 摆动能量积分 | 1.436 | 0.704 |
| 水平位置误差 RMS | 0.285 | 0.211 m |
| 水平位置误差峰值 | 0.563 | 0.397 m |
| 总包络触发率 | 3.54% | 0 |

该组结果显示 FAST-2 更有潜力，但后续重复发现摆角峰值存在明显相位敏感性，不能只依据这一组得出优于基线的最终结论。

---

## 7. 严格 FAST-2 与 PID 对比

选取：

```text
FAST-2：20260712_105419，初始摆角 RMS=0.418°
PID：20260712_105935，初始摆角 RMS=0.181°
```

| 指标 | FAST-2 | PID | FAST-2 相对变化 |
|---|---:|---:|---:|
| 摆角 RMS | 4.140° | 8.279° | 改善 50.0% |
| 摆角峰值 | 10.929° | 16.456° | 改善 33.6% |
| 实际摆动能量积分 | 0.9005 | 1.6649 J·s/kg | 改善 45.9% |
| 实际摆动能量峰值 | 0.1449 | 0.3217 J/kg | 改善 55.0% |
| 完整恢复窗位置 RMS | 0.198 | 0.153 m | 恶化 29.7% |
| 完整恢复窗位置峰值 | 0.376 | 0.267 m | 恶化 40.7% |
| 外扰期间位置 RMS | 0.095 | 0.093 m | 接近，恶化 1.9% |
| 外扰期间位置峰值 | 0.238 | 0.200 m | 恶化 19.4% |
| 外扰期间高度 RMS | 0.0229 | 0.0237 m | 基本相同 |
| 外扰期间高度峰值 | 0.0274 | 0.0285 m | 基本相同 |

FAST-2 严格组内部：

```text
P_candidate 正积分 = 0.07587 J/kg
P_final 正积分     = 0.05676 J/kg
下降               = 25.19%
ACTIVE 峰值        = 0.08000002 m/s²
总包络触发率       = 0
```

结论：FAST-2 明显降低 PID 条件下的摆角和摆动能量，高度控制与 PID 相当；代价是外扰后的水平位置恢复更慢，完整窗口位置代价超过 20%。

---

## 8. SHADOW 结果与实验方法问题

两组延长预悬停 SHADOW 的外扰前摆角仍约为 1.05° 和 1.10°，没有严格满足统一门槛，而且响应差异很大：

```text
一组摆角峰值约 1.66°
另一组摆角峰值约 8.88°
```

同一 FAST-2 也出现：

```text
初始摆角 RMS=0.917° → 外扰峰值=4.89°
初始摆角 RMS=0.418° → 外扰峰值=10.93°
```

初始摆角 RMS 更小但峰值更大，说明固定时刻施加半正弦外扰仍受到以下因素影响：

- 外扰开始瞬间的摆角；
- 外扰开始瞬间的摆速；
- 摆动方向；
- 摆动相位；
- 摆动向 west 外扰方向的投影。

因此当前数据可以证明 ACTIVE 的功率修正机制有效，但还不能严格证明 ACTIVE 优于匹配 SHADOW。

---

## 9. 当前阶段结论

可以确认：

1. ACTIVE 代码接入位置正确；
2. OFF、SHADOW、ACTIVE 模式行为正确；
3. FAST-1 和 FAST-2 的限幅、slew 和总水平包络正常；
4. FAST-2 多个有效组均使正 `P_final` 相对 `P_candidate` 下降约 25%～27%；
5. 命令功率与推力重构功率接近，未发现明显执行相位抵消；
6. FAST-2 相对 PID 明显降低摆角和真实摆动能量；
7. 高度问题已经基本排除；
8. FAST-2 存在水平位置恢复代价和 1°～2°附近长尾；
9. 当前最大障碍是实验初始相位一致性，而不是 ACTIVE 控制方向错误。

当前判断：

> 二阶外环 LADRC＋能量抗摆＋频率约束＋ACTIVE 能量协调方案整体可行，FAST-2 是优先档位；动态初态门控的代码校准已经完成，但在取得同门槛触发的匹配 SHADOW 数据前，不能宣称综合性能优于 PID 或 SHADOW。

---

## 10. 下一步最小计划

### 10.1 不修改控制律

保持：

```text
MC_HANG_PAS_K=1.0
MC_HANG_PAS_LIM=0.08
MC_HANG_PAS_SLW=1.5
MC_HANG_TOT_A=0.8
MC_HANG_ACC_LIM=0.2
```

不继续扫描 0.06、0.07 或超过 0.08，不增加主动陷波或执行模型。

### 10.2 验证已校准的动态门控

脚本已经使用以下实测可达到值：

```text
angle_rms <= 0.75 deg
rate_rms <= 0.06 rad/s
current_angle <= 0.75 deg
current_rate <= 0.06 rad/s
continuous >= 1.0 s
timeout = 60 s
```

首组只需确认门控能在 60 s 内放行、放行前不施加 wrench、放行后固定 2 s 才开始外扰；无需继续扫描门槛。

### 10.3 只重跑三组

```text
ACTIVE_FAST2 × 1
SHADOW_MATCHED × 1
PID_FAIR × 1
```

三组均由同一动态门控触发，门控通过后固定 2 s 开始同一外扰。

### 10.4 判断规则

FAST-2 继续进入正式试验需满足：

```text
正 P_final 相对 candidate 下降 >= 10%
摆角或实际能量相对 SHADOW 改善 >= 8%
水平位置 RMS 相对 SHADOW 恶化 <= 20%
位置峰值 < 0.60 m
无 failsafe / NaN / 持续包络触顶
```

若摆动改善但位置代价仍超过 20%，优先回退 FAST-1 或降低 `PAS_K` 一次，不同时修改 LADRC、AS 和门槛参数。

---

## 11. 自动运行命令

启动：

```bash
cd ~/PX4-Autopilot
PX4_MAV_BROADCAST=1 HEADLESS=1 make px4_sitl gz_zd680_hang
```

起飞：

```text
param set MIS_TAKEOFF_ALT 10
commander takeoff
```

FAST-2：

```bash
python3 Tools/simulation/gz/tools/position_offboard_flight_recorder.py \
  --active-batch-mode ACTIVE_FAST2 \
  --world default \
  --model-name zd680_hang_0 \
  --link-name hang_payload_link \
  --hang-joint-log on \
  --no-face-north-on-entry
```

SHADOW：

```bash
python3 Tools/simulation/gz/tools/position_offboard_flight_recorder.py \
  --active-batch-mode SHADOW_MATCHED \
  --world default \
  --model-name zd680_hang_0 \
  --link-name hang_payload_link \
  --hang-joint-log on \
  --no-face-north-on-entry
```

PID：

```bash
python3 Tools/simulation/gz/tools/position_offboard_flight_recorder.py \
  --active-batch-mode PID_FAIR \
  --world default \
  --model-name zd680_hang_0 \
  --link-name hang_payload_link \
  --hang-joint-log on \
  --no-face-north-on-entry
```

---

## 12. 数据位置

自动试验根目录：

```text
~/PX4-Autopilot/build/position_offboard_flight/
```

每组保存：

```text
position_offboard.ulg
hang_joint_samples.csv
live_samples.csv
events.csv
summary.csv
hangcoord_summary.csv
active_summary.json
validity.json
metadata.json
```

本报告只使用明确通过 wrench 投递/清除检查的数据；初态超限或清除无效的组已单独标记，没有混入正式结论。
