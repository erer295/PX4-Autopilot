# ZD680_HANG 第四阶段：位置恢复权限保护与输出 TD 决策方案（发给 CODEX）

生成日期：2026-07-12  
依据：`ZD680_HANG_第三阶段_ACTIVE自动测试阶段性总结_2026-07-12.md`  
目标分支：`rbf-ladrc-work`

---

## 1. 第三阶段的核心结论

当前方案为：

```text
二阶位置 LADRC（XY）
+ PX4 原生 Z-PID
+ 能量抗摆
+ 绳长相关频率约束
+ ACTIVE 命令级能量协调
+ 总水平加速度包络
```

第三阶段已经证明：

1. ACTIVE 已正确进入最终水平加速度链路；
2. FAST-2 可将命令正功率积分降低约 25%～27%；
3. 在严格有效组中，FAST-2 相对 PID：
   - 摆角 RMS 改善约 50%；
   - 摆角峰值改善约 33.6%；
   - 摆动能量积分改善约 45.9%；
   - 高度性能基本相同；
4. FAST-2 的主要代价不是外扰期间位置失控，而是外扰结束后的水平位置恢复较慢：
   - 完整恢复窗位置 RMS 比 PID 高约 29.7%；
   - 位置峰值比 PID 高约 40.7%；
5. FAST-2 没有持续总包络触发，也没有高频振荡或明显执行相位抵消；
6. 当前最大实验问题是外扰开始相位不一致，动态初态门控已经实现但需要正式验证。

因此当前主要矛盾已经从：

```text
LADRC 和抗摆互相注能
```

转变为：

```text
ACTIVE 成功减摆，但在恢复阶段占用了过多位置恢复权限。
```

---

## 2. 是否现在加入输出 TD

### 2.1 当前判断

本阶段不建议立即把输出 TD 加入闭环。

原因：

1. FAST-2 已经显著降低摆角和摆动能量，说明控制输出“过快过急”不再是当前最主要的问题；
2. 当前主要缺点是位置恢复偏慢，而输出 TD 会进一步限制 LADRC 加速度变化，可能让位置恢复更慢；
3. 当前有效 FAST-2 没有总包络持续触顶、没有高频抖动，也没有证据表明命令到真实加速度的执行相位抵消；
4. 传统参考 TD 对固定悬停设定值下的吊载外扰作用有限；
5. 输出 TD 只有在确认 LADRC 基础加速度 jerk、P95 或摆频附近频谱仍明显高于 PID 时才值得加入。

### 2.2 本阶段对 TD 的处理

本阶段只增加 TD 决策所需的日志和离线统计，不改变闭环：

```text
a_base_raw_x/y
a_base_jerk_x/y
a_final_jerk_x/y
base_accel_RMS/P95/peak
base_jerk_RMS/P95/peak
final_jerk_RMS/P95/peak
0.4～0.9 Hz 频带能量
```

只有出现以下证据，下一阶段才增加输出 TD：

```text
FAST-2 的 base acceleration P95 比 PID 高 >= 25%
或 jerk RMS/峰值比 PID 高 >= 30%
或 0.4～0.9 Hz 的基础控制频带能量明显高于 PID
且位置问题表现为过冲/反复回摆，而不是单纯恢复偏慢
```

---

## 3. 第四阶段的核心改动：位置恢复权限保护

### 3.1 通俗解释

ACTIVE 能量协调像一个“减摆刹车”。

FAST-2 说明刹车很有效，但有时刹车会抵消 LADRC 为恢复位置所做的动作，使无人机回到目标点太慢。

本阶段不是减小整个 FAST-2，而是增加一个规则：

> ACTIVE 可以减少激摆动作，但不能把 LADRC 朝目标位置的恢复加速度取消太多。

即：

- 与位置恢复方向无关的 ACTIVE 修正尽量保留；
- 如果 ACTIVE 修正正好反对位置恢复方向，则限制它最多取消一部分恢复加速度；
- 保证无人机始终保留足够的位置恢复控制权。

### 3.2 数学定义

定义水平位置误差：

\[
\boldsymbol e_p
=
\boldsymbol p_{sp}
-
\boldsymbol p.
\]

当位置误差不为零时：

\[
\hat{\boldsymbol e}_p
=
\frac{\boldsymbol e_p}
{\|\boldsymbol e_p\|+\varepsilon_p}.
\]

ACTIVE 前的候选加速度：

\[
\boldsymbol a_c
=
\boldsymbol a_{base}
+
\boldsymbol a_{AS}.
\]

现有 FAST-2 计算的原始 ACTIVE 修正：

\[
\Delta\boldsymbol a_{pas,raw}.
\]

候选命令沿位置恢复方向的有效分量：

\[
a_{recover}
=
\max
\left(
0,
\boldsymbol a_c^T\hat{\boldsymbol e}_p
\right).
\]

ACTIVE 修正沿位置恢复方向的分量：

\[
\Delta a_{\parallel}
=
\Delta\boldsymbol a_{pas,raw}^T
\hat{\boldsymbol e}_p.
\]

若：

\[
\Delta a_{\parallel}\ge0,
\]

说明 ACTIVE 没有阻碍位置恢复，不限制。

若：

\[
\Delta a_{\parallel}<0,
\]

则限制：

\[
\Delta a_{\parallel}
\ge
-\rho_p a_{recover}.
\]

初始建议：

\[
\rho_p=0.40.
\]

含义：

> ACTIVE 最多取消候选控制沿目标方向恢复加速度的 40%，至少保留 60% 的位置恢复能力。

将 ACTIVE 修正分解为：

\[
\Delta\boldsymbol a_{pas,raw}
=
\Delta a_{\parallel}\hat{\boldsymbol e}_p
+
\Delta\boldsymbol a_{\perp}.
\]

限制后：

\[
\Delta a_{\parallel,limited}
=
\max
\left(
\Delta a_{\parallel},
-\rho_p a_{recover}
\right).
\]

最终修正：

\[
\Delta\boldsymbol a_{pas,limited}
=
\Delta a_{\parallel,limited}
\hat{\boldsymbol e}_p
+
\Delta\boldsymbol a_{\perp}.
\]

随后继续经过原有：

```text
PAS_LIM
PAS_SLW
MC_HANG_TOT_A
```

不修改原有总包络。

### 3.3 小位置误差处理

当：

\[
\|\boldsymbol e_p\|<0.05\;m
\]

时，不启用位置权限限制，保持原 FAST-2。

原因：

- 位置误差很小时，没有必要为了极小的位置恢复牺牲减摆；
- 避免位置误差方向在零附近受噪声影响频繁变化。

### 3.4 数据无效处理

以下情况直接退化为原 FAST-2：

- 位置或设定值无效；
- 位置误差含 NaN/Inf；
- 位置误差小于 0.05 m；
- 候选加速度无效。

该模块只限制 ACTIVE 修正，不修改 LADRC、AS 或 Z 轴。

---

## 4. 控制模式

建议增加一个最少参数：

```text
MC_HANG_PAS_POS = 0：关闭位置权限保护，保持当前 FAST-2
MC_HANG_PAS_POS = 1：开启位置权限保护
```

增加：

```text
MC_HANG_PAS_PR = 0.40
```

含义：ACTIVE 最多取消 40% 的朝目标方向恢复加速度。

暂时不增加多个阈值、复杂状态机或动态优化器。

---

## 5. 强制控制链路

```text
LADRC nominal + disturbance
            ↓
a_base
            ↓
原能量抗摆
            ↓
a_candidate
            ↓
原 FAST-2 ACTIVE 修正
            ↓
位置恢复权限保护
            ↓
PAS 限幅和 slew
            ↓
总水平加速度包络
            ↓
姿态/推力设定
```

若当前代码的限幅和 slew 在原始修正内部完成，则位置权限保护应放在：

```text
原始功率投影计算之后
最终 PAS slew 和总包络之前
```

必须确保最终施加的修正和日志中的 limited 修正一致。

---

## 6. 日志要求

继续使用 `hangcoord` 或增加扩展字段，至少记录：

```text
position_error_x/y
position_error_norm
position_direction_x/y
candidate_recovery_component
pas_raw_parallel_component
pas_limited_parallel_component
pas_perpendicular_norm
pas_position_limiter_active
pas_position_limiter_ratio
delta_pas_raw_x/y
delta_pas_limited_x/y
P_candidate
P_after_raw_pas
P_after_limited_pas
P_final
```

同时增加 TD 决策日志：

```text
a_base_raw_x/y
a_base_jerk_x/y
a_final_jerk_x/y
```

自动统计：

```text
位置权限限制介入率
位置权限限制前后 PAS 修正 RMS/P95/峰值
保留的位置恢复比例
正功率积分变化
摆角 RMS/峰值
摆动能量积分
恢复到 5°/2°/1° 时间
外扰期和恢复期位置 RMS/峰值
加速度和 jerk RMS/P95/峰值
总包络触发率
```

---

## 7. 单元测试

必须覆盖：

1. `MC_HANG_PAS_POS=0` 时与当前 FAST-2 输出完全一致；
2. 位置误差小于 0.05 m 时不限制；
3. ACTIVE 修正不阻碍位置恢复时不限制；
4. ACTIVE 修正反对位置恢复且超过比例时正确限幅；
5. 限制后至少保留：
   \[
   (1-\rho_p)a_{recover}
   \]
   的候选恢复分量；
6. 与位置恢复方向垂直的 ACTIVE 修正不被错误删除；
7. NaN、无效状态和零位置误差数值稳定；
8. 最终水平加速度不突破 `MC_HANG_TOT_A`；
9. OFF、SHADOW 模式不受影响；
10. PID 路径不受影响。

---

## 8. 实验流程

### 8.1 先验证动态门控

使用已校准门槛：

```text
angle_rms <= 0.75 deg
rate_rms <= 0.06 rad/s
current_angle <= 0.75 deg
current_rate <= 0.06 rad/s
position error <= 0.10 m
horizontal speed <= 0.10 m/s
continuous >= 1.0 s
timeout = 60 s
门控通过后等待 2 s
```

确认在 60 s 内可放行，并且 wrench 只在门控通过后施加。

### 8.2 最少对比组

统一条件下运行：

```text
A：FAST-2 原方案，MC_HANG_PAS_POS=0
B：FAST-2 + 位置权限保护，MC_HANG_PAS_POS=1，PR=0.40
C：PID_FAIR
D：SHADOW_MATCHED
```

第一轮每组 1 次，只判断方向。

若 B 同时满足摆动和位置要求，再对 B、C、D 各重复 3 次。

### 8.3 统一条件

```text
L = 0.6 m
payload = 0.5 kg
0.5 N west 半正弦
持续 6 s
MC_HANG_PAS_LIM = 0.08 m/s²
MC_HANG_PAS_SLW = 1.5 m/s³
MC_HANG_TOT_A = 0.8 m/s²
其余 EKF、LADRC、AS、FRQ 参数保持第三阶段不变
```

---

## 9. 第四阶段快速通过标准

B 组相对当前 FAST-2 A 组，应优先满足：

```text
恢复窗位置 RMS 改善 >= 15%
或恢复窗位置峰值改善 >= 15%
```

同时要求：

```text
摆角 RMS 恶化 <= 10%
摆角峰值恶化 <= 10%
摆动能量积分恶化 <= 10%
正 P_final 积分仍比 candidate 下降 >= 10%
位置峰值 < 0.60 m
无 failsafe / NaN / 持续总包络触顶
```

相对 PID，目标为：

```text
摆角 RMS、峰值或能量积分继续明显优于 PID；
恢复窗位置 RMS 从当前约 +29.7% 的代价压缩到 <= +15%；
外扰期间位置 RMS 与 PID 保持接近。
```

---

## 10. 结果决策

### 情况 A：位置明显改善，减摆基本保持

保留 `MC_HANG_PAS_POS=1`、`PR=0.40`。

立即进入：

```text
提出方法 × 3
SHADOW × 3
PID × 3
纯二阶 LADRC × 3
```

然后进行绳长/质量鲁棒性试验。

### 情况 B：位置改善不足，但减摆保持

只调整一次：

```text
PR：0.40 -> 0.25
```

含义：ACTIVE 最多取消 25% 的位置恢复加速度，保留至少 75%。

不同时修改 PAS_LIM、LADRC 和 AS 参数。

### 情况 C：位置改善，但减摆明显恶化

调整一次：

```text
PR：0.40 -> 0.55
```

让 ACTIVE 获得更多减摆权限。

### 情况 D：位置和减摆都变差

检查：

- 位置误差方向定义是否正确；
- NED 坐标符号；
- 限制器接入位置；
- limited 修正是否与日志一致；
- 总包络是否二次改变修正。

修复前不增加 TD。

### 情况 E：位置权限保护后，位置仍然慢，并且 jerk/P95 明显高于 PID

这时才进入输出 TD 阶段。

推荐输出 TD 只处理：

```text
a_base = a_nominal + a_disturbance
```

不处理：

```text
能量抗摆
ACTIVE 修正
Z 轴
姿态/角速度内环
```

首轮只测试：

```text
omega_out = 3.0 rad/s
zeta_out = 1.0
jerk_limit = 1.0 m/s³
```

但输出 TD 不属于本次代码提交。

---

## 11. 对 CODEX 的通俗说明

当前 FAST-2 已经可以明显减小吊载摆动，但它在恢复阶段有时把无人机朝目标点的加速度抵消得太多，所以无人机回到目标点比较慢。

本阶段不要把整个 LADRC 变慢，也不要直接增加 TD。请给 ACTIVE 增加一个“位置恢复权限保护”：

- ACTIVE 仍然可以在错误摆动相位下修正命令；
- 但如果 ACTIVE 修正正在反对无人机回到目标点，则最多只允许它取消 40% 的恢复加速度；
- 至少保留 60% 的位置恢复能力；
- 与位置恢复方向垂直的减摆修正尽量保留；
- 小位置误差时继续使用原 FAST-2，不限制减摆。

这相当于告诉能量协调：

> 可以减摆，但不能为了减摆把回到目标点的控制权全部拿走。

输出 TD 暂时只做数据诊断。因为当前没有高频振荡或持续饱和，而主要问题是恢复太慢；若现在把 LADRC 输出再变慢，很可能让位置问题更严重。

---

## 12. 可直接发给 CODEX 的执行指令

请基于第三阶段当前工作树，实现“位置恢复权限保护”，本轮不要实现输出 TD。

具体要求：

1. 保持当前 FAST-2、能量抗摆、LADRC、频率约束和总包络不变；
2. 增加：
   - `MC_HANG_PAS_POS`
   - `MC_HANG_PAS_PR`，默认 `0.40`
3. 对现有 ACTIVE 修正进行位置方向分解；
4. 当 ACTIVE 修正反对位置恢复方向时，最多允许取消候选命令沿目标方向恢复加速度的 40%；
5. 当位置误差小于 `0.05 m` 或状态无效时，退化为原 FAST-2；
6. 保留 PAS 限幅、slew 和总水平包络；
7. OFF、SHADOW、PID 路径必须不变；
8. 增加位置方向分量、限制比例、限制前后功率和 jerk 日志；
9. 增加相应单元测试；
10. 自动测试支持：
    - FAST2_BASE
    - FAST2_POS_PROTECT
    - SHADOW_MATCHED
    - PID_FAIR
11. 使用已校准的动态初态门控；
12. 第一轮每组只跑一次并输出对比报告；
13. 不实现输出 TD、Z3 陷波、执行模型、RBF、CESO、MPC/QP 或内环 LADRC。

请完成代码、单元测试、SITL 编译、运行命令和自动分析报告后停止。
