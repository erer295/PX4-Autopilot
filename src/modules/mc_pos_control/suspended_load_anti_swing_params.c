/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * Suspended-load anti-swing outer loop enable
 *
 * When enabled and fresh suspended-load joint-state measurements are supplied
 * to the multicopter position controller, an additional horizontal
 * acceleration correction is added before attitude-setpoint generation.
 * With no valid suspended-load measurement this feature is a no-op.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_AS_EN, 0);

/**
 * Suspended-load anti-swing algorithm
 *
 * LegacyPD preserves the existing angle/rate controller. EnergyDamping uses
 * rope-length-scaled swing-rate feedback with a directly verifiable negative
 * control contribution to the pendulum energy derivative.
 *
 * 0: disabled
 * 1: legacy angle/rate PD
 * 2: energy damping
 *
 * @min 0
 * @max 2
 * @value 0 Disabled
 * @value 1 LegacyPD
 * @value 2 EnergyDamping
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_MODE, 1);

/**
 * Suspended-load anti-swing OFFBOARD-only gate
 *
 * When enabled, the optional anti-swing outer loop can only engage while the
 * vehicle is in OFFBOARD control. This keeps normal QGC takeoff, hold, and
 * position-mode flights identical to the stock PX4 outer loop even if
 * MC_HANG_AS_EN was left enabled from a previous experiment.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_OFFB, 1);

/**
 * Suspended-load rope length
 *
 * Rope length from the gimbal point to the payload center.
 *
 * @unit m
 * @min 0.05
 * @max 10.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_LEN, 0.60f);

/**
 * Suspended-load angle feedback gain
 *
 * Multiplies the filtered payload swing angle in the anti-swing acceleration
 * correction. Start at zero and increase only after rate damping is verified.
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_K_ANG, 0.0f);

/**
 * Suspended-load angular-rate damping gain
 *
 * Multiplies the filtered payload swing angular rate in the anti-swing
 * acceleration correction.
 *
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_K_RATE, 1.5f);

/**
 * Suspended-load energy damping ratio
 *
 * The energy mode rate gain is kd=2*zeta*sqrt(g*L). Start conservatively and
 * increase only after the joint-state sign is verified in a single-axis test.
 *
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ZETA, 0.25f);

/**
 * Suspended-load energy gate start
 *
 * Per-unit-mass swing energy below this value produces no energy-mode output.
 *
 * @min 0
 * @max 20
 * @decimal 3
 * @increment 0.005
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_E_MIN, 0.0f);

/**
 * Suspended-load energy gate full threshold
 *
 * Per-unit-mass swing energy at which the energy-mode output reaches full
 * authority. Values between E_MIN and E_FULL are blended linearly.
 *
 * @min 0
 * @max 20
 * @decimal 3
 * @increment 0.005
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_E_FULL, 0.02f);

/**
 * Suspended-load anti-swing acceleration limit
 *
 * Maximum norm of the horizontal anti-swing acceleration added to the position
 * controller acceleration setpoint.
 *
 * @unit m/s^2
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACC_LIM, 0.6f);

/**
 * Suspended-load anti-swing acceleration slew limit
 *
 * Maximum change rate of the horizontal anti-swing acceleration. Set to zero
 * to disable slew limiting.
 *
 * @unit m/s^3
 * @min 0
 * @max 100
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACC_SLW, 2.0f);

/**
 * Suspended-load swing filter cutoff
 *
 * First-order low-pass cutoff applied to swing angle and angular rate.
 *
 * @unit Hz
 * @min 0
 * @max 50
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_LPF_HZ, 4.0f);

/**
 * Suspended-load body-X sign
 *
 * Sign applied when mapping the Gazebo pitch joint to the controller body-X
 * swing angle. Use -1 if enabling anti-swing increases body-X swing.
 *
 * @min -1
 * @max 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_SIGN_X, 1);

/**
 * Suspended-load body-Y sign
 *
 * Sign applied when mapping the Gazebo roll joint to the controller body-Y
 * swing angle. Use -1 if enabling anti-swing increases body-Y swing.
 *
 * @min -1
 * @max 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_SIGN_Y, 1);

/**
 * Suspended-load maximum model angle
 *
 * Above this swing angle norm the small-angle position feedback part is
 * disabled and only rate damping remains active.
 *
 * @unit rad
 * @min 0
 * @max 3.14
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_MAX_ANG, 0.8f);

/**
 * Suspended-load measurement timeout
 *
 * Maximum age of a suspended-load joint-state measurement before anti-swing is
 * bypassed.
 *
 * @unit s
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_TIMEOUT, 0.2f);

/**
 * Suspended-load anti-swing activation delay
 *
 * Minimum time after takeoff flight state before the anti-swing correction may
 * engage. This prevents the optional outer loop from acting during takeoff
 * transients or immediately after reusing a still-moving suspended-load model.
 *
 * @unit s
 * @min 0
 * @max 30
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACT_DLY, 3.0f);

/**
 * Suspended-load anti-swing activation angle
 *
 * Maximum swing angle norm allowed before first engaging the anti-swing
 * correction. Set to zero to allow engagement regardless of initial swing
 * angle.
 *
 * @unit rad
 * @min 0
 * @max 3.14
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACT_ANG, 0.0f);

/**
 * Suspended-load anti-swing activation rate
 *
 * Maximum swing angular-rate norm allowed before first engaging the anti-swing
 * correction. Set to zero to allow engagement regardless of initial swing
 * angular rate.
 *
 * @unit rad/s
 * @min 0
 * @max 20
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACT_R, 0.0f);

/**
 * Suspended-load anti-swing activation stable time
 *
 * Required continuous time with swing angle and angular rate below activation
 * thresholds before the anti-swing correction may engage.
 *
 * @unit s
 * @min 0
 * @max 30
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_ACT_T, 0.0f);

/**
 * Suspended-load anti-swing ramp time
 *
 * Time used to ramp the anti-swing acceleration from zero to the requested
 * value after the activation delay and activation angle checks pass.
 *
 * @unit s
 * @min 0
 * @max 30
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_RAMP_T, 2.0f);

/**
 * Suspended-load anti-swing abort angle
 *
 * If the anti-swing correction is engaged and the filtered swing angle norm
 * exceeds this extreme angle, the correction slews to zero and waits for a
 * safe re-engagement condition. This is separate from MC_HANG_MAX_ANG, which
 * only disables the LegacyPD angle term outside the small-angle region.
 *
 * @unit rad
 * @min 0
 * @max 3.14
 * @decimal 2
 * @increment 0.02
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_SAFE_A, 0.8f);

/**
 * Suspended-load anti-swing safety re-arm delay
 *
 * Delay before the anti-swing correction may re-engage after it was disabled
 * by the in-flight safety angle. This is intentionally separate from the
 * takeoff activation delay, which only protects the first engagement after
 * takeoff.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_REARM, 1.0f);

/**
 * Suspended-load frequency scheduling enable
 *
 * When enabled, the horizontal LADRC controller and observer bandwidths are
 * capped using the known rope natural frequency. This is fixed-parameter
 * scheduling, not online rope-length estimation.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_FRQ_EN, 0);

/**
 * Horizontal LADRC frequency ratio
 *
 * Caps wc_xy at this ratio times sqrt(g/L).
 *
 * @min 0.05
 * @max 5
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_WC_R, 0.25f);

/**
 * Horizontal LESO frequency ratio
 *
 * Caps wo_xy at this ratio times sqrt(g/L).
 *
 * @min 0.1
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_WO_R, 0.60f);

/**
 * Horizontal LESO minimum bandwidth ratio
 *
 * Enforces wo_xy greater than or equal to this ratio times wc_xy after
 * frequency scheduling.
 *
 * @min 1
 * @max 10
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_WO_MIN, 2.0f);

/**
 * Suspended-load total horizontal acceleration
 *
 * Total horizontal acceleration budget used while swing-management is
 * requesting acceleration. The effective limit is the lower of this value
 * and MPC_ACC_HOR_MAX. Swing-management requests keep priority within this
 * budget and the base position controller uses the remaining authority.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 30
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_TOT_A, 3.0f);

/**
 * Suspended-load energy supervisor mode
 *
 * SHADOW computes and logs the correction without changing the command.
 * ACTIVE applies the same correction before the total horizontal acceleration
 * envelope. OFF and SHADOW preserve the previous closed loop.
 *
 * 0: disabled
 * 1: shadow diagnostics only
 * 2: active command correction
 *
 * @min 0
 * @max 2
 * @value 0 Disabled
 * @value 1 Shadow
 * @value 2 Active direct compensation
 * @value 3 HESO effective-frequency/confidence AS gain schedule
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_PAS_MD, 0);

/**
 * Energy supervisor activation energy
 *
 * The energy correction is gated off below this per-unit-mass swing energy.
 *
 * @min 0
 * @max 20
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_E, 0.005f);

/**
 * Energy supervisor minimum swing rate
 *
 * Prevents the projection denominator becoming ill-conditioned near zero
 * swing rate.
 *
 * @unit rad/s
 * @min 0
 * @max 20
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_R, 0.05f);

/**
 * Energy supervisor correction gain
 *
 * Gain applied to the limited minimum-power correction. Use 1.0 for the
 * FAST-1/FAST-2 limits to equal the effective applied correction limits.
 *
 * @min 0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_K, 0.20f);

/**
 * Energy supervisor correction limit
 *
 * Maximum raw correction norm before applying PAS_K. With PAS_K=1 this is
 * also the final ACTIVE correction limit.
 *
 * @unit m/s^2
 * @min 0
 * @max 5
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_LIM, 0.15f);

/**
 * Energy supervisor correction slew rate
 *
 * @unit m/s^3
 * @min 0
 * @max 20
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_SLW, 0.50f);

/**
 * Energy supervisor positive-power filter
 *
 * First-order low-pass cutoff for max(P_candidate, 0). Zero bypasses the
 * filter.
 *
 * @unit Hz
 * @min 0
 * @max 20
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_LPF, 1.0f);

/**
 * Energy supervisor positive-power deadband
 *
 * @min 0
 * @max 10
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_P, 0.0005f);

/**
 * Energy supervisor positive-power dwell
 *
 * All gates must remain satisfied for this short duration before a correction
 * is produced. When any gate clears, slew limiting returns it smoothly to zero.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_DLY, 0.10f);

/**
 * Energy supervisor position-recovery protection
 *
 * In ACTIVE mode, preserve part of the candidate acceleration toward the
 * position setpoint when the energy correction would oppose that recovery.
 * SHADOW and OFF are intentionally unaffected.
 *
 * 0: disabled (third-stage FAST-2 behavior)
 * 1: enabled
 *
 * @min 0
 * @max 1
 * @value 0 Disabled
 * @value 1 Enabled
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_PAS_POS, 0);

/**
 * Maximum position-recovery acceleration cancelled by ACTIVE
 *
 * When position protection is enabled, ACTIVE may oppose at most this
 * fraction of the candidate acceleration toward the position setpoint.
 * 0.40 retains at least 60 percent of the candidate recovery component.
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_PAS_PR, 0.40f);

/**
 * Frequency-selective load-disturbance observer mode
 *
 * SHADOW runs and logs the harmonic observer and projected candidate, while
 * ACTIVE applies it. HESO_GAIN schedules the existing anti-swing damping from
 * the legacy HESO frequency bank. RLS_SHADOW estimates frequency with a
 * window-integral RLS but keeps the anti-swing multiplier at one. ORACLE_GAIN
 * uses the model frequency with unit confidence to isolate scheduler value.
 * UNIFIED_SHAPING runs the conservative RLS confidence path for adaptive
 * input-shaping diagnostics and enables the position/acceleration/jerk AS
 * permission coordinator. It never applies direct HESO acceleration or an AS
 * gain above one.
 *
 * @value 0 Off
 * @value 1 Shadow
 * @value 2 Active
 * @value 3 HESO gain schedule
 * @value 4 RLS frequency shadow
 * @value 5 Oracle-frequency gain schedule
 * @value 6 Unified input-shaping coordinator
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_FSO_MD, 0);

/**
 * Frequency-selective observer bandwidth
 *
 * Error-dynamics bandwidth of the harmonic extended-state observer. The
 * disturbance internal-model frequency is computed from sqrt(g/L)/(2*pi).
 *
 * @unit Hz
 * @min 0.1
 * @max 10
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_BW, 0.30f);

/**
 * Frequency-selective compensation gain
 *
 * @min 0
 * @max 1
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_K, 0.20f);

/**
 * Frequency-selective disturbance phase lead
 *
 * Predicts the harmonic disturbance by this time before compensation. This
 * can account for attitude/thrust response delay without differentiating the
 * measured velocity.
 *
 * @unit s
 * @min 0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_LEAD, 0.05f);

/**
 * Frequency-selective compensation acceleration limit
 *
 * @unit m/s^2
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_LIM, 0.08f);

/**
 * Frequency-selective compensation slew rate
 *
 * @unit m/s^3
 * @min 0
 * @max 10
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_SLW, 0.30f);

/**
 * Frequency-selective compensation activation energy
 *
 * @min 0
 * @max 20
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_E, 0.003f);

/**
 * Frequency-selective compensation minimum swing rate
 *
 * @unit rad/s
 * @min 0
 * @max 20
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_R, 0.03f);

/**
 * Frequency-selective observer settling time
 *
 * No compensation is produced until the narrow-band estimate has converged
 * for this duration after anti-swing engagement.
 *
 * @unit s
 * @min 0
 * @max 20
 * @decimal 1
 * @increment 0.5
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_T, 2.0f);

/**
 * HESO frequency-estimate minimum confidence
 *
 * In gain-schedule mode, the observer-bank confidence is smoothly gated from
 * this value to this value plus 0.25. Below the threshold the anti-swing gain
 * is exactly the frozen base value.
 *
 * @min 0
 * @max 0.8
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_CF, 0.20f);

/**
 * Frequency-selective position protection
 *
 * When enabled, continuously tighten the compensation position-recovery
 * half-space as horizontal error approaches MC_HANG_FSO_PE. The constraint
 * is blended with the power, amplitude and slew constraints in one update.
 *
 * @boolean
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MC_HANG_FSO_POS, 1);

/**
 * Frequency-selective position-protection threshold
 *
 * Position protection reaches half weight at this horizontal error. It is
 * smoothly blended from zero at half this value to full at 1.5 times this
 * value, avoiding a hard constraint switch near the setpoint.
 *
 * @unit m
 * @min 0
 * @max 2
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_FSO_PE, 0.05f);
