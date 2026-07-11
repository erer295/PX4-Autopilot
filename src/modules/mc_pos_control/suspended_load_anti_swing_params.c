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
PARAM_DEFINE_FLOAT(MC_HANG_WC_R, 0.45f);

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
PARAM_DEFINE_FLOAT(MC_HANG_WO_R, 1.50f);

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
PARAM_DEFINE_FLOAT(MC_HANG_WO_MIN, 2.5f);

/**
 * Suspended-load total horizontal acceleration
 *
 * Total horizontal acceleration budget used while anti-swing is requesting
 * acceleration. The effective limit is the lower of this value and
 * MPC_ACC_HOR_MAX. Trajectory control has priority within this budget.
 *
 * @unit m/s^2
 * @min 0.1
 * @max 30
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MC_HANG_TOT_A, 3.0f);
