/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mc_rate_control_params.c
 *
 * Parameters for multicopter rate controller
 */

/**
 * Roll rate P gain
 *
 * Roll rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_P, 0.15f);

/**
 * Roll rate I gain
 *
 * Roll rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_I, 0.2f);

/**
 * Roll rate integrator limit
 *
 * Roll rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large roll moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RR_INT_LIM, 0.30f);

/**
 * Roll rate D gain
 *
 * Roll rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @max 0.01
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_D, 0.003f);

/**
 * Roll rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_FF, 0.0f);

/**
 * Roll rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_ROLLRATE_K * (MC_ROLLRATE_P * error
 * 			     + MC_ROLLRATE_I * error_integral
 * 			     + MC_ROLLRATE_D * error_derivative)
 * Set MC_ROLLRATE_P=1 to implement a PID in the ideal form.
 * Set MC_ROLLRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_ROLLRATE_K, 1.0f);

/**
 * Pitch rate P gain
 *
 * Pitch rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.01
 * @max 0.6
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_P, 0.15f);

/**
 * Pitch rate I gain
 *
 * Pitch rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_I, 0.2f);

/**
 * Pitch rate integrator limit
 *
 * Pitch rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large pitch moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PR_INT_LIM, 0.30f);

/**
 * Pitch rate D gain
 *
 * Pitch rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_D, 0.003f);

/**
 * Pitch rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_FF, 0.0f);

/**
 * Pitch rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_PITCHRATE_K * (MC_PITCHRATE_P * error
 * 			     + MC_PITCHRATE_I * error_integral
 * 			     + MC_PITCHRATE_D * error_derivative)
 * Set MC_PITCHRATE_P=1 to implement a PID in the ideal form.
 * Set MC_PITCHRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.01
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_PITCHRATE_K, 1.0f);

/**
 * Yaw rate P gain
 *
 * Yaw rate proportional gain, i.e. control output for angular speed error 1 rad/s.
 *
 * @min 0.0
 * @max 0.6
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_P, 0.2f);

/**
 * Yaw rate I gain
 *
 * Yaw rate integral gain. Can be set to compensate static thrust difference or gravity center offset.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_I, 0.1f);

/**
 * Yaw rate integrator limit
 *
 * Yaw rate integrator limit. Can be set to increase the amount of integrator available to counteract disturbances or reduced to improve settling time after large yaw moment trim changes.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YR_INT_LIM, 0.30f);

/**
 * Yaw rate D gain
 *
 * Yaw rate differential gain. Small values help reduce fast oscillations. If value is too big oscillations will appear again.
 *
 * @min 0.0
 * @decimal 2
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_D, 0.0f);

/**
 * Yaw rate feedforward
 *
 * Improves tracking performance.
 *
 * @min 0.0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_FF, 0.0f);

/**
 * Yaw rate controller gain
 *
 * Global gain of the controller.
 *
 * This gain scales the P, I and D terms of the controller:
 * output = MC_YAWRATE_K * (MC_YAWRATE_P * error
 * 			     + MC_YAWRATE_I * error_integral
 * 			     + MC_YAWRATE_D * error_derivative)
 * Set MC_YAWRATE_P=1 to implement a PID in the ideal form.
 * Set MC_YAWRATE_K=1 to implement a PID in the parallel form.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAWRATE_K, 1.0f);

/**
 * Battery power level scaler
 *
 * This compensates for voltage drop of the battery over time by attempting to
 * normalize performance across the operating range of the battery. The copter
 * should constantly behave as if it was fully charged with reduced max acceleration
 * at lower battery percentages. i.e. if hover is at 0.5 throttle at 100% battery,
 * it will still be 0.5 at 60% battery.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_BAT_SCALE_EN, 0);

/**
 * Low pass filter cutoff frequency for yaw torque setpoint
 *
 * Reduces vibrations by lowering high frequency torque caused by rotor acceleration.
 * 0 disables the filter
 *
 * @min 0
 * @max 10
 * @unit Hz
 * @decimal 3
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_YAW_TQ_CUTOFF, 2.f);

/**
 * Enable LADRC rate controller
 *
 * 0: use original PX4 PID rate controller
 * 1: use LADRC rate controller
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_LADRC_EN, 0);

/**
 * LADRC roll equivalent input gain
 *
 * This is the nominal input gain b0 for roll axis in:
 * rate_dot = total_disturbance + b0 * normalized_torque
 *
 * @min 0.001
 * @decimal 3
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_B0_R, 50.0f);

/**
 * LADRC pitch equivalent input gain
 *
 * This is the nominal input gain b0 for pitch axis in:
 * rate_dot = total_disturbance + b0 * normalized_torque
 *
 * @min 0.001
 * @decimal 3
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_B0_P, 50.0f);

/**
 * LADRC yaw equivalent input gain
 *
 * This is the nominal input gain b0 for yaw axis in:
 * rate_dot = total_disturbance + b0 * normalized_torque
 *
 * @min 0.001
 * @decimal 3
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_B0_Y, 20.0f);

/**
 * LADRC roll controller bandwidth
 *
 * Higher value gives faster roll rate tracking, but may cause oscillation.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WC_R, 8.0f);

/**
 * LADRC pitch controller bandwidth
 *
 * Higher value gives faster pitch rate tracking, but may cause oscillation.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WC_P, 8.0f);

/**
 * LADRC yaw controller bandwidth
 *
 * Higher value gives faster yaw rate tracking, but may cause oscillation.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WC_Y, 4.0f);

/**
 * LADRC roll observer bandwidth
 *
 * Higher value gives faster disturbance estimation, but increases noise sensitivity.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WO_R, 30.0f);

/**
 * LADRC pitch observer bandwidth
 *
 * Higher value gives faster disturbance estimation, but increases noise sensitivity.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WO_P, 30.0f);

/**
 * LADRC yaw observer bandwidth
 *
 * Higher value gives faster disturbance estimation, but increases noise sensitivity.
 *
 * @min 0.01
 * @max 500
 * @unit rad/s
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_WO_Y, 15.0f);

/**
 * LADRC roll normalized torque limit
 *
 * Limit the roll torque output of LADRC during initial tuning.
 *
 * @min 0.001
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_LIM_R, 0.35f);

/**
 * LADRC pitch normalized torque limit
 *
 * Limit the pitch torque output of LADRC during initial tuning.
 *
 * @min 0.001
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_LIM_P, 0.35f);

/**
 * LADRC yaw normalized torque limit
 *
 * Limit the yaw torque output of LADRC during initial tuning.
 *
 * @min 0.001
 * @max 1.0
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_LIM_Y, 0.20f);

/**
 * LADRC roll angular-acceleration damping
 *
 * Damping gain applied as -gain * roll angular acceleration on top of the
 * LADRC torque command. This mirrors the PX4 PID D path and can reduce fast
 * roll oscillations. Start with small values.
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_D_R, 0.0f);

/**
 * LADRC pitch angular-acceleration damping
 *
 * Damping gain applied as -gain * pitch angular acceleration on top of the
 * LADRC torque command. This mirrors the PX4 PID D path and can reduce fast
 * pitch oscillations. Start with small values.
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_D_P, 0.0f);

/**
 * LADRC yaw angular-acceleration damping
 *
 * Damping gain applied as -gain * yaw angular acceleration on top of the LADRC
 * torque command. Keep at zero unless yaw-rate oscillation is observed.
 *
 * @min 0
 * @max 1.0
 * @decimal 4
 * @increment 0.0005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_D_Y, 0.0f);

/**
 * LADRC TD operating mode
 *
 * Selects when the tracking differentiator is allowed to shape the LADRC body
 * rate setpoint. This switch is only active when MC_LADRC_EN is enabled.
 *
 * 0: disabled, LADRC uses raw rate setpoints
 * 1: always enabled in flight, not recommended for takeoff tests
 * 2: enabled after takeoff delay and stability checks, recommended
 * 3: enabled only in stable hover/anti-swing conditions
 *
 * @min 0
 * @max 3
 * @value 0 Disabled
 * @value 1 Always
 * @value 2 Safe ramp
 * @value 3 Hover only
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_LADRC_TD_MODE, 0);

/**
 * LADRC TD fast roll bandwidth
 *
 * Fast roll TD bandwidth used by MC_LADRC_TD_MODE 1 and 2. The short parameter
 * name means W_FAST_R.
 *
 * @min 0.1
 * @max 40.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_FW_R, 15.0f);

/**
 * LADRC TD fast pitch bandwidth
 *
 * Fast pitch TD bandwidth used by MC_LADRC_TD_MODE 1 and 2. The short parameter
 * name means W_FAST_P.
 *
 * @min 0.1
 * @max 40.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_FW_P, 15.0f);

/**
 * LADRC TD fast roll acceleration limit
 *
 * Fast roll TD v2 limit used by MC_LADRC_TD_MODE 1 and 2. The short parameter
 * name means A_FAST_R.
 *
 * @min 0.1
 * @max 200.0
 * @unit rad/s^2
 * @decimal 2
 * @increment 1.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_FA_R, 80.0f);

/**
 * LADRC TD fast pitch acceleration limit
 *
 * Fast pitch TD v2 limit used by MC_LADRC_TD_MODE 1 and 2. The short parameter
 * name means A_FAST_P.
 *
 * @min 0.1
 * @max 200.0
 * @unit rad/s^2
 * @decimal 2
 * @increment 1.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_FA_P, 80.0f);

/**
 * LADRC TD slow roll bandwidth
 *
 * Slow roll TD bandwidth used by MC_LADRC_TD_MODE 3. The short parameter name
 * means W_SLOW_R.
 *
 * @min 0.1
 * @max 40.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_SW_R, 8.0f);

/**
 * LADRC TD slow pitch bandwidth
 *
 * Slow pitch TD bandwidth used by MC_LADRC_TD_MODE 3. The short parameter name
 * means W_SLOW_P.
 *
 * @min 0.1
 * @max 40.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_SW_P, 8.0f);

/**
 * LADRC TD slow roll acceleration limit
 *
 * Slow roll TD v2 limit used by MC_LADRC_TD_MODE 3. The short parameter name
 * means A_SLOW_R.
 *
 * @min 0.1
 * @max 200.0
 * @unit rad/s^2
 * @decimal 2
 * @increment 1.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_SA_R, 30.0f);

/**
 * LADRC TD slow pitch acceleration limit
 *
 * Slow pitch TD v2 limit used by MC_LADRC_TD_MODE 3. The short parameter name
 * means A_SLOW_P.
 *
 * @min 0.1
 * @max 200.0
 * @unit rad/s^2
 * @decimal 2
 * @increment 1.0
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_SA_P, 30.0f);

/**
 * LADRC TD yaw bandwidth
 *
 * Yaw has weaker direct coupling into suspended-load swing than roll/pitch, so
 * it can usually use a higher TD bandwidth.
 *
 * @min 0.1
 * @max 20.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_W_Y, 5.0f);

/**
 * LADRC TD damping ratio
 *
 * 1.0 gives a critically damped second-order command filter.
 *
 * @min 0.5
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_ZETA, 1.0f);

/**
 * LADRC TD yaw acceleration limit
 *
 * Limits the rate of change of the smoothed yaw rate setpoint. This is the TD
 * v2 state and is equivalent to desired yaw angular acceleration.
 *
 * @min 0.1
 * @max 200.0
 * @unit rad/s^2
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_A_Y, 8.0f);

/**
 * LADRC TD activation delay after takeoff
 *
 * Keeps the TD bypassed for the first seconds after liftoff. In modes 2 and 3
 * the TD can only ramp in after this delay and the stability checks pass.
 *
 * @min 0.0
 * @max 20.0
 * @unit s
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_DLY, 4.0f);

/**
 * LADRC TD blend-in ramp time
 *
 * Smoothly blends from raw rate setpoint to TD-smoothed rate setpoint while the
 * TD safety gate remains true. Set to 0 for immediate activation.
 *
 * @min 0.0
 * @max 20.0
 * @unit s
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_RAMP, 4.0f);

/**
 * LADRC TD attitude gate
 *
 * Roll and pitch must both be below this threshold before modes 2 and 3 are
 * allowed to ramp TD into the LADRC rate setpoint.
 *
 * @min 0.0
 * @max 0.8
 * @unit rad
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_ATT, 0.15f);

/**
 * LADRC TD body-rate gate
 *
 * Roll and pitch body rates must both be below this threshold before modes 2
 * and 3 are allowed to ramp TD into the LADRC rate setpoint.
 *
 * @min 0.0
 * @max 5.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_RATE, 0.5f);

/**
 * LADRC TD rate-error gate
 *
 * Roll and pitch rate errors must both be below this threshold before modes 2
 * and 3 are allowed to ramp TD into the LADRC rate setpoint.
 *
 * @min 0.0
 * @max 5.0
 * @unit rad/s
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_ERR, 0.5f);

/**
 * LADRC TD feed-forward gain
 *
 * Reserved for a later LADRC extension that feeds TD v2 into the controller.
 * Keep at zero for the initial suspended-load TD-LADRC tests.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_LADRC_TD_FF, 0.0f);

/**
 * Enable RBF residual compensation after LADRC
 *
 * This switch is only active when MC_LADRC_EN is also enabled.
 *
 * 0: LADRC output is used directly
 * 1: use LADRC plus RBF residual compensation
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_EN, 0);

/**
 * Enable RBF residual compensation on roll axis
 *
 * This switch is only active when MC_LADRC_EN and MC_RBF_EN are enabled.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_EN_R, 1);

/**
 * Enable RBF residual compensation on pitch axis
 *
 * This switch is only active when MC_LADRC_EN and MC_RBF_EN are enabled.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_EN_P, 1);

/**
 * Enable RBF residual compensation on yaw axis
 *
 * This switch is only active when MC_LADRC_EN and MC_RBF_EN are enabled.
 * Keep disabled by default during initial residual-learning tests.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_EN_Y, 0);

/**
 * Enable RBF residual injection after LADRC
 *
 * This switch is only active when MC_LADRC_EN and MC_RBF_EN are also enabled.
 * When disabled, the RBF module still computes, learns, and logs diagnostics,
 * but the final torque command remains the LADRC output.
 *
 * 0: compute/learn/log RBF in bypass mode
 * 1: inject RBF residual torque into the LADRC torque command
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_INJECT_EN, 0);

/**
 * Enable online RBF residual adaptation
 *
 * The adaptation signal is based on body-rate tracking error. Keep disabled
 * for baseline LADRC tests and enable only when testing RBF-LADRC.
 *
 * @boolean
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_LEARN_EN, 0);

/**
 * Number of RBF basis functions
 *
 * Basis 0 is centered at zero tracking error. Additional bases are placed
 * symmetrically on the first three rate-error features by default.
 *
 * @min 0
 * @max 12
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_BASIS, 7);

/**
 * RBF input feature dimension
 *
 * Compatibility parameter kept for older logs. The active implementation uses
 * a fixed per-axis normalized input: bias, filtered rate error, LADRC torque,
 * LADRC disturbance compensation and residual angular acceleration. The
 * Gaussian distance ignores the bias slot.
 *
 * @min 1
 * @max 18
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_INT32(MC_RBF_IN_DIM, 5);

/**
 * RBF basis width
 *
 * Larger values make each basis function active over a wider flight condition.
 *
 * @min 0.001
 * @decimal 3
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_WIDTH, 0.3f);

/**
 * RBF basis center spacing
 *
 * Spacing used to place the automatically generated symmetric basis centers.
 *
 * @min 0
 * @decimal 3
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_SPACING, 0.3f);

/**
 * RBF learning rate
 *
 * Online adaptation gain for the RBF weights. Start with a small value and
 * increase gradually in SITL.
 *
 * @min 0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LR, 0.05f);

/**
 * RBF weight leakage
 *
 * Leakage term that slowly pulls RBF weights back toward zero.
 *
 * @min 0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LEAK, 0.05f);

/**
 * RBF roll normalized torque limit
 *
 * Absolute roll-axis limit of the residual normalized torque added on top of
 * LADRC.
 *
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LIM_R, 0.015f);

/**
 * RBF pitch normalized torque limit
 *
 * Absolute pitch-axis limit of the residual normalized torque added on top of
 * LADRC.
 *
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LIM_P, 0.015f);

/**
 * RBF yaw normalized torque limit
 *
 * Absolute yaw-axis limit of the residual normalized torque added on top of
 * LADRC. Keep this lower than roll and pitch because the yaw LADRC authority is
 * usually smaller.
 *
 * @min 0
 * @max 1.0
 * @decimal 3
 * @increment 0.005
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LIM_Y, 0.0f);

/**
 * RBF output low-pass alpha
 *
 * First-order smoothing factor applied after the RBF output limit. Larger
 * values make the residual torque change more slowly.
 *
 * @min 0
 * @max 0.999
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_LPF_ALPHA, 0.95f);

/**
 * RBF output slew-rate limit
 *
 * Maximum absolute change rate of the final RBF residual torque after output
 * limiting and low-pass filtering. Set to 0 to disable slew-rate limiting.
 *
 * @min 0
 * @max 10
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_DU_MAX, 0.5f);

/**
 * RBF residual target gain
 *
 * Scales the equivalent residual-disturbance torque target used for online RBF
 * learning. Values below 1 make the learned residual intentionally conservative.
 *
 * @min 0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ERR_GAIN, 0.05f);

/**
 * RBF residual target rate-error bandwidth
 *
 * Converts the low-frequency body-rate tracking error into an additional
 * equivalent normalized torque target for online RBF learning. This term helps
 * the residual network learn LADRC tracking leftovers that are not visible as
 * angular-acceleration model error alone.
 *
 * @unit rad/s
 * @min 0
 * @decimal 3
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ERR_WC, 0.0f);

/**
 * RBF residual target cutoff frequency
 *
 * Cutoff frequency for the equivalent residual-disturbance torque target used
 * by online RBF learning.
 *
 * @unit Hz
 * @min 0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_TGT_HZ, 2.0f);

/**
 * RBF residual acceleration cutoff frequency
 *
 * Cutoff frequency for the residual angular acceleration used by the per-axis
 * RBF-LADRC input and acceleration-residual learning target.
 *
 * @unit Hz
 * @min 0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_RES_HZ, 3.0f);

/**
 * RBF learning minimum filtered rate error
 *
 * Online RBF adaptation is disabled below this filtered rate-error magnitude
 * so small noise-driven errors only leak the weights back toward zero.
 *
 * @unit rad/s
 * @min 0
 * @decimal 4
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_MIN, 0.03f);

/**
 * RBF learning maximum filtered rate error
 *
 * Online RBF adaptation is disabled above this filtered rate-error magnitude
 * to avoid learning from large transients, aggressive steps or abnormal states.
 *
 * @unit rad/s
 * @min 0
 * @decimal 3
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_MAX, 1.5f);

/**
 * RBF learning maximum rate-setpoint slope
 *
 * Online RBF adaptation is disabled when the rate setpoint changes faster than
 * this limit. This avoids learning from commanded step or snap maneuvers.
 *
 * @unit rad/s^2
 * @min 0
 * @decimal 3
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_SPD_MAX, 5.0f);

/**
 * RBF filtered rate-error cutoff frequency
 *
 * Cutoff frequency for the filtered tracking error used by the learning gate.
 * It does not filter the RBF feature vector or the final torque command.
 *
 * @unit Hz
 * @min 0
 * @decimal 2
 * @increment 0.5
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_FILT_HZ, 3.0f);

/**
 * RBF feature limit
 *
 * Limits feature magnitude before evaluating the RBF basis functions.
 *
 * @min 0.001
 * @decimal 1
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_FEAT_LIM, 100.0f);

/**
 * RBF roll rate-error normalization scale
 *
 * Scale for filtered roll-rate error before evaluating the per-axis RBF basis.
 *
 * @unit rad/s
 * @min 0.001
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_SCALE_R, 0.20f);

/**
 * RBF pitch rate-error normalization scale
 *
 * Scale for filtered pitch-rate error before evaluating the per-axis RBF basis.
 *
 * @unit rad/s
 * @min 0.001
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_SCALE_P, 0.20f);

/**
 * RBF yaw rate-error normalization scale
 *
 * Scale for filtered yaw-rate error before evaluating the per-axis RBF basis.
 *
 * @unit rad/s
 * @min 0.001
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_E_SCALE_Y, 0.20f);

/**
 * RBF roll residual-acceleration normalization scale
 *
 * Scale for filtered roll residual angular acceleration before evaluating the
 * per-axis RBF basis.
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ACC_SC_R, 10.0f);

/**
 * RBF pitch residual-acceleration normalization scale
 *
 * Scale for filtered pitch residual angular acceleration before evaluating the
 * per-axis RBF basis.
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ACC_SC_P, 10.0f);

/**
 * RBF yaw residual-acceleration normalization scale
 *
 * Scale for filtered yaw residual angular acceleration before evaluating the
 * per-axis RBF basis.
 *
 * @unit rad/s^2
 * @min 0.001
 * @decimal 2
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ACC_SC_Y, 10.0f);

/**
 * RBF residual-acceleration learning freeze limit
 *
 * Online RBF adaptation is disabled while the filtered residual angular
 * acceleration exceeds this magnitude.
 *
 * @unit rad/s^2
 * @min 0
 * @decimal 1
 * @increment 1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_ACC_MAX, 30.0f);

/**
 * RBF sign-change freeze time
 *
 * Duration to freeze online RBF adaptation after the filtered rate-error sign
 * changes while the error is larger than MC_RBF_E_MIN.
 *
 * @unit s
 * @min 0
 * @decimal 2
 * @increment 0.1
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_FRZ_SGN_T, 0.30f);

/**
 * RBF target-jump freeze threshold
 *
 * Online RBF adaptation is frozen when the filtered residual torque target
 * changes by more than this value in one cycle.
 *
 * @min 0
 * @decimal 4
 * @increment 0.001
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_TGT_JUMP, 0.008f);

/**
 * RBF frozen-output decay
 *
 * Multiplier applied to the stored RBF residual output on axes that are frozen
 * by transient or saturation protection.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_FRZ_DEC, 0.98f);

/**
 * RBF final torque limit gain
 *
 * Final RBF-LADRC torque is constrained to LADRC_LIMIT multiplied by this gain.
 *
 * @min 0
 * @max 1.2
 * @decimal 3
 * @increment 0.01
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_FIN_GAIN, 1.0f);

/**
 * RBF roll injection gain
 *
 * Gain applied to roll RBF residual torque before adding it to LADRC torque.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_INJ_R, 1.0f);

/**
 * RBF pitch injection gain
 *
 * Gain applied to pitch RBF residual torque before adding it to LADRC torque.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_INJ_P, 1.0f);

/**
 * RBF yaw injection gain
 *
 * Gain applied to yaw RBF residual torque before adding it to LADRC torque.
 * Keep zero by default during initial RBF-LADRC tests.
 *
 * @min 0
 * @max 1
 * @decimal 3
 * @increment 0.05
 * @group Multicopter Rate Control
 */
PARAM_DEFINE_FLOAT(MC_RBF_INJ_Y, 0.0f);
