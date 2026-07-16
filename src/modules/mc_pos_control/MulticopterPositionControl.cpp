/****************************************************************************
 *
 *   Copyright (c) 2013-2020 PX4 Development Team. All rights reserved.
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

#include "MulticopterPositionControl.hpp"

#include <float.h>
#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>
#include <px4_platform_common/events.h>
#include <string.h>
#include "PositionControl/ControlMath.hpp"

using namespace matrix;

namespace
{

constexpr uint16_t kPositionLadrcDebugArrayId = 683;
constexpr const char kPositionLadrcDebugArrayName[] = "posladrc";

enum PositionLadrcDebugArrayIndex : uint8_t {
	POS_LADRC_TD_VX = 0,
	POS_LADRC_TD_VY,
	POS_LADRC_TD_VZ,
	POS_LADRC_TD_AX,
	POS_LADRC_TD_AY,
	POS_LADRC_TD_AZ,
	POS_LADRC_ACC_X,
	POS_LADRC_ACC_Y,
	POS_LADRC_ACC_Z,
	POS_LADRC_DIST_X,
	POS_LADRC_DIST_Y,
	POS_LADRC_DIST_Z,
	POS_LADRC_ENABLED,
	POS_LADRC_TD_ENABLED,
	POS_LADRC_MODE,
	POS_LADRC_OBS_IN_X,
	POS_LADRC_OBS_IN_Y,
	POS_LADRC_RAW_X,
	POS_LADRC_RAW_Y,
	POS_LADRC_FINAL_X,
	POS_LADRC_FINAL_Y,
	POS_LADRC_APPLIED_X,
	POS_LADRC_APPLIED_Y,
	POS_LADRC_WC_XY,
	POS_LADRC_WO_XY,
	POS_LADRC_NOMINAL_POSITION_X,
	POS_LADRC_NOMINAL_POSITION_Y,
	POS_LADRC_NOMINAL_VELOCITY_REFERENCE_X,
	POS_LADRC_NOMINAL_VELOCITY_REFERENCE_Y,
	POS_LADRC_NOMINAL_VELOCITY_STATE_X,
	POS_LADRC_NOMINAL_VELOCITY_STATE_Y,
	POS_LADRC_NOMINAL_ACCELERATION_DAMPING_X,
	POS_LADRC_NOMINAL_ACCELERATION_DAMPING_Y,
};

} // namespace

MulticopterPositionControl::MulticopterPositionControl(bool vtol) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_vehicle_attitude_setpoint_pub(vtol ? ORB_ID(mc_virtual_attitude_setpoint) : ORB_ID(vehicle_attitude_setpoint))
{
	_sample_interval_s.update(0.01f); // 100 Hz default
	parameters_update(true);
	_tilt_limit_slew_rate.setSlewRate(.2f);
	_takeoff_status_pub.advertise();
}

MulticopterPositionControl::~MulticopterPositionControl()
{
	perf_free(_cycle_perf);
}

bool MulticopterPositionControl::init()
{
	if (!_local_pos_sub.registerCallback()) {
		PX4_ERR("callback registration failed");
		return false;
	}

	_time_stamp_last_loop = hrt_absolute_time();
	ScheduleNow();

	return true;
}

void MulticopterPositionControl::updatePositionControllerSelection()
{
	const PositionControl::ControllerMode controller_mode =
		PositionControl::sanitizeControllerMode(_param_mc_pladrc_en.get());
	const bool td_enabled = _param_mc_pladrc_td_en.get();

	if (_param_mc_pladrc_en.get() == 1) {
		PX4_WARN("MC_PLADRC_EN=1 was removed; using PID (select 2 or 3 for LADRC2)");
	}

	if (!_position_controller_selection_initialized
	    || controller_mode != _position_controller_mode
	    || td_enabled != _reported_position_ladrc_td_en) {
		_position_controller_mode = controller_mode;
		_reported_position_ladrc_td_en = td_enabled;
		_position_controller_selection_initialized = true;

		PX4_INFO("MC position outer loop: %s (MC_PLADRC_EN=%d, MC_PLADRC_TD_EN=%d)",
			 PositionControl::controllerModeName(_position_controller_mode),
			 (int)_param_mc_pladrc_en.get(),
			 (int)td_enabled);
	}
}

void MulticopterPositionControl::parameters_update(bool force)
{
	// check for parameter updates
	if (_parameter_update_sub.updated() || force) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();

		float sample_freq_hz = 1.f / _sample_interval_s.mean();

		// velocity notch filter
		if ((_param_mpc_vel_nf_frq.get() > 0.f) && (_param_mpc_vel_nf_bw.get() > 0.f)) {
			_vel_xy_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());
			_vel_z_notch_filter.setParameters(sample_freq_hz, _param_mpc_vel_nf_frq.get(), _param_mpc_vel_nf_bw.get());

		} else {
			_vel_xy_notch_filter.disable();
			_vel_z_notch_filter.disable();
		}

		// velocity xy/z low pass filter
		if (_param_mpc_vel_lp.get() > 0.f) {
			_vel_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());
			_vel_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_vel_lp.get());

		} else {
			// disable filtering
			_vel_xy_lp_filter.setAlpha(1.f);
			_vel_z_lp_filter.setAlpha(1.f);
		}

		// velocity derivative xy/z low pass filter
		if (_param_mpc_veld_lp.get() > 0.f) {
			_vel_deriv_xy_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());
			_vel_deriv_z_lp_filter.setCutoffFreq(sample_freq_hz, _param_mpc_veld_lp.get());

		} else {
			// disable filtering
			_vel_deriv_xy_lp_filter.setAlpha(1.f);
			_vel_deriv_z_lp_filter.setAlpha(1.f);
		}



		int num_changed = 0;

		if (_param_sys_vehicle_resp.get() >= 0.f) {
			// make it less sensitive at the lower end
			float responsiveness = _param_sys_vehicle_resp.get() * _param_sys_vehicle_resp.get();

			num_changed += _param_mpc_acc_hor.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_hor_max.commit_no_notification(math::lerp(2.f, 15.f, responsiveness));
			num_changed += _param_mpc_man_y_max.commit_no_notification(math::lerp(80.f, 450.f, responsiveness));

			if (responsiveness > 0.6f) {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(0.f);

			} else {
				num_changed += _param_mpc_man_y_tau.commit_no_notification(math::lerp(0.5f, 0.f, responsiveness / 0.6f));
			}

			if (responsiveness < 0.5f) {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(45.f);

			} else {
				num_changed += _param_mpc_tiltmax_air.commit_no_notification(math::min(MAX_SAFE_TILT_DEG, math::lerp(45.f, 70.f,
						(responsiveness - 0.5f) * 2.f)));
			}

			num_changed += _param_mpc_acc_down_max.commit_no_notification(math::lerp(0.8f, 15.f, responsiveness));
			num_changed += _param_mpc_acc_up_max.commit_no_notification(math::lerp(1.f, 15.f, responsiveness));
			num_changed += _param_mpc_jerk_max.commit_no_notification(math::lerp(2.f, 50.f, responsiveness));
			num_changed += _param_mpc_jerk_auto.commit_no_notification(math::lerp(1.f, 25.f, responsiveness));
		}

		if (_param_mpc_xy_vel_all.get() >= 0.f) {
			float xy_vel = _param_mpc_xy_vel_all.get();
			num_changed += _param_mpc_vel_manual.commit_no_notification(xy_vel);
			num_changed += _param_mpc_vel_man_back.commit_no_notification(-1.f);
			num_changed += _param_mpc_vel_man_side.commit_no_notification(-1.f);
			num_changed += _param_mpc_xy_cruise.commit_no_notification(xy_vel);
			num_changed += _param_mpc_xy_vel_max.commit_no_notification(xy_vel);
		}

		if (_param_mpc_z_vel_all.get() >= 0.f) {
			float z_vel = _param_mpc_z_vel_all.get();
			num_changed += _param_mpc_z_v_auto_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_vel_max_up.commit_no_notification(z_vel);
			num_changed += _param_mpc_z_v_auto_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_z_vel_max_dn.commit_no_notification(z_vel * 0.75f);
			num_changed += _param_mpc_tko_speed.commit_no_notification(z_vel * 0.6f);
			num_changed += _param_mpc_land_speed.commit_no_notification(z_vel * 0.5f);
		}

		if (num_changed > 0) {
			param_notify_changes();
		}

		if (_param_mpc_tiltmax_air.get() > MAX_SAFE_TILT_DEG) {
			_param_mpc_tiltmax_air.set(MAX_SAFE_TILT_DEG);
			_param_mpc_tiltmax_air.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Tilt constrained to safe value\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_AIR</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_tilt_set"), events::Log::Warning,
					    "Maximum tilt limit has been constrained to a safe value", MAX_SAFE_TILT_DEG);
		}

		if (_param_mpc_tiltmax_lnd.get() > _param_mpc_tiltmax_air.get()) {
			_param_mpc_tiltmax_lnd.set(_param_mpc_tiltmax_air.get());
			_param_mpc_tiltmax_lnd.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Land tilt has been constrained by max tilt\t");
			/* EVENT
			 * @description <param>MPC_TILTMAX_LND</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_land_tilt_set"), events::Log::Warning,
					    "Land tilt limit has been constrained by maximum tilt", _param_mpc_tiltmax_air.get());
		}

		_control.setPositionGains(Vector3f(_param_mpc_xy_p.get(), _param_mpc_xy_p.get(), _param_mpc_z_p.get()));
		_control.setVelocityGains(
			Vector3f(_param_mpc_xy_vel_p_acc.get(), _param_mpc_xy_vel_p_acc.get(), _param_mpc_z_vel_p_acc.get()),
			Vector3f(_param_mpc_xy_vel_i_acc.get(), _param_mpc_xy_vel_i_acc.get(), _param_mpc_z_vel_i_acc.get()),
			Vector3f(_param_mpc_xy_vel_d_acc.get(), _param_mpc_xy_vel_d_acc.get(), _param_mpc_z_vel_d_acc.get()));

		LadrcPositionControl::Parameters position_ladrc_parameters{};
		float horizontal_wc = _param_mc_pladrc_wc_xy.get();
		float horizontal_wo = _param_mc_pladrc_wo_xy.get();
		const float horizontal_acceleration_budget = math::min(_param_mpc_acc_hor_max.get(),
										 _param_mc_hang_tot_a.get());

		if (_param_mc_hang_frq_en.get()) {
			const float natural_frequency = SuspendedLoadAntiSwing::naturalFrequency(_param_mc_hang_len.get());
			LadrcPositionControl::frequencyScheduledXYBandwidths(horizontal_wc, horizontal_wo, natural_frequency,
					_param_mc_hang_wc_r.get(), _param_mc_hang_wo_r.get(), _param_mc_hang_wo_min.get(),
					horizontal_wc, horizontal_wo);
		}

		position_ladrc_parameters.b0 = Vector3f(_param_mc_pladrc_b0_xy.get(),
							_param_mc_pladrc_b0_xy.get(),
							_param_mc_pladrc_b0_z.get());
		position_ladrc_parameters.wc = Vector3f(horizontal_wc,
							horizontal_wc,
							_param_mc_pladrc_wc_z.get());
		position_ladrc_parameters.wo = Vector3f(horizontal_wo,
							horizontal_wo,
							_param_mc_pladrc_wo_z.get());
		position_ladrc_parameters.acceleration_damping = Vector3f(_param_mc_pladrc_d_xy.get(),
				_param_mc_pladrc_d_xy.get(),
				_param_mc_pladrc_d_z.get());
		position_ladrc_parameters.velocity_feedback_weight = _param_mc_pladrc_vfb_w.get();
		// MC_HANG_TOT_A is the total vehicle horizontal-acceleration budget,
		// including the primary LADRC command. Keep the LADRC-specific limit as
		// an additional, never larger bound.
		position_ladrc_parameters.horizontal_acceleration_limit = math::min(_param_mc_pladrc_lim_xy.get(),
											 horizontal_acceleration_budget);
		position_ladrc_parameters.upward_acceleration_limit = _param_mc_pladrc_lim_up.get();
		position_ladrc_parameters.downward_acceleration_limit = _param_mc_pladrc_lim_dn.get();
		position_ladrc_parameters.td_enabled = _param_mc_pladrc_td_en.get();
		position_ladrc_parameters.td_bandwidth = Vector3f(_param_mc_pladrc_td_wxy.get(),
				_param_mc_pladrc_td_wxy.get(),
				_param_mc_pladrc_td_wz.get());
		position_ladrc_parameters.td_acceleration_limit = Vector3f(_param_mc_pladrc_td_axy.get(),
				_param_mc_pladrc_td_axy.get(),
				_param_mc_pladrc_td_az.get());
		position_ladrc_parameters.td_damping_ratio = _param_mc_pladrc_td_dmp.get();

		_control.setLadrcPositionControlParameters(position_ladrc_parameters);
		_control.setControllerMode(PositionControl::sanitizeControllerMode(_param_mc_pladrc_en.get()));
		_control.setHorizontalAccelerationLimit(horizontal_acceleration_budget);
		updatePositionControllerSelection();

		_control.setHorizontalThrustMargin(_param_mpc_thr_xy_marg.get());
		_control.decoupleHorizontalAndVecticalAcceleration(_param_mpc_acc_decouple.get());

		SuspendedLoadAntiSwing::Parameters anti_swing_parameters{};
		anti_swing_parameters.enabled = _param_mc_hang_as_en.get();
		anti_swing_parameters.mode = static_cast<SuspendedLoadAntiSwing::Mode>(_param_mc_hang_mode.get());
		anti_swing_parameters.rope_length = _param_mc_hang_len.get();
		anti_swing_parameters.angle_gain = _param_mc_hang_k_ang.get();
		anti_swing_parameters.rate_gain = _param_mc_hang_k_rate.get();
		anti_swing_parameters.energy_damping_ratio = _param_mc_hang_zeta.get();
		anti_swing_parameters.energy_gate_start = _param_mc_hang_e_min.get();
		anti_swing_parameters.energy_gate_full = _param_mc_hang_e_full.get();
		anti_swing_parameters.acceleration_limit = _param_mc_hang_acc_lim.get();
		anti_swing_parameters.acceleration_slew_rate = _param_mc_hang_acc_slw.get();
		anti_swing_parameters.filter_cutoff_hz = _param_mc_hang_lpf_hz.get();
		anti_swing_parameters.sign_x = _param_mc_hang_sign_x.get();
		anti_swing_parameters.sign_y = _param_mc_hang_sign_y.get();
		anti_swing_parameters.max_angle = _param_mc_hang_max_ang.get();
		anti_swing_parameters.timeout_s = _param_mc_hang_timeout.get();
		anti_swing_parameters.activation_delay = _param_mc_hang_act_dly.get();
		anti_swing_parameters.activation_max_angle = _param_mc_hang_act_ang.get();
		anti_swing_parameters.activation_max_rate = _param_mc_hang_act_r.get();
		anti_swing_parameters.activation_stable_time = _param_mc_hang_act_t.get();
		anti_swing_parameters.ramp_time = _param_mc_hang_ramp_t.get();
		anti_swing_parameters.abort_angle = _param_mc_hang_safe_a.get();
		anti_swing_parameters.rearm_delay = _param_mc_hang_rearm.get();
		_control.setSuspendedLoadAntiSwingParameters(anti_swing_parameters);

		SuspendedLoadEnergySupervisor::Parameters energy_supervisor_parameters{};
		energy_supervisor_parameters.mode = static_cast<SuspendedLoadEnergySupervisor::Mode>(_param_mc_hang_pas_md.get());
		energy_supervisor_parameters.energy_threshold = _param_mc_hang_pas_e.get();
		energy_supervisor_parameters.rate_min = _param_mc_hang_pas_r.get();
		energy_supervisor_parameters.gain = _param_mc_hang_pas_k.get();
		energy_supervisor_parameters.correction_limit = _param_mc_hang_pas_lim.get();
		energy_supervisor_parameters.correction_slew_rate = _param_mc_hang_pas_slw.get();
		energy_supervisor_parameters.power_lpf_cutoff_hz = _param_mc_hang_pas_lpf.get();
		energy_supervisor_parameters.power_deadband = _param_mc_hang_pas_p.get();
		energy_supervisor_parameters.dwell_time = _param_mc_hang_pas_dly.get();
		energy_supervisor_parameters.position_recovery_protection_enabled = _param_mc_hang_pas_pos.get() != 0;
		energy_supervisor_parameters.position_recovery_cancellation_ratio = _param_mc_hang_pas_pr.get();
		_control.setSuspendedLoadEnergySupervisorParameters(energy_supervisor_parameters);

		_goto_control.setParamMpcAccHor(_param_mpc_acc_hor.get());
		_goto_control.setParamMpcAccDownMax(_param_mpc_acc_down_max.get());
		_goto_control.setParamMpcAccUpMax(_param_mpc_acc_up_max.get());
		_goto_control.setParamMpcJerkAuto(_param_mpc_jerk_auto.get());
		_goto_control.setParamMpcXyCruise(_param_mpc_xy_cruise.get());
		_goto_control.setParamMpcXyErrMax(_param_mpc_xy_err_max.get());
		_goto_control.setParamMpcXyVelMax(_param_mpc_xy_vel_max.get());
		_goto_control.setParamMpcYawrautoMax(_param_mpc_yawrauto_max.get());
		_goto_control.setParamMpcYawrautoAcc(_param_mpc_yawrauto_acc.get());
		_goto_control.setParamMpcZVAutoDn(_param_mpc_z_v_auto_dn.get());
		_goto_control.setParamMpcZVAutoUp(_param_mpc_z_v_auto_up.get());

		// Check that the design parameters are inside the absolute maximum constraints
		if (_param_mpc_xy_cruise.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_xy_cruise.set(_param_mpc_xy_vel_max.get());
			_param_mpc_xy_cruise.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Cruise speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_XY_CRUISE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_cruise_set"), events::Log::Warning,
					    "Cruise speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_manual.get() > _param_mpc_xy_vel_max.get()) {
			_param_mpc_vel_manual.set(_param_mpc_xy_vel_max.get());
			_param_mpc_vel_manual.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MANUAL</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_set"), events::Log::Warning,
					    "Manual speed has been constrained by maximum speed", _param_mpc_xy_vel_max.get());
		}

		if (_param_mpc_vel_man_back.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_back.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_back.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual backward speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_BACK</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_back_set"), events::Log::Warning,
					    "Manual backward speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_vel_man_side.get() > _param_mpc_vel_manual.get()) {
			_param_mpc_vel_man_side.set(_param_mpc_vel_manual.get());
			_param_mpc_vel_man_side.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Manual sideways speed has been constrained by forward speed\t");
			/* EVENT
			 * @description <param>MPC_VEL_MAN_SIDE</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_man_vel_side_set"), events::Log::Warning,
					    "Manual sideways speed has been constrained by forward speed", _param_mpc_vel_manual.get());
		}

		if (_param_mpc_z_v_auto_up.get() > _param_mpc_z_vel_max_up.get()) {
			_param_mpc_z_v_auto_up.set(_param_mpc_z_vel_max_up.get());
			_param_mpc_z_v_auto_up.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Ascent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_UP</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_up_vel_set"), events::Log::Warning,
					    "Ascent speed has been constrained by max speed", _param_mpc_z_vel_max_up.get());
		}

		if (_param_mpc_z_v_auto_dn.get() > _param_mpc_z_vel_max_dn.get()) {
			_param_mpc_z_v_auto_dn.set(_param_mpc_z_vel_max_dn.get());
			_param_mpc_z_v_auto_dn.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Descent speed has been constrained by max speed\t");
			/* EVENT
			 * @description <param>MPC_Z_V_AUTO_DN</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_down_vel_set"), events::Log::Warning,
					    "Descent speed has been constrained by max speed", _param_mpc_z_vel_max_dn.get());
		}

		if (_param_mpc_thr_hover.get() > _param_mpc_thr_max.get() ||
		    _param_mpc_thr_hover.get() < _param_mpc_thr_min.get()) {
			_param_mpc_thr_hover.set(math::constrain(_param_mpc_thr_hover.get(), _param_mpc_thr_min.get(),
						 _param_mpc_thr_max.get()));
			_param_mpc_thr_hover.commit();
			mavlink_log_critical(&_mavlink_log_pub, "Hover thrust has been constrained by min/max\t");
			/* EVENT
			 * @description <param>MPC_THR_HOVER</param> is set to {1:.0}.
			 */
			events::send<float>(events::ID("mc_pos_ctrl_hover_thrust_set"), events::Log::Warning,
					    "Hover thrust has been constrained by min/max thrust", _param_mpc_thr_hover.get());
		}

		if (!_param_mpc_use_hte.get() || !_hover_thrust_initialized) {
			_control.setHoverThrust(_param_mpc_thr_hover.get());
			_hover_thrust_initialized = true;
		}

		// initialize vectors from params and enforce constraints
		_param_mpc_tko_speed.set(math::min(_param_mpc_tko_speed.get(), _param_mpc_z_vel_max_up.get()));
		_param_mpc_land_speed.set(math::min(_param_mpc_land_speed.get(), _param_mpc_z_vel_max_dn.get()));

		_takeoff.setSpoolupTime(_param_com_spoolup_time.get());
		_takeoff.setTakeoffRampTime(_param_mpc_tko_ramp_t.get());
		_takeoff.generateInitialRampValue(_param_mpc_z_vel_p_acc.get());
	}
}

PositionControlStates MulticopterPositionControl::set_vehicle_states(const vehicle_local_position_s
		&vehicle_local_position, const float dt_s)
{
	PositionControlStates states;

	const Vector2f position_xy(vehicle_local_position.x, vehicle_local_position.y);

	// only set position states if valid and finite
	if (vehicle_local_position.xy_valid && position_xy.isAllFinite()) {
		states.position.xy() = position_xy;

	} else {
		states.position(0) = states.position(1) = NAN;
	}

	if (PX4_ISFINITE(vehicle_local_position.z) && vehicle_local_position.z_valid) {
		states.position(2) = vehicle_local_position.z;

	} else {
		states.position(2) = NAN;
	}

	const Vector2f velocity_xy(vehicle_local_position.vx, vehicle_local_position.vy);

	if (vehicle_local_position.v_xy_valid && velocity_xy.isAllFinite()) {
		const Vector2f vel_xy_prev = _vel_xy_lp_filter.getState();

		// vel xy notch filter, then low pass filter
		states.velocity.xy() = _vel_xy_lp_filter.update(_vel_xy_notch_filter.apply(velocity_xy));

		// vel xy derivative low pass filter
		states.acceleration.xy() = _vel_deriv_xy_lp_filter.update((_vel_xy_lp_filter.getState() - vel_xy_prev) / dt_s);

	} else {
		states.velocity(0) = states.velocity(1) = NAN;
		states.acceleration(0) = states.acceleration(1) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_xy_lp_filter.reset({});
		_vel_xy_notch_filter.reset();
		_vel_deriv_xy_lp_filter.reset({});
	}

	if (PX4_ISFINITE(vehicle_local_position.vz) && vehicle_local_position.v_z_valid) {

		const float vel_z_prev = _vel_z_lp_filter.getState();

		// vel z notch filter, then low pass filter
		states.velocity(2) = _vel_z_lp_filter.update(_vel_z_notch_filter.apply(vehicle_local_position.vz));

		// vel z derivative low pass filter
		states.acceleration(2) = _vel_deriv_z_lp_filter.update((_vel_z_lp_filter.getState() - vel_z_prev) / dt_s);

	} else {
		states.velocity(2) = NAN;
		states.acceleration(2) = NAN;

		// reset filters to prevent acceleration spikes when regaining velocity
		_vel_z_lp_filter.reset({});
		_vel_z_notch_filter.reset();
		_vel_deriv_z_lp_filter.reset({});
	}

	states.yaw = vehicle_local_position.heading;

	return states;
}

void MulticopterPositionControl::Run()
{
	if (should_exit()) {
		_local_pos_sub.unregisterCallback();
		exit_and_cleanup();
		return;
	}

	// reschedule backup
	ScheduleDelayed(100_ms);

	parameters_update(false);

	perf_begin(_cycle_perf);
	vehicle_local_position_s vehicle_local_position;

	if (_local_pos_sub.update(&vehicle_local_position)) {
		const float dt =
			math::constrain(((vehicle_local_position.timestamp_sample - _time_stamp_last_loop) * 1e-6f), 0.002f, 0.04f);
		_time_stamp_last_loop = vehicle_local_position.timestamp_sample;

		_sample_interval_s.update(dt);

		if (_vehicle_control_mode_sub.updated()) {
			const bool previous_position_control_enabled = _vehicle_control_mode.flag_multicopter_position_control_enabled;

			if (_vehicle_control_mode_sub.update(&_vehicle_control_mode)) {
				if (!previous_position_control_enabled && _vehicle_control_mode.flag_multicopter_position_control_enabled) {
					_time_position_control_enabled = _vehicle_control_mode.timestamp;

				} else if (previous_position_control_enabled && !_vehicle_control_mode.flag_multicopter_position_control_enabled) {
					// clear existing setpoint when controller is no longer active
					_setpoint = PositionControl::empty_trajectory_setpoint;
				}
			}
		}

		_vehicle_land_detected_sub.update(&_vehicle_land_detected);

		if (_param_mpc_use_hte.get()) {
			hover_thrust_estimate_s hte;

			if (_hover_thrust_estimate_sub.update(&hte)) {
				if (hte.valid) {
					_control.updateHoverThrust(hte.hover_thrust);
				}
			}
		}

		PositionControlStates states{set_vehicle_states(vehicle_local_position, dt)};

		// if a goto setpoint available this publishes a trajectory setpoint to go there
		if (_goto_control.checkForSetpoint(vehicle_local_position.timestamp_sample,
						   _vehicle_control_mode.flag_multicopter_position_control_enabled)) {
			_goto_control.update(dt, states.position, states.yaw);
		}

		_trajectory_setpoint_sub.update(&_setpoint);
		updateSuspendedLoadJointState();

		adjustSetpointForEKFResets(vehicle_local_position, _setpoint);

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled) {
			// set failsafe setpoint if there hasn't been a new
			// trajectory setpoint since position control started
			if ((_setpoint.timestamp < _time_position_control_enabled)
			    && (vehicle_local_position.timestamp_sample > _time_position_control_enabled)) {

				_setpoint = generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, false);
			}
		}

		if (_vehicle_control_mode.flag_multicopter_position_control_enabled
		    && (_setpoint.timestamp >= _time_position_control_enabled)) {

			// update vehicle constraints and handle smooth takeoff
			_vehicle_constraints_sub.update(&_vehicle_constraints);

			// fix to prevent the takeoff ramp to ramp to a too high value or get stuck because of NAN
			// TODO: this should get obsolete once the takeoff limiting moves into the flight tasks
			if (!PX4_ISFINITE(_vehicle_constraints.speed_up) || (_vehicle_constraints.speed_up > _param_mpc_z_vel_max_up.get())) {
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
			}

			if (_vehicle_control_mode.flag_control_offboard_enabled) {

				const bool want_takeoff = _vehicle_control_mode.flag_armed
							  && (vehicle_local_position.timestamp_sample < _setpoint.timestamp + 1_s);

				if (want_takeoff && PX4_ISFINITE(_setpoint.position[2])
				    && (_setpoint.position[2] < states.position(2))) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.velocity[2])
					   && (_setpoint.velocity[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else if (want_takeoff && PX4_ISFINITE(_setpoint.acceleration[2])
					   && (_setpoint.acceleration[2] < 0.f)) {

					_vehicle_constraints.want_takeoff = true;

				} else {
					_vehicle_constraints.want_takeoff = false;
				}

				// override with defaults
				_vehicle_constraints.speed_up = _param_mpc_z_vel_max_up.get();
				_vehicle_constraints.speed_down = _param_mpc_z_vel_max_dn.get();
			}

			bool skip_takeoff = _param_com_throw_en.get();
			// handle smooth takeoff
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed,
						    _vehicle_constraints.want_takeoff,
						    _vehicle_constraints.speed_up, skip_takeoff, vehicle_local_position.timestamp_sample);

			const bool not_taken_off             = (_takeoff.getTakeoffState() < TakeoffState::rampup);
			const bool flying                    = (_takeoff.getTakeoffState() >= TakeoffState::flight);
			const bool flying_but_ground_contact = (flying && _vehicle_land_detected.ground_contact);
			const bool anti_swing_mode_allowed = !_param_mc_hang_offb.get()
							     || _vehicle_control_mode.flag_control_offboard_enabled;
			_control.setSuspendedLoadAntiSwingFlying(flying && !flying_but_ground_contact && anti_swing_mode_allowed);

			if (!flying) {
				_control.setHoverThrust(_param_mpc_thr_hover.get());
			}

			// make sure takeoff ramp is not amended by acceleration feed-forward
			if (_takeoff.getTakeoffState() == TakeoffState::rampup && PX4_ISFINITE(_setpoint.velocity[2])) {
				_setpoint.acceleration[2] = NAN;
			}

			if (not_taken_off || flying_but_ground_contact) {
				// we are not flying yet and need to avoid any corrections
				_setpoint = PositionControl::empty_trajectory_setpoint;
				_setpoint.timestamp = vehicle_local_position.timestamp_sample;
				Vector3f(0.f, 0.f, 100.f).copyTo(_setpoint.acceleration); // High downwards acceleration to make sure there's no thrust

				// prevent any integrator windup
				_control.resetIntegral();
				_control.resetLadrcPositionControl();
			}

			// limit tilt during takeoff ramupup
			const float tilt_limit_deg = (_takeoff.getTakeoffState() < TakeoffState::flight)
						     ? _param_mpc_tiltmax_lnd.get() : _param_mpc_tiltmax_air.get();
			_control.setTiltLimit(_tilt_limit_slew_rate.update(math::radians(tilt_limit_deg), dt));

			const float speed_up = _takeoff.updateRamp(dt,
					       PX4_ISFINITE(_vehicle_constraints.speed_up) ? _vehicle_constraints.speed_up : _param_mpc_z_vel_max_up.get());
			const float speed_down = PX4_ISFINITE(_vehicle_constraints.speed_down) ? _vehicle_constraints.speed_down :
						 _param_mpc_z_vel_max_dn.get();

			// Allow ramping from zero thrust on takeoff
			const float minimum_thrust = flying ? _param_mpc_thr_min.get() : 0.f;
			_control.setThrustLimits(minimum_thrust, _param_mpc_thr_max.get());

			float max_speed_xy = _param_mpc_xy_vel_max.get();

			if (PX4_ISFINITE(vehicle_local_position.vxy_max)) {
				max_speed_xy = math::min(max_speed_xy, vehicle_local_position.vxy_max);
			}

			_control.setVelocityLimits(
				max_speed_xy,
				math::min(speed_up, _param_mpc_z_vel_max_up.get()), // takeoff ramp starts with negative velocity limit
				math::max(speed_down, 0.f));

			_control.setInputSetpoint(_setpoint);

			// update states
			if (!PX4_ISFINITE(_setpoint.position[2])
			    && PX4_ISFINITE(_setpoint.velocity[2]) && (fabsf(_setpoint.velocity[2]) > FLT_EPSILON)
			    && PX4_ISFINITE(vehicle_local_position.z_deriv) && vehicle_local_position.z_valid && vehicle_local_position.v_z_valid) {
				// A change in velocity is demanded and the altitude is not controlled.
				// Set velocity to the derivative of position
				// because it has less bias but blend it in across the landing speed range
				//  <  MPC_LAND_SPEED: ramp up using altitude derivative without a step
				//  >= MPC_LAND_SPEED: use altitude derivative
				float weighting = fminf(fabsf(_setpoint.velocity[2]) / _param_mpc_land_speed.get(), 1.f);
				states.velocity(2) = vehicle_local_position.z_deriv * weighting + vehicle_local_position.vz * (1.f - weighting);
			}

			if ((!PX4_ISFINITE(_setpoint.velocity[0]) || !PX4_ISFINITE(_setpoint.velocity[1]))
			    && (!PX4_ISFINITE(_setpoint.position[0]) || !PX4_ISFINITE(_setpoint.position[1]))) {
				// Horizontal velocity is not controlled, reset the integrators to avoid
				// over-corrections when starting again.
				_control.resetIntegralXY();
			}

			_control.setState(states);

			const hrt_abstime now = hrt_absolute_time();

			// Run position control
			if (_control.update(dt, now)) {

				// Valid control update - store for fallback
				_last_valid_setpoint = _setpoint;

			} else {

				// Initial update failed - Try fallback if within timeout
				if (now < _last_valid_setpoint.timestamp + 200_ms) {
					// Use last valid setpoint
					adjustSetpointForEKFResets(vehicle_local_position, _last_valid_setpoint);
					_control.setInputSetpoint(_last_valid_setpoint);
				}

				// Still failing / not within timeout - Go to failsafe
				if (!_control.update(dt, now)) {

					_vehicle_constraints = {0, NAN, NAN, false, {}}; // reset constraints

					_control.setInputSetpoint(generateFailsafeSetpoint(vehicle_local_position.timestamp_sample, states, true));
					_control.setVelocityLimits(_param_mpc_xy_vel_max.get(), _param_mpc_z_vel_max_up.get(), _param_mpc_z_vel_max_dn.get());

					_control.update(dt, now);
				}
			}

			publishPositionLadrcStatus();
			publishSuspendedLoadAntiSwingStatus();
			publishSuspendedLoadCoordinationStatus();
			publishSuspendedLoadCoordinationPositionStatus();
			publishLadrcVelocityFeedbackStatus();

			// Publish internal position control setpoints
			// on top of the input/feed-forward setpoints these containt the PID corrections
			// This message is used by other modules (such as Landdetector) to determine vehicle intention.
			vehicle_local_position_setpoint_s local_pos_sp{};
			_control.getLocalPositionSetpoint(local_pos_sp);
			local_pos_sp.timestamp = hrt_absolute_time();
			_local_pos_sp_pub.publish(local_pos_sp);

			// Publish attitude setpoint output
			vehicle_attitude_setpoint_s attitude_setpoint{};
			_control.getAttitudeSetpoint(attitude_setpoint);
			attitude_setpoint.timestamp = hrt_absolute_time();
			_vehicle_attitude_setpoint_pub.publish(attitude_setpoint);

		} else {
			// an update is necessary here because otherwise the takeoff state doesn't get skipped with non-altitude-controlled modes
			_takeoff.updateTakeoffState(_vehicle_control_mode.flag_armed, _vehicle_land_detected.landed, false, 10.f, true,
						    vehicle_local_position.timestamp_sample);
			_control.setSuspendedLoadAntiSwingFlying(false);
			_control.resetIntegral();
			_control.resetLadrcPositionControl();
		}

		// Publish takeoff status
		const uint8_t takeoff_state = static_cast<uint8_t>(_takeoff.getTakeoffState());

		if (takeoff_state != _takeoff_status_pub.get().takeoff_state
		    || !isEqualF(_tilt_limit_slew_rate.getState(), _takeoff_status_pub.get().tilt_limit)) {
			_takeoff_status_pub.get().takeoff_state = takeoff_state;
			_takeoff_status_pub.get().tilt_limit = _tilt_limit_slew_rate.getState();
			_takeoff_status_pub.get().timestamp = hrt_absolute_time();
			_takeoff_status_pub.update();
		}
	}

	perf_end(_cycle_perf);
}

void MulticopterPositionControl::updateSuspendedLoadJointState()
{
	for (auto &subscription : _suspended_load_joint_state_subs) {
		debug_array_s debug_array{};

		if (subscription.update(&debug_array)) {
			SuspendedLoadAntiSwing::JointState joint_state{};

			if (suspended_load_joint_state_bridge::toJointState(debug_array, joint_state)) {
				_control.setSuspendedLoadJointState(joint_state);
			}
		}
	}
}

void MulticopterPositionControl::publishPositionLadrcStatus()
{
	if (!_control.ladrcPositionControlEnabled()) {
		return;
	}

	const LadrcPositionControl &position_ladrc = _control.ladrcPositionControl();
	const Vector3f &velocity_sp_td = position_ladrc.velocitySetpointTD();
	const Vector3f &td_velocity_derivative = position_ladrc.tdVelocityDerivative();
	const Vector3f &acceleration_sp = position_ladrc.accelerationSetpoint();
	const Vector3f &disturbance_compensation = position_ladrc.disturbanceCompensation();
	const Vector3f &observer_input = position_ladrc.observerInput();
	const Vector3f &nominal_position = position_ladrc.nominalPositionControl();
	const Vector3f &nominal_velocity_reference = position_ladrc.nominalVelocityReferenceControl();
	const Vector3f &nominal_velocity_state = position_ladrc.nominalVelocityStateControl();
	const Vector3f &nominal_acceleration_damping = position_ladrc.nominalAccelerationDampingControl();
	const Vector3f &controller_raw = _control.controllerRawAcceleration();
	const Vector3f &final_command = _control.finalAccelerationCommand();
	const Vector3f &applied_acceleration = _control.lastAppliedAcceleration();

	debug_array_s debug_array{};
	debug_array.timestamp = hrt_absolute_time();
	debug_array.id = kPositionLadrcDebugArrayId;
	memset(debug_array.name, 0, sizeof(debug_array.name));
	strncpy(debug_array.name, kPositionLadrcDebugArrayName, sizeof(debug_array.name) - 1);
	debug_array.data[POS_LADRC_TD_VX] = velocity_sp_td(0);
	debug_array.data[POS_LADRC_TD_VY] = velocity_sp_td(1);
	debug_array.data[POS_LADRC_TD_VZ] = velocity_sp_td(2);
	debug_array.data[POS_LADRC_TD_AX] = td_velocity_derivative(0);
	debug_array.data[POS_LADRC_TD_AY] = td_velocity_derivative(1);
	debug_array.data[POS_LADRC_TD_AZ] = td_velocity_derivative(2);
	debug_array.data[POS_LADRC_ACC_X] = acceleration_sp(0);
	debug_array.data[POS_LADRC_ACC_Y] = acceleration_sp(1);
	debug_array.data[POS_LADRC_ACC_Z] = acceleration_sp(2);
	debug_array.data[POS_LADRC_DIST_X] = disturbance_compensation(0);
	debug_array.data[POS_LADRC_DIST_Y] = disturbance_compensation(1);
	debug_array.data[POS_LADRC_DIST_Z] = disturbance_compensation(2);
	debug_array.data[POS_LADRC_ENABLED] = _control.ladrcPositionControlEnabled() ? 1.f : 0.f;
	debug_array.data[POS_LADRC_TD_ENABLED] = _param_mc_pladrc_td_en.get() ? 1.f : 0.f;
	debug_array.data[POS_LADRC_MODE] = static_cast<float>(static_cast<int32_t>(_control.controllerMode()));
	debug_array.data[POS_LADRC_OBS_IN_X] = observer_input(0);
	debug_array.data[POS_LADRC_OBS_IN_Y] = observer_input(1);
	debug_array.data[POS_LADRC_RAW_X] = controller_raw(0);
	debug_array.data[POS_LADRC_RAW_Y] = controller_raw(1);
	debug_array.data[POS_LADRC_FINAL_X] = final_command(0);
	debug_array.data[POS_LADRC_FINAL_Y] = final_command(1);
	debug_array.data[POS_LADRC_APPLIED_X] = applied_acceleration(0);
	debug_array.data[POS_LADRC_APPLIED_Y] = applied_acceleration(1);
	debug_array.data[POS_LADRC_WC_XY] = position_ladrc.controllerBandwidth()(0);
	debug_array.data[POS_LADRC_WO_XY] = position_ladrc.observerBandwidth()(0);
	debug_array.data[POS_LADRC_NOMINAL_POSITION_X] = nominal_position(0);
	debug_array.data[POS_LADRC_NOMINAL_POSITION_Y] = nominal_position(1);
	debug_array.data[POS_LADRC_NOMINAL_VELOCITY_REFERENCE_X] = nominal_velocity_reference(0);
	debug_array.data[POS_LADRC_NOMINAL_VELOCITY_REFERENCE_Y] = nominal_velocity_reference(1);
	debug_array.data[POS_LADRC_NOMINAL_VELOCITY_STATE_X] = nominal_velocity_state(0);
	debug_array.data[POS_LADRC_NOMINAL_VELOCITY_STATE_Y] = nominal_velocity_state(1);
	debug_array.data[POS_LADRC_NOMINAL_ACCELERATION_DAMPING_X] = nominal_acceleration_damping(0);
	debug_array.data[POS_LADRC_NOMINAL_ACCELERATION_DAMPING_Y] = nominal_acceleration_damping(1);
	_position_ladrc_status_pub.publish(debug_array);
}

void MulticopterPositionControl::publishSuspendedLoadAntiSwingStatus()
{
	debug_array_s debug_array{};
	suspended_load_anti_swing_status_bridge::fromStatus(_control.suspendedLoadAntiSwingStatus(), debug_array);
	_suspended_load_anti_swing_status_pub.publish(debug_array);
}

void MulticopterPositionControl::publishSuspendedLoadCoordinationStatus()
{
	debug_array_s debug_array{};
	suspended_load_coordination_status_bridge::fromStatus(_control.suspendedLoadCoordinationStatus(), debug_array);
	_suspended_load_coordination_status_pub.publish(debug_array);
}

void MulticopterPositionControl::publishSuspendedLoadCoordinationPositionStatus()
{
	debug_array_s debug_array{};
	suspended_load_coordination_position_status_bridge::fromStatus(_control.suspendedLoadCoordinationStatus(), debug_array);
	_suspended_load_coordination_position_status_pub.publish(debug_array);
}

void MulticopterPositionControl::publishLadrcVelocityFeedbackStatus()
{
	if (!_control.ladrcPositionControlEnabled()) {
		return;
	}

	const LadrcPositionControl &position_ladrc = _control.ladrcPositionControl();
	debug_array_s debug_array{};
	ladrc_velocity_feedback_status_bridge::fromStatus(_control.suspendedLoadCoordinationStatus(),
			position_ladrc.controllerBandwidth()(0), position_ladrc.observerBandwidth()(0), debug_array);
	_ladrc_velocity_feedback_status_pub.publish(debug_array);
}

trajectory_setpoint_s MulticopterPositionControl::generateFailsafeSetpoint(const hrt_abstime &now,
		const PositionControlStates &states, bool warn)
{
	// rate limit the warnings
	warn = warn && (now - _last_warn) > 2_s;

	if (warn) {
		PX4_WARN("invalid setpoints");
		_last_warn = now;
	}

	trajectory_setpoint_s failsafe_setpoint = PositionControl::empty_trajectory_setpoint;
	failsafe_setpoint.timestamp = now;

	if (Vector2f(states.velocity).isAllFinite()) {
		// don't move along xy
		failsafe_setpoint.velocity[0] = failsafe_setpoint.velocity[1] = 0.f;

		if (warn) {
			PX4_WARN("Failsafe: stop and wait");
		}

	} else {
		// descend with land speed since we can't stop
		failsafe_setpoint.acceleration[0] = failsafe_setpoint.acceleration[1] = 0.f;
		failsafe_setpoint.velocity[2] = _param_mpc_land_speed.get();

		if (warn) {
			PX4_WARN("Failsafe: blind land");
		}
	}

	if (PX4_ISFINITE(states.velocity(2))) {
		// don't move along z if we can stop in all dimensions
		if (!PX4_ISFINITE(failsafe_setpoint.velocity[2])) {
			failsafe_setpoint.velocity[2] = 0.f;
		}

	} else {
		// emergency descend with a bit below hover thrust
		failsafe_setpoint.velocity[2] = NAN;
		failsafe_setpoint.acceleration[2] = .3f;

		if (warn) {
			PX4_WARN("Failsafe: blind descent");
		}
	}

	return failsafe_setpoint;
}

void MulticopterPositionControl::adjustSetpointForEKFResets(const vehicle_local_position_s &vehicle_local_position,
		trajectory_setpoint_s &setpoint)
{
	if ((setpoint.timestamp != 0) && (setpoint.timestamp < vehicle_local_position.timestamp)) {
		if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
			setpoint.velocity[0] += vehicle_local_position.delta_vxy[0];
			setpoint.velocity[1] += vehicle_local_position.delta_vxy[1];
		}

		if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
			setpoint.velocity[2] += vehicle_local_position.delta_vz;
		}

		if (vehicle_local_position.xy_reset_counter != _xy_reset_counter) {
			setpoint.position[0] += vehicle_local_position.delta_xy[0];
			setpoint.position[1] += vehicle_local_position.delta_xy[1];
		}

		if (vehicle_local_position.z_reset_counter != _z_reset_counter) {
			setpoint.position[2] += vehicle_local_position.delta_z;
		}

		if (vehicle_local_position.heading_reset_counter != _heading_reset_counter) {
			setpoint.yaw = wrap_pi(setpoint.yaw + vehicle_local_position.delta_heading);
		}
	}

	if (vehicle_local_position.vxy_reset_counter != _vxy_reset_counter) {
		_vel_xy_lp_filter.reset(_vel_xy_lp_filter.getState() + Vector2f(vehicle_local_position.delta_vxy));
		_vel_xy_notch_filter.reset();
	}

	if (vehicle_local_position.vz_reset_counter != _vz_reset_counter) {
		_vel_z_lp_filter.reset(_vel_z_lp_filter.getState() + vehicle_local_position.delta_vz);
		_vel_z_notch_filter.reset();
	}

	// save latest reset counters
	_vxy_reset_counter = vehicle_local_position.vxy_reset_counter;
	_vz_reset_counter = vehicle_local_position.vz_reset_counter;
	_xy_reset_counter = vehicle_local_position.xy_reset_counter;
	_z_reset_counter = vehicle_local_position.z_reset_counter;
	_heading_reset_counter = vehicle_local_position.heading_reset_counter;
}

int MulticopterPositionControl::task_spawn(int argc, char *argv[])
{
	bool vtol = false;

	if (argc > 1) {
		if (strcmp(argv[1], "vtol") == 0) {
			vtol = true;
		}
	}

	MulticopterPositionControl *instance = new MulticopterPositionControl(vtol);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int MulticopterPositionControl::print_status()
{
	PX4_INFO("MC position outer loop: %s (MC_PLADRC_EN=%d, MC_PLADRC_TD_EN=%d)",
		 PositionControl::controllerModeName(_position_controller_mode),
		 (int)_param_mc_pladrc_en.get(),
		 (int)_param_mc_pladrc_td_en.get());

	return 0;
}

int MulticopterPositionControl::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int MulticopterPositionControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
The controller has two loops: a P loop for position error and a PID loop for velocity error.
Output of the velocity controller is thrust vector that is split to thrust direction
(i.e. rotation matrix for multicopter orientation) and thrust scalar (i.e. multicopter thrust itself).

The default outer-loop implementation is the original PX4 position P plus velocity PID path.
Set MC_PLADRC_EN=0 for PID, 2 for all-axis second-order LADRC, or 3 for
second-order LADRC on X/Y plus original PX4 PID on Z. Value 1 is retired and
falls back to PID. The second-order LADRC uses position, velocity and
disturbance ESO states and directly generates the acceleration correction.
Set MC_PLADRC_TD_EN=1 to shape the LADRC velocity-setpoint input with the
tracking differentiator.

The controller doesn't use Euler angles for its work, they are generated only for more human-friendly control and
logging.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("mc_pos_control", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_ARG("vtol", "VTOL mode", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[])
{
	return MulticopterPositionControl::main(argc, argv);
}
