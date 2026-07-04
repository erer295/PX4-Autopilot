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

#pragma once

#include "ladrc_rate_control/LadrcRateControl.hpp"
#include "rbf_residual_compensation/RbfResidualCompensation.hpp"

#include <lib/rate_control/rate_control.hpp>
#include <lib/mathlib/math/filter/AlphaFilter.hpp>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/systemlib/mavlink_log.h>
#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/actuator_controls_status.h>
#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/control_allocator_status.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rate_ctrl_status.h>
#include <uORB/topics/vehicle_angular_velocity.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>
#include <uORB/topics/vehicle_torque_setpoint.h>

using namespace time_literals;

class MulticopterRateControl : public ModuleBase<MulticopterRateControl>, public ModuleParams, public px4::WorkItem
{
public:
	MulticopterRateControl(bool vtol = false);
	~MulticopterRateControl() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * initialize some vectors/matrices from parameters
	 */
	void parameters_updated();

	void updateRateControllerSelection();

	void updateActuatorControlsStatus(const vehicle_torque_setpoint_s &vehicle_torque_setpoint, float dt);

	RateControl _rate_control; ///< class for rate control calculations
	LadrcRateControl _ladrc_rate_control; ///< LADRC class for rate control calculations
	RbfResidualCompensation _rbf_residual_compensation; ///< RBF residual compensation for LADRC

	uORB::Subscription _actuator_motors_sub{ORB_ID(actuator_motors)};
	uORB::Subscription _battery_status_sub{ORB_ID(battery_status)};
	uORB::Subscription _control_allocator_status_sub{ORB_ID(control_allocator_status)};
	uORB::Subscription _manual_control_setpoint_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_control_mode_sub{ORB_ID(vehicle_control_mode)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_rates_setpoint_sub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::SubscriptionCallbackWorkItem _vehicle_angular_velocity_sub{this, ORB_ID(vehicle_angular_velocity)};

	uORB::Publication<actuator_controls_status_s>	_actuator_controls_status_pub{ORB_ID(actuator_controls_status_0)};
	uORB::PublicationMulti<rate_ctrl_status_s>	_controller_status_pub{ORB_ID(rate_ctrl_status)};
	uORB::Publication<vehicle_rates_setpoint_s>	_vehicle_rates_setpoint_pub{ORB_ID(vehicle_rates_setpoint)};
	uORB::Publication<vehicle_torque_setpoint_s>	_vehicle_torque_setpoint_pub;
	uORB::Publication<vehicle_thrust_setpoint_s>	_vehicle_thrust_setpoint_pub;

	vehicle_control_mode_s	_vehicle_control_mode{};
	vehicle_status_s	_vehicle_status{};

	bool _landed{true};
	bool _maybe_landed{true};

	hrt_abstime _last_run{0};

	perf_counter_t	_loop_perf;			/**< loop duration performance counter */

	// keep setpoint values between updates
	matrix::Vector3f _acro_rate_max;		/**< max attitude rates in acro mode */
	matrix::Vector3f _rates_setpoint{};

	float _battery_status_scale{0.0f};
	matrix::Vector3f _thrust_setpoint{};

	bool _use_ladrc{false};
	bool _use_rbf_ladrc{false};
	bool _rate_controller_selection_initialized{false};
	bool _reported_rbf_en{false};
	bool _reported_rbf_inject_en{false};
	bool _reported_rbf_learn_en{false};
	bool _last_control_cycle_ladrc{false};
	bool _last_control_cycle_rbf_ladrc{false};

	// Last published torque is used to initialize LADRC when switching from PID.
	matrix::Vector3f _last_published_torque{};

	// RBF learning gate state.
	AlphaFilter<matrix::Vector3f> _rbf_rate_error_lpf;
	AlphaFilter<matrix::Vector3f> _rbf_target_lpf;
	AlphaFilter<matrix::Vector3f> _rbf_residual_accel_lpf;
	matrix::Vector3f _rbf_last_rates_setpoint{};
	matrix::Vector3f _rbf_last_rate_error_filtered{};
	matrix::Vector3f _rbf_last_target{};
	matrix::Vector3f _rbf_freeze_time_remaining_s{};
	matrix::Vector<bool, 3> _torque_saturation_positive{};
	matrix::Vector<bool, 3> _torque_saturation_negative{};
	hrt_abstime _rbf_attitude_gate_timestamp{0};
	hrt_abstime _rbf_allocation_gate_timestamp{0};
	hrt_abstime _rbf_actuator_gate_timestamp{0};
	bool _rbf_last_rates_setpoint_valid{false};
	bool _rbf_last_rate_error_sign_valid[3] {};
	bool _rbf_attitude_gate_ok{false};
	bool _rbf_allocation_gate_ok{false};
	bool _rbf_actuator_gate_ok{false};
	uint8_t _rbf_freeze_reason[3] {};

	float _energy_integration_time{0.0f};
	float _control_energy[4] {};

	AlphaFilter<float> _output_lpf_yaw;

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::MC_ROLLRATE_P>) _param_mc_rollrate_p,
		(ParamFloat<px4::params::MC_ROLLRATE_I>) _param_mc_rollrate_i,
		(ParamFloat<px4::params::MC_RR_INT_LIM>) _param_mc_rr_int_lim,
		(ParamFloat<px4::params::MC_ROLLRATE_D>) _param_mc_rollrate_d,
		(ParamFloat<px4::params::MC_ROLLRATE_FF>) _param_mc_rollrate_ff,
		(ParamFloat<px4::params::MC_ROLLRATE_K>) _param_mc_rollrate_k,

		(ParamFloat<px4::params::MC_PITCHRATE_P>) _param_mc_pitchrate_p,
		(ParamFloat<px4::params::MC_PITCHRATE_I>) _param_mc_pitchrate_i,
		(ParamFloat<px4::params::MC_PR_INT_LIM>) _param_mc_pr_int_lim,
		(ParamFloat<px4::params::MC_PITCHRATE_D>) _param_mc_pitchrate_d,
		(ParamFloat<px4::params::MC_PITCHRATE_FF>) _param_mc_pitchrate_ff,
		(ParamFloat<px4::params::MC_PITCHRATE_K>) _param_mc_pitchrate_k,

		(ParamFloat<px4::params::MC_YAWRATE_P>) _param_mc_yawrate_p,
		(ParamFloat<px4::params::MC_YAWRATE_I>) _param_mc_yawrate_i,
		(ParamFloat<px4::params::MC_YR_INT_LIM>) _param_mc_yr_int_lim,
		(ParamFloat<px4::params::MC_YAWRATE_D>) _param_mc_yawrate_d,
		(ParamFloat<px4::params::MC_YAWRATE_FF>) _param_mc_yawrate_ff,
		(ParamFloat<px4::params::MC_YAWRATE_K>) _param_mc_yawrate_k,
		(ParamFloat<px4::params::MC_YAW_TQ_CUTOFF>) _param_mc_yaw_tq_cutoff,

		(ParamFloat<px4::params::MC_ACRO_R_MAX>) _param_mc_acro_r_max,
		(ParamFloat<px4::params::MC_ACRO_P_MAX>) _param_mc_acro_p_max,
		(ParamFloat<px4::params::MC_ACRO_Y_MAX>) _param_mc_acro_y_max,
		(ParamFloat<px4::params::MC_ACRO_EXPO>) _param_mc_acro_expo,			/**< expo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_EXPO_Y>) _param_mc_acro_expo_y,				/**< expo stick curve shape (yaw) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPO>) _param_mc_acro_supexpo,		/**< superexpo stick curve shape (roll & pitch) */
		(ParamFloat<px4::params::MC_ACRO_SUPEXPOY>) _param_mc_acro_supexpoy,		/**< superexpo stick curve shape (yaw) */

		(ParamBool<px4::params::MC_BAT_SCALE_EN>) _param_mc_bat_scale_en,

		// LADRC manual controller selection
		(ParamBool<px4::params::MC_LADRC_EN>) _param_mc_ladrc_en,

		// LADRC equivalent input gain b0
		(ParamFloat<px4::params::MC_LADRC_B0_R>) _param_mc_ladrc_b0_r,
		(ParamFloat<px4::params::MC_LADRC_B0_P>) _param_mc_ladrc_b0_p,
		(ParamFloat<px4::params::MC_LADRC_B0_Y>) _param_mc_ladrc_b0_y,

		// LADRC controller bandwidth wc
		(ParamFloat<px4::params::MC_LADRC_WC_R>) _param_mc_ladrc_wc_r,
		(ParamFloat<px4::params::MC_LADRC_WC_P>) _param_mc_ladrc_wc_p,
		(ParamFloat<px4::params::MC_LADRC_WC_Y>) _param_mc_ladrc_wc_y,

		// LADRC observer bandwidth wo
		(ParamFloat<px4::params::MC_LADRC_WO_R>) _param_mc_ladrc_wo_r,
		(ParamFloat<px4::params::MC_LADRC_WO_P>) _param_mc_ladrc_wo_p,
		(ParamFloat<px4::params::MC_LADRC_WO_Y>) _param_mc_ladrc_wo_y,

		// LADRC normalized torque limit
		(ParamFloat<px4::params::MC_LADRC_LIM_R>) _param_mc_ladrc_lim_r,
		(ParamFloat<px4::params::MC_LADRC_LIM_P>) _param_mc_ladrc_lim_p,
		(ParamFloat<px4::params::MC_LADRC_LIM_Y>) _param_mc_ladrc_lim_y,

		// LADRC angular-acceleration damping, equivalent to PID D path
		(ParamFloat<px4::params::MC_LADRC_D_R>) _param_mc_ladrc_d_r,
		(ParamFloat<px4::params::MC_LADRC_D_P>) _param_mc_ladrc_d_p,
		(ParamFloat<px4::params::MC_LADRC_D_Y>) _param_mc_ladrc_d_y,

		// RBF residual compensation for LADRC
		(ParamBool<px4::params::MC_RBF_EN>) _param_mc_rbf_en,
		(ParamBool<px4::params::MC_RBF_EN_R>) _param_mc_rbf_en_r,
		(ParamBool<px4::params::MC_RBF_EN_P>) _param_mc_rbf_en_p,
		(ParamBool<px4::params::MC_RBF_EN_Y>) _param_mc_rbf_en_y,
		(ParamBool<px4::params::MC_RBF_INJECT_EN>) _param_mc_rbf_inject_en,
		(ParamBool<px4::params::MC_RBF_LEARN_EN>) _param_mc_rbf_learn_en,
		(ParamInt<px4::params::MC_RBF_BASIS>) _param_mc_rbf_basis,
		(ParamInt<px4::params::MC_RBF_IN_DIM>) _param_mc_rbf_in_dim,
		(ParamFloat<px4::params::MC_RBF_WIDTH>) _param_mc_rbf_width,
		(ParamFloat<px4::params::MC_RBF_SPACING>) _param_mc_rbf_spacing,
		(ParamFloat<px4::params::MC_RBF_LR>) _param_mc_rbf_lr,
		(ParamFloat<px4::params::MC_RBF_LEAK>) _param_mc_rbf_leak,
		(ParamFloat<px4::params::MC_RBF_LIM_R>) _param_mc_rbf_lim_r,
		(ParamFloat<px4::params::MC_RBF_LIM_P>) _param_mc_rbf_lim_p,
		(ParamFloat<px4::params::MC_RBF_LIM_Y>) _param_mc_rbf_lim_y,
		(ParamFloat<px4::params::MC_RBF_LPF_ALPHA>) _param_mc_rbf_lpf_alpha,
		(ParamFloat<px4::params::MC_RBF_DU_MAX>) _param_mc_rbf_du_max,
		(ParamFloat<px4::params::MC_RBF_ERR_GAIN>) _param_mc_rbf_err_gain,
		(ParamFloat<px4::params::MC_RBF_ERR_WC>) _param_mc_rbf_err_wc,
		(ParamFloat<px4::params::MC_RBF_TGT_HZ>) _param_mc_rbf_tgt_hz,
		(ParamFloat<px4::params::MC_RBF_RES_HZ>) _param_mc_rbf_res_hz,
		(ParamFloat<px4::params::MC_RBF_E_MIN>) _param_mc_rbf_e_min,
		(ParamFloat<px4::params::MC_RBF_E_MAX>) _param_mc_rbf_e_max,
		(ParamFloat<px4::params::MC_RBF_SPD_MAX>) _param_mc_rbf_spd_max,
		(ParamFloat<px4::params::MC_RBF_E_FILT_HZ>) _param_mc_rbf_e_filt_hz,
		(ParamFloat<px4::params::MC_RBF_FEAT_LIM>) _param_mc_rbf_feat_lim,
		(ParamFloat<px4::params::MC_RBF_E_SCALE_R>) _param_mc_rbf_e_scale_r,
		(ParamFloat<px4::params::MC_RBF_E_SCALE_P>) _param_mc_rbf_e_scale_p,
		(ParamFloat<px4::params::MC_RBF_E_SCALE_Y>) _param_mc_rbf_e_scale_y,
		(ParamFloat<px4::params::MC_RBF_ACC_SC_R>) _param_mc_rbf_acc_sc_r,
		(ParamFloat<px4::params::MC_RBF_ACC_SC_P>) _param_mc_rbf_acc_sc_p,
		(ParamFloat<px4::params::MC_RBF_ACC_SC_Y>) _param_mc_rbf_acc_sc_y,
		(ParamFloat<px4::params::MC_RBF_ACC_MAX>) _param_mc_rbf_acc_max,
		(ParamFloat<px4::params::MC_RBF_FRZ_SGN_T>) _param_mc_rbf_frz_sgn_t,
		(ParamFloat<px4::params::MC_RBF_TGT_JUMP>) _param_mc_rbf_tgt_jump,
		(ParamFloat<px4::params::MC_RBF_FRZ_DEC>) _param_mc_rbf_frz_dec,
		(ParamFloat<px4::params::MC_RBF_FIN_GAIN>) _param_mc_rbf_fin_gain,
		(ParamFloat<px4::params::MC_RBF_INJ_R>) _param_mc_rbf_inj_r,
		(ParamFloat<px4::params::MC_RBF_INJ_P>) _param_mc_rbf_inj_p,
		(ParamFloat<px4::params::MC_RBF_INJ_Y>) _param_mc_rbf_inj_y
	)
};
