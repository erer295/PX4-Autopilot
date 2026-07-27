/****************************************************************************
 *
 *   Copyright (c) 2018 - 2019 PX4 Development Team. All rights reserved.
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
 * @file PositionControl.cpp
 */

#include "PositionControl.hpp"
#include "ControlMath.hpp"
#include <float.h>
#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>
#include <geo/geo.h>

using namespace matrix;

namespace
{

float smoothStepPermission(float value, float lower, float upper)
{
	if (!PX4_ISFINITE(value) || !PX4_ISFINITE(lower) || !PX4_ISFINITE(upper) || upper <= lower) {
		return value >= upper ? 1.f : 0.f;
	}

	const float ratio = math::constrain((value - lower) / (upper - lower), 0.f, 1.f);
	return ratio * ratio * (3.f - 2.f * ratio);
}

} // namespace

const trajectory_setpoint_s PositionControl::empty_trajectory_setpoint = {0, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, {NAN, NAN, NAN}, NAN, NAN};

PositionControl::ControllerMode PositionControl::sanitizeControllerMode(int32_t mode)
{
	switch (mode) {
	case static_cast<int32_t>(ControllerMode::PID):
		return ControllerMode::PID;

	case static_cast<int32_t>(ControllerMode::LadrcSecondOrder):
		return ControllerMode::LadrcSecondOrder;

	case static_cast<int32_t>(ControllerMode::HybridLadrcSecondOrderXY):
		return ControllerMode::HybridLadrcSecondOrderXY;

	default:
		return ControllerMode::PID;
	}
}

const char *PositionControl::controllerModeName(ControllerMode mode)
{
	switch (mode) {
	case ControllerMode::LadrcSecondOrder:
		return "LADRC2";

	case ControllerMode::HybridLadrcSecondOrderXY:
		return "LADRC2-XY+PID-Z";

	case ControllerMode::PID:
	default:
		return "PID";
	}
}

void PositionControl::setVelocityGains(const Vector3f &P, const Vector3f &I, const Vector3f &D)
{
	_gain_vel_p = P;
	_gain_vel_i = I;
	_gain_vel_d = D;
}

void PositionControl::setControllerMode(ControllerMode mode)
{
	if (_controller_mode != mode) {
		_vel_int.zero();
		_last_acc_sp_velocity.zero();
		_ladrc_position_control.reset();
		_last_controller_mode = ControllerMode::PID;
	}

	_controller_mode = mode;
	_ladrc_position_control.setEnabled(mode != ControllerMode::PID);
}

void PositionControl::resetLadrcPositionControl()
{
	_ladrc_position_control.reset();
	_last_acc_sp_velocity.zero();
	_last_applied_acceleration.zero();
	_controller_raw_acceleration.zero();
	_final_acceleration_command.zero();
	_last_controller_mode = ControllerMode::PID;
}

void PositionControl::setVelocityLimits(const float vel_horizontal, const float vel_up, const float vel_down)
{
	_lim_vel_horizontal = vel_horizontal;
	_lim_vel_up = vel_up;
	_lim_vel_down = vel_down;
}

void PositionControl::setThrustLimits(const float min, const float max)
{
	// make sure there's always enough thrust vector length to infer the attitude
	_lim_thr_min = math::max(min, 10e-4f);
	_lim_thr_max = max;
}

void PositionControl::setHorizontalThrustMargin(const float margin)
{
	_lim_thr_xy_margin = margin;
}

void PositionControl::updateHoverThrust(const float hover_thrust_new)
{
	// Given that the equation for thrust is T = a_sp * Th / g - Th
	// with a_sp = desired acceleration, Th = hover thrust and g = gravity constant,
	// we want to find the acceleration that needs to be added to the integrator in order obtain
	// the same thrust after replacing the current hover thrust by the new one.
	// T' = T => a_sp' * Th' / g - Th' = a_sp * Th / g - Th
	// so a_sp' = (a_sp - g) * Th / Th' + g
	// we can then add a_sp' - a_sp to the current integrator to absorb the effect of changing Th by Th'
	const float previous_hover_thrust = _hover_thrust;
	setHoverThrust(hover_thrust_new);

	_vel_int(2) += (_acc_sp(2) - CONSTANTS_ONE_G) * previous_hover_thrust / _hover_thrust
		       + CONSTANTS_ONE_G - _acc_sp(2);
}

void PositionControl::setState(const PositionControlStates &states)
{
	_pos = states.position;
	_vel = states.velocity;
	_yaw = states.yaw;
	_vel_dot = states.acceleration;
}

void PositionControl::setInputSetpoint(const trajectory_setpoint_s &setpoint)
{
	_pos_sp = Vector3f(setpoint.position);
	_vel_sp = Vector3f(setpoint.velocity);
	_acc_sp = Vector3f(setpoint.acceleration);
	_yaw_sp = setpoint.yaw;
	_yawspeed_sp = setpoint.yawspeed;
}

bool PositionControl::update(const float dt, uint64_t now)
{
	_control_timestamp = now;
	bool valid = _inputValid();

	if (valid) {
		if (_controller_mode == ControllerMode::LadrcSecondOrder) {
			_secondOrderLadrcPositionControl(dt);

		} else if (_controller_mode == ControllerMode::HybridLadrcSecondOrderXY) {
			_hybridSecondOrderLadrcXYControl(dt);

		} else {
			_positionControl();
			_velocityControl(dt);
		}

		_yawspeed_sp = PX4_ISFINITE(_yawspeed_sp) ? _yawspeed_sp : 0.f;
		_yaw_sp = PX4_ISFINITE(_yaw_sp) ? _yaw_sp : _yaw; // TODO: better way to disable yaw control
	}

	// There has to be a valid output acceleration and thrust setpoint otherwise something went wrong
	return valid && _acc_sp.isAllFinite() && _thr_sp.isAllFinite();
}

void PositionControl::_positionControl()
{
	// P-position controller
	Vector3f vel_sp_position = (_pos_sp - _pos).emult(_gain_pos_p);
	// Position and feed-forward velocity setpoints or position states being NAN results in them not having an influence
	ControlMath::addIfNotNanVector3f(_vel_sp, vel_sp_position);
	// make sure there are no NAN elements for further reference while constraining
	ControlMath::setZeroIfNanVector3f(vel_sp_position);

	// Constrain horizontal velocity by prioritizing the velocity component along the
	// the desired position setpoint over the feed-forward term.
	_vel_sp.xy() = ControlMath::constrainXY(vel_sp_position.xy(), (_vel_sp - vel_sp_position).xy(), _lim_vel_horizontal);
	// Constrain velocity in z-direction.
	_vel_sp(2) = math::constrain(_vel_sp(2), -_lim_vel_up, _lim_vel_down);
}

void PositionControl::_velocityControl(const float dt)
{
	// Constrain vertical velocity integral
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);

	Vector3f vel_error = _vel_sp - _vel;
	Vector3f acc_sp_velocity{};
	// This path is now exclusively the original PX4 velocity PID. The only
	// selectable LADRC outer-loop paths are second-order modes 2 and 3.
	acc_sp_velocity = vel_error.emult(_gain_vel_p) + _vel_int - _vel_dot.emult(_gain_vel_d);
	_last_controller_mode = ControllerMode::PID;

	Vector3f acc_sp_velocity_logged = acc_sp_velocity;
	ControlMath::setZeroIfNanVector3f(acc_sp_velocity_logged);
	_last_acc_sp_velocity = acc_sp_velocity_logged;
	_controller_raw_acceleration = acc_sp_velocity_logged;

	// No control input from setpoints or corresponding states which are NAN
	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_velocity);

	_accelerationControlAndThrustSaturation(dt, vel_error, true);
}

void PositionControl::_secondOrderLadrcPositionControl(const float dt)
{
	Vector3f velocity_sp_control = _vel_sp;

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_pos_sp(i)) && !PX4_ISFINITE(velocity_sp_control(i))) {
			velocity_sp_control(i) = 0.f;
		}
	}

	if (PX4_ISFINITE(velocity_sp_control(0)) && PX4_ISFINITE(velocity_sp_control(1))) {
		const Vector2f velocity_sp_xy(velocity_sp_control(0), velocity_sp_control(1));
		const float velocity_sp_xy_norm = velocity_sp_xy.norm();

		if (velocity_sp_xy_norm > _lim_vel_horizontal) {
			velocity_sp_control.xy() = velocity_sp_xy / velocity_sp_xy_norm * _lim_vel_horizontal;
		}

	} else {
		for (int i = 0; i < 2; i++) {
			if (PX4_ISFINITE(velocity_sp_control(i))) {
				velocity_sp_control(i) = math::constrain(velocity_sp_control(i),
							 -_lim_vel_horizontal, _lim_vel_horizontal);
			}
		}
	}

	if (PX4_ISFINITE(velocity_sp_control(2))) {
		velocity_sp_control(2) = math::constrain(velocity_sp_control(2), -_lim_vel_up, _lim_vel_down);
	}

	if (_last_controller_mode != ControllerMode::LadrcSecondOrder) {
		_ladrc_position_control.initializeSecondOrderBumpless(_pos, _vel, _pos_sp, velocity_sp_control,
				_last_applied_acceleration);
	}

	Vector3f acc_sp_position = _ladrc_position_control.updateSecondOrder(_pos, _vel, _pos_sp, velocity_sp_control,
				   _vel_dot, dt, false);
	_vel_int.zero();
	_last_controller_mode = ControllerMode::LadrcSecondOrder;

	Vector3f acc_sp_position_logged = acc_sp_position;
	ControlMath::setZeroIfNanVector3f(acc_sp_position_logged);
	_last_acc_sp_velocity = acc_sp_position_logged;
	_controller_raw_acceleration = acc_sp_position_logged;

	for (int i = 0; i < 3; i++) {
		if (PX4_ISFINITE(_pos_sp(i)) && !PX4_ISFINITE(_vel_sp(i))) {
			_vel_sp(i) = velocity_sp_control(i);
		}
	}

	ControlMath::addIfNotNanVector3f(_acc_sp, acc_sp_position);

	_accelerationControlAndThrustSaturation(dt, velocity_sp_control - _vel, false);
}

void PositionControl::_hybridSecondOrderLadrcXYControl(const float dt)
{
	// Preserve the raw horizontal feed-forward. _positionControl() is still
	// used for the complete, stock PX4 Z position-to-velocity path.
	const Vector2f velocity_ff_xy(_vel_sp);
	_positionControl();

	Vector3f position_sp_ladrc{_pos_sp(0), _pos_sp(1), NAN};
	Vector3f velocity_sp_ladrc{velocity_ff_xy(0), velocity_ff_xy(1), NAN};

	for (int i = 0; i < 2; i++) {
		if (PX4_ISFINITE(position_sp_ladrc(i)) && !PX4_ISFINITE(velocity_sp_ladrc(i))) {
			velocity_sp_ladrc(i) = 0.f;
		}
	}

	if (PX4_ISFINITE(velocity_sp_ladrc(0)) && PX4_ISFINITE(velocity_sp_ladrc(1))) {
		const Vector2f velocity_sp_xy(velocity_sp_ladrc);
		const float velocity_sp_xy_norm = velocity_sp_xy.norm();

		if (velocity_sp_xy_norm > _lim_vel_horizontal) {
			velocity_sp_ladrc.xy() = velocity_sp_xy / velocity_sp_xy_norm * _lim_vel_horizontal;
		}
	}

	if (_last_controller_mode != ControllerMode::HybridLadrcSecondOrderXY) {
		_ladrc_position_control.initializeSecondOrderBumpless(_pos, _vel, position_sp_ladrc, velocity_sp_ladrc,
				_last_applied_acceleration);
	}

	const Vector3f acceleration_ladrc = _ladrc_position_control.updateSecondOrder(_pos, _vel, position_sp_ladrc,
					    velocity_sp_ladrc, _vel_dot, dt, false);
	Vector3f velocity_error{};
	velocity_error(2) = _vel_sp(2) - _vel(2);

	// Z must remain the original PX4 position/velocity PID path.  In
	// particular, do not initialise its integrator by cancelling the P/D
	// terms when entering the hybrid mode. During a takeoff trajectory this
	// would make the command equal the previous (hover) acceleration and can
	// suppress the braking acceleration after a large altitude step.
	Vector3f acceleration_control{acceleration_ladrc(0), acceleration_ladrc(1),
				      velocity_error(2) *_gain_vel_p(2) + _vel_int(2) - _vel_dot(2) *_gain_vel_d(2)};
	ControlMath::setZeroIfNanVector3f(acceleration_control);

	_vel_sp.xy() = velocity_sp_ladrc.xy();
	_vel_int.xy() = Vector2f{};
	_vel_int(2) = math::constrain(_vel_int(2), -CONSTANTS_ONE_G, CONSTANTS_ONE_G);
	_last_controller_mode = ControllerMode::HybridLadrcSecondOrderXY;
	_last_acc_sp_velocity = acceleration_control;
	_controller_raw_acceleration = acceleration_control;

	ControlMath::addIfNotNanVector3f(_acc_sp, acceleration_control);
	_accelerationControlAndThrustSaturation(dt, velocity_error, true, false);
	_vel_int.xy() = Vector2f{};
}

void PositionControl::_accelerationControlAndThrustSaturation(const float dt,
		Vector3f vel_error,
		bool update_velocity_integral,
		bool update_horizontal_integral)
{
	_coordination_status = {};
	_coordination_status.timestamp_sample = _control_timestamp;
	_coordination_status.ladrc_nominal_ned = _ladrc_position_control.nominalControl().xy();
	_coordination_status.ladrc_nominal_position_ned = _ladrc_position_control.nominalPositionControl().xy();
	_coordination_status.ladrc_nominal_velocity_reference_ned =
		_ladrc_position_control.nominalVelocityReferenceControl().xy();
	_coordination_status.ladrc_nominal_velocity_state_ned =
		_ladrc_position_control.nominalVelocityStateControl().xy();
	_coordination_status.ladrc_nominal_acceleration_damping_ned =
		_ladrc_position_control.nominalAccelerationDampingControl().xy();
	_coordination_status.ladrc_velocity_setpoint_ned = _ladrc_position_control.velocityFeedbackSetpoint().xy();
	_coordination_status.ladrc_velocity_ekf_ned = _ladrc_position_control.velocityFeedbackMeasurement().xy();
	_coordination_status.ladrc_velocity_z2_ned = _ladrc_position_control.observerStateZ2().xy();
	_coordination_status.ladrc_velocity_feedback_ned = _ladrc_position_control.velocityFeedbackState().xy();
	_coordination_status.ladrc_velocity_tracking_ned = _ladrc_position_control.velocityTrackingControl().xy();
	_coordination_status.ladrc_velocity_observer_error_ned =
		_ladrc_position_control.velocityObserverErrorControl().xy();
	_coordination_status.ladrc_velocity_total_ned = _ladrc_position_control.velocityTotalControl().xy();
	_coordination_status.ladrc_vfb_requested_weight =
		_ladrc_position_control.velocityFeedbackRequestedWeight().xy();
	_coordination_status.ladrc_vfb_effective_weight =
		_ladrc_position_control.velocityFeedbackEffectiveWeight().xy();
	_coordination_status.ladrc_vfb_velocity_valid = _ladrc_position_control.velocityFeedbackValid().xy();
	_coordination_status.ladrc_vfb_fallback = _ladrc_position_control.velocityFeedbackFallback().xy();
	_coordination_status.ladrc_dist_raw_ned = _ladrc_position_control.disturbanceCompensationRaw().xy();
	_coordination_status.ladrc_dist_selected_ned = _ladrc_position_control.disturbanceCompensationSelected().xy();

	Vector2f base_acceleration(_acc_sp);
	bool total_acc_saturated = false;

	// The suspended-load total acceleration budget applies even while the
	// optional anti-swing term is disabled. Otherwise a primary LADRC command
	// can repeatedly tilt the vehicle to its own limit while chasing a swinging
	// load, which defeats the payload-safe acceleration envelope.
	if (_lim_acc_horizontal > FLT_EPSILON && base_acceleration.norm() > _lim_acc_horizontal) {
		base_acceleration = base_acceleration.normalized() * _lim_acc_horizontal;
		total_acc_saturated = true;
	}

	// The HESO scheduler output from the preceding control sample only scales
	// the existing AS damping path. OFF/invalid scheduler state is exactly 1.
	_suspended_load_anti_swing.setGainScheduleScale(
		_suspended_load_frequency_selective_observer.status().gain_scale_applied);
	const Vector2f anti_swing_requested_raw =
		_suspended_load_anti_swing.update(dt, _control_timestamp, _yaw, _suspended_load_anti_swing_flying);
	_coordination_status.anti_swing_requested_ned = anti_swing_requested_raw;
	const SuspendedLoadAntiSwing::Status &anti_swing_status = _suspended_load_anti_swing.status();
	const Vector2f position_error_ned{
		_pos_sp(0) - _pos(0),
		_pos_sp(1) - _pos(1)
	};
	const SuspendedLoadFrequencySelectiveObserver::Status &frequency_selective_status =
		_suspended_load_frequency_selective_observer.update(
			dt, _vel.xy(), _last_applied_acceleration.xy(),
			anti_swing_status.angle_filtered, anti_swing_status.rate_filtered,
			position_error_ned, _yaw, anti_swing_status.rope_length,
			anti_swing_status.energy_per_mass, anti_swing_status.engaged,
			anti_swing_status.measurement_valid && _suspended_load_jerk_valid);
	const bool unified_coordinator = frequency_selective_status.mode
				       == SuspendedLoadFrequencySelectiveObserver::Mode::UnifiedShapingCoordinator;
	Vector2f anti_swing_requested = anti_swing_requested_raw;

	if (unified_coordinator) {
		const float position_error_norm = position_error_ned.norm();
		const float position_permission = 1.f - smoothStepPermission(position_error_norm, 0.05f, 0.12f);
		float acceleration_permission = 1.f;

		if (_lim_acc_horizontal > FLT_EPSILON && anti_swing_requested_raw.norm() > FLT_EPSILON) {
			const float remaining_acceleration = math::max(_lim_acc_horizontal - base_acceleration.norm(), 0.f);
			acceleration_permission = math::constrain(
				remaining_acceleration / anti_swing_requested_raw.norm(), 0.f, 1.f);
		}

	float jerk_permission = 1.f;

	if (_suspended_load_jerk_valid && PX4_ISFINITE(dt) && dt > FLT_EPSILON) {
		const Vector2f base_jerk_vector =
			(base_acceleration - _last_suspended_load_base_acceleration) / dt;
		const float base_jerk = base_jerk_vector.norm();
		jerk_permission = 1.f - smoothStepPermission(base_jerk, 1.5f, 3.f);
	}

		const float permission_target = math::min(position_permission,
					      math::min(acceleration_permission, jerk_permission));
		const float permission_delta = math::constrain(permission_target - _suspended_load_as_permission,
					 -dt, dt);
		_suspended_load_as_permission = math::constrain(_suspended_load_as_permission + permission_delta, 0.f, 1.f);
		anti_swing_requested *= _suspended_load_as_permission;
		_coordination_status.selector_mode = 6;
		_coordination_status.selector_blend = _suspended_load_as_permission;

	} else {
		_suspended_load_as_permission = 1.f;
	}

	const Vector2f frequency_selective_requested = frequency_selective_status.compensation_applied_ned;
	const Vector2f passivity_candidate = base_acceleration + anti_swing_requested + frequency_selective_requested;
	const SuspendedLoadEnergySupervisor::Status &energy_supervisor_status = _suspended_load_energy_supervisor.update(
				dt, passivity_candidate, anti_swing_status.rate_filtered, position_error_ned, _yaw,
				anti_swing_status.rope_length, anti_swing_status.energy_per_mass,
				anti_swing_status.engaged, anti_swing_status.measurement_valid);
	_coordination_status.passivity_shadow_correction_norm = energy_supervisor_status.correction_shadow_ned.norm();
	_coordination_status.passivity_shadow_correction_ned = energy_supervisor_status.correction_shadow_ned;
	_coordination_status.passivity_candidate_acceleration_ned = energy_supervisor_status.candidate_acceleration_ned;
	_coordination_status.passivity_active_correction_ned = energy_supervisor_status.correction_active_ned;
	_coordination_status.passivity_projected_acceleration_ned = energy_supervisor_status.projected_acceleration_ned;
	_coordination_status.passivity_candidate_power = energy_supervisor_status.candidate_power;
	_coordination_status.passivity_positive_power_filtered = energy_supervisor_status.positive_power_filtered;
	_coordination_status.passivity_projected_power_shadow = energy_supervisor_status.projected_power_shadow;
	_coordination_status.passivity_projected_power = energy_supervisor_status.projected_power;
	_coordination_status.passivity_raw_correction_power = energy_supervisor_status.raw_correction_power;
	_coordination_status.passivity_position_limited_power = energy_supervisor_status.position_limited_power;
	_coordination_status.position_error_ned = energy_supervisor_status.position_error_ned;
	_coordination_status.position_direction_ned = energy_supervisor_status.position_direction_ned;
	_coordination_status.position_error_norm = energy_supervisor_status.position_error_ned.norm();
	_coordination_status.passivity_candidate_recovery_component = energy_supervisor_status.candidate_recovery_component;
	_coordination_status.passivity_raw_parallel_component = energy_supervisor_status.raw_parallel_component;
	_coordination_status.passivity_limited_parallel_component = energy_supervisor_status.limited_parallel_component;
	_coordination_status.passivity_perpendicular_component_norm = energy_supervisor_status.perpendicular_component_norm;
	_coordination_status.passivity_position_limiter_ratio = energy_supervisor_status.position_limiter_ratio;
	_coordination_status.passivity_raw_correction_ned = energy_supervisor_status.correction_raw_ned;
	_coordination_status.passivity_position_limited_correction_ned =
		energy_supervisor_status.correction_position_limited_ned;
	_coordination_status.swing_energy_per_mass = energy_supervisor_status.energy_per_mass;
	_coordination_status.passivity_dwell_elapsed = energy_supervisor_status.dwell_elapsed;
	_coordination_status.passivity_mode = static_cast<int32_t>(energy_supervisor_status.mode);
	_coordination_status.passivity_shadow_gate_active = energy_supervisor_status.gate_active;
	_coordination_status.passivity_limit_hit = energy_supervisor_status.correction_limit_hit;
	_coordination_status.passivity_slew_active = energy_supervisor_status.correction_slew_active;
	_coordination_status.passivity_position_limiter_active = energy_supervisor_status.position_limiter_active;
	_coordination_status.base_acceleration_ned = base_acceleration;

	// OFF and SHADOW return a strict zero here. ACTIVE reuses the exact same
	// computed correction and inserts it before the existing total XY envelope.
	const Vector2f passivity_active_requested = energy_supervisor_status.correction_active_ned;
	const Vector2f swing_management_requested = anti_swing_requested + frequency_selective_requested
					   + passivity_active_requested;
	Vector2f combined_acceleration{};
	Vector2f anti_swing_applied = anti_swing_requested;
	Vector2f frequency_selective_applied = frequency_selective_requested;
	Vector2f passivity_active_applied = passivity_active_requested;

	if (unified_coordinator && _lim_acc_horizontal > FLT_EPSILON) {
		// The position controller owns the primary envelope in the unified
		// architecture. Swing management receives only the remaining vector
		// authority, preventing the position loop from fighting a privileged AS
		// request after the permission calculation.
		combined_acceleration = ControlMath::constrainXY(base_acceleration, swing_management_requested,
							  _lim_acc_horizontal);
		const Vector2f swing_management_applied = combined_acceleration - base_acceleration;
		const float management_norm = swing_management_requested.norm();
		const float management_scale = management_norm > FLT_EPSILON
					       ? math::constrain(swing_management_applied.norm() / management_norm, 0.f, 1.f)
					       : 0.f;
		anti_swing_applied = anti_swing_requested * management_scale;
		frequency_selective_applied = frequency_selective_requested * management_scale;
		passivity_active_applied = passivity_active_requested * management_scale;
		total_acc_saturated = total_acc_saturated || management_scale < 1.f - 1e-4f;

	} else if (_lim_acc_horizontal > FLT_EPSILON) {
		if ((base_acceleration + swing_management_requested).norm() > _lim_acc_horizontal) {
			total_acc_saturated = true;
		}

		// Reserve the bounded swing-management request first. If it alone
		// exceeds the envelope, scale AS and ACTIVE together so their relative
		// direction is preserved; otherwise the base controller uses the rest.
		if (swing_management_requested.norm() >= _lim_acc_horizontal) {
			const float management_scale = _lim_acc_horizontal / swing_management_requested.norm();
			anti_swing_applied *= management_scale;
			frequency_selective_applied *= management_scale;
			passivity_active_applied *= management_scale;
			combined_acceleration = anti_swing_applied + frequency_selective_applied + passivity_active_applied;

		} else {
			combined_acceleration = ControlMath::constrainXY(swing_management_requested, base_acceleration,
										  _lim_acc_horizontal);
		}

	} else {
		combined_acceleration = base_acceleration + swing_management_requested;
	}

	_suspended_load_anti_swing.setAppliedAccelerationNed(anti_swing_applied);
	_suspended_load_frequency_selective_observer.setAppliedCompensationNed(frequency_selective_applied,
			anti_swing_status.rate_filtered, position_error_ned, _yaw, anti_swing_status.rope_length);
	_acc_sp.xy() = combined_acceleration;
	_final_acceleration_command = _acc_sp;
	_coordination_status.anti_swing_applied_ned = anti_swing_applied;
	_coordination_status.passivity_active_correction_applied_ned = passivity_active_applied;
	_coordination_status.passivity_active = passivity_active_applied.norm() > FLT_EPSILON;
	_coordination_status.final_command_ned = combined_acceleration;
	_coordination_status.total_acc_saturated = total_acc_saturated;

	if (_suspended_load_jerk_valid && PX4_ISFINITE(dt) && dt > FLT_EPSILON) {
		_coordination_status.base_jerk_ned = (base_acceleration - _last_suspended_load_base_acceleration) / dt;
		_coordination_status.final_jerk_ned = (combined_acceleration - _last_suspended_load_final_acceleration) / dt;
	}

	_suspended_load_jerk_valid = base_acceleration.isAllFinite() && combined_acceleration.isAllFinite();

	if (_suspended_load_jerk_valid) {
		_last_suspended_load_base_acceleration = base_acceleration;
		_last_suspended_load_final_acceleration = combined_acceleration;
	}

	_accelerationControl();

	if (update_velocity_integral) {
		// Integrator anti-windup in vertical direction
		if ((_thr_sp(2) >= -_lim_thr_min && vel_error(2) >= 0.f) ||
		    (_thr_sp(2) <= -_lim_thr_max && vel_error(2) <= 0.f)) {
			vel_error(2) = 0.f;
		}
	}

	// Prioritize vertical control while keeping a horizontal margin
	const Vector2f thrust_sp_xy(_thr_sp);
	const float thrust_sp_xy_norm = thrust_sp_xy.norm();
	const float thrust_max_squared = math::sq(_lim_thr_max);

	// Determine how much vertical thrust is left keeping horizontal margin
	const float allocated_horizontal_thrust = math::min(thrust_sp_xy_norm, _lim_thr_xy_margin);
	const float thrust_z_max_squared = thrust_max_squared - math::sq(allocated_horizontal_thrust);

	// Saturate maximal vertical thrust
	_thr_sp(2) = math::max(_thr_sp(2), -sqrtf(thrust_z_max_squared));

	// Determine how much horizontal thrust is left after prioritizing vertical control
	const float thrust_max_xy_squared = thrust_max_squared - math::sq(_thr_sp(2));
	float thrust_max_xy = 0.f;

	if (thrust_max_xy_squared > 0.f) {
		thrust_max_xy = sqrtf(thrust_max_xy_squared);
	}

	// Saturate thrust in horizontal direction
	if (thrust_sp_xy_norm > thrust_max_xy) {
		_thr_sp.xy() = thrust_sp_xy / thrust_sp_xy_norm * thrust_max_xy;
	}

	if (update_velocity_integral && update_horizontal_integral) {
		// Use tracking Anti-Windup for horizontal direction: during saturation, the integrator is used to unsaturate the output
		// see Anti-Reset Windup for PID controllers, L.Rundqwist, 1990
		const Vector2f acc_sp_xy_produced = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);

		// The produced acceleration can be greater or smaller than the desired acceleration due to the saturations and the actual vertical thrust (computed independently).
		// The ARW loop needs to run if the signal is saturated only.
		if (_acc_sp.xy().norm_squared() > acc_sp_xy_produced.norm_squared()) {
			const float arw_gain = 2.f / _gain_vel_p(0);
			const Vector2f acc_sp_xy = _acc_sp.xy();

			vel_error.xy() = Vector2f(vel_error) - arw_gain * (acc_sp_xy - acc_sp_xy_produced);
		}
	}

	if (update_velocity_integral) {
		if (!update_horizontal_integral) {
			vel_error.xy() = Vector2f{};
		}

		// Make sure integral doesn't get NAN
		ControlMath::setZeroIfNanVector3f(vel_error);
		// Update integral part of velocity control
		_vel_int += vel_error.emult(_gain_vel_i) * dt;

		if (!update_horizontal_integral) {
			_vel_int.xy() = Vector2f{};
		}
	}

	// Reconstruct the acceleration produced by the final saturated thrust and
	// use it as the known input on the next LESO update.
	if (_thr_sp.isAllFinite() && PX4_ISFINITE(_hover_thrust) && _hover_thrust > FLT_EPSILON) {
		Vector3f applied_acceleration{};
		applied_acceleration.xy() = Vector2f(_thr_sp) * (CONSTANTS_ONE_G / _hover_thrust);
		applied_acceleration(2) = (_thr_sp(2) + _hover_thrust) * (CONSTANTS_ONE_G / _hover_thrust);

		if (applied_acceleration.isAllFinite()) {
			_last_applied_acceleration = applied_acceleration;
		}
	}

	_ladrc_position_control.setAppliedAcceleration(_last_applied_acceleration);
	_coordination_status.thrust_reconstructed_ned = _last_applied_acceleration.xy();
	_updateSuspendedLoadCoordinationStatus();
}

float PositionControl::horizontalSwingPower(const Vector2f &acceleration_ned,
		const Vector2f &swing_rate_heading,
		float yaw,
		float rope_length)
{
	if (!acceleration_ned.isAllFinite() || !swing_rate_heading.isAllFinite()
	    || !PX4_ISFINITE(yaw) || !PX4_ISFINITE(rope_length) || rope_length < 0.05f) {
		return NAN;
	}

	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	const Vector2f acceleration_heading{
		yaw_cos * acceleration_ned(0) + yaw_sin * acceleration_ned(1),
		-yaw_sin * acceleration_ned(0) + yaw_cos * acceleration_ned(1)
	};

	return -rope_length * acceleration_heading.dot(swing_rate_heading);
}

void PositionControl::_updateSuspendedLoadCoordinationStatus()
{
	const SuspendedLoadAntiSwing::Status &anti_swing_status = _suspended_load_anti_swing.status();
	_coordination_status.swing_rate_norm = anti_swing_status.rate_filtered.norm();
	_coordination_status.valid = anti_swing_status.measurement_valid
				     && anti_swing_status.rate_filtered.isAllFinite()
				     && PX4_ISFINITE(_yaw)
				     && PX4_ISFINITE(anti_swing_status.rope_length)
				     && anti_swing_status.rope_length >= 0.05f;

	if (!_coordination_status.valid) {
		_coordination_status.power_nominal = NAN;
		_coordination_status.power_nominal_position = NAN;
		_coordination_status.power_nominal_velocity_reference = NAN;
		_coordination_status.power_nominal_velocity_state = NAN;
		_coordination_status.power_nominal_velocity_error = NAN;
		_coordination_status.power_nominal_acceleration_damping = NAN;
		_coordination_status.power_velocity_tracking = NAN;
		_coordination_status.power_velocity_observer_error = NAN;
		_coordination_status.power_velocity_total = NAN;
		_coordination_status.power_dist_raw = NAN;
		_coordination_status.power_dist_selected = NAN;
		_coordination_status.power_anti_requested = NAN;
		_coordination_status.power_anti_applied = NAN;
		_coordination_status.power_passivity_applied = NAN;
		_coordination_status.power_final_command = NAN;
		_coordination_status.power_thrust_reconstructed = NAN;
		_coordination_status.passivity_final_power = NAN;
		return;
	}

	const Vector2f &rate_heading = anti_swing_status.rate_filtered;
	const float rope_length = anti_swing_status.rope_length;
	_coordination_status.power_nominal = horizontalSwingPower(
			_coordination_status.ladrc_nominal_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_nominal_position = horizontalSwingPower(
			_coordination_status.ladrc_nominal_position_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_nominal_velocity_reference = horizontalSwingPower(
			_coordination_status.ladrc_nominal_velocity_reference_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_nominal_velocity_state = horizontalSwingPower(
			_coordination_status.ladrc_nominal_velocity_state_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_nominal_velocity_error = horizontalSwingPower(
			_coordination_status.ladrc_nominal_velocity_reference_ned
			+ _coordination_status.ladrc_nominal_velocity_state_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_nominal_acceleration_damping = horizontalSwingPower(
			_coordination_status.ladrc_nominal_acceleration_damping_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_velocity_tracking = horizontalSwingPower(
			_coordination_status.ladrc_velocity_tracking_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_velocity_observer_error = horizontalSwingPower(
			_coordination_status.ladrc_velocity_observer_error_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_velocity_total = horizontalSwingPower(
			_coordination_status.ladrc_velocity_total_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_dist_raw = horizontalSwingPower(
			_coordination_status.ladrc_dist_raw_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_dist_selected = horizontalSwingPower(
			_coordination_status.ladrc_dist_selected_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_anti_requested = horizontalSwingPower(
			_coordination_status.anti_swing_requested_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_anti_applied = horizontalSwingPower(
			_coordination_status.anti_swing_applied_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_passivity_applied = horizontalSwingPower(
			_coordination_status.passivity_active_correction_applied_ned, rate_heading, _yaw, rope_length);
	_coordination_status.power_final_command = horizontalSwingPower(
			_coordination_status.final_command_ned, rate_heading, _yaw, rope_length);
	_coordination_status.passivity_final_power = _coordination_status.power_final_command;
	_coordination_status.power_thrust_reconstructed = horizontalSwingPower(
			_coordination_status.thrust_reconstructed_ned, rate_heading, _yaw, rope_length);
}

void PositionControl::_accelerationControl()
{
	// Assume standard acceleration due to gravity in vertical direction for attitude generation
	float z_specific_force = -CONSTANTS_ONE_G;

	if (!_decouple_horizontal_and_vertical_acceleration) {
		// Include vertical acceleration setpoint for better horizontal acceleration tracking
		z_specific_force += _acc_sp(2);
	}

	Vector3f body_z = Vector3f(-_acc_sp(0), -_acc_sp(1), -z_specific_force).normalized();
	ControlMath::limitTilt(body_z, Vector3f(0, 0, 1), _lim_tilt);
	// Convert to thrust assuming hover thrust produces standard gravity
	const float thrust_ned_z = _acc_sp(2) * (_hover_thrust / CONSTANTS_ONE_G) - _hover_thrust;
	// Project thrust to planned body attitude
	const float cos_ned_body = (Vector3f(0, 0, 1).dot(body_z));
	const float collective_thrust = math::min(thrust_ned_z / cos_ned_body, -_lim_thr_min);
	_thr_sp = body_z * collective_thrust;
}

bool PositionControl::_inputValid()
{
	bool valid = true;

	// Every axis x, y, z needs to have some setpoint
	for (int i = 0; i <= 2; i++) {
		valid = valid && (PX4_ISFINITE(_pos_sp(i)) || PX4_ISFINITE(_vel_sp(i)) || PX4_ISFINITE(_acc_sp(i)));
	}

	// x and y input setpoints always have to come in pairs
	valid = valid && (PX4_ISFINITE(_pos_sp(0)) == PX4_ISFINITE(_pos_sp(1)));
	valid = valid && (PX4_ISFINITE(_vel_sp(0)) == PX4_ISFINITE(_vel_sp(1)));
	valid = valid && (PX4_ISFINITE(_acc_sp(0)) == PX4_ISFINITE(_acc_sp(1)));

	// For each controlled state the estimate has to be valid
	for (int i = 0; i <= 2; i++) {
		if (PX4_ISFINITE(_pos_sp(i))) {
			valid = valid && PX4_ISFINITE(_pos(i));
		}

		if (PX4_ISFINITE(_vel_sp(i))) {
			valid = valid && PX4_ISFINITE(_vel(i)) && PX4_ISFINITE(_vel_dot(i));
		}
	}

	return valid;
}

void PositionControl::getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const
{
	local_position_setpoint.x = _pos_sp(0);
	local_position_setpoint.y = _pos_sp(1);
	local_position_setpoint.z = _pos_sp(2);
	local_position_setpoint.yaw = _yaw_sp;
	local_position_setpoint.yawspeed = _yawspeed_sp;
	local_position_setpoint.vx = _vel_sp(0);
	local_position_setpoint.vy = _vel_sp(1);
	local_position_setpoint.vz = _vel_sp(2);
	_acc_sp.copyTo(local_position_setpoint.acceleration);
	_thr_sp.copyTo(local_position_setpoint.thrust);
}

void PositionControl::getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const
{
	ControlMath::thrustToAttitude(_thr_sp, _yaw_sp, attitude_setpoint);
	attitude_setpoint.yaw_sp_move_rate = _yawspeed_sp;
}
