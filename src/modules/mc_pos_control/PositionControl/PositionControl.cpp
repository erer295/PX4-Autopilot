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
	Vector2f base_acceleration(_acc_sp);

	// The suspended-load total acceleration budget applies even while the
	// optional anti-swing term is disabled. Otherwise a primary LADRC command
	// can repeatedly tilt the vehicle to its own limit while chasing a swinging
	// load, which defeats the payload-safe acceleration envelope.
	if (_lim_acc_horizontal > FLT_EPSILON && base_acceleration.norm() > _lim_acc_horizontal) {
		base_acceleration = base_acceleration.normalized() * _lim_acc_horizontal;
	}

	const Vector2f anti_swing_requested =
		_suspended_load_anti_swing.update(dt, _control_timestamp, _yaw, _suspended_load_anti_swing_flying);
	Vector2f combined_acceleration{};
	Vector2f anti_swing_applied = anti_swing_requested;

	if (_lim_acc_horizontal > FLT_EPSILON) {
		// Once energy damping is active, reserve its bounded correction first.
		// Giving an already-saturated primary LADRC command absolute priority
		// leaves no actuator authority for the damping term, so the load energy
		// cannot decrease even though the anti-swing controller is engaged.
		if (anti_swing_requested.norm() >= _lim_acc_horizontal) {
			anti_swing_applied = anti_swing_requested.normalized() * _lim_acc_horizontal;
			combined_acceleration = anti_swing_applied;

		} else {
			combined_acceleration = ControlMath::constrainXY(anti_swing_requested, base_acceleration,
										  _lim_acc_horizontal);
		}

	} else {
		combined_acceleration = base_acceleration + anti_swing_requested;
	}

	_suspended_load_anti_swing.setAppliedAccelerationNed(anti_swing_applied);
	_acc_sp.xy() = combined_acceleration;
	_final_acceleration_command = _acc_sp;

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
