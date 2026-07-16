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
 * @file PositionControl.hpp
 *
 * A cascaded position controller for position/velocity control only.
 */

#pragma once

#include "LadrcPositionControl.hpp"

#include <stdint.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>

#include <SuspendedLoadAntiSwing.hpp>
#include <SuspendedLoadEnergySupervisor.hpp>
#include <SuspendedLoadCoordinationStatus.hpp>

struct PositionControlStates {
	matrix::Vector3f position;
	matrix::Vector3f velocity;
	matrix::Vector3f acceleration;
	float yaw;
};

/**
 * 	Core Position-Control for MC.
 * 	This class contains P-controller for position and
 * 	PID-controller for velocity.
 * 	Inputs:
 * 		vehicle position/velocity/yaw
 * 		desired set-point position/velocity/thrust/yaw/yaw-speed
 * 		constraints that are stricter than global limits
 * 	Output
 * 		thrust vector and a yaw-setpoint
 *
 * 	If there is a position and a velocity set-point present, then
 * 	the velocity set-point is used as feed-forward. If feed-forward is
 * 	active, then the velocity component of the P-controller output has
 * 	priority over the feed-forward component.
 *
 * 	A setpoint that is NAN is considered as not set.
 * 	If there is a position/velocity- and thrust-setpoint present, then
 *  the thrust-setpoint is ommitted and recomputed from position-velocity-PID-loop.
 */
class PositionControl
{
public:
	enum class ControllerMode : int32_t {
		PID = 0,
		LadrcSecondOrder = 2,
		HybridLadrcSecondOrderXY = 3,
	};

	PositionControl() = default;
	~PositionControl() = default;

	static ControllerMode sanitizeControllerMode(int32_t mode);
	static const char *controllerModeName(ControllerMode mode);

	/**
	 * Set the position control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 */
	void setPositionGains(const matrix::Vector3f &P) { _gain_pos_p = P; }

	/**
	 * Set the velocity control gains
	 * @param P 3D vector of proportional gains for x,y,z axis
	 * @param I 3D vector of integral gains
	 * @param D 3D vector of derivative gains
	 */
	void setVelocityGains(const matrix::Vector3f &P, const matrix::Vector3f &I, const matrix::Vector3f &D);

	/**
	 * Configure the optional LADRC position/velocity controller.
	 */
	void setLadrcPositionControlParameters(const LadrcPositionControl::Parameters &parameters)
	{
		_ladrc_position_control.setParameters(parameters);
	}

	void setControllerMode(ControllerMode mode);
	ControllerMode controllerMode() const { return _controller_mode; }
	bool ladrcPositionControlEnabled() const { return _controller_mode != ControllerMode::PID; }
	void resetLadrcPositionControl();
	const LadrcPositionControl &ladrcPositionControl() const { return _ladrc_position_control; }
	const matrix::Vector3f &lastAppliedAcceleration() const { return _last_applied_acceleration; }
	const matrix::Vector3f &controllerRawAcceleration() const { return _controller_raw_acceleration; }
	const matrix::Vector3f &finalAccelerationCommand() const { return _final_acceleration_command; }
	const matrix::Vector3f &velocityIntegral() const { return _vel_int; }
	const SuspendedLoadCoordinationStatus &suspendedLoadCoordinationStatus() const { return _coordination_status; }

	/**
	 * Predicted specific swing power from a NED horizontal acceleration.
	 *
	 * The swing rate is expressed in the heading-aligned horizontal frame.
	 * Only yaw is used for the NED-to-heading rotation (small-tilt model).
	 */
	static float horizontalSwingPower(const matrix::Vector2f &acceleration_ned,
					  const matrix::Vector2f &swing_rate_heading,
					  float yaw,
					  float rope_length);

	/**
	 * Set the maximum velocity to execute with feed forward and position control
	 * @param vel_horizontal horizontal velocity limit
	 * @param vel_up upwards velocity limit
	 * @param vel_down downwards velocity limit
	 */
	void setVelocityLimits(const float vel_horizontal, const float vel_up, float vel_down);

	/**
	 * Set the minimum and maximum collective normalized thrust [0,1] that can be output by the controller
	 * @param min minimum thrust e.g. 0.1 or 0
	 * @param max maximum thrust e.g. 0.9 or 1
	 */
	void setThrustLimits(const float min, const float max);

	/**
	 * Set margin that is kept for horizontal control when prioritizing vertical thrust
	 * @param margin of normalized thrust that is kept for horizontal control e.g. 0.3
	 */
	void setHorizontalThrustMargin(const float margin);
	void setHorizontalAccelerationLimit(float limit) { _lim_acc_horizontal = math::max(limit, 0.f); }

	/**
	 * Set the maximum tilt angle in radians the output attitude is allowed to have
	 * @param tilt angle in radians from level orientation
	 */
	void setTiltLimit(const float tilt) { _lim_tilt = tilt; }

	/**
	 * Set the normalized hover thrust
	 * @param hover_thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	 */
	void setHoverThrust(const float hover_thrust) { _hover_thrust = math::constrain(hover_thrust, HOVER_THRUST_MIN, HOVER_THRUST_MAX); }

	/**
	 * Update the hover thrust without immediately affecting the output
	 * by adjusting the integrator. This prevents propagating the dynamics
	 * of the hover thrust signal directly to the output of the controller.
	 */
	void updateHoverThrust(const float hover_thrust_new);

	/**
	 * Pass the current vehicle state to the controller
	 * @param PositionControlStates structure
	 */
	void setState(const PositionControlStates &states);

	/**
	 * Pass the desired setpoints
	 * Note: NAN value means no feed forward/leave state uncontrolled if there's no higher order setpoint.
	 * @param setpoint setpoints including feed-forwards to execute in update()
	 */
	void setInputSetpoint(const trajectory_setpoint_s &setpoint);

	/**
	 * Configure optional suspended-load anti-swing acceleration correction.
	 *
	 * The correction is passive unless enabled and fed with a fresh suspended
	 * load joint state. With the default disabled parameters the original PX4
	 * position-control behavior is unchanged.
	 */
	void setSuspendedLoadAntiSwingParameters(const SuspendedLoadAntiSwing::Parameters &parameters)
	{
		_suspended_load_anti_swing.setParameters(parameters);
	}
	void setSuspendedLoadEnergySupervisorParameters(const SuspendedLoadEnergySupervisor::Parameters &parameters)
	{
		_suspended_load_energy_supervisor.setParameters(parameters);
	}

	void setSuspendedLoadJointState(const SuspendedLoadAntiSwing::JointState &joint_state)
	{
		_suspended_load_anti_swing.setJointState(joint_state);
	}

	void resetSuspendedLoadAntiSwing() { _suspended_load_anti_swing.reset(); }
	void setSuspendedLoadAntiSwingFlying(bool flying) { _suspended_load_anti_swing_flying = flying; }
	const matrix::Vector2f &suspendedLoadAntiSwingAcceleration() const
	{
		return _suspended_load_anti_swing.lastAccelerationNed();
	}

	const SuspendedLoadAntiSwing::Status &suspendedLoadAntiSwingStatus() const
	{
		return _suspended_load_anti_swing.status();
	}

	/**
	 * Apply P-position and PID-velocity controller that updates the member
	 * thrust, yaw- and yawspeed-setpoints.
	 * @see _thr_sp
	 * @see _yaw_sp
	 * @see _yawspeed_sp
	 * @param dt time in seconds since last iteration
	 * @return true if update succeeded and output setpoint is executable, false if not
	 */
	bool update(const float dt, uint64_t now = 0);

	/**
	 * Set the integral term in xy to 0.
	 * @see _vel_int
	 */
	void resetIntegral() { _vel_int.setZero(); }
	void resetIntegralXY() { _vel_int.xy() = matrix::Vector2f(); }

	/**
	 * If set, the tilt setpoint is computed by assuming no vertical acceleration
	 */
	void decoupleHorizontalAndVecticalAcceleration(bool val) { _decouple_horizontal_and_vertical_acceleration = val; }

	/**
	 * Get the controllers output local position setpoint
	 * These setpoints are the ones which were executed on including PID output and feed-forward.
	 * The acceleration or thrust setpoints can be used for attitude control.
	 * @param local_position_setpoint reference to struct to fill up
	 */
	void getLocalPositionSetpoint(vehicle_local_position_setpoint_s &local_position_setpoint) const;

	/**
	 * Get the controllers output attitude setpoint
	 * This attitude setpoint was generated from the resulting acceleration setpoint after position and velocity control.
	 * It needs to be executed by the attitude controller to achieve velocity and position tracking.
	 * @param attitude_setpoint reference to struct to fill up
	 */
	void getAttitudeSetpoint(vehicle_attitude_setpoint_s &attitude_setpoint) const;

	/**
	 * All setpoints are set to NAN (uncontrolled). Timestampt zero.
	 */
	static const trajectory_setpoint_s empty_trajectory_setpoint;

private:
	// The range limits of the hover thrust configuration/estimate
	static constexpr float HOVER_THRUST_MIN = 0.05f;
	static constexpr float HOVER_THRUST_MAX = 0.9f;

	bool _inputValid();

	void _positionControl(); ///< Position proportional control
	void _velocityControl(const float dt); ///< Velocity PID control
	void _secondOrderLadrcPositionControl(const float dt); ///< Position/velocity second-order LADRC control
	void _hybridSecondOrderLadrcXYControl(const float dt); ///< XY second-order LADRC plus original Z PID
	void _accelerationControlAndThrustSaturation(const float dt,
			matrix::Vector3f vel_error,
			bool update_velocity_integral,
			bool update_horizontal_integral = true);
	void _accelerationControl(); ///< Acceleration setpoint processing
	void _updateSuspendedLoadCoordinationStatus();

	// Gains
	matrix::Vector3f _gain_pos_p; ///< Position control proportional gain
	matrix::Vector3f _gain_vel_p; ///< Velocity control proportional gain
	matrix::Vector3f _gain_vel_i; ///< Velocity control integral gain
	matrix::Vector3f _gain_vel_d; ///< Velocity control derivative gain
	LadrcPositionControl _ladrc_position_control{}; ///< optional LADRC replacement for velocity PID

	// Limits
	float _lim_vel_horizontal{}; ///< Horizontal velocity limit with feed forward and position control
	float _lim_vel_up{}; ///< Upwards velocity limit with feed forward and position control
	float _lim_vel_down{}; ///< Downwards velocity limit with feed forward and position control
	float _lim_thr_min{}; ///< Minimum collective thrust allowed as output [-1,0] e.g. -0.9
	float _lim_thr_max{}; ///< Maximum collective thrust allowed as output [-1,0] e.g. -0.1
	float _lim_thr_xy_margin{}; ///< Margin to keep for horizontal control when saturating prioritized vertical thrust
	float _lim_tilt{}; ///< Maximum tilt from level the output attitude is allowed to have
	float _lim_acc_horizontal{100.f}; ///< total horizontal acceleration budget

	float _hover_thrust{}; ///< Thrust [HOVER_THRUST_MIN, HOVER_THRUST_MAX] with which the vehicle hovers not accelerating down or up with level orientation
	bool _decouple_horizontal_and_vertical_acceleration{true}; ///< Ignore vertical acceleration setpoint to remove its effect on the tilt setpoint

	// States
	matrix::Vector3f _pos; /**< current position */
	matrix::Vector3f _vel; /**< current velocity */
	matrix::Vector3f _vel_dot; /**< velocity derivative (replacement for acceleration estimate) */
	matrix::Vector3f _vel_int; /**< integral term of the velocity controller */
	matrix::Vector3f _last_acc_sp_velocity{}; /**< last velocity-loop acceleration correction */
	matrix::Vector3f _last_applied_acceleration{}; /**< acceleration reconstructed from final saturated thrust */
	matrix::Vector3f _controller_raw_acceleration{}; /**< controller correction before anti-swing composition */
	matrix::Vector3f _final_acceleration_command{}; /**< acceleration after anti-swing composition */
	float _yaw{}; /**< current heading */
	uint64_t _control_timestamp{0}; /**< caller-provided monotonic time for anti-swing freshness checks */

	// Setpoints
	matrix::Vector3f _pos_sp; /**< desired position */
	matrix::Vector3f _vel_sp; /**< desired velocity */
	matrix::Vector3f _acc_sp; /**< desired acceleration */
	matrix::Vector3f _thr_sp; /**< desired thrust */
	float _yaw_sp{}; /**< desired heading */
	float _yawspeed_sp{}; /** desired yaw-speed */

	SuspendedLoadAntiSwing _suspended_load_anti_swing{};
	SuspendedLoadEnergySupervisor _suspended_load_energy_supervisor{};
	SuspendedLoadCoordinationStatus _coordination_status{};
	matrix::Vector2f _last_suspended_load_base_acceleration{};
	matrix::Vector2f _last_suspended_load_final_acceleration{};
	bool _suspended_load_jerk_valid{false};
	bool _suspended_load_anti_swing_flying{false};
	ControllerMode _controller_mode{ControllerMode::PID};
	ControllerMode _last_controller_mode{ControllerMode::PID};
};
