/****************************************************************************
 *
 *   LADRC rate controller for PX4 multicopter rate control module.
 *
 *   This file is intended to be used together with LadrcRateControl.cpp under:
 *
 *     PX4-Autopilot/src/modules/mc_rate_control/ladrc_rate_control/
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>

#include <lib/matrix/matrix/math.hpp>
#include <uORB/topics/rate_ctrl_status.h>

/**
 * @brief First-order LADRC controller for PX4 multicopter body-rate control.
 *
 * This class is the separated LADRC alternative to the original PX4 RateControl
 * class inside src/modules/mc_rate_control.
 *
 * For each body-rate axis, the controlled object is simplified as:
 *
 *     rate_dot = f_total + b0 * u
 *
 * where:
 *
 *     rate    : measured body angular rate [rad/s]
 *     u       : normalized body torque command
 *     b0      : nominal input gain
 *     f_total : total disturbance, including wind disturbance, model error,
 *               inertia coupling, motor lag and aerodynamic uncertainty
 *
 * LESO:
 *
 *     e      = rate - z1
 *     z1_dot = z2 + b0 * u_last + beta1 * e
 *     z2_dot = beta2 * e
 *
 * Control law:
 *
 *     u = (wc * (rate_sp - z1) - z2) / b0
 *
 * Bandwidth parameterization:
 *
 *     beta1 = 2 * wo
 *     beta2 = wo * wo
 *
 * This controller keeps the same update style as PX4 v1.16 RateControl:
 *
 *     update(rate, rate_sp, angular_accel, dt, landed)
 *
 * It outputs normalized roll/pitch/yaw torque setpoints.
 */
class LadrcRateControl
{
public:
	LadrcRateControl() = default;
	~LadrcRateControl() = default;

	/**
	 * @brief Set LADRC nominal input gains and bandwidths.
	 *
	 * @param b0 equivalent input gain for roll, pitch and yaw axes
	 * @param wc controller bandwidth for roll, pitch and yaw axes [rad/s]
	 * @param wo observer bandwidth for roll, pitch and yaw axes [rad/s]
	 */
	void setLadrcGains(const matrix::Vector3f &b0,
			   const matrix::Vector3f &wc,
			   const matrix::Vector3f &wo);

	/**
	 * @brief Set angular-acceleration damping gains.
	 *
	 * This term mirrors the original PX4 PID D term and is applied as:
	 *
	 *     u = u_ladrc - d_accel * angular_accel
	 *
	 * Keep it small; it is a fast damping path on top of the LADRC output.
	 */
	void setAngularAccelDamping(const matrix::Vector3f &angular_accel_damping);

	/**
	 * @brief Set normalized torque output limit for each axis.
	 *
	 * Normal PX4 multicopter torque setpoint range is usually [-1, 1].
	 * During initial SITL tests, use a smaller limit such as:
	 *
	 *     roll/pitch: 0.20 ~ 0.40
	 *     yaw       : 0.10 ~ 0.25
	 */
	void setTorqueLimit(const matrix::Vector3f &torque_limit);

	/**
	 * @brief Set actuator saturation status from control allocation feedback.
	 *
	 * Saturation feedback is used to freeze disturbance-state learning while the
	 * allocator cannot realize the requested torque. The full LADRC output is
	 * deliberately not forced to zero on saturation, because that creates a
	 * discontinuous torque command and can excite a limit cycle.
	 */
	void setSaturationStatus(const matrix::Vector<bool, 3> &saturation_positive,
				 const matrix::Vector<bool, 3> &saturation_negative);

	void setPositiveSaturationFlag(size_t axis, bool is_saturated);
	void setNegativeSaturationFlag(size_t axis, bool is_saturated);

	/**
	 * @brief Provide the torque command that was actually published in the
	 * previous cycle.
	 *
	 * Call this after yaw output filtering and any battery scaling/clamping in
	 * MulticopterRateControl. The LESO then predicts the plant with the same
	 * torque command that was sent to the control allocator.
	 */
	void setAppliedTorque(const matrix::Vector3f &applied_torque);

	/**
	 * @brief Initialize the observer for a smooth PID-to-LADRC transition.
	 *
	 * z1 is aligned with the measured rate and z2 is selected so that the first
	 * LADRC torque command is close to the already applied torque command.
	 */
	void initializeBumpless(const matrix::Vector3f &rate,
				const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &applied_torque);

	/**
	 * @brief Run one LADRC control cycle.
	 *
	 * @param rate current body angular rate [rad/s]
	 * @param rate_sp desired body angular rate [rad/s]
	 * @param angular_accel current body angular acceleration [rad/s^2], used for optional damping
	 * @param dt control period [s]
	 * @param landed true when vehicle is landed or maybe landed
	 *
	 * @return normalized body torque setpoint for roll, pitch and yaw
	 */
	matrix::Vector3f update(const matrix::Vector3f &rate,
				const matrix::Vector3f &rate_sp,
				const matrix::Vector3f &angular_accel,
				float dt,
				bool landed);

	/**
	 * @brief Reset all internal states.
	 */
	void reset();

	/**
	 * @brief Reset observer with current measured body rate.
	 *
	 * z1 is initialized to current rate, and z2/output states are cleared.
	 * Use this when disarmed, landed, or restarting SITL. Use
	 * initializeBumpless() for an in-flight controller transition.
	 */
	void reset(const matrix::Vector3f &rate);

	/**
	 * @brief Fill PX4 rate controller status message.
	 *
	 * In original PX4 PID mode:
	 *
	 *     rollspeed_integ / pitchspeed_integ / yawspeed_integ
	 *
	 * represent PID integrator states.
	 *
	 * In LADRC mode, these fields are reused to log the LADRC disturbance
	 * compensation torque:
	 *
	 *     disturbance_compensation = -z2 / b0
	 *
	 * This is useful for checking LESO behavior in ULog.
	 */
	void getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status) const;

private:
	void updateObserverGains();

	// Nominal input gain:
	//   rate_dot = f_total + b0 * u
	matrix::Vector3f _b0{50.f, 50.f, 20.f};

	// Controller bandwidth
	matrix::Vector3f _wc{8.f, 8.f, 4.f};

	// Observer bandwidth
	matrix::Vector3f _wo{30.f, 30.f, 15.f};

	// LESO gains:
	//   beta1 = 2 * wo
	//   beta2 = wo^2
	matrix::Vector3f _beta1{60.f, 60.f, 30.f};
	matrix::Vector3f _beta2{900.f, 900.f, 225.f};

	// Normalized torque limit for roll, pitch and yaw. Conservative initial
	// flight-test defaults reduce the authority of an unvalidated observer.
	matrix::Vector3f _torque_limit{0.35f, 0.35f, 0.20f};

	// Optional angular-acceleration damping, equivalent to the PX4 PID D path.
	matrix::Vector3f _angular_accel_damping{};

	// LESO states
	matrix::Vector3f _z1{};
	matrix::Vector3f _z2{};

	// Last final torque command published to the control allocator, used by
	// the observer during the following sample interval.
	matrix::Vector3f _u_observer{};

	// Last torque output
	matrix::Vector3f _u{};

	// Logged disturbance compensation term:
	//   -z2 / b0
	matrix::Vector3f _disturbance_compensation{};

	// Control allocation saturation feedback
	matrix::Vector<bool, 3> _control_allocator_saturation_positive{};
	matrix::Vector<bool, 3> _control_allocator_saturation_negative{};

	bool _initialized{false};
};
