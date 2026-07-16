/****************************************************************************
 *
 *   LADRC position/velocity controller for PX4 multicopter position control.
 *
 ****************************************************************************/

#pragma once

#include <lib/matrix/matrix/math.hpp>

class LadrcPositionControl
{
public:
	struct Parameters {
		matrix::Vector3f b0{1.f, 1.f, 1.f};
		matrix::Vector3f wc{1.8f, 1.8f, 4.f};
		matrix::Vector3f wo{6.f, 6.f, 10.f};
		matrix::Vector3f acceleration_damping{};
		float velocity_feedback_weight{0.f};
		float horizontal_acceleration_limit{3.f};
		float upward_acceleration_limit{3.f};
		float downward_acceleration_limit{2.f};
		bool td_enabled{true};
		matrix::Vector3f td_bandwidth{2.f, 2.f, 3.f};
		matrix::Vector3f td_acceleration_limit{2.f, 2.f, 2.f};
		float td_damping_ratio{1.f};
	};

	LadrcPositionControl() = default;
	~LadrcPositionControl() = default;

	void setParameters(const Parameters &parameters);
	static void frequencyScheduledXYBandwidths(float controller_bandwidth_base,
			float observer_bandwidth_base,
			float natural_frequency,
			float controller_frequency_ratio,
			float observer_frequency_ratio,
			float observer_minimum_ratio,
			float &controller_bandwidth_effective,
			float &observer_bandwidth_effective);
	void setEnabled(bool enabled);
	bool enabled() const { return _enabled; }
	void setAppliedAcceleration(const matrix::Vector3f &applied_acceleration);

	void initializeSecondOrderBumpless(const matrix::Vector3f &position,
					   const matrix::Vector3f &velocity,
					   const matrix::Vector3f &position_sp,
					   const matrix::Vector3f &velocity_sp,
					   const matrix::Vector3f &applied_acceleration);

	matrix::Vector3f updateSecondOrder(const matrix::Vector3f &position,
					   const matrix::Vector3f &velocity,
					   const matrix::Vector3f &position_sp,
					   const matrix::Vector3f &velocity_sp,
					   const matrix::Vector3f &velocity_dot,
					   float dt,
					   bool reset);

	void reset();

	const matrix::Vector3f &velocitySetpointTD() const { return _velocity_sp_td; }
	const matrix::Vector3f &tdVelocityDerivative() const { return _td_velocity_derivative; }
	const matrix::Vector3f &accelerationSetpoint() const { return _acceleration_sp; }
	const matrix::Vector3f &nominalControl() const { return _nominal_control; }
	const matrix::Vector3f &nominalPositionControl() const { return _nominal_position_control; }
	const matrix::Vector3f &nominalVelocityReferenceControl() const { return _nominal_velocity_reference_control; }
	const matrix::Vector3f &nominalVelocityStateControl() const { return _nominal_velocity_state_control; }
	const matrix::Vector3f &nominalAccelerationDampingControl() const { return _nominal_acceleration_damping_control; }
	const matrix::Vector3f &observerStateZ2() const { return _z2; }
	const matrix::Vector3f &velocityFeedbackSetpoint() const { return _velocity_feedback_setpoint; }
	const matrix::Vector3f &velocityFeedbackMeasurement() const { return _velocity_feedback_measurement; }
	const matrix::Vector3f &velocityFeedbackState() const { return _velocity_feedback_state; }
	const matrix::Vector3f &velocityTrackingControl() const { return _velocity_tracking_control; }
	const matrix::Vector3f &velocityObserverErrorControl() const { return _velocity_observer_error_control; }
	const matrix::Vector3f &velocityTotalControl() const { return _velocity_total_control; }
	const matrix::Vector3f &velocityFeedbackRequestedWeight() const { return _velocity_feedback_requested_weight; }
	const matrix::Vector3f &velocityFeedbackEffectiveWeight() const { return _velocity_feedback_effective_weight; }
	const matrix::Vector3f &velocityFeedbackValid() const { return _velocity_feedback_valid; }
	const matrix::Vector3f &velocityFeedbackFallback() const { return _velocity_feedback_fallback; }
	const matrix::Vector3f &disturbanceCompensationRaw() const { return _disturbance_compensation_raw; }
	const matrix::Vector3f &disturbanceCompensationSelected() const { return _disturbance_compensation_selected; }
	const matrix::Vector3f &disturbanceCompensation() const { return _disturbance_compensation_selected; }
	const matrix::Vector3f &observerStateZ3() const { return _z3; }
	const matrix::Vector3f &observerInput() const { return _u_observer; }
	const matrix::Vector3f &controllerBandwidth() const { return _parameters.wc; }
	const matrix::Vector3f &observerBandwidth() const { return _parameters.wo; }

private:
	void updateObserverGains();
	matrix::Vector3f updateTD(const matrix::Vector3f &velocity_sp,
				  const matrix::Vector3f &velocity,
				  float dt,
				  bool reset_td);
	float accelerationLimit(int axis) const;
	void resetVelocityOnlyAxis(int axis, const matrix::Vector3f &velocity);
	void resetSecondOrderAxis(int axis,
				  const matrix::Vector3f &position,
				  const matrix::Vector3f &velocity,
				  const matrix::Vector3f &position_sp,
				  const matrix::Vector3f &velocity_sp);
	void constrainAcceleration(matrix::Vector3f &acceleration) const;

	Parameters _parameters{};
	matrix::Vector3f _beta1{12.f, 12.f, 20.f};
	matrix::Vector3f _beta2{36.f, 36.f, 100.f};
	matrix::Vector3f _beta1_second_order{18.f, 18.f, 30.f};
	matrix::Vector3f _beta2_second_order{108.f, 108.f, 300.f};
	matrix::Vector3f _beta3_second_order{216.f, 216.f, 1000.f};

	matrix::Vector3f _z1{};
	matrix::Vector3f _z2{};
	matrix::Vector3f _z3{};
	matrix::Vector3f _u_observer{};
	matrix::Vector3f _acceleration_sp{};
	matrix::Vector3f _nominal_control{};
	// Read-only diagnostics for the exact nominal decomposition:
	// u_nominal = u_position + u_velocity_reference + u_velocity_state + u_acceleration_damping.
	matrix::Vector3f _nominal_position_control{};
	matrix::Vector3f _nominal_velocity_reference_control{};
	matrix::Vector3f _nominal_velocity_state_control{};
	matrix::Vector3f _nominal_acceleration_damping_control{};
	// Horizontal second-order velocity-feedback decomposition. These values
	// are diagnostics only; the selected feedback is used by the nominal law.
	matrix::Vector3f _velocity_feedback_setpoint{};
	matrix::Vector3f _velocity_feedback_measurement{};
	matrix::Vector3f _velocity_feedback_state{};
	matrix::Vector3f _velocity_tracking_control{};
	matrix::Vector3f _velocity_observer_error_control{};
	matrix::Vector3f _velocity_total_control{};
	matrix::Vector3f _velocity_feedback_requested_weight{};
	matrix::Vector3f _velocity_feedback_effective_weight{};
	matrix::Vector3f _velocity_feedback_valid{};
	matrix::Vector3f _velocity_feedback_fallback{};
	matrix::Vector3f _disturbance_compensation_raw{};
	matrix::Vector3f _disturbance_compensation_selected{};

	matrix::Vector3f _velocity_sp_td{};
	matrix::Vector3f _td_velocity_derivative{};

	bool _enabled{false};
	bool _initialized{false};
	bool _td_initialized{false};
	bool _second_order_axis_active[3] {};
};
