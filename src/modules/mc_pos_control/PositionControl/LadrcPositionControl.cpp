/****************************************************************************
 *
 *   LADRC position/velocity controller for PX4 multicopter position control.
 *
 ****************************************************************************/

#include "LadrcPositionControl.hpp"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

constexpr float kMinAbsB0 = 1.0e-3f;
constexpr float kMinBandwidth = 1.0e-2f;
constexpr float kMaxBandwidth = 100.f;
constexpr float kMinAccelerationLimit = 1.0e-3f;
constexpr float kMaxAccelerationLimit = 50.f;
constexpr float kMaxAccelerationDamping = 10.f;
constexpr float kMaxVelocityEstimate = 100.f;

static inline bool isFinite(float value)
{
	return PX4_ISFINITE(value);
}

static inline float safeB0(float value, float fallback)
{
	if (!isFinite(value) || fabsf(value) < kMinAbsB0) {
		value = fallback;

		if (!isFinite(value) || fabsf(value) < kMinAbsB0) {
			value = 1.f;
		}
	}

	return value;
}

static inline float safeBandwidth(float value, float fallback)
{
	if (!isFinite(value)) {
		value = fallback;
	}

	return math::constrain(value, kMinBandwidth, kMaxBandwidth);
}

static inline float safeAccelerationLimit(float value, float fallback)
{
	if (!isFinite(value) || value < kMinAccelerationLimit) {
		value = fallback;
	}

	return math::constrain(value, kMinAccelerationLimit, kMaxAccelerationLimit);
}

static inline float safeAccelerationDamping(float value)
{
	if (!isFinite(value) || value < 0.f) {
		return 0.f;
	}

	return math::constrain(value, 0.f, kMaxAccelerationDamping);
}

} // namespace

void LadrcPositionControl::setParameters(const Parameters &parameters)
{
	for (int i = 0; i < 3; i++) {
		_parameters.b0(i) = safeB0(parameters.b0(i), _parameters.b0(i));
		_parameters.wc(i) = safeBandwidth(parameters.wc(i), _parameters.wc(i));
		_parameters.wo(i) = safeBandwidth(parameters.wo(i), _parameters.wo(i));
		_parameters.acceleration_damping(i) = safeAccelerationDamping(parameters.acceleration_damping(i));
		_parameters.td_bandwidth(i) = safeBandwidth(parameters.td_bandwidth(i), _parameters.td_bandwidth(i));
		_parameters.td_acceleration_limit(i) =
			safeAccelerationLimit(parameters.td_acceleration_limit(i), _parameters.td_acceleration_limit(i));
	}

	_parameters.horizontal_acceleration_limit =
		safeAccelerationLimit(parameters.horizontal_acceleration_limit, _parameters.horizontal_acceleration_limit);
	_parameters.upward_acceleration_limit =
		safeAccelerationLimit(parameters.upward_acceleration_limit, _parameters.upward_acceleration_limit);
	_parameters.downward_acceleration_limit =
		safeAccelerationLimit(parameters.downward_acceleration_limit, _parameters.downward_acceleration_limit);
	_parameters.td_enabled = parameters.td_enabled;
	_parameters.td_damping_ratio = math::constrain(parameters.td_damping_ratio, 0.5f, 2.f);

	updateObserverGains();
}

void LadrcPositionControl::frequencyScheduledXYBandwidths(float controller_bandwidth_base,
		float observer_bandwidth_base,
		float natural_frequency,
		float controller_frequency_ratio,
		float observer_frequency_ratio,
		float observer_minimum_ratio,
		float &controller_bandwidth_effective,
		float &observer_bandwidth_effective)
{
	controller_bandwidth_effective = safeBandwidth(controller_bandwidth_base, 1.8f);
	observer_bandwidth_effective = safeBandwidth(observer_bandwidth_base, 6.f);

	if (!isFinite(natural_frequency) || natural_frequency <= FLT_EPSILON
	    || !isFinite(controller_frequency_ratio) || controller_frequency_ratio <= FLT_EPSILON
	    || !isFinite(observer_frequency_ratio) || observer_frequency_ratio <= FLT_EPSILON
	    || !isFinite(observer_minimum_ratio) || observer_minimum_ratio < 1.f) {
		return;
	}

	controller_bandwidth_effective = math::min(controller_bandwidth_effective,
					 controller_frequency_ratio * natural_frequency);
	observer_bandwidth_effective = math::max(observer_minimum_ratio * controller_bandwidth_effective,
				       math::min(observer_bandwidth_effective,
						       observer_frequency_ratio * natural_frequency));
}

void LadrcPositionControl::setEnabled(bool enabled)
{
	if (_enabled != enabled) {
		reset();
	}

	_enabled = enabled;
}

void LadrcPositionControl::setAppliedAcceleration(const Vector3f &applied_acceleration)
{
	// Feed the LESO with what the vehicle could actually produce after all
	// composition and saturation. Invalid axes retain the last valid input.
	for (int i = 0; i < 3; i++) {
		if (isFinite(applied_acceleration(i))) {
			_u_observer(i) = applied_acceleration(i);
		}
	}
}

void LadrcPositionControl::initializeSecondOrderBumpless(const Vector3f &position,
		const Vector3f &velocity,
		const Vector3f &position_sp,
		const Vector3f &velocity_sp,
		const Vector3f &applied_acceleration)
{
	Vector3f applied_acceleration_safe{};

	for (int i = 0; i < 3; i++) {
		applied_acceleration_safe(i) = isFinite(applied_acceleration(i)) ? applied_acceleration(i) : 0.f;
	}

	for (int i = 0; i < 3; i++) {
		const float measured_position = isFinite(position(i)) ? position(i)
						: (isFinite(position_sp(i)) ? position_sp(i) : 0.f);
		const float measured_velocity = isFinite(velocity(i)) ? velocity(i)
						: (isFinite(velocity_sp(i)) ? velocity_sp(i) : 0.f);
		const float desired_position = isFinite(position_sp(i)) ? position_sp(i) : measured_position;
		const float desired_velocity = isFinite(velocity_sp(i)) ? velocity_sp(i) : 0.f;
		const float kp = _parameters.wc(i) * _parameters.wc(i);
		const float kd = 2.f * _parameters.wc(i);
		const float z3_limit = fmaxf(fabsf(_parameters.b0(i)) * accelerationLimit(i), kMinAccelerationLimit);

		_z1(i) = measured_position;
		_z2(i) = measured_velocity;
		const float reference_acceleration = kp * (desired_position - measured_position)
						     + kd * (desired_velocity - measured_velocity);
		const float bumpless_disturbance = reference_acceleration - _parameters.b0(i) * applied_acceleration_safe(i);

		// A large position step may make the bumpless candidate many times larger
		// than the physical LESO disturbance range. Do not clamp that controller
		// transient into z3: it would be interpreted as a real disturbance and can
		// keep accelerating through the target altitude. Use the candidate only
		// when it is physically representable; otherwise start with zero unknown
		// disturbance and the actual applied acceleration as the known input.
		_z3(i) = isFinite(bumpless_disturbance) && fabsf(bumpless_disturbance) <= z3_limit
			 ? bumpless_disturbance : 0.f;
		_u_observer(i) = applied_acceleration_safe(i);
		_acceleration_sp(i) = applied_acceleration_safe(i);
		_disturbance_compensation(i) = -_z3(i) / _parameters.b0(i);
		_velocity_sp_td(i) = desired_velocity;
		_td_velocity_derivative(i) = 0.f;
		_second_order_axis_active[i] = true;
	}

	_initialized = true;
	_td_initialized = true;
}

Vector3f LadrcPositionControl::updateSecondOrder(const Vector3f &position,
		const Vector3f &velocity,
		const Vector3f &position_sp,
		const Vector3f &velocity_sp,
		const Vector3f &velocity_dot,
		float dt,
		bool reset_observer)
{
	Vector3f acceleration_sp{NAN, NAN, NAN};

	if (!_enabled) {
		return acceleration_sp;
	}

	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return _acceleration_sp;
	}

	if (!_initialized || reset_observer) {
		initializeSecondOrderBumpless(position, velocity, position_sp, velocity_sp, _acceleration_sp);
	}

	Vector3f velocity_sp_target = velocity_sp;

	for (int i = 0; i < 3; i++) {
		if (isFinite(position_sp(i)) && !isFinite(velocity_sp_target(i))) {
			velocity_sp_target(i) = 0.f;
		}
	}

	const Vector3f velocity_sp_control = _parameters.td_enabled
					     ? updateTD(velocity_sp_target, velocity, dt, reset_observer)
					     : velocity_sp_target;

	if (!_parameters.td_enabled) {
		_velocity_sp_td = velocity_sp_target;
		_td_velocity_derivative.zero();
		_td_initialized = false;
	}

	for (int i = 0; i < 3; i++) {
		const bool position_controlled = isFinite(position(i)) && isFinite(position_sp(i));
		const bool velocity_controlled = isFinite(velocity(i)) && isFinite(velocity_sp_control(i));

		if (!position_controlled && !velocity_controlled) {
			acceleration_sp(i) = NAN;
			_u_observer(i) = 0.f;
			_z1(i) = 0.f;
			_z2(i) = 0.f;
			_z3(i) = 0.f;
			_disturbance_compensation(i) = 0.f;
			_second_order_axis_active[i] = false;
			continue;
		}

		if (!position_controlled) {
			if (_second_order_axis_active[i] || !isFinite(_z1(i)) || !isFinite(_z2(i))) {
				resetVelocityOnlyAxis(i, velocity);
			}

			const float observer_error = velocity(i) - _z1(i);
			const float z1_dot = _z2(i)
					     + _parameters.b0(i) * _u_observer(i)
					     + _beta1(i) * observer_error;
			const float z2_dot = _beta2(i) * observer_error;
			const float z1_new = _z1(i) + dt * z1_dot;
			const float z2_new = _z2(i) + dt * z2_dot;

			if (isFinite(z1_new)) {
				_z1(i) = z1_new;
			}

			if (isFinite(z2_new)) {
				const float z2_limit = fmaxf(fabsf(_parameters.b0(i)) * accelerationLimit(i), kMinAccelerationLimit);
				_z2(i) = math::constrain(z2_new, -z2_limit, z2_limit);
			}

			float u = (_parameters.wc(i) * (velocity_sp_control(i) - _z1(i)) - _z2(i)) / _parameters.b0(i);

			if (_parameters.acceleration_damping(i) > 0.f && isFinite(velocity_dot(i))) {
				u -= _parameters.acceleration_damping(i) * velocity_dot(i);
			}

			acceleration_sp(i) = isFinite(u) ? u : 0.f;
			_disturbance_compensation(i) = -_z2(i) / _parameters.b0(i);
			_z3(i) = 0.f;
			_second_order_axis_active[i] = false;
			continue;
		}

		if (!_second_order_axis_active[i]
		    || !isFinite(_z1(i)) || !isFinite(_z2(i)) || !isFinite(_z3(i))) {
			resetSecondOrderAxis(i, position, velocity, position_sp, velocity_sp_control);
		}

		const float observer_error = position(i) - _z1(i);
		const float z1_dot = _z2(i) + _beta1_second_order(i) * observer_error;
		const float z2_dot = _z3(i)
				     + _parameters.b0(i) * _u_observer(i)
				     + _beta2_second_order(i) * observer_error;
		const float z3_dot = _beta3_second_order(i) * observer_error;
		const float z1_new = _z1(i) + dt * z1_dot;
		const float z2_new = _z2(i) + dt * z2_dot;
		const float z3_new = _z3(i) + dt * z3_dot;

		if (isFinite(z1_new)) {
			_z1(i) = z1_new;
		}

		if (isFinite(z2_new)) {
			_z2(i) = math::constrain(z2_new, -kMaxVelocityEstimate, kMaxVelocityEstimate);
		}

		if (isFinite(z3_new)) {
			const float z3_limit = fmaxf(fabsf(_parameters.b0(i)) * accelerationLimit(i), kMinAccelerationLimit);
			_z3(i) = math::constrain(z3_new, -z3_limit, z3_limit);
		}

		const float desired_velocity = velocity_controlled ? velocity_sp_control(i) : 0.f;
		const float kp = _parameters.wc(i) * _parameters.wc(i);
		const float kd = 2.f * _parameters.wc(i);
		float u = (kp * (position_sp(i) - _z1(i))
			   + kd * (desired_velocity - _z2(i))
			   - _z3(i)) / _parameters.b0(i);

		if (_parameters.acceleration_damping(i) > 0.f && isFinite(velocity_dot(i))) {
			u -= _parameters.acceleration_damping(i) * velocity_dot(i);
		}

		acceleration_sp(i) = isFinite(u) ? u : 0.f;
		_disturbance_compensation(i) = -_z3(i) / _parameters.b0(i);
		_second_order_axis_active[i] = true;
	}

	constrainAcceleration(acceleration_sp);

	for (int i = 0; i < 3; i++) {
		_u_observer(i) = isFinite(acceleration_sp(i)) ? acceleration_sp(i) : 0.f;
	}

	_acceleration_sp = acceleration_sp;
	return _acceleration_sp;
}

void LadrcPositionControl::reset()
{
	_z1.zero();
	_z2.zero();
	_z3.zero();
	_u_observer.zero();
	_acceleration_sp.zero();
	_disturbance_compensation.zero();
	_velocity_sp_td.zero();
	_td_velocity_derivative.zero();
	_initialized = false;
	_td_initialized = false;

	for (int i = 0; i < 3; i++) {
		_second_order_axis_active[i] = false;
	}
}

void LadrcPositionControl::updateObserverGains()
{
	for (int i = 0; i < 3; i++) {
		_beta1(i) = 2.f * _parameters.wo(i);
		_beta2(i) = _parameters.wo(i) * _parameters.wo(i);
		_beta1_second_order(i) = 3.f * _parameters.wo(i);
		_beta2_second_order(i) = 3.f * _parameters.wo(i) * _parameters.wo(i);
		_beta3_second_order(i) = _parameters.wo(i) * _parameters.wo(i) * _parameters.wo(i);
	}
}

Vector3f LadrcPositionControl::updateTD(const Vector3f &velocity_sp,
					const Vector3f &velocity,
					float dt,
					bool reset_td)
{
	if (reset_td || !_td_initialized || !PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		for (int i = 0; i < 3; i++) {
			_velocity_sp_td(i) = isFinite(velocity_sp(i)) ? velocity_sp(i) : NAN;
			_td_velocity_derivative(i) = 0.f;
		}

		_td_initialized = true;
		return _velocity_sp_td;
	}

	for (int i = 0; i < 3; i++) {
		if (!isFinite(velocity_sp(i))) {
			_velocity_sp_td(i) = NAN;
			_td_velocity_derivative(i) = 0.f;
			continue;
		}

		if (!isFinite(_velocity_sp_td(i)) || !isFinite(_td_velocity_derivative(i))) {
			_velocity_sp_td(i) = isFinite(velocity(i)) ? velocity(i) : velocity_sp(i);
			_td_velocity_derivative(i) = 0.f;
		}

		const float w_td = _parameters.td_bandwidth(i);
		const float accel_limit = _parameters.td_acceleration_limit(i);
		const float v2_dot = -2.f * _parameters.td_damping_ratio * w_td * _td_velocity_derivative(i)
				     + w_td * w_td * (velocity_sp(i) - _velocity_sp_td(i));
		const float v2_new = _td_velocity_derivative(i) + dt * v2_dot;

		_td_velocity_derivative(i) = isFinite(v2_new)
					     ? math::constrain(v2_new, -accel_limit, accel_limit)
					     : 0.f;

		const float v1_new = _velocity_sp_td(i) + dt * _td_velocity_derivative(i);
		_velocity_sp_td(i) = isFinite(v1_new) ? v1_new : velocity_sp(i);
	}

	return _velocity_sp_td;
}

float LadrcPositionControl::accelerationLimit(int axis) const
{
	return (axis == 2)
	       ? fmaxf(_parameters.upward_acceleration_limit, _parameters.downward_acceleration_limit)
	       : _parameters.horizontal_acceleration_limit;
}

void LadrcPositionControl::resetVelocityOnlyAxis(int axis, const Vector3f &velocity)
{
	_z1(axis) = isFinite(velocity(axis)) ? velocity(axis) : 0.f;
	_z2(axis) = 0.f;
	_z3(axis) = 0.f;
	_u_observer(axis) = 0.f;
	_disturbance_compensation(axis) = 0.f;
	_second_order_axis_active[axis] = false;
}

void LadrcPositionControl::resetSecondOrderAxis(int axis,
		const Vector3f &position,
		const Vector3f &velocity,
		const Vector3f &position_sp,
		const Vector3f &velocity_sp)
{
	const float measured_position = isFinite(position(axis)) ? position(axis)
					: (isFinite(position_sp(axis)) ? position_sp(axis) : 0.f);
	const float measured_velocity = isFinite(velocity(axis)) ? velocity(axis)
					: (isFinite(velocity_sp(axis)) ? velocity_sp(axis) : 0.f);

	_z1(axis) = measured_position;
	_z2(axis) = measured_velocity;
	_z3(axis) = 0.f;
	_u_observer(axis) = 0.f;
	_disturbance_compensation(axis) = 0.f;
	_second_order_axis_active[axis] = true;
}

void LadrcPositionControl::constrainAcceleration(Vector3f &acceleration) const
{
	if (isFinite(acceleration(0)) && isFinite(acceleration(1))) {
		const Vector2f acceleration_xy(acceleration(0), acceleration(1));
		const float acceleration_xy_norm = acceleration_xy.norm();

		if (acceleration_xy_norm > _parameters.horizontal_acceleration_limit) {
			acceleration.xy() = acceleration_xy / acceleration_xy_norm * _parameters.horizontal_acceleration_limit;
		}

	} else {
		for (int i = 0; i < 2; i++) {
			if (isFinite(acceleration(i))) {
				acceleration(i) = math::constrain(acceleration(i),
								  -_parameters.horizontal_acceleration_limit,
								  _parameters.horizontal_acceleration_limit);
			}
		}
	}

	if (isFinite(acceleration(2))) {
		acceleration(2) = math::constrain(acceleration(2),
						  -_parameters.upward_acceleration_limit,
						  _parameters.downward_acceleration_limit);
	}
}
