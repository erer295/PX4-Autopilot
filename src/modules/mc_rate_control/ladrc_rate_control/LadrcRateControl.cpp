/****************************************************************************
 *
 *   LADRC rate controller for PX4 multicopter rate control module.
 *
 *   This file is intended to be used together with LadrcRateControl.hpp under:
 *
 *     PX4-Autopilot/src/modules/mc_rate_control/ladrc_rate_control/
 *
 ****************************************************************************/

#include "LadrcRateControl.hpp"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

constexpr float kMinAbsB0 = 1.0e-3f;
constexpr float kMinBandwidth = 1.0e-2f;
constexpr float kMaxBandwidth = 500.0f;
constexpr float kMinTorqueLimit = 1.0e-3f;
constexpr float kDefaultTorqueLimit = 1.0f;
constexpr float kMaxAngularAccelDamping = 1.0f;

static inline bool isFinite(float value)
{
	return PX4_ISFINITE(value);
}

static inline float safeBandwidth(float value, float fallback)
{
	if (!isFinite(value)) {
		value = fallback;
	}

	return math::constrain(value, kMinBandwidth, kMaxBandwidth);
}

static inline float safeB0(float value, float fallback)
{
	if (!isFinite(value) || fabsf(value) < kMinAbsB0) {
		value = fallback;

		if (!isFinite(value) || fabsf(value) < kMinAbsB0) {
			value = 1.0f;
		}
	}

	return value;
}

static inline float safeTorqueLimit(float value)
{
	if (!isFinite(value) || value < kMinTorqueLimit) {
		value = kDefaultTorqueLimit;
	}

	return math::constrain(value, kMinTorqueLimit, kDefaultTorqueLimit);
}

static inline float safeAngularAccelDamping(float value)
{
	if (!isFinite(value) || value < 0.f) {
		return 0.f;
	}

	return math::constrain(value, 0.f, kMaxAngularAccelDamping);
}

} // namespace

void LadrcRateControl::setLadrcGains(const Vector3f &b0,
				     const Vector3f &wc,
				     const Vector3f &wo)
{
	for (int i = 0; i < 3; i++) {
		_b0(i) = safeB0(b0(i), _b0(i));
		_wc(i) = safeBandwidth(wc(i), _wc(i));
		_wo(i) = safeBandwidth(wo(i), _wo(i));
	}

	updateObserverGains();
}

void LadrcRateControl::setTorqueLimit(const Vector3f &torque_limit)
{
	for (int i = 0; i < 3; i++) {
		_torque_limit(i) = safeTorqueLimit(torque_limit(i));
	}
}

void LadrcRateControl::setAngularAccelDamping(const Vector3f &angular_accel_damping)
{
	for (int i = 0; i < 3; i++) {
		_angular_accel_damping(i) = safeAngularAccelDamping(angular_accel_damping(i));
	}
}

void LadrcRateControl::setSaturationStatus(const Vector<bool, 3> &saturation_positive,
		const Vector<bool, 3> &saturation_negative)
{
	_control_allocator_saturation_positive = saturation_positive;
	_control_allocator_saturation_negative = saturation_negative;
}

void LadrcRateControl::setPositiveSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_positive(axis) = is_saturated;
	}
}

void LadrcRateControl::setNegativeSaturationFlag(size_t axis, bool is_saturated)
{
	if (axis < 3) {
		_control_allocator_saturation_negative(axis) = is_saturated;
	}
}

void LadrcRateControl::setAppliedTorque(const Vector3f &applied_torque)
{
	for (int i = 0; i < 3; i++) {
		// The PX4 torque setpoint interface is normalized. Keep the observer input
		// finite and bounded even if an upstream fault produces an invalid value.
		_u_observer(i) = isFinite(applied_torque(i))
				 ? math::constrain(applied_torque(i), -1.f, 1.f)
				 : 0.f;
	}
}

void LadrcRateControl::initializeBumpless(const Vector3f &rate,
		const Vector3f &rate_sp,
		const Vector3f &applied_torque)
{
	for (int i = 0; i < 3; i++) {
		const float measured_rate = isFinite(rate(i)) ? rate(i) : 0.f;
		const float desired_rate = isFinite(rate_sp(i)) ? rate_sp(i) : measured_rate;
		const float applied = isFinite(applied_torque(i))
				      ? math::constrain(applied_torque(i), -1.f, 1.f)
				      : 0.f;

		// The candidate LADRC output must respect its own initial-flight limit.
		const float limited_output = math::constrain(applied, -_torque_limit(i), _torque_limit(i));
		const float z2_limit = fmaxf(fabsf(_b0(i)) * fmaxf(_torque_limit(i), kMinTorqueLimit),
					     kMinTorqueLimit);

		_z1(i) = measured_rate;
		_z2(i) = math::constrain(_wc(i) * (desired_rate - measured_rate) - _b0(i) * limited_output,
					 -z2_limit, z2_limit);
		_u(i) = limited_output;
		_u_unconstrained(i) = limited_output;
		_output_limited(i) = false;
		_disturbance_compensation(i) = -_z2(i) / _b0(i);
	}

	// The observer model must still see the torque that was physically requested
	// during the previous sample interval, even if it is above the provisional
	// LADRC authority limit used during a blended handover.
	setAppliedTorque(applied_torque);
	_initialized = true;
}

Vector3f LadrcRateControl::update(const Vector3f &rate,
				  const Vector3f &rate_sp,
				  const Vector3f &angular_accel,
				  float dt,
				  bool landed)
{
	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return _u;
	}

	// When landed or maybe landed, do not let the LESO learn ground-contact
	// effects, idle motor behavior, or actuator saturation as airborne disturbance.
	if (!_initialized || landed) {
		reset(rate);
	}

	Vector3f torque_setpoint{};

	for (int i = 0; i < 3; i++) {
		if (!isFinite(rate(i)) || !isFinite(rate_sp(i))) {
			torque_setpoint(i) = 0.f;
			_u_unconstrained(i) = 0.f;
			_output_limited(i) = false;
			_u_observer(i) = 0.f;
			continue;
		}

		const bool saturated_positive = _control_allocator_saturation_positive(i);
		const bool saturated_negative = _control_allocator_saturation_negative(i);
		const bool saturated = saturated_positive || saturated_negative;

		// LESO:
		//   e      = y - z1
		//   z1_dot = z2 + b0 * u_last + beta1 * e
		//   z2_dot = beta2 * e
		const float observer_error = rate(i) - _z1(i);

		const float z1_dot = _z2(i)
				     + _b0(i) * _u_observer(i)
				     + _beta1(i) * observer_error;

		float z2_dot = _beta2(i) * observer_error;

		// If control allocation is saturated, the actually achieved torque may
		// be different from the published setpoint. Freezing z2 avoids learning
		// this actuator limitation as an external disturbance. Do not set the
		// complete torque output to zero: that discontinuity can create chatter.
		if (landed || saturated) {
			z2_dot = 0.f;
		}

		const float z1_new = _z1(i) + dt * z1_dot;
		const float z2_new = _z2(i) + dt * z2_dot;

		if (isFinite(z1_new)) {
			_z1(i) = z1_new;
		}

		if (isFinite(z2_new)) {
			// Since u is normalized torque, a conservative disturbance bound is:
			// |z2| <= |b0| * torque_limit.
			const float z2_limit = fmaxf(fabsf(_b0(i)) * fmaxf(_torque_limit(i), kMinTorqueLimit),
						     kMinTorqueLimit);

			_z2(i) = math::constrain(z2_new, -z2_limit, z2_limit);
		}

		// LADRC control law:
		//   u = (wc * (rate_sp - z1) - z2) / b0
		const float rate_error = rate_sp(i) - _z1(i);
		float u = (_wc(i) * rate_error - _z2(i)) / _b0(i);

		if (_angular_accel_damping(i) > 0.f && isFinite(angular_accel(i))) {
			u -= _angular_accel_damping(i) * angular_accel(i);
		}

		_u_unconstrained(i) = isFinite(u) ? u : 0.f;
		const float constrained_u = math::constrain(_u_unconstrained(i), -_torque_limit(i), _torque_limit(i));
		_output_limited(i) = fabsf(constrained_u - _u_unconstrained(i)) > FLT_EPSILON;
		u = constrained_u;

		if (!isFinite(u)) {
			u = 0.f;
		}

		torque_setpoint(i) = u;

		// _u_observer is intentionally not overwritten here. It is updated by
		// LadrcRateControl::setAppliedTorque() after all external torque
		// processing (yaw LPF and battery scaling) has been applied.

		// Log the disturbance compensation term. This is not a PID integral.
		const float disturbance_compensation = -_z2(i) / _b0(i);
		_disturbance_compensation(i) = isFinite(disturbance_compensation) ? disturbance_compensation : 0.f;
	}

	_u = torque_setpoint;
	return _u;
}

void LadrcRateControl::reset()
{
	_z1.zero();
	_z2.zero();
	_u.zero();
	_u_unconstrained.zero();
	_output_limited = Vector<bool, 3>{};
	_u_observer.zero();
	_disturbance_compensation.zero();

	_initialized = false;
}

void LadrcRateControl::reset(const Vector3f &rate)
{
	for (int i = 0; i < 3; i++) {
		_z1(i) = isFinite(rate(i)) ? rate(i) : 0.f;
	}

	_z2.zero();
	_u.zero();
	_u_unconstrained.zero();
	_output_limited = Vector<bool, 3>{};
	_u_observer.zero();
	_disturbance_compensation.zero();

	_initialized = true;
}

void LadrcRateControl::getRateControlStatus(rate_ctrl_status_s &rate_ctrl_status) const
{
	// In original PX4 PID mode these fields are PID integrators.
	// In LADRC mode, reuse them to log the disturbance compensation torque:
	//
	//   disturbance_compensation = -z2 / b0
	//
	// This makes it possible to check the LESO disturbance compensation
	// directly in ULog through rate_ctrl_status.
	rate_ctrl_status.rollspeed_integ = _disturbance_compensation(0);
	rate_ctrl_status.pitchspeed_integ = _disturbance_compensation(1);
	rate_ctrl_status.yawspeed_integ = _disturbance_compensation(2);
}

void LadrcRateControl::updateObserverGains()
{
	for (int i = 0; i < 3; i++) {
		// First-order LADRC LESO bandwidth parameterization:
		//   beta1 = 2 * wo
		//   beta2 = wo^2
		_beta1(i) = 2.f * _wo(i);
		_beta2(i) = _wo(i) * _wo(i);
	}
}
