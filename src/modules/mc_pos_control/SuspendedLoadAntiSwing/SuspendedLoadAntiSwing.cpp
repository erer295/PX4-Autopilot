/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "SuspendedLoadAntiSwing.hpp"

#include <float.h>
#include <math.h>

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

constexpr float kTwoPi = 6.28318530718f;
constexpr float kMinRopeLength = 0.05f;
constexpr float kMaxRopeLength = 10.f;
constexpr float kMaxAccelerationLimit = 10.f;
constexpr float kMaxAccelerationSlewRate = 100.f;
constexpr float kMaxGain = 100.f;
constexpr float kMaxFilterCutoffHz = 50.f;
constexpr float kMaxTimeoutS = 5.f;
constexpr float kMaxActivationDelayS = 30.f;
constexpr float kMaxActivationStableTimeS = 30.f;
constexpr float kMaxActivationRate = 20.f;
constexpr float kMaxRampTimeS = 30.f;
constexpr float kMaxRearmDelayS = 10.f;
constexpr float kPi = 3.14159265359f;
constexpr uint64_t kUsecPerSecond = 1000000ULL;

} // namespace

void SuspendedLoadAntiSwing::setParameters(const Parameters &parameters)
{
	_parameters.enabled = parameters.enabled;
	_parameters.rope_length = math::constrain(sanitizeFinite(parameters.rope_length, 0.6f), kMinRopeLength, kMaxRopeLength);
	_parameters.angle_gain = math::constrain(sanitizeFinite(parameters.angle_gain, 0.f), 0.f, kMaxGain);
	_parameters.rate_gain = math::constrain(sanitizeFinite(parameters.rate_gain, 1.5f), 0.f, kMaxGain);
	_parameters.acceleration_limit = math::constrain(sanitizeFinite(parameters.acceleration_limit, 0.6f), 0.f,
					   kMaxAccelerationLimit);
	_parameters.acceleration_slew_rate = math::constrain(sanitizeFinite(parameters.acceleration_slew_rate, 2.f), 0.f,
					     kMaxAccelerationSlewRate);
	_parameters.filter_cutoff_hz = math::constrain(sanitizeFinite(parameters.filter_cutoff_hz, 4.f), 0.f,
				      kMaxFilterCutoffHz);
	_parameters.max_angle = math::constrain(sanitizeFinite(parameters.max_angle, 0.8f), 0.f, kPi);
	_parameters.timeout_s = math::constrain(sanitizeFinite(parameters.timeout_s, 0.2f), 0.f, kMaxTimeoutS);
	_parameters.activation_delay = math::constrain(sanitizeFinite(parameters.activation_delay, 3.f), 0.f,
				       kMaxActivationDelayS);
	_parameters.activation_max_angle = math::constrain(sanitizeFinite(parameters.activation_max_angle, 0.05f), 0.f,
					      kPi);
	_parameters.activation_max_rate = math::constrain(sanitizeFinite(parameters.activation_max_rate, 0.08f), 0.f,
					   kMaxActivationRate);
	_parameters.activation_stable_time = math::constrain(sanitizeFinite(parameters.activation_stable_time, 2.f), 0.f,
					     kMaxActivationStableTimeS);
	_parameters.ramp_time = math::constrain(sanitizeFinite(parameters.ramp_time, 2.f), 0.f, kMaxRampTimeS);
	_parameters.safety_angle = math::constrain(sanitizeFinite(parameters.safety_angle, 0.12f), 0.f, kPi);
	_parameters.rearm_delay = math::constrain(sanitizeFinite(parameters.rearm_delay, 1.f), 0.f, kMaxRearmDelayS);
	_parameters.sign_x = parameters.sign_x;
	_parameters.sign_y = parameters.sign_y;

	if (!_parameters.enabled || _parameters.acceleration_limit <= FLT_EPSILON) {
		reset();
	}
}

void SuspendedLoadAntiSwing::setJointState(const JointState &joint_state)
{
	_joint_state = joint_state;

	if (!_joint_state.valid) {
		_active = false;
	}
}

Vector2f SuspendedLoadAntiSwing::update(float dt, uint64_t now, float yaw, bool flying)
{
	_safety_limited = false;

	if (!measurementUsable(now, flying) || !PX4_ISFINITE(dt) || dt <= FLT_EPSILON || !PX4_ISFINITE(yaw)) {
		resetActivation(true);
		updateStatus(now);
		return _last_acceleration_ned;
	}

	if (_flying_since == 0) {
		_flying_since = now;
	}

	const Vector2f angle_body = jointStateToBodyAngle();
	const Vector2f rate_body = jointStateToBodyRate();

	if (!_filter_initialized) {
		_angle_filtered = angle_body;
		_rate_filtered = rate_body;
		_filter_initialized = true;

	} else {
		_angle_filtered = filtered(_angle_filtered, angle_body, dt);
		_rate_filtered = filtered(_rate_filtered, rate_body, dt);
	}

	if (_engaged && safetyLimitExceeded()) {
		safetyDisengage(now);
		updateStatus(now);
		return _last_acceleration_ned;
	}

	if (!_engaged) {
		if (!activationReady(now)) {
			_active = false;
			_last_acceleration_ned.zero();
			_last_ramp_scale = 0.f;
			updateStatus(now);
			return _last_acceleration_ned;
		}

		_engaged = true;
		_engaged_since = now;
		_safety_rearm_since = 0;
		_rearming_after_safety = false;
		_last_acceleration_ned.zero();
	}

	const bool angle_model_valid = _parameters.max_angle <= FLT_EPSILON
				       || _angle_filtered.norm() <= _parameters.max_angle;
	const Vector2f angle_feedback = angle_model_valid ? _angle_filtered : Vector2f{};

	Vector2f acceleration_body =
		(angle_feedback * _parameters.angle_gain + _rate_filtered * _parameters.rate_gain) * _parameters.rope_length;
	acceleration_body = constrainNorm(acceleration_body, _parameters.acceleration_limit);
	_last_ramp_scale = activationRamp(now);
	acceleration_body *= _last_ramp_scale;

	Vector2f acceleration_ned{};
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	acceleration_ned(0) = yaw_cos * acceleration_body(0) - yaw_sin * acceleration_body(1);
	acceleration_ned(1) = yaw_sin * acceleration_body(0) + yaw_cos * acceleration_body(1);

	if (_parameters.acceleration_slew_rate > FLT_EPSILON) {
		const float max_delta = _parameters.acceleration_slew_rate * dt;
		const Vector2f delta = acceleration_ned - _last_acceleration_ned;
		acceleration_ned = _last_acceleration_ned + constrainNorm(delta, max_delta);
	}

	_last_acceleration_ned = constrainNorm(acceleration_ned, _parameters.acceleration_limit);
	_active = _last_acceleration_ned.norm_squared() > FLT_EPSILON;
	updateStatus(now);

	return _last_acceleration_ned;
}

void SuspendedLoadAntiSwing::reset()
{
	_joint_state = JointState{};
	resetActivation(true);
}

void SuspendedLoadAntiSwing::resetActivation(bool reset_flying_since)
{
	_angle_filtered.zero();
	_rate_filtered.zero();
	_last_acceleration_ned.zero();

	if (reset_flying_since) {
		_flying_since = 0;
	}

	_activation_ready_since = 0;
	_engaged_since = 0;
	_safety_rearm_since = 0;
	_last_ramp_scale = 0.f;
	_filter_initialized = false;
	_engaged = false;
	_active = false;
	_rearming_after_safety = false;
	_safety_limited = false;
	updateStatus(0);
}

void SuspendedLoadAntiSwing::safetyDisengage(uint64_t now)
{
	_last_acceleration_ned.zero();
	_activation_ready_since = 0;
	_engaged_since = 0;
	_safety_rearm_since = now;
	_last_ramp_scale = 0.f;
	_engaged = false;
	_active = false;
	_rearming_after_safety = true;
	_safety_limited = true;
}

void SuspendedLoadAntiSwing::updateStatus(uint64_t now)
{
	_status.angle_filtered = _angle_filtered;
	_status.rate_filtered = _rate_filtered;
	_status.acceleration_ned = _last_acceleration_ned;
	_status.timestamp_sample = now;
	_status.ramp_scale = _last_ramp_scale;
	_status.angle_norm = _angle_filtered.norm();
	_status.active = _active;
	_status.engaged = _engaged;
	_status.rearming = _rearming_after_safety;
	_status.safety_limited = _safety_limited;
}

float SuspendedLoadAntiSwing::sanitizeFinite(float value, float fallback)
{
	return PX4_ISFINITE(value) ? value : fallback;
}

Vector2f SuspendedLoadAntiSwing::constrainNorm(const Vector2f &value, float limit)
{
	const float safe_limit = PX4_ISFINITE(limit) ? fmaxf(limit, 0.f) : 0.f;
	const float norm = value.norm();

	if (safe_limit <= FLT_EPSILON || !PX4_ISFINITE(norm)) {
		return Vector2f{};
	}

	if (norm <= safe_limit) {
		return value;
	}

	return value * (safe_limit / norm);
}

float SuspendedLoadAntiSwing::signFromInt(int value)
{
	return value < 0 ? -1.f : 1.f;
}

float SuspendedLoadAntiSwing::elapsedSeconds(uint64_t now, uint64_t since)
{
	if (since == 0 || now <= since) {
		return 0.f;
	}

	return (float)(now - since) / (float)kUsecPerSecond;
}

Vector2f SuspendedLoadAntiSwing::filtered(const Vector2f &previous, const Vector2f &input, float dt) const
{
	if (_parameters.filter_cutoff_hz <= FLT_EPSILON || !PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		return input;
	}

	const float alpha = expf(-kTwoPi * _parameters.filter_cutoff_hz * dt);
	return alpha * previous + (1.f - alpha) * input;
}

float SuspendedLoadAntiSwing::activationRamp(uint64_t now) const
{
	if (_parameters.ramp_time <= FLT_EPSILON) {
		return 1.f;
	}

	return math::constrain(elapsedSeconds(now, _engaged_since) / _parameters.ramp_time, 0.f, 1.f);
}

bool SuspendedLoadAntiSwing::measurementFresh(uint64_t now) const
{
	if (_parameters.timeout_s <= FLT_EPSILON) {
		return true;
	}

	if (_joint_state.timestamp_sample == 0 || now < _joint_state.timestamp_sample) {
		return false;
	}

	const uint64_t timeout_us = (uint64_t)(_parameters.timeout_s * (float)kUsecPerSecond);
	return now - _joint_state.timestamp_sample <= timeout_us;
}

bool SuspendedLoadAntiSwing::measurementUsable(uint64_t now, bool flying) const
{
	return _parameters.enabled
	       && flying
	       && _joint_state.valid
	       && measurementFresh(now)
	       && PX4_ISFINITE(_joint_state.roll_angle)
	       && PX4_ISFINITE(_joint_state.pitch_angle)
	       && PX4_ISFINITE(_joint_state.roll_rate)
	       && PX4_ISFINITE(_joint_state.pitch_rate);
}

bool SuspendedLoadAntiSwing::activationReady(uint64_t now)
{
	const bool rearming = _rearming_after_safety && _safety_rearm_since != 0;
	const uint64_t delay_since = rearming ? _safety_rearm_since : _flying_since;
	const float delay_s = rearming ? _parameters.rearm_delay : _parameters.activation_delay;
	const bool delay_ready = delay_s <= FLT_EPSILON
				 || elapsedSeconds(now, delay_since) >= delay_s;
	const bool angle_ready = _parameters.activation_max_angle <= FLT_EPSILON
				 || _angle_filtered.norm() <= _parameters.activation_max_angle;
	const bool rate_ready = _parameters.activation_max_rate <= FLT_EPSILON
				|| _rate_filtered.norm() <= _parameters.activation_max_rate;

	if (!delay_ready || !angle_ready || !rate_ready) {
		_activation_ready_since = 0;
		return false;
	}

	if (_parameters.activation_stable_time <= FLT_EPSILON) {
		return true;
	}

	if (_activation_ready_since == 0) {
		_activation_ready_since = now;
		return false;
	}

	return elapsedSeconds(now, _activation_ready_since) >= _parameters.activation_stable_time;
}

bool SuspendedLoadAntiSwing::safetyLimitExceeded() const
{
	return _parameters.safety_angle > FLT_EPSILON
	       && _angle_filtered.norm() > _parameters.safety_angle;
}

Vector2f SuspendedLoadAntiSwing::jointStateToBodyAngle() const
{
	// SDF joints are roll about X and pitch about Y. For small angles the
	// pitch joint primarily maps to body X displacement, while roll maps to Y.
	return Vector2f(signFromInt(_parameters.sign_x) * _joint_state.pitch_angle,
			signFromInt(_parameters.sign_y) * _joint_state.roll_angle);
}

Vector2f SuspendedLoadAntiSwing::jointStateToBodyRate() const
{
	return Vector2f(signFromInt(_parameters.sign_x) * _joint_state.pitch_rate,
			signFromInt(_parameters.sign_y) * _joint_state.roll_rate);
}
