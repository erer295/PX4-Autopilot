/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "SuspendedLoadAntiSwing.hpp"

#include <float.h>
#include <math.h>

#include <lib/geo/geo.h>
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
constexpr float kMaxDampingRatio = 2.f;
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

	switch (parameters.mode) {
	case Mode::Off:
	case Mode::LegacyPD:
	case Mode::EnergyDamping:
		_parameters.mode = parameters.mode;
		break;

	default:
		_parameters.mode = Mode::Off;
		break;
	}

	_parameters.rope_length = math::constrain(sanitizeFinite(parameters.rope_length, 0.6f), kMinRopeLength, kMaxRopeLength);
	_parameters.angle_gain = math::constrain(sanitizeFinite(parameters.angle_gain, 0.f), 0.f, kMaxGain);
	_parameters.rate_gain = math::constrain(sanitizeFinite(parameters.rate_gain, 1.5f), 0.f, kMaxGain);
	_parameters.energy_damping_ratio = math::constrain(sanitizeFinite(parameters.energy_damping_ratio, 0.25f), 0.f,
					   kMaxDampingRatio);
	_parameters.energy_gate_start = math::max(sanitizeFinite(parameters.energy_gate_start, 0.f), 0.f);
	_parameters.energy_gate_full = math::max(sanitizeFinite(parameters.energy_gate_full, 0.02f), 0.f);
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
	_parameters.abort_angle = math::constrain(sanitizeFinite(parameters.abort_angle, 0.8f), 0.f, kPi);
	_parameters.rearm_delay = math::constrain(sanitizeFinite(parameters.rearm_delay, 1.f), 0.f, kMaxRearmDelayS);
	_parameters.sign_x = parameters.sign_x;
	_parameters.sign_y = parameters.sign_y;

	if (!_parameters.enabled || _parameters.mode == Mode::Off || _parameters.acceleration_limit <= FLT_EPSILON) {
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

void SuspendedLoadAntiSwing::setAppliedAccelerationNed(const Vector2f &applied_acceleration)
{
	if (PX4_ISFINITE(applied_acceleration(0)) && PX4_ISFINITE(applied_acceleration(1))) {
		_applied_acceleration_ned = applied_acceleration;
		_status.acceleration_applied_ned = applied_acceleration;
	}
}

void SuspendedLoadAntiSwing::setGainScheduleScale(float scale)
{
	// The scheduler may only reshape the existing damping gain. Keeping a
	// bounded, unity-centred scale makes loss of estimator validity a strict
	// fallback to the frozen PID+AS controller.
	_gain_schedule_scale = math::constrain(sanitizeFinite(scale, 1.f), 0.5f, 1.5f);
}

Vector2f SuspendedLoadAntiSwing::update(float dt, uint64_t now, float yaw, bool flying)
{
	_safety_limited = false;
	_measurement_usable = measurementUsable(now, flying);

	if (!_measurement_usable || !PX4_ISFINITE(dt) || dt <= FLT_EPSILON || !PX4_ISFINITE(yaw)) {
		_measurement_usable = false;
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
		slewRequestedAccelerationToZero(dt);
		updateStatus(now);
		return _last_acceleration_ned;
	}

	if (!_engaged) {
		if (!activationReady(now)) {
			_active = false;

			if (_rearming_after_safety) {
				slewRequestedAccelerationToZero(dt);

			} else {
				_last_acceleration_ned.zero();
			}

			_raw_acceleration_ned.zero();
			_applied_acceleration_ned.zero();
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

	Vector2f acceleration_body{};

	if (_parameters.mode == Mode::LegacyPD) {
		const bool angle_model_valid = _parameters.max_angle <= FLT_EPSILON
					       || _angle_filtered.norm() <= _parameters.max_angle;
		const Vector2f angle_feedback = angle_model_valid ? _angle_filtered : Vector2f{};
		acceleration_body =
			(angle_feedback * _parameters.angle_gain + _rate_filtered * _parameters.rate_gain) * _parameters.rope_length;

	} else if (_parameters.mode == Mode::EnergyDamping) {
		const float damping_gain = 2.f * _parameters.energy_damping_ratio
					   * sqrtf(CONSTANTS_ONE_G * _parameters.rope_length)
					   * _gain_schedule_scale;
		const float energy = perUnitMassEnergy(_angle_filtered, _rate_filtered, _parameters.rope_length);
		acceleration_body = damping_gain * _rate_filtered
				    * energyGate(energy, _parameters.energy_gate_start, _parameters.energy_gate_full);
	}

	Vector2f acceleration_ned_raw{};
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	acceleration_ned_raw(0) = yaw_cos * acceleration_body(0) - yaw_sin * acceleration_body(1);
	acceleration_ned_raw(1) = yaw_sin * acceleration_body(0) + yaw_cos * acceleration_body(1);
	_raw_acceleration_ned = acceleration_ned_raw;

	acceleration_body = constrainNorm(acceleration_body, _parameters.acceleration_limit);
	_last_ramp_scale = activationRamp(now);
	acceleration_body *= _last_ramp_scale;

	Vector2f acceleration_ned{};
	acceleration_ned(0) = yaw_cos * acceleration_body(0) - yaw_sin * acceleration_body(1);
	acceleration_ned(1) = yaw_sin * acceleration_body(0) + yaw_cos * acceleration_body(1);

	if (_parameters.acceleration_slew_rate > FLT_EPSILON) {
		const float max_delta = _parameters.acceleration_slew_rate * dt;
		const Vector2f delta = acceleration_ned - _last_acceleration_ned;
		acceleration_ned = _last_acceleration_ned + constrainNorm(delta, max_delta);
	}

	_last_acceleration_ned = constrainNorm(acceleration_ned, _parameters.acceleration_limit);
	_applied_acceleration_ned = _last_acceleration_ned;
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
	_raw_acceleration_ned.zero();
	_last_acceleration_ned.zero();
	_applied_acceleration_ned.zero();

	if (reset_flying_since) {
		_flying_since = 0;
	}

	_activation_ready_since = 0;
	_engaged_since = 0;
	_safety_rearm_since = 0;
	_last_ramp_scale = 0.f;
	_gain_schedule_scale = 1.f;
	_filter_initialized = false;
	_engaged = false;
	_active = false;
	_rearming_after_safety = false;
	_safety_limited = false;
	_measurement_usable = false;
	updateStatus(0);
}

void SuspendedLoadAntiSwing::safetyDisengage(uint64_t now)
{
	_activation_ready_since = 0;
	_engaged_since = 0;
	_safety_rearm_since = now;
	_last_ramp_scale = 0.f;
	_engaged = false;
	_active = false;
	_rearming_after_safety = true;
	_safety_limited = true;
}

void SuspendedLoadAntiSwing::slewRequestedAccelerationToZero(float dt)
{
	_raw_acceleration_ned.zero();

	if (_parameters.acceleration_slew_rate > FLT_EPSILON && PX4_ISFINITE(dt) && dt > FLT_EPSILON) {
		const float max_delta = _parameters.acceleration_slew_rate * dt;
		_last_acceleration_ned += constrainNorm(-_last_acceleration_ned, max_delta);

	} else {
		_last_acceleration_ned.zero();
	}

	_applied_acceleration_ned = _last_acceleration_ned;
}

void SuspendedLoadAntiSwing::updateStatus(uint64_t now)
{
	_status.angle_filtered = _angle_filtered;
	_status.rate_filtered = _rate_filtered;
	_status.acceleration_raw_ned = _raw_acceleration_ned;
	_status.acceleration_requested_ned = _last_acceleration_ned;
	_status.acceleration_applied_ned = _applied_acceleration_ned;
	_status.timestamp_sample = now;
	_status.ramp_scale = _last_ramp_scale;
	_status.angle_norm = _angle_filtered.norm();
	_status.natural_frequency = naturalFrequency(_parameters.rope_length);
	_status.energy_per_mass = perUnitMassEnergy(_angle_filtered, _rate_filtered, _parameters.rope_length);
	_status.energy_gate = _parameters.mode == Mode::EnergyDamping
			      ? energyGate(_status.energy_per_mass, _parameters.energy_gate_start, _parameters.energy_gate_full)
			      : 0.f;
	_status.damping_gain = _parameters.mode == Mode::EnergyDamping
				       ? 2.f * _parameters.energy_damping_ratio * sqrtf(CONSTANTS_ONE_G * _parameters.rope_length)
				       * _gain_schedule_scale
				       : 0.f;
	_status.gain_schedule_scale = _gain_schedule_scale;
	_status.rope_length = _parameters.rope_length;
	_status.mode = (_parameters.enabled ? _parameters.mode : Mode::Off);
	_status.measurement_valid = _measurement_usable;
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
	       && _parameters.mode != Mode::Off
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
	const bool below_abort_angle = _parameters.abort_angle <= FLT_EPSILON
				       || _angle_filtered.norm() <= _parameters.abort_angle;

	if (!delay_ready || !angle_ready || !rate_ready || !below_abort_angle) {
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
	return _parameters.abort_angle > FLT_EPSILON
	       && _angle_filtered.norm() > _parameters.abort_angle;
}

float SuspendedLoadAntiSwing::naturalFrequency(float rope_length)
{
	if (!PX4_ISFINITE(rope_length) || rope_length < kMinRopeLength) {
		return 0.f;
	}

	return sqrtf(CONSTANTS_ONE_G / rope_length);
}

float SuspendedLoadAntiSwing::perUnitMassEnergy(const Vector2f &angle, const Vector2f &rate, float rope_length)
{
	if (!PX4_ISFINITE(angle(0)) || !PX4_ISFINITE(angle(1))
	    || !PX4_ISFINITE(rate(0)) || !PX4_ISFINITE(rate(1))
	    || !PX4_ISFINITE(rope_length) || rope_length < kMinRopeLength) {
		return 0.f;
	}

	const float kinetic = 0.5f * rope_length * rope_length * rate.norm_squared();
	const float potential = CONSTANTS_ONE_G * rope_length
				* (2.f - cosf(angle(0)) - cosf(angle(1)));
	return fmaxf(kinetic + potential, 0.f);
}

float SuspendedLoadAntiSwing::energyGate(float energy, float gate_start, float gate_full)
{
	if (!PX4_ISFINITE(energy) || energy <= gate_start) {
		return 0.f;
	}

	if (!PX4_ISFINITE(gate_full) || gate_full <= gate_start + FLT_EPSILON) {
		return 1.f;
	}

	return math::constrain((energy - gate_start) / (gate_full - gate_start), 0.f, 1.f);
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
