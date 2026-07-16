/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "SuspendedLoadEnergySupervisor.hpp"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using matrix::Vector2f;

void SuspendedLoadEnergySupervisor::setParameters(const Parameters &parameters)
{
	_parameters = parameters;
	_parameters.energy_threshold = math::max(parameters.energy_threshold, 0.f);
	_parameters.rate_min = math::max(parameters.rate_min, 0.f);
	_parameters.gain = math::constrain(parameters.gain, 0.f, 1.f);
	_parameters.correction_limit = math::max(parameters.correction_limit, 0.f);
	_parameters.correction_slew_rate = math::max(parameters.correction_slew_rate, 0.f);
	_parameters.power_lpf_cutoff_hz = math::max(parameters.power_lpf_cutoff_hz, 0.f);
	_parameters.power_deadband = math::max(parameters.power_deadband, 0.f);
	_parameters.dwell_time = math::max(parameters.dwell_time, 0.f);
	_parameters.position_recovery_cancellation_ratio = math::constrain(
			parameters.position_recovery_cancellation_ratio, 0.f, 1.f);

	if (parameters.mode != Mode::Shadow && parameters.mode != Mode::Active) {
		_parameters.mode = Mode::Off;
		reset();
	}
}

void SuspendedLoadEnergySupervisor::reset()
{
	_status = {};
	_status.mode = _parameters.mode;
	_last_shadow_correction_ned.zero();
	_last_active_correction_ned.zero();
	_positive_power_filtered = 0.f;
	_dwell_elapsed = 0.f;
}

const SuspendedLoadEnergySupervisor::Status &SuspendedLoadEnergySupervisor::update(float dt,
		const Vector2f &candidate_acceleration_ned,
		const Vector2f &swing_rate_heading,
		const Vector2f &position_error_ned,
		float yaw,
		float rope_length,
		float energy_per_mass,
		bool anti_swing_engaged,
		bool measurement_valid)
{
	_status = {};
	_status.mode = _parameters.mode;

	if (_parameters.mode == Mode::Off) {
		reset();
		return _status;
	}

	if (!PX4_ISFINITE(dt) || dt <= FLT_EPSILON) {
		reset();
		return _status;
	}

	const bool finite_measurement = candidate_acceleration_ned.isAllFinite()
					&& swing_rate_heading.isAllFinite()
					&& PX4_ISFINITE(yaw)
					&& PX4_ISFINITE(rope_length) && rope_length >= 0.05f
					&& PX4_ISFINITE(energy_per_mass);
	const bool state_valid = measurement_valid && finite_measurement;
	Vector2f correction_raw_ned{};
	Vector2f correction_position_limited_ned{};

	if (!state_valid) {
		_positive_power_filtered = 0.f;
		_dwell_elapsed = 0.f;

	} else {
		_status.valid = true;
		_status.candidate_acceleration_ned = candidate_acceleration_ned;
		_status.projected_acceleration_ned = candidate_acceleration_ned;
		_status.energy_per_mass = energy_per_mass;
		_status.candidate_power = predictedPower(candidate_acceleration_ned, swing_rate_heading, yaw, rope_length);

		if (position_error_ned.isAllFinite()) {
			_status.position_error_ned = position_error_ned;
			const float position_error_norm = position_error_ned.norm();

			if (PX4_ISFINITE(position_error_norm) && position_error_norm >= 0.05f) {
				_status.position_direction_ned = position_error_ned / position_error_norm;
				_status.candidate_recovery_component = math::max(0.f,
						candidate_acceleration_ned.dot(_status.position_direction_ned));
			}
		}

		const float positive_power = math::max(_status.candidate_power, 0.f);

		if (_parameters.power_lpf_cutoff_hz > FLT_EPSILON) {
			const float time_constant = 1.f / (2.f * M_PI_F * _parameters.power_lpf_cutoff_hz);
			const float alpha = math::constrain(dt / (time_constant + dt), 0.f, 1.f);
			_positive_power_filtered += alpha * (positive_power - _positive_power_filtered);

		} else {
			_positive_power_filtered = positive_power;
		}

		const float rate_norm = swing_rate_heading.norm();
		const bool gate_ready = anti_swing_engaged
					&& energy_per_mass > _parameters.energy_threshold
					&& PX4_ISFINITE(rate_norm) && rate_norm > _parameters.rate_min
					&& _positive_power_filtered > _parameters.power_deadband;

		if (gate_ready) {
			_dwell_elapsed += dt;

		} else {
			_dwell_elapsed = 0.f;
		}

		_status.gate_active = gate_ready && _dwell_elapsed >= _parameters.dwell_time;

		if (_status.gate_active) {
			const Vector2f candidate_heading = nedToHeading(candidate_acceleration_ned, yaw);
			const float rate_norm_squared = swing_rate_heading.norm_squared();
			const float lambda = math::max(0.f,
						      -candidate_heading.dot(swing_rate_heading) / (rate_norm_squared + 1e-6f));
			const Vector2f correction_raw_heading = lambda * swing_rate_heading;
			correction_raw_ned = headingToNed(correction_raw_heading, yaw);
			correction_position_limited_ned = correction_raw_ned;
			_status.correction_limit_hit = correction_raw_heading.norm() > _parameters.correction_limit;

			if (_status.position_direction_ned.norm() > FLT_EPSILON) {
				const Vector2f &position_direction_ned = _status.position_direction_ned;
				_status.raw_parallel_component = correction_raw_ned.dot(position_direction_ned);
				_status.limited_parallel_component = _status.raw_parallel_component;
				const Vector2f perpendicular_correction = correction_raw_ned
						- _status.raw_parallel_component * position_direction_ned;
				_status.perpendicular_component_norm = perpendicular_correction.norm();

				const bool position_protection_enabled = _parameters.mode == Mode::Active
						&& _parameters.position_recovery_protection_enabled;

				if (position_protection_enabled && _status.raw_parallel_component < 0.f) {
					const float minimum_parallel_component =
						-_parameters.position_recovery_cancellation_ratio
						* _status.candidate_recovery_component;

					if (_status.raw_parallel_component < minimum_parallel_component) {
						_status.limited_parallel_component = minimum_parallel_component;
						_status.position_limiter_active = true;
						_status.position_limiter_ratio = _status.limited_parallel_component
								/ _status.raw_parallel_component;
					}
				}

				correction_position_limited_ned = perpendicular_correction
						+ _status.limited_parallel_component * position_direction_ned;
			}
		}
	}

	_status.correction_raw_ned = correction_raw_ned;
	_status.correction_position_limited_ned = correction_position_limited_ned;
	const Vector2f shadow_target_ned = constrainNorm(correction_raw_ned,
			_parameters.correction_limit) * _parameters.gain;
	const Vector2f active_target_ned = constrainNorm(correction_position_limited_ned,
			_parameters.correction_limit) * _parameters.gain;

	auto slew_to_target = [this, dt](const Vector2f &target, Vector2f &last_correction) {
		const Vector2f correction_delta = target - last_correction;

		if (_parameters.correction_slew_rate > FLT_EPSILON) {
			const float max_delta = _parameters.correction_slew_rate * dt;
			const bool slew_active = correction_delta.norm() > max_delta;
			last_correction += constrainNorm(correction_delta, max_delta);
			return slew_active;
		}

		last_correction = target;
		return false;
	};

	const bool shadow_slew_active = slew_to_target(shadow_target_ned, _last_shadow_correction_ned);
	const bool active_slew_active = slew_to_target(active_target_ned, _last_active_correction_ned);

	_status.positive_power_filtered = _positive_power_filtered;
	_status.dwell_elapsed = _dwell_elapsed;
	_status.correction_shadow_ned = _last_shadow_correction_ned;
	_status.correction_active_ned = _parameters.mode == Mode::Active ? _last_active_correction_ned : Vector2f{};
	_status.correction_slew_active = _parameters.mode == Mode::Active ? active_slew_active : shadow_slew_active;
	_status.active = _parameters.mode == Mode::Active && _last_active_correction_ned.norm() > FLT_EPSILON;

	if (state_valid) {
		_status.raw_correction_power = predictedPower(candidate_acceleration_ned + correction_raw_ned,
				swing_rate_heading, yaw, rope_length);
		_status.position_limited_power = predictedPower(candidate_acceleration_ned + correction_position_limited_ned,
				swing_rate_heading, yaw, rope_length);
		_status.projected_acceleration_ned = candidate_acceleration_ned
				+ (_parameters.mode == Mode::Active ? _last_active_correction_ned : _last_shadow_correction_ned);
		_status.projected_power = predictedPower(_status.projected_acceleration_ned,
				swing_rate_heading, yaw, rope_length);
		_status.projected_power_shadow = predictedPower(candidate_acceleration_ned + _last_shadow_correction_ned,
				swing_rate_heading, yaw, rope_length);
	}

	return _status;
}

float SuspendedLoadEnergySupervisor::predictedPower(const Vector2f &acceleration_ned,
		const Vector2f &swing_rate_heading,
		float yaw,
		float rope_length)
{
	if (!acceleration_ned.isAllFinite() || !swing_rate_heading.isAllFinite()
	    || !PX4_ISFINITE(yaw) || !PX4_ISFINITE(rope_length) || rope_length < 0.05f) {
		return NAN;
	}

	return -rope_length * nedToHeading(acceleration_ned, yaw).dot(swing_rate_heading);
}

Vector2f SuspendedLoadEnergySupervisor::constrainNorm(const Vector2f &value, float limit)
{
	const float safe_limit = PX4_ISFINITE(limit) ? math::max(limit, 0.f) : 0.f;
	const float norm = value.norm();

	if (safe_limit <= FLT_EPSILON || !PX4_ISFINITE(norm)) {
		return Vector2f{};
	}

	return norm > safe_limit ? value * (safe_limit / norm) : value;
}

Vector2f SuspendedLoadEnergySupervisor::headingToNed(const Vector2f &value_heading, float yaw)
{
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	return Vector2f{yaw_cos * value_heading(0) - yaw_sin * value_heading(1),
			yaw_sin * value_heading(0) + yaw_cos * value_heading(1)};
}

Vector2f SuspendedLoadEnergySupervisor::nedToHeading(const Vector2f &value_ned, float yaw)
{
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	return Vector2f{yaw_cos * value_ned(0) + yaw_sin * value_ned(1),
			-yaw_sin * value_ned(0) + yaw_cos * value_ned(1)};
}
