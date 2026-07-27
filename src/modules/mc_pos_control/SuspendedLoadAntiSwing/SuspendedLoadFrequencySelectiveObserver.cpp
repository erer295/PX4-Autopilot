/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "SuspendedLoadFrequencySelectiveObserver.hpp"

#include <float.h>
#include <math.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

constexpr float kFrequencyRatios[] = {0.70f, 0.80f, 0.90f, 1.00f, 1.10f, 1.20f, 1.30f};
constexpr float kFrequencyScoreTimeConstant = 3.f;
constexpr float kFrequencyEstimateTimeConstant = 1.5f;
constexpr float kConfidenceTimeConstant = 1.f;
constexpr float kFrequencyRatioStep = 0.10f;

static_assert(sizeof(kFrequencyRatios) / sizeof(kFrequencyRatios[0]) == 7,
	      "frequency ratio bank must match observer storage");

} // namespace

void SuspendedLoadFrequencySelectiveObserver::setParameters(const Parameters &parameters)
{
	const Mode previous_mode = _parameters.mode;
	_parameters = parameters;
	_parameters.mode = static_cast<Mode>(math::constrain(static_cast<int32_t>(parameters.mode),
			   static_cast<int32_t>(Mode::Off), static_cast<int32_t>(Mode::UnifiedShapingCoordinator)));
	_parameters.bandwidth_hz = math::constrain(parameters.bandwidth_hz, 0.10f, 10.f);
	_parameters.compensation_gain = math::constrain(parameters.compensation_gain, 0.f, 1.f);
	_parameters.phase_lead_s = math::constrain(parameters.phase_lead_s, 0.f, 0.5f);
	_parameters.acceleration_limit = math::max(parameters.acceleration_limit, 0.f);
	_parameters.acceleration_slew_rate = math::max(parameters.acceleration_slew_rate, 0.f);
	_parameters.energy_threshold = math::max(parameters.energy_threshold, 0.f);
	_parameters.rate_min = math::max(parameters.rate_min, 0.f);
	_parameters.settling_time = math::max(parameters.settling_time, 0.f);
	_parameters.confidence_min = math::constrain(parameters.confidence_min, 0.f, 0.8f);
	_parameters.position_error_threshold = math::max(parameters.position_error_threshold, 0.f);

	if (_parameters.mode == Mode::Off || previous_mode != _parameters.mode) {
		reset();
	}
}

const SuspendedLoadFrequencySelectiveObserver::Status &SuspendedLoadFrequencySelectiveObserver::update(
	float dt,
	const Vector2f &measured_velocity_ned,
	const Vector2f &known_acceleration_ned,
	const Vector2f &swing_angle_heading,
	const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned,
	float yaw,
	float rope_length,
	float energy_per_mass,
	bool anti_swing_engaged,
	bool measurement_valid)
{
	_status = {};
	_status.measured_velocity_ned = measured_velocity_ned;
	_status.known_acceleration_ned = known_acceleration_ned;
	_status.position_error_ned = position_error_ned;
	_status.energy_per_mass = energy_per_mass;
	_status.center_frequency_hz = centerFrequencyHz(rope_length);
	_status.effective_frequency_hz = _status.center_frequency_hz;
	_status.effective_frequency_ratio = 1.f;
	_status.gain_scale_raw = 1.f;
	_status.gain_scale_applied = (_parameters.mode == Mode::GainSchedule
				      || _parameters.mode == Mode::OracleGainSchedule) ? _last_gain_scale : 1.f;
	_status.mode = _parameters.mode;

	const bool finite = PX4_ISFINITE(dt) && dt > FLT_EPSILON
			    && dt <= 0.05f
			    && measured_velocity_ned.isAllFinite()
			    && known_acceleration_ned.isAllFinite()
			    && swing_angle_heading.isAllFinite()
			    && swing_rate_heading.isAllFinite()
			    && PX4_ISFINITE(yaw)
			    && PX4_ISFINITE(rope_length) && rope_length >= 0.05f
			    && PX4_ISFINITE(energy_per_mass)
			    && PX4_ISFINITE(_status.center_frequency_hz);

	if (_parameters.mode == Mode::Off || !measurement_valid || !anti_swing_engaged || !finite) {
		reset();
		_status.center_frequency_hz = centerFrequencyHz(rope_length);
		_status.energy_per_mass = PX4_ISFINITE(energy_per_mass) ? energy_per_mass : 0.f;
		_status.position_error_ned = position_error_ned.isAllFinite() ? position_error_ned : Vector2f{};
		_status.effective_frequency_hz = _status.center_frequency_hz;
		_status.effective_frequency_ratio = 1.f;
		_status.gain_scale_raw = 1.f;
		_status.gain_scale_applied = 1.f;
		_status.mode = _parameters.mode;
		return _status;
	}

	_status.valid = true;

	if (_parameters.mode == Mode::GainSchedule) {
		return updateGainSchedule(dt, known_acceleration_ned, swing_angle_heading, swing_rate_heading,
			       position_error_ned, yaw, rope_length, energy_per_mass);
	}

	if (_parameters.mode == Mode::RlsShadow || _parameters.mode == Mode::UnifiedShapingCoordinator) {
		return updateRlsShadow(dt, known_acceleration_ned, swing_angle_heading, swing_rate_heading,
			       position_error_ned, yaw, rope_length, energy_per_mass);
	}

	if (_parameters.mode == Mode::OracleGainSchedule) {
		return updateOracleGainSchedule(dt, swing_rate_heading, position_error_ned, energy_per_mass);
	}

	if (!_initialized) {
		_velocity_estimate_ned = measured_velocity_ned;
		_disturbance_estimate_ned.zero();
		_disturbance_derivative_ned.zero();
		_initialized = true;
	}

	// Harmonic ESO for x = [velocity, disturbance, disturbance derivative].
	// The disturbance internal model is d_ddot + wn^2*d = 0. Observer gains
	// place all error poles at -wo and avoid differentiating measured velocity.
	const float natural_frequency = 2.f * M_PI_F * _status.center_frequency_hz;
	const float observer_bandwidth = 2.f * M_PI_F * _parameters.bandwidth_hz;
	const float l1 = 3.f * observer_bandwidth;
	const float l2 = 3.f * observer_bandwidth * observer_bandwidth
			 - natural_frequency * natural_frequency;
	const float l3 = observer_bandwidth * observer_bandwidth * observer_bandwidth
			 - 3.f * observer_bandwidth * natural_frequency * natural_frequency;
	const Vector2f innovation = measured_velocity_ned - _velocity_estimate_ned;
	const Vector2f velocity_derivative = known_acceleration_ned + _disturbance_estimate_ned + l1 * innovation;
	const Vector2f disturbance_derivative = _disturbance_derivative_ned + l2 * innovation;
	const Vector2f disturbance_second_derivative = -natural_frequency * natural_frequency
			 * _disturbance_estimate_ned + l3 * innovation;
	_velocity_estimate_ned += dt * velocity_derivative;
	_disturbance_estimate_ned += dt * disturbance_derivative;
	_disturbance_derivative_ned += dt * disturbance_second_derivative;

	if (!_velocity_estimate_ned.isAllFinite() || !_disturbance_estimate_ned.isAllFinite()
	    || !_disturbance_derivative_ned.isAllFinite()) {
		reset();
		_status.center_frequency_hz = centerFrequencyHz(rope_length);
		_status.energy_per_mass = energy_per_mass;
		_status.mode = _parameters.mode;
		return _status;
	}

	_status.estimated_velocity_ned = _velocity_estimate_ned;
	_status.velocity_innovation_ned = measured_velocity_ned - _velocity_estimate_ned;
	_status.disturbance_estimate_ned = _disturbance_estimate_ned;
	_status.disturbance_derivative_ned = _disturbance_derivative_ned;
	const float phase_lead = natural_frequency * _parameters.phase_lead_s;
	_status.disturbance_phase_lead_ned = cosf(phase_lead) * _disturbance_estimate_ned
			 + sinf(phase_lead) * _disturbance_derivative_ned / natural_frequency;

	_observer_elapsed += dt;
	_status.observer_elapsed = _observer_elapsed;
	_status.settled = _observer_elapsed >= _parameters.settling_time;

	const float swing_rate_norm = swing_rate_heading.norm();
	const float settled_scale = _parameters.settling_time <= FLT_EPSILON ? 1.f
				    : smoothStep(_observer_elapsed, _parameters.settling_time,
						 _parameters.settling_time + 0.5f);
	const float energy_scale = _parameters.energy_threshold <= FLT_EPSILON ? 1.f
				   : smoothStep(energy_per_mass, 0.5f * _parameters.energy_threshold,
						1.5f * _parameters.energy_threshold);
	const float rate_scale = _parameters.rate_min <= FLT_EPSILON ? 1.f
				 : smoothStep(swing_rate_norm, 0.5f * _parameters.rate_min,
					      1.5f * _parameters.rate_min);
	_status.gate_scale = settled_scale * energy_scale * rate_scale;
	_status.power_constraint_weight = rate_scale;
	_status.gate_active = _status.gate_scale > 1e-3f;
	_status.compensation_raw_ned = -_parameters.compensation_gain * _status.gate_scale
				       * _status.disturbance_phase_lead_ned;
	const Vector2f target = constrainNorm(_status.compensation_raw_ned, _parameters.acceleration_limit);

	_status.raw_compensation_power = predictedPower(target, swing_rate_heading, yaw, rope_length);
	_status.raw_position_alignment = positionAlignment(target, position_error_ned);
	const float position_error_norm = position_error_ned.isAllFinite() ? position_error_ned.norm() : 0.f;
	_status.position_constraint_weight = _parameters.position_protection_enabled
					     && PX4_ISFINITE(position_error_norm)
					     ? smoothStep(position_error_norm,
							  0.5f * _parameters.position_error_threshold,
							  1.5f * _parameters.position_error_threshold)
					     : 0.f;
	_status.power_constraint_lower_bound = -(1.f - _status.power_constraint_weight)
					       * _parameters.acceleration_limit;
	_status.position_constraint_lower_bound = -(1.f - _status.position_constraint_weight)
						  * _parameters.acceleration_limit;
	const float maximum_delta = _parameters.acceleration_slew_rate > FLT_EPSILON
				    ? _parameters.acceleration_slew_rate * dt : INFINITY;
	_last_compensation_ned = projectToSmoothUnifiedConstraints(target, _last_compensation_ned,
				 swing_rate_heading, position_error_ned, yaw,
				 _parameters.acceleration_limit, maximum_delta,
				 _status.power_constraint_weight, _status.position_constraint_weight,
				 _status.power_limited, _status.position_limited, _status.slew_limited,
				 _status.unified_constraint_feasible);
	_status.compensation_projected_ned = _last_compensation_ned;
	_status.projected_compensation_power = predictedPower(_last_compensation_ned, swing_rate_heading, yaw, rope_length);
	_status.projected_position_alignment = positionAlignment(_last_compensation_ned, position_error_ned);

	if (_parameters.mode == Mode::Active) {
		_status.compensation_applied_ned = _last_compensation_ned;
		_status.applied_compensation_power = _status.projected_compensation_power;
		_status.applied_position_alignment = _status.projected_position_alignment;
		_status.active = _last_compensation_ned.norm() > FLT_EPSILON;
	}

	return _status;
}

void SuspendedLoadFrequencySelectiveObserver::updateScheduleFromEstimate(
	float dt, float frequency_ratio, float confidence,
	const Vector2f &swing_rate_heading, const Vector2f &position_error_ned,
	float energy_per_mass, bool apply_schedule)
{
	const float energy_full = math::max(3.f * _parameters.energy_threshold,
				 _parameters.energy_threshold + 1e-4f);
	const float energy_gate = _parameters.energy_threshold <= FLT_EPSILON ? 1.f
				  : smoothStep(energy_per_mass, _parameters.energy_threshold, energy_full);
	const float rate_gate = _parameters.rate_min <= FLT_EPSILON ? 1.f
				: smoothStep(swing_rate_heading.norm(), 0.5f * _parameters.rate_min,
					     1.5f * _parameters.rate_min);
	const float signal_gate = math::max(energy_gate, rate_gate);
	const float confidence_full = math::min(_parameters.confidence_min + 0.25f, 0.95f);
	const float confidence_gate = smoothStep(confidence, _parameters.confidence_min, confidence_full);
	const float position_error_norm = position_error_ned.norm();
	const float position_gate = _parameters.position_protection_enabled
				    ? 1.f - smoothStep(position_error_norm,
						 0.5f * _parameters.position_error_threshold,
						 2.f * _parameters.position_error_threshold)
				    : 1.f;
	const float schedule_gate = confidence_gate * energy_gate * position_gate;
	const float frequency_boost_scale = math::constrain(frequency_ratio, 0.9f, 1.1f);
	const float maximum_gain_scale = math::min(1.f + _parameters.compensation_gain, 1.5f);
	const float target_gain_scale = apply_schedule
					? math::constrain(1.f + _parameters.compensation_gain
							  * schedule_gate * frequency_boost_scale,
							  1.f, maximum_gain_scale)
					: 1.f;
	const float maximum_gain_delta = _parameters.acceleration_slew_rate > FLT_EPSILON
					 ? _parameters.acceleration_slew_rate * dt : INFINITY;
	_last_gain_scale += math::constrain(target_gain_scale - _last_gain_scale,
			    -maximum_gain_delta, maximum_gain_delta);

	_status.effective_frequency_ratio = frequency_ratio;
	_status.effective_frequency_hz = _status.center_frequency_hz * frequency_ratio;
	_status.frequency_confidence = confidence;
	_status.confidence_gate = confidence_gate;
	_status.schedule_energy_gate = energy_gate;
	_status.schedule_position_gate = position_gate;
	_status.gain_scale_raw = target_gain_scale;
	_status.gain_scale_applied = apply_schedule ? _last_gain_scale : 1.f;
	_status.frequency_signal_gate = signal_gate;
	_status.gate_scale = schedule_gate;
	_status.gate_active = schedule_gate > 1e-3f;
	_status.schedule_active = apply_schedule && fabsf(_last_gain_scale - 1.f) > 1e-3f;
	_status.active = _status.schedule_active;
}

const SuspendedLoadFrequencySelectiveObserver::Status &
SuspendedLoadFrequencySelectiveObserver::updateOracleGainSchedule(
	float dt, const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned, float energy_per_mass)
{
	_observer_elapsed += dt;
	_status.observer_elapsed = _observer_elapsed;
	_status.settled = true;
	_status.frequency_best_bin = 3.f;
	_status.frequency_best_score = 0.f;
	_status.frequency_worst_score = 0.f;
	updateScheduleFromEstimate(dt, 1.f, 1.f, swing_rate_heading,
				   position_error_ned, energy_per_mass, true);
	return _status;
}

void SuspendedLoadFrequencySelectiveObserver::updateRlsFrequencyEstimate(
	float dt, const Vector2f &forcing_heading,
	const Vector2f &swing_angle_heading, const Vector2f &swing_rate_heading,
	float nominal_frequency, float energy_per_mass)
{
	if (!_rls_initialized) {
		_rls_theta[0] = nominal_frequency * nominal_frequency;
		_rls_theta[1] = 0.10f * nominal_frequency;
		_rls_covariance[0][0] = 30.f;
		_rls_covariance[0][1] = 0.f;
		_rls_covariance[1][0] = 0.f;
		_rls_covariance[1][1] = 10.f;
		_rls_information = 0.f;
		_rls_residual_variance = 1e-4f;
		_rls_history_head = 0;
		_rls_history_count = 0;
		_rls_initialized = true;
	}

	const int write_index = _rls_history_head;
	_rls_angle_history[write_index] = swing_angle_heading;
	_rls_rate_history[write_index] = swing_rate_heading;
	_rls_forcing_history[write_index] = forcing_heading;
	_rls_dt_history[write_index] = dt;
	_rls_history_head = (_rls_history_head + 1) % kRlsHistoryCapacity;
	_rls_history_count = math::min(_rls_history_count + 1, kRlsHistoryCapacity);

	Vector2f angle_integral{};
	Vector2f rate_integral{};
	Vector2f forcing_integral{};
	float window_elapsed = 0.f;
	int newest_index = (_rls_history_head + kRlsHistoryCapacity - 1) % kRlsHistoryCapacity;
	int current_index = newest_index;
	int oldest_index = newest_index;

	for (int sample = 1; sample < _rls_history_count && window_elapsed < 0.40f; ++sample) {
		const int previous_index = (current_index + kRlsHistoryCapacity - 1) % kRlsHistoryCapacity;
		const float step = math::constrain(_rls_dt_history[current_index], 1e-4f, 0.05f);
		angle_integral += 0.5f * (_rls_angle_history[current_index]
					 + _rls_angle_history[previous_index]) * step;
		rate_integral += 0.5f * (_rls_rate_history[current_index]
					 + _rls_rate_history[previous_index]) * step;
		forcing_integral += 0.5f * (_rls_forcing_history[current_index]
					   + _rls_forcing_history[previous_index]) * step;
		window_elapsed += step;
		oldest_index = previous_index;
		current_index = previous_index;
	}

	float residual_sum = 0.f;
	float excitation_sum = 0.f;

	if (window_elapsed >= 0.35f) {
		const Vector2f response = _rls_rate_history[newest_index]
					  - _rls_rate_history[oldest_index] - forcing_integral;
		const float forgetting = expf(-dt / 15.f);

		for (int axis = 0; axis < 2; ++axis) {
			const float phi[2] = {-angle_integral(axis), -rate_integral(axis)};
			const float excitation = phi[0] * phi[0] + phi[1] * phi[1];

			if (excitation < 1e-10f) {
				continue;
			}

			const float p_phi[2] = {
				_rls_covariance[0][0] * phi[0] + _rls_covariance[0][1] * phi[1],
				_rls_covariance[1][0] * phi[0] + _rls_covariance[1][1] * phi[1]
			};
			const float denominator = math::max(forgetting + phi[0] * p_phi[0] + phi[1] * p_phi[1], 1e-8f);
			const float gain[2] = {p_phi[0] / denominator, p_phi[1] / denominator};
			const float innovation = response(axis) - phi[0] * _rls_theta[0] - phi[1] * _rls_theta[1];
			_rls_theta[0] += gain[0] * innovation;
			_rls_theta[1] += gain[1] * innovation;

			const float row0[2] = {
				phi[0] * _rls_covariance[0][0] + phi[1] * _rls_covariance[1][0],
				phi[0] * _rls_covariance[0][1] + phi[1] * _rls_covariance[1][1]
			};
			_rls_covariance[0][0] = (_rls_covariance[0][0] - gain[0] * row0[0]) / forgetting;
			_rls_covariance[0][1] = (_rls_covariance[0][1] - gain[0] * row0[1]) / forgetting;
			_rls_covariance[1][0] = (_rls_covariance[1][0] - gain[1] * row0[0]) / forgetting;
			_rls_covariance[1][1] = (_rls_covariance[1][1] - gain[1] * row0[1]) / forgetting;
			residual_sum += innovation * innovation;
			excitation_sum += excitation;
		}
	}

	const float minimum_q = 0.70f * 0.70f * nominal_frequency * nominal_frequency;
	const float maximum_q = 1.30f * 1.30f * nominal_frequency * nominal_frequency;
	_rls_theta[0] = math::constrain(_rls_theta[0], minimum_q, maximum_q);
	_rls_theta[1] = math::constrain(_rls_theta[1], 0.f, 1.5f * nominal_frequency);
	const float raw_ratio = sqrtf(_rls_theta[0]) / nominal_frequency;
	const float frequency_residual = raw_ratio - _effective_frequency_ratio;
	const float estimate_alpha = expf(-dt / kFrequencyEstimateTimeConstant);
	_effective_frequency_ratio = estimate_alpha * _effective_frequency_ratio
					 + (1.f - estimate_alpha) * raw_ratio;
	_frequency_ratio_variance = estimate_alpha * _frequency_ratio_variance
				    + (1.f - estimate_alpha) * frequency_residual * frequency_residual;
	const float audit_alpha = expf(-dt / 2.f);
	_rls_information = audit_alpha * _rls_information
			 + (1.f - audit_alpha) * excitation_sum / math::max(window_elapsed, 1e-4f);
	_rls_residual_variance = audit_alpha * _rls_residual_variance
				 + (1.f - audit_alpha) * residual_sum;
	const float information_gate = smoothStep(_rls_information, 1e-5f, 8e-4f);
	const float stability = 1.f - smoothStep(sqrtf(math::max(_frequency_ratio_variance, 0.f)), 0.02f, 0.08f);
	const float residual_gate = 1.f - smoothStep(_rls_residual_variance, 2e-4f, 4e-3f);
	const float energy_full = math::max(3.f * _parameters.energy_threshold,
				 _parameters.energy_threshold + 1e-4f);
	const float energy_gate = _parameters.energy_threshold <= FLT_EPSILON ? 1.f
				  : smoothStep(energy_per_mass, _parameters.energy_threshold, energy_full);
	const float rate_gate = _parameters.rate_min <= FLT_EPSILON ? 1.f
				: smoothStep(swing_rate_heading.norm(), 0.5f * _parameters.rate_min,
					     1.5f * _parameters.rate_min);
	const float confidence_target = information_gate * stability * residual_gate * math::max(energy_gate, rate_gate);
	const float confidence_alpha = expf(-dt / kConfidenceTimeConstant);
	_frequency_confidence_filtered = confidence_alpha * _frequency_confidence_filtered
					 + (1.f - confidence_alpha) * confidence_target;
	_status.frequency_best_bin = -1.f;
	_status.frequency_best_score = _rls_residual_variance;
	_status.frequency_worst_score = _rls_information;
}

const SuspendedLoadFrequencySelectiveObserver::Status &
SuspendedLoadFrequencySelectiveObserver::updateRlsShadow(
	float dt, const Vector2f &known_acceleration_ned,
	const Vector2f &swing_angle_heading, const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned,
	float yaw, float rope_length, float energy_per_mass)
{
	const float nominal_frequency = 2.f * M_PI_F * _status.center_frequency_hz;
	const Vector2f forcing_heading = -nedToHeading(known_acceleration_ned, yaw) / rope_length;
	updateRlsFrequencyEstimate(dt, forcing_heading, swing_angle_heading, swing_rate_heading,
				   nominal_frequency, energy_per_mass);
	_observer_elapsed += dt;
	_status.observer_elapsed = _observer_elapsed;
	_status.settled = _observer_elapsed >= _parameters.settling_time;
	updateScheduleFromEstimate(dt, _effective_frequency_ratio, _frequency_confidence_filtered,
				   swing_rate_heading, position_error_ned, energy_per_mass, false);
	return _status;
}

const SuspendedLoadFrequencySelectiveObserver::Status &SuspendedLoadFrequencySelectiveObserver::updateGainSchedule(
	float dt, const Vector2f &known_acceleration_ned,
	const Vector2f &swing_angle_heading, const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned, float yaw, float rope_length, float energy_per_mass)
{
	const float nominal_frequency = 2.f * M_PI_F * _status.center_frequency_hz;
	const float observer_bandwidth = 2.f * M_PI_F * _parameters.bandwidth_hz;
	const Vector2f forcing_heading = -nedToHeading(known_acceleration_ned, yaw) / rope_length;

	if (!_frequency_bank_initialized) {
		for (int bin = 0; bin < kFrequencyBinCount; ++bin) {
			_frequency_angle_estimate[bin] = swing_angle_heading;
			_frequency_rate_estimate[bin] = swing_rate_heading;
			_frequency_disturbance_estimate[bin].zero();
			_frequency_fit_score[bin] = 1e-4f;
		}

		_effective_frequency_ratio = 1.f;
		_frequency_ratio_variance = 0.f;
		_frequency_confidence_filtered = 0.f;
		_last_gain_scale = 1.f;
		_frequency_bank_initialized = true;
	}

	const float l1 = 3.f * observer_bandwidth;
	const float l3 = observer_bandwidth * observer_bandwidth * observer_bandwidth;
	const float score_alpha = expf(-dt / kFrequencyScoreTimeConstant);
	float best_score = INFINITY;
	float worst_score = 0.f;
	int best_bin = 0;

	// A parallel bank of third-order HESOs tests a narrow set of physically
	// plausible pendulum frequencies. Each observer estimates the unmodelled
	// angular acceleration needed by its candidate oscillator. Frequency is
	// selected by the joint angle/rate innovation, not by compensation benefit.
	for (int bin = 0; bin < kFrequencyBinCount; ++bin) {
		const float candidate_frequency = nominal_frequency * kFrequencyRatios[bin];
		const float candidate_frequency_squared = candidate_frequency * candidate_frequency;
		const float l2 = 3.f * observer_bandwidth * observer_bandwidth - candidate_frequency_squared;
		const Vector2f angle_innovation = swing_angle_heading - _frequency_angle_estimate[bin];
		const Vector2f angle_estimate_previous = _frequency_angle_estimate[bin];
		const Vector2f rate_estimate_previous = _frequency_rate_estimate[bin];
		_frequency_angle_estimate[bin] += dt * (rate_estimate_previous + l1 * angle_innovation);
		_frequency_rate_estimate[bin] += dt * (-candidate_frequency_squared * angle_estimate_previous
							 + forcing_heading + _frequency_disturbance_estimate[bin]
							 + l2 * angle_innovation);
		_frequency_disturbance_estimate[bin] += dt * l3 * angle_innovation;

		const Vector2f angle_residual = swing_angle_heading - _frequency_angle_estimate[bin];
		const Vector2f rate_residual = swing_rate_heading - _frequency_rate_estimate[bin];
		const float fit_sample = angle_residual.norm_squared()
					 + rate_residual.norm_squared() / (nominal_frequency * nominal_frequency);
		_frequency_fit_score[bin] = score_alpha * _frequency_fit_score[bin]
					    + (1.f - score_alpha) * fit_sample;

		if (!_frequency_angle_estimate[bin].isAllFinite() || !_frequency_rate_estimate[bin].isAllFinite()
		    || !_frequency_disturbance_estimate[bin].isAllFinite()
		    || !PX4_ISFINITE(_frequency_fit_score[bin])) {
			reset();
			_status.center_frequency_hz = centerFrequencyHz(rope_length);
			_status.effective_frequency_hz = _status.center_frequency_hz;
			_status.effective_frequency_ratio = 1.f;
			_status.gain_scale_raw = 1.f;
			_status.gain_scale_applied = 1.f;
			_status.mode = _parameters.mode;
			return _status;
		}

		if (_frequency_fit_score[bin] < best_score) {
			best_score = _frequency_fit_score[bin];
			best_bin = bin;
		}

		worst_score = math::max(worst_score, _frequency_fit_score[bin]);
	}

	float bin_offset = 0.f;

	if (best_bin > 0 && best_bin + 1 < kFrequencyBinCount) {
		const float left = _frequency_fit_score[best_bin - 1];
		const float center = _frequency_fit_score[best_bin];
		const float right = _frequency_fit_score[best_bin + 1];
		const float curvature = left - 2.f * center + right;

		if (curvature > 1e-9f) {
			bin_offset = math::constrain(0.5f * (left - right) / curvature, -0.5f, 0.5f);
		}
	}

	const float instantaneous_ratio = math::constrain(kFrequencyRatios[best_bin]
					  + kFrequencyRatioStep * bin_offset, kFrequencyRatios[0],
					  kFrequencyRatios[kFrequencyBinCount - 1]);
	const float frequency_alpha = expf(-dt / kFrequencyEstimateTimeConstant);
	const float frequency_residual = instantaneous_ratio - _effective_frequency_ratio;
	_effective_frequency_ratio += (1.f - frequency_alpha) * frequency_residual;
	_frequency_ratio_variance = frequency_alpha * _frequency_ratio_variance
				    + (1.f - frequency_alpha) * frequency_residual * frequency_residual;

	_observer_elapsed += dt;
	_status.observer_elapsed = _observer_elapsed;
	_status.settled = _observer_elapsed >= _parameters.settling_time;
	const float settled_scale = _parameters.settling_time <= FLT_EPSILON ? 1.f
				    : smoothStep(_observer_elapsed, _parameters.settling_time,
						 _parameters.settling_time + 0.5f);
	const float energy_full = math::max(3.f * _parameters.energy_threshold,
					 _parameters.energy_threshold + 1e-4f);
	const float energy_gate = _parameters.energy_threshold <= FLT_EPSILON ? 1.f
				  : smoothStep(energy_per_mass, _parameters.energy_threshold, energy_full);
	const float rate_gate = _parameters.rate_min <= FLT_EPSILON ? 1.f
				: smoothStep(swing_rate_heading.norm(), 0.5f * _parameters.rate_min,
					     1.5f * _parameters.rate_min);
	const float signal_gate = math::max(energy_gate, rate_gate);
	const float contrast = math::constrain((worst_score - best_score)
			       / (worst_score + best_score + 1e-9f), 0.f, 1.f);
	const float frequency_standard_deviation = sqrtf(math::max(_frequency_ratio_variance, 0.f));
	const float stability = 1.f - smoothStep(frequency_standard_deviation, 0.03f, 0.15f);
	const float confidence_target = sqrtf(contrast) * stability * signal_gate * settled_scale;
	const float confidence_alpha = expf(-dt / kConfidenceTimeConstant);
	_frequency_confidence_filtered = confidence_alpha * _frequency_confidence_filtered
					 + (1.f - confidence_alpha) * confidence_target;

	const float confidence_full = math::min(_parameters.confidence_min + 0.25f, 0.95f);
	const float confidence_gate = smoothStep(_frequency_confidence_filtered,
				      _parameters.confidence_min, confidence_full);
	const float position_error_norm = position_error_ned.norm();
	const float position_gate = _parameters.position_protection_enabled
				    ? 1.f - smoothStep(position_error_norm,
						 0.5f * _parameters.position_error_threshold,
						 2.f * _parameters.position_error_threshold)
				    : 1.f;
	const float schedule_gate = confidence_gate * energy_gate * position_gate;
	const float frequency_boost_scale = math::constrain(_effective_frequency_ratio, 0.9f, 1.1f);
	const float maximum_gain_scale = math::min(1.f + _parameters.compensation_gain, 1.5f);
	const float target_gain_scale = math::constrain(1.f + _parameters.compensation_gain
					* schedule_gate * frequency_boost_scale, 1.f, maximum_gain_scale);
	const float maximum_gain_delta = _parameters.acceleration_slew_rate > FLT_EPSILON
					 ? _parameters.acceleration_slew_rate * dt : INFINITY;
	_last_gain_scale += math::constrain(target_gain_scale - _last_gain_scale,
			    -maximum_gain_delta, maximum_gain_delta);

	_status.effective_frequency_ratio = _effective_frequency_ratio;
	_status.effective_frequency_hz = _status.center_frequency_hz * _effective_frequency_ratio;
	_status.frequency_confidence = _frequency_confidence_filtered;
	_status.confidence_gate = confidence_gate;
	_status.schedule_energy_gate = energy_gate;
	_status.schedule_position_gate = position_gate;
	_status.gain_scale_raw = target_gain_scale;
	_status.gain_scale_applied = _last_gain_scale;
	_status.frequency_best_bin = static_cast<float>(best_bin);
	_status.frequency_best_score = best_score;
	_status.frequency_worst_score = worst_score;
	_status.frequency_signal_gate = signal_gate;
	_status.gate_scale = schedule_gate;
	_status.gate_active = schedule_gate > 1e-3f;
	_status.schedule_active = fabsf(_last_gain_scale - 1.f) > 1e-3f;
	_status.active = _status.schedule_active;
	return _status;
}

void SuspendedLoadFrequencySelectiveObserver::reset()
{
	_status = {};
	_status.effective_frequency_ratio = 1.f;
	_status.gain_scale_raw = 1.f;
	_status.gain_scale_applied = 1.f;
	_velocity_estimate_ned.setZero();
	_disturbance_estimate_ned.setZero();
	_disturbance_derivative_ned.setZero();
	_last_compensation_ned.setZero();

	for (int bin = 0; bin < kFrequencyBinCount; ++bin) {
		_frequency_angle_estimate[bin].zero();
		_frequency_rate_estimate[bin].zero();
		_frequency_disturbance_estimate[bin].zero();
		_frequency_fit_score[bin] = 0.f;
	}

	_effective_frequency_ratio = 1.f;
	_frequency_ratio_variance = 0.f;
	_frequency_confidence_filtered = 0.f;
	_last_gain_scale = 1.f;
	_observer_elapsed = 0.f;
	for (int sample = 0; sample < kRlsHistoryCapacity; ++sample) {
		_rls_angle_history[sample].zero();
		_rls_rate_history[sample].zero();
		_rls_forcing_history[sample].zero();
		_rls_dt_history[sample] = 0.f;
	}
	_rls_theta[0] = 0.f;
	_rls_theta[1] = 0.f;
	_rls_covariance[0][0] = 0.f;
	_rls_covariance[0][1] = 0.f;
	_rls_covariance[1][0] = 0.f;
	_rls_covariance[1][1] = 0.f;
	_rls_information = 0.f;
	_rls_residual_variance = 1e-4f;
	_rls_history_head = 0;
	_rls_history_count = 0;
	_rls_initialized = false;
	_initialized = false;
	_frequency_bank_initialized = false;
}

void SuspendedLoadFrequencySelectiveObserver::setAppliedCompensationNed(
	const Vector2f &applied_compensation_ned, const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned, float yaw, float rope_length)
{
	_status.compensation_applied_ned = applied_compensation_ned;
	_status.applied_compensation_power = predictedPower(applied_compensation_ned, swing_rate_heading, yaw, rope_length);
	_status.applied_position_alignment = positionAlignment(applied_compensation_ned, position_error_ned);
	_status.active = _parameters.mode == Mode::Active && applied_compensation_ned.norm() > FLT_EPSILON;
}

float SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(float rope_length)
{
	return PX4_ISFINITE(rope_length) && rope_length >= 0.05f
	       ? sqrtf(CONSTANTS_ONE_G / rope_length) / (2.f * M_PI_F)
	       : NAN;
}

float SuspendedLoadFrequencySelectiveObserver::predictedPower(const Vector2f &acceleration_ned,
	const Vector2f &swing_rate_heading, float yaw, float rope_length)
{
	if (!acceleration_ned.isAllFinite() || !swing_rate_heading.isAllFinite()
	    || !PX4_ISFINITE(yaw) || !PX4_ISFINITE(rope_length) || rope_length < 0.05f) {
		return NAN;
	}

	return -rope_length * nedToHeading(acceleration_ned, yaw).dot(swing_rate_heading);
}

Vector2f SuspendedLoadFrequencySelectiveObserver::projectToNonPositivePower(const Vector2f &acceleration_ned,
	const Vector2f &swing_rate_heading, float yaw, bool &limited)
{
	limited = false;

	if (!acceleration_ned.isAllFinite() || !swing_rate_heading.isAllFinite() || !PX4_ISFINITE(yaw)) {
		return Vector2f{};
	}

	Vector2f acceleration_heading = nedToHeading(acceleration_ned, yaw);
	const float rate_norm_squared = swing_rate_heading.norm_squared();
	const float acceleration_rate_dot = acceleration_heading.dot(swing_rate_heading);

	// P_comp = -L*a_comp dot theta_dot.  Positive P_comp injects energy.
	if (acceleration_rate_dot < 0.f && rate_norm_squared > 1e-8f) {
		acceleration_heading -= (acceleration_rate_dot / rate_norm_squared) * swing_rate_heading;
		limited = true;
	}

	return headingToNed(acceleration_heading, yaw);
}

Vector2f SuspendedLoadFrequencySelectiveObserver::projectToPowerAndPositionConstraints(
	const Vector2f &acceleration_ned, const Vector2f &swing_rate_heading,
	const Vector2f &position_error_ned, float yaw, bool position_constraint_enabled,
	bool &power_limited, bool &position_limited)
{
	power_limited = false;
	position_limited = false;

	if (!acceleration_ned.isAllFinite() || !swing_rate_heading.isAllFinite() || !PX4_ISFINITE(yaw)
	    || (position_constraint_enabled && !position_error_ned.isAllFinite())) {
		return Vector2f{};
	}

	const Vector2f power_normal_ned = headingToNed(swing_rate_heading, yaw);
	const Vector2f position_normal_ned = position_constraint_enabled ? position_error_ned : Vector2f{};
	const float power_norm_squared = power_normal_ned.norm_squared();
	const float position_norm_squared = position_normal_ned.norm_squared();
	const bool use_power = power_norm_squared > 1e-8f;
	const bool use_position = position_norm_squared > 1e-8f;
	const float tolerance = 1e-7f;

	auto satisfies = [tolerance](const Vector2f &value, const Vector2f &normal, bool enabled) {
		return !enabled || value.dot(normal) >= -tolerance;
	};
	const bool raw_power_feasible = satisfies(acceleration_ned, power_normal_ned, use_power);
	const bool raw_position_feasible = satisfies(acceleration_ned, position_normal_ned, use_position);
	power_limited = !raw_power_feasible;
	position_limited = !raw_position_feasible;

	if (raw_power_feasible && raw_position_feasible) {
		return acceleration_ned;
	}

	// Euclidean projection onto the intersection of two homogeneous
	// half-spaces. In 2-D the optimum is either on one boundary or at the
	// intersection; zero is always a feasible intersection candidate.
	Vector2f best{};
	float best_distance_squared = (acceleration_ned - best).norm_squared();

	auto consider_boundary = [&](const Vector2f &normal, float norm_squared) {
		if (norm_squared <= 1e-8f) {
			return;
		}

		const Vector2f candidate = acceleration_ned
				- acceleration_ned.dot(normal) / norm_squared * normal;

		if (satisfies(candidate, power_normal_ned, use_power)
		    && satisfies(candidate, position_normal_ned, use_position)) {
			const float distance_squared = (acceleration_ned - candidate).norm_squared();

			if (distance_squared < best_distance_squared) {
				best = candidate;
				best_distance_squared = distance_squared;
			}
		}
	};

	consider_boundary(power_normal_ned, power_norm_squared);
	consider_boundary(position_normal_ned, position_norm_squared);
	return best;
}

Vector2f SuspendedLoadFrequencySelectiveObserver::projectToSmoothUnifiedConstraints(
	const Vector2f &target_acceleration_ned, const Vector2f &previous_acceleration_ned,
	const Vector2f &swing_rate_heading, const Vector2f &position_error_ned,
	float yaw, float acceleration_limit, float maximum_delta,
	float power_constraint_weight, float position_constraint_weight,
	bool &power_limited, bool &position_limited, bool &slew_limited, bool &feasible)
{
	power_limited = false;
	position_limited = false;
	slew_limited = false;
	feasible = false;

	if (!target_acceleration_ned.isAllFinite() || !previous_acceleration_ned.isAllFinite()
	    || !swing_rate_heading.isAllFinite() || !position_error_ned.isAllFinite() || !PX4_ISFINITE(yaw)) {
		return Vector2f{};
	}

	const float limit = PX4_ISFINITE(acceleration_limit) ? math::max(acceleration_limit, 0.f) : 0.f;
	const bool slew_enabled = PX4_ISFINITE(maximum_delta);
	const float delta_limit = slew_enabled ? math::max(maximum_delta, 0.f) : INFINITY;
	const float power_weight = math::constrain(power_constraint_weight, 0.f, 1.f);
	const float position_weight = math::constrain(position_constraint_weight, 0.f, 1.f);
	Vector2f power_normal = headingToNed(swing_rate_heading, yaw);
	Vector2f position_normal = position_error_ned;
	const float power_normal_norm = power_normal.norm();
	const float position_normal_norm = position_normal.norm();
	const bool use_power = power_weight > FLT_EPSILON && power_normal_norm > 1e-4f;
	const bool use_position = position_weight > FLT_EPSILON && position_normal_norm > 1e-4f;

	if (use_power) {
		power_normal /= power_normal_norm;
	}

	if (use_position) {
		position_normal /= position_normal_norm;
	}

	const float power_lower_bound = -(1.f - power_weight) * limit;
	const float position_lower_bound = -(1.f - position_weight) * limit;
	const float tolerance = math::max(1e-6f, 2e-4f * limit);

	auto project_halfspace = [](const Vector2f &value, const Vector2f &normal, float lower_bound, bool enabled) {
		if (!enabled) {
			return value;
		}

		const float alignment = value.dot(normal);
		return alignment < lower_bound ? value + (lower_bound - alignment) * normal : value;
	};

	auto satisfies = [&](const Vector2f &value) {
		return value.isAllFinite()
		       && value.norm() <= limit + tolerance
		       && (!use_power || value.dot(power_normal) >= power_lower_bound - tolerance)
		       && (!use_position || value.dot(position_normal) >= position_lower_bound - tolerance)
		       && (!slew_enabled || (value - previous_acceleration_ned).norm() <= delta_limit + tolerance);
	};

	power_limited = use_power && target_acceleration_ned.dot(power_normal) < power_lower_bound - tolerance;
	position_limited = use_position
			   && target_acceleration_ned.dot(position_normal) < position_lower_bound - tolerance;
	slew_limited = slew_enabled
			&& (target_acceleration_ned - previous_acceleration_ned).norm() > delta_limit + tolerance;

	// Dykstra projection onto one amplitude disk, two continuously tightened
	// half-spaces and the slew disk. This replaces the former
	// project/slew/re-project sequence, so the value returned here is the only
	// state update applied in this control cycle.
	Vector2f value = target_acceleration_ned;
	Vector2f residuals[4] {};
	Vector2f best{};
	float best_distance_squared = INFINITY;
	bool found_feasible = false;

	for (int iteration = 0; iteration < 32; ++iteration) {
		for (int constraint = 0; constraint < 4; ++constraint) {
			const Vector2f input = value + residuals[constraint];
			Vector2f projected = input;

			switch (constraint) {
			case 0:
				projected = constrainNorm(input, limit);
				break;

			case 1:
				projected = project_halfspace(input, power_normal, power_lower_bound, use_power);
				break;

			case 2:
				projected = project_halfspace(input, position_normal, position_lower_bound, use_position);
				break;

			case 3:
				if (slew_enabled) {
					projected = previous_acceleration_ned
						    + constrainNorm(input - previous_acceleration_ned, delta_limit);
				}

				break;
			}

			residuals[constraint] = input - projected;
			value = projected;

			if (satisfies(value)) {
				const float distance_squared = (value - target_acceleration_ned).norm_squared();

				if (!found_feasible || distance_squared < best_distance_squared) {
					best = value;
					best_distance_squared = distance_squared;
					found_feasible = true;
				}
			}
		}
	}

	if (satisfies(value)) {
		best = value;
		found_feasible = true;
	}

	if (found_feasible) {
		feasible = true;
		return best;
	}

	// A moving half-space can momentarily be incompatible with a very small
	// slew disk. In that case preserve the hard smoothness limit and decay
	// toward zero; the diagnostic flag exposes the relaxed power/position
	// constraint instead of hiding a discontinuous emergency projection.
	Vector2f fallback = previous_acceleration_ned;

	if (slew_enabled) {
		fallback += constrainNorm(-previous_acceleration_ned, delta_limit);

	} else {
		fallback.zero();
	}

	return constrainNorm(fallback, limit);
}

Vector2f SuspendedLoadFrequencySelectiveObserver::constrainNorm(const Vector2f &value, float limit)
{
	const float safe_limit = PX4_ISFINITE(limit) ? math::max(limit, 0.f) : 0.f;
	const float norm = value.norm();

	if (safe_limit <= FLT_EPSILON || !PX4_ISFINITE(norm)) {
		return Vector2f{};
	}

	return norm > safe_limit ? value * (safe_limit / norm) : value;
}

float SuspendedLoadFrequencySelectiveObserver::positionAlignment(const Vector2f &acceleration_ned,
		const Vector2f &position_error_ned)
{
	if (!acceleration_ned.isAllFinite() || !position_error_ned.isAllFinite()) {
		return NAN;
	}

	const float error_norm = position_error_ned.norm();
	return error_norm > FLT_EPSILON ? acceleration_ned.dot(position_error_ned / error_norm) : 0.f;
}

float SuspendedLoadFrequencySelectiveObserver::smoothStep(float value, float lower, float upper)
{
	if (!PX4_ISFINITE(value) || !PX4_ISFINITE(lower) || !PX4_ISFINITE(upper)) {
		return 0.f;
	}

	if (upper <= lower + FLT_EPSILON) {
		return value >= upper ? 1.f : 0.f;
	}

	const float normalized = math::constrain((value - lower) / (upper - lower), 0.f, 1.f);
	return normalized * normalized * (3.f - 2.f * normalized);
}

Vector2f SuspendedLoadFrequencySelectiveObserver::nedToHeading(const Vector2f &value_ned, float yaw)
{
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	return Vector2f{yaw_cos * value_ned(0) + yaw_sin * value_ned(1),
			-yaw_sin * value_ned(0) + yaw_cos * value_ned(1)};
}

Vector2f SuspendedLoadFrequencySelectiveObserver::headingToNed(const Vector2f &value_heading, float yaw)
{
	const float yaw_cos = cosf(yaw);
	const float yaw_sin = sinf(yaw);
	return Vector2f{yaw_cos * value_heading(0) - yaw_sin * value_heading(1),
			yaw_sin * value_heading(0) + yaw_cos * value_heading(1)};
}
