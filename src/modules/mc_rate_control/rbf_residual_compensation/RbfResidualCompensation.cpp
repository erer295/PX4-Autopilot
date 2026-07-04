/****************************************************************************
 *
 *   RBF residual compensation module for multicopter rate control research.
 *
 ****************************************************************************/

#include "RbfResidualCompensation.hpp"

#include <float.h>
#include <math.h>

#include <mathlib/mathlib.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

constexpr float kMinWidth = 1.0e-3f;
constexpr float kDefaultWidth = 1.0f;
constexpr float kMinFeatureLimit = 1.0e-3f;
constexpr float kMaxFeatureLimit = 1.0e6f;
constexpr float kMaxAdaptationGain = 1000.0f;
constexpr float kMaxOutputRateLimit = 1000.0f;
constexpr float kActivationEpsilon = 1.0e-6f;
constexpr float kMinExpArgument = -80.0f;

static inline bool isFinite(float value)
{
	return PX4_ISFINITE(value);
}

static inline uint8_t sanitizeCount(uint8_t value, uint8_t max_value)
{
	return value <= max_value ? value : max_value;
}

static inline float sanitizeNonNegative(float value, float fallback, float upper_limit)
{
	if (!isFinite(value) || value < 0.f) {
		value = fallback;
	}

	return math::constrain(value, 0.f, upper_limit);
}

static inline float sanitizeWidth(float value)
{
	if (!isFinite(value) || value < kMinWidth) {
		value = kDefaultWidth;
	}

	return fmaxf(value, kMinWidth);
}

static inline float sanitizeAlpha(float value)
{
	if (!isFinite(value)) {
		return 0.f;
	}

	return math::constrain(value, 0.f, 0.999f);
}

} // namespace

void RbfResidualCompensation::configure(const Parameters &parameters)
{
	(void)parameters.input_dimension;
	_parameters.input_dimension = kDefaultInputDimension;

	_parameters.basis_count = sanitizeCount(parameters.basis_count, kMaxBasisCount);
	_parameters.learning_rate = sanitizeNonNegative(parameters.learning_rate, 0.f, kMaxAdaptationGain);
	_parameters.leakage = sanitizeNonNegative(parameters.leakage, 0.f, kMaxAdaptationGain);
	_parameters.output_limit(0) = sanitizeNonNegative(parameters.output_limit(0), 0.f, 1.f);
	_parameters.output_limit(1) = sanitizeNonNegative(parameters.output_limit(1), 0.f, 1.f);
	_parameters.output_limit(2) = sanitizeNonNegative(parameters.output_limit(2), 0.f, 1.f);
	_parameters.output_lpf_alpha = sanitizeAlpha(parameters.output_lpf_alpha);
	_parameters.output_rate_limit = sanitizeNonNegative(parameters.output_rate_limit, 0.f, kMaxOutputRateLimit);
	_parameters.feature_limit = sanitizeNonNegative(parameters.feature_limit, 100.f, kMaxFeatureLimit);

	if (_parameters.feature_limit < kMinFeatureLimit) {
		_parameters.feature_limit = kMinFeatureLimit;
	}

	_parameters.enabled = parameters.enabled;
	_parameters.learning_enabled = parameters.learning_enabled;
	_parameters.normalize_activation = parameters.normalize_activation;

	for (size_t i = 0; i < _parameters.basis_count; i++) {
		_widths[i] = sanitizeWidth(_widths[i]);
	}

	if (!_parameters.enabled || !hasOutputLimit()) {
		resetOutputState();
	}
}

void RbfResidualCompensation::setEnabled(bool enabled)
{
	_parameters.enabled = enabled;

	if (!enabled) {
		resetOutputState();
	}
}

void RbfResidualCompensation::setLearningEnabled(bool enabled)
{
	_parameters.learning_enabled = enabled;
}

bool RbfResidualCompensation::setBasis(size_t basis_index, const FeatureVector &center, float width)
{
	if (basis_index >= kMaxBasisCount) {
		return false;
	}

	for (size_t i = 0; i < kMaxInputDimension; i++) {
		_centers[basis_index][i] = sanitizedFeatureValue(center(i));
	}

	_widths[basis_index] = sanitizeWidth(width);

	if (basis_index + 1 > _parameters.basis_count) {
		_parameters.basis_count = basis_index + 1;
	}

	return true;
}

void RbfResidualCompensation::configureRateErrorBasis(size_t basis_count, float width, float center_spacing)
{
	_parameters.basis_count = math::min(basis_count, kMaxBasisCount);

	if (_parameters.basis_count == 0) {
		return;
	}

	const float safe_width = sanitizeWidth(width);
	const float spacing = isFinite(center_spacing) ? fabsf(center_spacing) : safe_width;

	for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
		_widths[basis] = safe_width;

		for (size_t feature = 0; feature < kMaxInputDimension; feature++) {
			_centers[basis][feature] = 0.f;
		}
	}

	// Basis 0 remains centered at zero. The default 7-basis layout covers the
	// per-axis normalized residual-compensator inputs:
	//   1/2: +/- filtered rate error
	//   3/4: +/- residual angular acceleration
	//   5/6: +/- LADRC disturbance compensation torque
	// Extra bases, if configured, continue with +/- LADRC torque.
	static constexpr size_t kFeaturePattern[] = {1, 1, 4, 4, 3, 3, 2, 2};

	for (size_t basis = 1; basis < _parameters.basis_count; basis++) {
		const size_t pattern_index = (basis - 1) % (sizeof(kFeaturePattern) / sizeof(kFeaturePattern[0]));
		const size_t feature_index = kFeaturePattern[pattern_index];
		const float sign = ((basis - 1) % 2 == 0) ? 1.f : -1.f;

		_centers[basis][feature_index] = sanitizedFeatureValue(sign * spacing);
	}
}

void RbfResidualCompensation::clearBasis()
{
	for (size_t basis = 0; basis < kMaxBasisCount; basis++) {
		_widths[basis] = 0.f;

		for (size_t feature = 0; feature < kMaxInputDimension; feature++) {
			_centers[basis][feature] = 0.f;
		}
	}

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		for (size_t basis = 0; basis < kMaxBasisCount; basis++) {
			_activation[axis][basis] = 0.f;
		}
	}

	_parameters.basis_count = 0;
	resetWeights();
}

void RbfResidualCompensation::resetWeights()
{
	for (size_t axis = 0; axis < kAxisCount; axis++) {
		for (size_t basis = 0; basis < kMaxBasisCount; basis++) {
			_weights[axis][basis] = 0.f;
		}
	}

	resetOutputState();
}

void RbfResidualCompensation::reset()
{
	resetWeights();

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		for (size_t basis = 0; basis < kMaxBasisCount; basis++) {
			_activation[axis][basis] = 0.f;
		}
	}

	resetOutputState();
}

RbfResidualCompensation::FeatureVector
RbfResidualCompensation::makeFeatureVector(const LadrcBridgeInput &input) const
{
	FeatureVector features{};

	const AxisVector rate_error = input.rate_sp - input.rate;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		const size_t offset = axis * kPerAxisInputDimension;
		features(offset + 0) = 1.f;
		features(offset + 1) = sanitizedFeatureValue(rate_error(axis));
		features(offset + 2) = sanitizedFeatureValue(input.ladrc_torque(axis));
		features(offset + 3) = sanitizedFeatureValue(input.ladrc_disturbance_compensation(axis));
		features(offset + 4) = sanitizedFeatureValue(input.angular_accel(axis));
	}

	return features;
}

RbfResidualCompensation::AxisVector
RbfResidualCompensation::update(const FeatureVector &features, float dt)
{
	if (!_parameters.enabled || !hasOutputLimit() || _parameters.basis_count == 0) {
		resetOutputState();
		return _last_compensation;
	}

	computeBasisActivation(features);
	_last_output_saturated = false;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		float output = 0.f;

		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			output += _weights[axis][basis] * _activation[axis][basis];
		}

		_last_raw_output(axis) = isFinite(output) ? output : 0.f;
		_last_saturated_output(axis) = boundedResidual(_last_raw_output(axis), axis);

		const float alpha = _parameters.output_lpf_alpha;
		_last_lpf_output(axis) = alpha * _last_lpf_output(axis) + (1.f - alpha) * _last_saturated_output(axis);
		float compensation = _last_lpf_output(axis);

		if (isFinite(dt) && dt > FLT_EPSILON && _parameters.output_rate_limit > FLT_EPSILON) {
			const float max_delta = _parameters.output_rate_limit * dt;
			const float previous = isFinite(_last_compensation(axis)) ? _last_compensation(axis) : 0.f;
			compensation = math::constrain(compensation, previous - max_delta, previous + max_delta);
		}

		_last_compensation(axis) = boundedResidual(compensation, axis);

		if (fabsf(_last_raw_output(axis) - _last_saturated_output(axis)) > FLT_EPSILON) {
			_last_output_saturated = true;
		}
	}

	return _last_compensation;
}

RbfResidualCompensation::AxisVector
RbfResidualCompensation::updateFromLadrc(const LadrcBridgeInput &input, float dt)
{
	return update(makeFeatureVector(input), dt);
}

void RbfResidualCompensation::learn(const FeatureVector &features, const AxisVector &target_residual, float dt)
{
	matrix::Vector<bool, kAxisCount> valid;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		valid(axis) = true;
	}

	learn(features, target_residual, valid, dt);
}

void RbfResidualCompensation::learn(const FeatureVector &features,
				   const AxisVector &target_residual,
				   const matrix::Vector<bool, kAxisCount> &valid,
				   float dt)
{
	if (!_parameters.enabled || !_parameters.learning_enabled || _parameters.basis_count == 0) {
		return;
	}

	if (!isFinite(dt) || dt <= FLT_EPSILON) {
		return;
	}

	computeBasisActivation(features);

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		const bool adapt_axis = valid(axis) && isFinite(target_residual(axis));
		const float target = adapt_axis ? boundedResidual(target_residual(axis), axis) : 0.f;
		const float output_limit = _parameters.output_limit(axis);
		float prediction = 0.f;
		float activation_norm_sq = 1.0e-3f;

		// RBF-LADRC improvement: fit the residual target with an NLMS-style
		// prediction error instead of integrating the target directly.
		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			prediction += _weights[axis][basis] * _activation[axis][basis];
			activation_norm_sq += _activation[axis][basis] * _activation[axis][basis];
		}

		const float fit_error = target - prediction;

		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			const float adaptation = adapt_axis ?
						 _parameters.learning_rate * fit_error * _activation[axis][basis] / activation_norm_sq : 0.f;
			const float weight_dot = adaptation - _parameters.leakage * _weights[axis][basis];

			const float weight = _weights[axis][basis] + dt * weight_dot;

			if (isFinite(weight)) {
				_weights[axis][basis] = math::constrain(weight, -output_limit, output_limit);
			}
		}
	}
}

void RbfResidualCompensation::learnFromLadrc(const LadrcBridgeInput &input,
		const ResidualLearningTarget &target,
		float dt)
{
	learn(makeFeatureVector(input), target.torque_residual, target.valid, dt);
}

RbfResidualCompensation::AxisVector
RbfResidualCompensation::compensate(const AxisVector &ladrc_torque) const
{
	AxisVector torque{};

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		const float base_torque = isFinite(ladrc_torque(axis)) ? ladrc_torque(axis) : 0.f;
		const float residual = isFinite(_last_compensation(axis)) ? _last_compensation(axis) : 0.f;

		torque(axis) = math::constrain(base_torque + residual, -1.f, 1.f);
	}

	return torque;
}

void RbfResidualCompensation::decayOutput(const matrix::Vector<bool, kAxisCount> &axis_enabled, float decay)
{
	const float safe_decay = isFinite(decay) ? math::constrain(decay, 0.f, 1.f) : 0.f;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		if (axis_enabled(axis)) {
			_last_raw_output(axis) *= safe_decay;
			_last_saturated_output(axis) *= safe_decay;
			_last_lpf_output(axis) *= safe_decay;
			_last_compensation(axis) *= safe_decay;
		}
	}
}

float RbfResidualCompensation::getWeightNorm() const
{
	float norm_sq = 0.f;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			norm_sq += _weights[axis][basis] * _weights[axis][basis];
		}
	}

	return sqrtf(norm_sq);
}

float RbfResidualCompensation::getBasisActivation(size_t basis_index) const
{
	if (basis_index >= kMaxBasisCount) {
		return 0.f;
	}

	float activation = 0.f;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		activation = fmaxf(activation, _activation[axis][basis_index]);
	}

	return activation;
}

void RbfResidualCompensation::computeBasisActivation(const FeatureVector &features)
{
	_last_phi_max = 0.f;

	for (size_t axis = 0; axis < kAxisCount; axis++) {
		float activation_sum = 0.f;
		const size_t axis_offset = axis * kPerAxisInputDimension;

		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			const float width = sanitizeWidth(_widths[basis]);
			const float inv_width_sq = 1.f / (width * width);
			float distance_sq = 0.f;

			for (size_t feature = kPerAxisDistanceStart; feature < kPerAxisInputDimension; feature++) {
				const float error = sanitizedFeatureValue(features(axis_offset + feature)) - _centers[basis][feature];
				distance_sq += error * error;
			}

			float exponent = -0.5f * distance_sq * inv_width_sq;
			exponent = fmaxf(exponent, kMinExpArgument);

			_activation[axis][basis] = expf(exponent);
			activation_sum += _activation[axis][basis];
			_last_phi_max = fmaxf(_last_phi_max, _activation[axis][basis]);
		}

		for (size_t basis = _parameters.basis_count; basis < kMaxBasisCount; basis++) {
			_activation[axis][basis] = 0.f;
		}

		if (_parameters.normalize_activation && activation_sum > kActivationEpsilon) {
			for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
				_activation[axis][basis] /= activation_sum;
			}
		}

		for (size_t basis = 0; basis < _parameters.basis_count; basis++) {
			_last_phi_max = fmaxf(_last_phi_max, _activation[axis][basis]);
		}
	}
}

void RbfResidualCompensation::resetOutputState()
{
	_last_raw_output.zero();
	_last_saturated_output.zero();
	_last_lpf_output.zero();
	_last_compensation.zero();
	_last_phi_max = 0.f;
	_last_output_saturated = false;
}

bool RbfResidualCompensation::hasOutputLimit() const
{
	for (size_t axis = 0; axis < kAxisCount; axis++) {
		if (_parameters.output_limit(axis) > FLT_EPSILON) {
			return true;
		}
	}

	return false;
}

float RbfResidualCompensation::sanitizedFeatureValue(float value) const
{
	if (!isFinite(value)) {
		return 0.f;
	}

	return math::constrain(value, -_parameters.feature_limit, _parameters.feature_limit);
}

float RbfResidualCompensation::boundedResidual(float value, size_t axis) const
{
	if (axis >= kAxisCount) {
		return 0.f;
	}

	const float output_limit = _parameters.output_limit(axis);

	if (!isFinite(value) || output_limit <= FLT_EPSILON) {
		return 0.f;
	}

	return math::constrain(value, -output_limit, output_limit);
}
