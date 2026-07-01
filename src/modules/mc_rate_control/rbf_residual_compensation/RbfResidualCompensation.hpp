/****************************************************************************
 *
 *   RBF residual compensation module for multicopter rate control research.
 *
 *   This module is intentionally independent from LadrcRateControl. It exposes
 *   a small bridge interface so the multicopter rate controller can feed LADRC
 *   signals into the RBF compensator without changing the LADRC implementation.
 *
 ****************************************************************************/

#pragma once

#include <stddef.h>
#include <stdint.h>

#include <lib/matrix/matrix/math.hpp>

/**
 * @brief Fixed-size RBF neural compensator for LADRC residual torque.
 *
 * Intended signal path:
 *
 *     LADRC torque u_ladrc
 *         + RBF residual torque u_rbf
 *         = final normalized torque setpoint
 *
 * The class does not subscribe, publish, allocate memory, or own PX4
 * parameters. The hosting controller is expected to:
 *
 *  - configure basis count, output limits, and adaptation gains
 *  - provide feature vectors or LadrcBridgeInput snapshots
 *  - provide a residual learning target when online adaptation is enabled
 *
 * Default construction is a no-op: output and learning are disabled.
 */
class RbfResidualCompensation
{
public:
	static constexpr size_t kAxisCount = 3;
	static constexpr size_t kMaxInputDimension = 18;
	static constexpr size_t kMaxBasisCount = 12;
	static constexpr uint8_t kDefaultInputDimension = kMaxInputDimension;

	using FeatureVector = matrix::Vector<float, kMaxInputDimension>;
	using AxisVector = matrix::Vector3f;

	struct Parameters {
		uint8_t input_dimension{kDefaultInputDimension};
		uint8_t basis_count{0};

		float learning_rate{0.f};
		float leakage{0.f};
		AxisVector output_limit{};
		float output_lpf_alpha{0.f};
		float output_rate_limit{0.f};
		float feature_limit{100.f};

		bool enabled{false};
		bool learning_enabled{false};
		bool normalize_activation{true};
	};

	/**
	 * @brief Snapshot prepared by the hosting controller after LADRC update.
	 *
	 * Feature order produced by makeFeatureVector():
	 *
	 *   0..2   rate_sp - rate
	 *   3..5   measured body rate
	 *   6..8   measured angular acceleration
	 *   9..11  LADRC torque before RBF
	 *   12..14 LADRC disturbance compensation term, if available
	 *   15..17 previous/final applied torque, if available
	 */
	struct LadrcBridgeInput {
		AxisVector rate{};
		AxisVector rate_sp{};
		AxisVector angular_accel{};
		AxisVector ladrc_torque{};
		AxisVector ladrc_disturbance_compensation{};
		AxisVector applied_torque{};
	};

	struct ResidualLearningTarget {
		AxisVector torque_residual{};
		matrix::Vector<bool, kAxisCount> valid{};
	};

	RbfResidualCompensation() = default;
	~RbfResidualCompensation() = default;

	void configure(const Parameters &parameters);
	const Parameters &parameters() const { return _parameters; }

	void setEnabled(bool enabled);
	void setLearningEnabled(bool enabled);
	bool isEnabled() const { return _parameters.enabled; }
	bool isLearningEnabled() const { return _parameters.learning_enabled; }

	bool setBasis(size_t basis_index, const FeatureVector &center, float width);
	void configureRateErrorBasis(size_t basis_count, float width, float center_spacing);
	void clearBasis();

	void resetWeights();
	void reset();

	FeatureVector makeFeatureVector(const LadrcBridgeInput &input) const;

	/**
	 * @brief Predict residual normalized torque from a feature vector.
	 */
	AxisVector update(const FeatureVector &features, float dt);

	/**
	 * @brief Convenience bridge for use immediately after LADRC update.
	 */
	AxisVector updateFromLadrc(const LadrcBridgeInput &input, float dt);

	/**
	 * @brief Adapt weights toward a residual torque target for all axes.
	 */
	void learn(const FeatureVector &features, const AxisVector &target_residual, float dt);

	/**
	 * @brief Adapt weights toward a residual torque target on selected axes.
	 *
	 * Axes marked invalid do not adapt toward the target; their weights only
	 * decay through leakage.
	 */
	void learn(const FeatureVector &features,
		   const AxisVector &target_residual,
		   const matrix::Vector<bool, kAxisCount> &valid,
		   float dt);

	void learnFromLadrc(const LadrcBridgeInput &input,
			    const ResidualLearningTarget &target,
			    float dt);

	/**
	 * @brief Add the last RBF residual to an existing LADRC torque command.
	 */
	AxisVector compensate(const AxisVector &ladrc_torque) const;

	const AxisVector &getLastCompensation() const { return _last_compensation; }
	const AxisVector &getLastRawOutput() const { return _last_raw_output; }
	float getWeightNorm() const;
	float getMaxActivation() const { return _last_phi_max; }
	bool getLastOutputSaturated() const { return _last_output_saturated; }
	float getBasisActivation(size_t basis_index) const;

private:
	void computeBasisActivation(const FeatureVector &features);
	void resetOutputState();
	bool hasOutputLimit() const;
	float sanitizedFeatureValue(float value) const;
	float boundedResidual(float value, size_t axis) const;

	Parameters _parameters{};

	float _centers[kMaxBasisCount][kMaxInputDimension] {};
	float _widths[kMaxBasisCount] {};
	float _weights[kAxisCount][kMaxBasisCount] {};
	float _activation[kMaxBasisCount] {};

	AxisVector _last_raw_output{};
	AxisVector _last_saturated_output{};
	AxisVector _last_lpf_output{};
	AxisVector _last_compensation{};
	float _last_phi_max{0.f};
	bool _last_output_saturated{false};
};
