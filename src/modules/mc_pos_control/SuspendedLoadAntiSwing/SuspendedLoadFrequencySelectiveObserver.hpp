/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <matrix/matrix/math.hpp>

/**
 * Harmonic suspended-load disturbance observer with a dissipative output.
 *
 * A third-order internal-model observer estimates the periodic acceleration
 * disturbance from horizontal velocity and the known acceleration input. Its
 * disturbance model is tuned online to the rope natural frequency. The output
 * is projected onto the half-space whose predicted contribution to swing
 * power is non-positive. It is an add-on to the stock position PID and does
 * not replace either the PID or the direct anti-swing damping term.
 */
class SuspendedLoadFrequencySelectiveObserver
{
public:
	enum class Mode : int32_t {
		Off = 0,
		Shadow = 1,
		Active = 2,
		GainSchedule = 3,
		RlsShadow = 4,
		OracleGainSchedule = 5,
		UnifiedShapingCoordinator = 6,
	};

	struct Parameters {
		Mode mode{Mode::Off};
		float bandwidth_hz{0.30f};
		float compensation_gain{0.20f};
		float phase_lead_s{0.05f};
		float acceleration_limit{0.08f};
		float acceleration_slew_rate{0.30f};
		float energy_threshold{0.003f};
		float rate_min{0.03f};
		float settling_time{2.f};
		float confidence_min{0.20f};
		bool position_protection_enabled{true};
		float position_error_threshold{0.05f};
	};

	struct Status {
		matrix::Vector2f measured_velocity_ned{};
		matrix::Vector2f known_acceleration_ned{};
		matrix::Vector2f estimated_velocity_ned{};
		matrix::Vector2f velocity_innovation_ned{};
		matrix::Vector2f disturbance_estimate_ned{};
		matrix::Vector2f disturbance_derivative_ned{};
		matrix::Vector2f disturbance_phase_lead_ned{};
		matrix::Vector2f compensation_raw_ned{};
		matrix::Vector2f compensation_projected_ned{};
		matrix::Vector2f compensation_applied_ned{};
		matrix::Vector2f position_error_ned{};
		float center_frequency_hz{0.f};
		float observer_elapsed{0.f};
		float energy_per_mass{0.f};
		float raw_compensation_power{0.f};
		float projected_compensation_power{0.f};
		float applied_compensation_power{0.f};
		float raw_position_alignment{0.f};
		float projected_position_alignment{0.f};
		float applied_position_alignment{0.f};
		float gate_scale{0.f};
		float power_constraint_weight{0.f};
		float position_constraint_weight{0.f};
		float power_constraint_lower_bound{0.f};
		float position_constraint_lower_bound{0.f};
		float effective_frequency_hz{0.f};
		float effective_frequency_ratio{1.f};
		float frequency_confidence{0.f};
		float confidence_gate{0.f};
		float schedule_energy_gate{0.f};
		float schedule_position_gate{0.f};
		float gain_scale_raw{1.f};
		float gain_scale_applied{1.f};
		float frequency_best_bin{0.f};
		float frequency_best_score{0.f};
		float frequency_worst_score{0.f};
		float frequency_signal_gate{0.f};
		Mode mode{Mode::Off};
		bool valid{false};
		bool settled{false};
		bool gate_active{false};
		bool active{false};
		bool power_limited{false};
		bool position_limited{false};
		bool slew_limited{false};
		bool unified_constraint_feasible{true};
		bool schedule_active{false};
	};

	void setParameters(const Parameters &parameters);

	const Status &update(float dt,
		     const matrix::Vector2f &measured_velocity_ned,
		     const matrix::Vector2f &known_acceleration_ned,
		     const matrix::Vector2f &swing_angle_heading,
		     const matrix::Vector2f &swing_rate_heading,
			     const matrix::Vector2f &position_error_ned,
			     float yaw,
			     float rope_length,
			     float energy_per_mass,
			     bool anti_swing_engaged,
			     bool measurement_valid);

	void reset();
	void setAppliedCompensationNed(const matrix::Vector2f &applied_compensation_ned,
				       const matrix::Vector2f &swing_rate_heading,
				       const matrix::Vector2f &position_error_ned,
				       float yaw, float rope_length);
	const Status &status() const { return _status; }

	static float centerFrequencyHz(float rope_length);
	static float predictedPower(const matrix::Vector2f &acceleration_ned,
				    const matrix::Vector2f &swing_rate_heading,
				    float yaw,
				    float rope_length);
	static matrix::Vector2f projectToNonPositivePower(const matrix::Vector2f &acceleration_ned,
			const matrix::Vector2f &swing_rate_heading, float yaw, bool &limited);
	static matrix::Vector2f projectToPowerAndPositionConstraints(
			const matrix::Vector2f &acceleration_ned,
			const matrix::Vector2f &swing_rate_heading,
			const matrix::Vector2f &position_error_ned,
			float yaw, bool position_constraint_enabled,
			bool &power_limited, bool &position_limited);
	static matrix::Vector2f projectToSmoothUnifiedConstraints(
			const matrix::Vector2f &target_acceleration_ned,
			const matrix::Vector2f &previous_acceleration_ned,
			const matrix::Vector2f &swing_rate_heading,
			const matrix::Vector2f &position_error_ned,
			float yaw, float acceleration_limit, float maximum_delta,
			float power_constraint_weight, float position_constraint_weight,
			bool &power_limited, bool &position_limited, bool &slew_limited,
			bool &feasible);

private:
	static constexpr int kFrequencyBinCount = 7;

	const Status &updateGainSchedule(float dt,
				 const matrix::Vector2f &known_acceleration_ned,
				 const matrix::Vector2f &swing_angle_heading,
				 const matrix::Vector2f &swing_rate_heading,
				 const matrix::Vector2f &position_error_ned,
				 float yaw, float rope_length, float energy_per_mass);
	const Status &updateRlsShadow(float dt,
				 const matrix::Vector2f &known_acceleration_ned,
				 const matrix::Vector2f &swing_angle_heading,
				 const matrix::Vector2f &swing_rate_heading,
				 const matrix::Vector2f &position_error_ned,
				 float yaw, float rope_length, float energy_per_mass);
	const Status &updateOracleGainSchedule(float dt,
				 const matrix::Vector2f &swing_rate_heading,
				 const matrix::Vector2f &position_error_ned,
				 float energy_per_mass);
	void updateRlsFrequencyEstimate(float dt,
				  const matrix::Vector2f &forcing_heading,
				  const matrix::Vector2f &swing_angle_heading,
				  const matrix::Vector2f &swing_rate_heading,
				  float nominal_frequency, float energy_per_mass);
	void updateScheduleFromEstimate(float dt, float frequency_ratio, float confidence,
					const matrix::Vector2f &swing_rate_heading,
					const matrix::Vector2f &position_error_ned,
					float energy_per_mass, bool apply_schedule);
	static matrix::Vector2f constrainNorm(const matrix::Vector2f &value, float limit);
	static float positionAlignment(const matrix::Vector2f &acceleration_ned,
				       const matrix::Vector2f &position_error_ned);
	static float smoothStep(float value, float lower, float upper);
	static matrix::Vector2f nedToHeading(const matrix::Vector2f &value_ned, float yaw);
	static matrix::Vector2f headingToNed(const matrix::Vector2f &value_heading, float yaw);

	Parameters _parameters{};
	Status _status{};
	matrix::Vector2f _velocity_estimate_ned{};
	matrix::Vector2f _disturbance_estimate_ned{};
	matrix::Vector2f _disturbance_derivative_ned{};
	matrix::Vector2f _last_compensation_ned{};
	matrix::Vector2f _frequency_angle_estimate[kFrequencyBinCount] {};
	matrix::Vector2f _frequency_rate_estimate[kFrequencyBinCount] {};
	matrix::Vector2f _frequency_disturbance_estimate[kFrequencyBinCount] {};
	float _frequency_fit_score[kFrequencyBinCount] {};
	float _effective_frequency_ratio{1.f};
	float _frequency_ratio_variance{0.f};
	float _frequency_confidence_filtered{0.f};
	float _last_gain_scale{1.f};
	float _observer_elapsed{0.f};
	bool _initialized{false};
	bool _frequency_bank_initialized{false};

	static constexpr int kRlsHistoryCapacity = 64;
	matrix::Vector2f _rls_angle_history[kRlsHistoryCapacity] {};
	matrix::Vector2f _rls_rate_history[kRlsHistoryCapacity] {};
	matrix::Vector2f _rls_forcing_history[kRlsHistoryCapacity] {};
	float _rls_dt_history[kRlsHistoryCapacity] {};
	float _rls_theta[2] {};
	float _rls_covariance[2][2] {};
	float _rls_information{0.f};
	float _rls_residual_variance{1e-4f};
	int _rls_history_head{0};
	int _rls_history_count{0};
	bool _rls_initialized{false};
};
