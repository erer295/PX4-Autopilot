/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <matrix/matrix/math.hpp>

/** Suspended-load command-level energy coordinator shared by SHADOW and ACTIVE. */
class SuspendedLoadEnergySupervisor
{
public:
	enum class Mode : int32_t {
		Off = 0,
		Shadow = 1,
		Active = 2,
	};

	struct Parameters {
		Mode mode{Mode::Off};
		float energy_threshold{0.005f};
		float rate_min{0.05f};
		float gain{0.20f};
		float correction_limit{0.15f};
		float correction_slew_rate{0.5f};
		float power_lpf_cutoff_hz{1.f};
		float power_deadband{0.0005f};
		float dwell_time{0.10f};
		// When enabled in ACTIVE mode, retain a minimum portion of the
		// candidate acceleration that points back toward the position setpoint.
		bool position_recovery_protection_enabled{false};
		float position_recovery_cancellation_ratio{0.40f};
	};

	struct Status {
		matrix::Vector2f candidate_acceleration_ned{};
		matrix::Vector2f correction_shadow_ned{};
		matrix::Vector2f correction_active_ned{};
		matrix::Vector2f correction_raw_ned{};
		matrix::Vector2f correction_position_limited_ned{};
		matrix::Vector2f projected_acceleration_ned{};
		matrix::Vector2f position_error_ned{};
		matrix::Vector2f position_direction_ned{};
		float candidate_power{0.f};
		float positive_power_filtered{0.f};
		float projected_power_shadow{0.f};
		float projected_power{0.f};
		float raw_correction_power{0.f};
		float position_limited_power{0.f};
		float candidate_recovery_component{0.f};
		float raw_parallel_component{0.f};
		float limited_parallel_component{0.f};
		float perpendicular_component_norm{0.f};
		float position_limiter_ratio{1.f};
		float energy_per_mass{0.f};
		float dwell_elapsed{0.f};
		Mode mode{Mode::Off};
		bool valid{false};
		bool gate_active{false};
		bool active{false};
		bool correction_limit_hit{false};
		bool correction_slew_active{false};
		bool position_limiter_active{false};
	};

	void setParameters(const Parameters &parameters);
	void reset();

	/**
	 * Compute the one shared SHADOW/ACTIVE correction. The caller decides
	 * whether correction_active_ned is composed into the vehicle command.
	 */
	const Status &update(float dt,
			     const matrix::Vector2f &candidate_acceleration_ned,
			     const matrix::Vector2f &swing_rate_heading,
			     const matrix::Vector2f &position_error_ned,
			     float yaw,
			     float rope_length,
			     float energy_per_mass,
			     bool anti_swing_engaged,
			     bool measurement_valid);

	const Status &status() const { return _status; }

	static float predictedPower(const matrix::Vector2f &acceleration_ned,
				    const matrix::Vector2f &swing_rate_heading,
				    float yaw,
				    float rope_length);

private:
	static matrix::Vector2f constrainNorm(const matrix::Vector2f &value, float limit);
	static matrix::Vector2f headingToNed(const matrix::Vector2f &value_heading, float yaw);
	static matrix::Vector2f nedToHeading(const matrix::Vector2f &value_ned, float yaw);

	Parameters _parameters{};
	Status _status{};
	matrix::Vector2f _last_shadow_correction_ned{};
	matrix::Vector2f _last_active_correction_ned{};
	float _positive_power_filtered{0.f};
	float _dwell_elapsed{0.f};
};
