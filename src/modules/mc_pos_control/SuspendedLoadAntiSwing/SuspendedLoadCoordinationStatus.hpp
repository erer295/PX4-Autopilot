/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#include <matrix/matrix/math.hpp>

/** Horizontal suspended-load control decomposition for OFF/SHADOW/ACTIVE. */
struct SuspendedLoadCoordinationStatus {
	uint64_t timestamp_sample{0};
	matrix::Vector2f ladrc_nominal_ned{};
	matrix::Vector2f ladrc_nominal_position_ned{};
	matrix::Vector2f ladrc_nominal_velocity_reference_ned{};
	matrix::Vector2f ladrc_nominal_velocity_state_ned{};
	matrix::Vector2f ladrc_nominal_acceleration_damping_ned{};
	matrix::Vector2f ladrc_velocity_setpoint_ned{};
	matrix::Vector2f ladrc_velocity_ekf_ned{};
	matrix::Vector2f ladrc_velocity_z2_ned{};
	matrix::Vector2f ladrc_velocity_feedback_ned{};
	matrix::Vector2f ladrc_velocity_tracking_ned{};
	matrix::Vector2f ladrc_velocity_observer_error_ned{};
	matrix::Vector2f ladrc_velocity_total_ned{};
	matrix::Vector2f ladrc_vfb_requested_weight{};
	matrix::Vector2f ladrc_vfb_effective_weight{};
	matrix::Vector2f ladrc_vfb_velocity_valid{};
	matrix::Vector2f ladrc_vfb_fallback{};
	matrix::Vector2f ladrc_dist_raw_ned{};
	matrix::Vector2f ladrc_dist_selected_ned{};
	matrix::Vector2f anti_swing_requested_ned{};
	matrix::Vector2f anti_swing_applied_ned{};
	matrix::Vector2f final_command_ned{};
	matrix::Vector2f thrust_reconstructed_ned{};
	float power_nominal{0.f};
	float power_nominal_position{0.f};
	float power_nominal_velocity_reference{0.f};
	float power_nominal_velocity_state{0.f};
	float power_nominal_velocity_error{0.f};
	float power_nominal_acceleration_damping{0.f};
	float power_velocity_tracking{0.f};
	float power_velocity_observer_error{0.f};
	float power_velocity_total{0.f};
	float power_dist_raw{0.f};
	float power_dist_selected{0.f};
	float power_anti_requested{0.f};
	float power_anti_applied{0.f};
	float power_passivity_applied{0.f};
	float power_final_command{0.f};
	float power_thrust_reconstructed{0.f};
	float swing_rate_norm{0.f};
	int32_t selector_mode{0};
	float selector_blend{0.f};
	float passivity_shadow_correction_norm{0.f};
	matrix::Vector2f passivity_shadow_correction_ned{};
	matrix::Vector2f passivity_candidate_acceleration_ned{};
	matrix::Vector2f passivity_active_correction_ned{};
	matrix::Vector2f passivity_active_correction_applied_ned{};
	matrix::Vector2f passivity_projected_acceleration_ned{};
	matrix::Vector2f base_acceleration_ned{};
	matrix::Vector2f base_jerk_ned{};
	matrix::Vector2f final_jerk_ned{};
	matrix::Vector2f position_error_ned{};
	matrix::Vector2f position_direction_ned{};
	matrix::Vector2f passivity_raw_correction_ned{};
	matrix::Vector2f passivity_position_limited_correction_ned{};
	float passivity_candidate_power{0.f};
	float passivity_positive_power_filtered{0.f};
	float passivity_projected_power_shadow{0.f};
	float passivity_projected_power{0.f};
	float passivity_raw_correction_power{0.f};
	float passivity_position_limited_power{0.f};
	float passivity_final_power{0.f};
	float position_error_norm{0.f};
	float passivity_candidate_recovery_component{0.f};
	float passivity_raw_parallel_component{0.f};
	float passivity_limited_parallel_component{0.f};
	float passivity_perpendicular_component_norm{0.f};
	float passivity_position_limiter_ratio{1.f};
	float swing_energy_per_mass{0.f};
	float passivity_dwell_elapsed{0.f};
	int32_t passivity_mode{0};
	bool passivity_shadow_gate_active{false};
	bool passivity_active{false};
	bool passivity_limit_hit{false};
	bool passivity_slew_active{false};
	bool passivity_position_limiter_active{false};
	bool total_acc_saturated{false};
	bool valid{false};
};
