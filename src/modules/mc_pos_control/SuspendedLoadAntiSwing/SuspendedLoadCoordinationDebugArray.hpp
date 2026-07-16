/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "SuspendedLoadCoordinationStatus.hpp"

#include <string.h>

#include <uORB/topics/debug_array.h>

namespace suspended_load_coordination_status_bridge
{

static constexpr uint16_t kDebugArrayId = 684;
static constexpr const char kDebugArrayName[] = "hangcoord";

enum DataIndex : uint8_t {
	NOMINAL_NORTH = 0,
	NOMINAL_EAST,
	DIST_RAW_NORTH,
	DIST_RAW_EAST,
	DIST_SELECTED_NORTH,
	DIST_SELECTED_EAST,
	ANTI_REQUESTED_NORTH,
	ANTI_REQUESTED_EAST,
	ANTI_APPLIED_NORTH,
	ANTI_APPLIED_EAST,
	FINAL_COMMAND_NORTH,
	FINAL_COMMAND_EAST,
	THRUST_RECONSTRUCTED_NORTH,
	THRUST_RECONSTRUCTED_EAST,
	POWER_NOMINAL,
	POWER_DIST_RAW,
	POWER_DIST_SELECTED,
	POWER_ANTI_REQUESTED,
	POWER_ANTI_APPLIED,
	POWER_FINAL_COMMAND,
	POWER_THRUST_RECONSTRUCTED,
	SWING_RATE_NORM,
	VALID,
	SELECTOR_MODE,
	SELECTOR_BLEND,
	PASSIVITY_SHADOW_CORRECTION_NORM,
	PASSIVITY_ACTIVE,
	TOTAL_ACC_SATURATED,
	PASSIVITY_SHADOW_CORRECTION_NORTH,
	PASSIVITY_SHADOW_CORRECTION_EAST,
	PASSIVITY_CANDIDATE_POWER,
	PASSIVITY_POSITIVE_POWER_FILTERED,
	PASSIVITY_PROJECTED_POWER_SHADOW,
	PASSIVITY_MODE,
	PASSIVITY_SHADOW_GATE_ACTIVE,
	PASSIVITY_DWELL_ELAPSED,
	PASSIVITY_CANDIDATE_NORTH,
	PASSIVITY_CANDIDATE_EAST,
	PASSIVITY_ACTIVE_CORRECTION_NORTH,
	PASSIVITY_ACTIVE_CORRECTION_EAST,
	PASSIVITY_PROJECTED_NORTH,
	PASSIVITY_PROJECTED_EAST,
	PASSIVITY_PROJECTED_POWER,
	SWING_ENERGY_PER_MASS,
	PASSIVITY_LIMIT_HIT,
	PASSIVITY_SLEW_ACTIVE,
	PASSIVITY_ACTIVE_APPLIED_NORTH,
	PASSIVITY_ACTIVE_APPLIED_EAST,
	NOMINAL_POSITION_NORTH,
	NOMINAL_POSITION_EAST,
	NOMINAL_VELOCITY_ERROR_NORTH,
	NOMINAL_VELOCITY_ERROR_EAST,
	POWER_NOMINAL_POSITION,
	POWER_NOMINAL_VELOCITY_ERROR,
	POWER_NOMINAL_VELOCITY_REFERENCE,
	POWER_NOMINAL_VELOCITY_STATE,
	POWER_NOMINAL_ACCELERATION_DAMPING,
	DATA_COUNT
};

static_assert(DATA_COUNT <= 58, "hangcoord exceeds debug_array capacity");

inline void fromStatus(const SuspendedLoadCoordinationStatus &status, debug_array_s &debug_array)
{
	debug_array.timestamp = status.timestamp_sample;
	debug_array.id = kDebugArrayId;
	memset(debug_array.name, 0, sizeof(debug_array.name));
	strncpy(debug_array.name, kDebugArrayName, sizeof(debug_array.name) - 1);

	debug_array.data[NOMINAL_NORTH] = status.ladrc_nominal_ned(0);
	debug_array.data[NOMINAL_EAST] = status.ladrc_nominal_ned(1);
	debug_array.data[DIST_RAW_NORTH] = status.ladrc_dist_raw_ned(0);
	debug_array.data[DIST_RAW_EAST] = status.ladrc_dist_raw_ned(1);
	debug_array.data[DIST_SELECTED_NORTH] = status.ladrc_dist_selected_ned(0);
	debug_array.data[DIST_SELECTED_EAST] = status.ladrc_dist_selected_ned(1);
	debug_array.data[ANTI_REQUESTED_NORTH] = status.anti_swing_requested_ned(0);
	debug_array.data[ANTI_REQUESTED_EAST] = status.anti_swing_requested_ned(1);
	debug_array.data[ANTI_APPLIED_NORTH] = status.anti_swing_applied_ned(0);
	debug_array.data[ANTI_APPLIED_EAST] = status.anti_swing_applied_ned(1);
	debug_array.data[FINAL_COMMAND_NORTH] = status.final_command_ned(0);
	debug_array.data[FINAL_COMMAND_EAST] = status.final_command_ned(1);
	debug_array.data[THRUST_RECONSTRUCTED_NORTH] = status.thrust_reconstructed_ned(0);
	debug_array.data[THRUST_RECONSTRUCTED_EAST] = status.thrust_reconstructed_ned(1);
	debug_array.data[POWER_NOMINAL] = status.power_nominal;
	debug_array.data[POWER_DIST_RAW] = status.power_dist_raw;
	debug_array.data[POWER_DIST_SELECTED] = status.power_dist_selected;
	debug_array.data[POWER_ANTI_REQUESTED] = status.power_anti_requested;
	debug_array.data[POWER_ANTI_APPLIED] = status.power_anti_applied;
	debug_array.data[POWER_FINAL_COMMAND] = status.power_final_command;
	debug_array.data[POWER_THRUST_RECONSTRUCTED] = status.power_thrust_reconstructed;
	debug_array.data[SWING_RATE_NORM] = status.swing_rate_norm;
	debug_array.data[VALID] = status.valid ? 1.f : 0.f;
	debug_array.data[SELECTOR_MODE] = static_cast<float>(status.selector_mode);
	debug_array.data[SELECTOR_BLEND] = status.selector_blend;
	debug_array.data[PASSIVITY_SHADOW_CORRECTION_NORM] = status.passivity_shadow_correction_norm;
	debug_array.data[PASSIVITY_ACTIVE] = status.passivity_active ? 1.f : 0.f;
	debug_array.data[TOTAL_ACC_SATURATED] = status.total_acc_saturated ? 1.f : 0.f;
	debug_array.data[PASSIVITY_SHADOW_CORRECTION_NORTH] = status.passivity_shadow_correction_ned(0);
	debug_array.data[PASSIVITY_SHADOW_CORRECTION_EAST] = status.passivity_shadow_correction_ned(1);
	debug_array.data[PASSIVITY_CANDIDATE_POWER] = status.passivity_candidate_power;
	debug_array.data[PASSIVITY_POSITIVE_POWER_FILTERED] = status.passivity_positive_power_filtered;
	debug_array.data[PASSIVITY_PROJECTED_POWER_SHADOW] = status.passivity_projected_power_shadow;
	debug_array.data[PASSIVITY_MODE] = static_cast<float>(status.passivity_mode);
	debug_array.data[PASSIVITY_SHADOW_GATE_ACTIVE] = status.passivity_shadow_gate_active ? 1.f : 0.f;
	debug_array.data[PASSIVITY_DWELL_ELAPSED] = status.passivity_dwell_elapsed;
	debug_array.data[PASSIVITY_CANDIDATE_NORTH] = status.passivity_candidate_acceleration_ned(0);
	debug_array.data[PASSIVITY_CANDIDATE_EAST] = status.passivity_candidate_acceleration_ned(1);
	debug_array.data[PASSIVITY_ACTIVE_CORRECTION_NORTH] = status.passivity_active_correction_ned(0);
	debug_array.data[PASSIVITY_ACTIVE_CORRECTION_EAST] = status.passivity_active_correction_ned(1);
	debug_array.data[PASSIVITY_PROJECTED_NORTH] = status.passivity_projected_acceleration_ned(0);
	debug_array.data[PASSIVITY_PROJECTED_EAST] = status.passivity_projected_acceleration_ned(1);
	debug_array.data[PASSIVITY_PROJECTED_POWER] = status.passivity_projected_power;
	debug_array.data[SWING_ENERGY_PER_MASS] = status.swing_energy_per_mass;
	debug_array.data[PASSIVITY_LIMIT_HIT] = status.passivity_limit_hit ? 1.f : 0.f;
	debug_array.data[PASSIVITY_SLEW_ACTIVE] = status.passivity_slew_active ? 1.f : 0.f;
	debug_array.data[PASSIVITY_ACTIVE_APPLIED_NORTH] = status.passivity_active_correction_applied_ned(0);
	debug_array.data[PASSIVITY_ACTIVE_APPLIED_EAST] = status.passivity_active_correction_applied_ned(1);
	debug_array.data[NOMINAL_POSITION_NORTH] = status.ladrc_nominal_position_ned(0);
	debug_array.data[NOMINAL_POSITION_EAST] = status.ladrc_nominal_position_ned(1);
	const matrix::Vector2f nominal_velocity_error = status.ladrc_nominal_velocity_reference_ned
			+ status.ladrc_nominal_velocity_state_ned;
	debug_array.data[NOMINAL_VELOCITY_ERROR_NORTH] = nominal_velocity_error(0);
	debug_array.data[NOMINAL_VELOCITY_ERROR_EAST] = nominal_velocity_error(1);
	debug_array.data[POWER_NOMINAL_POSITION] = status.power_nominal_position;
	debug_array.data[POWER_NOMINAL_VELOCITY_ERROR] = status.power_nominal_velocity_error;
	debug_array.data[POWER_NOMINAL_VELOCITY_REFERENCE] = status.power_nominal_velocity_reference;
	debug_array.data[POWER_NOMINAL_VELOCITY_STATE] = status.power_nominal_velocity_state;
	debug_array.data[POWER_NOMINAL_ACCELERATION_DAMPING] = status.power_nominal_acceleration_damping;
}

} // namespace suspended_load_coordination_status_bridge
