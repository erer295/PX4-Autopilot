/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "SuspendedLoadCoordinationStatus.hpp"

#include <string.h>

#include <uORB/topics/debug_array.h>

namespace suspended_load_coordination_position_status_bridge
{

static constexpr uint16_t kDebugArrayId = 685;
static constexpr const char kDebugArrayName[] = "hangpos4";

enum DataIndex : uint8_t {
	POSITION_ERROR_NORTH = 0,
	POSITION_ERROR_EAST,
	POSITION_ERROR_NORM,
	POSITION_DIRECTION_NORTH,
	POSITION_DIRECTION_EAST,
	CANDIDATE_RECOVERY_COMPONENT,
	PAS_RAW_PARALLEL_COMPONENT,
	PAS_LIMITED_PARALLEL_COMPONENT,
	PAS_PERPENDICULAR_NORM,
	PAS_POSITION_LIMITER_ACTIVE,
	PAS_POSITION_LIMITER_RATIO,
	PAS_RAW_NORTH,
	PAS_RAW_EAST,
	PAS_POSITION_LIMITED_NORTH,
	PAS_POSITION_LIMITED_EAST,
	POWER_CANDIDATE,
	POWER_AFTER_RAW_PAS,
	POWER_AFTER_POSITION_LIMITED_PAS,
	POWER_FINAL,
	BASE_ACCELERATION_NORTH,
	BASE_ACCELERATION_EAST,
	BASE_JERK_NORTH,
	BASE_JERK_EAST,
	FINAL_JERK_NORTH,
	FINAL_JERK_EAST,
	DATA_COUNT
};

static_assert(DATA_COUNT <= 58, "hangpos4 exceeds debug_array capacity");

inline void fromStatus(const SuspendedLoadCoordinationStatus &status, debug_array_s &debug_array)
{
	debug_array.timestamp = status.timestamp_sample;
	debug_array.id = kDebugArrayId;
	memset(debug_array.name, 0, sizeof(debug_array.name));
	strncpy(debug_array.name, kDebugArrayName, sizeof(debug_array.name) - 1);

	debug_array.data[POSITION_ERROR_NORTH] = status.position_error_ned(0);
	debug_array.data[POSITION_ERROR_EAST] = status.position_error_ned(1);
	debug_array.data[POSITION_ERROR_NORM] = status.position_error_norm;
	debug_array.data[POSITION_DIRECTION_NORTH] = status.position_direction_ned(0);
	debug_array.data[POSITION_DIRECTION_EAST] = status.position_direction_ned(1);
	debug_array.data[CANDIDATE_RECOVERY_COMPONENT] = status.passivity_candidate_recovery_component;
	debug_array.data[PAS_RAW_PARALLEL_COMPONENT] = status.passivity_raw_parallel_component;
	debug_array.data[PAS_LIMITED_PARALLEL_COMPONENT] = status.passivity_limited_parallel_component;
	debug_array.data[PAS_PERPENDICULAR_NORM] = status.passivity_perpendicular_component_norm;
	debug_array.data[PAS_POSITION_LIMITER_ACTIVE] = status.passivity_position_limiter_active ? 1.f : 0.f;
	debug_array.data[PAS_POSITION_LIMITER_RATIO] = status.passivity_position_limiter_ratio;
	debug_array.data[PAS_RAW_NORTH] = status.passivity_raw_correction_ned(0);
	debug_array.data[PAS_RAW_EAST] = status.passivity_raw_correction_ned(1);
	debug_array.data[PAS_POSITION_LIMITED_NORTH] = status.passivity_position_limited_correction_ned(0);
	debug_array.data[PAS_POSITION_LIMITED_EAST] = status.passivity_position_limited_correction_ned(1);
	debug_array.data[POWER_CANDIDATE] = status.passivity_candidate_power;
	debug_array.data[POWER_AFTER_RAW_PAS] = status.passivity_raw_correction_power;
	debug_array.data[POWER_AFTER_POSITION_LIMITED_PAS] = status.passivity_position_limited_power;
	debug_array.data[POWER_FINAL] = status.passivity_final_power;
	debug_array.data[BASE_ACCELERATION_NORTH] = status.base_acceleration_ned(0);
	debug_array.data[BASE_ACCELERATION_EAST] = status.base_acceleration_ned(1);
	debug_array.data[BASE_JERK_NORTH] = status.base_jerk_ned(0);
	debug_array.data[BASE_JERK_EAST] = status.base_jerk_ned(1);
	debug_array.data[FINAL_JERK_NORTH] = status.final_jerk_ned(0);
	debug_array.data[FINAL_JERK_EAST] = status.final_jerk_ned(1);
}

} // namespace suspended_load_coordination_position_status_bridge
