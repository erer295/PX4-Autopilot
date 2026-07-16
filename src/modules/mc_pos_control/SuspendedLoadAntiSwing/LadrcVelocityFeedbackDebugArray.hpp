/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "SuspendedLoadCoordinationStatus.hpp"

#include <string.h>

#include <uORB/topics/debug_array.h>

namespace ladrc_velocity_feedback_status_bridge
{

static constexpr uint16_t kDebugArrayId = 686;
static constexpr const char kDebugArrayName[] = "hangvfb";

enum DataIndex : uint8_t {
	VELOCITY_SETPOINT_NORTH = 0,
	VELOCITY_SETPOINT_EAST,
	VELOCITY_EKF_NORTH,
	VELOCITY_EKF_EAST,
	VELOCITY_Z2_NORTH,
	VELOCITY_Z2_EAST,
	VELOCITY_FEEDBACK_NORTH,
	VELOCITY_FEEDBACK_EAST,
	SETPOINT_MINUS_EKF_NORTH,
	SETPOINT_MINUS_EKF_EAST,
	EKF_MINUS_Z2_NORTH,
	EKF_MINUS_Z2_EAST,
	SETPOINT_MINUS_FEEDBACK_NORTH,
	SETPOINT_MINUS_FEEDBACK_EAST,
	ACCELERATION_POSITION_NORTH,
	ACCELERATION_POSITION_EAST,
	ACCELERATION_TRACKING_NORTH,
	ACCELERATION_TRACKING_EAST,
	ACCELERATION_OBSERVER_ERROR_NORTH,
	ACCELERATION_OBSERVER_ERROR_EAST,
	ACCELERATION_VELOCITY_TOTAL_NORTH,
	ACCELERATION_VELOCITY_TOTAL_EAST,
	ACCELERATION_NOMINAL_NORTH,
	ACCELERATION_NOMINAL_EAST,
	ACCELERATION_Z3_NORTH,
	ACCELERATION_Z3_EAST,
	ACCELERATION_AS_NORTH,
	ACCELERATION_AS_EAST,
	ACCELERATION_PAS_NORTH,
	ACCELERATION_PAS_EAST,
	ACCELERATION_FINAL_NORTH,
	ACCELERATION_FINAL_EAST,
	REQUESTED_WEIGHT,
	EFFECTIVE_WEIGHT,
	FALLBACK_ACTIVE,
	VELOCITY_VALID,
	POWER_POSITION,
	POWER_TRACKING,
	POWER_OBSERVER_ERROR,
	POWER_VELOCITY_TOTAL,
	POWER_NOMINAL,
	POWER_Z3,
	POWER_AS,
	POWER_PAS,
	POWER_FINAL,
	CONTROLLER_BANDWIDTH,
	OBSERVER_BANDWIDTH,
	NOMINAL_IDENTITY_ERROR_NORTH,
	NOMINAL_IDENTITY_ERROR_EAST,
	VELOCITY_IDENTITY_ERROR_NORTH,
	VELOCITY_IDENTITY_ERROR_EAST,
	DATA_COUNT
};

static_assert(DATA_COUNT <= 58, "hangvfb exceeds debug_array capacity");

inline void fromStatus(const SuspendedLoadCoordinationStatus &status,
		float controller_bandwidth, float observer_bandwidth, debug_array_s &debug_array)
{
	debug_array.timestamp = status.timestamp_sample;
	debug_array.id = kDebugArrayId;
	memset(debug_array.name, 0, sizeof(debug_array.name));
	strncpy(debug_array.name, kDebugArrayName, sizeof(debug_array.name) - 1);

	const matrix::Vector2f setpoint_minus_ekf = status.ladrc_velocity_setpoint_ned - status.ladrc_velocity_ekf_ned;
	const matrix::Vector2f ekf_minus_z2 = status.ladrc_velocity_ekf_ned - status.ladrc_velocity_z2_ned;
	const matrix::Vector2f setpoint_minus_feedback =
		status.ladrc_velocity_setpoint_ned - status.ladrc_velocity_feedback_ned;
	const matrix::Vector2f nominal_identity_error = status.ladrc_nominal_ned
		- status.ladrc_nominal_position_ned - status.ladrc_velocity_total_ned
		- status.ladrc_nominal_acceleration_damping_ned;
	const matrix::Vector2f velocity_identity_error = status.ladrc_velocity_total_ned
		- status.ladrc_velocity_tracking_ned - status.ladrc_velocity_observer_error_ned;

	status.ladrc_velocity_setpoint_ned.copyTo(&debug_array.data[VELOCITY_SETPOINT_NORTH]);
	status.ladrc_velocity_ekf_ned.copyTo(&debug_array.data[VELOCITY_EKF_NORTH]);
	status.ladrc_velocity_z2_ned.copyTo(&debug_array.data[VELOCITY_Z2_NORTH]);
	status.ladrc_velocity_feedback_ned.copyTo(&debug_array.data[VELOCITY_FEEDBACK_NORTH]);
	setpoint_minus_ekf.copyTo(&debug_array.data[SETPOINT_MINUS_EKF_NORTH]);
	ekf_minus_z2.copyTo(&debug_array.data[EKF_MINUS_Z2_NORTH]);
	setpoint_minus_feedback.copyTo(&debug_array.data[SETPOINT_MINUS_FEEDBACK_NORTH]);
	status.ladrc_nominal_position_ned.copyTo(&debug_array.data[ACCELERATION_POSITION_NORTH]);
	status.ladrc_velocity_tracking_ned.copyTo(&debug_array.data[ACCELERATION_TRACKING_NORTH]);
	status.ladrc_velocity_observer_error_ned.copyTo(&debug_array.data[ACCELERATION_OBSERVER_ERROR_NORTH]);
	status.ladrc_velocity_total_ned.copyTo(&debug_array.data[ACCELERATION_VELOCITY_TOTAL_NORTH]);
	status.ladrc_nominal_ned.copyTo(&debug_array.data[ACCELERATION_NOMINAL_NORTH]);
	status.ladrc_dist_selected_ned.copyTo(&debug_array.data[ACCELERATION_Z3_NORTH]);
	status.anti_swing_applied_ned.copyTo(&debug_array.data[ACCELERATION_AS_NORTH]);
	status.passivity_active_correction_applied_ned.copyTo(&debug_array.data[ACCELERATION_PAS_NORTH]);
	status.final_command_ned.copyTo(&debug_array.data[ACCELERATION_FINAL_NORTH]);
	debug_array.data[REQUESTED_WEIGHT] = 0.5f * (status.ladrc_vfb_requested_weight(0)
			+ status.ladrc_vfb_requested_weight(1));
	debug_array.data[EFFECTIVE_WEIGHT] = 0.5f * (status.ladrc_vfb_effective_weight(0)
			+ status.ladrc_vfb_effective_weight(1));
	debug_array.data[FALLBACK_ACTIVE] = status.ladrc_vfb_fallback(0) > 0.5f
			|| status.ladrc_vfb_fallback(1) > 0.5f ? 1.f : 0.f;
	debug_array.data[VELOCITY_VALID] = status.ladrc_vfb_velocity_valid(0) > 0.5f
			&& status.ladrc_vfb_velocity_valid(1) > 0.5f ? 1.f : 0.f;
	debug_array.data[POWER_POSITION] = status.power_nominal_position;
	debug_array.data[POWER_TRACKING] = status.power_velocity_tracking;
	debug_array.data[POWER_OBSERVER_ERROR] = status.power_velocity_observer_error;
	debug_array.data[POWER_VELOCITY_TOTAL] = status.power_velocity_total;
	debug_array.data[POWER_NOMINAL] = status.power_nominal;
	debug_array.data[POWER_Z3] = status.power_dist_selected;
	debug_array.data[POWER_AS] = status.power_anti_applied;
	debug_array.data[POWER_PAS] = status.power_passivity_applied;
	debug_array.data[POWER_FINAL] = status.power_final_command;
	debug_array.data[CONTROLLER_BANDWIDTH] = controller_bandwidth;
	debug_array.data[OBSERVER_BANDWIDTH] = observer_bandwidth;
	nominal_identity_error.copyTo(&debug_array.data[NOMINAL_IDENTITY_ERROR_NORTH]);
	velocity_identity_error.copyTo(&debug_array.data[VELOCITY_IDENTITY_ERROR_NORTH]);
}

} // namespace ladrc_velocity_feedback_status_bridge
