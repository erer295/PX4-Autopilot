/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file SuspendedLoadJointStateDebugArray.hpp
 *
 * Shared debug_array transport convention for suspended-load joint states.
 */

#pragma once

#include "SuspendedLoadAntiSwing.hpp"

#include <px4_platform_common/defines.h>
#include <uORB/topics/debug_array.h>

#include <string.h>

namespace suspended_load_joint_state_bridge
{

static constexpr uint16_t kDebugArrayId = 680;
static constexpr const char kDebugArrayName[] = "hangjoint";

enum DataIndex : uint8_t {
	ROLL_ANGLE = 0,
	PITCH_ANGLE,
	ROLL_RATE,
	PITCH_RATE,
	DATA_COUNT
};

inline void copyDebugArrayName(char name[10])
{
	memset(name, 0, 10);
	strncpy(name, kDebugArrayName, 9);
}

inline bool isSuspendedLoadJointState(const debug_array_s &debug_array)
{
	return debug_array.id == kDebugArrayId
	       && strncmp(debug_array.name, kDebugArrayName, sizeof(debug_array.name)) == 0;
}

inline bool toJointState(const debug_array_s &debug_array, SuspendedLoadAntiSwing::JointState &joint_state)
{
	if (!isSuspendedLoadJointState(debug_array)
	    || !PX4_ISFINITE(debug_array.data[ROLL_ANGLE])
	    || !PX4_ISFINITE(debug_array.data[PITCH_ANGLE])
	    || !PX4_ISFINITE(debug_array.data[ROLL_RATE])
	    || !PX4_ISFINITE(debug_array.data[PITCH_RATE])) {
		return false;
	}

	joint_state.roll_angle = debug_array.data[ROLL_ANGLE];
	joint_state.pitch_angle = debug_array.data[PITCH_ANGLE];
	joint_state.roll_rate = debug_array.data[ROLL_RATE];
	joint_state.pitch_rate = debug_array.data[PITCH_RATE];
	joint_state.timestamp_sample = debug_array.timestamp;
	joint_state.valid = true;

	return true;
}

inline void fromJointState(const SuspendedLoadAntiSwing::JointState &joint_state, debug_array_s &debug_array)
{
	debug_array.timestamp = joint_state.timestamp_sample;
	debug_array.id = kDebugArrayId;
	copyDebugArrayName(debug_array.name);

	debug_array.data[ROLL_ANGLE] = joint_state.roll_angle;
	debug_array.data[PITCH_ANGLE] = joint_state.pitch_angle;
	debug_array.data[ROLL_RATE] = joint_state.roll_rate;
	debug_array.data[PITCH_RATE] = joint_state.pitch_rate;
}

} // namespace suspended_load_joint_state_bridge

namespace suspended_load_anti_swing_status_bridge
{

static constexpr uint16_t kDebugArrayId = 681;
static constexpr const char kDebugArrayName[] = "hangas";

enum DataIndex : uint8_t {
	ANGLE_BODY_X = 0,
	ANGLE_BODY_Y,
	RATE_BODY_X,
	RATE_BODY_Y,
	ACCEL_REQUESTED_NORTH,
	ACCEL_REQUESTED_EAST,
	ACTIVE,
	ENGAGED,
	RAMP_SCALE,
	SAFETY_STATE,
	NATURAL_FREQUENCY,
	ENERGY_PER_MASS,
	ENERGY_GATE,
	DAMPING_GAIN,
	ACCEL_RAW_NORTH,
	ACCEL_RAW_EAST,
	ACCEL_APPLIED_NORTH,
	ACCEL_APPLIED_EAST,
	MODE,
	GAIN_SCHEDULE_SCALE,
	DATA_COUNT
};

inline void copyDebugArrayName(char name[10])
{
	memset(name, 0, 10);
	strncpy(name, kDebugArrayName, 9);
}

inline void fromStatus(const SuspendedLoadAntiSwing::Status &status, debug_array_s &debug_array)
{
	debug_array.timestamp = status.timestamp_sample;
	debug_array.id = kDebugArrayId;
	copyDebugArrayName(debug_array.name);

	debug_array.data[ANGLE_BODY_X] = status.angle_filtered(0);
	debug_array.data[ANGLE_BODY_Y] = status.angle_filtered(1);
	debug_array.data[RATE_BODY_X] = status.rate_filtered(0);
	debug_array.data[RATE_BODY_Y] = status.rate_filtered(1);
	debug_array.data[ACCEL_REQUESTED_NORTH] = status.acceleration_requested_ned(0);
	debug_array.data[ACCEL_REQUESTED_EAST] = status.acceleration_requested_ned(1);
	debug_array.data[ACTIVE] = status.active ? 1.f : 0.f;
	debug_array.data[ENGAGED] = status.engaged ? 1.f : 0.f;
	debug_array.data[RAMP_SCALE] = status.ramp_scale;
	debug_array.data[SAFETY_STATE] = status.safety_limited ? 2.f : (status.rearming ? 1.f : 0.f);
	debug_array.data[NATURAL_FREQUENCY] = status.natural_frequency;
	debug_array.data[ENERGY_PER_MASS] = status.energy_per_mass;
	debug_array.data[ENERGY_GATE] = status.energy_gate;
	debug_array.data[DAMPING_GAIN] = status.damping_gain;
	debug_array.data[ACCEL_RAW_NORTH] = status.acceleration_raw_ned(0);
	debug_array.data[ACCEL_RAW_EAST] = status.acceleration_raw_ned(1);
	debug_array.data[ACCEL_APPLIED_NORTH] = status.acceleration_applied_ned(0);
	debug_array.data[ACCEL_APPLIED_EAST] = status.acceleration_applied_ned(1);
	debug_array.data[MODE] = static_cast<float>(static_cast<int32_t>(status.mode));
	debug_array.data[GAIN_SCHEDULE_SCALE] = status.gain_schedule_scale;
}

} // namespace suspended_load_anti_swing_status_bridge
