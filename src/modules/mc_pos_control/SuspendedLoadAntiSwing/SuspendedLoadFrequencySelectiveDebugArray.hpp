/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "SuspendedLoadFrequencySelectiveObserver.hpp"

#include <string.h>
#include <uORB/topics/debug_array.h>

namespace suspended_load_frequency_selective_status_bridge
{

static constexpr uint16_t kDebugArrayId = 687;
static constexpr const char kDebugArrayName[] = "hangfso";

enum DataIndex : uint8_t {
	MEASURED_VELOCITY_NORTH = 0,
	MEASURED_VELOCITY_EAST,
	KNOWN_ACCELERATION_NORTH,
	KNOWN_ACCELERATION_EAST,
	ESTIMATED_VELOCITY_NORTH,
	ESTIMATED_VELOCITY_EAST,
	VELOCITY_INNOVATION_NORTH,
	VELOCITY_INNOVATION_EAST,
	DISTURBANCE_ESTIMATE_NORTH,
	DISTURBANCE_ESTIMATE_EAST,
	DISTURBANCE_DERIVATIVE_NORTH,
	DISTURBANCE_DERIVATIVE_EAST,
	DISTURBANCE_PHASE_LEAD_NORTH,
	DISTURBANCE_PHASE_LEAD_EAST,
	COMPENSATION_RAW_NORTH,
	COMPENSATION_RAW_EAST,
	COMPENSATION_PROJECTED_NORTH,
	COMPENSATION_PROJECTED_EAST,
	COMPENSATION_APPLIED_NORTH,
	COMPENSATION_APPLIED_EAST,
	CENTER_FREQUENCY_HZ,
	OBSERVER_ELAPSED,
	ENERGY_PER_MASS,
	POWER_RAW,
	POWER_PROJECTED,
	POWER_APPLIED,
	MODE,
	VALID,
	SETTLED,
	GATE_ACTIVE,
	ACTIVE,
	POWER_LIMITED,
	SLEW_LIMITED,
	POSITION_ERROR_NORTH,
	POSITION_ERROR_EAST,
	POSITION_ALIGNMENT_RAW,
	POSITION_ALIGNMENT_PROJECTED,
	POSITION_ALIGNMENT_APPLIED,
	POSITION_LIMITED,
	GATE_SCALE,
	POWER_CONSTRAINT_WEIGHT,
	POSITION_CONSTRAINT_WEIGHT,
	POWER_CONSTRAINT_LOWER_BOUND,
	POSITION_CONSTRAINT_LOWER_BOUND,
	UNIFIED_CONSTRAINT_FEASIBLE,
	EFFECTIVE_FREQUENCY_HZ,
	EFFECTIVE_FREQUENCY_RATIO,
	FREQUENCY_CONFIDENCE,
	CONFIDENCE_GATE,
	SCHEDULE_ENERGY_GATE,
	SCHEDULE_POSITION_GATE,
	GAIN_SCALE_RAW,
	GAIN_SCALE_APPLIED,
	FREQUENCY_BEST_BIN,
	FREQUENCY_BEST_SCORE,
	FREQUENCY_WORST_SCORE,
	FREQUENCY_SIGNAL_GATE,
	SCHEDULE_ACTIVE,
	DATA_COUNT
};

static_assert(DATA_COUNT <= 58, "hangfso exceeds debug_array capacity");

inline void fromStatus(const SuspendedLoadFrequencySelectiveObserver::Status &status,
		debug_array_s &debug_array, uint64_t timestamp)
{
	debug_array.timestamp = timestamp;
	debug_array.id = kDebugArrayId;
	memset(debug_array.name, 0, sizeof(debug_array.name));
	strncpy(debug_array.name, kDebugArrayName, sizeof(debug_array.name) - 1);
	debug_array.data[MEASURED_VELOCITY_NORTH] = status.measured_velocity_ned(0);
	debug_array.data[MEASURED_VELOCITY_EAST] = status.measured_velocity_ned(1);
	debug_array.data[KNOWN_ACCELERATION_NORTH] = status.known_acceleration_ned(0);
	debug_array.data[KNOWN_ACCELERATION_EAST] = status.known_acceleration_ned(1);
	debug_array.data[ESTIMATED_VELOCITY_NORTH] = status.estimated_velocity_ned(0);
	debug_array.data[ESTIMATED_VELOCITY_EAST] = status.estimated_velocity_ned(1);
	debug_array.data[VELOCITY_INNOVATION_NORTH] = status.velocity_innovation_ned(0);
	debug_array.data[VELOCITY_INNOVATION_EAST] = status.velocity_innovation_ned(1);
	debug_array.data[DISTURBANCE_ESTIMATE_NORTH] = status.disturbance_estimate_ned(0);
	debug_array.data[DISTURBANCE_ESTIMATE_EAST] = status.disturbance_estimate_ned(1);
	debug_array.data[DISTURBANCE_DERIVATIVE_NORTH] = status.disturbance_derivative_ned(0);
	debug_array.data[DISTURBANCE_DERIVATIVE_EAST] = status.disturbance_derivative_ned(1);
	debug_array.data[DISTURBANCE_PHASE_LEAD_NORTH] = status.disturbance_phase_lead_ned(0);
	debug_array.data[DISTURBANCE_PHASE_LEAD_EAST] = status.disturbance_phase_lead_ned(1);
	debug_array.data[COMPENSATION_RAW_NORTH] = status.compensation_raw_ned(0);
	debug_array.data[COMPENSATION_RAW_EAST] = status.compensation_raw_ned(1);
	debug_array.data[COMPENSATION_PROJECTED_NORTH] = status.compensation_projected_ned(0);
	debug_array.data[COMPENSATION_PROJECTED_EAST] = status.compensation_projected_ned(1);
	debug_array.data[COMPENSATION_APPLIED_NORTH] = status.compensation_applied_ned(0);
	debug_array.data[COMPENSATION_APPLIED_EAST] = status.compensation_applied_ned(1);
	debug_array.data[CENTER_FREQUENCY_HZ] = status.center_frequency_hz;
	debug_array.data[OBSERVER_ELAPSED] = status.observer_elapsed;
	debug_array.data[ENERGY_PER_MASS] = status.energy_per_mass;
	debug_array.data[POWER_RAW] = status.raw_compensation_power;
	debug_array.data[POWER_PROJECTED] = status.projected_compensation_power;
	debug_array.data[POWER_APPLIED] = status.applied_compensation_power;
	debug_array.data[MODE] = static_cast<float>(status.mode);
	debug_array.data[VALID] = status.valid ? 1.f : 0.f;
	debug_array.data[SETTLED] = status.settled ? 1.f : 0.f;
	debug_array.data[GATE_ACTIVE] = status.gate_active ? 1.f : 0.f;
	debug_array.data[ACTIVE] = status.active ? 1.f : 0.f;
	debug_array.data[POWER_LIMITED] = status.power_limited ? 1.f : 0.f;
	debug_array.data[SLEW_LIMITED] = status.slew_limited ? 1.f : 0.f;
	debug_array.data[POSITION_ERROR_NORTH] = status.position_error_ned(0);
	debug_array.data[POSITION_ERROR_EAST] = status.position_error_ned(1);
	debug_array.data[POSITION_ALIGNMENT_RAW] = status.raw_position_alignment;
	debug_array.data[POSITION_ALIGNMENT_PROJECTED] = status.projected_position_alignment;
	debug_array.data[POSITION_ALIGNMENT_APPLIED] = status.applied_position_alignment;
	debug_array.data[POSITION_LIMITED] = status.position_limited ? 1.f : 0.f;
	debug_array.data[GATE_SCALE] = status.gate_scale;
	debug_array.data[POWER_CONSTRAINT_WEIGHT] = status.power_constraint_weight;
	debug_array.data[POSITION_CONSTRAINT_WEIGHT] = status.position_constraint_weight;
	debug_array.data[POWER_CONSTRAINT_LOWER_BOUND] = status.power_constraint_lower_bound;
	debug_array.data[POSITION_CONSTRAINT_LOWER_BOUND] = status.position_constraint_lower_bound;
	debug_array.data[UNIFIED_CONSTRAINT_FEASIBLE] = status.unified_constraint_feasible ? 1.f : 0.f;
	debug_array.data[EFFECTIVE_FREQUENCY_HZ] = status.effective_frequency_hz;
	debug_array.data[EFFECTIVE_FREQUENCY_RATIO] = status.effective_frequency_ratio;
	debug_array.data[FREQUENCY_CONFIDENCE] = status.frequency_confidence;
	debug_array.data[CONFIDENCE_GATE] = status.confidence_gate;
	debug_array.data[SCHEDULE_ENERGY_GATE] = status.schedule_energy_gate;
	debug_array.data[SCHEDULE_POSITION_GATE] = status.schedule_position_gate;
	debug_array.data[GAIN_SCALE_RAW] = status.gain_scale_raw;
	debug_array.data[GAIN_SCALE_APPLIED] = status.gain_scale_applied;
	debug_array.data[FREQUENCY_BEST_BIN] = status.frequency_best_bin;
	debug_array.data[FREQUENCY_BEST_SCORE] = status.frequency_best_score;
	debug_array.data[FREQUENCY_WORST_SCORE] = status.frequency_worst_score;
	debug_array.data[FREQUENCY_SIGNAL_GATE] = status.frequency_signal_gate;
	debug_array.data[SCHEDULE_ACTIVE] = status.schedule_active ? 1.f : 0.f;
}

} // namespace suspended_load_frequency_selective_status_bridge
