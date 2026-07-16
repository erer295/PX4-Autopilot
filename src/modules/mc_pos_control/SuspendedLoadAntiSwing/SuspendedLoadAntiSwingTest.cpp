/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "SuspendedLoadAntiSwing.hpp"
#include "SuspendedLoadEnergySupervisor.hpp"

#include <lib/geo/geo.h>
#include <px4_platform_common/defines.h>

using namespace matrix;

namespace
{

SuspendedLoadAntiSwing::Parameters energyParameters()
{
	SuspendedLoadAntiSwing::Parameters parameters{};
	parameters.enabled = true;
	parameters.mode = SuspendedLoadAntiSwing::Mode::EnergyDamping;
	parameters.rope_length = 0.6f;
	parameters.energy_damping_ratio = 0.25f;
	parameters.energy_gate_start = 0.f;
	parameters.energy_gate_full = 0.f;
	parameters.acceleration_limit = 10.f;
	parameters.acceleration_slew_rate = 0.f;
	parameters.filter_cutoff_hz = 0.f;
	parameters.activation_delay = 0.f;
	parameters.activation_max_angle = 0.f;
	parameters.activation_max_rate = 0.f;
	parameters.activation_stable_time = 0.f;
	parameters.ramp_time = 0.f;
	parameters.abort_angle = 0.8f;
	return parameters;
}

SuspendedLoadAntiSwing::JointState jointState(uint64_t timestamp, float pitch_angle, float pitch_rate)
{
	SuspendedLoadAntiSwing::JointState state{};
	state.pitch_angle = pitch_angle;
	state.pitch_rate = pitch_rate;
	state.timestamp_sample = timestamp;
	state.valid = true;
	return state;
}

} // namespace

SuspendedLoadEnergySupervisor::Parameters shadowParameters()
{
	SuspendedLoadEnergySupervisor::Parameters parameters{};
	parameters.mode = SuspendedLoadEnergySupervisor::Mode::Shadow;
	parameters.energy_threshold = 0.f;
	parameters.rate_min = 0.f;
	parameters.gain = 0.25f;
	parameters.correction_limit = 0.2f;
	parameters.correction_slew_rate = 0.f;
	parameters.power_lpf_cutoff_hz = 0.f;
	parameters.power_deadband = 0.f;
	parameters.dwell_time = 0.f;
	return parameters;
}

SuspendedLoadEnergySupervisor::Parameters activeParameters()
{
	auto parameters = shadowParameters();
	parameters.mode = SuspendedLoadEnergySupervisor::Mode::Active;
	return parameters;
}

SuspendedLoadEnergySupervisor::Parameters positionProtectedActiveParameters()
{
	auto parameters = activeParameters();
	parameters.gain = 1.f;
	parameters.correction_limit = 10.f;
	parameters.position_recovery_protection_enabled = true;
	parameters.position_recovery_cancellation_ratio = 0.40f;
	return parameters;
}

TEST(SuspendedLoadAntiSwingTest, NaturalFrequencyFollowsRopeLength)
{
	const float frequency_short = SuspendedLoadAntiSwing::naturalFrequency(0.4f);
	const float frequency_medium = SuspendedLoadAntiSwing::naturalFrequency(0.6f);
	const float frequency_long = SuspendedLoadAntiSwing::naturalFrequency(0.8f);

	EXPECT_NEAR(frequency_short, sqrtf(CONSTANTS_ONE_G / 0.4f), 1e-6f);
	EXPECT_NEAR(frequency_medium, sqrtf(CONSTANTS_ONE_G / 0.6f), 1e-6f);
	EXPECT_NEAR(frequency_long, sqrtf(CONSTANTS_ONE_G / 0.8f), 1e-6f);
	EXPECT_GT(frequency_short, frequency_medium);
	EXPECT_GT(frequency_medium, frequency_long);
}

TEST(SuspendedLoadAntiSwingTest, EnergyIsFiniteAndPositiveAwayFromRest)
{
	EXPECT_FLOAT_EQ(SuspendedLoadAntiSwing::perUnitMassEnergy(Vector2f{}, Vector2f{}, 0.6f), 0.f);
	EXPECT_GT(SuspendedLoadAntiSwing::perUnitMassEnergy(Vector2f{0.1f, -0.05f}, Vector2f{}, 0.6f), 0.f);
	EXPECT_GT(SuspendedLoadAntiSwing::perUnitMassEnergy(Vector2f{}, Vector2f{0.2f, -0.3f}, 0.6f), 0.f);
	EXPECT_FLOAT_EQ(SuspendedLoadAntiSwing::energyGate(0.f, 0.f, 0.02f), 0.f);
	EXPECT_FLOAT_EQ(SuspendedLoadAntiSwing::energyGate(0.02f, 0.f, 0.02f), 1.f);
}

TEST(SuspendedLoadAntiSwingTest, EnergyDampingHasNonPositiveControlEnergyDerivative)
{
	SuspendedLoadAntiSwing anti_swing;
	const auto parameters = energyParameters();
	anti_swing.setParameters(parameters);
	anti_swing.setJointState(jointState(1000000, 0.1f, 0.35f));

	anti_swing.update(0.01f, 1000000, 0.f, true);
	const auto &status = anti_swing.status();
	const float energy_derivative_control =
		-parameters.rope_length * status.acceleration_raw_ned.dot(status.rate_filtered);

	EXPECT_LE(energy_derivative_control, 1e-5f);
	EXPECT_LT(energy_derivative_control, 0.f);
}

TEST(SuspendedLoadAntiSwingTest, GazeboFluJointSignsMapToPx4FrdDissipativeRate)
{
	SuspendedLoadAntiSwing anti_swing;
	auto parameters = energyParameters();
	parameters.sign_x = -1;
	parameters.sign_y = -1;
	anti_swing.setParameters(parameters);

	SuspendedLoadAntiSwing::JointState state{};
	state.pitch_angle = 0.1f;
	state.pitch_rate = 0.35f;
	state.roll_angle = -0.05f;
	state.roll_rate = -0.2f;
	state.timestamp_sample = 1000000;
	state.valid = true;
	anti_swing.setJointState(state);

	anti_swing.update(0.01f, 1000000, 0.f, true);
	const auto &status = anti_swing.status();

	EXPECT_FLOAT_EQ(status.rate_filtered(0), -state.pitch_rate);
	EXPECT_FLOAT_EQ(status.rate_filtered(1), -state.roll_rate);
	EXPECT_LT(-parameters.rope_length * status.acceleration_raw_ned.dot(status.rate_filtered), 0.f);
}

TEST(SuspendedLoadAntiSwingTest, OutputRespectsSlewMagnitudeAndTimeout)
{
	SuspendedLoadAntiSwing anti_swing;
	auto parameters = energyParameters();
	parameters.acceleration_limit = 0.3f;
	parameters.acceleration_slew_rate = 1.f;
	parameters.timeout_s = 0.2f;
	anti_swing.setParameters(parameters);
	anti_swing.setJointState(jointState(1000000, 0.1f, 4.f));

	const Vector2f first_output = anti_swing.update(0.1f, 1000000, 0.f, true);
	EXPECT_LE(first_output.norm(), 0.10001f);
	EXPECT_LE(first_output.norm(), parameters.acceleration_limit);

	const Vector2f timed_out = anti_swing.update(0.1f, 1300001, 0.f, true);
	EXPECT_EQ(timed_out, Vector2f{});
	EXPECT_FALSE(anti_swing.status().active);
}

TEST(SuspendedLoadEnergySupervisorTest, NonPositiveCandidatePowerProducesNoCorrection)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(shadowParameters());
	const auto &status = supervisor.update(0.01f, Vector2f{0.4f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			     0.f, 0.6f, 0.02f, true, true);

	EXPECT_LT(status.candidate_power, 0.f);
	EXPECT_EQ(status.correction_shadow_ned, Vector2f{});
	EXPECT_FALSE(status.gate_active);
}

TEST(SuspendedLoadEnergySupervisorTest, PositivePowerShadowCorrectionIsDissipativeAndLimited)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(shadowParameters());
	const auto &status = supervisor.update(0.01f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			     0.f, 0.6f, 0.02f, true, true);

	EXPECT_GT(status.candidate_power, 0.f);
	EXPECT_TRUE(status.gate_active);
	EXPECT_GT(status.correction_shadow_ned(0), 0.f);
	EXPECT_NEAR(status.correction_shadow_ned(1), 0.f, 1e-6f);
	EXPECT_LE(status.correction_shadow_ned.norm(), 0.050001f);
	EXPECT_LE(status.projected_power_shadow, status.candidate_power);
}

TEST(SuspendedLoadEnergySupervisorTest, OffProducesStrictZeroCorrection)
{
	SuspendedLoadEnergySupervisor supervisor;
	auto parameters = activeParameters();
	parameters.mode = SuspendedLoadEnergySupervisor::Mode::Off;
	supervisor.setParameters(parameters);
	const auto &status = supervisor.update(0.01f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			     0.f, 0.6f, 0.02f, true, true);

	EXPECT_EQ(status.correction_shadow_ned, Vector2f{});
	EXPECT_EQ(status.correction_active_ned, Vector2f{});
	EXPECT_FALSE(status.active);
}

TEST(SuspendedLoadEnergySupervisorTest, ActiveUsesSharedDissipativeCorrection)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(activeParameters());
	const auto &status = supervisor.update(0.01f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			     0.f, 0.6f, 0.02f, true, true);

	EXPECT_TRUE(status.gate_active);
	EXPECT_TRUE(status.active);
	EXPECT_EQ(status.correction_active_ned, status.correction_shadow_ned);
	EXPECT_LE(status.correction_active_ned.norm(), 0.050001f);
	EXPECT_LE(status.projected_power, status.candidate_power + 1e-6f);
}

TEST(SuspendedLoadEnergySupervisorTest, PositionProtectionDisabledMatchesThirdStageActive)
{
	SuspendedLoadEnergySupervisor baseline;
	auto baseline_parameters = positionProtectedActiveParameters();
	baseline_parameters.position_recovery_protection_enabled = false;
	baseline.setParameters(baseline_parameters);

	SuspendedLoadEnergySupervisor protected_supervisor;
	protected_supervisor.setParameters(positionProtectedActiveParameters());

	const auto &baseline_status = baseline.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);
	const auto &protected_status = protected_supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(baseline_status.position_limiter_active);
	EXPECT_TRUE(protected_status.position_limiter_active);
	EXPECT_NEAR(baseline_status.correction_active_ned(0), -1.f, 1e-4f);
	EXPECT_NEAR(protected_status.correction_active_ned(0), -0.4f, 1e-4f);
}

TEST(SuspendedLoadEnergySupervisorTest, PositionProtectionPreservesConfiguredRecoveryComponent)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(positionProtectedActiveParameters());
	const auto &status = supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_TRUE(status.position_limiter_active);
	EXPECT_NEAR(status.candidate_recovery_component, 1.f, 1e-5f);
	EXPECT_NEAR(status.raw_parallel_component, -1.f, 1e-4f);
	EXPECT_NEAR(status.limited_parallel_component, -0.4f, 1e-4f);
	EXPECT_NEAR(status.position_limiter_ratio, 0.4f, 1e-4f);
	EXPECT_NEAR((Vector2f{1.f, 0.f} + status.correction_active_ned).dot(Vector2f{1.f, 0.f}), 0.6f, 1e-4f);
}

TEST(SuspendedLoadEnergySupervisorTest, PositionProtectionBypassesSmallOrInvalidPositionError)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(positionProtectedActiveParameters());
	const auto &small_error = supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{0.049f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(small_error.position_limiter_active);
	EXPECT_NEAR(small_error.correction_active_ned(0), -1.f, 1e-4f);

	supervisor.reset();
	const auto &invalid_error = supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{NAN, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(invalid_error.position_limiter_active);
	EXPECT_NEAR(invalid_error.correction_active_ned(0), -1.f, 1e-4f);
	EXPECT_TRUE(invalid_error.correction_active_ned.isAllFinite());
}

TEST(SuspendedLoadEnergySupervisorTest, PositionProtectionLeavesHelpfulAndPerpendicularCorrectionsUntouched)
{
	SuspendedLoadEnergySupervisor supervisor;
	supervisor.setParameters(positionProtectedActiveParameters());
	const auto &helpful = supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{-1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(helpful.position_limiter_active);
	EXPECT_NEAR(helpful.correction_active_ned(0), helpful.correction_raw_ned(0), 1e-5f);

	supervisor.reset();
	const auto &perpendicular = supervisor.update(0.01f, Vector2f{1.f, -1.f}, Vector2f{0.f, 1.f},
			Vector2f{1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(perpendicular.position_limiter_active);
	EXPECT_NEAR(perpendicular.raw_parallel_component, 0.f, 1e-5f);
	EXPECT_GT(perpendicular.perpendicular_component_norm, 0.9f);
	EXPECT_NEAR(perpendicular.correction_active_ned(0), 0.f, 1e-5f);
	EXPECT_GT(perpendicular.correction_active_ned(1), 0.9f);
}

TEST(SuspendedLoadEnergySupervisorTest, PositionProtectionDoesNotChangeShadowPath)
{
	SuspendedLoadEnergySupervisor supervisor;
	auto parameters = positionProtectedActiveParameters();
	parameters.mode = SuspendedLoadEnergySupervisor::Mode::Shadow;
	supervisor.setParameters(parameters);
	const auto &status = supervisor.update(0.01f, Vector2f{1.f, 0.f}, Vector2f{-0.5f, 0.f},
			Vector2f{1.f, 0.f}, 0.f, 0.6f, 0.02f, true, true);

	EXPECT_FALSE(status.position_limiter_active);
	EXPECT_EQ(status.correction_active_ned, Vector2f{});
	EXPECT_NEAR(status.correction_shadow_ned(0), -1.f, 1e-4f);
}

TEST(SuspendedLoadEnergySupervisorTest, InvalidMeasurementSlewsTowardZeroWithoutNonfiniteOutput)
{
	SuspendedLoadEnergySupervisor supervisor;
	auto parameters = activeParameters();
	parameters.correction_slew_rate = 0.1f;
	supervisor.setParameters(parameters);
	const auto &valid = supervisor.update(0.1f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			    0.f, 0.6f, 0.02f, true, true);
	const float valid_norm = valid.correction_active_ned.norm();
	const auto &invalid = supervisor.update(0.1f, Vector2f{NAN, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
			      0.f, 0.6f, 0.02f, true, false);

	EXPECT_FALSE(invalid.valid);
	EXPECT_TRUE(invalid.correction_active_ned.isAllFinite());
	EXPECT_LE(invalid.correction_active_ned.norm(), valid_norm);
}

TEST(SuspendedLoadEnergySupervisorTest, EnergyAndRateGatesPreventIllConditionedProjection)
{
	SuspendedLoadEnergySupervisor supervisor;
	auto parameters = shadowParameters();
	parameters.energy_threshold = 0.01f;
	parameters.rate_min = 0.1f;
	supervisor.setParameters(parameters);

	const auto &low_energy = supervisor.update(0.01f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
					 0.f, 0.6f, 0.005f, true, true);
	EXPECT_FALSE(low_energy.gate_active);
	EXPECT_EQ(low_energy.correction_shadow_ned, Vector2f{});

	const auto &low_rate = supervisor.update(0.01f, Vector2f{-1.f, 0.f}, Vector2f{0.05f, 0.f}, Vector2f{},
			       0.f, 0.6f, 0.02f, true, true);
	EXPECT_FALSE(low_rate.gate_active);
	EXPECT_EQ(low_rate.correction_shadow_ned, Vector2f{});
}

TEST(SuspendedLoadEnergySupervisorTest, DwellAndSlewLimitShadowCorrection)
{
	SuspendedLoadEnergySupervisor supervisor;
	auto parameters = shadowParameters();
	parameters.correction_limit = 1.f;
	parameters.correction_slew_rate = 0.1f;
	parameters.dwell_time = 0.2f;
	supervisor.setParameters(parameters);

	const auto &before_dwell = supervisor.update(0.1f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
						    0.f, 0.6f, 0.02f, true, true);
	EXPECT_FALSE(before_dwell.gate_active);
	EXPECT_EQ(before_dwell.correction_shadow_ned, Vector2f{});

	const auto &after_dwell = supervisor.update(0.1f, Vector2f{-1.f, 0.f}, Vector2f{0.5f, 0.f}, Vector2f{},
						   0.f, 0.6f, 0.02f, true, true);
	EXPECT_TRUE(after_dwell.gate_active);
	EXPECT_GT(after_dwell.correction_shadow_ned.norm(), 0.f);
	EXPECT_LE(after_dwell.correction_shadow_ned.norm(), 0.010001f);
}
