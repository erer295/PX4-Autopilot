/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "SuspendedLoadAntiSwing.hpp"

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
