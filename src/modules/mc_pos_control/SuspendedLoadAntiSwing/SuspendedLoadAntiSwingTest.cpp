/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include <gtest/gtest.h>

#include "SuspendedLoadAntiSwing.hpp"
#include "SuspendedLoadEnergySupervisor.hpp"
#include "SuspendedLoadFrequencySelectiveObserver.hpp"

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

SuspendedLoadFrequencySelectiveObserver::Parameters frequencySelectiveParameters()
{
	SuspendedLoadFrequencySelectiveObserver::Parameters parameters{};
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::Active;
	parameters.bandwidth_hz = 0.30f;
	parameters.compensation_gain = 0.5f;
	parameters.phase_lead_s = 0.f;
	parameters.acceleration_limit = 10.f;
	parameters.acceleration_slew_rate = 0.f;
	parameters.energy_threshold = 0.f;
	parameters.rate_min = 0.f;
	parameters.settling_time = 0.f;
	parameters.position_protection_enabled = false;
	return parameters;
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

TEST(SuspendedLoadFrequencySelectiveObserverTest, CenterFrequencyFollowsRopeLength)
{
	const float frequency_hz = SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(0.6f);
	EXPECT_NEAR(frequency_hz, sqrtf(CONSTANTS_ONE_G / 0.6f) / (2.f * M_PI_F), 1e-6f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, TracksNaturalFrequencyAndRejectsSecondHarmonic)
{
	SuspendedLoadFrequencySelectiveObserver fundamental_observer;
	SuspendedLoadFrequencySelectiveObserver second_harmonic_observer;
	const auto parameters = frequencySelectiveParameters();
	fundamental_observer.setParameters(parameters);
	second_harmonic_observer.setParameters(parameters);

	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	const float frequency_hz = SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);
	float fundamental_estimate_energy = 0.f;
	float second_harmonic_estimate_energy = 0.f;
	int samples = 0;

	for (int i = 0; i < 3000; ++i) {
		const float time = i * dt;
		const Vector2f fundamental_velocity{0.4f / (2.f * M_PI_F * frequency_hz)
						     * sinf(2.f * M_PI_F * frequency_hz * time), 0.f};
		const Vector2f second_harmonic_velocity{0.4f / (4.f * M_PI_F * frequency_hz)
						       * sinf(4.f * M_PI_F * frequency_hz * time), 0.f};
		const auto &fundamental = fundamental_observer.update(dt, fundamental_velocity, Vector2f{}, Vector2f{},
				 Vector2f{1.f, 0.f}, Vector2f{}, 0.f, rope_length, 1.f, true, true);
		const auto &second_harmonic = second_harmonic_observer.update(dt, second_harmonic_velocity, Vector2f{},
					   Vector2f{}, Vector2f{1.f, 0.f}, Vector2f{}, 0.f, rope_length, 1.f, true, true);

		if (i >= 2000) {
			fundamental_estimate_energy += fundamental.disturbance_estimate_ned.norm_squared();
			second_harmonic_estimate_energy += second_harmonic.disturbance_estimate_ned.norm_squared();
			++samples;
		}
	}

	const float fundamental_rms = sqrtf(fundamental_estimate_energy / samples);
	const float second_harmonic_rms = sqrtf(second_harmonic_estimate_energy / samples);
	EXPECT_GT(fundamental_rms, 0.25f);
	EXPECT_LT(second_harmonic_rms, 0.25f * fundamental_rms);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, ShadowEstimatesButAppliesStrictZero)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::Shadow;
	observer.setParameters(parameters);
	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	const float frequency_hz = SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);

	for (int i = 0; i < 2000; ++i) {
		const float time = i * dt;
		const Vector2f velocity{0.4f / (2.f * M_PI_F * frequency_hz)
					* sinf(2.f * M_PI_F * frequency_hz * time), 0.f};
		observer.update(dt, velocity, Vector2f{}, Vector2f{}, Vector2f{0.f, 1.f}, Vector2f{},
				0.f, rope_length, 1.f, true, true);
	}

	const auto &status = observer.status();
	EXPECT_GT(status.disturbance_estimate_ned.norm(), 0.1f);
	EXPECT_GT(status.compensation_projected_ned.norm(), 0.f);
	EXPECT_FLOAT_EQ(status.compensation_applied_ned.norm(), 0.f);
	EXPECT_FALSE(status.active);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, GainScheduleEstimatesFrequencyAndNeverAddsAcceleration)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::GainSchedule;
	parameters.compensation_gain = 0.40f;
	parameters.acceleration_slew_rate = 0.f;
	parameters.energy_threshold = 0.003f;
	parameters.rate_min = 0.03f;
	parameters.settling_time = 0.f;
	parameters.confidence_min = 0.20f;
	parameters.position_protection_enabled = true;
	parameters.position_error_threshold = 0.05f;
	observer.setParameters(parameters);

	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	constexpr float true_frequency_ratio = 1.10f;
	const float nominal_frequency = 2.f * M_PI_F
				      * SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);

	for (int i = 0; i < 4000; ++i) {
		const float time = i * dt;
		const float phase = true_frequency_ratio * nominal_frequency * time;
		const Vector2f angle{0.12f * sinf(phase), 0.04f * cosf(phase)};
		const Vector2f rate{0.12f * true_frequency_ratio * nominal_frequency * cosf(phase),
				   -0.04f * true_frequency_ratio * nominal_frequency * sinf(phase)};
		observer.update(dt, Vector2f{}, Vector2f{}, angle, rate, Vector2f{},
				0.f, rope_length, 0.02f, true, true);
	}

	const auto &status = observer.status();
	EXPECT_NEAR(status.effective_frequency_ratio, true_frequency_ratio, 0.04f);
	EXPECT_GT(status.frequency_confidence, parameters.confidence_min);
	EXPECT_GT(status.gain_scale_applied, 1.f);
	EXPECT_LE(status.gain_scale_applied, 1.4f + 1e-5f);
	EXPECT_FLOAT_EQ(status.compensation_applied_ned.norm(), 0.f);
	EXPECT_TRUE(status.schedule_active);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, GainScheduleFallsBackAtLargePositionError)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::GainSchedule;
	parameters.compensation_gain = 0.40f;
	parameters.acceleration_slew_rate = 0.f;
	parameters.energy_threshold = 0.f;
	parameters.rate_min = 0.f;
	parameters.settling_time = 0.f;
	parameters.confidence_min = 0.f;
	parameters.position_protection_enabled = true;
	parameters.position_error_threshold = 0.05f;
	observer.setParameters(parameters);

	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	const float frequency = 2.f * M_PI_F
				* SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);

	for (int i = 0; i < 2500; ++i) {
		const float phase = frequency * i * dt;
		const Vector2f angle{0.1f * sinf(phase), 0.f};
		const Vector2f rate{0.1f * frequency * cosf(phase), 0.f};
		observer.update(dt, Vector2f{}, Vector2f{}, angle, rate, Vector2f{0.2f, 0.f},
				0.f, rope_length, 0.02f, true, true);
	}

	EXPECT_NEAR(observer.status().schedule_position_gate, 0.f, 1e-6f);
	EXPECT_NEAR(observer.status().gain_scale_applied, 1.f, 1e-6f);
	EXPECT_FLOAT_EQ(observer.status().compensation_applied_ned.norm(), 0.f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, RlsShadowEstimatesFrequencyButKeepsUnityGain)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::RlsShadow;
	parameters.energy_threshold = 0.003f;
	parameters.rate_min = 0.03f;
	parameters.settling_time = 0.f;
	observer.setParameters(parameters);

	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	constexpr float true_frequency_ratio = 1.10f;
	const float nominal_frequency = 2.f * M_PI_F
				      * SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);
	Vector2f angle{0.05f, -0.03f};
	Vector2f rate{};

	for (int i = 0; i < 6000; ++i) {
		const float time = i * dt;
		const Vector2f forcing{0.30f * sinf(2.f * M_PI_F * 0.17f * time),
				       0.24f * sinf(2.f * M_PI_F * 0.13f * time + 0.5f)};
		const float frequency = true_frequency_ratio * nominal_frequency;
		const Vector2f acceleration = -frequency * frequency * angle
					      - 0.08f * frequency * rate + forcing;
		rate += dt * acceleration;
		angle += dt * rate;
		const Vector2f known_acceleration_ned = -rope_length * forcing;
		observer.update(dt, Vector2f{}, known_acceleration_ned, angle, rate, Vector2f{},
				0.f, rope_length, 0.02f, true, true);
	}

	const auto &status = observer.status();
	EXPECT_NEAR(status.effective_frequency_ratio, true_frequency_ratio, 0.03f);
	EXPECT_FLOAT_EQ(status.gain_scale_raw, 1.f);
	EXPECT_FLOAT_EQ(status.gain_scale_applied, 1.f);
	EXPECT_FLOAT_EQ(status.compensation_applied_ned.norm(), 0.f);
	EXPECT_FALSE(status.schedule_active);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, OracleScheduleIsolatesGainLaw)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::OracleGainSchedule;
	parameters.compensation_gain = 0.40f;
	parameters.acceleration_slew_rate = 0.f;
	parameters.energy_threshold = 0.f;
	parameters.rate_min = 0.f;
	parameters.confidence_min = 0.20f;
	parameters.position_protection_enabled = true;
	parameters.position_error_threshold = 0.05f;
	observer.setParameters(parameters);

	observer.update(0.01f, Vector2f{}, Vector2f{}, Vector2f{0.1f, 0.f}, Vector2f{0.2f, 0.f},
			Vector2f{}, 0.f, 0.6f, 0.02f, true, true);
	EXPECT_FLOAT_EQ(observer.status().effective_frequency_ratio, 1.f);
	EXPECT_FLOAT_EQ(observer.status().frequency_confidence, 1.f);
	EXPECT_NEAR(observer.status().gain_scale_applied, 1.4f, 1e-6f);
	EXPECT_FLOAT_EQ(observer.status().compensation_applied_ned.norm(), 0.f);

	observer.update(0.01f, Vector2f{}, Vector2f{}, Vector2f{0.1f, 0.f}, Vector2f{0.2f, 0.f},
			Vector2f{0.2f, 0.f}, 0.f, 0.6f, 0.02f, true, true);
	EXPECT_NEAR(observer.status().schedule_position_gate, 0.f, 1e-6f);
	EXPECT_NEAR(observer.status().gain_scale_applied, 1.f, 1e-6f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, UnifiedShapingCoordinatorNeverAmplifiesAntiSwing)
{
	SuspendedLoadFrequencySelectiveObserver observer;
	auto parameters = frequencySelectiveParameters();
	parameters.mode = SuspendedLoadFrequencySelectiveObserver::Mode::UnifiedShapingCoordinator;
	parameters.energy_threshold = 0.003f;
	parameters.rate_min = 0.03f;
	parameters.settling_time = 0.f;
	observer.setParameters(parameters);

	constexpr float dt = 0.01f;
	constexpr float rope_length = 0.6f;
	constexpr float true_frequency_ratio = 1.08f;
	const float nominal_frequency = 2.f * M_PI_F
				      * SuspendedLoadFrequencySelectiveObserver::centerFrequencyHz(rope_length);
	Vector2f angle{0.05f, -0.03f};
	Vector2f rate{};

	for (int i = 0; i < 6000; ++i) {
		const float time = i * dt;
		const Vector2f forcing{0.30f * sinf(2.f * M_PI_F * 0.17f * time),
				       0.24f * sinf(2.f * M_PI_F * 0.13f * time + 0.5f)};
		const float frequency = true_frequency_ratio * nominal_frequency;
		const Vector2f acceleration = -frequency * frequency * angle
					      - 0.08f * frequency * rate + forcing;
		rate += dt * acceleration;
		angle += dt * rate;
		observer.update(dt, Vector2f{}, -rope_length * forcing, angle, rate, Vector2f{},
				0.f, rope_length, 0.02f, true, true);
	}

	const auto &status = observer.status();
	EXPECT_EQ(status.mode, SuspendedLoadFrequencySelectiveObserver::Mode::UnifiedShapingCoordinator);
	EXPECT_NEAR(status.effective_frequency_ratio, true_frequency_ratio, 0.03f);
	EXPECT_FLOAT_EQ(status.gain_scale_raw, 1.f);
	EXPECT_FLOAT_EQ(status.gain_scale_applied, 1.f);
	EXPECT_FLOAT_EQ(status.compensation_applied_ned.norm(), 0.f);
	EXPECT_FALSE(status.schedule_active);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, PowerProjectionNeverInjectsSwingEnergy)
{
	bool limited = false;
	const Vector2f rate_heading{0.4f, -0.2f};
	const Vector2f injecting_acceleration_ned{-0.3f, 0.15f};
	const Vector2f projected = SuspendedLoadFrequencySelectiveObserver::projectToNonPositivePower(
			injecting_acceleration_ned, rate_heading, 0.f, limited);

	EXPECT_TRUE(limited);
	EXPECT_GT(SuspendedLoadFrequencySelectiveObserver::predictedPower(
			injecting_acceleration_ned, rate_heading, 0.f, 0.6f), 0.f);
	EXPECT_LE(SuspendedLoadFrequencySelectiveObserver::predictedPower(
			projected, rate_heading, 0.f, 0.6f), 1e-6f);
	EXPECT_LE(projected.norm(), injecting_acceleration_ned.norm() + 1e-6f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, JointProjectionProtectsSwingPowerAndPositionRecovery)
{
	bool power_limited = false;
	bool position_limited = false;
	const Vector2f projected = SuspendedLoadFrequencySelectiveObserver::projectToPowerAndPositionConstraints(
			Vector2f{-0.3f, -0.4f}, Vector2f{1.f, 0.f}, Vector2f{0.f, 1.f}, 0.f, true,
			power_limited, position_limited);

	EXPECT_TRUE(power_limited);
	EXPECT_TRUE(position_limited);
	EXPECT_LE(SuspendedLoadFrequencySelectiveObserver::predictedPower(
			projected, Vector2f{1.f, 0.f}, 0.f, 0.6f), 1e-6f);
	EXPECT_GE(projected.dot(Vector2f{0.f, 1.f}), -1e-6f);
	EXPECT_NEAR(projected.norm(), 0.f, 1e-6f);

	const Vector2f boundary_projected =
		SuspendedLoadFrequencySelectiveObserver::projectToPowerAndPositionConstraints(
			Vector2f{-0.3f, 0.2f}, Vector2f{1.f, 0.f}, Vector2f{0.f, 1.f}, 0.f, true,
			power_limited, position_limited);
	EXPECT_TRUE(power_limited);
	EXPECT_FALSE(position_limited);
	EXPECT_NEAR(boundary_projected(0), 0.f, 1e-6f);
	EXPECT_NEAR(boundary_projected(1), 0.2f, 1e-6f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, SmoothUnifiedConstraintRespectsPowerPositionAndSlewTogether)
{
	bool power_limited = false;
	bool position_limited = false;
	bool slew_limited = false;
	bool feasible = false;
	const Vector2f previous{0.04f, 0.f};
	const Vector2f projected = SuspendedLoadFrequencySelectiveObserver::projectToSmoothUnifiedConstraints(
			Vector2f{-0.08f, -0.08f}, previous, Vector2f{1.f, 0.f}, Vector2f{0.f, 1.f},
			0.f, 0.08f, 0.01f, 1.f, 1.f,
			power_limited, position_limited, slew_limited, feasible);

	EXPECT_TRUE(feasible);
	EXPECT_TRUE(power_limited);
	EXPECT_TRUE(position_limited);
	EXPECT_TRUE(slew_limited);
	EXPECT_LE((projected - previous).norm(), 0.01002f);
	EXPECT_LE(projected.norm(), 0.08002f);
	EXPECT_LE(SuspendedLoadFrequencySelectiveObserver::predictedPower(
			projected, Vector2f{1.f, 0.f}, 0.f, 0.6f), 1e-5f);
	EXPECT_GE(projected.dot(Vector2f{0.f, 1.f}), -2e-5f);
}

TEST(SuspendedLoadFrequencySelectiveObserverTest, SmoothUnifiedConstraintNeverBypassesFinalSlewLimit)
{
	Vector2f previous{};
	constexpr float maximum_delta = 0.003f;

	for (int i = 0; i < 80; ++i) {
		const float angle = 0.09f * i;
		const Vector2f rate{cosf(angle), sinf(angle)};
		const Vector2f position_error{-sinf(0.7f * angle), cosf(0.7f * angle)};
		bool power_limited = false;
		bool position_limited = false;
		bool slew_limited = false;
		bool feasible = false;
		const Vector2f projected = SuspendedLoadFrequencySelectiveObserver::projectToSmoothUnifiedConstraints(
				Vector2f{-0.07f * cosf(1.3f * angle), -0.07f * sinf(1.3f * angle)},
				previous, rate, position_error, 0.f, 0.08f, maximum_delta, 1.f, 0.8f,
				power_limited, position_limited, slew_limited, feasible);

		EXPECT_TRUE(projected.isAllFinite());
		EXPECT_LE((projected - previous).norm(), maximum_delta + 2e-5f);
		EXPECT_LE(projected.norm(), 0.08002f);

		if (feasible) {
			EXPECT_LE(SuspendedLoadFrequencySelectiveObserver::predictedPower(
					projected, rate, 0.f, 0.6f), 1e-5f);
		}

		previous = projected;
	}
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
