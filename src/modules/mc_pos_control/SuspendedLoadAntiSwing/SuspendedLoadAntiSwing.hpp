/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file SuspendedLoadAntiSwing.hpp
 *
 * Optional suspended-load anti-swing outer-loop helper for multicopter
 * position control. The helper is passive unless it is enabled and supplied
 * with a fresh suspended-load joint-state measurement.
 */

#pragma once

#include <stdint.h>

#include <matrix/matrix/math.hpp>

class SuspendedLoadAntiSwing
{
public:
	struct Parameters {
		bool enabled{false};
		float rope_length{0.6f};
		float angle_gain{0.f};
		float rate_gain{1.5f};
		float acceleration_limit{0.6f};
		float acceleration_slew_rate{2.f};
		float filter_cutoff_hz{4.f};
		float max_angle{0.8f};
		float timeout_s{0.2f};
		float activation_delay{3.f};
		float activation_max_angle{0.05f};
		float activation_max_rate{0.08f};
		float activation_stable_time{2.f};
		float ramp_time{2.f};
		float safety_angle{0.12f};
		float rearm_delay{1.f};
		int sign_x{1};
		int sign_y{1};
	};

	struct JointState {
		float roll_angle{0.f};
		float pitch_angle{0.f};
		float roll_rate{0.f};
		float pitch_rate{0.f};
		uint64_t timestamp_sample{0};
		bool valid{false};
	};

	struct Status {
		matrix::Vector2f angle_filtered{};
		matrix::Vector2f rate_filtered{};
		matrix::Vector2f acceleration_ned{};
		uint64_t timestamp_sample{0};
		float ramp_scale{0.f};
		float angle_norm{0.f};
		bool active{false};
		bool engaged{false};
		bool rearming{false};
		bool safety_limited{false};
	};

	void setParameters(const Parameters &parameters);
	void setJointState(const JointState &joint_state);

	matrix::Vector2f update(float dt, uint64_t now, float yaw, bool flying);
	void reset();

	const matrix::Vector2f &lastAccelerationNed() const { return _last_acceleration_ned; }
	const Status &status() const { return _status; }
	bool active() const { return _active; }

private:
	static float sanitizeFinite(float value, float fallback);
	static matrix::Vector2f constrainNorm(const matrix::Vector2f &value, float limit);
	static float signFromInt(int value);
	static float elapsedSeconds(uint64_t now, uint64_t since);

	matrix::Vector2f filtered(const matrix::Vector2f &previous, const matrix::Vector2f &input, float dt) const;
	float activationRamp(uint64_t now) const;
	bool measurementFresh(uint64_t now) const;
	bool measurementUsable(uint64_t now, bool flying) const;
	bool activationReady(uint64_t now);
	void resetActivation(bool reset_flying_since);
	void safetyDisengage(uint64_t now);
	void updateStatus(uint64_t now);
	bool safetyLimitExceeded() const;
	matrix::Vector2f jointStateToBodyAngle() const;
	matrix::Vector2f jointStateToBodyRate() const;

	Parameters _parameters{};
	JointState _joint_state{};

	matrix::Vector2f _angle_filtered{};
	matrix::Vector2f _rate_filtered{};
	matrix::Vector2f _last_acceleration_ned{};
	Status _status{};

	uint64_t _flying_since{0};
	uint64_t _activation_ready_since{0};
	uint64_t _engaged_since{0};
	uint64_t _safety_rearm_since{0};

	float _last_ramp_scale{0.f};
	bool _filter_initialized{false};
	bool _engaged{false};
	bool _active{false};
	bool _rearming_after_safety{false};
	bool _safety_limited{false};
};
