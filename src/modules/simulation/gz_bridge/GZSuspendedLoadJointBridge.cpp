/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "GZSuspendedLoadJointBridge.hpp"

#include <drivers/drv_hrt.h>
#include <px4_platform_common/log.h>

bool GZSuspendedLoadJointBridge::init(const std::string &world_name, const std::string &model_name)
{
	const std::string joint_state_topic = "/world/" + world_name + "/model/" + model_name + "/joint_state";

	if (!_node.Subscribe(joint_state_topic, &GZSuspendedLoadJointBridge::jointStateCallback, this)) {
		PX4_WARN("optional suspended-load joint_state not available: %s", joint_state_topic.c_str());
		return true;
	}

	PX4_INFO("subscribed suspended-load joint_state: %s", joint_state_topic.c_str());
	return true;
}

void GZSuspendedLoadJointBridge::jointStateCallback(const gz::msgs::Model &msg)
{
	SuspendedLoadAntiSwing::JointState joint_state{};
	joint_state.timestamp_sample = hrt_absolute_time();

	bool roll_found = false;
	bool pitch_found = false;

	for (int i = 0; i < msg.joint_size(); ++i) {
		const gz::msgs::Joint &joint = msg.joint(i);

		if (!joint.has_axis1()) {
			continue;
		}

		const std::string &joint_name = joint.name();
		const float position = static_cast<float>(joint.axis1().position());
		const float velocity = static_cast<float>(joint.axis1().velocity());

		if (isRollJointName(joint_name)) {
			joint_state.roll_angle = position;
			joint_state.roll_rate = velocity;
			roll_found = true;

		} else if (isPitchJointName(joint_name)) {
			joint_state.pitch_angle = position;
			joint_state.pitch_rate = velocity;
			pitch_found = true;
		}
	}

	if (!roll_found || !pitch_found) {
		return;
	}

	joint_state.valid = true;

	debug_array_s debug_array{};
	suspended_load_joint_state_bridge::fromJointState(joint_state, debug_array);
	_joint_state_pub.publish(debug_array);
}

bool GZSuspendedLoadJointBridge::isRollJointName(const std::string &joint_name)
{
	return joint_name.find("hang_roll_joint") != std::string::npos;
}

bool GZSuspendedLoadJointBridge::isPitchJointName(const std::string &joint_name)
{
	return joint_name.find("hang_pitch_joint") != std::string::npos;
}
