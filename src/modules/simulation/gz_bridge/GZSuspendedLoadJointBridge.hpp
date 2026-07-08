/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include <SuspendedLoadJointStateDebugArray.hpp>

#include <gz/msgs.hh>
#include <gz/transport.hh>

#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/debug_array.h>

#include <string>

class GZSuspendedLoadJointBridge
{
public:
	explicit GZSuspendedLoadJointBridge(gz::transport::Node &node) :
		_node(node)
	{}

	bool init(const std::string &world_name, const std::string &model_name);

private:
	void jointStateCallback(const gz::msgs::Model &msg);

	static bool isRollJointName(const std::string &joint_name);
	static bool isPitchJointName(const std::string &joint_name);

	gz::transport::Node &_node;
	uORB::PublicationMulti<debug_array_s> _joint_state_pub{ORB_ID(debug_array)};
};
