/*
 * jointcontrol.hpp
 *
 *  Created on: 27/06/2013
 *      Author: rescue
 */
#include <ros/ros.h>
#include <crosbot_ui/renders/robot/jointcontrol.hpp>

#include <sensor_msgs/JointState.h>

namespace crosbot {

class JointControlConnection  {
public:
	ros::Subscriber jointSub;
	ros::Publisher jointPub;

	void callbackJoints(const sensor_msgs::JointStateConstPtr& state) {
		for (size_t i = 0; i < state->name.size(); ++i) {
			JointController::Joint *joint = JointController::findJoint(state->name[i]);

			{{
				Lock lock(JointController::jLock, true);
				if (joint == NULL) {
					JointController::joints.push_back(JointController::Joint(state->name[i]));
					joint = &JointController::joints[JointController::joints.size() - 1];
				}
				joint->pos = state->position[i];
			}}
		}
	}

	void connect() {
		ros::NodeHandle nh;
		if (!jointSub)
			jointSub = nh.subscribe("/joint_states", 2, &JointControlConnection::callbackJoints, this);
		if (!jointPub)
			jointPub = nh.advertise< sensor_msgs::JointState >("/joint_control", 1);
	}

	void shutdown() {
		jointPub.shutdown();
		jointSub.shutdown();
	}
};
JointControlConnection connection;

ReadWriteLock JointController::jLock;
std::vector< JointController::Joint > JointController::joints;

void JointController::connect() {
	connection.connect();
}

void JointController::shutdown() {
	connection.shutdown();
}

JointController::Joint *JointController::findJoint(const std::string& joint) {
	Lock lock(jLock);
	for (size_t i = 0; i < joints.size(); ++i) {
		if (joints[i].name == joint) {
			return &joints[i];
		}
	}
	return NULL;
}

double JointController::getPos(const std::string& joint) {
	Lock lock(jLock);
	for (size_t i = 0; i < joints.size(); ++i) {
		Joint& j = joints[i];
		if (j.name == joint)
			return j.pos;
	}

	return NAN;
}

void JointController::setPos(const std::string& joint, double pos) {
	if (!connection.jointPub)
		return;

	Joint *j = findJoint(joint);
	sensor_msgs::JointState state;
	state.header.stamp = ros::Time::now();
	state.name.push_back(joint);
	state.position.push_back(pos);

	if (j != NULL) {
		j->desiredPos = pos;
		if (j->pos < j->desiredPos) {
			state.velocity.push_back(1);
		} else {
			state.velocity.push_back(-1);
		}
	}

	connection.jointPub.publish(state);
}

void JointController::setVel(const std::string& joint, double vel) {
	if (!connection.jointPub)
		return;

	Joint *j = findJoint(joint);
	sensor_msgs::JointState state;
	state.header.stamp = ros::Time::now();
	state.name.push_back(joint);
	state.position.push_back(INFINITY);
	state.velocity.push_back(vel);

	connection.jointPub.publish(state);
}

void JointController::zero(const std::vector< std::string >& joints) {
	if (joints.size() == 0 || !connection.jointPub)
		return;

	sensor_msgs::JointState state;
	state.header.stamp = ros::Time::now();
	for (size_t i = 0; i < joints.size(); ++i) {
		Joint *j = findJoint(joints[i]);
		if (j == NULL)
			continue;
		state.name.push_back(j->name);
		state.position.push_back(0);
		j->desiredPos = 0;
	}

	connection.jointPub.publish(state);
}

} // namespace crosbot
