/*
 * jointcontrol.hpp
 *
 *  Created on: 27/06/2013
 *      Author: rescue
 */

#ifndef CROSBOT_JOINTCONTROL_HPP_
#define CROSBOT_JOINTCONTROL_HPP_

#include <ros/ros.h>
#include <crosbot/thread.hpp>

namespace crosbot {

class JointController {
protected:
	class Joint {
	public:
		std::string name;
		double pos, desiredPos;
		Joint(const std::string& name) : name(name), pos(NAN), desiredPos(NAN) {}
	};


	static ReadWriteLock jLock;
	static std::vector< Joint > joints;
	static Joint *findJoint(const std::string& joint);
public:
	static void connect();
	static void shutdown();
	static double getPos(const std::string& joint);
	static void setPos(const std::string& joint, double pos);
	static void setVel(const std::string& joint, double vel);
	static void zero(const std::vector< std::string >& joints);

	friend class JointControlConnection;
};

} // namespace crosbot

#endif /* CROSBOT_JOINTCONTROL_HPP_ */
