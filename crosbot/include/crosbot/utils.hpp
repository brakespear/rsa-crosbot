/*
 * data.h
 *
 *  Created on: 08/08/2011
 *      Author: rescue
 */

#ifndef CROSBOT_UTILS_H_
#define CROSBOT_UTILS_H_

//#ifdef ROS_VERSION
//
//#ifndef MODULE_NAME
//#define MODULE_NAME		"crosbot"
//#endif
//
//#define LOG(...)		ROS_LOG(ros::console::levels::Info, MODULE_NAME, __VA_ARGS__)
//#define DEBUG(...)	ROS_LOG(ros::console::levels::Debug, MODULE_NAME, __VA_ARGS__)
//#define WARN(...)		ROS_LOG(ros::console::levels::Warn, MODULE_NAME, __VA_ARGS__)
//#define ERROR(...)	ROS_LOG(ros::console::levels::Error, MODULE_NAME, __VA_ARGS__)
//
//#else

#define LOG(...)	printf(__VA_ARGS__)
#define DEBUG(...)	printf(__VA_ARGS__)
#define WARN(...)	printf(__VA_ARGS__)
#define ERROR(...)	fprintf(stderr, __VA_ARGS__)

//#endif

#ifdef TOPIC_TOOLS_SHAPE_SHIFTER_H

template<class M>
inline bool matchesType(const topic_tools::ShapeShifter& shifter) {
	return ros::message_traits::datatype<M>() == shifter.getDataType() &&
			ros::message_traits::md5sum<M>() == shifter.getMD5Sum();
}

#endif

#endif /* CROSBOT_UTILS_H_ */
