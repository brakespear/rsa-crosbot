/*
 * geometry.h
 *
 *  Created on: 28/11/2011
 *      Author: mmcgill
 */

#ifndef CROSBOT_GEOMETRY_H_
#define CROSBOT_GEOMETRY_H_

// Common defines
#include <crosbot/geometry/defines.hpp>

// Include all geometry types
#include <crosbot/geometry/index2d.hpp>
#include <crosbot/geometry/pointCloud.hpp>
#include <crosbot/geometry/points.hpp>
#include <crosbot/geometry/poses.hpp>
#include <crosbot/geometry/quaternion.hpp>

// TODO: remove
//#include <crosbot/serialization.hpp>
//#include <crosbot/serialization/data.hpp>
//#include <crosbot/serialization/geometry.hpp>

//#ifdef ROS_VERSION
//
//#include <geometry_msgs/Point.h>
//#include <geometry_msgs/PointStamped.h>
//#include <geometry_msgs/Point32.h>
//#include <geometry_msgs/Quaternion.h>
//#include <geometry_msgs/Pose.h>
//#include <geometry_msgs/PoseStamped.h>
//#include <geometry_msgs/Transform.h>
//#include <nav_msgs/Odometry.h>
//#include <tf/tf.h>
//
//#endif

//#ifdef ROS_VERSION
//
//// TODO: Confirm that these sizes are the same as the ROS message serialization sizes
//ROS_STATIC_ASSERT(sizeof(crosbot::Point3D) == 24);
//
//ROS_STATIC_ASSERT(sizeof(crosbot::Quaternion) == 32);
//ROS_STATIC_ASSERT(sizeof(crosbot::Pose3D) == 56);
//
//namespace ros {
//namespace message_traits {
//
//template <> struct IsFixedSize<crosbot::Point3D> : public TrueType {};
//template <> struct IsSimple<crosbot::Point3D> : public TrueType {};
//
//template <> struct IsFixedSize<crosbot::Quaternion> : public TrueType {};
//template <> struct IsSimple<crosbot::Quaternion> : public TrueType {};
//
//template <> struct IsFixedSize<crosbot::Pose3D> : public TrueType {};
//template <> struct IsSimple<crosbot::Pose3D> : public TrueType {};
//
//template <>
//struct MD5Sum<crosbot::Point3D> {
//	static const char* value()
//	{
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Point>::static_value1 == 0x4a842b65f413084dULL);
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Point>::static_value2 == 0xc2b10fb484ea7f17ULL);
//
//		return MD5Sum<geometry_msgs::Point>::value();
//	}
//
//	static const char* value(const crosbot::Point3D& m)
//	{
//		return MD5Sum<geometry_msgs::Point>::value();
//	}
//};
//
//template <>
//struct DataType<crosbot::Point3D> {
//	static const char* value() {
//		return DataType<geometry_msgs::Point>::value();
//	}
//
//	static const char* value(const crosbot::Point3D& m) {
//		return DataType<geometry_msgs::Point>::value();
//	}
//};
//
//template <>
//struct Definition<crosbot::Point3D> {
//	static const char* value() {
//		return Definition<geometry_msgs::Point>::value();
//	}
//
//	static const char* value(const crosbot::Point3D& m) {
//		return Definition<geometry_msgs::Point>::value();
//	}
//};
//
//template <>
//struct MD5Sum<crosbot::Quaternion> {
//	static const char* value()
//	{
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Quaternion>::static_value1 == 0xa779879fadf01607ULL);
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Quaternion>::static_value2 == 0x34f906b8c19c7004ULL);
//
//		return MD5Sum<geometry_msgs::Quaternion>::value();
//	}
//
//	static const char* value(const crosbot::Quaternion& m)
//	{
//		return MD5Sum<geometry_msgs::Quaternion>::value();
//	}
//};
//
//template <>
//struct DataType<crosbot::Quaternion> {
//	static const char* value() {
//		return DataType<geometry_msgs::Quaternion>::value();
//	}
//
//	static const char* value(const crosbot::Quaternion& m) {
//		return DataType<geometry_msgs::Quaternion>::value();
//	}
//};
//
//template <>
//struct Definition<crosbot::Quaternion> {
//	static const char* value() {
//		return Definition<geometry_msgs::Quaternion>::value();
//	}
//
//	static const char* value(const crosbot::Quaternion& m) {
//		return Definition<geometry_msgs::Quaternion>::value();
//	}
//};
//
//template <>
//struct MD5Sum<crosbot::Pose3D> {
//	static const char* value()
//	{
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Pose>::static_value1 == 0xe45d45a5a1ce597bULL);
//		ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Pose>::static_value2 == 0x249e23fb30fc871fULL);
//
//		return MD5Sum<geometry_msgs::Pose>::value();
//	}
//
//	static const char* value(const crosbot::Pose3D& m)
//	{
//		return MD5Sum<geometry_msgs::Pose>::value();
//	}
//};
//
//template <>
//struct DataType<crosbot::Pose3D> {
//	static const char* value() {
//		return DataType<geometry_msgs::Pose>::value();
//	}
//
//	static const char* value(const crosbot::Pose3D& m) {
//		return DataType<geometry_msgs::Pose>::value();
//	}
//};
//
//template <>
//struct Definition<crosbot::Pose3D> {
//	static const char* value() {
//		return Definition<geometry_msgs::Pose>::value();
//	}
//
//	static const char* value(const crosbot::Pose3D& m) {
//		return Definition<geometry_msgs::Pose>::value();
//	}
//};
//
//} // namespace message_traits
//} // namespace ros
//
//#endif /* ROS_VERSION */

#endif /* CROSBOT_GEOMETRY_H_ */
