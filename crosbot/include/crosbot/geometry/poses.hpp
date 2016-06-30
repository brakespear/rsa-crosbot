/*
 * poses.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_POSES_HPP_
#define CROSBOT_GEOMETRY_POSES_HPP_

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ostream>
#include <string>

#include <crosbot/geometry/defines.hpp>
#include <crosbot/geometry/points.hpp>
#include <crosbot/geometry/quaternion.hpp>

#ifdef ROS_VERSION
#include <nav_msgs/Odometry.h>
#endif

namespace crosbot {

struct Pose2D {
public:
    Point2D position;
    double orientation;

    Pose2D() : orientation(0) {}
    Pose2D(const Point2D& position, const double& orientation) :
        position(position), orientation(orientation) {}
    Pose2D(const Pose2D& p) : position(p.position), orientation(p.orientation) {}
    Pose2D(double x, double y, double orientation) : position(x,y), orientation(orientation) {}

    inline Pose2D& operator=(const Pose2D& p) {
        position = p.position; orientation = p.orientation;
        return *this;
    }

    inline bool operator==(const Pose2D& p) const {
        return position == p.position && orientation == p.orientation;
    }

    inline bool operator!=(const Pose2D& p) const {
        return position != p.position || orientation != p.orientation;
    }

    inline bool hasNAN() const {
        return position.hasNAN() || ::isnan(orientation);
    }

    inline bool isFinite() const {
        return position.isFinite() &&
                !std::isnan(orientation) &&
                orientation != INFINITY && orientation != -INFINITY;
    }

    inline bool withinError(const Pose2D& p) const {
        return position.withinError(p.position) &&
                orientation >= p.orientation - CROSBOT_MATH_ERROR &&
                orientation <= p.orientation + CROSBOT_MATH_ERROR;
    }

    inline static Pose2D parse(std::string str) {
        size_t f = str.find_first_of(')');
        if (f == str.npos)
            return Pose2D();
        Point2D p = Point2D::parse(str.substr(0, f+1));
        std::string aStr = str.substr(f+1);
        double a;
        if (sscanf(aStr.c_str(), "(%lf)", &a) == 1) {
        } else if (sscanf(aStr.c_str(), " (%lf)", &a) == 1) {
        } else if (sscanf(aStr.c_str(), "( %lf )", &a) == 1) {
        } else if (sscanf(aStr.c_str(), " ( %lf )", &a) == 1) {
        } else {
            a = 0;
        }

        return Pose2D(p, DEG2RAD(a));
    }
};
inline std::ostream& operator<<(std::ostream& os, const Pose2D& p) {
    return os << "Pose(" << p.position << ", " << p.orientation << ")";
}

struct Pose3D {
public:
    Point3D position;
    Quaternion orientation;

    Pose3D() {}
    Pose3D(const Point3D& position, const Quaternion& orientation) :
        position(position), orientation(orientation) {}
    Pose3D(const Pose3D& p) : position(p.position), orientation(p.orientation) {}
    Pose3D(double x, double y, double z) : position(x,y,z) {}
    Pose3D(double x, double y, double z, double yaw, double pitch, double roll) :
        position(x,y,z), orientation(yaw, pitch, roll) {}

    inline Pose3D& operator=(const Pose3D& p) {
        position = p.position; orientation = p.orientation;
        return *this;
    }

    inline bool operator==(const Pose3D& p) const {
        return position == p.position && orientation == p.orientation;
    }

    inline bool operator!=(const Pose3D& p) const {
        return position != p.position || orientation != p.orientation;
    }

    inline bool hasNAN() const {
        return position.hasNAN() || orientation.hasNAN();
    }

    inline bool isFinite() const {
        return position.isFinite() && orientation.isFinite();
    }

    inline bool withinError(const Pose3D& p) const {
        return position.withinError(p.position) && orientation.withinError(p.orientation);
    }

    inline void setYPR(const double& yaw, const double& pitch, const double& roll) {
        orientation.setYPR(yaw, pitch, roll);
    }

    inline void getYPR(double& yaw, double& pitch, double& roll) const {
        orientation.getYPR(yaw, pitch, roll);
    }

    inline static Pose3D parse(std::string str) {
        size_t f = str.find_first_of(')');
        if (f == str.npos)
            return Pose3D();
        ++f;
        Point3D p = Point3D::parse(str.substr(0, f));
        while (f < str.size() && isspace(str[f])) {
            ++f;
        }
        Quaternion q = Quaternion::parse(str.substr(f));
        return Pose3D(p, q);
    }

#ifdef ROS_VERSION

    Pose3D(const geometry_msgs::Pose& p) : position(p.position), orientation(p.orientation) {}
    inline Pose3D& operator=(const geometry_msgs::Pose& p) {
        position = p.position; orientation = p.orientation;
        return *this;
    }

    Pose3D(const geometry_msgs::PoseStamped& p) : position(p.pose.position), orientation(p.pose.orientation) {}
    inline Pose3D& operator=(const geometry_msgs::PoseStamped& p) {
        position = p.pose.position; orientation = p.pose.orientation;
        return *this;
    }

    Pose3D(const geometry_msgs::Transform& tr) : position(tr.translation), orientation(tr.rotation) {}
    inline Pose3D& operator=(const geometry_msgs::Transform& tr) {
        position = tr.translation; orientation = tr.rotation;
        return *this;
    }

    Pose3D(const tf::Transform& tr) : position(tr.getOrigin()), orientation(tr.getRotation()) {}

    Pose3D(const nav_msgs::Odometry& odom) :
        position(odom.pose.pose.position),
        orientation(odom.pose.pose.orientation)
    {}

    inline Pose3D& operator=(const nav_msgs::Odometry& odom) {
        position = odom.pose.pose.position;
        orientation = odom.pose.pose.orientation;
        return *this;
    }

    inline geometry_msgs::Pose toROS() const {
        geometry_msgs::Pose rval;
        rval.position.x = position.x;
        rval.position.y = position.y;
        rval.position.z = position.z;

        rval.orientation.x = orientation.x;
        rval.orientation.y = orientation.y;
        rval.orientation.z = orientation.z;
        rval.orientation.w = orientation.w;

        return rval;
    }

    inline tf::Transform toTF() const {
        return tf::Transform(orientation.toTF(), position.toTF());
    }

#endif
};
typedef Pose3D Pose;
inline std::ostream& operator<<(std::ostream& os, const Pose& p) {
    return os << "Pose(" << p.position << ", " << p.orientation << ")";
}

/**
 * Methods to transform between Poses and btTransforms
 */
//inline void getPoseFromTransform(const btTransform& trans, geometry_msgs::Pose& pose) {
//    const btVector3& pos = trans.getOrigin();
//    pose.position.x = pos.x();
//    pose.position.y = pos.y();
//    pose.position.z = pos.z();
//    btQuaternion q =  trans.getRotation();
//    pose.orientation.x = q.x();
//    pose.orientation.y = q.y();
//    pose.orientation.z = q.z();
//    pose.orientation.w = q.w();
//}
//
//inline void getTransformFromPose(const geometry_msgs::Pose& pose, btTransform& trans) {
//    btVector3 pos(pose.position.x, pose.position.y, pose.position.z);
//    trans.setOrigin(pos);
//    btQuaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
//    trans.setRotation(q);
//}

} // namespace crosbot


#ifdef ROS_VERSION

// TODO: Confirm that these sizes are the same as the ROS message serialization sizes
ROS_STATIC_ASSERT(sizeof(crosbot::Pose3D) == 56);

namespace ros {
namespace message_traits {

template <> struct IsFixedSize<crosbot::Pose3D> : public TrueType {};
template <> struct IsSimple<crosbot::Pose3D> : public TrueType {};

template <>
struct MD5Sum<crosbot::Pose3D> {
    static const char* value()
    {
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Pose>::static_value1 == 0xe45d45a5a1ce597bULL);
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Pose>::static_value2 == 0x249e23fb30fc871fULL);

        return MD5Sum<geometry_msgs::Pose>::value();
    }

    static const char* value(const crosbot::Pose3D& m)
    {
        return MD5Sum<geometry_msgs::Pose>::value();
    }
};

template <>
struct DataType<crosbot::Pose3D> {
    static const char* value() {
        return DataType<geometry_msgs::Pose>::value();
    }

    static const char* value(const crosbot::Pose3D& m) {
        return DataType<geometry_msgs::Pose>::value();
    }
};

template <>
struct Definition<crosbot::Pose3D> {
    static const char* value() {
        return Definition<geometry_msgs::Pose>::value();
    }

    static const char* value(const crosbot::Pose3D& m) {
        return Definition<geometry_msgs::Pose>::value();
    }
};


} // namespace message_traits
} // namespace ros

#endif


#endif /* CROSBOT_GEOMETRY_POSES_HPP_ */
