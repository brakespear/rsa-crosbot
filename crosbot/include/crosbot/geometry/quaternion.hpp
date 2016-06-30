/*
 * quaternion.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_QUATERNION_HPP_
#define CROSBOT_GEOMETRY_QUATERNION_HPP_

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <ostream>

#include <crosbot/geometry/defines.hpp>

#ifdef ROS_VERSION
#include <geometry_msgs/Quaternion.h>
#include <tf/tf.h>
#endif

namespace crosbot {

/**
 * 3D Quaternion (x,y,z,w)
 */
struct Quaternion {
public:
    double x, y, z, w;

    Quaternion() : x(0), y(0), z(0), w(1) {}
    Quaternion(const double& x, const double& y, const double& z, const double& w) : x(x), y(y), z(z), w(w) {}
    Quaternion(const double& yaw, const double& pitch, const double& roll) {
        setYPR(yaw, pitch, roll);
    }
    Quaternion(const Quaternion& q) : x(q.x), y(q.y), z(q.z), w(q.w) {}

    inline Quaternion& operator=(const Quaternion& q) {
        x = q.x; y = q.y; z = q.z; w = q.w;
        return *this;
    }

    inline bool operator==(const Quaternion& q) const {
        return x == q.x && y == q.y && z == q.z && w == q.w;
    }

    inline bool operator!=(const Quaternion& q) const {
        return x != q.x || y != q.y || z != q.z || w != q.w;
    }

    inline bool hasNAN() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z) || std::isnan(w);
    }

    inline bool isFinite() const {
        return !hasNAN() && x != INFINITY && y != INFINITY && z != INFINITY && w != INFINITY &&
                x != -INFINITY && y != -INFINITY && z != -INFINITY && w != -INFINITY;
    }

    inline bool withinError(const Quaternion& q) const {
        return x >= q.x - CROSBOT_MATH_ERROR && x <= q.x + CROSBOT_MATH_ERROR &&
                y >= q.y - CROSBOT_MATH_ERROR && y <= q.y + CROSBOT_MATH_ERROR &&
                z >= q.z - CROSBOT_MATH_ERROR && z <= q.z + CROSBOT_MATH_ERROR &&
                w >= q.w - CROSBOT_MATH_ERROR && w <= q.w + CROSBOT_MATH_ERROR;
    }

    void setYPR(const double& yaw, const double& pitch, const double& roll);

    void getYPR(double& yaw, double& pitch, double& roll) const;

    inline static Quaternion parse(std::string str) {
        double x,y,z,w;
        if (sscanf(str.c_str(), "(%lf,%lf,%lf,%lf)", &x,&y,&z,&w) == 4) {
        } else if (sscanf(str.c_str(), "(%lf %lf %lf %lf)", &x,&y,&z,&w) == 4) {
        } else if (sscanf(str.c_str(), "(%lf, %lf, %lf, %lf)", &x,&y,&z,&w) == 4) {
        } else if (sscanf(str.c_str(), "( %lf %lf %lf %lf )", &x,&y,&z,&w) == 4) {
        } else if (sscanf(str.c_str(), "( %lf, %lf, %lf, %lf )", &x,&y,&z,&w) == 4) {
        } else if (sscanf(str.c_str(), "( %lf , %lf , %lf , %lf )", &x,&y,&z,&w) == 4) {
        } else {
            double roll, pitch, yaw;
            if (sscanf(str.c_str(), "(%lf,%lf,%lf)", &roll,&pitch,&yaw) == 3) {
            } else if (sscanf(str.c_str(), "(%lf %lf %lf)", &roll,&pitch,&yaw) == 3) {
            } else if (sscanf(str.c_str(), "(%lf, %lf, %lf)", &roll,&pitch,&yaw) == 3) {
            } else if (sscanf(str.c_str(), "( %lf %lf %lf )", &roll,&pitch,&yaw) == 3) {
            } else if (sscanf(str.c_str(), "( %lf, %lf, %lf )", &roll,&pitch,&yaw) == 3) {
            } else if (sscanf(str.c_str(), "( %lf , %lf , %lf )", &roll,&pitch,&yaw) == 3) {
            } else {
                return Quaternion();
            }
            return Quaternion(DEG2RAD(yaw),DEG2RAD(pitch),DEG2RAD(roll));
        }
        return Quaternion(x,y,z,w);
    }

#ifdef ROS_VERSION

    Quaternion(const geometry_msgs::Quaternion& q) : x(q.x), y(q.y), z(q.z), w(q.w) {}
    inline Quaternion& operator=(const geometry_msgs::Quaternion& q) {
        x = q.x; y = q.y; z = q.z; w = q.w;
        return *this;
    }

    Quaternion(const tf::Quaternion& q) : x(q.x()), y(q.y()), z(q.z()), w(q.w()) {}
    inline Quaternion& operator=(const tf::Quaternion& q) {
        x = q.x(); y = q.y(); z = q.z(); w = q.w();
        return *this;
    }

    inline geometry_msgs::Quaternion toROS() const {
        geometry_msgs::Quaternion rval;
        rval.x = x; rval.y = y; rval.z = z; rval.w = w;
        return rval;
    }

    inline tf::Quaternion toTF() const {
        return tf::Quaternion(x, y, z, w);
    }

#endif
};
inline std::ostream& operator<<(std::ostream& os, const Quaternion& q) {
    return os << "Quaternion(" << q.x << ", " << q.y << ", " << q.z <<", " << q.w << ")";
}

/**
 * Getter and setter methods for roll, pitch and yaw in a quaternion.
 */
//inline void getRPY(const geometry_msgs::Quaternion& q, double& roll, double& pitch, double& yaw) {
//    btQuaternion btq(q.x, q.y, q.z, q.w);
////    btMatrix3x3(btq).getRPY(roll, pitch, yaw);
//    btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
//}
//
//inline void setRPY(geometry_msgs::Quaternion& q, const double& roll, const double& pitch, const double& yaw) {
//    btQuaternion btq;
//    btMatrix3x3 mat;
//    mat.setEulerYPR(yaw, pitch, roll);
//    mat.getRotation(btq);
//
//    q.x = btq.x();
//    q.y = btq.y();
//    q.z = btq.z();
//    q.w = btq.w();
//}

} // namespace crosbot


#ifdef ROS_VERSION

// TODO: Confirm that these sizes are the same as the ROS message serialization sizes
ROS_STATIC_ASSERT(sizeof(crosbot::Quaternion) == 32);

namespace ros {
namespace message_traits {

template <> struct IsFixedSize<crosbot::Quaternion> : public TrueType {};
template <> struct IsSimple<crosbot::Quaternion> : public TrueType {};

template <>
struct MD5Sum<crosbot::Quaternion> {
    static const char* value()
    {
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Quaternion>::static_value1 == 0xa779879fadf01607ULL);
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Quaternion>::static_value2 == 0x34f906b8c19c7004ULL);

        return MD5Sum<geometry_msgs::Quaternion>::value();
    }

    static const char* value(const crosbot::Quaternion& m)
    {
        return MD5Sum<geometry_msgs::Quaternion>::value();
    }
};

template <>
struct DataType<crosbot::Quaternion> {
    static const char* value() {
        return DataType<geometry_msgs::Quaternion>::value();
    }

    static const char* value(const crosbot::Quaternion& m) {
        return DataType<geometry_msgs::Quaternion>::value();
    }
};

template <>
struct Definition<crosbot::Quaternion> {
    static const char* value() {
        return Definition<geometry_msgs::Quaternion>::value();
    }

    static const char* value(const crosbot::Quaternion& m) {
        return Definition<geometry_msgs::Quaternion>::value();
    }
};

} // namespace message_traits
} // namespace ros

#endif


#endif /* CROSBOT_GEOMETRY_QUATERNION_HPP_ */
