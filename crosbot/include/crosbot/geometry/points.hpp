/*
 * points.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_POINTS_HPP_
#define CROSBOT_GEOMETRY_POINTS_HPP_

#include <cmath>
#include <cstdint>
#include <ostream>
#include <sstream>
#include <string>

#include <crosbot/data/colour.hpp>
#include <crosbot/geometry/defines.hpp>

#ifdef ROS_VERSION
#include <crosbot/ColouredPointMsg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Point32.h>
#include <tf/tf.h>
#endif

namespace crosbot {

/**
 * 2D Cartesian point (x,y) to double (floating-point) precision
 */
struct Point2D {
public:
    double x, y;

    Point2D() : x(0), y(0) {}
    Point2D(const double& x, const double& y) : x(x), y(y) {}
    Point2D(const Point2D& p) : x(p.x), y(p.y) {}

    inline Point2D& operator=(const Point2D& p) {
        x = p.x; y = p.y;
        return *this;
    }

    inline Point2D operator+(const Point2D& p) const {
        return Point2D(x+p.x, y+p.y);
    }

    inline Point2D& operator+=(const Point2D& p) {
        x += p.x; y += p.y;
        return *this;
    }

    inline Point2D operator-(const Point2D& p) const {
        return Point2D(x-p.x, y-p.y);
    }

    inline Point2D& operator-=(const Point2D& p) {
        x -= p.x; y -= p.y;
        return *this;
    }

    inline Point2D operator*(const double& scale) const {
        return Point2D(x*scale, y*scale);
    }

    inline Point2D& operator*=(const double& scale) {
        x *= scale; y *= scale;
        return *this;
    }

    inline Point2D operator/(const double& scale) const {
        return Point2D(x/scale, y/scale);
    }

    inline Point2D& operator/=(const double& scale) {
        x /= scale; y /= scale;
        return *this;
    }

    inline bool operator==(const Point2D& p) const {
        return x == p.x && y == p.y;
    }

    inline bool operator!=(const Point2D& p) const {
        return x != p.x || y != p.y;
    }

    inline bool hasNAN() const {
        return std::isnan(x) || std::isnan(y);
    }

    inline bool isFinite() const {
        return !hasNAN() && x != INFINITY && y != INFINITY &&
                x != -INFINITY && y != -INFINITY;
    }

    inline bool withinError(const Point2D& p) const {
        return x >= p.x - CROSBOT_MATH_ERROR && x <= p.x + CROSBOT_MATH_ERROR &&
                y >= p.y - CROSBOT_MATH_ERROR && y <= p.y + CROSBOT_MATH_ERROR;
    }

    inline double distanceTo(const Point2D& p) const {
        double dx = x - p.x, dy = y - p.y;
        return sqrt(dx*dx+dy*dy);
    }

    // Gives the angle between two points interpreted as vectors
    inline double angleTo(const Point2D& other) {
        Point2D thisNorm = normalise();
        Point2D otherNorm = other.normalise();
        return acos(thisNorm.dot(otherNorm));
    }

    inline double length() const {
        return sqrt(x*x+y*y);
    }

    inline Point2D normalise() const {
        double len = length();
        return Point2D(x / len, y / len);
    }

    inline double dot(const Point2D& p) const {
        return x*p.x + y*p.y;
    }

    inline std::string toString() const;

    inline static Point2D parse(std::string str) {
        double x, y;
        if (sscanf(str.c_str(), "(%lf,%lf)", &x,&y) == 2) {
        } else if (sscanf(str.c_str(), "(%lf %lf)", &x,&y) == 2) {
        } else if (sscanf(str.c_str(), "(%lf, %lf)", &x,&y) == 2) {
        } else if (sscanf(str.c_str(), "( %lf %lf )", &x,&y) == 2) {
        } else if (sscanf(str.c_str(), "( %lf, %lf )", &x,&y) == 2) {
        } else if (sscanf(str.c_str(), "( %lf , %lf )", &x,&y) == 2) {
        } else {
            return Point2D();
        }

        return Point2D(x,y);
    }
};
inline std::ostream& operator<<(std::ostream& os, const Point2D& p) {
    return os << "Point2D(" << p.x << ", " << p.y << ")";
}
inline std::string Point2D::toString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

/**
 * 3D Cartesian point (x,y,z) to double (floating-point) precision
 */
struct Point3D {
public:
    double x, y, z;

    Point3D() : x(0), y(0), z(0) {}
    Point3D(const double& x, const double& y, const double& z) : x(x), y(y), z(z) {}
    Point3D(const Point3D& p) : x(p.x), y(p.y), z(p.z) {}

    inline Point3D& operator=(const Point3D& p) {
        x = p.x; y = p.y; z = p.z;
        return *this;
    }

    inline Point3D operator+(const Point3D& p) const {
        return Point3D(x+p.x, y+p.y, z+p.z);
    }

    inline Point3D& operator+=(const Point3D& p) {
        x += p.x; y += p.y; z += p.z;
        return *this;
    }

    inline Point3D operator-(const Point3D& p) const {
        return Point3D(x-p.x, y-p.y, z-p.z);
    }

    inline Point3D& operator-=(const Point3D& p) {
        x -= p.x; y -= p.y; z -= p.z;
        return *this;
    }

    inline Point3D operator*(const double& scale) const {
        return Point3D(x*scale, y*scale, z*scale);
    }

    inline Point3D& operator*=(const double& scale) {
        x *= scale; y *= scale; z *= scale;
        return *this;
    }

    inline Point3D operator/(const double& scale) const {
        return Point3D(x/scale, y/scale, z/scale);
    }

    inline Point3D& operator/=(const double& scale) {
        x /= scale; y /= scale; z /= scale;
        return *this;
    }

    inline bool operator==(const Point3D& p) const {
        return x == p.x && y == p.y && z == p.z;
    }

    inline bool operator!=(const Point3D& p) const {
        return x != p.x || y != p.y || z != p.z;
    }

    inline bool hasNAN() const {
        return std::isnan(x) || std::isnan(y) || std::isnan(z);
    }

    inline bool isFinite() const {
        return !hasNAN() && x != INFINITY && y != INFINITY && z != INFINITY &&
                x != -INFINITY && y != -INFINITY && z != -INFINITY;
    }

    inline bool withinError(const Point3D& p) const {
        return x >= p.x - CROSBOT_MATH_ERROR && x <= p.x + CROSBOT_MATH_ERROR &&
                y >= p.y - CROSBOT_MATH_ERROR && y <= p.y + CROSBOT_MATH_ERROR &&
                z >= p.z - CROSBOT_MATH_ERROR && z <= p.z + CROSBOT_MATH_ERROR;
    }

    inline double distanceTo(const Point3D& p) const {
        double dx = x - p.x, dy = y - p.y, dz = z - p.z;
        return sqrt(dx*dx+dy*dy+dz*dz);
    }

    inline double length() const {
        return sqrt(x*x+y*y+z*z);
    }

    inline Point3D normalise() const {
        double len = length();
        return Point3D(x / len, y / len, z / len);
    }

    inline double dot(const Point3D& p) const {
        return x*p.x + y*p.y + z*p.z;
    }

    inline Point3D cross(const Point3D& p) const {
        return Point3D(y*p.z - p.y*z, z*p.x - p.z*x, x*p.y - p.x*y);
    }

    inline std::string toString() const;

    inline static Point3D parse(std::string str) {
        double x, y, z;
        if (sscanf(str.c_str(), "(%lf,%lf,%lf)", &x,&y,&z) == 3) {
        } else if (sscanf(str.c_str(), "(%lf %lf %lf)", &x,&y,&z) == 3) {
        } else if (sscanf(str.c_str(), "(%lf, %lf, %lf)", &x,&y,&z) == 3) {
        } else if (sscanf(str.c_str(), "( %lf %lf %lf )", &x,&y,&z) == 3) {
        } else if (sscanf(str.c_str(), "( %lf, %lf, %lf )", &x,&y,&z) == 3) {
        } else if (sscanf(str.c_str(), "( %lf , %lf , %lf )", &x,&y,&z) == 3) {
        } else {
            return Point3D();
        }

        return Point3D(x,y,z);
    }

#ifdef ROS_VERSION

    Point3D(const geometry_msgs::Point& p) : x(p.x), y(p.y), z(p.z) {}
    inline Point3D& operator=(const geometry_msgs::Point& p) {
        x = p.x; y = p.y; z = p.z;
        return *this;
    }

    Point3D(const geometry_msgs::PointStamped& p) : x(p.point.x), y(p.point.y), z(p.point.z) {}
    inline Point3D& operator=(const geometry_msgs::PointStamped& p) {
        x = p.point.x; y = p.point.y; z = p.point.z;
        return *this;
    }

    Point3D(const geometry_msgs::Point32& p) : x(p.x), y(p.y), z(p.z) {}
    inline Point3D& operator=(const geometry_msgs::Point32& p) {
        x = p.x; y = p.y; z = p.z;
        return *this;
    }

    Point3D(const geometry_msgs::Vector3& v) : x(v.x), y(v.y), z(v.z) {}
    inline Point3D& operator=(const geometry_msgs::Vector3& v) {
        x = v.x; y = v.y; z = v.z;
        return *this;
    }

    Point3D(const tf::Vector3& v) : x(v.x()), y(v.y()), z(v.z()) {}
    inline Point3D& operator=(const tf::Vector3& v) {
        x = v.x(); y = v.y(); z = v.z();
        return *this;
    }

    inline geometry_msgs::Point toROS() const {
        geometry_msgs::Point rval;
        rval.x = x; rval.y = y; rval.z = z;
        return rval;
    }

    inline geometry_msgs::Point32 toROS32() const {
        geometry_msgs::Point32 rval;
        rval.x = x; rval.y = y; rval.z = z;
        return rval;
    }

    inline tf::Vector3 toTF() const {
        return tf::Vector3(x,y,z);
    }
#endif
};

/**
 * Default point type is Point3D
 */
typedef Point3D Point;

inline std::ostream& operator<<(std::ostream& os, const Point& p) {
    return os << "Point(" << p.x << ", " << p.y << ", " << p.z << ")";
}
inline std::string Point3D::toString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

#ifdef ROS_VERSION
// XXX: Why does ROS not generate an equality operator?

inline bool operator==(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}
inline bool operator!=(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return !(p1 == p2);
}
inline bool operator==(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
    return q1.x == q2.x && q1.y == q2.y && q1.z == q2.z && q1.w == q2.w;
}
inline bool operator!=(const geometry_msgs::Quaternion& q1, const geometry_msgs::Quaternion& q2) {
    return !(q1 == q2);
}
inline bool operator==(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
    return p1.position == p2.position && p1.orientation == p2.orientation;
}
inline bool operator!=(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2) {
    return !(p1 == p2);
}

//inline geometry_msgs::Quaternion zeroQuaternion() {
//    geometry_msgs::Quaternion rval;
//    rval.x = rval.y = rval.z = 0;
//    rval.w = 1;
//    return rval;
//}
//inline geometry_msgs::Pose zeroPose() {
//    geometry_msgs::Pose rval;
//    rval.position.x = rval.position.y = rval.position.z = 0;
//    rval.orientation = zeroQuaternion();
//    return rval;
//}

/**
 * Tests that a value isn't corrupt.
 */
inline bool hasNAN(const geometry_msgs::Point& p) {
    return std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z);
}

inline bool hasNAN(const geometry_msgs::Quaternion& q) {
    return std::isnan(q.x) || std::isnan(q.y) || std::isnan(q.z) || std::isnan(q.w);
}

inline bool hasNAN(const geometry_msgs::Pose& p) {
    return hasNAN(p.position) || hasNAN(p.orientation);
}

/**
 * Assorted functions for converting to/from ROS types. And fixing stupid ROS incompleteness.
 */

//inline geometry_msgs::Point& operator=(geometry_msgs::Point& lhs, const casros::Point3D& rhs) {
//  lhs.x = rhs.x; lhs.y = rhs.y; lhs.z = rhs.z;
//  return lhs;
//}
//
//inline casros::Point3D& operator=(casros::Point3D& lhs, const geometry_msgs::Point& rhs) {
//  lhs.x = rhs.x; lhs.y = rhs.y; lhs.z = rhs.z;
//  return lhs;
//}

#endif // ROS_VERSION

/**
 * Coloured 3D Cartesian point combining crosbot::Point3D and crosbot::Colour
 */
struct ColouredPoint {
public:
    crosbot::Point3D point;
    Colour colour;

    ColouredPoint() {}
    ColouredPoint(Point p) : point(p) {}
    ColouredPoint(Point p, Colour c) : point(p), colour(c) {}
    ColouredPoint(double x, double y, double z) : point(x,y,z) {}
    ColouredPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b) : point(x,y,z), colour(r,g,b) {}
    ColouredPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b, uint8_t a) : point(x,y,z), colour(r,g,b,a) {}

    inline bool hasNAN() const { return point.hasNAN(); }

    inline std::string toString() const;

#ifdef ROS_VERSION

    ColouredPoint(const ColouredPointMsg& cp) : point(cp.p), colour(cp.c) {}

    inline ColouredPoint& operator=(const ColouredPointMsg& cp) {
        point = cp.p; colour = cp.c;
        return *this;
    }

    inline ColouredPointMsg toROS() const {
        ColouredPointMsg rval;

        rval.p.x = point.x;
        rval.p.y = point.y;
        rval.p.z = point.z;

        rval.c.r = colour.r;
        rval.c.g = colour.g;
        rval.c.b = colour.b;
        rval.c.a = colour.a;

        return rval;
    }

#endif
};
inline std::ostream& operator<<(std::ostream& os, const ColouredPoint& cp) {
    return os << "ColouredPoint(" << cp.point << ", " << cp.colour << ")";
}
inline std::string ColouredPoint::toString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

} // namespace crosbot



#ifdef ROS_VERSION

// TODO: Confirm that these sizes are the same as the ROS message serialization sizes
ROS_STATIC_ASSERT(sizeof(crosbot::Point3D) == 24);

namespace ros {
namespace message_traits {

template <> struct IsFixedSize<crosbot::Point3D> : public TrueType {};
template <> struct IsSimple<crosbot::Point3D> : public TrueType {};

template <>
struct MD5Sum<crosbot::Point3D> {
    static const char* value()
    {
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Point>::static_value1 == 0x4a842b65f413084dULL);
        ROS_STATIC_ASSERT(MD5Sum<geometry_msgs::Point>::static_value2 == 0xc2b10fb484ea7f17ULL);

        return MD5Sum<geometry_msgs::Point>::value();
    }

    static const char* value(const crosbot::Point3D& m)
    {
        return MD5Sum<geometry_msgs::Point>::value();
    }
};

template <>
struct DataType<crosbot::Point3D> {
    static const char* value() {
        return DataType<geometry_msgs::Point>::value();
    }

    static const char* value(const crosbot::Point3D& m) {
        return DataType<geometry_msgs::Point>::value();
    }
};

template <>
struct Definition<crosbot::Point3D> {
    static const char* value() {
        return Definition<geometry_msgs::Point>::value();
    }

    static const char* value(const crosbot::Point3D& m) {
        return Definition<geometry_msgs::Point>::value();
    }
};

} // namespace message_traits
} // namespace ros

#endif

#endif /* CROSBOT_GEOMETRY_POINTS_HPP_ */
