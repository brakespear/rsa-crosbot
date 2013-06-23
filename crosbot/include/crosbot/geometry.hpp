/*
 * geometry.h
 *
 *  Created on: 28/11/2011
 *      Author: mmcgill
 */

#ifndef CROSBOT_GEOMETRY_H_
#define CROSBOT_GEOMETRY_H_

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

#include <math.h>
#include <cmath>
#include <stdint.h>

#include <crosbot/serialization.hpp>

namespace crosbot {

#define CROSBOT_MATH_ERROR   0.00001

/**
 * Macros for converting between degrees and radians
 */
#define DEG2RAD(A)      ((A)*M_PI/180)
#define RAD2DEG(A)      ((A)*180/M_PI)

/**
 * Macro to normalise an angle to between Pi and -Pi.
 */
#define NORMALISE_ANGLE(_A_)    if (_A_ != INFINITY && _A_ != -INFINITY && _A_ != NAN) {       \
                                    while (_A_ > M_PIl) { _A_ -= 2* M_PIl; }                    \
                                    while (_A_ < -M_PIl) { _A_ += 2*M_PIl; }                    \
                                }

/**
 * Common frames of reference used throughout casros.
 */
#define FRAME_SENSOR        0
#define FRAME_ROBOT         1
#define FRAME_WORLD         2

} // namespace crosbot

#ifdef ROS_VERSION

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Point32.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Transform.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>

#endif

#include <LinearMath/btQuaternion.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btTransform.h>

#include <cmath>
#include <ostream>

namespace crosbot {

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

    inline std::string toString() const {
        char cStr[256];
        sprintf(cStr, "(%.3lf, %.3lf)", x, y);
        return std::string(cStr);
    }

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
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Point2D);
} // namespace serialization

struct Point3D {
public:
	double x, y, z;

	Point3D() : x(0), y(0), z(0) {}
	Point3D(const double& x, const double& y, const double& z) : x(x), y(y), z(z) {}
	Point3D(const Point3D& p) : x(p.x), y(p.y), z(p.z) {}
	Point3D(const btVector3& v) : x(v.x()), y(v.y()), z(v.z()) {}

	inline Point3D& operator=(const Point3D& p) {
		x = p.x; y = p.y; z = p.z;
		return *this;
	}

	inline Point3D& operator=(const btVector3& v) {
		x = v.x(); y = v.y(); z = v.z();
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

    inline std::string toString() const {
        char cStr[256];
        sprintf(cStr, "(%.3lf, %.3lf, %.3lf)", x, y, z);
        return std::string(cStr);
    }

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

    inline btVector3 getVector() const {
    	return btVector3(x,y,z);
    }

#ifdef ROS_VERSION

	Point3D(const geometry_msgs::Point& p) : x(p.x), y(p.y), z(p.z) {}

	inline Point3D& operator=(const geometry_msgs::Point& p) {
		x = p.x; y = p.y; z = p.z;
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
#endif
};
typedef Point3D Point;
inline std::ostream& operator<<(std::ostream& os, const Point& p) {
	return os << "Point(" << p.x << ", " << p.y << ", " << p.z << ")";
}
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Point3D);
} // namespace serialization

struct Quaternion {
public:
	double x, y, z, w;

	Quaternion() : x(0), y(0), z(0), w(1) {}
	Quaternion(const double& x, const double& y, const double& z, const double& w) : x(x), y(y), z(z), w(w) {}
	Quaternion(const double& yaw, const double& pitch, const double& roll) {
		setYPR(yaw, pitch, roll);
	}
	Quaternion(const Quaternion& q) : x(q.x), y(q.y), z(q.z), w(q.w) {}
	Quaternion(const btQuaternion& q) : x(q.x()), y(q.y()), z(q.z()), w(q.w()) {}

	inline Quaternion& operator=(const Quaternion& q) {
		x = q.x; y = q.y; z = q.z; w = q.w;
		return *this;
	}

	inline Quaternion& operator=(const btQuaternion& q) {
		x = q.x(); y = q.y(); z = q.z(); w = q.w();
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

	inline void setYPR(const double& yaw, const double& pitch, const double& roll) {
		btQuaternion btq;
		btMatrix3x3 btm;
		btm.setEulerYPR(yaw, pitch, roll);
		btm.getRotation(btq);

		x = btq.x(); y = btq.y(); z = btq.z(); w = btq.w();
	}

	inline void getYPR(double& yaw, double& pitch, double& roll) const {
		btQuaternion btq(x, y, z, w);
		btMatrix3x3(btq).getEulerYPR(yaw, pitch, roll);
	}

	inline btQuaternion getBullet() const {
		return btQuaternion(x, y, z, w);
	}

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
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Quaternion);
} // namespace serialization

struct Pose2D {
public:
	Point2D position;
	double orientation;

	Pose2D() {}
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
		return position.hasNAN() || std::isnan(orientation);
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
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Pose2D);
} // namespace serialization

struct Pose3D {
public:
	Point3D position;
	Quaternion orientation;

	Pose3D() {}
	Pose3D(const Point3D& position, const Quaternion& orientation) :
		position(position), orientation(orientation) {}
	Pose3D(const Pose3D& p) : position(p.position), orientation(p.orientation) {}
	Pose3D(const btTransform& t) : position(t.getOrigin()), orientation(t.getRotation()) {}
	Pose3D(double x, double y, double z) : position(x,y,z) {}
	Pose3D(double x, double y, double z, double yaw, double pitch, double roll) :
		position(x,y,z), orientation(yaw, pitch, roll) {}

	inline Pose3D& operator=(const Pose3D& p) {
		position = p.position; orientation = p.orientation;
		return *this;
	}

	inline Pose3D& operator=(const btTransform& t) {
		position = t.getOrigin(); orientation = t.getRotation();
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


	inline void fromTransform(const btTransform& trans) {
	    position = trans.getOrigin();
	    orientation =  trans.getRotation();
	}

	inline void getTransform(btTransform& trans) const {
	    btVector3 pos(position.x, position.y, position.z);
	    trans.setOrigin(pos);
	    btQuaternion q(orientation.x, orientation.y, orientation.z, orientation.w);
	    trans.setRotation(q);
	}

	inline btTransform getTransform() const {
	    btVector3 pos(position.x, position.y, position.z);
	    btQuaternion q(orientation.x, orientation.y, orientation.z, orientation.w);

	    return btTransform(q, pos);
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
#endif
};
typedef Pose3D Pose;
inline std::ostream& operator<<(std::ostream& os, const Pose& p) {
	return os << "Pose(" << p.position << ", " << p.orientation << ")";
}
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Pose3D);
} // namespace serialization

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

/**
 * Assorted functions for converting to/from ROS types. And fixing stupid ROS incompleteness.
 */

//inline geometry_msgs::Point& operator=(geometry_msgs::Point& lhs, const casros::Point3D& rhs) {
//	lhs.x = rhs.x; lhs.y = rhs.y; lhs.z = rhs.z;
//	return lhs;
//}
//
//inline casros::Point3D& operator=(casros::Point3D& lhs, const geometry_msgs::Point& rhs) {
//	lhs.x = rhs.x; lhs.y = rhs.y; lhs.z = rhs.z;
//	return lhs;
//}

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

#endif // ROS_VERSION

struct Index2D {
public:
    int x, y;

    Index2D() : x(0), y(0) {}
    Index2D(int x, int y) : x(x), y(y) {}
    Index2D(const Index2D& idx) : x(idx.x), y(idx.y) {}

    inline double distanceTo(const Index2D& next) const {
        int dx = x - next.x, dy = y - next.y;
        return sqrt((double)(dx*dx + dy*dy));
    }

    inline bool operator==(const Index2D& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    inline bool operator!=(const Index2D& rhs) const {
        return x != rhs.x || y != rhs.y;
    }

    inline Index2D operator+(const Index2D& rhs) const {
        return Index2D(x + rhs.x, y + rhs.y);
    }

    inline Index2D operator-(const Index2D& rhs) const {
        return Index2D(x - rhs.x, y - rhs.y);
    }
};
inline std::ostream& operator<<(std::ostream& os, const Index2D& i) {
	return os << "Index2D(" << i.x << ", " << i.y << ")";
}
namespace serialization {
	CASROS_SIMPLE_SERIALIZER(Index2D);
} // namespace serialization

} // namespace crosbot


#ifdef ROS_VERSION

// TODO: Confirm that these sizes are the same as the ROS message serialization sizes
ROS_STATIC_ASSERT(sizeof(crosbot::Point3D) == 24);

ROS_STATIC_ASSERT(sizeof(crosbot::Quaternion) == 32);
ROS_STATIC_ASSERT(sizeof(crosbot::Pose3D) == 56);

namespace ros {

namespace message_traits {

template <> struct IsFixedSize<crosbot::Point3D> : public TrueType {};
template <> struct IsSimple<crosbot::Point3D> : public TrueType {};

template <> struct IsFixedSize<crosbot::Quaternion> : public TrueType {};
template <> struct IsSimple<crosbot::Quaternion> : public TrueType {};

template <> struct IsFixedSize<crosbot::Pose3D> : public TrueType {};
template <> struct IsSimple<crosbot::Pose3D> : public TrueType {};

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

namespace serialization {

template<>
struct Serializer<crosbot::Point3D> {
	template<typename Stream, typename T>
	inline static void allInOne(Stream& stream, T t) {
		stream.next(t.x);
		stream.next(t.y);
		stream.next(t.z);
	}
	ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<crosbot::Quaternion> {
	template<typename Stream, typename T>
	inline static void allInOne(Stream& stream, T t) {
		stream.next(t.x);
		stream.next(t.y);
		stream.next(t.z);
		stream.next(t.w);
	}
	ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<crosbot::Pose3D> {
	template<typename Stream, typename T>
	inline static void allInOne(Stream& stream, T t) {
		stream.next(t.position);
		stream.next(t.orientation);
	}
	ROS_DECLARE_ALLINONE_SERIALIZER;
};

} // namespace serialization

} // namespace ros

#endif /* ROS_VERSION */

#endif /* CROSBOT_GEOMETRY_H_ */
