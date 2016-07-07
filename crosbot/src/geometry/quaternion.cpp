/*
 * quaternion.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */


#include <ros/ros.h>

#include <crosbot/geometry/quaternion.hpp>

#include <tf/tf.h>

namespace crosbot {

/**
 * Replaces deprecated function of the same name in tf::Quaternion.
 * Implementation copied from <tf/LinearMath/Quaternion.h> for ROS indigo.
 */
tf::Quaternion setEulerZYX(const tfScalar& yaw, const tfScalar& pitch, const tfScalar& roll)
{
    tf::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    return q;
}

void Quaternion::setYPR(const double& yaw, const double& pitch, const double& roll) {
    tf::Quaternion q;

    // Change to remove depreciated function
    //q.setEulerZYX(yaw, pitch, roll);

    q = setEulerZYX(yaw, pitch, roll);

    *this = q;
}

void Quaternion::getYPR(double& yaw, double& pitch, double& roll) const {
    tf::Matrix3x3 mat(toTF());
    mat.getEulerYPR(yaw, pitch, roll);
}

} // namespace crosbot
