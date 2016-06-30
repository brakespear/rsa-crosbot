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

void Quaternion::setYPR(const double& yaw, const double& pitch, const double& roll) {
    tf::Quaternion q;
    q.setEulerZYX(yaw, pitch, roll);
    *this = q;
}

void Quaternion::getYPR(double& yaw, double& pitch, double& roll) const {
    tf::Matrix3x3 mat(toTF());
    mat.getEulerYPR(yaw, pitch, roll);
}

} // namespace crosbot
