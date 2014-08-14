/*
 * graphSlamDisplay.cpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlamDisplay::GraphSlamDisplay() {
}

void GraphSlamDisplay::initialise(ros::NodeHandle &nh) {
   //ros::NodeHandle paramNH("~");
}

void GraphSlamDisplay::start() {
}

void GraphSlamDisplay::stop() {
}

void GraphSlamDisplay::addMap(LocalMapInfoPtr localMapPoints) {

}

PointCloud &GraphSlamDisplay::getPointCloud() {
   return points;
}

