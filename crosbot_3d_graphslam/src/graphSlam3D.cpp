/*
 * graphslam3D.cpp
 *
 * Created on: 04/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/graphSlam3D.hpp>

using namespace std;
using namespace crosbot;

GraphSlam3D::Graphslam3D() {
   finishedSetup = false;
}

void GraphSlam3D::initialise(ros::NodeHandle &nh) {
}

void GraphSlam3D::start() {
}

void GraphSlam3D::stop() {
}

void GraphSlam3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

   finishedSetup = true;
}

void GraphSlam3D::haveOptimised(vector<LocalMapInfoPtr> newMapPositions) {
   if (finishedSetup) {
   }
}


