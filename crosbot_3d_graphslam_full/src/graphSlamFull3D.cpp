/*
 * graphslamFull3D.cpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_full/graphSlamFull3D.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlamFull3D::GraphSlamFull3D() {
}

void GraphSlamFull3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
}

void GraphSlamFull3D::start() {

}

void GraphSlamFull3D::stop() {

}

void GraphSlamFull3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

}

void GraphSlamFull3D::haveOptimised(vector<LocalMapInfoPtr> newMapPositions,
      vector<int> iNodes, vector<int> jNodes, bool wasFullLoop) {

}



