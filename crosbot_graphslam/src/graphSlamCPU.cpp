/*
 * graphSlamCPU.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * CPU version of graph slam
 */

#include <newmat/newmat.h>
#include <crosbot_graphslam/graphSlamCPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

using namespace NEWMAT;

GraphSlamCPU::GraphSlamCPU() {
   numIterations = 0;
   finishedSetup = false;
   currentLocalMap = 0;
}

void GraphSlamCPU::initialise(ros::NodeHandle &nh) {

   GraphSlam::initialise(nh);

   ros::NodeHandle paramNH("~");
}

void GraphSlamCPU::start() {
}

void GraphSlamCPU::stop() {
}

void GraphSlamCPU::initialiseTrack(Pose icpPose, PointCloudPtr cloud) {
}

void GraphSlamCPU::updateTrack(Pose icpPose, PointCloudPtr cloud) {
   cout << "The icp pose from slam is: " << icpPose << endl;
   finishedSetup = true;
}

GraphSlamCPU::~GraphSlamCPU() {
}

void GraphSlamCPU::getGlobalMap(LocalMapPtr curMap) {
}

void GraphSlamCPU::getGlobalMapPosition(int mapIndex, double& gx, double& gy,
      double& gth) {
}
