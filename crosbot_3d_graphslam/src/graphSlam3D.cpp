/*
 * graphslam3D.cpp
 *
 * Created on: 04/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/graphSlam3D.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlam3D::GraphSlam3D() {
   finishedSetup = false;
}

void GraphSlam3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("CellSize", CellSize, 0.1);
   paramNH.param<double>("LocalMapWidth", LocalMapWidth, 10.0);
   paramNH.param<double>("LocalMapHeight", LocalMapHeight, 4.0);
}

void GraphSlam3D::start() {
   localMap = new VoxelGrid(LocalMapWidth, LocalMapHeight, CellSize);
}

void GraphSlam3D::stop() {
}

void GraphSlam3D::addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose) {
}

void GraphSlam3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

   {{ Lock lock(masterLock);

   if (finishedSetup) {
      PointCloudPtr cloud = localMap->extractPoints();
      LocalMapInfoPtr oldLocalMap = new LocalMapInfo(maps[currentMap]->getPose(), currentMap,
            cloud);
      graphSlam3DNode->publishLocalMap(oldLocalMap);
      localMap->clearGrid();
   }
   currentMap = localMapInfo->index;
   maps.push_back(new Local3DMap(localMapInfo->pose));

   }}
   finishedSetup = true;
}

void GraphSlam3D::haveOptimised(vector<LocalMapInfoPtr> newMapPositions) {
   if (finishedSetup) {
   }
}


