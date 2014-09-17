/*
 * graphSlam3DCPU.cpp
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/graphSlam3DCPU.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlam3DCPU::GraphSlam3DCPU() : GraphSlam3D() {
}

GraphSlam3DCPU::~GraphSlam3DCPU() {
}

void GraphSlam3DCPU::initialise(ros::NodeHandle &nh) {
   GraphSlam3D::initialise(nh);
   ros::NodeHandle paramNH("~");
   paramNH.param<int>("ObsThres", ObsThresh, 2);
}

void GraphSlam3DCPU::start() {
   localMap = new VoxelGrid(LocalMapWidth, LocalMapHeight, CellSize);
}

void GraphSlam3DCPU::stop() {
}

void GraphSlam3DCPU::addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose) {

   if (finishedSetup) {

      //TODO: do I want to have a distance filter?
      //depthPoints->filterDistance(minDist, maxDist);

      //Calculate offset of robot inside current local map
      tf::Transform offset =  (maps[currentMap]->getPose().toTF().inverse()) * slamPose.toTF();
      //Transform points from kinect frame of reference to be relative to local map
      depthPoints->transform(offset * sensorPose.toTF());

      {{ Lock lock(masterLock);

      localMap->addScan(depthPoints);

      }}
   }
}

void GraphSlam3DCPU::newLocalMap(LocalMapInfoPtr localMapInfo) {

   {{ Lock lock(masterLock);

   if (finishedSetup) {
      PointCloudPtr cloud = localMap->extractPoints(ObsThresh);
      LocalMapInfoPtr oldLocalMap = new LocalMapInfo(maps[currentMap]->getPose(), currentMap,
            cloud);
      graphSlam3DNode->publishLocalMap(oldLocalMap);
      localMap->clearGrid();
   }
   currentMap = localMapInfo->index;
   if (maps.size() == currentMap) {
      maps.push_back(new Local3DMap(localMapInfo->pose));
   } else {
      //At the moment graph slam creates new local maps in existing order, so just adding
      //a map onto the end should work
      cout << "Indexes don't match. ERROR" << endl;
   }

   }}
   finishedSetup = true;
}

void GraphSlam3DCPU::haveOptimised(vector<LocalMapInfoPtr> newMapPositions) {
   if (finishedSetup) {

      {{ Lock lock(masterLock); 

          //Changes to local maps to be sent to viewer
          vector<LocalMapInfoPtr> changes;

          int i;
          for (i = 0; i < newMapPositions.size(); i++) {
             int index = newMapPositions[i]->index;
             maps[index]->updatePose(newMapPositions[i]->pose);
             changes.push_back(new LocalMapInfo(maps[index]->getPose(), index));
          }
          //TODO: Do 3D loop closing here
          
         graphSlam3DNode->publishOptimisedMapPositions(changes);

      }}

   }
}

