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
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("LocalMapWidth", LocalMapWidth, 10.0);
   paramNH.param<double>("LocalMapHeight", LocalMapHeight, 4.0);
   paramNH.param<int>("ObsThres", ObsThresh, 2);
}

void GraphSlam3D::start() {
   localMap = new VoxelGrid(LocalMapWidth, LocalMapHeight, CellSize);
}

void GraphSlam3D::stop() {
}

void GraphSlam3D::addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose) {

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

void GraphSlam3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

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

void GraphSlam3D::haveOptimised(vector<LocalMapInfoPtr> newMapPositions) {
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


