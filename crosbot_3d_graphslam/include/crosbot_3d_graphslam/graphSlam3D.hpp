/*
 * graphSlam3D.hpp
 *
 * Created on: 04/08/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAM3D_HPP_
#define GRAPHSLAM3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>

#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_3d_graphslam/local3DMap.hpp>
#include <crosbot_3d_graphslam/graphSlam3DNode.hpp>

using namespace std;
using namespace crosbot;

class GraphSlam3DNode;

class GraphSlam3D {
public:
   GraphSlam3D();

   /*
    * Initialise parameters
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Start 3d graph slam
    */
   void start();

   /*
    * Shutdown node
    */
   void stop();

   /*
    * Add a new frame to the current local map
    */
   void addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose);

   /*
    * Create a new local map
    */
   void newLocalMap(LocalMapInfoPtr localMapInfo);

   /*
    * A list of new positions of local maps
    */
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions);

   GraphSlam3DNode *graphSlam3DNode;
private:

   /*
    * Configuration parameters
    */

   //Size of a single cell in a local map in metres
   double CellSize;
   //Total width of the local map
   double LocalMapWidth;
   //Height of the local map
   double LocalMapHeight;
   //Minimum number of observations in a cell needed before considered occupied
   int ObsThresh;

   //Set to true when the first local map message is received from graph slam
   bool finishedSetup;

   VoxelGrid *localMap;
   vector<Local3DMap *> maps;
   int currentMap;

   ReadWriteLock masterLock;

};
   

#endif
