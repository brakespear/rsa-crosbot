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
   virtual void initialise(ros::NodeHandle &nh);

   /*
    * Start 3d graph slam
    */
   virtual void start() = 0;

   /*
    * Shutdown node
    */
   virtual void stop() = 0;

   /*
    * Add a new frame to the current local map
    */
   virtual void addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose) = 0;

   /*
    * Create a new local map
    */
   virtual void newLocalMap(LocalMapInfoPtr localMapInfo) = 0;

   /*
    * A list of new positions of local maps
    */
   virtual void haveOptimised(vector<LocalMapInfoPtr> newMapPositions) = 0;

   /*
    * Set the parameters of the registered depth camera
    */
   void setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty);

   GraphSlam3DNode *graphSlam3DNode;
protected:

   /*
    * Configuration parameters
    */

   //Size of a single cell in a local map in metres
   double CellSize;
   //Total width of the local map
   double LocalMapWidth;
   //Height of the local map
   double LocalMapHeight;


   //Set to true when the first local map message is received from graph slam
   bool finishedSetup;
   //Set to true when have received the depth camera params
   bool receivedCameraParams;

   /*
    * Camera params
    */
   double fx;
   double fy;
   double cx;
   double cy;
   double tx;
   double ty;

   ReadWriteLock masterLock;

};
   

#endif
