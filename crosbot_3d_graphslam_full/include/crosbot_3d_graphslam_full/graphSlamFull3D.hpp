/*
 * graphSlamFull3D.hpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#ifndef GRAPHSLAMFULL3D_HPP_
#define GRAPHSLAMFULL3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot/thread.hpp>

#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_3d_graphslam_full/graphSlamFull3DNode.hpp>

using namespace std;
using namespace crosbot;

class GraphSlamFull3DNode;

class GraphSlamFull3D {
public:
   GraphSlamFull3D();

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
    * Create a new local map
    */
   void newLocalMap(LocalMapInfoPtr localMapInfo);

   /*
    * A list of new positions of local maps
    */
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions, vector<int> iNodes, vector<int> jNodes,
         bool wasFullLoop);

   GraphSlamFull3DNode *graphSlamFull3DNode;
protected:

   /*
    * Configuration parameters
    */

   //ReadWriteLock masterLock;

};
   

#endif
