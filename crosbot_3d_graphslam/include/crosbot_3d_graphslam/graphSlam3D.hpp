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

using namespace std;
using namespace crosbot;

class GraphSlam3D {
public:
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
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions);

private:

   bool finishedSetup;

};
   

#endif
