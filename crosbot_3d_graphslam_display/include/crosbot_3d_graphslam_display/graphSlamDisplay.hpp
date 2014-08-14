/*
 * graphSlamDisplay.hpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAMDISPLAY_HPP_
#define GRAPHSLAMDISPLAY_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot_graphslam/localMap.hpp>


using namespace std;
using namespace crosbot;

class GraphSlamDisplay {
public:
   GraphSlamDisplay();

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
    * Adds a new local map with its points
    */
   void addMap(LocalMapInfoPtr localMapPoints);

   /*
    * Returns the point cloud of entire map
    */
   PointCloud &getPointCloud();

private:

   /*
    * Configuration parameters
    */


   /*
    * Total world points
    */
   PointCloud points;
};
   

#endif
