/*
 * graphSlam.hpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */

#ifndef GRAPHSLAM_HPP_
#define GRAPHSLAM_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot_map/localmap.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace crosbot;
using namespace std;

class GraphSlam {

public:
   /*
    * Config attributes for graph slam
    */
   //total width and length of the global map in metres
   double MapSize;
   //width and length of a map cell in metres
   double CellSize;
protected:
   //Time in us between transfers of global map images
   int ImgTransmitTime;

   /*
    * Other fields
    */
   /*
    * The time the last image was grabbed from slam
    */
   Time lastImgTime;

   /*
    * Gets the global map in the standard crosbot format
    */
   virtual void getGlobalMap(LocalMapPtr curMap) = 0;

public:
   /*
    * Initialise the position tracker
    */
   virtual void initialise(ros::NodeHandle &nh);

   /*
    * Start graph slam. After this is called, graph slam
    * is ready to take scans
    */
   virtual void start() = 0;

   /*
    * Stop graph slam
    */
   virtual void stop() = 0;

   /*
    * Called for processing the first scan
    */
   virtual void initialiseTrack(Pose icpPose, PointCloudPtr cloud) = 0;

   /*
    * Update graph slam with the lastest scan
    */
   virtual void updateTrack(Pose icpPose, PointCloudPtr cloud) = 0;

   /*
    * Grabs the current map
    */
   crosbot::ImagePtr drawMap(LocalMapPtr globalMap);

private:

};

#endif
