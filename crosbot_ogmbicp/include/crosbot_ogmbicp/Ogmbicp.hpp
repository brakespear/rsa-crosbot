/*
 * Ogmbicp.hpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */

#ifndef OGMBICP_HPP_
#define OGMBICP_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace crosbot;
using namespace std;

class Ogmbicp {
public:
   /*
    * Initialise the position tracker
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Start the position tracker. After this is called, the position
    * tracker is ready to take scans
    */
   void start();

   /*
    * Stop the position tracker
    */
   void stop();

   /*
    * Called for processing the first scan
    */
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);

   /*
    * Update the position tracker with the lastest scan
    */
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);

};

#endif
