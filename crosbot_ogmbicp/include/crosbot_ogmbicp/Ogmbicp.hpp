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
protected:
   /*
    * Config attributes for ogmbicp
    */
   //total width and length of the local map in metres
   double MapSize;
   //width and length of a 2D local map cell in metres
   double CellSize;
   //height of a 3D map cell in metres
   double CellHeight;
   //Maximum distancew two adjacent laser points can be away from each other to be included in the match
   double MaxSegLen;

   /*
    * Other fields
    */
   //Current position of the robot
   Pose curPose;

   /*
    * Absolute pose of the laser
    */
   Pose laserPose;

public:
   /*
    * Initialise the position tracker
    */
   virtual void initialise(ros::NodeHandle &nh);

   /*
    * Start the position tracker. After this is called, the position
    * tracker is ready to take scans
    */
   virtual void start() = 0;

   /*
    * Stop the position tracker
    */
   virtual void stop() = 0;

   /*
    * Called for processing the first scan
    */
   virtual void initialiseTrack(Pose sensorPose, PointCloudPtr cloud) = 0;

   /*
    * Update the position tracker with the lastest scan
    */
   virtual void updateTrack(Pose sensorPose, PointCloudPtr cloud) = 0;

};

#endif
