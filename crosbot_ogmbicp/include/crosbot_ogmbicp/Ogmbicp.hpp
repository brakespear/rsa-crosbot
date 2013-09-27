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
   //maximum number of iterations to align a scan
   int MaxIterations;
   //maximum errors in an iteration of icp permitted for it to stop
   double MaxErrorTh;
   double MaxErrorXY;
   double MaxErrorZ;
   //Minimum and maximum heights relative to the robot a laser point can have to be 
   //added into the mapand used for alignment
   double MinAddHeight;
   double MaxAddHeight;
   //Height of the floor. Points between Floor Height and MinAddHeight are considered
   //floor obstacles
   double FloorHeight;
   //Minimum distance a laser point can be from the robot to used for alignment and
   //added into the map
   double LaserMinDist;
   double LaserMaxDistance;
   //Should z values of laser points be ignored?
   bool IgnoreZValues;
   //Minimum number of laser points needed to match in each iteration of ogmbicp
   int MinGoodCount;

   //--- Probably no need to change these

   //Maximum distance two adjacent laser points can be away from each other to be included in the match
   double MaxSegLen;
   //Maximum distance of a laser point away from the robot for it to be used in alignment. < 0 means no limit
   double LaserMaxAlign;
   //Use a variable value for L
   bool UseVariableL;
   //Value used in distance weighting
   double AlignmentDFix;
   //L used in ogmbicp metric
   double LValue;
   ///Minimum number of laser points required to be in a column to it to be used in alignment
   int MinCellCount;
   //Wegith matches acccording to distance and observation count
   bool UseFactor;
   //Use simpler calculations for H and ignore movement in z
   bool UseSimpleH;
   //Maximum number of laser points stored in a cell
   int MaxObservations;
   //Minimum factor possible when UseFactor is true
   double MinFactor;

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
