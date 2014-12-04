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
#include <crosbot_map/localmap.hpp>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

using namespace crosbot;
using namespace std;

class Ogmbicp {

public:
   /*
    * Config attributes for ogmbicp
    */
   //total width and length of the local map in metres
   double MapSize;
   //width and length of a 2D local map cell in metres
   double CellSize;
   /*
    * z value
    */
   double zOffset;
   double floorHeight;
protected:
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
   //Speed at which points disappear from map. Larger value and they will stay longer
   double LifeRatio;
   //Initial height of the robot
   double InitHeight;

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
   //Weigh matches acccording to distance and observation count
   bool UseFactor;
   //Use simpler calculations for H and ignore movement in z
   bool UseSimpleH;
   //Maximum number of laser points stored in a cell
   int MaxObservations;
   //Minimum factor possible when UseFactor is true
   double MinFactor;
   //Number of matching grid cells examined in more detail during ICP
   int FullSearchSize;
   //Algorithm used for weighting nearby points in ICP
   int NearestAlgorithm;
   //Furthest away a match will be aligned
   double MaxAlignDistance;
   //The number of scans to add to the map before alignment is started
   int InitialScans;
   //Maximum move possible for a scan
   double MaxMoveXYZ;
   double MaxMoveTh;
   //Every MaxScanSkip scan is added to the map
   int MaxScanSkip;
   //Every AddSkipCount scan replaces laser points in relevant cell
   int AddSkipCount;
   //Maximum number of times scans can fail to align in a row until cells are reset
   int MaxFail;
   //Use previous move as a starting guess for the next move
   bool UsePriorMove;
   //Use odometry as a starting guess for the next move
   bool UseOdometry;
   //Time in us between transfers of local map images
   int ImgTransmitTime;
   //Number of seconds of history recent scans are stored for
   double ScanListTime;
   //Use the orientation from the IMU as the starting point for the optimisation
   //bool UseIMUOrientation;
   //Discard scans when pitch and roll exceed certain allowances
   bool DiscardScansOrientation;
   //Threshold of pitch and roll for discarding scans!s!=
   double DiscardThreshold;



   /*
    * Other fields
    */
   /*
    * Absolute pose of the laser
    */
   Pose laserPose;
   /*
    * The last odom pose
    */
   Pose oldOdom;
   /*
    * The last time an image was grabbed from the position tracker
    */
   Time lastImgTime;

   /*
    * List of recent scans from the laser aligned according to the position tracker
    */
   deque<PointCloudPtr> recentScans;

   /*
    * Checks to make sure orientation values from IMU are being received
    */
   //bool isOrientationValid;
   /*
    * Current yaw from IMU
    */
   //double imuYaw;
   /*
    * should the next scan be discarded
    */
   bool discardScan;


   /*
    * Transforms the displacement to robot relative movement
    */
   void transformToRobot(double &dx, double &dy, double &dz, double &dth);

   /*
    * Gets the current local map of the robot in the standard crosbot format
    */
   virtual void getLocalMap(LocalMapPtr curMap) = 0;

public:
   //Current position of the robot
   Pose curPose;
   //True if the code has finished initialising
   bool finishedSetup;

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
   virtual void initialiseTrack(Pose sensorPose, PointCloudPtr cloud, 
         Pose odomPose) = 0;

   /*
    * Update the position tracker with the lastest scan
    */
   virtual void updateTrack(Pose sensorPose, PointCloudPtr cloud,
         Pose odomPose) = 0;

   /*
    * Grabs the current local map
    */
   crosbot::ImagePtr drawMap(LocalMapPtr localMap);

   /*
    * Generates a list of the most recent laser scans aligned according to the icp frame
    */
   void getRecentScans(deque<PointCloudPtr> &recent);

   /*
    * Reads the orientation data from the imu
    */
   void processImuOrientation(const geometry_msgs::Quaternion& quat);

   /*
    * Guess the next ICP move from odom readings
    */
   void getOdomGuess(tf::Transform oldOdom, tf::Transform newOdom,
         tf::Transform icpCur, double &x, double &y, double &th);

private:
   //double yawOffset;

};

#endif
