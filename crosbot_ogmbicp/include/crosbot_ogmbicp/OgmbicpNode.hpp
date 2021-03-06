/*
 * OgmbicpNode.hpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */


#ifndef OGMBICPNODE_HPP_
#define OGMBICPNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/String.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_map/localmap.hpp>

#include <crosbot_ogmbicp/Ogmbicp.hpp>
#include <crosbot_ogmbicp/GetRecentScans.h>

#define DEFAULT_ICPFRAME "/icp"
#define DEFAULT_BASEFRAME "/base_link"
#define DEFAULT_ODOMFRAME ""
#define DEFAULT_MAXWAIT4TRANSFORM 2.0

using namespace std;

class OgmbicpNode {
public:

   OgmbicpNode(Ogmbicp&);

   /*
    * Initialises the ogmbicp node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts down the ogmbicp node
    */
   void shutdown();

private:
  
   /*
    * ROS config params for position tracking
    */
   string icp_frame, base_frame, odom_frame;
   string scan_sub, orientation_sub, z_sub;
   string local_map_image_pub, local_map_pub;
   string recent_scans_srv;

   string reset_map_sub;


   /*
    * ROS connections
    */
   ros::Subscriber scanSubscriber;
   ros::Subscriber orientationSubscriber;
   ros::Subscriber zSub;
   ros::Subscriber resetMapSubscriber;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;
   ros::Publisher imagePub;
   ros::Publisher localMapPub;
   ros::ServiceServer recentScansServer;

   bool UseExternalZ;
   bool UseFloorHeight;
   bool UseStampedOrientation;

   Ogmbicp &pos_tracker;
   //Is it the initial scan?
   bool isInit;
   LocalMapPtr localMap;

   bool getRecentScans(crosbot_ogmbicp::GetRecentScans::Request& req,
         crosbot_ogmbicp::GetRecentScans::Response& res);

   /*
    * Main callback for position tracker. Processes a new scan
    */
   void callbackScan(const sensor_msgs::LaserScanConstPtr& lastestScan);

   /*
    * Callback for the orientation from the IMU
    */
   //void callbackOrientation(const geometry_msgs::Quaternion& quat);
   void callbackOrientation(const geometry_msgs::Quaternion& quat);
   void callbackOrientationStamped(const geometry_msgs::QuaternionStamped& quat);
   void callbackZ(const geometry_msgs::Vector3& vec);

   /*
    * Callback to reset the position tracker
    */
   void callbackResetMap(const std_msgs::String& name);

   /*
    * Gets a transform from a pose
    */
   geometry_msgs::TransformStamped getTransform(const Pose& pose, std::string childFrame, 
                           std::string frameName, ros::Time stamp = ros::Time::now());

};


#endif
