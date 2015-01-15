/*
 * positionTrack3DFullNode.hpp
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#ifndef POSITIONTRACKFULL3DNODE_HPP_
#define POSITIONTRACKFULL3DNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/PointCloud2.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>

#include <crosbot_3d_position_track_full/positionTrackFull3D.hpp>

using namespace std;
using namespace crosbot;

class PositionTrackFull3D;
class PositionTrackFull3DNode {
public:

   PositionTrackFull3DNode(PositionTrackFull3D&);

   /*
    * Initialise the node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts doen the node
    */
   void shutdown();

   /*
    * Publishes the local map information with points
    */
   void publishLocalMap(LocalMapInfoPtr localMap);

   /*
    * Publishes all the points in the local area
    */
   void publishAllPoints();


private:
   /*
    * ROS config params
    */
   string icp_frame, base_frame, icp_frame_z;
   string depth_sub, rgb_sub, camera_info_sub;
   string local_map_sub, local_map_pub;
   string map_points_pub;
   string z_pub;

   /*
    * Other config params
    */
   //Should the result be published as a transform on top of icp_frame
   //or as a message? (or both)
   bool PublishTransform;
   bool PublishMessage;
   //The size of the queue for rgb and depth images
   int QueueSize;
   //Should we subsribe to and publish local map information?
   //(should be true if using with graph slam, false otherwise)
   bool UseLocalMaps;
   //Should the current local map points be published?
   //This can slow the algorithm down if selected 
   bool OutputCurrentMap;
   //How long between outputting the local area (in usecs)
   double OutputCurrentMapRate;

   /*
    * ROS connections
    */
   ros::Publisher zPub;
   ros::Publisher localMapPub;
   ros::Publisher mapPointsPub;
   ros::Subscriber cameraInfoSub;
   ros::Subscriber localMapSub;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;

   typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;   
   message_filters::Subscriber<sensor_msgs::Image> *depthSub;
   message_filters::Subscriber<sensor_msgs::Image> *rgbSub;
   message_filters::Synchronizer<SyncPolicy> *sync;


   PositionTrackFull3D& position_track_3d;

   bool isInit;
   bool receivedCameraParams;
   ros::Time lastPublishedPoints;
   sensor_msgs::PointCloud2 allPointsMsg;

   void callbackKinect(const sensor_msgs::ImageConstPtr& depthImage, 
         const sensor_msgs::ImageConstPtr& rgbImage);

   void callbackCameraInfo(const sensor_msgs::CameraInfo& camInfo);

   void callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo);

   //Gets the transform message of a pose
   geometry_msgs::TransformStamped getTransform(const Pose& pose, 
         std::string childFrame, std::string frameName, ros::Time stamp);

};

#endif
