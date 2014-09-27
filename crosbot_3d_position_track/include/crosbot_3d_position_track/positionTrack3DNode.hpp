/*
 * positionTrack3DNode.hpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#ifndef POSITIONTRACK3DNODE_HPP_
#define POSITIONTRACK3DNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>

#include <crosbot_3d_position_track/positionTrack3D.hpp>

using namespace std;
using namespace crosbot;

class PositionTrack3DNode {
public:

   PositionTrack3DNode(PositionTrack3D&);

   /*
    * Initialise the node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts doen the node
    */
   void shutdown();

private:
   /*
    * ROS config params
    */
   string icp_frame, base_frame, icp_frame_z;
   string kinect_sub;
   string z_pub;

   /*
    * Other config params
    */
   //Number of kinect points to skip
   int SkipPoints;
   //Should the result be published as a transform on top of icp_frame
   //or as a message? (or both)
   bool PublishTransform;
   bool PublishMessage;

   /*
    * ROS connections
    */
   ros::Subscriber kinectSub;
   ros::Publisher zPub;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;


   PositionTrack3D& position_track_3d;

   bool isInit;

   /*
    * Callback for receiving a new frame from a 3d camera
    */
   void callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud);

   //Gets the transform message of a pose
   geometry_msgs::TransformStamped getTransform(const Pose& pose, 
         std::string childFrame, std::string frameName, ros::Time stamp);

};

#endif
