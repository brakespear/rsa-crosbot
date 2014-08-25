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
   string icp_frame, base_frame;
   string kinect_sub;

   /*
    * Other config params
    */
   //Number of kinect points to skip
   int SkipPoints;

   /*
    * ROS connections
    */
   ros::Subscriber kinectSub;
   tf::TransformListener tfListener;


   PositionTrack3D& position_track_3d;

   /*
    * Callback for receiving a new frame from a 3d camera
    */
   void callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud);

};

#endif
