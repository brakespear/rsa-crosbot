/*
 * positionTrack3DNode.cpp
 *
 * Created on: 24/08/2014
 *    Author: adrianr
 *
 * Interface of kinect position tracking with ROS
 */

#include <crosbot_3d_position_track/positionTrack3DNode.hpp>

using namespace std;
using namespace crosbot;

PositionTrack3DNode::PositionTrack3DNode(PositionTrack3D& positionTrack): 
   position_track_3d(positionTrack) {
}

void PositionTrack3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, "/icp");
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");

   position_track_3d.initialise(nh);
   position_track_3d.start();

   kinectSub = nh.subscribe(kinect_sub, 1, &PositionTrack3DNode::callbackKinect, this);

}

void PositionTrack3DNode::shutdown() {
   kinectSub.shutdown();
   position_track_3d.stop();
}

void PositionTrack3DNode::callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
   //cout << "jump" << endl;
}

