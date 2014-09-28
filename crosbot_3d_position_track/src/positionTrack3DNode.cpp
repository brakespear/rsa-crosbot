/*
 * positionTrack3DNode.cpp
 *
 * Created on: 24/08/2014
 *    Author: adrianr
 *
 * Interface of kinect position tracking with ROS
 */

#include <crosbot_3d_position_track/positionTrack3DNode.hpp>
#include <crosbot_3d_graphslam/depthPoints.hpp>

using namespace std;
using namespace crosbot;

PositionTrack3DNode::PositionTrack3DNode(PositionTrack3D& positionTrack): 
   position_track_3d(positionTrack) {
      isInit = true;
}

void PositionTrack3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, "/icp");
   paramNH.param<std::string>("base_frame", base_frame, "/base_link");
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");

   paramNH.param<std::string>("icp_frame_z", icp_frame_z, "/icp_z");
   paramNH.param<std::string>("z_pub", z_pub, "z_values");

   paramNH.param<int>("SkipPoints", SkipPoints, 1);
   paramNH.param<bool>("PublishTransform", PublishTransform, true);
   paramNH.param<bool>("PublishMessage", PublishMessage, true);

   position_track_3d.initialise(nh);
   position_track_3d.start();

   kinectSub = nh.subscribe(kinect_sub, 1, &PositionTrack3DNode::callbackKinect, this);
   if (PublishMessage) {
      zPub = nh.advertise<geometry_msgs::Vector3>(z_pub, 1);
   }

}

void PositionTrack3DNode::shutdown() {
   kinectSub.shutdown();
   position_track_3d.stop();
}

void PositionTrack3DNode::callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
   Pose icpPose;
   Pose sensorPose;
   tf::StampedTransform kin2Base, base2Icp;

   try {
      tfListener.waitForTransform(base_frame, ptCloud->header.frame_id,
             ptCloud->header.stamp, ros::Duration(1, 0));
  		tfListener.lookupTransform(base_frame,
   				ptCloud->header.frame_id, ptCloud->header.stamp, kin2Base);
  		sensorPose = kin2Base;

      tfListener.waitForTransform(icp_frame, base_frame, ptCloud->header.stamp, ros::Duration(1,0));
      tfListener.lookupTransform(icp_frame, base_frame, ptCloud->header.stamp, base2Icp);
      icpPose = base2Icp;
   } catch (tf::TransformException& ex) {
 		fprintf(stderr, "position track 3d: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		ptCloud->header.stamp.sec, ptCloud->header.stamp.nsec);
   	return;
   }

   DepthPointsPtr depthPoints = new DepthPoints(ptCloud, SkipPoints, true);
   if (isInit) {
      position_track_3d.initialiseFrame(depthPoints, sensorPose, icpPose);
      isInit = false;
   } else {
      float floorHeight = NAN;
      Pose newIcpPose = position_track_3d.processFrame(depthPoints, sensorPose, icpPose, &floorHeight);

      if (PublishMessage) {
         tf::Vector3 pose = newIcpPose.position.toTF();
         pose[0] = 0;
         pose[1] = 0;
         if (!isnan(floorHeight)) {
            pose[0] = floorHeight;
         }
         geometry_msgs::Vector3 vecMsg;
         tf::vector3TFToMsg(pose, vecMsg);
         zPub.publish(vecMsg);
      }
      if (PublishTransform) {
         icpPose = newIcpPose.toTF() * icpPose.toTF().inverse();
         geometry_msgs::TransformStamped icpTs = getTransform(icpPose, icp_frame, icp_frame_z,
               ptCloud->header.stamp);
         tfPub.sendTransform(icpTs);
      }
   }
}

geometry_msgs::TransformStamped PositionTrack3DNode::getTransform(const Pose& pose, std::string childFrame, 
                           std::string frameName, ros::Time stamp) {
   geometry_msgs::TransformStamped ts;
   ts.header.frame_id = frameName;
   ts.header.stamp = stamp;
   ts.child_frame_id = childFrame;
   ts.transform.translation.x = pose.position.x;
   ts.transform.translation.y = pose.position.y;
   ts.transform.translation.z = pose.position.z;
   ts.transform.rotation.x = pose.orientation.x;
   ts.transform.rotation.y = pose.orientation.y;
   ts.transform.rotation.z = pose.orientation.z;
   ts.transform.rotation.w = pose.orientation.w;
   return ts;

}


