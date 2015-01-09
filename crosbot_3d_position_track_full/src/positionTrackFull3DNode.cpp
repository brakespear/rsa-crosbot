/*
 * positionTrackFull3DNode.cpp
 *
 * Created on: 12/12/2014
 *    Author: adrianr
 *
 * Interface of kinect position tracking with ROS
 */

#include <crosbot_3d_position_track_full/positionTrackFull3DNode.hpp>
#include <image_geometry/pinhole_camera_model.h>

using namespace std;
using namespace crosbot;

PositionTrackFull3DNode::PositionTrackFull3DNode(PositionTrackFull3D& positionTrack): 
   position_track_3d(positionTrack) {
      isInit = true;
      receivedCameraParams = false;
}

void PositionTrackFull3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, "/icp");
   paramNH.param<std::string>("base_frame", base_frame, "/base_link");
   paramNH.param<std::string>("icp_frame_z", icp_frame_z, "/icp_z");
   paramNH.param<std::string>("depth_sub", depth_sub, "/camera/depth_registered/image_raw");
   paramNH.param<std::string>("rgb_sub", rgb_sub, "/camera/rgb/image_raw");
   paramNH.param<std::string>("camera_info_sub", camera_info_sub, "/camera/rgb/camera_info");
   paramNH.param<std::string>("z_pub", z_pub, "z_values");

   paramNH.param<bool>("PublishTransform", PublishTransform, true);
   paramNH.param<bool>("PublishMessage", PublishMessage, true);
   paramNH.param<int>("QueueSize", QueueSize, 5);

   position_track_3d.initialise(nh);
   position_track_3d.start();

   if (PublishMessage) {
      zPub = nh.advertise<geometry_msgs::Vector3>(z_pub, 1);
   }

   cameraInfoSub = nh.subscribe(camera_info_sub, 1, &PositionTrackFull3DNode::callbackCameraInfo, this);
   depthSub = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_sub, QueueSize);
   rgbSub = new message_filters::Subscriber<sensor_msgs::Image> (nh, rgb_sub, QueueSize);
   sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(QueueSize), *depthSub, *rgbSub);
   sync->registerCallback(boost::bind(&PositionTrackFull3DNode::callbackKinect, this, _1, _2));
}

void PositionTrackFull3DNode::shutdown() {
   position_track_3d.stop();
   delete sync;
   delete rgbSub;
   delete depthSub;
}

void PositionTrackFull3DNode::callbackKinect(const sensor_msgs::ImageConstPtr& depthImage, 
      const sensor_msgs::ImageConstPtr& rgbImage) {

   if (!receivedCameraParams) {
      return;
   }

   Pose icpPose;
   Pose sensorPose;
   tf::StampedTransform kin2Base, base2Icp;

   try {
      tfListener.waitForTransform(base_frame, rgbImage->header.frame_id,
             rgbImage->header.stamp, ros::Duration(1, 0));
  		tfListener.lookupTransform(base_frame,
   				rgbImage->header.frame_id, rgbImage->header.stamp, kin2Base);
  		sensorPose = kin2Base;

      tfListener.waitForTransform(icp_frame, base_frame, rgbImage->header.stamp, ros::Duration(1,0));
      tfListener.lookupTransform(icp_frame, base_frame, rgbImage->header.stamp, base2Icp);
      icpPose = base2Icp;
   } catch (tf::TransformException& ex) {
 		fprintf(stderr, "position track full 3d: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		rgbImage->header.stamp.sec, rgbImage->header.stamp.nsec);
   	return;
   }

   if (isInit) {
      position_track_3d.initialiseFrame(depthImage, rgbImage, sensorPose, icpPose);
      isInit = false;
   } else {
      float floorHeight = NAN;
      Pose newIcpPose = position_track_3d.processFrame(depthImage, rgbImage, sensorPose, 
            icpPose, &floorHeight);

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
               rgbImage->header.stamp);
         tfPub.sendTransform(icpTs);
      }
   }
}

void PositionTrackFull3DNode::callbackCameraInfo(const sensor_msgs::CameraInfo& camInfo) {
   image_geometry::PinholeCameraModel cameraModel;
   cameraModel.fromCameraInfo(camInfo);
   double fx = cameraModel.fx();
   double fy = cameraModel.fy();
   double cx = cameraModel.cx();
   double cy = cameraModel.cy();
   double tx = cameraModel.Tx();
   double ty = cameraModel.Ty();

   position_track_3d.setCameraParams(fx, fy, cx, cy, tx, ty);
   receivedCameraParams = true;
   cameraInfoSub.shutdown();
}

geometry_msgs::TransformStamped PositionTrackFull3DNode::getTransform(const Pose& pose, std::string childFrame, 
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


