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

//PositionTrackFull3DNode::PositionTrackFull3DNode(PositionTrackFull3D& positionTrack):
PositionTrackFull3DNode::PositionTrackFull3DNode():
   position_track_3d(NULL),
   depthOnly(false),
   sync(NULL),
   depthSub(NULL),
   rgbSub(NULL),
   UseLocalMaps(true),
   OutputCurrentMap(true),
   OutputCurrentMapRate(0),
   QueueSize(1),
   PublishMessage(true),
   PublishTransform(true),
   EnableDebugImage(true)
{
   reset();
}

void PositionTrackFull3DNode::reset() {
   isInit = true;
   receivedCameraParams = false;
   lastPublishedPoints = ros::Time::now();
}

void PositionTrackFull3DNode::resetMap() {
   ROS_INFO("***********************");
   ROS_INFO("PositionTrackFull3DNode :: Resetting Map");
   ROS_INFO("***********************");

   // Shutdown
   shutdown();

   // Reset
   reset();

   // Initialise
   initialise(nh3DConfig);
}

void PositionTrackFull3DNode::initialise(ros::NodeHandle& nh) {
   // Store node handle
   nh3DConfig = nh;

   // Load config
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, "/icp");
   paramNH.param<std::string>("base_frame", base_frame, "/base_link");
   paramNH.param<std::string>("icp_frame_z", icp_frame_z, "/icp_z");
   paramNH.param<std::string>("depth_sub", depth_sub, "/camera/depth_registered/image_raw");
   paramNH.param<std::string>("rgb_sub", rgb_sub, "/camera/rgb/image_raw");
   paramNH.param<std::string>("camera_info_sub", camera_info_sub, "/camera/rgb/camera_info");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMapInfo");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localMapPoints");
   paramNH.param<std::string>("map_points_pub", map_points_pub, "icpMapPoints");
   paramNH.param<std::string>("z_pub", z_pub, "z_values");
   paramNH.param<std::string>("force_map_pub", force_map_pub, "forceMapPub");

   paramNH.param<bool>("PublishTransform", PublishTransform, true);
   paramNH.param<bool>("PublishMessage", PublishMessage, true);
   paramNH.param<int>("QueueSize", QueueSize, 5);
   paramNH.param<bool>("UseLocalMaps", UseLocalMaps, true);
   paramNH.param<bool>("OutputCurrentMap", OutputCurrentMap, true);
   paramNH.param<double>("OutputCurrentMapRate", OutputCurrentMapRate, 5000000); 

   // Create 3D position tracker
   position_track_3d = new PositionTrackFull3D();
   position_track_3d->positionTrack3DNode = this;
   position_track_3d->UseLocalMaps = UseLocalMaps;
   position_track_3d->initialise(nh);
   position_track_3d->start();

   if (PublishMessage) {
      zPub = nh.advertise<geometry_msgs::Vector3>(z_pub, 1);
   }

   // Setup Kinect subscription
   paramNH.param<bool>("depth_only", depthOnly, false);
   cameraInfoSub = nh.subscribe(camera_info_sub, 1, &PositionTrackFull3DNode::callbackCameraInfo, this);
   if (depthOnly) {
      depthOnlySub = nh.subscribe(depth_sub, 1, &PositionTrackFull3DNode::callbackKinectDepthOnly, this);

      int ImageHeight = 0;
      int ImageWidth = 0;
      paramNH.param<int>("ImageHeight", ImageHeight, 640);
      paramNH.param<int>("ImageWidth", ImageWidth, 480);

      // Dummy Image
      //    See: http://docs.ros.org/api/sensor_msgs/html/msg/Image.html
      sensor_msgs::Image *dummyImage = new sensor_msgs::Image();
      dummyImage->height = ImageHeight;
      dummyImage->width = ImageWidth;
      dummyImage->is_bigendian = 0;
      dummyImage->encoding = "rgb8";
      dummyImage->step = 3*dummyImage->width;
      dummyImage->data.resize(dummyImage->step * dummyImage->height, 0);
      this->dummyImage = sensor_msgs::ImageConstPtr(dummyImage);
   } else {
       depthSub = new message_filters::Subscriber<sensor_msgs::Image>(nh, depth_sub, QueueSize);
       rgbSub = new message_filters::Subscriber<sensor_msgs::Image> (nh, rgb_sub, QueueSize);
       sync = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(QueueSize), *depthSub, *rgbSub);
       sync->registerCallback(boost::bind(&PositionTrackFull3DNode::callbackKinect, this, _1, _2));
   }

   if (UseLocalMaps) {
      localMapSub = nh.subscribe(local_map_sub, 10, &PositionTrackFull3DNode::callbackLocalMap, this);
      localMapPub = nh.advertise<crosbot_graphslam::LocalMapMsg>(local_map_pub, 10);
      forceMapSub = nh.subscribe(force_map_pub, 1, &PositionTrackFull3DNode::callbackForceMap, this);
   }
   if (OutputCurrentMap) {
      mapPointsPub = nh.advertise<sensor_msgs::PointCloud2>(map_points_pub, 1);
      allPointsMsg.header.frame_id = icp_frame_z;
      allPointsMsg.is_bigendian = false;
      allPointsMsg.is_dense = true;
      allPointsMsg.height = 1;
      allPointsMsg.fields.resize(4);
      allPointsMsg.fields[0].name = "x";
      allPointsMsg.fields[1].name = "y";
      allPointsMsg.fields[2].name = "z";
      allPointsMsg.fields[3].name = "rgb";
      for (int i = 0; i < 4; i++) {
         allPointsMsg.fields[i].offset = i * sizeof(float);
         allPointsMsg.fields[i].datatype = 7;
         allPointsMsg.fields[i].count = 1;
      }
      allPointsMsg.fields[3].offset = 16;
      allPointsMsg.point_step = sizeof(float) * 8;
   }

   // Debugging
   paramNH.param<bool>("EnableDebugImage", EnableDebugImage, true);
   if (EnableDebugImage) {
      outImagePub = nh.advertise<sensor_msgs::Image>("outImage", 1);
   }

   // Map Reset
   paramNH.param<std::string>("reset_map_sub", reset_map_sub, "resetMap");
   resetMapSub = nh.subscribe(reset_map_sub, 1, &PositionTrackFull3DNode::callbackResetMap, this);
}

void PositionTrackFull3DNode::shutdown() {
   // Shutdown subs
   cameraInfoSub.shutdown();
   if (depthOnly) {
      depthOnlySub.shutdown();
   } else {
      delete sync;
      delete rgbSub;
      delete depthSub;
   }
   if (UseLocalMaps) localMapSub.shutdown();
   if (UseLocalMaps) forceMapSub.shutdown();
   if (OutputCurrentMap) mapPointsPub.shutdown();
   resetMapSub.shutdown();

   // Shutdown publishers
   if (PublishMessage) zPub.shutdown();
   if (UseLocalMaps) localMapPub.shutdown();
   if (EnableDebugImage) outImagePub.shutdown();

   // Stop position tracker and destroy
   position_track_3d->stop();
   delete position_track_3d;
}

void PositionTrackFull3DNode::callbackKinectDepthOnly(const sensor_msgs::ImageConstPtr& depthImage) {
   try {
      // Set rgb dummy image header
      //    Need to do nasty casting stuff so the header can change...
      //boost::shared_ptr< ::sensor_msgs::Image >
      sensor_msgs::ImagePtr tmpDummyImage(boost::const_pointer_cast<sensor_msgs::Image>(dummyImage));
      tmpDummyImage->header.seq = depthImage->header.seq;
      tmpDummyImage->header.stamp = depthImage->header.stamp;
      tmpDummyImage->header.frame_id = depthImage->header.frame_id;

      callbackKinect(depthImage, dummyImage);
   } catch (...) {
      ROS_ERROR("PositionTrackFull3DNode :: Error processing with only depth image");
   }
}

void PositionTrackFull3DNode::callbackKinect(const sensor_msgs::ImageConstPtr& depthImage, 
      const sensor_msgs::ImageConstPtr& rgbImage) {

   curTimeStamp = depthImage->header.stamp;
   kinectFrame = rgbImage->header.frame_id;

   if (!receivedCameraParams) {
      ROS_WARN("Postrack3d: camera params not yet received");
      return;
   }
   Pose icpPose;
   Pose sensorPose;
   tf::StampedTransform kin2Base, base2Icp;

   try {
      tfListener.waitForTransform(base_frame, depthImage->header.frame_id,
             depthImage->header.stamp, ros::Duration(1, 0));
        tfListener.lookupTransform(base_frame,
               depthImage->header.frame_id, depthImage->header.stamp, kin2Base);
        sensorPose = kin2Base;

      tfListener.waitForTransform(icp_frame, base_frame, depthImage->header.stamp, ros::Duration(1,0));
      tfListener.lookupTransform(icp_frame, base_frame, depthImage->header.stamp, base2Icp);
      icpPose = base2Icp;
   } catch (tf::TransformException& ex) {
       fprintf(stderr, "position track full 3d: Error getting transform. (%s) (%d.%d)\n", ex.what(),
         depthImage->header.stamp.sec, depthImage->header.stamp.nsec);
       return;
   }
   /*cout << "Success!!!!" << endl;
   double yy,pp,rr;
   sensorPose.getYPR(yy,pp,rr);
   cout << sensorPose.position.x << " " << sensorPose.position.y << " " << sensorPose.position.z << " " <<
      yy << " " << pp << " " << rr << endl;

   sensorPose.position.x = 0.1;
   sensorPose.position.y = -0.04;
   sensorPose.position.z = 0.3;
   double y = -1.57;
   double p = 0;
   double r = -1.57;
   sensorPose.setYPR(y,p,r);*/

   bool outputMapPoints = false;
   if (OutputCurrentMap) {
      ros::Time currentTime = ros::Time::now();
      if ((currentTime - lastPublishedPoints).toSec() > OutputCurrentMapRate / 1000000.0) {
         outputMapPoints = true;
         lastPublishedPoints = currentTime;
      }
   }

   if (isInit) {
      position_track_3d->initialiseFrame(depthImage, rgbImage, sensorPose, icpPose);
      isInit = false;
   } else {
      float floorHeight = NAN;
      Pose newIcpPose = position_track_3d->processFrame(depthImage, rgbImage, sensorPose,
            icpPose, &floorHeight, outputMapPoints, allPointsMsg.data);

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

   position_track_3d->setCameraParams(fx, fy, cx, cy, tx, ty);
   receivedCameraParams = true;
   cameraInfoSub.shutdown();
}

void PositionTrackFull3DNode::callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo) {
   LocalMapInfoPtr localM = new LocalMapInfo(localMapInfo);
   position_track_3d->newLocalMap(localM);
}

void PositionTrackFull3DNode::publishLocalMap(LocalMapInfoPtr localMap) {
   if (UseLocalMaps) { //Shouldn't be needed, but just in case
      localMap->cloud->frameID = base_frame; //TODO: what frame should these be in?
      localMap->normals->frameID = base_frame;
      localMapPub.publish(localMap->toROS());
   }
}

void PositionTrackFull3DNode::publishAllPoints() {
   ROS_INFO("********publishing points***************");
   allPointsMsg.header.stamp = ros::Time::now();
   allPointsMsg.row_step = allPointsMsg.data.size();
   allPointsMsg.width = allPointsMsg.row_step / allPointsMsg.point_step;
   mapPointsPub.publish(allPointsMsg);
}

void PositionTrackFull3DNode::callbackForceMap(const std_msgs::String& ignore) {
   position_track_3d->forceMapPub();
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

void PositionTrackFull3DNode::callbackResetMap(const std_msgs::String& name) {
   if (name.data == "reset_map") {
      resetMap();
   }
}

bool PositionTrackFull3DNode::debugImageEnabled() {
   return EnableDebugImage;
}

void PositionTrackFull3DNode::outputImage(vector<uint8_t>& data) {
   sensor_msgs::Image im;
   im.header.stamp = curTimeStamp;
   im.header.frame_id = kinectFrame;
   im.height = 480;
   im.width = 640;
   //im.height = 120;
   //im.width = 160;
   im.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
   im.is_bigendian = 1;
   im.step = sizeof(float) * 640;
   //im.step = sizeof(float) * 160;
   im.data = data;
   outImagePub.publish(im);
}


