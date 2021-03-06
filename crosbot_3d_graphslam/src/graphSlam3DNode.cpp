/*
 * graphSlam3DNode.cpp
 *
 * Created on: 01/08/2014
 *    Author: adrianr
 *
 * Interface of kinect graph slam with ROS
 */

#include <crosbot_3d_graphslam/graphSlam3DNode.hpp>
#include <crosbot_3d_graphslam/depthPoints.hpp>

#include<image_geometry/pinhole_camera_model.h>

using namespace std;
using namespace crosbot;

GraphSlam3DNode::GraphSlam3DNode(GraphSlam3D& graphSlam): 
   graph_slam_3d(graphSlam) {

   graph_slam_3d.graphSlam3DNode = this;
}

void GraphSlam3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   paramNH.param<std::string>("base_frame", base_frame, "/base_link");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMapInfo");
   paramNH.param<std::string>("optimise_map_srv", optimise_map_srv, "optimiseMapInfo");
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");
   paramNH.param<std::string>("camera_info_sub", camera_info_sub, "/camera/rgb/camera_info");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localMapPoints");
   paramNH.param<std::string>("optimsed_local_maps_pub", optimised_local_maps_pub, "optimised3DLocalMaps");

   paramNH.param<int>("SkipPoints", SkipPoints, 1);

   graph_slam_3d.initialise(nh);
   graph_slam_3d.start();

   kinectSub = nh.subscribe(kinect_sub, 1, &GraphSlam3DNode::callbackKinect, this);
   cameraInfoSub = nh.subscribe(camera_info_sub, 1, &GraphSlam3DNode::callbackCameraInfo, this);
   localMapSub = nh.subscribe(local_map_sub, 10, &GraphSlam3DNode::callbackLocalMap, this);
   //optimiseMapSub = nh.subscribe(optimise_map_sub, 10, &GraphSlam3DNode::callbackOptimiseMap, this);
   optimiseMapSrv = nh.advertiseService(optimise_map_srv, &GraphSlam3DNode::callbackOptimiseMap, this);
   localMapPub = nh.advertise<crosbot_graphslam::LocalMapMsg>(local_map_pub, 10);
   optimisedLocalMapsPub = nh.advertise<crosbot_graphslam::LocalMapMsgList>(optimised_local_maps_pub, 10);

}

void GraphSlam3DNode::shutdown() {
   kinectSub.shutdown();
   localMapSub.shutdown();
   //optimiseMapSub.shutdown();
   graph_slam_3d.stop();
}

void GraphSlam3DNode::callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {

   Pose slamPose;
   Pose sensorPose;
   tf::StampedTransform kin2Base, base2Slam;

   try {
      tfListener.waitForTransform(base_frame, ptCloud->header.frame_id,
             ptCloud->header.stamp, ros::Duration(1, 0));
  		tfListener.lookupTransform(base_frame,
   				ptCloud->header.frame_id, ptCloud->header.stamp, kin2Base);
  		sensorPose = kin2Base;

      tfListener.waitForTransform(slam_frame, base_frame, ptCloud->header.stamp, ros::Duration(1,0));
      tfListener.lookupTransform(slam_frame, base_frame, ptCloud->header.stamp, base2Slam);
      slamPose = base2Slam;
   } catch (tf::TransformException& ex) {
 		fprintf(stderr, "graph slam 3d: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		ptCloud->header.stamp.sec, ptCloud->header.stamp.nsec);
   	return;
   }

   DepthPointsPtr depthPoints = new DepthPoints(ptCloud, SkipPoints);
   graph_slam_3d.addFrame(depthPoints, sensorPose, slamPose);
}

void GraphSlam3DNode::callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo) {
   LocalMapInfoPtr localM = new LocalMapInfo(localMapInfo);
   graph_slam_3d.newLocalMap(localM);
}

//void GraphSlam3DNode::callbackOptimiseMap(const crosbot_graphslam::LocalMapMsgListConstPtr& localMapMsgList) {
bool GraphSlam3DNode::callbackOptimiseMap(crosbot_graphslam::LoopClose::Request& req,
      crosbot_graphslam::LoopClose::Response& res) {

   bool wasFullLoop = req.wasFullLoop;

   vector<LocalMapInfoPtr> newPos;
   for (int i = 0; i < req.localMaps.size(); i++) {
      newPos.push_back(new LocalMapInfo(req.localMaps[i]));
   }
   vector<int> iNodes;
   vector<int> jNodes;
   iNodes.resize(req.i.size());
   jNodes.resize(req.j.size());
   for (int i = 0; i < req.i.size(); i++) {
      iNodes[i] = req.i[i];
      jNodes[i] = req.j[i];
   }
   graph_slam_3d.haveOptimised(newPos, iNodes, jNodes, wasFullLoop);
   return true;
}

void GraphSlam3DNode::publishLocalMap(LocalMapInfoPtr localMap) {
   localMap->cloud->frameID = slam_frame;
   localMap->normals->frameID = slam_frame;
   localMapPub.publish(localMap->toROS());
}

void GraphSlam3DNode::publishOptimisedMapPositions(vector<LocalMapInfoPtr> &localMaps) {
   crosbot_graphslam::LocalMapMsgList list;
   list.localMaps.resize(localMaps.size());
   for (int i = 0; i < localMaps.size(); i++) {
      list.localMaps[i] = *(localMaps[i]->toROSsmall());
   }
   optimisedLocalMapsPub.publish(list);
}

void GraphSlam3DNode::callbackCameraInfo(const sensor_msgs::CameraInfo& camInfo) {
   image_geometry::PinholeCameraModel cameraModel;
   cameraModel.fromCameraInfo(camInfo);
   double fx = cameraModel.fx();
   double fy = cameraModel.fy();
   double cx = cameraModel.cx();
   double cy = cameraModel.cy();
   double tx = cameraModel.Tx();
   double ty = cameraModel.Ty();

   graph_slam_3d.setCameraParams(fx, fy, cx, cy, tx, ty);

   cameraInfoSub.shutdown();
}
