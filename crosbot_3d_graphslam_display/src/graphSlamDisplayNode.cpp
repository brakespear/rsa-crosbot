/*
 * graphSlamDisplayNode.cpp
 *
 * Created on: 13/08/2014
 *    Author: adrianr
 *
 * Interface of kinect graph slam display with ROS
 */

#include <crosbot_3d_graphslam_display/graphSlamDisplayNode.hpp>

using namespace std;
using namespace crosbot;

GraphSlamDisplayNode::GraphSlamDisplayNode(GraphSlamDisplay& graphSlamDisplay): 
   graph_slam_display(graphSlamDisplay) {

}

void GraphSlamDisplayNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMapPoints");
   //Subsribe to new optimised map positions given by 3D slam
   paramNH.param<std::string>("optimise_map_sub", optimise_map_sub, "optimised3DLocalMaps");
   paramNH.param<std::string>("save_map_sub", save_map_sub, "saveMap");
   paramNH.param<std::string>("save_pose_sub", save_pose_sub, "savePose");
   paramNH.param<std::string>("point_cloud_pub", point_cloud_pub, "pointCloudPub");

   graph_slam_display.initialise(nh);

   localMapSub = nh.subscribe(local_map_sub, 20, &GraphSlamDisplayNode::callbackLocalMap, this);
   optimiseMapSub = nh.subscribe(optimise_map_sub, 20, &GraphSlamDisplayNode::callbackOptimiseMap, this);
   saveMapSub = nh.subscribe(save_map_sub, 1, &GraphSlamDisplayNode::callbackSaveMap, this);
   savePoseSub = nh.subscribe(save_pose_sub, 1, &GraphSlamDisplayNode::callbackSavePose, this);

   if (graph_slam_display.PublishPointCloud) {
      pointCloudPub = nh.advertise<sensor_msgs::PointCloud>(point_cloud_pub, 3);
   }
}

void GraphSlamDisplayNode::shutdown() {
   localMapSub.shutdown();
   optimiseMapSub.shutdown();
   saveMapSub.shutdown();
   savePoseSub.shutdown();
   graph_slam_display.stop();
}

void GraphSlamDisplayNode::callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapPoints) {
   LocalMapInfoPtr localM = new LocalMapInfo(localMapPoints);
   graph_slam_display.addMap(localM);
   publishPointCloud();
}

void GraphSlamDisplayNode::callbackOptimiseMap(const crosbot_graphslam::LocalMapMsgListConstPtr& localMapMsgList) {
   vector<LocalMapInfoPtr> newPos;
   for (int i = 0; i < localMapMsgList->localMaps.size(); i++) {
      newPos.push_back(new LocalMapInfo(localMapMsgList->localMaps[i]));
   }
   graph_slam_display.correctMap(newPos);
}

void GraphSlamDisplayNode::callbackSaveMap(const std_msgs::String& filename) {
   graph_slam_display.outputMapToFile(filename.data);
}

void GraphSlamDisplayNode::callbackSavePose(const std_msgs::String& filename) {
   graph_slam_display.outputPoseToFile(filename.data);
}

void GraphSlamDisplayNode::publishPointCloud() {
   if (graph_slam_display.PublishPointCloud) {
      PointCloud &pc = graph_slam_display.getPointCloud();
      pointCloudPub.publish(pc.toROS1());
   }
}

