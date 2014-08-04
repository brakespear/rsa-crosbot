/*
 * graphSlam3DNode.cpp
 *
 * Created on: 01/08/2014
 *    Author: adrianr
 *
 * Interface of kinect graph slam with ROS
 */

#include <crosbot_3d_graphslam/graphSlam3DNode.hpp>

using namespace std;
using namespace crosbot;

GraphSlam3DNode::GraphSlam3DNode(GraphSlam3D& graphSlam): 
   graph_slam_3d(graphSlam) {
}

void GraphSlam3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMap");
   paramNH.param<std::string>("optimise_map_sub", optimise_map_sub, "optimiseMap");
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localMapPub");

   graph_slam_3d.initialise(nh);
   graph_slam_3d.start();

   kinectSub = nh.subscribe(kinect_sub, 1, &GraphSlam3DNode::callbackKinect, this);
   localMapSub = nh.subscribe(local_map_sub, 1, &GraphSlam3DNode::callbackLocalMap, this);
   optimiseMapSub = nh.subscribe(optimise_map_sub, 1, &GraphSlam3DNode::callbackOptimiseMap, this);
   localMapPub = nh.advertise<crosbot_graphslam::LocalMapMsg>(local_map_pub, 10);

}

void GraphSlam3DNode::shutdown() {

   graph_slam_3d.stop();
}

void GraphSlam3DNode::callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
   cout << "jump" << endl;
}

void GraphSlam3DNode::callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo) {
   cout << "again" << endl;
}

void GraphSlam3DNode::callbackOptimiseMap(const crosbot_graphslam::LocalMapMsgListConstPtr& localMapMsgList) {
   cout << "yolo" << endl;
}

