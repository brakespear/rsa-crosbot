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

void GraphSlam3DNode::initialise(ros::Nodehandle& nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMap");
   paramNH.param<std::string>("optimise_map_sub", optimise_map_sub, "optimiseMap");
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localMapPub");


}

void GraphSlam3DNode::shutdown() {

}

