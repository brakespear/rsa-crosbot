/*
 * graphSlam3DNode.hpp
 *
 * Created on: 01/08/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAM3DNODE_HPP_
#define GRAPHSLAM3DNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_braodcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_graphslam/LocalMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>
#include <crosbot_graphslam/LocalMapMshList.h>

#include <crosbot_3d_graphslam/graphSlam3D.hpp>

using namespace std;
using namespace crosbot;

class GraphSlam3DNode {
public:

   GraphSlam3DNode(GraphSlam3D&);

   /*
    * Initialise the node
    */
   void initialise(ros::Nodehandle& nh);

   /*
    * Shuts doen the node
    */
   void shutdown();

private:
   /*
    * ROS config params
    */
   string slam_frame;
   string local_map_sub, optimise_map_sub, kinect_sub;
   string local_map_pub;

   GraphSlam3D& graph_slam_3d;

};

#endif
