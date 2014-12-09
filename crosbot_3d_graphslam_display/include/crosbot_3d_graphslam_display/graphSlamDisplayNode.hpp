/*
 * graphSlamDisplayNode.hpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAMDISPLAYNODE_HPP_
#define GRAPHSLAMDISPLAYNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>
#include <crosbot_graphslam/LocalMapMsgList.h>

#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>

using namespace std;
using namespace crosbot;

class GraphSlamDisplayNode {
public:

   GraphSlamDisplayNode(GraphSlamDisplay&);

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
   string local_map_sub, optimise_map_sub, save_map_sub;
   string point_cloud_pub;

   /*
    * ROS connections
    */
   ros::Subscriber localMapSub;
   ros::Subscriber optimiseMapSub;
   ros::Subscriber saveMapSub;
   ros::Publisher pointCloudPub;

   
   GraphSlamDisplay& graph_slam_display;

   /* 
    * Callback for receiving information about the start of a new local map
    */
   void callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapPoints);

   /*
    * Callback for receiving information about local maps after they have been optimised
    */
   void callbackOptimiseMap(const crosbot_graphslam::LocalMapMsgListConstPtr& localMapMsgList);
   
   /*
    * Callback for saving the current global map to a file in vtk format
    */
   void callbackSaveMap(const std_msgs::String& filename);

   /*
    * Publishes the entire point cloud
    */
   void publishPointCloud();


};

#endif
