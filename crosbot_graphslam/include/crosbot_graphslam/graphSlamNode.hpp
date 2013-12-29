/*
 * graphSlamNode.hpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */


#ifndef GRAPHSLAMNODE_HPP_
#define GRAPHSLAMNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_map/localmap.hpp>

#include <crosbot_graphslam/graphSlam.hpp>

#define DEFAULT_ICPFRAME "/icp"
#define DEFAULT_BASEFRAME "/base_link"
#define DEFAULT_MAXWAIT4TRANSFORM 2.0

using namespace std;

class GraphSlamNode {
public:

   GraphSlamNode(GraphSlam&);

   /*
    * Initialises the graph slam node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts down the graph slam node
    */
   void shutdown();

private:
  
   /*
    * ROS config params for graph slam
    */
   string icp_frame, base_frame, slam_frame;
   string scan_sub;
   string global_map_image_pub;

   /*
    * ROS connections
    */
   ros::Subscriber scanSubscriber;
   ros::Publisher imagePub;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;

   GraphSlam &graph_slam;
   LocalMapPtr globalMap;

   //Is it the first scan?
   bool isInit;

   /*
    * Main callback for graph slam. Processes a new scan
    */
   void callbackScan(const sensor_msgs::LaserScanConstPtr& lastestScan);

};


#endif
