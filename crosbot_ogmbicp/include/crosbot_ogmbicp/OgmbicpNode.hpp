/*
 * OgmbicpNode.hpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */


#ifndef OGMBICPNODE_HPP_
#define OGMBICPNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>

#include <crosbot_ogmbicp/Ogmbicp.hpp>

#define DEFAULT_ICPFRAME "/icp"
#define DEFAULT_BASEFRAME "/base_link"
#define DEFAULT_ODOMFRAME ""
#define DEFAULT_MAXWAIT4TRANSFORM 2.0

using namespace std;

class OgmbicpNode {
public:

   OgmbicpNode(Ogmbicp&);

   /*
    * Initialises the ogmbicp node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts down the ogmbicp node
    */
   void shutdown();

private:
  
   /*
    * ROS config params for position tracking
    */
   string icp_frame, base_frame, odom_frame;
   string scan_sub;


   /*
    * ROS connections
    */
   ros::Subscriber scanSubscriber;
   tf::TransformListener tfListener;

   Ogmbicp &pos_tracker;
   //Is it the initial scan?
   bool isInit;

   /*
    * Main callback for position tracker. Processes a new scan
    */
   void callbackScan(const sensor_msgs::LaserScanConstPtr& lastestScan);

};


#endif
