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
#include "sensor_msgs/PointCloud2.h"
#include <nav_msgs/Path.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_map/localmap.hpp>
#include <crosbot_map/tag.hpp>
#include <crosbot_map/ListSnaps.h>
#include <crosbot_map/GetSnap.h>
#include <crosbot_map/ModifySnap.h>

#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>
#include <crosbot_graphslam/LoopClose.h>

#include <crosbot_graphslam/factory.hpp>
#include <crosbot_graphslam/graphSlam.hpp>

#define DEFAULT_ICPFRAME "/icp"
#define DEFAULT_BASEFRAME "/base_link"
#define DEFAULT_MAXWAIT4TRANSFORM 2.0

using namespace std;
using namespace crosbot;

class GraphSlam;
class GraphSlamNode {
public:

   GraphSlamNode(FactoryGraphSlam& factoryGraphSlam);

   /*
    * Initialises the graph slam node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts down the graph slam node
    */
   void shutdown();

   /**
    * Causes this graphslam node to reset/re-initialise itself
    */
   void reset();

   /*
    * Publishes information about a local map
    */
   void publishLocalMapInfo(LocalMapInfo& info);

   /*
    * Publishes info about local maps that have changed position
    */
   void publishOptimiseLocalMapInfo(vector<LocalMapInfoPtr>& localMapInfo, vector<int> iCon, 
         vector<int> jCon, bool wasFullLoop);

private:
  
   /**
    * Namespaces for subscribing to topics, etc.
    */
   string nhNamespace;
   string paramNamespace;

   /**
    * Factory for generating a new GraphSlam entity
    */
   FactoryGraphSlam& factoryGraphSlam;

   /*
    * ROS config params for graph slam
    */
   string icp_frame, base_frame, slam_frame;
   string scan_sub, snap_sub, reset_sub;
   string global_map_image_pub, slam_history_pub, global_grid_pub, local_map_pub;
   string snap_list_srv, snap_update_srv, snap_get_srv, optimise_map_srv;

   /*
    * Publish an additional map slice of a higher height
    */
   bool IncludeHighMapSlice;
   double HighMapSliceHeight;

   /*
    * Publish information about local maps
    * - When created and when global map is optimised
    */
   bool PublishLocalMapInfo;

   /*
    * ROS connections
    */
   ros::Subscriber scanSubscriber;
   ros::Subscriber snapSubscriber;
   ros::Subscriber resetSubscriber;
   ros::Publisher imagePub;
   ros::Publisher slamHistoryPub;
   ros::Publisher localMapInfoPub;
   ros::ServiceClient optimiseMapService;
   vector<ros::Publisher> slamGridPubs;
   ros::ServiceServer snapListServer;
   ros::ServiceServer snapUpdateServer;
   ros::ServiceServer snapGetServer;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;

   //Kinect ros options
   bool useKinect;
   string kinect_sub;
   string world_pub;
   ros::Subscriber kinectSub;
   ros::Publisher worldMap;
   int globalMapPublishRate;
   int kinectCaptureRate;
   //Other kinect values
   ros::Time lastCaptured;
   ros::Time lastPublishedMap;
   sensor_msgs::PointCloud2 worldScan;

   //Debugging publisher
   ros::Publisher imageTestPub;
   LocalMapPtr testMap;

   /**
    * GraphSlam implementation
    */
   GraphSlam *graph_slam;

   vector<LocalMapPtr> globalMaps;

   //Is it the first scan?
   bool isInit;
   //Height of the map slices
   vector<double> mapSlices;

   /*
    * Main callback for graph slam. Processes a new scan
    */
   void callbackScan(const sensor_msgs::LaserScanConstPtr& lastestScan);
   /*
    * Callback for adding a new snap
    */
   void callbackSnaps(const crosbot_map::SnapMsg& newSnapMsg);
   /**
    * Callback for reset map notification
    */
   void callbackResetMap(std_msgs::StringConstPtr resetMsg);

   /*
    * Service to return a list of snaps
    */
   bool getSnapsList(crosbot_map::ListSnaps::Request& req, 
         crosbot_map::ListSnaps::Response& res);
   /*
    * Updates a snap with the given information
    */
   bool snapUpdate(crosbot_map::ModifySnap::Request& req, 
         crosbot_map::ModifySnap::Response& res);
   /*
    * Returns the requested snap
    */
   bool snapGet(crosbot_map::GetSnap::Request& req, 
         crosbot_map::GetSnap::Response& res);

   /*
    * Publishes a history of slam poses
    */
   void publishSlamHistory();

   /*
    * Turns a pose into a stamped transform
    */
   geometry_msgs::TransformStamped getTransform(const Pose& pose,
         std::string childFrame, std::string frameName, ros::Time stamp);

   /*
    * Kinect callback
    */
   void callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud);
   

};


#endif
