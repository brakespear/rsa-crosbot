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
#include <nav_msgs/Path.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_map/localmap.hpp>
#include <crosbot_map/tag.hpp>
#include <crosbot_map/ListSnaps.h>
#include <crosbot_map/GetSnap.h>
#include <crosbot_map/ModifySnap.h>

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
   string scan_sub, snap_sub;
   string global_map_image_pub, slam_history_pub, global_grid_pub;
   string snap_list_srv, snap_update_srv, snap_get_srv;

   /*
    * Publish an additional map slice of a higher height
    */
   bool IncludeHighMapSlice;
   double HighMapSliceHeight;

   /*
    * ROS connections
    */
   ros::Subscriber scanSubscriber;
   ros::Subscriber snapSub;
   ros::Publisher imagePub;
   ros::Publisher slamHistoryPub;
   vector<ros::Publisher> slamGridPubs;
   ros::ServiceServer snapListServer;
   ros::ServiceServer snapUpdateServer;
   ros::ServiceServer snapGetServer;
   tf::TransformListener tfListener;
   tf::TransformBroadcaster tfPub;

   //Debugging publisher
   ros::Publisher imageTestPub;
   LocalMapPtr testMap;

   GraphSlam &graph_slam;
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

};


#endif
