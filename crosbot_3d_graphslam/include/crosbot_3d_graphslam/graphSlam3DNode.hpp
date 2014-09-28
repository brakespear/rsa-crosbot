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
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/CameraInfo.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>
#include <crosbot_graphslam/LoopClose.h>

#include <crosbot_3d_graphslam/graphSlam3D.hpp>

using namespace std;
using namespace crosbot;

class GraphSlam3D;
class GraphSlam3DNode {
public:

   GraphSlam3DNode(GraphSlam3D&);

   /*
    * Initialise the node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts doen the node
    */
   void shutdown();
   
   /*
    * Publishes the full information about a local map
    */
   void publishLocalMap(LocalMapInfoPtr localMap);

   /*
    * Publishes the new positions of local maps
    */
   void publishOptimisedMapPositions(vector<LocalMapInfoPtr> &localMaps);

private:
   /*
    * ROS config params
    */
   string slam_frame, base_frame;
   string local_map_sub, kinect_sub, camera_info_sub;
   string local_map_pub, optimised_local_maps_pub;
   string optimise_map_srv;

   /*
    * ROS connections
    */
   ros::Subscriber kinectSub;
   ros::Subscriber localMapSub;
   ros::ServiceServer optimiseMapSrv;
   ros::Subscriber cameraInfoSub;
   ros::Publisher localMapPub;
   ros::Publisher optimisedLocalMapsPub;
   tf::TransformListener tfListener;

   /*
    * Other params
    */
   //Number of points to skip when processing kinect scan
   int SkipPoints;

   GraphSlam3D& graph_slam_3d;

   /*
    * Callback for receiving a new frame from a 3d camera
    */
   void callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud);

   /*
    * Callback for receiving information about the start of a new local map
    */
   void callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo);

   /*
    * Callback for receiving information about local maps after they have been optimised
    */
   bool callbackOptimiseMap(crosbot_graphslam::LoopClose::Request& req,
         crosbot_graphslam::LoopClose::Response& res);

   /*
    * Callback for receiving camera info information
    */
   void callbackCameraInfo(const sensor_msgs::CameraInfo& camInfo);


};

#endif
