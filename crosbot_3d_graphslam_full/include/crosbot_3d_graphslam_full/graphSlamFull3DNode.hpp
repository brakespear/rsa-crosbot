/*
 * graphSlamFull3DNode.hpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#ifndef GRAPHSLAMFULL3DNODE_HPP_
#define GRAPHSLAMFULL3DNODE_HPP_

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>

#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_graphslam/LocalMapMsg.h>
#include <crosbot_graphslam/LocalMapMsgList.h>
#include <crosbot_graphslam/LoopClose.h>

#include <crosbot_3d_graphslam_full/graphSlamFull3D.hpp>

using namespace std;
using namespace crosbot;

class GraphSlamFull3D;
class GraphSlamFull3DNode {
public:

   GraphSlamFull3DNode(GraphSlamFull3D&);

   /*
    * Initialise the node
    */
   void initialise(ros::NodeHandle& nh);

   /*
    * Shuts doen the node
    */
   void shutdown();
   
   /*
    * Publishes the new positions of local maps
    */
   void publishOptimisedMapPositions(vector<LocalMapInfoPtr> &localMaps);

private:
   /*
    * ROS config params
    */
   //string slam_frame, base_frame;
   string local_map_sub;
   string optimised_local_maps_pub;
   string optimise_map_srv;

   /*
    * ROS connections
    */
   ros::Subscriber localMapSub;
   ros::ServiceServer optimiseMapSrv;
   ros::Publisher optimisedLocalMapsPub;
   //tf::TransformListener tfListener;

   /*
    * Other params
    */

   GraphSlamFull3D& graph_slam_full_3d;

   /*
    * Callback for receiving information about the end of a new local map
    */
   void callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo);

   /*
    * Callback for receiving information about local maps after they have been optimised
    */
   bool callbackOptimiseMap(crosbot_graphslam::LoopClose::Request& req,
         crosbot_graphslam::LoopClose::Response& res);

};

#endif
