/*
 * graphSlamFull3DNode.cpp
 *
 * Created on: 16/01/2015
 *    Author: adrianr
 *
 * Interface of 3d graph slam with ROS
 */

#include <crosbot_3d_graphslam_full/graphSlamFull3DNode.hpp>

using namespace std;
using namespace crosbot;

GraphSlamFull3DNode::GraphSlamFull3DNode(GraphSlamFull3D& graphSlam): 
   graph_slam_full_3d(graphSlam) {

   graph_slam_full_3d.graphSlamFull3DNode = this;
}

void GraphSlamFull3DNode::initialise(ros::NodeHandle& nh) {
   ros::NodeHandle paramNH("~");
   //paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   //paramNH.param<std::string>("base_frame", base_frame, "/base_link");
   paramNH.param<std::string>("local_map_sub", local_map_sub, "localMapPoints");
   paramNH.param<std::string>("optimise_map_srv", optimise_map_srv, "optimiseMapInfo");
   paramNH.param<std::string>("optimsed_local_maps_pub", optimised_local_maps_pub, "optimised3DLocalMaps");

   graph_slam_full_3d.initialise(nh);
   graph_slam_full_3d.start();

   localMapSub = nh.subscribe(local_map_sub, 10, &GraphSlamFull3DNode::callbackLocalMap, this);
   optimiseMapSrv = nh.advertiseService(optimise_map_srv, &GraphSlamFull3DNode::callbackOptimiseMap, this);
   optimisedLocalMapsPub = nh.advertise<crosbot_graphslam::LocalMapMsgList>(optimised_local_maps_pub, 10);

}

void GraphSlamFull3DNode::shutdown() {
   localMapSub.shutdown();
   //optimiseMapSub.shutdown();
   graph_slam_full_3d.stop();
}

void GraphSlamFull3DNode::callbackLocalMap(const crosbot_graphslam::LocalMapMsgConstPtr& localMapInfo) {
   LocalMapInfoPtr localM = new LocalMapInfo(localMapInfo);
   graph_slam_full_3d.newLocalMap(localM);
}

bool GraphSlamFull3DNode::callbackOptimiseMap(crosbot_graphslam::LoopClose::Request& req,
      crosbot_graphslam::LoopClose::Response& res) {

   bool wasFullLoop = req.wasFullLoop;

   vector<LocalMapInfoPtr> newPos;
   for (int i = 0; i < req.localMaps.size(); i++) {
      newPos.push_back(new LocalMapInfo(req.localMaps[i]));
   }
   vector<int> iNodes;
   vector<int> jNodes;
   iNodes.resize(req.i.size());
   jNodes.resize(req.j.size());
   for (int i = 0; i < req.i.size(); i++) {
      iNodes[i] = req.i[i];
      jNodes[i] = req.j[i];
   }
   graph_slam_full_3d.haveOptimised(newPos, iNodes, jNodes, wasFullLoop);
   return true;
}

void GraphSlamFull3DNode::publishOptimisedMapPositions(vector<LocalMapInfoPtr> &localMaps) {
   crosbot_graphslam::LocalMapMsgList list;
   list.localMaps.resize(localMaps.size());
   for (int i = 0; i < localMaps.size(); i++) {
      list.localMaps[i] = *(localMaps[i]->toROSsmall());
   }
   optimisedLocalMapsPub.publish(list);
}

