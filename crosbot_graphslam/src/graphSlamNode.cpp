/*
 * GraphSlamNode.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * Interface of graph slam algorithm with ROS
 */

#include <crosbot_graphslam/graphSlamNode.hpp>

using namespace std;
using namespace crosbot;

GraphSlamNode::GraphSlamNode(GraphSlam &graphSlam): icp_frame(DEFAULT_ICPFRAME),
                            base_frame(DEFAULT_BASEFRAME),
                            graph_slam(graphSlam)
{
   isInit = false;
}

void GraphSlamNode::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, DEFAULT_ICPFRAME);
   paramNH.param<std::string>("base_frame", base_frame, DEFAULT_BASEFRAME);
   paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   paramNH.param<std::string>("scan_sub", scan_sub, "scan");
   paramNH.param<std::string>("snap_sub", snap_sub, "snaps");
   paramNH.param<std::string>("global_map_image_pub", global_map_image_pub, "globalImage");
   paramNH.param<std::string>("slam_history_pub", slam_history_pub, "slamHistory");
   paramNH.param<std::string>("snap_list_srv", snap_list_srv, "snaps_list");
   paramNH.param<std::string>("snap_update_srv", snap_update_srv, "snap_update");
   paramNH.param<std::string>("snap_get_srv", snap_get_srv, "snap_get");

   graph_slam.initialise(nh);
   graph_slam.start();

   scanSubscriber = nh.subscribe(scan_sub, 1, &GraphSlamNode::callbackScan, this);
   snapSub = nh.subscribe(snap_sub, 1, &GraphSlamNode::callbackSnaps, this);
   imagePub = nh.advertise<sensor_msgs::Image>(global_map_image_pub, 1);
   slamHistoryPub = nh.advertise<nav_msgs::Path>(slam_history_pub, 1);
   snapListServer = nh.advertiseService(snap_list_srv, &GraphSlamNode::getSnapsList, this);
   snapUpdateServer = nh.advertiseService(snap_update_srv, &GraphSlamNode::snapUpdate, this);
   snapGetServer = nh.advertiseService(snap_get_srv, &GraphSlamNode::snapGet, this);
}

void GraphSlamNode::shutdown() {
   scanSubscriber.shutdown();
   graph_slam.stop();

}

void GraphSlamNode::callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
   Pose sensorPose, icpPose;
  	tf::StampedTransform laser2Base, base2Icp;
  	try {
  		tfListener.waitForTransform(base_frame, latestScan->header.frame_id,
             latestScan->header.stamp, ros::Duration(1, 0));
  		tfListener.lookupTransform(base_frame,
   				latestScan->header.frame_id, latestScan->header.stamp, laser2Base);
  		sensorPose = laser2Base;

      tfListener.waitForTransform(base_frame, icp_frame, latestScan->header.stamp, ros::Duration(1,0));
      tfListener.lookupTransform(base_frame, icp_frame, latestScan->header.stamp, base2Icp);
      icpPose = base2Icp;

  	} catch (tf::TransformException& ex) {
 		fprintf(stderr, "graph slam: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		latestScan->header.stamp.sec, latestScan->header.stamp.nsec);
   	return;
   }
   PointCloudPtr cloud = new PointCloud(base_frame, PointCloud(latestScan, true), sensorPose);
   if (!isInit) {
      isInit = true;
      graph_slam.initialiseTrack(icpPose, cloud);
      uint32_t dim = (uint32_t) (graph_slam.DimGlobalOG);
      globalMap = new LocalMap(dim, dim, graph_slam.CellSize, slam_frame);
   } else {
      graph_slam.updateTrack(icpPose, cloud);
   }
   ImagePtr image = graph_slam.drawMap(globalMap, icpPose);
   if (image != NULL) {
      publishSlamHistory();
      graph_slam.addSlamTrack(image);
      //localMapPub.publish(globalMap->getGrid());
      imagePub.publish(image->toROS());
   }
   Pose slamPose = graph_slam.slamPose;
   Pose correction = icpPose.toTF().inverse() * slamPose.toTF();
   geometry_msgs::TransformStamped slamCor = getTransform(correction, icp_frame, slam_frame, base2Icp.stamp_);
   tfPub.sendTransform(slamCor);
}

void GraphSlamNode::callbackSnaps(const crosbot_map::SnapMsg& newSnapMsg) {
   SnapPtr newSnap = new Snap(newSnapMsg);
   graph_slam.addSnap(newSnap);
}

bool GraphSlamNode::getSnapsList(crosbot_map::ListSnaps::Request& req, 
         crosbot_map::ListSnaps::Response& res) {
   if (!graph_slam.finishedSetup) {
      return false;
   }
   vector<SnapPtr> list;
   graph_slam.getSnaps(list);
   res.snaps.resize(list.size());
   int i;
   for (i = 0; i < list.size(); i++) {
      res.snaps[i] = *list[i]->toROSsmall();
   }
   return true;
}

bool GraphSlamNode::snapUpdate(crosbot_map::ModifySnap::Request& req, 
         crosbot_map::ModifySnap::Response& res) {
   if (!graph_slam.finishedSetup) {
      return false;
   }
   return graph_slam.updateSnap(req.id.data, req.type.data, 
         req.description.data, req.status.data);

}

bool GraphSlamNode::snapGet(crosbot_map::GetSnap::Request& req, 
         crosbot_map::GetSnap::Response& res) {
   if (!graph_slam.finishedSetup) {
      return false;
   }
   SnapPtr snap = new Snap();
   if (!graph_slam.getSnap(req.id.data, req.type.data, snap)) {
      return false;
   }
   res.snap = *(snap->toROS());
   return true;
}

void GraphSlamNode::publishSlamHistory() {
   vector<geometry_msgs::PoseStamped>& history = graph_slam.getSlamHistory();
   nav_msgs::Path path;
   path.poses = history;
   path.header.frame_id = slam_frame;
   path.header.stamp = ros::Time::now();
   slamHistoryPub.publish(path);
}

geometry_msgs::TransformStamped GraphSlamNode::getTransform(const Pose& pose, std::string childFrame, 
                           std::string frameName, ros::Time stamp) {
   geometry_msgs::TransformStamped ts;
   ts.header.frame_id = frameName;
   ts.header.stamp = stamp;
   ts.child_frame_id = childFrame;
   ts.transform.translation.x = pose.position.x;
   ts.transform.translation.y = pose.position.y;
   ts.transform.translation.z = pose.position.z;
   ts.transform.rotation.x = pose.orientation.x;
   ts.transform.rotation.y = pose.orientation.y;
   ts.transform.rotation.z = pose.orientation.z;
   ts.transform.rotation.w = pose.orientation.w;
   return ts;

}

