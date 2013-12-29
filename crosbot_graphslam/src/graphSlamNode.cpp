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
   paramNH.param<std::string>("global_map_image_pub", global_map_image_pub, "globalImage");

   graph_slam.initialise(nh);
   graph_slam.start();

   scanSubscriber = nh.subscribe(scan_sub, 1, &GraphSlamNode::callbackScan, this);

   imagePub = nh.advertise<sensor_msgs::Image>(global_map_image_pub, 1);
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
      uint32_t dim = (uint32_t) (graph_slam.MapSize / graph_slam.CellSize);
      globalMap = new LocalMap(dim, dim, graph_slam.CellSize, slam_frame);
   } else {
      graph_slam.updateTrack(icpPose, cloud);
   }
   ImagePtr image = graph_slam.drawMap(globalMap);
   if (image != NULL) {
      //localMapPub.publish(globalMap->getGrid());
      imagePub.publish(image->toROS());
   }
   //TODO: add the publishing of the slam pose here
}

