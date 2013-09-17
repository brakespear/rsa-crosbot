/*
 * OgmbicpNode.cpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/OgmbicpNode.hpp>

using namespace std;
using namespace crosbot;

OgmbicpNode::OgmbicpNode(Ogmbicp &posTracker): icp_frame(DEFAULT_ICPFRAME),
                            base_frame(DEFAULT_BASEFRAME),
                            odom_frame(DEFAULT_ODOMFRAME),
                            pos_tracker(posTracker)
{
   pos_tracker = posTracker;
   isInit = false;
}

void OgmbicpNode::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, DEFAULT_ICPFRAME);
   paramNH.param<std::string>("base_frame", base_frame, DEFAULT_BASEFRAME);
   paramNH.param<std::string>("odom_frame", odom_frame, DEFAULT_ODOMFRAME);
   paramNH.param<std::string>("scan_sub", scan_sub, "scan");

   pos_tracker.initialise(nh);
   pos_tracker.start();

   scanSubscriber = nh.subscribe(scan_sub, 1, &OgmbicpNode::callbackScan, this);
}

void OgmbicpNode::shutdown() {
   scanSubscriber.shutdown();
   pos_tracker.stop();

}

void OgmbicpNode::callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
   Pose odomPose, sensorPose;
  	tf::StampedTransform laser2Base, base2Odom;
  	bool haveOdometry = odom_frame != "";
  	try {
  		tfListener.waitForTransform(base_frame, latestScan->header.frame_id,
             latestScan->header.stamp, ros::Duration(1, 0));
  		tfListener.lookupTransform(base_frame,
   				latestScan->header.frame_id, latestScan->header.stamp, laser2Base);
  		sensorPose = laser2Base;

  		if (haveOdometry) {
     		tfListener.waitForTransform(odom_frame, base_frame, latestScan->header.stamp, ros::Duration(1, 0));
     		tfListener.lookupTransform(odom_frame, base_frame,
     				latestScan->header.stamp, base2Odom);
     		odomPose = base2Odom;
 		}
  	} catch (tf::TransformException& ex) {
 		fprintf(stderr, "ogmbicp: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		latestScan->header.stamp.sec, latestScan->header.stamp.nsec);
   	return;
   }
   PointCloudPtr cloud = new PointCloud(base_frame, PointCloud(latestScan, true), sensorPose);
   if (!isInit) {
      isInit = true;
      pos_tracker.initialiseTrack(sensorPose, cloud);
   } else {
      pos_tracker.updateTrack(sensorPose, cloud);
   }
}
