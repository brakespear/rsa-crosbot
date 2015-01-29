/*
 * OgmbicpNode.cpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 *
 * Interface of Ogmbicp algorithm with ROS
 */

#include <crosbot_ogmbicp/OgmbicpNode.hpp>

using namespace std;
using namespace crosbot;

OgmbicpNode::OgmbicpNode(Ogmbicp &posTracker): icp_frame(DEFAULT_ICPFRAME),
                            base_frame(DEFAULT_BASEFRAME),
                            odom_frame(DEFAULT_ODOMFRAME),
                            pos_tracker(posTracker)
{
   //pos_tracker = posTracker;
   isInit = false;
}

void OgmbicpNode::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<std::string>("icp_frame", icp_frame, DEFAULT_ICPFRAME);
   paramNH.param<std::string>("base_frame", base_frame, DEFAULT_BASEFRAME);
   paramNH.param<std::string>("odom_frame", odom_frame, DEFAULT_ODOMFRAME);
   paramNH.param<std::string>("scan_sub", scan_sub, "/scan");
   paramNH.param<std::string>("local_map_image_pub", local_map_image_pub, "localImage");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localGrid");
   paramNH.param<std::string>("recent_scans_srv", recent_scans_srv, "icpRecentScans");
   paramNH.param<std::string>("orientation_sub", orientation_sub, "/orientation");
   paramNH.param<std::string>("reset_map_sub", reset_map_sub, "resetMap");

   pos_tracker.initialise(nh);
   pos_tracker.start();

   scanSubscriber = nh.subscribe(scan_sub, 1, &OgmbicpNode::callbackScan, this);
   orientationSubscriber = nh.subscribe(orientation_sub, 1, &OgmbicpNode::callbackOrientation, this);
   imagePub = nh.advertise<sensor_msgs::Image>(local_map_image_pub, 1);
   localMapPub = nh.advertise<nav_msgs::OccupancyGrid>(local_map_pub, 1);
   recentScansServer = nh.advertiseService(recent_scans_srv, &OgmbicpNode::getRecentScans, this);

   resetMapSubscriber = nh.subscribe(reset_map_sub, 1, &OgmbicpNode::callbackResetMap, this);

}

void OgmbicpNode::shutdown() {
   scanSubscriber.shutdown();
   pos_tracker.stop();

}

bool OgmbicpNode::getRecentScans(crosbot_ogmbicp::GetRecentScans::Request& req,
         crosbot_ogmbicp::GetRecentScans::Response& res) {
   deque<PointCloudPtr> recent;
   if (!pos_tracker.finishedSetup) {
      return false;
   }
   pos_tracker.getRecentScans(recent);
   sensor_msgs::PointCloud2& pc = res.scans;
   //sensor_msgs::PointCloud2 *pc = new sensor_msgs::PointCloud2();
   pc.header.stamp = recent.back()->timestamp.toROS();
   pc.header.frame_id = icp_frame;
   pc.is_dense = true;
   pc.is_bigendian = false;
   pc.height = recent.size();
   uint32_t width = recent.back()->cloud.size();

   pc.width = width;
   pc.point_step = sizeof(float) * 3;
   pc.row_step = pc.point_step * width;
   pc.fields.resize(3);
   pc.fields[0].name = "x";
   pc.fields[1].name = "y";
   pc.fields[2].name = "z";
   for (int i = 0; i < 3; i++) {
      pc.fields[i].offset = i * sizeof(float);
      pc.fields[i].datatype = 7;
      pc.fields[i].count = 1;
   }
   pc.data.resize(pc.point_step * width * pc.height);
   for (uint64_t i = 0; i < pc.height; i++) {
      uint64_t off = i * pc.point_step * recent.back()->cloud.size();
      std::vector<Point> cur = recent.at(i)->cloud;
      uint64_t j = 0;
      for (; j < width && j < cur.size(); j++) {
         uint64_t localOff = j * pc.point_step;
         float arr[3];
         arr[0] = cur[j].x;
         arr[1] = cur[j].y;
         arr[2] = cur[j].z;
         memcpy(&(pc.data[off + localOff]), arr, pc.point_step);
      }
      for (; j < width; j++) {
         uint64_t localOff = j * pc.point_step;
         memset(&(pc.data[off + localOff]), 0, pc.point_step);
      }
   }

   return true;
}


void OgmbicpNode::callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
   Pose odomPose, sensorPose;
  	tf::StampedTransform laser2Base, base2Odom;
   //odom_frame = "/odom";
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
   //sensorPose.position = Point();
   if (!isInit) {
      isInit = true;
      pos_tracker.initialiseTrack(sensorPose, cloud);
      uint32_t dim = (uint32_t) (pos_tracker.MapSize / pos_tracker.CellSize);
      localMap = new LocalMap(dim, dim, pos_tracker.CellSize, icp_frame);
   } else {
      pos_tracker.updateTrack(sensorPose, cloud);
   }
   ImagePtr image = pos_tracker.drawMap(localMap);
   if (image != NULL) {
      localMapPub.publish(localMap->getGrid());
      imagePub.publish(image->toROS());
   }

   Pose icpPose = pos_tracker.curPose;
   //cout << "Pose from icp is: " << icpPose << endl;
   if (haveOdometry) {
      icpPose = icpPose.toTF() * odomPose.toTF().inverse();
      geometry_msgs::TransformStamped icpTs = getTransform(icpPose, odom_frame, icp_frame, latestScan->header.stamp);
      tfPub.sendTransform(icpTs);
   } else {
      geometry_msgs::TransformStamped icpTs = getTransform(icpPose, base_frame, icp_frame, latestScan->header.stamp);
      tfPub.sendTransform(icpTs);
   }
}

void OgmbicpNode::callbackOrientation(const geometry_msgs::Quaternion& quat) {
   pos_tracker.processImuOrientation(quat);
}

void OgmbicpNode::callbackResetMap(const std_msgs::String& name) {
   ROS_INFO("OgmbicpNode :: Resetting Map");
   pos_tracker.resetMap();
}

geometry_msgs::TransformStamped OgmbicpNode::getTransform(const Pose& pose, std::string childFrame, 
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

