/*
 * GraphSlamNode.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * Interface of graph slam algorithm with ROS
 */

#include <ros/ros.h>

#include <crosbot_graphslam/graphSlamNode.hpp>

using namespace std;
using namespace crosbot;

GraphSlamNode::GraphSlamNode(FactoryGraphSlam& factoryGraphSlam) :
    factoryGraphSlam(factoryGraphSlam),
    icp_frame(DEFAULT_ICPFRAME),
    base_frame(DEFAULT_BASEFRAME)
{
   isInit = false;
   lastCaptured = ros::Time::now();
   lastPublishedMap = ros::Time::now();

   useKinect = false;
   kinectCaptureRate = 0;
   PublishLocalMapInfo = false;
   HighMapSliceHeight = false;
   IncludeHighMapSlice = false;
   globalMapPublishRate = 0;

   graph_slam = NULL;
}

void GraphSlamNode::initialise(ros::NodeHandle &nh) {
   ROS_INFO("GraphSlamNode :: initialise");

   isInit = false;
   lastCaptured = ros::Time::now();
   lastPublishedMap = ros::Time::now();

   nhNamespace = nh.getNamespace();
   paramNamespace = "~";
   ros::NodeHandle paramNH(paramNamespace);
   paramNH.param<std::string>("icp_frame", icp_frame, DEFAULT_ICPFRAME);
   paramNH.param<std::string>("base_frame", base_frame, DEFAULT_BASEFRAME);
   paramNH.param<std::string>("slam_frame", slam_frame, "/slam");
   paramNH.param<std::string>("scan_sub", scan_sub, "scan");
   paramNH.param<std::string>("snap_sub", snap_sub, "snaps");
   paramNH.param<std::string>("reset_sub", reset_sub, "/graphslam/resetMap");
   paramNH.param<std::string>("global_map_image_pub", global_map_image_pub, "globalImage");
   paramNH.param<std::string>("slam_history_pub", slam_history_pub, "slamHistory");
   paramNH.param<std::string>("global_grid_pub", global_grid_pub, "slamGrid");
   paramNH.param<std::string>("local_map_pub", local_map_pub, "localMapInfo");
   paramNH.param<std::string>("optimise_map_srv", optimise_map_srv, "optimiseMapInfo");
   paramNH.param<std::string>("snap_list_srv", snap_list_srv, "snaps_list");
   paramNH.param<std::string>("snap_update_srv", snap_update_srv, "snap_update");
   paramNH.param<std::string>("snap_get_srv", snap_get_srv, "snap_get");

   paramNH.param<bool>("useKinect", useKinect, false);
   paramNH.param<std::string>("kinect_sub", kinect_sub, "/camera/depth_registered/points");
   paramNH.param<std::string>("world_pub", world_pub, "worldMap");
   paramNH.param<int>("kinectCaptureRate", kinectCaptureRate, 1500000);
   paramNH.param<int>("globalMapPublishRate", globalMapPublishRate, 25000000);


   paramNH.param<bool>("IncludeHighMapsSlice", IncludeHighMapSlice, false);
   paramNH.param<double>("HighMapSliceHeight", HighMapSliceHeight, 2.0);
   paramNH.param<bool>("PublishLocalMapInfo", PublishLocalMapInfo, true);

   // Configure graph slam entity
   graph_slam = factoryGraphSlam.makeGraphSlam();
   graph_slam->graphSlamNode = this;
   graph_slam->initialise(nh);
   graph_slam->start();

   slamGridPubs.push_back(nh.advertise<nav_msgs::OccupancyGrid>(global_grid_pub, 1, true));
   mapSlices.push_back(graph_slam->MinAddHeight);
   if (IncludeHighMapSlice) {
      string name = global_grid_pub + "High";
      slamGridPubs.push_back(nh.advertise<nav_msgs::OccupancyGrid>(name, 1));
      mapSlices.push_back(HighMapSliceHeight);
   }
   scanSubscriber = nh.subscribe(scan_sub, 1, &GraphSlamNode::callbackScan, this);
   snapSubscriber = nh.subscribe(snap_sub, 1, &GraphSlamNode::callbackSnaps, this);
   resetSubscriber = nh.subscribe(reset_sub, 1, &GraphSlamNode::callbackResetMap, this);
   imagePub = nh.advertise<sensor_msgs::Image>(global_map_image_pub, 1);
   slamHistoryPub = nh.advertise<nav_msgs::Path>(slam_history_pub, 1);
   snapListServer = nh.advertiseService(snap_list_srv, &GraphSlamNode::getSnapsList, this);
   snapUpdateServer = nh.advertiseService(snap_update_srv, &GraphSlamNode::snapUpdate, this);
   snapGetServer = nh.advertiseService(snap_get_srv, &GraphSlamNode::snapGet, this);

   if (PublishLocalMapInfo) {
      localMapInfoPub = nh.advertise<crosbot_graphslam::LocalMapMsg>(local_map_pub, 1);
      //optimiseMapPub = nh.advertise<crosbot_graphslam::LocalMapMsgList>(optimise_map_pub, 1);
      optimiseMapService = nh.serviceClient<crosbot_graphslam::LoopClose>(optimise_map_srv);
   }

   //Kinect subscriber
   if (useKinect) {
      kinectSub = nh.subscribe(kinect_sub, 1, &GraphSlamNode::callbackKinect, this);
      worldMap = nh.advertise<sensor_msgs::PointCloud2>(world_pub, 1, true);
      worldScan.header.frame_id = slam_frame;
      worldScan.is_bigendian = false;
      worldScan.is_dense = true;
      worldScan.height = 1;
      worldScan.fields.resize(4);
      worldScan.fields[0].name = "x";
      worldScan.fields[1].name = "y";
      worldScan.fields[2].name = "z";
      worldScan.fields[3].name = "rgb";
      for (int i = 0; i < 4; i++) {
         worldScan.fields[i].offset = i * sizeof(float);
         worldScan.fields[i].datatype = 7;
         worldScan.fields[i].count = 1;
      }
      worldScan.fields[3].offset = 16;
      worldScan.point_step = sizeof(float) * 8;
   }

   //Debugging publisher
   imageTestPub = nh.advertise<sensor_msgs::Image>("slamTest", 1);

   ROS_INFO("GraphSlamNode :: initialise - done");
}

void GraphSlamNode::shutdown() {
   // Shutdown subscribers, publishers and services
   ROS_INFO("GraphSlamNode :: shutdown");
   scanSubscriber.shutdown();
   snapSubscriber.shutdown();
   resetSubscriber.shutdown();
   imagePub.shutdown();
   slamHistoryPub.shutdown();
   localMapInfoPub.shutdown();
   optimiseMapService.shutdown();
   snapListServer.shutdown();
   snapUpdateServer.shutdown();
   snapGetServer.shutdown();

   for(ros::Publisher pub : slamGridPubs) {
      pub.shutdown();
   }
   slamGridPubs.clear();

   // Stop and delete GraphSlam
   graph_slam->stop();
   delete graph_slam;

   // Clear out data
   globalMaps.clear();
   mapSlices.clear();
   testMap = NULL;
}

void GraphSlamNode::reset() {
   // Shutdown
   shutdown();

   // Re-initialise - initialise already calls graph_slam->start()
   ros::NodeHandle nh(nhNamespace);
   initialise(nh);
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

      tfListener.waitForTransform(icp_frame, base_frame, latestScan->header.stamp, ros::Duration(1, 0));
      tfListener.lookupTransform(icp_frame, base_frame, latestScan->header.stamp, base2Icp);
      icpPose = base2Icp;

   } catch (tf::TransformException& ex) {
      fprintf(stderr, "graph slam: Error getting transform. (%s) (%d.%d)\n", ex.what(),
      latestScan->header.stamp.sec, latestScan->header.stamp.nsec);
      return;
   } 

   PointCloudPtr cloud = new PointCloud(base_frame, PointCloud(latestScan, true), sensorPose);
   if (!isInit) {
      isInit = true;
      graph_slam->initialiseTrack(icpPose, cloud);
      uint32_t dim = (uint32_t) (graph_slam->DimGlobalOG);
      int i;
      for (i = 0; i < mapSlices.size(); i++) {
         globalMaps.push_back(new LocalMap(dim, dim, graph_slam->CellSize, slam_frame));
      }
      //Debugging publisher
      dim = (uint32_t)(graph_slam->DimLocalOG);
      //dim = (uint32_t)(graph_slam->DimGlobalOG);
      testMap = new LocalMap(dim, dim, graph_slam->CellSize, slam_frame);
      graph_slam->testMap = testMap;
   } else {
      graph_slam->updateTrack(icpPose, cloud, latestScan->header.stamp);
   }

   ImagePtr image = graph_slam->drawMap(globalMaps, icpPose, mapSlices);
   if (image != NULL) {
      publishSlamHistory();
      graph_slam->addSlamTrack(image);
      int i;
      for (i = 0; i < mapSlices.size(); i++) {
         slamGridPubs[i].publish(globalMaps[i]->getGrid());
      }
      imagePub.publish(image->toROS());

      //Debugging publisher
      imageTestPub.publish((testMap->getImage())->toROS());
   }

   Pose slamPose = graph_slam->slamPose;
   Pose correction = slamPose.toTF() * icpPose.toTF().inverse();
   geometry_msgs::TransformStamped slamCor = getTransform(correction, icp_frame, slam_frame, base2Icp.stamp_);
   tfPub.sendTransform(slamCor);

}

void GraphSlamNode::callbackSnaps(const crosbot_map::SnapMsg& newSnapMsg) {
   SnapPtr newSnap = new Snap(newSnapMsg);
   graph_slam->addSnap(newSnap);
}

void GraphSlamNode::callbackResetMap(std_msgs::StringConstPtr resetMsg) {
   ROS_INFO("GraphSlamNode :: Resetting Map");
   reset();
}

bool GraphSlamNode::getSnapsList(crosbot_map::ListSnaps::Request& req, 
         crosbot_map::ListSnaps::Response& res) {
   if (!graph_slam->finishedSetup) {
      return false;
   }
   vector<SnapPtr> list;
   graph_slam->getSnaps(list);
   res.snaps.resize(list.size());
   int i;
   for (i = 0; i < list.size(); i++) {
      res.snaps[i] = *list[i]->toROSsmall();
   }
   return true;
}

bool GraphSlamNode::snapUpdate(crosbot_map::ModifySnap::Request& req, 
         crosbot_map::ModifySnap::Response& res) {
   if (!graph_slam->finishedSetup) {
      return false;
   }
   return graph_slam->updateSnap(req.id.data, req.type.data,
         req.description.data, req.status.data);

}

bool GraphSlamNode::snapGet(crosbot_map::GetSnap::Request& req, 
         crosbot_map::GetSnap::Response& res) {
   if (!graph_slam->finishedSetup) {
      return false;
   }
   SnapPtr snap = new Snap();
   if (!graph_slam->getSnap(req.id.data, req.type.data, snap)) {
      return false;
   }
   res.snap = *(snap->toROS());
   return true;
}

void GraphSlamNode::publishSlamHistory() {
   vector<geometry_msgs::PoseStamped>& history = graph_slam->getSlamHistory();
   nav_msgs::Path path;
   path.poses = history;
   path.header.frame_id = slam_frame;
   path.header.stamp = ros::Time::now();
   slamHistoryPub.publish(path);
}

void GraphSlamNode::publishLocalMapInfo(LocalMapInfo& info) {
   if (PublishLocalMapInfo) {
      localMapInfoPub.publish(info.toROSsmall());
   }
}

void GraphSlamNode::publishOptimiseLocalMapInfo(vector<LocalMapInfoPtr>& localMapInfo, vector<int> iCon, vector<int> jCon, bool wasFullLoop) {
   if (PublishLocalMapInfo) {
      /*crosbot_graphslam::LocalMapMsgList list;
      list.localMaps.resize(localMapInfo.size());
      for (int i = 0; i < localMapInfo.size(); i++) {
         list.localMaps[i] = *(localMapInfo[i]->toROSsmall());
      }
      optimiseMapPub.publish(list);*/
      crosbot_graphslam::LoopClose optInfo;
      optInfo.request.wasFullLoop = wasFullLoop;
      optInfo.request.i.resize(iCon.size());
      optInfo.request.j.resize(jCon.size());
      for (int i = 0; i < iCon.size(); i++) {
         optInfo.request.i[i] = iCon[i];
         optInfo.request.j[i] = jCon[i];
      }
      optInfo.request.localMaps.resize(localMapInfo.size());
      for (int i = 0; i < localMapInfo.size(); i++) {
         optInfo.request.localMaps[i] = *(localMapInfo[i]->toROSsmall());
      }
      if (!optimiseMapService.call(optInfo)) {
         cout << "ERROR: service call to 3d graph slam didn't work" << endl;
      }
      //Do any response needed
   }
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

void GraphSlamNode::callbackKinect(const sensor_msgs::PointCloud2ConstPtr& ptCloud) {
   if (ptCloud->header.stamp > lastCaptured + ros::Duration(kinectCaptureRate / 1000000.0)) {
      lastCaptured = ptCloud->header.stamp;

      Pose sensorPose;
  	   tf::StampedTransform kin2Base;
     	try {
  	   	tfListener.waitForTransform(base_frame, ptCloud->header.frame_id,
                ptCloud->header.stamp, ros::Duration(1, 0));
     		tfListener.lookupTransform(base_frame,
      				ptCloud->header.frame_id, ptCloud->header.stamp, kin2Base);
  		   sensorPose = kin2Base;
  	   } catch (tf::TransformException& ex) {
 		   fprintf(stderr, "graph slam: Error getting transform. (%s) (%d.%d)\n", ex.what(),
   		   ptCloud->header.stamp.sec, ptCloud->header.stamp.nsec);
      	return;
      }
      graph_slam->captureScan(ptCloud->data, sensorPose);

      /*cout << "Length is: " << ptCloud->fields.size() << " " << ptCloud->point_step<< endl;
      cout << ptCloud->row_step << " " << ptCloud->width << " " << ptCloud->height << " " << (int)ptCloud->is_dense << " " << (int)ptCloud->is_bigendian << endl;
      cout << ptCloud->fields[0].name << " " << (int)ptCloud->fields[0].datatype << " " << (int)ptCloud->fields[0].offset << " " << (int)ptCloud->fields[0].count << endl;
      cout << ptCloud->fields[1].name << " " << (int)ptCloud->fields[1].datatype << " " << (int)ptCloud->fields[1].offset << " " << (int)ptCloud->fields[1].count << endl;
      cout << ptCloud->fields[2].name << " " << (int)ptCloud->fields[2].datatype << " " << (int)ptCloud->fields[2].offset << " " << (int)ptCloud->fields[2].count << endl;
      cout << ptCloud->fields[3].name << " " << (int)ptCloud->fields[3].datatype << " " << (int)ptCloud->fields[3].offset << " " << (int)ptCloud->fields[3].count << endl;*/

      ros::Time currentTime = ros::Time::now();
      if ((currentTime - lastPublishedMap).toSec() > globalMapPublishRate / 1000000.0) {
         cout << "Publishing point cloud" << endl;
         //Publish the point cloud
         worldScan.header.stamp = ros::Time::now();
         graph_slam->getPoints(worldScan.data);
         worldScan.row_step = worldScan.data.size();
         worldScan.width = worldScan.row_step / worldScan.point_step;
         worldMap.publish(worldScan);
         lastPublishedMap = currentTime;
      }
   }


}


