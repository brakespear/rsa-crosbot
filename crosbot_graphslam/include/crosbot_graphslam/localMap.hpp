/*
 * localMap.hpp
 *
 * Created on: 29/7/2014
 *     Author: adrianr
 */

#ifndef CROSBOT_GRAPHSLAM_LOCALMAP_HPP_
#define CROSBOT_GRAPHSLAM_LOCALMAP_HPP_

#include <crosbot/data.hpp>
#include <crosbot/PointCloudMsg.h>
#include <crosbot_graphslam/LocalMapMsg.h>
namespace crosbot {

class LocalMapInfo : public Data {
public:

   /*
    * The global position of the origin of the local map
    */
   Pose pose;

   /*
    * Index of the local map
    */
   uint32_t index;

   /*
    * Point cloud of the local map
    */
   PointCloudPtr cloud;

   LocalMapInfo(): index(-1) {}
   LocalMapInfo(Pose pose, uint32_t index): pose(pose), index(index) {}
   LocalMapInfo(Pose pose, uint32_t index, PointCloudPtr cloud): 
      pose(pose), index(index), cloud(cloud) {}


#ifdef ROS_VERSION
   inline LocalMapInfo& operator=(const crosbot_graphslam::LocalMapMsg& msg) {
      index = msg.index;
      timestamp = msg.header.stamp;
      pose = msg.pose;

      if(msg.cloud.size() == 1) {
         cloud = new PointCloud(msg.cloud[0]);
      }

      return *this;
   }

   inline LocalMapInfo& operator=(const crosbot_graphslam::LocalMapMsgConstPtr& msg) {
      return *this = *msg;
   }

   LocalMapInfo(const crosbot_graphslam::LocalMapMsg& msg) {
      *this = msg;
   }

   LocalMapInfo(const crosbot_graphslam::LocalMapMsgConstPtr& msg) {
      *this = *msg;
   }

   inline crosbot_graphslam::LocalMapMsgPtr toROS() const {
      crosbot_graphslam::LocalMapMsgPtr rval(new crosbot_graphslam::LocalMapMsg());
      rval->index = index;
      rval->header.stamp = timestamp.toROS();
      rval->pose = pose.toROS();

      rval->cloud.push_back(*(cloud->tosROS1()));

      return rval;
   }

   inline crosbot_graphslam::LocalMapMsgPtr toROSsmall() const {
      crosbot_graphslam::LocalMapMsgPtr rval(new crosbot_graphslam::LocalMapMsg());
      rval->index = index;
      rval->header.stamp = timestamp.toROS();
      rval->pose = pose.toROS();

      return rval;
   }
#endif
};
typedef Handle<LocalMapInfo> LocalMapInfoPtr;

/*namespace serialization {

template <>
class Serializer<LocalMapInfo> {
public:
};

} //namespace serialization*/

} //namespace crosbot

#endif

