/*
 * depthPoints.hpp
 *
 * Created on: 08/08/2014
 *     Author: adrianr
 */

#ifndef DEPTHPOINTS_HPP_
#define DEPTHPOINTS_HPP_

#include <crosbot/data.hpp>
#include <crosbot/geometry.hpp>
#include <tf/tf.h>
#include <sensor_msgs/PointCloud2.h>

using namespace crosbot;

class DepthPoints : public PointCloud {
public:

   //vector of normals?
   
   int width;
   int height;
   
   DepthPoints(const sensor_msgs::PointCloud2ConstPtr& c, int skip = 1, bool ignoreColour = false);
   //DepthPoints(const sensor_msgs::PointCloud2ConstPtr& cloud, double minDist, double maxDist, 
   //      int skip, bool filter = false, bool calcNormals = false);

   /*
    * Transform the valid points according to the pose
    */ 
   void transform(Pose correction);
   void transform(tf::Transform trans);

   /*
    * Filter out points too close or too far
    */
   void filterDistance(double minDist, double maxDist);

   //calc normals
   //bilateral filter

};
typedef Handle<DepthPoints> DepthPointsPtr;

#endif
