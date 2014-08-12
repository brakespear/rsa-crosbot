/*
 * depthPoints.cpp
 *
 * Created on: 08/08/2014
 *     Author: adrianr
 */

#include<ros/ros.h>
#include<crosbot_3d_graphslam/depthPoints.hpp>

//DepthPoints::DepthPoints(const sensor_msgs::PointCloud2ConstPtr& cloud,double minDist, 
//      double maxDist, int skip, bool filter = false, bool calcNormals = false) {

DepthPoints::DepthPoints(const sensor_msgs::PointCloud2ConstPtr& c, int skip = 1) { 
   timestamp = c->header.stamp;
   frameID = c->header.frame_id;

   //Read the structure of the point cloud
   width = c->width / skip;
   height = c->height / skip;   

   int numPoints = height * width;
   uint32_t pointSize = c->point_step * skip;
   uint32_t rowSize = c->row_step * skip;

   int offsetX = -1, offsetY = -1, offsetZ = -1, offsetColour = -1;
   bool haveRGBA = false;

   for (int f = 0; f < c->fields.size(); ++f) {
      const sensor_msgs::PointField& field = c->fields[f];
      if (strcasecmp(field.name.c_str(), "x") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
         offsetX = field.offset;
      } else if (strcasecmp(field.name.c_str(), "y") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
         offsetY = field.offset;
      } else if (strcasecmp(field.name.c_str(), "z") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
         offsetZ = field.offset;
      } else if (strcasecmp(field.name.c_str(), "rgb") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
         offsetColour = field.offset;
      } else if (strcasecmp(field.name.c_str(), "rgba") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
         offsetColour = field.offset;
         haveRGBA = true;
      }
   }
   
   cloud.resize(numPoints);
   if (offsetColour >= 0) {
      colours.resize(numPoints);
   }
   int numAdded = 0;
   int row, col;
   for (uint32_t y = 0; y < height; ++y) {
      const uint8_t *point = &(c->data[y * rowSize]);
      for (uint32_t x = 0; x < width; ++x, point += pointSize) {
         Point& p = cloud[y * width + x];
         if (offsetX >= 0) {
            p.x = *(float *)(point + offsetX);
         }
         if (offsetY >= 0) {
            p.y = *(float *)(point + offsetY);
         }
         if (offsetZ >= 0) {
            p.z = *(float *)(point + offsetZ);
         }
         if (offsetColour >= 0) {
            Colour& colour = colours[y * width + x];
            colour.b = *(point + offsetColour);
            colour.g = *(point + offsetColour + 1);
            colour.r = *(point + offsetColour + 2);
            if (haveRGBA) {
               colour.a = *(point + offsetColour + 3);
            } else {
               colour.a = 255;
            }
         }
      }
   }
}

void DepthPoints::transform(tf::Transform trans) {
  for(int i = 0; i < cloud.size(); i++) {
     if (!isnan(cloud[i].x)) {
        cloud[i] = trans * cloud[i].toTF();
     }
  }
}

void DepthPoints::transform(Pose correction) {
  transform(correction.toTF());
}

void DepthPoints::filterDistance(double minDist, double maxDist) {
   double minD = minDist * minDist;
   double maxD = maxDist * maxDist;

   for (int i = 0; i < cloud.size(); i++) {
      if (!isnan(cloud[i].x)) {
         double dist = cloud[i].x * cloud[i].x + cloud[i].y * cloud[i].y + cloud[i].z * cloud[i].z;
         if (dist < minD || dist > maxD) {
            cloud[i].x = NAN;
         }
      }
   }
}

