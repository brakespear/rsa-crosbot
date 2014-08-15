/*
 * graphSlamDisplay.cpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlamDisplay::GraphSlamDisplay() {
}

void GraphSlamDisplay::initialise(ros::NodeHandle &nh) {
   //ros::NodeHandle paramNH("~");
}

void GraphSlamDisplay::start() {
}

void GraphSlamDisplay::stop() {
}

void GraphSlamDisplay::addMap(LocalMapInfoPtr localMapPoints) {
   points.timestamp = localMapPoints->timestamp;
   points.frameID = localMapPoints->cloud->frameID;
   
   tf::Transform mapPose = localMapPoints->pose.toTF();
   int startIndex = points.cloud.size();
   points.cloud.resize(startIndex + localMapPoints->cloud->cloud.size());
   //points.colours.resize(startIndex + localMapPoints->cloud->colours.size());

   cout << "Number of points in cloud received is: " << localMapPoints->cloud->cloud.size() << " " << localMapPoints->cloud->colours.size() << endl;
   for (int i = 0; i < localMapPoints->cloud->cloud.size(); i++) {
      points.cloud[startIndex + i] = mapPose * localMapPoints->cloud->cloud[i].toTF(); 
      //points.colours[startIndex + i] = localMapPoints->cloud->colours[i];
   }
}

PointCloud &GraphSlamDisplay::getPointCloud() {
   return points;
}

