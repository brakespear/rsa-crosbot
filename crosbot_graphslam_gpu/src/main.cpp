/*
 * main.cpp
 *
 * Created on: 8/1/2014
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_graphslam/graphSlamNode.hpp>
#include <crosbot_graphslam_gpu/graphSlamGPU.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "graphslam_gpu");

   ros::NodeHandle nh;

   GraphSlamGPU graphSlam;
   GraphSlamNode node(graphSlam);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


