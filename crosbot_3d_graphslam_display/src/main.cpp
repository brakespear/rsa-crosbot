/*
 * main.cpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_3d_graphslam_display/graphSlamDisplayNode.hpp>
#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "graphSlamDisplay");

   ros::NodeHandle nh;

   GraphSlamDisplay graphSlamDisplay;
   GraphSlamDisplayNode node(graphSlamDisplay);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


