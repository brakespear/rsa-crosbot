/*
 * main.cpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_ogmbicp/OgmbicpNode.hpp>
#include <crosbot_ogmbicp_gpu/OgmbicpGPU.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "ogmbicp_gpu");

   ros::NodeHandle nh;

   OgmbicpGPU posTracker;
   OgmbicpNode node(posTracker);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


