/*
 * main.cpp
 *
 * Created on: 12/9/2013
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_ogmbicp/OgmbicpNode.hpp>
#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "ogmbicp");

   ros::NodeHandle nh;

   OgmbicpCPU posTracker;
   OgmbicpNode node(posTracker);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


