/*
 * main.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_graphslam/graphSlamNode.hpp>
#include <crosbot_graphslam/graphSlamCPU.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "graphslam");

   ros::NodeHandle nh;

   FactoryGraphSlamCPU factoryGraphSlam;
   GraphSlamNode node(factoryGraphSlam);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


