/*
 * main.cpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_3d_graphslam_full/graphSlamFull3DNode.hpp>
#include <crosbot_3d_graphslam_full/graphSlamFull3D.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "graphslamFull3D");

   ros::NodeHandle nh;

   GraphSlamFull3D graphSlamFull3D;
   GraphSlamFull3DNode node(graphSlamFull3D);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


