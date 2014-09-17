/*
 * main.cpp
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_3d_graphslam/graphSlam3DNode.hpp>
#include <crosbot_3d_graphslam_gpu/graphSlam3DGPU.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "graphslam3DGPU");

   ros::NodeHandle nh;

   GraphSlam3DGPU graphSlam3D;
   GraphSlam3DNode node(graphSlam3D);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


