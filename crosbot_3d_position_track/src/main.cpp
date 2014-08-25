/*
 * main.cpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_3d_position_track/positionTrack3DNode.hpp>
#include <crosbot_3d_position_track/positionTrack3D.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "positiontrack3D");

   ros::NodeHandle nh;

   PositionTrack3D positionTrack3D;
   PositionTrack3DNode node(positionTrack3D);
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


