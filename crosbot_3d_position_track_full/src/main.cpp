/*
 * main.cpp
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#include <ros/ros.h>

#include <crosbot_3d_position_track_full/positionTrackFull3DNode.hpp>
#include <crosbot_3d_position_track_full/positionTrackFull3D.hpp>

int main(int argc, char **argv) {
   ros::init(argc, argv, "positiontrackfull3D");

   ros::NodeHandle nh;

   //PositionTrackFull3D positionTrackFull3D;
   //PositionTrackFull3DNode node(positionTrackFull3D);
   PositionTrackFull3DNode node;
   node.initialise(nh);

   while (ros::ok()) {
      ros::spin();
   }
   node.shutdown();

   return 0;
}


