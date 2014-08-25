/*
 * positionTrack3D.hpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#ifndef POSITIONTRACK3D_HPP_
#define POSITIONTRACK3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>

using namespace std;
using namespace crosbot;

class PositionTrack3D {
public:
   /*
    * Initialise parameters
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Start 3d position tracking
    */
   void start();

   /*
    * Shutdown node
    */
   void stop();

};
   

#endif
