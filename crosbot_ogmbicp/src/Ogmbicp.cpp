/*
 * Ogmbicp.cpp
 *
 * Created on: 16/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/Ogmbicp.hpp>

void Ogmbicp::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("MapSize", MapSize, 10);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("CellHeight", CellHeight, 0.05);

}

