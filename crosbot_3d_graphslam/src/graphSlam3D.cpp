/*
 * graphslam3D.cpp
 *
 * Created on: 04/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/graphSlam3D.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

GraphSlam3D::GraphSlam3D() {
   finishedSetup = false;
   receivedCameraParams = false;
}

void GraphSlam3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("LocalMapWidth", LocalMapWidth, 10.0);
   paramNH.param<double>("LocalMapHeight", LocalMapHeight, 4.0);
}

void GraphSlam3D::setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty) {

   this->fx = fx;
   this->fy = fy;
   this->cx = cx;
   this->cy = cy;
   this->tx = tx;
   this->ty = ty;
   receivedCameraParams = true;
}

