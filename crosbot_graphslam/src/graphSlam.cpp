/*
 * graphSlam.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * Common code for all graph slam methods    
 */

#include <newmat/newmat.h>
#include <crosbot_graphslam/graphSlam.hpp>

using namespace NEWMAT;

void GraphSlam::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("MapSize", MapSize, 60);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<int>("ImgTransmitTime", ImgTransmitTime, 2000000);
}

crosbot::ImagePtr GraphSlam::drawMap(LocalMapPtr globalMap) {
   Time currentTime = Time::now();
   if ((currentTime - lastImgTime).toSec() > ImgTransmitTime / 1000000.0) {
      lastImgTime = currentTime;
      getGlobalMap(globalMap);
      ImagePtr globalMapImage = globalMap->getImage();
      return globalMapImage;
   } else {
      return NULL;
   }
}

