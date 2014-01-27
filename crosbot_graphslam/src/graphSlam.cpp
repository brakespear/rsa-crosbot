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
   paramNH.param<double>("GlobalMapSize", GlobalMapSize, 50);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<int>("ImgTransmitTime", ImgTransmitTime, 2000000);
   paramNH.param<double>("MinAddHeight", MinAddHeight, 1.2);
   paramNH.param<double>("MaxAddHeight", MaxAddHeight, 3.0);
   paramNH.param<double>("LaserMinDist", LaserMinDist, 0.4);
   paramNH.param<double>("LaserMaxDist", LaserMaxDist, 10.0);
   paramNH.param<double>("LocalMapSize", LocalMapSize, 16);
   paramNH.param<double>("LocalMapDistance", LocalMapDistance, 2.0);
   paramNH.param<int>("SearchSize", SearchSize, 2);
   paramNH.param<double>("MaxAlignDistance", MaxAlignDistance, 0.3);
   paramNH.param<double>("LValue", LValue, 2);
   paramNH.param<int>("InformationScaleFactor", InformationScaleFactor, 15);
   paramNH.param<double>("MaxCovar", MaxCovar, 1.5);
   //paramNH.param<double>("CorrelationThreshold", CorrelationThreshold, 3.4);
   paramNH.param<double>("CorrelationThreshold", CorrelationThreshold, 3.5);
   paramNH.param<int>("MinGoodCount", MinGoodCount, 10);
   //paramNH.param<int>("FinalMinGoodCount", FinalMinGoodCount, 175);
   paramNH.param<int>("FinalMinGoodCount", FinalMinGoodCount, 225);
   //paramNH.param<int>("FinalMinGoodCount", FinalMinGoodCount, 40);
   paramNH.param<int>("MaxIterations", MaxIterations, 60);
   paramNH.param<double>("MaxErrorTheta", MaxErrorTheta, 0.03);
   paramNH.param<double>("MaxErrorDisp", MaxErrorDisp, 0.0005);
   paramNH.param<int>("MaxNumConstraints", MaxNumConstraints, 200);
   paramNH.param<int>("MaxNumLoopConstraints", MaxNumLoopConstraints, 50);
   paramNH.param<bool>("LocalMapCombine", LocalMapCombine, false);
   paramNH.param<double>("MaxThetaOptimise", MaxThetaOptimise, M_PI / 2.0);
   paramNH.param<int>("HistoryTime", HistoryTime, 1000000);
   paramNH.param<int>("MinObservationCount", MinObservationCount, 30);
   paramNH.param<double>("InitHeight", InitHeight, 1.0);
   paramNH.param<bool>("LocalMapWarp", LocalMapWarp, true);

   DimLocalOG = LocalMapSize / CellSize;
   DimGlobalOG = GlobalMapSize / CellSize;

}

crosbot::ImagePtr GraphSlam::drawMap(vector<LocalMapPtr> globalMaps, Pose icpPose, 
      vector<double> mapSlices) {
   Time currentTime = Time::now();
   if ((currentTime - lastHistoryTime).toSec() > HistoryTime / 1000000.0) {
      lastHistoryTime = currentTime;
      addPoseHistory(icpPose);
   }

   if ((currentTime - lastImgTime).toSec() > ImgTransmitTime / 1000000.0) {
      lastImgTime = currentTime;
      getGlobalMap(globalMaps, mapSlices);
      ImagePtr globalMapImage = globalMaps[0]->getImage();
      return globalMapImage;
   } else {
      return NULL;
   }
}

void GraphSlam::addSnap(SnapPtr newSnap) {

{{ Lock lock(masterLockSmith);

   SlamSnap *newSlamSnap = new SlamSnap();
   newSlamSnap->snap = newSnap;
   newSlamSnap->localMapIndex = currentLocalMap; 
   newSlamSnap->localOffset = currentLocalMapICPPose;

   double tempX = newSnap->robot.position.x - currentLocalMapICPPose.position.x;
   double tempY = newSnap->robot.position.y - currentLocalMapICPPose.position.y;
   double yawRobot, yawMap, pitch, roll;
   newSnap->robot.getYPR(yawRobot, pitch, roll);
   currentLocalMapICPPose.getYPR(yawMap, pitch, roll);
   double cosTh = cos(-yawMap);
   double sinTh = sin(-yawMap);
   newSlamSnap->localOffset.position.x = tempX * cosTh - tempY * sinTh;
   newSlamSnap->localOffset.position.y = tempX * sinTh + tempY * cosTh;
   yawRobot -= yawMap;
   ANGNORM(yawRobot);
   newSlamSnap->localOffset.setYPR(yawRobot, pitch, roll);

   snaps.push_back(newSlamSnap);

}}
   
}

void GraphSlam::getSnaps(vector<SnapPtr>& list) {

{{ Lock lock(masterLockSmith);

   list.resize(snaps.size());
   int i;
   for(i = 0; i < snaps.size(); i++) {
      list[i] = snaps[i]->snap;
      int localMapIndex = snaps[i]->localMapIndex;

      double yaw, roll, pitch;

      double gx, gy, gth;
      getGlobalMapPosition(localMapIndex, gx, gy, gth);

      double cosTh = cos(gth);
      double sinTh = sin(gth);

      list[i]->robot.position.x = snaps[i]->localOffset.position.x * cosTh - 
                                 snaps[i]->localOffset.position.y * sinTh +
                                 gx;
      list[i]->robot.position.y = snaps[i]->localOffset.position.x * sinTh + 
                                 snaps[i]->localOffset.position.y * cosTh +
                                 gy;
      snaps[i]->localOffset.getYPR(yaw, roll, pitch);
      yaw += gth;
      ANGNORM(yaw);
      list[i]->robot.setYPR(yaw, roll, pitch);
   }

}}
}

bool GraphSlam::updateSnap(uint32_t id, uint8_t type, string description, uint8_t status) {
   {{ Lock lock(masterLockSmith);

   int i;
   for (i = snaps.size() - 1; i >= 0; --i) {
      if (snaps[i]->snap->id == id && snaps[i]->snap->type == (Snap::Type)type) {
         snaps[i]->snap->description = description;
         snaps[i]->snap->status = (Snap::Status)status;
         return true;
      }
   }
   return false;

}}

}

bool GraphSlam::getSnap(uint32_t id, uint8_t type, SnapPtr& snap) {

{{ Lock lock(masterLockSmith);

   int i;
   for (i = snaps.size() - 1; i >= 0; --i) {
      if (snaps[i]->snap->id == id && snaps[i]->snap->type == (Snap::Type)type) {
         snap = snaps[i]->snap;
         return true;
      }
   }
   return false;


}}

}

void GraphSlam::addPoseHistory(Pose icpPose) {
   SlamHistory historyNode;
   historyNode.localMapIndex = currentLocalMap;
   historyNode.localPose = currentLocalMapICPPose;

   double tempX = icpPose.position.x - currentLocalMapICPPose.position.x;
   double tempY = icpPose.position.y - currentLocalMapICPPose.position.y;
   double yawRobot, yawMap, pitch, roll;
   icpPose.getYPR(yawRobot, pitch, roll);
   currentLocalMapICPPose.getYPR(yawMap, pitch, roll);
   double cosTh = cos(-yawMap);
   double sinTh = sin(-yawMap);

   historyNode.localPose.position.x = tempX * cosTh - tempY * sinTh;
   historyNode.localPose.position.y = tempX * sinTh + tempY * cosTh;
   yawRobot -= yawMap;
   ANGNORM(yawRobot);
   historyNode.localPose.setYPR(yawRobot, pitch, roll);
   historyNode.stamp = ros::Time::now();
   //historyNode.localPose.header.frame_id = "slam_frame";
   historySlam.push_back(historyNode);

}

vector<geometry_msgs::PoseStamped>& GraphSlam::getSlamHistory() {
   int i;
   for(i = historySlamPoses.size(); i < historySlam.size(); ++i) {
      int localMapIndex = historySlam[i].localMapIndex;
      double yaw, roll, pitch;

      double gx, gy, gth;
      getGlobalMapPosition(localMapIndex, gx, gy, gth);

      double cosTh = cos(gth);
      double sinTh = sin(gth);

      Pose slamPos = historySlam[i].localPose;
      slamPos.position.x = historySlam[i].localPose.position.x * cosTh - 
                           historySlam[i].localPose.position.y * sinTh +
                           gx;
      slamPos.position.y = historySlam[i].localPose.position.x * sinTh + 
                           historySlam[i].localPose.position.y * cosTh +
                           gy;
      slamPos.getYPR(yaw, roll, pitch);
      yaw += gth;
      ANGNORM(yaw);
      slamPos.setYPR(yaw, roll, pitch);
      geometry_msgs::Pose rosPos = slamPos.toROS();
      geometry_msgs::PoseStamped stampedPos;
      stampedPos.pose = rosPos;
      stampedPos.header.frame_id = "slam_frame";
      stampedPos.header.stamp = historySlam[i].stamp;
      historySlamPoses.push_back(stampedPos);
      historySlam[i].currentSlamPose = slamPos;
   }
   return historySlamPoses;
}

void GraphSlam::addSlamTrack(ImagePtr mapImage) {

   int i;
   for(i = 0; i < historySlam.size(); i++) {

      double x = historySlam[i].currentSlamPose.position.x;
      double y = historySlam[i].currentSlamPose.position.y;
      double off = (DimGlobalOG * CellSize) / 2.0;
      int i = (x + off) / CellSize;
      int j = (y + off) / CellSize;
      uint8_t *pxl = ((uint8_t *)mapImage->data) + ((DimGlobalOG - j - 1)
            * mapImage->step) + (i * 3);
      pxl[0] = 0;
      pxl[1] = 255;
      pxl[2] = 0;
   }

}

void GraphSlam::fixSnapPositions(int combineIndex, double alignX, double alignY, double alignTh) {
   int k;
   //Adjust the local map coords of the recent snaps
   for (k = snaps.size() - 1; k >= 0 && snaps[k]->localMapIndex == currentLocalMap; k--) {
      snaps[k]->localMapIndex = combineIndex;
      double yaw, pitch, roll;
      snaps[k]->localOffset.getYPR(yaw, pitch, roll);
                     
      double cosTh = cos(-alignTh);
      double sinTh = sin(-alignTh);
      double tempX = snaps[k]->localOffset.position.x - alignX;
      double tempY = snaps[k]->localOffset.position.y - alignY;
      snaps[k]->localOffset.position.x = tempX * cosTh - tempY * sinTh;
      snaps[k]->localOffset.position.y = tempX * sinTh + tempY * cosTh;
      yaw -= alignTh;
      ANGNORM(yaw);
      snaps[k]->localOffset.setYPR(yaw, pitch, roll);
   }
}

void GraphSlam::fixSlamHistoryPositions(int combineIndex, double alignX, 
      double alignY, double alignTh) {
   //Adjust the local map coords of the recent slam history
   int k;
   for(k = historySlam.size() - 1; k >= 0 && historySlam[k].localMapIndex ==
                currentLocalMap; --k) {
      historySlam[k].localMapIndex = combineIndex;
      double yaw, roll, pitch;
      historySlam[k].localPose.getYPR(yaw, pitch, roll);
      double cosTh = cos(-alignTh);
      double sinTh = sin(-alignTh);
      double tempX = historySlam[k].localPose.position.x - alignX;
      double tempY = historySlam[k].localPose.position.y - alignY;
      historySlam[k].localPose.position.x = tempX * cosTh - tempY * sinTh;
      historySlam[k].localPose.position.y = tempX * sinTh + tempY * cosTh;
      yaw -= alignTh;
      ANGNORM(yaw);
      historySlam[k].localPose.setYPR(yaw, pitch, roll);
   }
}


