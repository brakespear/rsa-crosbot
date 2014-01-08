/*
 * graphSlamCPU.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * CPU version of graph slam
 */

#include <newmat/newmat.h>
#include <crosbot_graphslam/graphSlamCPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

using namespace NEWMAT;

GraphSlamCPU::GraphSlamCPU() {
   numIterations = 0;
   finishedSetup = false;
   currentLocalMap = 0;
   numGlobalPoints = 0;
   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   resetMap = false;
   lastDrawnGlobalPoints = 0;
   parentLocalMap = -1;
   nextLocalMap = 0;
   combineMode = 0;
   numConstraints = 0;
}

void GraphSlamCPU::initialise(ros::NodeHandle &nh) {

   GraphSlam::initialise(nh);

   ros::NodeHandle paramNH("~");
}

void GraphSlamCPU::start() {
   common = new SlamCommon;
   common->localOG = new int[DimLocalOG * DimLocalOG];
   common->localOGCount = new int[DimLocalOG * DimLocalOG];
   common->localOGZ = new double[DimLocalOG * DimLocalOG];
   common->constraintType = new int[MaxNumConstraints];
   common->constraintIndex = new int[MaxNumConstraints];
   common->loopConstraintParent = new int[MaxNumLoopConstraints];
   common->loopConstraintI = new int[MaxNumLoopConstraints];
   common->loopConstraintJ = new int[MaxNumLoopConstraints];
   common->loopConstraintXDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintYDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintThetaDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintInfo = new (double[MaxNumLoopConstraints][3][3]);
   common->graphHessian = new (double[MaxNumConstraints + 1][3]);
}

void GraphSlamCPU::stop() {
   delete [] common->localOG;
   delete [] common->localOGCount;
   delete [] common->localOGZ;
   delete [] common->constraintType;
   delete [] common->constraintIndex;
   delete [] common->loopConstraintParent;
   delete [] common->loopConstraintI;
   delete [] common->loopConstraintJ;
   delete [] common->loopConstraintXDisp;
   delete [] common->loopConstraintYDisp;
   delete [] common->loopConstraintThetaDisp;
   delete [] common->loopConstraintInfo;
   delete [] common->graphHessian;
   delete common;
}

void GraphSlamCPU::initialiseTrack(Pose icpPose, PointCloudPtr cloud) {
   currentLocalMapICPPose = icpPose;
   oldICPPose = icpPose;

   LocalMap temp;
   localMaps.push_back(temp);
   clearMap(0);
   localMaps[0].currentGlobalPosX = 0;
   localMaps[0].currentGlobalPosY = 0;
   localMaps[0].currentGlobalPosTh = 0;
   localMaps[0].parentOffsetX = 0;
   localMaps[0].parentOffsetY = 0;
   localMaps[0].parentOffsetTh = 0;
   common->startICPTh = 0;
   common->currentOffsetX = 0;
   common->currentOffsetY = 0;
   common->currentOffsetTh = 0;
   for(int i = 0; i < DimLocalOG * DimLocalOG; i++) {
      common->localOG[i] = -1;
      common->localOGCount[i] = 0;
      common->localOGZ[i] = MinAddHeight;
   }
   for(int i = 0; i < MaxNumConstraints + 1; i++) {
      common->graphHessian[i][0] = 0;  
      common->graphHessian[i][1] = 0;  
      common->graphHessian[i][2] = 0;  
   }
   for(int i = 0; i < NUM_ORIENTATION_BINS; i++) {
      double theta = (2 * M_PI * i / (float) NUM_ORIENTATION_BINS) - 
              (M_PI * (2.0f * NUM_ORIENTATION_BINS - 1) / (float) (2 * NUM_ORIENTATION_BINS));
      common->histCos[i] = cos(theta);
      common->histSin[i] = sin(theta);
   }
   memset(localMaps[0].globalCovar, 0, sizeof(double) * 9);
   memset(common->A, 0, sizeof(double) * 9);
   memset(common->B, 0, sizeof(double) * 3);
   common->goodCount = 0;
   common->numIterations = 0;
   common->scaleFactor[0] = INFINITY;
   common->scaleFactor[1] = INFINITY;
   common->scaleFactor[2] = INFINITY;
   common->infoCount = 0;
   common->numConstraints = 0;
   common->numLoopConstraints = 0;
   common->minMapRangeX = INFINITY;
   common->maxMapRangeX = 0;
   common->minMapRangeY = INFINITY;
   common->maxMapRangeY = 0;
   common->combineIndex = -1;
   common->combineMode = 0;
   localMaps[0].indexParentNode = -1;
   localMaps[0].treeLevel = 0;

}

void GraphSlamCPU::updateTrack(Pose icpPose, PointCloudPtr cloud) {
   //cout << "The icp pose from slam is: " << icpPose << endl;

   ros::WallTime t1 = ros::WallTime::now();

   double diffX, diffY, diffTh;

   //Update the current positions
   slamPose.position.z = icpPose.position.z;

   //get the difference in icp position since the last update
   diffX = icpPose.position.x - oldICPPose.position.x; 
   diffY = icpPose.position.y - oldICPPose.position.y; 
   double yi, pi, ri;
   double yo, po, ro;
   icpPose.getYPR(yi, pi, ri);
   oldICPPose.getYPR(yo, po, ro);
   diffTh = yi - yo;
   ANGNORM(diffTh);
   //Update the slam position
   double ys, ps, rs;
   slamPose.getYPR(ys, ps, rs);
   ys += diffTh;
   ANGNORM(ys);
   double angleError = ys - yi;
   double cosTh = cos(angleError);
   double sinTh = sin(angleError);
   slamPose.position.x += cosTh * diffX - sinTh * diffY;
   slamPose.position.y += sinTh * diffX + cosTh * diffY;
   slamPose.setYPR(ys, pi, ri);
   //Update the offset in the current local map
   offsetFromParentX += diffX;
   offsetFromParentY += diffY;
   offsetFromParentTh += diffTh;
   ANGNORM(offsetFromParentTh);
   //Need to convert diffX and diffY to be relative to the current local map
   cosTh = cos(-common->startICPTh);
   sinTh = sin(-common->startICPTh);
   common->currentOffsetX += diffX * cosTh - diffY * sinTh;
   common->currentOffsetY += diffX * sinTh + diffY * cosTh;
   common->currentOffsetTh += diffTh;
   ANGNORM(common->currentOffsetTh);

   double cosThG = cos(localMaps[currentLocalMap].currentGlobalPosTh);
   double sinThG = sin(localMaps[currentLocalMap].currentGlobalPosTh);

   //Add the points to the current local map
   int i, j;
   cosTh = cos(common->currentOffsetTh);
   sinTh = sin(common->currentOffsetTh);
   for (i = 0; i < cloud->cloud.size() - 1; ++i) {
      double dist = cloud->cloud[i].x * cloud->cloud[i].x + cloud->cloud[i].y * cloud->cloud[i].y;
      if (dist > LaserMinDist * LaserMinDist && dist < LaserMaxDist * LaserMaxDist &&
         cloud->cloud[i].z + InitHeight > MinAddHeight && cloud->cloud[i].z + InitHeight < MaxAddHeight) {

         double curPointX, curPointY;
         curPointX = cloud->cloud[i].x * cosTh - cloud->cloud[i].y * sinTh + common->currentOffsetX;
         curPointY = cloud->cloud[i].x * sinTh + cloud->cloud[i].y * cosTh + common->currentOffsetY;
         int ogIndex = getLocalOGIndex(curPointX, curPointY);
         if (ogIndex >= 0 && common->localOGCount[ogIndex] < MinObservationCount) {
            common->localOGCount[ogIndex]++;
            common->localOGZ[ogIndex] = std::max(common->localOGZ[ogIndex], cloud->cloud[i].z);
         } else if (ogIndex >= 0 && common->localOGCount[ogIndex] > MinObservationCount) {
            localMaps[currentLocalMap].pointsZ[common->localOG[ogIndex]] = std::max(
               localMaps[currentLocalMap].pointsZ[common->localOG[ogIndex]], cloud->cloud[i].z);
            globalMapHeights[numGlobalPoints + common->localOG[ogIndex]] = 
               localMaps[currentLocalMap].pointsZ[common->localOG[ogIndex]];
         } else if (ogIndex >= 0 && localMaps[currentLocalMap].numPoints < MAX_LOCAL_POINTS) {
            //Add the point to the local map
            common->localOGCount[ogIndex]++;
            int index = localMaps[currentLocalMap].numPoints;
            common->localOG[ogIndex] = index;

            localMaps[currentLocalMap].pointsX[index] = curPointX;
            localMaps[currentLocalMap].pointsY[index] = curPointY;

            double curPointNxtX, curPointNxtY;
            curPointNxtX = cloud->cloud[i + 1].x * cosTh - 
               cloud->cloud[i + 1].y * sinTh + common->currentOffsetX;
            curPointNxtY = cloud->cloud[i + 1].x * sinTh + 
               cloud->cloud[i + 1].y * cosTh + common->currentOffsetY;
            common->pointsNxtX[index] = curPointNxtX;
            common->pointsNxtY[index] = curPointNxtY;

            localMaps[currentLocalMap].pointsZ[index] = std::max(
               common->localOGZ[ogIndex], cloud->cloud[i].z);

            if (curPointX < common->minMapRangeX) {
               common->minMapRangeX = curPointX;
            }
            if (curPointX > common->maxMapRangeX) {
               common->maxMapRangeX = curPointX;
            }
            if (curPointY < common->minMapRangeY) {
               common->minMapRangeY = curPointY;
            }
            if (curPointY > common->maxMapRangeY) {
               common->maxMapRangeY = curPointY;
            }
            //Update the histograms
            double orien = atan2(curPointNxtY - curPointY, curPointNxtX - curPointX);
            orien += M_PI/2;
            if (orien >= M_PI) {
               orien -= 2 * M_PI;
            }
            int orienIndex = (orien + M_PI) * NUM_ORIENTATION_BINS / (2*M_PI);
            localMaps[currentLocalMap].orientationHist[orienIndex]++;
            double mapSize = DimLocalOG * CellSize;
            for(j = 0; j < NUM_ORIENTATION_BINS; j++) {
               double dist = curPointX * common->histCos[j] + 
                                 curPointY * common->histSin[j];
               int projIndex = (dist + mapSize / 2.0f) / 
                                 (mapSize / (double) NUM_PROJECTION_BINS);
               if (projIndex < 0 || projIndex >= NUM_PROJECTION_BINS) {
                  continue;
               }
               double normalX = curPointY - curPointNxtY;
               double normalY = curPointNxtX - curPointX;
               double normalise = sqrt(normalX * normalX + normalY * normalY);
               double weight = normalX / normalise * common->histCos[j] +
                                   normalY / normalise * common->histSin[j];
               localMaps[currentLocalMap].projectionHist[j][projIndex] += weight;
            }
            //Add the point to the global map
            int globalIndex = convertToGlobalPosition(curPointX, curPointY, 
                           currentLocalMap, cosThG, sinThG);
            if (numGlobalPoints + localMaps[currentLocalMap].numPoints < globalMap.size()) {
               globalMap[numGlobalPoints + localMaps[currentLocalMap].numPoints] = globalIndex;
               globalMapHeights[numGlobalPoints + common->localOG[ogIndex]] = 
                  localMaps[currentLocalMap].pointsZ[common->localOG[ogIndex]];
            } else {
               globalMap.push_back(globalIndex);
               globalMapHeights.push_back(localMaps[currentLocalMap].pointsZ[common->localOG[ogIndex]]);
            }
            localMaps[currentLocalMap].numPoints++;
         }

      }
   }

   double temp = sqrt(offsetFromParentX * offsetFromParentX +
         offsetFromParentY * offsetFromParentY);
   if (temp >= LocalMapDistance) {
      cout << "Creating a new local map " << currentLocalMap << endl;
      finishMap(angleError, yi, icpPose);
   }

   oldICPPose = icpPose;

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms " << endl;
   }

   finishedSetup = true;
}

void GraphSlamCPU::finishMap(double angleError, double icpTh, Pose icpPose) {

{{ Lock lock(masterLockSmith);

   int numLocalPoints = localMaps[currentLocalMap].numPoints;

   int oldLocalMap = currentLocalMap;
   int numOldMapPoints = numLocalPoints;

   if (parentLocalMap >= 0) {
      getHessianMatch(-1);
      prepareLocalMap();
      bool needOptimisation = false;
      if (combineMode > 0) {
         int combineIndex = common->combineIndex;
         if (combineIndex >= 0) {
            cout << "****Combining node: " << currentLocalMap << " with " << combineIndex << endl;
            int nI = 0;
            int matchSuccess = 0;
            while (matchSuccess == 0) {
               alignICP(combineIndex, 0);
               calculateICPMatrix(0);
               matchSuccess = common->matchSuccess;
               nI++;
            }
            cout << "Number of iterations: " << nI << endl;
            if (matchSuccess == 1) {
               cout << "Combining map succeeded" << endl;
               int numOtherGlobalPoints = 0;
               int i;
               for (i = 0; i < combineIndex; i++) {
                  numOtherGlobalPoints += localMaps[i].numPoints;
               }
               int temp = localMaps[combineIndex].numPoints;
               combineNodes(angleError, numOtherGlobalPoints);
               if (combineMode == 1) {
                  needOptimisation = true;
                  numConstraints++;
               }
               numLocalPoints = localMaps[combineIndex].numPoints - temp;

               //Need to update snaps and slam history to the combined local map
               double alignX = common->potentialMatchX[0];
               double alignY = common->potentialMatchY[0];
               double alignTh = common->potentialMatchTh[0];
               fixSnapPositions(combineIndex, alignX, alignY, alignTh);
               fixSlamHistoryPositions(combineIndex, alignX, alignY, alignTh);

               currentLocalMap = combineIndex;
               resetMap = true;
            } else {
               cout << "Combing map alignment failed" << matchSuccess << endl;
               combineMode = 0;
            }
         } else {
            combineMode = 0;
         }
      }
      if (!combineMode) {
         nextLocalMap++;
         findPotentialMatches();
         numConstraints++;
         if (common->numPotentialMatches > 0 && !resetMap) {
            cout << "************" << endl;
            cout << "Potential Match found with maps:" ;
            for (int h = 0; h < common->numPotentialMatches; h++) {
                cout << common->potentialMatches[h] << " ";
            }
            cout << endl << "************" << endl;

            int i;
            for (i = 0; i < common->numPotentialMatches; i++) {
               int matchSuccess = 0;
               int c = 0;
               while (matchSuccess == 0) {
                  alignICP(common->potentialMatches[i], i);
                  calculateICPMatrix(i);
                  matchSuccess = common->matchSuccess;
                  c++;
               }
               cout << "Number of iterations in alignment: " << c << " " << matchSuccess << endl;
               if (matchSuccess == 1) {
                  getHessianMatch(numConstraints);
                  finaliseInformationMatrix();
                  numConstraints++;
                  needOptimisation = true;
                  //Only add one loop closing constraint
                  break;
               } else {
                  cout << "Alignment failed" << endl;
               }
            }
         }
      }
      if (needOptimisation) {
         cout << "Optimising graph" << endl;
         double posBeforeX = localMaps[currentLocalMap].currentGlobalPosX;
         double posBeforeY = localMaps[currentLocalMap].currentGlobalPosY;
         double posBeforeTh = localMaps[currentLocalMap].currentGlobalPosTh;
         cout << "pos before: " << posBeforeX << " " << posBeforeY << " " << posBeforeTh << endl;
         for (int numIterations = 1; numIterations < 6; numIterations++) {
            getGlobalHessianMatrix();
            calculateOptimisationChange(numIterations);
            updateGlobalPositions();
         }
         updateGlobalMap();
         resetMap = true;
         historySlamPoses.resize(0);

         double posChangeX = localMaps[currentLocalMap].currentGlobalPosX - posBeforeX;
         double posChangeY = localMaps[currentLocalMap].currentGlobalPosY - posBeforeY;
         double posChangeTh = localMaps[currentLocalMap].currentGlobalPosTh - posBeforeTh;
         cout << "pos change: " << posChangeX << " " << posChangeY << " " << posChangeTh << endl;

         slamPose.position.x += posChangeX;
         slamPose.position.y += posChangeY;
         double ys, ps, rs;
         slamPose.getYPR(ys, ps, rs);
         ys += posChangeTh;
         ANGNORM(ys);
         slamPose.setYPR(ys, ps, rs);

         if (LocalMapCombine) {
            combineMode++;
         }
      }
   } else {
      prepareLocalMap();
      nextLocalMap++;
   }
   createNewLocalMap(oldLocalMap, nextLocalMap, currentLocalMap, angleError, icpTh);
   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   currentLocalMapICPPose = icpPose;
   numGlobalPoints += numLocalPoints;
   parentLocalMap = currentLocalMap;
   currentLocalMap = nextLocalMap;

}}

}

GraphSlamCPU::~GraphSlamCPU() {
}

void GraphSlamCPU::getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices) {

   int x, y, mapNum;
   crosbot::LocalMap::Cell *cellsP;
   if (resetMap) {
      resetMap = false;
      //cout << "Reseting map" << endl;
      lastDrawnGlobalPoints = 0;
      for(mapNum = 0; mapNum < curMap.size(); mapNum++) {
         for(y = 0; y < curMap[mapNum]->height; y++) {
            cellsP = &(curMap[mapNum]->cells[y][0]);
            for(x = 0; x< curMap[mapNum]->width; x++) {
               cellsP->current = false;
               cellsP->hits = 0;
               cellsP++;
            }
         }
      }
   }
   int numLocalPoints = localMaps[currentLocalMap].numPoints;
   //cout << numLocalPoints << " " << numGlobalPoints << " " << lastDrawnGlobalPoints << endl;
   if (numGlobalPoints + numLocalPoints > lastDrawnGlobalPoints) {
      double off = (DimGlobalOG * CellSize) / 2.0 
                 - CellSize / 2.0;
      x = numGlobalPoints - MAX_LOCAL_POINTS > lastDrawnGlobalPoints ?
           lastDrawnGlobalPoints : numGlobalPoints - MAX_LOCAL_POINTS;
      if (x < 0) {
         x = 0;
      }
      for(; x < numGlobalPoints + numLocalPoints; x++) {
         int index = globalMap[x];
         int yi = index / DimGlobalOG;
         int xi = index % DimGlobalOG;
         for(mapNum = 0; mapNum < curMap.size(); mapNum++) {
            cellsP = &(curMap[mapNum]->cells[yi][xi]);
            if (globalMapHeights[x] >= mapSlices[mapNum]) {
               if (x < numGlobalPoints) {
                  cellsP->current = false;
                  cellsP->hits = curMap[mapNum]->maxHits;
               } else {
                  cellsP->current = true;
                  cellsP->hits = curMap[mapNum]->maxHits;
               }
            }
         }
      }
      lastDrawnGlobalPoints = numGlobalPoints + numLocalPoints;
   }
}

void GraphSlamCPU::getGlobalMapPosition(int mapIndex, double& gx, double& gy,
      double& gth) {

   gx = localMaps[mapIndex].currentGlobalPosX;
   gy = localMaps[mapIndex].currentGlobalPosY;
   gth = localMaps[mapIndex].currentGlobalPosTh;
}

void GraphSlamCPU::clearMap(int mapIndex) {
   localMaps[mapIndex].numPoints = 0;
   memset(localMaps[mapIndex].orientationHist, 0, sizeof(double) * NUM_ORIENTATION_BINS);
   memset(localMaps[mapIndex].entropyHist, 0, sizeof(double) * NUM_ORIENTATION_BINS);
   memset(localMaps[mapIndex].projectionHist, 0, sizeof(double) * NUM_ORIENTATION_BINS
         * NUM_PROJECTION_BINS);
   memset(localMaps[mapIndex].parentInfo, 0, sizeof(double) * 9);
}

int GraphSlamCPU::getLocalOGIndex(double x, double y) {
   int i,j;
   float off = (DimLocalOG * CellSize) / 2.0f;
   i = (x + off) / CellSize;
   j = (y + off) / CellSize;
   //check to see if the point fits inside the occupancy grid
   if (i >= 0 && i < DimLocalOG && j >= 0 && j < DimLocalOG) {
      return j * DimLocalOG + i;
   } else {
      return -1;
   }
}

int GraphSlamCPU::convertToGlobalPosition(double x, double y, int mapIndex, double cosTh, double sinTh) {
   double gx = (x * cosTh - y * sinTh) + localMaps[mapIndex].currentGlobalPosX;
   double gy = (x * sinTh + y * cosTh) + localMaps[mapIndex].currentGlobalPosY;
   double off = DimGlobalOG * CellSize / 2.0;
   int i = (gx + off) / CellSize;
   int j = (gy + off) / CellSize;
   return j * DimGlobalOG + i;
}

void GraphSlamCPU::createNewLocalMap(int oldLocalMap, int newLocalMap, int parentLocalMap, double angleError, double icpTh) {
   
   for(int i = 0; i < DimLocalOG * DimLocalOG; i++) {
      common->localOG[i] = -1;
      common->localOGCount[i] = 0;
      common->localOGZ[i] = MinAddHeight;
   }
   if (newLocalMap == localMaps.size()) {
      LocalMap temp;
      localMaps.push_back(temp);
   }
   clearMap(newLocalMap);
   //Set the map centre for the old local map. This is normally done in the prepare
   //local map kernel, but it is not called for the first local map
   if (oldLocalMap == 0) {
      localMaps[0].mapCentreX = (common->minMapRangeX + common->maxMapRangeX) / 2.0;
      localMaps[0].mapCentreY = (common->minMapRangeY + common->maxMapRangeY) / 2.0;
   }
   localMaps[oldLocalMap].robotMapCentreX = common->currentOffsetX/2.0;
   localMaps[oldLocalMap].robotMapCentreY = common->currentOffsetY/2.0;
   localMaps[newLocalMap].indexParentNode = parentLocalMap;
   localMaps[newLocalMap].treeLevel = localMaps[parentLocalMap].treeLevel + 1;
   common->minMapRangeX = INFINITY;
   common->maxMapRangeX = 0;
   common->minMapRangeY = INFINITY;
   common->maxMapRangeY = 0;
   common->numPotentialMatches = 0;
   common->startICPTh = icpTh;

   double cosTh = cos(angleError);
   double sinTh = sin(angleError);
   double tempX, tempY;
   tempX = cosTh * offsetFromParentX - sinTh * offsetFromParentY;
   tempY = sinTh * offsetFromParentX + cosTh * offsetFromParentY;
   localMaps[newLocalMap].currentGlobalPosX = localMaps[parentLocalMap].currentGlobalPosX
      + tempX;
   localMaps[newLocalMap].currentGlobalPosY = localMaps[parentLocalMap].currentGlobalPosY
      + tempY;
   localMaps[newLocalMap].currentGlobalPosTh = localMaps[parentLocalMap].currentGlobalPosTh
      + offsetFromParentTh;
   ANGNORM(localMaps[newLocalMap].currentGlobalPosTh);

   //Make the offset relative to the parent instead of the global coord system
   cosTh = cos(-localMaps[parentLocalMap].currentGlobalPosTh);
   sinTh = sin(-localMaps[parentLocalMap].currentGlobalPosTh);
   localMaps[newLocalMap].parentOffsetX = cosTh * tempX - sinTh * tempY;
   localMaps[newLocalMap].parentOffsetY = sinTh * tempX + cosTh * tempY;
   localMaps[newLocalMap].parentOffsetTh = offsetFromParentTh;

   common->currentOffsetX = 0;
   common->currentOffsetY = 0;
   common->currentOffsetTh = 0;

}

void GraphSlamCPU::getHessianMatch(int constraintIndex) {
   int i,j;
   common->infoCount = 0;
   double info[3][3];
   for (j = 0; j < 3; j++) {
      for (i = 0; i < 3; i++) {
         info[j][i] = 0;
      }
   }
   int otherMap;
   double offsetX;
   double offsetY;
   double offsetTh;
   if (constraintIndex < 0) {
      offsetX = localMaps[currentLocalMap].parentOffsetX;
      offsetY = localMaps[currentLocalMap].parentOffsetY;
      offsetTh = localMaps[currentLocalMap].parentOffsetTh;
      otherMap = localMaps[currentLocalMap].indexParentNode;
   } else {
      int cIndex = common->constraintIndex[constraintIndex];
      offsetX = common->loopConstraintXDisp[cIndex];
      offsetY = common->loopConstraintYDisp[cIndex];
      offsetTh = common->loopConstraintThetaDisp[cIndex];
      otherMap = common->loopConstraintI[cIndex];
   }
   double cosThN = cos(-offsetTh);
   double sinThN = sin(-offsetTh);
   double cosTh = cos(offsetTh);
   double sinTh = sin(offsetTh);
   for(int index = 0; index < localMaps[otherMap].numPoints; index++) {
      
      int matchIndex = -1;
      double transformedPointX;
      double transformedPointY;
      //Transform the point from a previous local map into the reference frame of the
      //current local map so that it can be matched to the occupancy grid
      convertReferenceFrame(localMaps[otherMap].pointsX[index], 
                  localMaps[otherMap].pointsY[index], offsetX, offsetY, cosThN, sinThN, 
                  &transformedPointX, &transformedPointY);
      matchIndex = findMatchingPoint(transformedPointX, transformedPointY, 1);
      
      if (matchIndex >= 0) {

         double mapGradX;
         double mapGradY;
         double x,y;
         mapGradX = common->pointsNxtX[matchIndex] - localMaps[currentLocalMap].pointsX[matchIndex];
         mapGradY = common->pointsNxtY[matchIndex] - localMaps[currentLocalMap].pointsY[matchIndex];
         double length = sqrt(mapGradX * mapGradX + mapGradY * mapGradY);
         mapGradX /= length;
         mapGradY /= length;
         if (length > 0) {
            x = -sinTh * transformedPointX - cosTh * transformedPointY;
            y = cosTh * transformedPointX - sinTh * transformedPointY;

            //normalise the movement theta differential???
            /*length = sqrt(transformedPoint.x * transformedPoint.x + transformedPoint.y *
                  transformedPoint.y);
            x /= length;
            y /= length;*/

            double temp = x * mapGradX + y * mapGradY;
            info[0][0] += mapGradX * mapGradX;
            info[0][1] += mapGradX * mapGradY;
            info[0][2] += mapGradX * temp;
            info[1][0] += mapGradX * mapGradY;
            info[1][1] += mapGradY * mapGradY;
            info[1][2] += mapGradY * temp;
            info[2][0] += mapGradX * temp;
            info[2][1] += mapGradY * temp;
            info[2][2] += temp * temp;
            common->infoCount++;
         }
      }   
   }
   for (j = 0; j < 3; j++) {
      for (i = 0; i < 3; i++) {
         if (constraintIndex < 0) {
            localMaps[currentLocalMap].parentInfo[j][i] = info[j][i];
         } else {
            int cIndex = common->constraintIndex[constraintIndex];
            common->loopConstraintInfo[cIndex][j][i] = info[j][i];
         }
      }
   }

}

void GraphSlamCPU::convertReferenceFrame(double pX, double pY, double offsetX, double offsetY,
         double cosTh, double sinTh, double *pointX, double *pointY) {
   double tempX;
   double tempY;
   tempX = pX - offsetX;
   tempY = pY - offsetY;
   *pointX = tempX * cosTh - tempY * sinTh;
   *pointY = tempX * sinTh + tempY * cosTh;
}

int GraphSlamCPU::findMatchingPoint(double pointX, double pointY, int searchFactor) {
   int ogIndex = getLocalOGIndex(pointX, pointY);
   if (ogIndex >= 0) {
      int ogX = ogIndex % DimLocalOG;
      int ogY = ogIndex / DimLocalOG;
      int i,j;
      double minWeight = INFINITY;
      int minOGIndex = -1;
      int searchSize = SearchSize * searchFactor;
      for (i = ogX - searchSize; i <= ogX + searchSize; i++) {
         for (j = ogY - searchSize; j <= ogY + searchSize; j++) {
            if (i >= 0 && i < DimLocalOG && j >= 0 && j < DimLocalOG) {
               ogIndex = j * DimLocalOG + i;
               if (common->localOG[ogIndex] >= 0) {
                  double ogPointX;
                  double ogPointY;
                  ogPointX = localMaps[currentLocalMap].pointsX[common->localOG[ogIndex]];
                  ogPointY = localMaps[currentLocalMap].pointsY[common->localOG[ogIndex]];
                  double min = getMetricValue(pointX, pointY, ogPointX, ogPointY);
                  if (min < minWeight) {
                     minWeight = min;
                     minOGIndex = common->localOG[ogIndex];
                  }
               }
            }
         }
      }
      return minOGIndex;
   } else {
      //The point does not fit in the current local map, so can't be used for alignment
      return -1;
   }
}

double GraphSlamCPU::getMetricValue(double pointX, double pointY,
        double ogPointX, double ogPointY) {
   double dx = ogPointX - pointX;
   double dy = ogPointY - pointY;
   if (dx * dx + dy * dy > MaxAlignDistance) {
      return INFINITY;
   }
   return dx * dx + dy * dy - 
          (SQ(dx * pointY - dy * pointX)) / 
          (pointX * pointX + pointY * pointY + LValue);
}


void GraphSlamCPU::prepareLocalMap() {
   //Finalise the histograms
   int i;
   for(int index = 0; index < NUM_ORIENTATION_BINS; index++) {
      double sum = 0;
      double entropySum = 0;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         sum += localMaps[currentLocalMap].projectionHist[index][i] * 
                localMaps[currentLocalMap].projectionHist[index][i];
      }
      sum = sqrt(sum);
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         localMaps[currentLocalMap].projectionHist[index][i] /= sum;
         entropySum += fabs(localMaps[currentLocalMap].projectionHist[index][i]);
      }
      sum = 0;
      double temp;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         temp = fabs(localMaps[currentLocalMap].projectionHist[index][i]) / entropySum;
         if (temp > 0) {
            sum += temp * std::log(temp);
         }
      }
      sum *= -1;
      sum = pow(2, sum);
      localMaps[currentLocalMap].entropyHist[index] = sum;
   }

   double max = 0;
   double finalSum = 0;
   double orienSum = 0;
   for (i = 0; i < NUM_ORIENTATION_BINS; i++) {
      if (localMaps[currentLocalMap].entropyHist[i] > max) {
         max = localMaps[currentLocalMap].entropyHist[i];
      }
   }
   for(i = 0; i < NUM_ORIENTATION_BINS; i++) {
      finalSum += (localMaps[currentLocalMap].entropyHist[i] - max) * 
                  (localMaps[currentLocalMap].entropyHist[i] - max);
      orienSum += localMaps[currentLocalMap].orientationHist[i] *
                  localMaps[currentLocalMap].orientationHist[i];
   }
   finalSum = sqrt(finalSum);
   orienSum = sqrt(orienSum);
   
   for(int index = 0; index < NUM_ORIENTATION_BINS; index++) {
      localMaps[currentLocalMap].entropyHist[index] = 
               -(localMaps[currentLocalMap].entropyHist[index] - max) / finalSum;
      localMaps[currentLocalMap].orientationHist[index] /= orienSum;
   }

   //Fix up the hessian matrixpreviously calculated
   double a[3][3];
   double b[3][3];
   double c[3][3];
   if (currentLocalMap > 0) {
      //Need to rotate the hessian matrix as it was calculated in the opposite displacement
      //to the actual move - rotate info matrix b by RbR^T where R is the homogeneous 
      //rotation matrix of the angle of the last move
      
      int x,y;

      localMaps[currentLocalMap].mapCentreX = 
         (common->minMapRangeX + common->maxMapRangeX) / 2.0;
      localMaps[currentLocalMap].mapCentreY = 
         (common->minMapRangeY + common->maxMapRangeY) / 2.0;

      double cosTh = cos(- localMaps[currentLocalMap].parentOffsetTh);
      double sinTh = sin(- localMaps[currentLocalMap].parentOffsetTh);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;

      if (common->infoCount < InformationScaleFactor) {
         common->infoCount = InformationScaleFactor;
      }

      for (y = 0; y < 3; y++) {
         for(x = 0; x < 3; x++) {
            //Adjust the parent info matrix to be relative to the parent instead
            //of relative to the current node
            b[y][x] = localMaps[currentLocalMap].parentInfo[y][x] / 
                        ((double)common->infoCount / (double)InformationScaleFactor);
         }
      }
      mult3x3Matrix(a, b, c);

      a[1][0] *= -1;
      a[0][1] *= -1;
      
      mult3x3Matrix(c, a, b);
      for (y = 0; y < 3; y++) {
         for(x = 0; x < 3; x++) {
            localMaps[currentLocalMap].parentInfo[y][x] = b[y][x];
         }
      }
      
      //Now calculate the global covar matrix for the node in its
      //current position - b currently holds the parent info matrix
      cosTh = cos(localMaps[localMaps[currentLocalMap].indexParentNode].currentGlobalPosTh);
      sinTh = sin(localMaps[localMaps[currentLocalMap].indexParentNode].currentGlobalPosTh);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
      
      mult3x3Matrix(a, b, c);
      //transpose the parent rotation matrix
      double temp = a[1][0];
      a[1][0] = a[0][1];
      a[0][1] = temp;
      mult3x3Matrix(c, a, b);
      //now invert it to get the covar matrix
      for (y = 0; y < 3; y++) {
         for(x = 0; x < 3; x++) {
            localMaps[currentLocalMap].globalCovar[y][x] = b[y][x];
         }
      }
      invert3x3Matrix(localMaps[currentLocalMap].globalCovar, a);
      for (y = 0; y < 3; y++) {
         for(x = 0; x < 3; x++) {
            localMaps[currentLocalMap].globalCovar[y][x] = a[y][x];
         }
      }
      //Fiddle with the covar to make it nicer
      covarFiddle(localMaps[currentLocalMap].globalCovar);
      
      common->infoCount = 0;

      //Start combining nodes
      int combined = 0;
      if (common->combineIndex >= 0) {
         /*float2 mapCurPos = convertToGlobalCoord(localMaps[currentMap].mapCentre.x, 
                              localMaps[currentMap].mapCentre.y, 
                              localMaps[currentMap].currentGlobalPos);*/
         double mapCurPosX;
         double mapCurPosY;
         convertToGlobalCoord(localMaps[currentLocalMap].robotMapCentreX, 
                              localMaps[currentLocalMap].robotMapCentreY, 
                              localMaps[currentLocalMap].currentGlobalPosX,
                              localMaps[currentLocalMap].currentGlobalPosY,
                              localMaps[currentLocalMap].currentGlobalPosTh,
                              &mapCurPosX, &mapCurPosY);
         for(i = -1; i <= nextLocalMap && !combined; i++) {
            int pIndex = -1;
            if (i == -1) {
               pIndex = localMaps[common->combineIndex].indexParentNode;
            } else if (i < nextLocalMap && localMaps[i].indexParentNode == common->combineIndex) {
               pIndex = i;
            } else if (i == nextLocalMap) {
               pIndex = common->combineIndex;
            }
            if (pIndex >= 0) {
               double mapOtherPosX;
               double mapOtherPosY;
               convertToGlobalCoord(localMaps[pIndex].robotMapCentreX, 
                     localMaps[pIndex].robotMapCentreY, 
                     localMaps[pIndex].currentGlobalPosX,
                     localMaps[pIndex].currentGlobalPosY,
                     localMaps[pIndex].currentGlobalPosTh,
                     &mapOtherPosX, &mapOtherPosY);
               if (fabs(mapOtherPosX - mapCurPosX) < LocalMapDistance/* * 1.5f */&&
                        fabs(mapOtherPosY - mapCurPosY) < LocalMapDistance/* * 1.5f*/) {
                  combined = 1;
                  common->combineIndex = pIndex;
                  double tempX = localMaps[currentLocalMap].currentGlobalPosX - 
                        localMaps[pIndex].currentGlobalPosX;
                  double tempY = localMaps[currentLocalMap].currentGlobalPosY - 
                        localMaps[pIndex].currentGlobalPosY;
                  double tempTh = localMaps[currentLocalMap].currentGlobalPosTh - 
                        localMaps[pIndex].currentGlobalPosTh;
                  cosTh = cos(localMaps[currentLocalMap].currentGlobalPosTh);
                  sinTh = cos(localMaps[currentLocalMap].currentGlobalPosTh);
                  common->potentialMatchX[0] = tempX * cosTh - tempY * sinTh;
                  common->potentialMatchY[0] = tempX * sinTh + tempY * cosTh;
                  common->potentialMatchTh[0] = tempTh;
                  ANGNORM(common->potentialMatchTh[0]);
               }
            }
         }
         if (!combined) {
            common->combineIndex = -1;
            common->combineMode = 0;
         }
      }
      if (combined == 0) {
         int cIndex = common->numConstraints;
         common->numConstraints++;
         common->constraintType[cIndex] = 0;
         common->constraintIndex[cIndex] = currentLocalMap;
      }
   }
}

void GraphSlamCPU::mult3x3Matrix(double a[3][3], double b[3][3], double res[3][3]) {
   for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
         res[y][x] = a[y][0] * b[0][x] + a[y][1] * b[1][x]  + a[y][2] * b[2][x];
      }
   }
}

void GraphSlamCPU::invert3x3Matrix(double m[3][3], double res[3][3]) {
   double det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
              m[0][1] * (m[2][2] * m[1][0] - m[1][2] * m[2][0]) +
              m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
   for (int y= 0; y < 3; y++) {
      for(int x = 0; x< 3; x++) {
         int minx = min((x + 1) % 3, (x + 2) % 3);
         int maxx = max((x + 1) % 3, (x + 2) % 3);
         int miny = min((y + 1) % 3, (y + 2) % 3);
         int maxy = max((y + 1) % 3, (y + 2) % 3);

         double temp = m[miny][minx] * m[maxy][maxx] - m[miny][maxx] * m[maxy][minx];
         if ((x == 1 || y == 1) && !(x == 1 && y == 1)) {
            temp *= -1;
         }
         if (det != 0) {
            res[x][y] = temp * 1/det;
         } else {
            res[x][y] = 0.0001;
         }
      }
   }
}

void GraphSlamCPU::covarFiddle(double m[3][3]) {

   double max = MaxCovar;
   for(int y = 0; y < 3; y++) {
      if (m[y][y] < 0) {
         m[y][y] *= -1;
      }
      if (m[y][y] > max) {
         max = m[y][y];
      }
   }

   for(int y= 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
         if(max > MaxCovar) {
            m[y][x] = m[y][x] * MaxCovar/max;
         }
      }
   }
}

void GraphSlamCPU::convertToGlobalCoord(double x, double y, double localPosX,
      double localPosY, double localPosTh, double *resX, double *resY) {
   double cosTh = cos(localPosTh);
   double sinTh = sin(localPosTh);
   *resX = (x * cosTh - y * sinTh) + localPosX;
   *resY = (x * sinTh + y * cosTh) + localPosY;
}

void GraphSlamCPU::findPotentialMatches() {
   double totalCovar[3][3];
   int tempCorrIndex;
   int peaks[NUM_ORIENTATION_BINS];
   double orienCorr[NUM_ORIENTATION_BINS];
   double entCorr[NUM_ORIENTATION_BINS];
   int numPeaks = 0;
   int maxPeak;

   for(int globalWarp = 0; globalWarp < nextLocalMap; globalWarp++) {
      if (globalWarp != currentLocalMap && globalWarp != localMaps[currentLocalMap].indexParentNode) {
         numPeaks = 0;
         int parentIndex = -1;
         int x,y;

         int minLevel = min(localMaps[currentLocalMap].treeLevel, localMaps[globalWarp].treeLevel);

         for (y = 0; y < 3; y++) {
            for(x = 0; x < 3; x++) {
               int mapIndex = currentLocalMap;
               totalCovar[y][x] = 0;
               while (localMaps[mapIndex].treeLevel > minLevel) {
                  totalCovar[y][x] += localMaps[mapIndex].globalCovar[y][x];
                  mapIndex = localMaps[mapIndex].indexParentNode;
               }
               int mapIndexGlobal = globalWarp;
               while (localMaps[mapIndexGlobal].treeLevel > minLevel) {
                  totalCovar[y][x] += localMaps[mapIndexGlobal].globalCovar[y][x];
                  mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
               }
               while (mapIndex != mapIndexGlobal) {
                  totalCovar[y][x] += localMaps[mapIndex].globalCovar[y][x];
                  totalCovar[y][x] += localMaps[mapIndexGlobal].globalCovar[y][x];
                  mapIndex = localMaps[mapIndex].indexParentNode;
                  mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
               }
               parentIndex = mapIndex;
            }
         }
      
         //Now have calculated the total covar. Need to now see if a potential match, and 
         //if so, perform a histogram correlation
         double mapCurPosX;
         double mapCurPosY;
         double mapOtherPosX;
         double mapOtherPosY;
         convertToGlobalCoord(localMaps[currentLocalMap].mapCentreX, 
                           localMaps[currentLocalMap].mapCentreY, 
                           localMaps[currentLocalMap].currentGlobalPosX,
                           localMaps[currentLocalMap].currentGlobalPosY,
                           localMaps[currentLocalMap].currentGlobalPosTh,
                           &mapCurPosX, &mapCurPosY);
         convertToGlobalCoord(localMaps[globalWarp].mapCentreX, 
                           localMaps[globalWarp].mapCentreY, 
                           localMaps[globalWarp].currentGlobalPosX,
                           localMaps[globalWarp].currentGlobalPosY,
                           localMaps[globalWarp].currentGlobalPosTh,
                           &mapOtherPosX, &mapOtherPosY);
         /*cout << "covar of map " << currentLocalMap << " to map " << globalWarp <<
            " is " << totalCovar[0][0] << " " << totalCovar[1][1] << " pos " <<
            mapCurPosX << " " << mapCurPosY << " " << mapOtherPosX << " " << mapOtherPosY << endl;*/
         if (fabs(mapCurPosX - mapOtherPosX) < totalCovar[0][0] &&
               fabs(mapCurPosY - mapOtherPosY) < totalCovar[1][1]) {
            //In the right area for a match, so do histogram correlation
            int i, j;
            //Perform the correlations for the entropy and orientation histograms
            for (i = 0; i < NUM_ORIENTATION_BINS; i++) {
               orienCorr[i] = 0;
               entCorr[i] = 0;
               for(j = 0; j < NUM_ORIENTATION_BINS; j++) {
                 int temp = i + j;
                  if(temp >= NUM_ORIENTATION_BINS) {
                     temp = temp - NUM_ORIENTATION_BINS;
                  }
                  orienCorr[i] += localMaps[currentLocalMap].orientationHist[j] * 
                                            localMaps[globalWarp].orientationHist[temp];
                  entCorr[i] += localMaps[currentLocalMap].entropyHist[j] *
                                             localMaps[globalWarp].entropyHist[temp];
               }
            }
            //Find the peaks in the histograms
            for (i = 0; i < NUM_ORIENTATION_BINS; i++) {
               bool isPeak = findIfPeak(orienCorr, i);
               if (!isPeak) {
                  isPeak = findIfPeak(entCorr, i);
               }
               if (isPeak) {
                  peaks[numPeaks] = i;
                  numPeaks++;
               }
            }
            //Find the index of the largest peak in the orientation histogram
            double max = 0;
            int maxIndex = -1;
            for (i = 0; i < numPeaks; i++) {
               if (orienCorr[peaks[i]] > max) {
                  max = orienCorr[peaks[i]];
                  maxIndex = peaks[i];
               }
            }
            maxPeak = maxIndex;
            //Go through the peaks and perform the projection correlations as well
            double maxCorrScore = 0;
            int maxOrien = 0;
            int maxX = 0;
            int maxY = 0;

            for (i = 0; i < numPeaks; i++) {
               double maxValue;
               int maxIndex;
               double maxValue90;
               int maxIndex90;
               maxValue = correlateProjection(localMaps[currentLocalMap].projectionHist,
                   localMaps[globalWarp].projectionHist, maxPeak,
                   peaks[i], &maxIndex);
               maxValue90 = correlateProjection(localMaps[currentLocalMap].projectionHist,
                   localMaps[globalWarp].projectionHist, (maxPeak + 
                   NUM_ORIENTATION_BINS/4)%NUM_ORIENTATION_BINS,
                   peaks[i], &maxIndex90);
               double score = maxValue + maxValue90 + orienCorr[peaks[i]] +
                         entCorr[peaks[i]];
               if (score > maxCorrScore) {
                  maxCorrScore = score;
                  maxOrien = peaks[i];
                  maxX = maxIndex;
                  maxY = maxIndex90;
               }            
            }
            cout << "Correlation score of map " << globalWarp << " is " << maxCorrScore << endl;
            if (maxCorrScore > CorrelationThreshold) {
               int res = common->numPotentialMatches;
               common->numPotentialMatches++;
               common->potentialMatches[res] = globalWarp;
               //convert to x,y,theta for the next stage of alignment
               double mapSize = DimLocalOG * CellSize;
               double tempX = (maxX - NUM_PROJECTION_BINS) * mapSize / (float) NUM_PROJECTION_BINS;
               double tempY = (maxY - NUM_PROJECTION_BINS) * mapSize / (float) NUM_PROJECTION_BINS;
               double maxTheta = (2 * M_PI * maxPeak) / NUM_ORIENTATION_BINS - 
                             (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
                              / (float) (2 * NUM_ORIENTATION_BINS));
               double cosTh = cos(maxTheta);
               double sinTh = sin(maxTheta);
               common->potentialMatchX[res] = (tempX + sinTh * tempY / cosTh) / 
                                              (cosTh + sinTh * sinTh / cosTh);
               common->potentialMatchY[res] = (cosTh * common->potentialMatchX[res] - tempX) / sinTh;
               //This is what it should be:
               //common->potentialMatchTheta[res] = (2 * M_PI * maxOrien) / NUM_ORIENTATION_BINS - 
               //                                    (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
               //                                    / (float) (2 * NUM_ORIENTATION_BINS));
               //This is what works:
               common->potentialMatchTh[res] = ((2 * M_PI * maxOrien) / NUM_ORIENTATION_BINS - 
                                                   (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
                                                   / (float) (2 * NUM_ORIENTATION_BINS)) + M_PI) * -1;
               common->potentialMatchX[res] *= -1;
               common->potentialMatchY[res] *= -1;
               common->potentialMatchParent[res] = parentIndex;
               ANGNORM(common->potentialMatchTh[res]);
            }
         }
      }
   }
}

bool GraphSlamCPU::findIfPeak(double *corr, int i) {
   bool isPeak = true;
   int j;
   if (corr[i] > 0.8) {
      for(j = -2; j < 3; j++) {
         int adjust = (i + j) % NUM_ORIENTATION_BINS;
         if (adjust < 0) {
            adjust = NUM_ORIENTATION_BINS + adjust;
         }
         if (corr[adjust] > corr[i]) {
            isPeak = false;
         }
      }
   } else {
      isPeak = false;
   }
   return isPeak;
}

double GraphSlamCPU::correlateProjection(double proj1[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS],
         double proj2[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS], int startIndex,
         int offset, int *maxIndex) {
   int sec = (startIndex + offset) % NUM_ORIENTATION_BINS;
   int off;
   double out;

   double maxCorr = -INFINITY;
   *maxIndex = -1;
   int i, j;
   int index;

   for (i = 0; i < NUM_PROJECTION_BINS * 2; i++) {
      off = i - NUM_PROJECTION_BINS;
      out = 0;
      for (j = 0; j < NUM_PROJECTION_BINS; j++) {
         index = off + j;
         if (index >= 0 && index < NUM_PROJECTION_BINS) {
            out += proj1[startIndex][j] * proj2[sec][index];
         }
      }
      if (out > maxCorr) {
         maxCorr = out;
         *maxIndex = i;
      }
   }
   return maxCorr;
}

void GraphSlamCPU::alignICP(int otherMap, int mIndex) {
   int x,y;
   for(y = 0; y < 3; y++) {
      for(x = 0; x < 3; x++) {
         common->A[y][x] = 0;
      }
      common->B[y] = 0;
   }
   common->goodCount = 0;

   double offsetX = common->potentialMatchX[mIndex];
   double offsetY = common->potentialMatchY[mIndex];
   double offsetTh = common->potentialMatchTh[mIndex];
   double cosTh = cos(offsetTh);
   double sinTh = sin(offsetTh);
   //cout << offsetX << " " << offsetY << " " << offsetTh << endl;
   for (int index = 0; index < localMaps[otherMap].numPoints; index++) {
      int searchFactor = 1;
      if (common->numIterations < 2) {
         searchFactor = 3;
      } else if (common->numIterations < 4) {
         searchFactor = 2;
      }
      //Transform the points. Offsets are currently relative to the new map, so shouldn't use
      //convertReferenceFrame
      double transformedPointX;
      double transformedPointY;
      double pX = localMaps[otherMap].pointsX[index];
      double pY = localMaps[otherMap].pointsY[index];
      transformedPointX = (pX * cosTh - pY * sinTh) + offsetX;
      transformedPointY = (pX * sinTh + pY * cosTh) + offsetY;
      //TODO: looks at the number of beighbouring cells examined to find a matching point
      int matchIndex = findMatchingPoint(transformedPointX, transformedPointY, searchFactor);
      //If found a matching point, calculate its contribution to the match
      if (matchIndex >= 0) {
         double x12, y12, k, temp;
         //lPoint is the point on the map ie. mapPoint
         //point is the laser point not in the map ie. transformedPoint
         double mapPointX = localMaps[currentLocalMap].pointsX[matchIndex];
         double mapPointY = localMaps[currentLocalMap].pointsY[matchIndex];
         //Doing this as per UseSimpleH in pogmbicp code as no 3d yet
         x12 = mapPointX * mapPointX;
         y12 = mapPointY * mapPointY;
         k = 1.0 / (x12 + y12 + LValue * LValue);
         temp = 1.0 - y12 * k;
         common->A[0][0] += temp;
         temp = mapPointX * mapPointY * k;
         common->A[0][1] += temp;
         temp = - transformedPointY + mapPointY * (transformedPointY * mapPointY +
                transformedPointX * mapPointX) * k;
         common->A[0][2] += temp;
         temp = 1.0 - x12 * k;
         common->A[1][1] += temp;
         temp = transformedPointX - mapPointX * (transformedPointY * mapPointY +
                transformedPointX * mapPointX) * k;
         common->A[1][2] += temp;
         temp = x12 + y12 - SQ(transformedPointY * mapPointY +
                transformedPointX * mapPointX) * k;
         common->A[2][2] += temp;
         common->B[0] += transformedPointX - mapPointX - mapPointY *
             (mapPointY * transformedPointX - mapPointX * transformedPointY) * k;
         common->B[1] += transformedPointY - mapPointY + mapPointX *
             (mapPointY * transformedPointX - mapPointX * transformedPointY) * k;
         common->B[2] += (mapPointY * transformedPointX - mapPointX *
             transformedPointY) * (-1.0 +
            (mapPointY * transformedPointY + mapPointX * transformedPointX) * k);
         common->goodCount++;
      }
   }
}

void GraphSlamCPU::calculateICPMatrix(int matchIndex) {
   double shift[3];
   int index;

   for (index = 0; index < 3; index++) {
      shift[index] = -common->B[index] / common->A[index][index];
   }
   for (index = 0; index < 2; index++) {
      shift[index] += (common->A[index][index + 1] * common->B[index + 1]) /
         (common->A[index][index] * common->A[index + 1][index + 1]);
   }
   int finished = 0;
   shift[0] += common->B[2] * (common->A[0][2] * common->A[1][1] - 
         common->A[0][1] * common->A[1][2]) / (common->A[0][0] *
         common->A[1][1] * common->A[2][2]);

   if (common->goodCount > MinGoodCount) {
      /*cout << "shifts: " << shift[0] << " " << shift[1] << " " << 
         shift[2] << " " << common->goodCount << endl;*/
      //Add the amount of the shift to the move offset
      common->potentialMatchX[matchIndex] += shift[0];
      common->potentialMatchY[matchIndex] += shift[1];
      common->potentialMatchTh[matchIndex] += shift[2];

      if (common->numIterations >= MaxIterations &&
            common->goodCount > FinalMinGoodCount) {
         //If the max number of iterations have been exceeded, but the match
         //strength is good, end it successfully
         common->matchSuccess = 1;
         finished = 1;
         cout << "Finish success" << endl;
      } else if (fabs(shift[0]) <= MaxErrorDisp &&
               fabs(shift[1]) <= MaxErrorDisp &&
               fabs(shift[2]) <= MaxErrorTheta &&
               common->goodCount > FinalMinGoodCount) {
         common->matchSuccess = 1;
         finished = 1;
         cout << "Finished succcess " << common->goodCount << endl;
      } else if (common->numIterations >= MaxIterations) {
         common->matchSuccess = -1;
         finished = 1;
         cout << "Too many iterations " << common->numIterations << endl;
      }
   } else {
      //Match failed
      common->matchSuccess = -1;
      finished = 1;
      cout << "Match failed good count: " << common->goodCount << endl;
   }
   if (finished) {
      common->numIterations = 0;
      //TODO: add MaxThetaOptimise code here
      if (common->combineIndex < 0 && common->matchSuccess > 0) {
         //If successful match, add match information to data structures
         int mIndex = common->numConstraints;
         common->numConstraints++;
         int loopIndex = common->numLoopConstraints;
         common->numLoopConstraints++;
         common->constraintType[mIndex] = 1;
         common->constraintIndex[mIndex] = loopIndex;
         common->loopConstraintI[loopIndex] = common->potentialMatches[matchIndex];
         common->loopConstraintJ[loopIndex] = currentLocalMap;
         common->loopConstraintParent[loopIndex] = common->potentialMatchParent[matchIndex];
         common->loopConstraintInfo[loopIndex][0][0] = 0;
         common->loopConstraintInfo[loopIndex][0][1] = 0;
         common->loopConstraintInfo[loopIndex][0][2] = 0;
         common->loopConstraintInfo[loopIndex][1][0] = 0;
         common->loopConstraintInfo[loopIndex][1][1] = 0;
         common->loopConstraintInfo[loopIndex][1][2] = 0;
         common->loopConstraintInfo[loopIndex][2][0] = 0;
         common->loopConstraintInfo[loopIndex][2][1] = 0;
         common->loopConstraintInfo[loopIndex][2][2] = 0;
         common->loopConstraintThetaDisp[loopIndex] = - common->potentialMatchTh[matchIndex];
         double cosTh = cos(-common->potentialMatchTh[matchIndex]);
         double sinTh = sin(-common->potentialMatchTh[matchIndex]);
         common->loopConstraintXDisp[loopIndex] = (cosTh * -common->potentialMatchX[matchIndex] -
                                                  sinTh * -common->potentialMatchY[matchIndex]);
         common->loopConstraintYDisp[loopIndex] = (sinTh * -common->potentialMatchX[matchIndex] +
                                                  cosTh * -common->potentialMatchY[matchIndex]);
         ANGNORM(common->loopConstraintThetaDisp[loopIndex]);
         //common->loopConstraintXDisp[loopIndex] = - common->potentialMatchX[matchIndex];
         //common->loopConstraintYDisp[loopIndex] = - common->potentialMatchY[matchIndex];
         if(LocalMapCombine) {
            common->combineIndex = common->potentialMatches[matchIndex];
         }
      } else if (common->combineIndex >= 0 && common->matchSuccess < 0) {
         int cIndex = common->numConstraints;
         common->numConstraints++;
         common->constraintType[cIndex] = 0;
         common->constraintIndex[cIndex] = currentLocalMap;
         common->combineIndex = -1;
         common->combineMode = 0;
      }
   } else {
      common->numIterations++;
      common->matchSuccess = 0;
   }

}

void GraphSlamCPU::finaliseInformationMatrix() {
   double a[3][3];
   double b[3][3];
   double c[3][3];

   int cIndex = common->constraintIndex[common->numConstraints - 1];

   double cosTh = cos(- common->loopConstraintThetaDisp[cIndex]);
   double sinTh = sin(- common->loopConstraintThetaDisp[cIndex]);
   a[0][0] = cosTh;
   a[1][1] = cosTh;
   a[1][0] = sinTh;
   a[0][1] = -sinTh;
   a[2][2] = 1;
   a[0][2] = 0;
   a[1][2] = 0;
   a[2][0] = 0;
   a[2][1] = 0;

   if (common->infoCount < InformationScaleFactor) {
      common->infoCount = InformationScaleFactor;
   }
   
   for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
 
         //Adjust the parent info matrix to be relative to the parent instead
         //of relative to the current node
         b[y][x] = common->loopConstraintInfo[cIndex][y][x] /= 
                        ((double)common->infoCount / (double)InformationScaleFactor);
      }
   }
   mult3x3Matrix(a, b, c);
   a[1][0] *= -1;
   a[0][1] *= -1;
   mult3x3Matrix(c, a, b);
   for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
         common->loopConstraintInfo[cIndex][y][x] = b[y][x];
      }
   }
   common->infoCount = 0;

}

void GraphSlamCPU::getGlobalHessianMatrix() {
   //Reset the changeInPos variables before the optimise step
   for (int index = 0; index < nextLocalMap; index++) {
      localMaps[index].changeInPos[0] = 0;
      localMaps[index].changeInPos[1] = 0;
      localMaps[index].changeInPos[2] = 0;
   }

   double a[3][3];
   double b[3][3];
   double c[3][3];
   for (int globalWarp = 0; globalWarp < common->numConstraints; globalWarp++) {

      int constraintType = common->constraintType[globalWarp];
      int constraintIndex = common->constraintIndex[globalWarp];
      int iNode;
      int jNode;
      int parentIndex;
      
      if (constraintType == 1) {
         iNode = common->loopConstraintI[constraintIndex];
         parentIndex = common->loopConstraintParent[constraintIndex];
         jNode = common->loopConstraintJ[constraintIndex];
         for (int y = 0; y < 3; y++) {
            for(int x = 0; x < 3; x++) {
               b[y][x] = common->loopConstraintInfo[constraintIndex][y][x];
            }
         }
      } else {
         iNode = localMaps[constraintIndex].indexParentNode;
         parentIndex = iNode;
         jNode = constraintIndex;
         for (int y = 0; y < 3; y++) {
            for(int x = 0; x < 3; x++) {
               b[y][x] = localMaps[constraintIndex].parentInfo[y][x];
            }
         }
      }
      double cosTh = cos(localMaps[iNode].currentGlobalPosTh);
      double sinTh = sin(localMaps[iNode].currentGlobalPosTh);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
      
      mult3x3Matrix(a, b, c);
      a[1][0] *= -1;
      a[0][1] *= -1;      
      mult3x3Matrix(c, a, b);
      //Now have info matrix for constraint in global reference frame in b[warpNum]
      for (int warpIndex = 0; warpIndex < 3; warpIndex++) {
         int tempNode = iNode;
         while (tempNode != parentIndex) {
            tempNode = localMaps[tempNode].indexParentNode;
            common->graphHessian[tempNode][warpIndex] += b[warpIndex][warpIndex];
         }
         tempNode = jNode;
         while (tempNode != parentIndex) {
            common->graphHessian[tempNode][warpIndex] += b[warpIndex][warpIndex];
            tempNode = localMaps[tempNode].indexParentNode;
         }
         if (b[warpIndex][warpIndex] < common->scaleFactor[warpIndex]) {
            common->scaleFactor[warpIndex] = b[warpIndex][warpIndex];
         }
      }
   }
}

void GraphSlamCPU::calculateOptimisationChange(int numIterations) {
   double a[3][3];
   double b[3][3];
   double c[3][3];
   double constraint[3];
   double residual[3];

   for (int globalWarp = 0; globalWarp < common->numConstraints; globalWarp++) {
      
      int constraintType = common->constraintType[globalWarp];
      int constraintIndex = common->constraintIndex[globalWarp];
      int iNode;
      int jNode;
      int parentIndex;
      int x,y;
      
      if (constraintType == 1) {
         iNode = common->loopConstraintI[constraintIndex];
         parentIndex = common->loopConstraintParent[constraintIndex];
         jNode = common->loopConstraintJ[constraintIndex];
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               b[y][x] = common->loopConstraintInfo[constraintIndex][y][x];
            }
         }
         constraint[0] = common->loopConstraintXDisp[constraintIndex];
         constraint[1] = common->loopConstraintYDisp[constraintIndex];
         constraint[2] = common->loopConstraintThetaDisp[constraintIndex];
         
      } else {
         iNode = localMaps[constraintIndex].indexParentNode;
         parentIndex = iNode;
         jNode = constraintIndex;
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               b[y][x] = localMaps[constraintIndex].parentInfo[y][x];
            }
         }
         constraint[0] = localMaps[constraintIndex].parentOffsetX;
         constraint[1] = localMaps[constraintIndex].parentOffsetY;
         constraint[2] = localMaps[constraintIndex].parentOffsetTh;
      }
      int pathLength = (localMaps[iNode].treeLevel - localMaps[parentIndex].treeLevel) +
                       (localMaps[jNode].treeLevel - localMaps[parentIndex].treeLevel);

      //Calculate the rotated information matrix for the constraint 
      //answer is in b - available to all nodes in the warp
      double cosTh = cos(localMaps[iNode].currentGlobalPosTh);
      double sinTh = sin(localMaps[iNode].currentGlobalPosTh);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;

      //Calculate sum inverse dm's and the residual for the constraint
      double dm[3];
      int tempNode;
      double tempPos;

      double commonValue[3];
      double scaleFactor[3];

      ANGNORM(constraint[2]);
      for(int warpIndex = 0; warpIndex < 3; warpIndex++) {
         dm[warpIndex];
         residual[warpIndex] = 0;
         tempNode = iNode;
         while (tempNode != parentIndex) {
            tempPos = getGlobalPosIndex(tempNode, warpIndex);
            tempNode = localMaps[tempNode].indexParentNode;
            dm[warpIndex] += 1 / common->graphHessian[tempNode][warpIndex];
            residual[warpIndex] += getGlobalPosIndex(
                  tempNode, warpIndex) - tempPos;
         }
         tempNode = jNode;
         while (tempNode != parentIndex) {
            dm[warpIndex] += 1 / common->graphHessian[tempNode][warpIndex];
            tempPos = getGlobalPosIndex(tempNode, warpIndex);
            tempNode = localMaps[tempNode].indexParentNode;
            residual[warpIndex] += tempPos - 
               getGlobalPosIndex(tempNode, warpIndex);
         }
         if (warpIndex == 2) {
            ANGNORM(residual[warpIndex]);
         }
         residual[warpIndex] -= (a[warpIndex][0] * constraint[0] +
                      a[warpIndex][1] * constraint[1] +
                      a[warpIndex][2] * constraint[2]);
         if (warpIndex == 2) {
            ANGNORM(residual[warpIndex]);
         }
         residual[warpIndex] *= -1;
         dm[warpIndex] = 1/dm[warpIndex];

      }

      mult3x3Matrix(a, b, c);
      a[1][0] *= -1;
      a[0][1] *= -1;
      mult3x3Matrix(c, a, b);

      for (int warpIndex = 0; warpIndex < 3; warpIndex++) {
         commonValue[warpIndex] = b[warpIndex][0] * residual[0] +
                       b[warpIndex][1] * residual[1] +
                       b[warpIndex][2] * residual[2];

         scaleFactor[warpIndex] = 1 / ((double) numIterations * 
               common->scaleFactor[warpIndex] * ((double) nextLocalMap - 1));
      
         //Now do the calculations for each node in the constraint
         double adjust = scaleFactor[warpIndex] * pathLength * commonValue[warpIndex];
         if (warpIndex == 2) {
            ANGNORM(adjust);
         }
         if (fabs(adjust) > fabs(residual[warpIndex])) {
            adjust = residual[warpIndex];
         }
         tempNode = iNode;
         while (tempNode != parentIndex) {
            tempNode = localMaps[tempNode].indexParentNode;
            double value = adjust * dm[warpIndex] * 
                          1/common->graphHessian[tempNode][warpIndex] * -1;
            localMaps[tempNode].changeInPos[warpIndex] += value;
         }
         tempNode = jNode;
         while (tempNode != parentIndex) {
            double value = adjust * dm[warpIndex] * 
                          1/common->graphHessian[tempNode][warpIndex];
            localMaps[tempNode].changeInPos[warpIndex] += value;
            tempNode = localMaps[tempNode].indexParentNode;
         }
      }
   }
}

double GraphSlamCPU::getGlobalPosIndex(int node, int index) {
   if (index == 0) return localMaps[node].currentGlobalPosX;
   else if (index == 1) return localMaps[node].currentGlobalPosY;
   else if (index == 2) return localMaps[node].currentGlobalPosTh;
   return 0;
}


void GraphSlamCPU::updateGlobalPositions() {
   //Firstly reset the graphHessian and scale factor variables for the next optimisation run
   for (int index = 0; index < MaxNumConstraints + 1; index++) {
      common->graphHessian[index][0] = 0;
      common->graphHessian[index][1] = 0;
      common->graphHessian[index][2] = 0;
   }
   common->scaleFactor[0] = INFINITY;
   common->scaleFactor[1] = INFINITY;
   common->scaleFactor[2] = INFINITY;

   for (int index = 0; index < nextLocalMap; index++) {
      int curMap = index;
      double posChangeX = 0;
      double posChangeY = 0;
      double posChangeTh = 0;
      while (curMap >= 0) {
         posChangeX += localMaps[curMap].changeInPos[0];
         posChangeY += localMaps[curMap].changeInPos[1];
         posChangeTh += localMaps[curMap].changeInPos[2];
         curMap = localMaps[curMap].indexParentNode;
      }
      localMaps[index].currentGlobalPosX += posChangeX;
      localMaps[index].currentGlobalPosY += posChangeY;
      localMaps[index].currentGlobalPosTh += posChangeTh;
      ANGNORM(localMaps[index].currentGlobalPosTh);
   }
}

void GraphSlamCPU::updateGlobalMap() {
   //Firstly, update the current globalCovar for each local map
   double a[3][3];
   double b[3][3];
   double c[3][3];
   int i;
   int x,y;

   for (i = 1; i < nextLocalMap; i++) {
      double parentAngle = localMaps[localMaps[i].indexParentNode]
                                    .currentGlobalPosTh;
      double cosTh = cos(parentAngle);
      double sinTh = sin(parentAngle);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
      
      for (int y = 0; y < 3; y++) {
         for (int x = 0; x < 3; x++) {
            b[y][x] = localMaps[i].parentInfo[y][x];
         }
      }
      mult3x3Matrix(a, b, c);
      a[1][0] *= -1;
      a[0][1] *= -1;
      mult3x3Matrix(c, a, b);
      //now invert it to get the covar matrix
      for (int y = 0; y < 3; y++) {
         for (int x = 0; x < 3; x++) {
            localMaps[i].globalCovar[y][x] = b[y][x];
         }
      }
      invert3x3Matrix(localMaps[i].globalCovar, a);
      for (int y = 0; y < 3; y++) {
         for (int x = 0; x < 3; x++) {
            localMaps[i].globalCovar[y][x] = a[y][x];
         }
      }
      //Fiddle with the covar to make it nicer
      covarFiddle(localMaps[i].globalCovar);
   }

   //Now update the global map
   int globalOffset = 0;
   for(i = 0; i < nextLocalMap; i++) {
      double cosTh = cos(localMaps[i].currentGlobalPosTh);
      double sinTh = sin(localMaps[i].currentGlobalPosTh);
      for (int index = 0; index < localMaps[i].numPoints; index++) {
         int globalIndex = convertToGlobalPosition(localMaps[i].pointsX[index], 
                           localMaps[i].pointsY[index], i, cosTh, sinTh);
         globalMap[globalOffset + index] = globalIndex;
         globalMapHeights[globalOffset + index] = localMaps[i].pointsZ[index];
      }
      globalOffset += localMaps[i].numPoints;
   }
}

void GraphSlamCPU::combineNodes(double alignError, int numOtherGlobalPoints) {
   int numNewPoints = localMaps[currentLocalMap].numPoints;
   int numOldPoints = localMaps[common->combineIndex].numPoints;  

   double skip = 1.0;
   if (numNewPoints + numOldPoints > MAX_LOCAL_POINTS) {
      skip = (numNewPoints + numOldPoints) / MAX_LOCAL_POINTS;
   }
   double cosTh = cos(-common->potentialMatchTh[0]);
   double sinTh = sin(-common->potentialMatchTh[0]);
   for (int index = 0; index < localMaps[currentLocalMap].numPoints; index++) {
      double temp = (double)((int)(((double)index) / skip)) * skip;
      if (temp >= index || (int)(temp + skip) == index) {
         //Point will be combined into the old map
         //First need to transform point to reference frame of old map
         double transformedPointX;
         double transformedPointY;
         double pX = localMaps[currentLocalMap].pointsX[index] - 
               common->potentialMatchX[0];
         double pY = localMaps[currentLocalMap].pointsY[index] - 
               common->potentialMatchY[0];
         transformedPointX = (pX * cosTh - pY * sinTh);
         transformedPointY = (pX * sinTh + pY * cosTh);

         int numPointsToAdd = (int)((double)numNewPoints / skip);
         int pointNum = ceil(index/ skip);
         int space = MAX_LOCAL_POINTS - numOldPoints;
         if (pointNum < space) {
            localMaps[common->combineIndex].pointsX[numOldPoints + pointNum] = transformedPointX;
            localMaps[common->combineIndex].pointsY[numOldPoints + pointNum] = transformedPointY;

            globalMap[numGlobalPoints + pointNum] = convertToGlobalPosition(transformedPointX, 
                  transformedPointY, common->combineIndex, cosTh, sinTh);
            globalMapHeights[numGlobalPoints + pointNum] = localMaps[currentLocalMap].pointsZ[index];
         } else {
            numPointsToAdd -= space;
            double spacer = (numOldPoints / (double)numPointsToAdd);
            pointNum -= space;
            localMaps[common->combineIndex].pointsX[(int)(pointNum * spacer)] = transformedPointX;
            localMaps[common->combineIndex].pointsY[(int)(pointNum * spacer)] = transformedPointY;
            
            globalMap[numOtherGlobalPoints + (int)(pointNum * spacer)] = convertToGlobalPosition( 
                  transformedPointX, transformedPointY, common->combineIndex, cosTh, sinTh);
            globalMapHeights[numGlobalPoints + pointNum] = localMaps[currentLocalMap].pointsZ[index]; 
         }
      }
   }
   int shift = common->potentialMatchTh[0] * (double)NUM_ORIENTATION_BINS / (2 * M_PI);
   int i;
   for (int index = 0; index < NUM_ORIENTATION_BINS; index++) {
      int toI = index + shift;
      if (toI < 0) {
         toI += NUM_ORIENTATION_BINS;
      }
      toI = toI % NUM_ORIENTATION_BINS;
      localMaps[common->combineIndex].orientationHist[index] = 
            (localMaps[common->combineIndex].orientationHist[index] + 
             localMaps[currentLocalMap].orientationHist[toI]) / 2.0;
      localMaps[common->combineIndex].entropyHist[index] = 
            (localMaps[common->combineIndex].entropyHist[index] + 
             localMaps[currentLocalMap].entropyHist[toI]) / 2.0;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         localMaps[common->combineIndex].projectionHist[index][i] = 
            (localMaps[common->combineIndex].projectionHist[index][i] + 
             localMaps[currentLocalMap].projectionHist[toI][i]) / 2.0;
      }
   }
   double a[3][3];
   double b[3][3];
   double c[3][3];
   double adjustPos[3];
   double correctedOffset[3];
   int x,y;

   //If are combining multiple nodes, refine the constraints
   if (common->combineMode > 0 && 
         localMaps[currentLocalMap].indexParentNode != common->combineMode) {
      
      cosTh = cos(-localMaps[currentLocalMap].parentOffsetTh);
      sinTh = sin(-localMaps[currentLocalMap].parentOffsetTh);

      double tempX;
      double tempY;
      tempX = cosTh * -common->potentialMatchX[0] - sinTh * -common->potentialMatchY[0];
      tempY = sinTh * -common->potentialMatchX[0] + cosTh * -common->potentialMatchY[0];

      correctedOffset[0] = localMaps[currentLocalMap].parentOffsetX - tempX;
      correctedOffset[1] = localMaps[currentLocalMap].parentOffsetY - tempY;
      correctedOffset[2] = localMaps[currentLocalMap].parentOffsetTh - common->potentialMatchTh[0];
      
      double existingOffsetX;
      double existingOffsetY;
      double existingOffsetTh;

      if (localMaps[currentLocalMap].indexParentNode != localMaps[common->combineIndex].indexParentNode) {
         correctedOffset[2] *= -1;
         tempX = -correctedOffset[0];
         tempY = -correctedOffset[1];
         cosTh = cos(correctedOffset[2]);
         sinTh = sin(correctedOffset[2]);
         correctedOffset[0] = cosTh * tempX - sinTh * tempY;
         correctedOffset[1] = sinTh * tempX + cosTh * tempY;

         
         a[0][0] = cosTh;
         a[1][1] = cosTh;
         a[1][0] = sinTh;
         a[0][1] = -sinTh;
         a[2][2] = 1;
         a[0][2] = 0;
         a[1][2] = 0;
         a[2][0] = 0;
         a[2][1] = 0;
         
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               b[y][x] = localMaps[currentLocalMap].parentInfo[y][x];
            }
         }
         mult3x3Matrix(a,b,c);
         a[1][0] *= -1;
         a[0][1] *= -1;
         mult3x3Matrix(c,a,b);
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               localMaps[currentLocalMap].parentInfo[y][x] = b[y][x];

               b[y][x] = localMaps[localMaps[currentLocalMap].indexParentNode].parentInfo[y][x];
            }
         }

         existingOffsetX = localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetX;
         existingOffsetY = localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetY;
         existingOffsetTh = localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetTh;
      } else {
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               b[y][x] = localMaps[common->combineIndex].parentInfo[y][x];
            }
         }
         existingOffsetX = localMaps[common->combineIndex].parentOffsetX;
         existingOffsetY = localMaps[common->combineIndex].parentOffsetY;
         existingOffsetTh = localMaps[common->combineIndex].parentOffsetTh;
      }
      for (y = 0; y < 3; y++) {
         for (x = 0; x < 3; x++) {
            a[y][x] = b[y][x] + localMaps[currentLocalMap].parentInfo[y][x];
         }
      }
      invert3x3Matrix(a, c);
      for (int index = 0; index < 3; index++) {
         adjustPos[index] = b[index][0] * existingOffsetX +
                            b[index][1] * existingOffsetY +
                            b[index][2] * existingOffsetTh +
                            localMaps[currentLocalMap].parentInfo[index][0] * correctedOffset[0] +
                            localMaps[currentLocalMap].parentInfo[index][1] * correctedOffset[1] +
                            localMaps[currentLocalMap].parentInfo[index][2] * correctedOffset[2];
         correctedOffset[index] = c[index][0] * adjustPos[0] +
                                  c[index][1] * adjustPos[1] +
                                  c[index][2] * adjustPos[2];
      }
      if (localMaps[currentLocalMap].indexParentNode != localMaps[common->combineIndex].indexParentNode) {
         localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetX = correctedOffset[0];
         localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetY = correctedOffset[1];
         localMaps[localMaps[currentLocalMap].indexParentNode].parentOffsetTh = correctedOffset[2];
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               localMaps[localMaps[currentLocalMap].indexParentNode].parentInfo[y][x] = a[y][x];
            }
         }
      } else {
         localMaps[common->combineIndex].parentOffsetX = correctedOffset[0];
         localMaps[common->combineIndex].parentOffsetY = correctedOffset[1];
         localMaps[common->combineIndex].parentOffsetTh = correctedOffset[2];
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               localMaps[common->combineIndex].parentInfo[y][x] = a[y][x];
            }
         }
      }
   }

   localMaps[common->combineIndex].numPoints += localMaps[currentLocalMap].numPoints;
   if (localMaps[common->combineIndex].numPoints > MAX_LOCAL_POINTS) {
      localMaps[common->combineIndex].numPoints = MAX_LOCAL_POINTS;
   }
   double newMapCentreX = common->currentOffsetX/2.0;
   double newMapCentreY = common->currentOffsetY/2.0;
   localMaps[common->combineIndex].robotMapCentreX = 
         (localMaps[common->combineIndex].robotMapCentreX + newMapCentreX) / 2.0;
   localMaps[common->combineIndex].robotMapCentreY = 
         (localMaps[common->combineIndex].robotMapCentreY + newMapCentreY) / 2.0;

   if (common->combineMode == 0 && 
         common->loopConstraintI[common->numLoopConstraints - 1] != common->combineIndex) {
      //If making an initial combining, add constraint to the parent of currentMap to combineIndex
      int mIndex = common->numConstraints;
      common->numConstraints++;
      int loopIndex = common->numLoopConstraints;
      common->numLoopConstraints++;
      common->constraintType[mIndex] = 1;
      common->constraintIndex[mIndex] = loopIndex;
      common->loopConstraintI[loopIndex] = common->combineIndex;
      common->loopConstraintJ[loopIndex] = localMaps[currentLocalMap].indexParentNode;
      common->loopConstraintParent[loopIndex] = common->loopConstraintParent[loopIndex - 1];
      common->loopConstraintInfo[loopIndex][0][0] = localMaps[currentLocalMap].parentInfo[0][0];
      common->loopConstraintInfo[loopIndex][0][1] = localMaps[currentLocalMap].parentInfo[0][1];
      common->loopConstraintInfo[loopIndex][0][2] = localMaps[currentLocalMap].parentInfo[0][2];
      common->loopConstraintInfo[loopIndex][1][0] = localMaps[currentLocalMap].parentInfo[1][0];
      common->loopConstraintInfo[loopIndex][1][1] = localMaps[currentLocalMap].parentInfo[1][1];
      common->loopConstraintInfo[loopIndex][1][2] = localMaps[currentLocalMap].parentInfo[1][2];
      common->loopConstraintInfo[loopIndex][2][0] = localMaps[currentLocalMap].parentInfo[2][0];
      common->loopConstraintInfo[loopIndex][2][1] = localMaps[currentLocalMap].parentInfo[2][1];
      common->loopConstraintInfo[loopIndex][2][2] = localMaps[currentLocalMap].parentInfo[2][2];

      common->loopConstraintThetaDisp[loopIndex] = -(common->potentialMatchTh[0] 
            + localMaps[currentLocalMap].parentOffsetTh);
      cosTh = cos(-localMaps[currentLocalMap].parentOffsetTh);
      sinTh = sin(-localMaps[currentLocalMap].parentOffsetTh);
      double tempX;
      double tempY;
      tempX = (cosTh * -localMaps[currentLocalMap].parentOffsetX - 
               sinTh * -localMaps[currentLocalMap].parentOffsetY);
      tempY = (sinTh * -localMaps[currentLocalMap].parentOffsetX + 
               cosTh * -localMaps[currentLocalMap].parentOffsetY);
      tempX -= common->potentialMatchX[0];
      tempY -= common->potentialMatchY[0];

      cosTh = cos(-common->potentialMatchTh[0]);
      sinTh = sin(-common->potentialMatchTh[0]);
      common->loopConstraintXDisp[loopIndex] = -(cosTh * -tempX -
                                               sinTh * -tempY);
      common->loopConstraintYDisp[loopIndex] = -(sinTh * -tempX +
                                               cosTh * -tempY);
      ANGNORM(common->loopConstraintThetaDisp[loopIndex]);
   }

   //Fix up the offset from the current node so the next node starts in the right place
   
   //convert to global slam coords
   cosTh = cos(alignError);
   sinTh = sin(alignError);
   double tempX = cosTh * offsetFromParentX - sinTh * offsetFromParentY;
   double tempY = sinTh * offsetFromParentX + cosTh * offsetFromParentY;

   //calculate the new offset wrt global slam coords
   cosTh = cos(localMaps[currentLocalMap].currentGlobalPosTh);
   sinTh = sin(localMaps[currentLocalMap].currentGlobalPosTh);
   double newOffX = (cosTh * -common->potentialMatchX[0] - sinTh * -common->potentialMatchY[0]);
   double newOffY = (sinTh * -common->potentialMatchX[0] + cosTh * -common->potentialMatchY[0]);
   newOffX += tempX;
   newOffY += tempY;

   //convert back to pos track coords and save answer in parent offset of curNode
   //localMaps[currentMap].parentOffset = offCurMap;
   offsetFromParentTh = offsetFromParentTh - common->potentialMatchTh[0];
   cosTh = cos(-alignError);
   sinTh = sin(-alignError);
   offsetFromParentX = cosTh * newOffX - sinTh * newOffY;
   offsetFromParentY = sinTh * newOffX + cosTh * newOffY;

   //make global pos of robot correction and save result in currentglobalpos of
   //current map
   slamPose.position.x = localMaps[common->combineIndex].currentGlobalPosX + newOffX;
   slamPose.position.y = localMaps[common->combineIndex].currentGlobalPosY + newOffY;
   double ys, ps, rs;
   slamPose.getYPR(ys, ps, rs);
   ys = localMaps[common->combineIndex].currentGlobalPosTh + offsetFromParentTh;
   ANGNORM(ys);
   slamPose.setYPR(ys, ps, rs);

   common->combineMode = 1;
}


