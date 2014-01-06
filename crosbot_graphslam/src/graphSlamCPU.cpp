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
}

void GraphSlamCPU::stop() {
   delete [] common->localOG;
   delete [] common->localOGCount;
   delete [] common->localOGZ;
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
      cout << "Creating a new local map" << endl;
      finishMap(angleError, yi, icpPose);
   }

   oldICPPose = icpPose;

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms ";
   }

   finishedSetup = true;
}

void GraphSlamCPU::finishMap(double angleError, double icpTh, Pose icpPose) {
   int numLocalPoints = localMaps[currentLocalMap].numPoints;
   //

   int oldLocalMap = currentLocalMap;
   int numOldMapPoints = numLocalPoints;


   //TODO: temp line below. next local map is not correct
   createNewLocalMap(oldLocalMap, currentLocalMap + 1, currentLocalMap, angleError, icpTh);
   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   currentLocalMapICPPose = icpPose;
   numGlobalPoints += numLocalPoints;
   //currentLocalMap = nextLocalMap;
   //TODO: temp line below, not correct
   currentLocalMap++;
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

