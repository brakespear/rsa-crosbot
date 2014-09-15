/*
 * graphSlamCPU.cpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 *
 * CPU version of graph slam
 */

//NOTE: requires libsuitesparse-dev to be installed

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

   previousScore = 0;
   lastFullLoopIndex = -1;
   previousINode = 0;

   //tempO = false;
   alreadyOutput = false;

   //kinect params
   lastCloudPublished = 0;
   messageSize = 0;
   activeMapIndex = -1;
}

void GraphSlamCPU::initialise(ros::NodeHandle &nh) {

   GraphSlam::initialise(nh);

   ros::NodeHandle paramNH("~");
}

void GraphSlamCPU::start() {
   common = new SlamCommon;
   common->localOG = new int[DimLocalOG * DimLocalOG];
   common->grid = new GridCell[DimLocalOG * DimLocalOG];
   common->constraintType = new int[MaxNumConstraints];
   common->constraintIndex = new int[MaxNumConstraints];
   common->loopConstraintParent = new int[MaxNumLoopConstraints];
   common->loopConstraintI = new int[MaxNumLoopConstraints];
   common->loopConstraintJ = new int[MaxNumLoopConstraints];
   common->loopConstraintXDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintYDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintThetaDisp = new double[MaxNumLoopConstraints];
   common->loopConstraintWeight = new double[MaxNumLoopConstraints];
   common->loopConstraintFull = new bool[MaxNumLoopConstraints];
   common->loopConstraintInfo = new (double[MaxNumLoopConstraints][3][3]);
   common->graphHessian = new (double[MaxNumConstraints + 1][3]);
}

void GraphSlamCPU::stop() {
   delete [] common->localOG;
   delete [] common->grid;
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
   localMaps[0].minOptNode = 0;
   common->startICPTh = 0;
   common->currentOffsetX = 0;
   common->currentOffsetY = 0;
   common->currentOffsetTh = 0;
   for(int i = 0; i < DimLocalOG * DimLocalOG; i++) {
      common->localOG[i] = -1;
      common->grid[i].p.x = 0;
      common->grid[i].p.y = 0;
      common->grid[i].p.z = MinAddHeight;
      common->grid[i].gradX = 0;
      common->grid[i].gradY = 0;
      common->grid[i].count = 0;
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

void GraphSlamCPU::updateTrack(Pose icpPose, PointCloudPtr cloud, ros::Time stamp) {
   //cout << "The icp pose from slam is: " << icpPose << endl;

   didOptimise = false;
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

   Scan *newScan = new Scan();
   memset(newScan->covar, 0, sizeof(double) * 9);
   newScan->correction[0] = 0;
   newScan->correction[1] = 0;
   newScan->correction[2] = 0;
   newScan->pose[0] = common->currentOffsetX;
   newScan->pose[1] = common->currentOffsetY;
   newScan->pose[2] = common->currentOffsetTh;
   newScan->stamp = stamp;
   vector<Point> scanPoints;
   double tempCovar[3][3];
   int count = 0;
   for (i = 0; i < cloud->cloud.size(); ++i) {
      double dist = cloud->cloud[i].x * cloud->cloud[i].x + cloud->cloud[i].y * cloud->cloud[i].y;
      if (dist > LaserMinDist * LaserMinDist && dist < LaserMaxDist * LaserMaxDist &&
         cloud->cloud[i].z + InitHeight > MinAddHeight && cloud->cloud[i].z + InitHeight < MaxAddHeight) {
         Point p;
         p.x = cloud->cloud[i].x * cosTh - cloud->cloud[i].y * sinTh + common->currentOffsetX;
         p.y = cloud->cloud[i].x * sinTh + cloud->cloud[i].y * cosTh + common->currentOffsetY;
         p.z = cloud->cloud[i].z + InitHeight;
         scanPoints.push_back(p);
         newScan->points.push_back(cloud->cloud[i]);

         if (localMaps[currentLocalMap].scans.size() > 0) {
            int ogIndex = getLocalOGIndex(p.x, p.y);
            if (ogIndex >= 0) {
               double x,y;
               x = -sinTh * p.x - cosTh * p.y;
               y = cosTh * p.x - sinTh * p.y;
               double length = sqrt(common->grid[ogIndex].gradX * common->grid[ogIndex].gradX +
                     common->grid[ogIndex].gradY * common->grid[ogIndex].gradY);
               double mapGradX = common->grid[ogIndex].gradX / length;
               double mapGradY = common->grid[ogIndex].gradY / length;
               if (length > 0) {
                  double temp = x * mapGradX + y * mapGradY;
                  //TODO: find a better way to estimate the angular covar of a scan
                  //double temp = 0.5;
                  tempCovar[0][0] += mapGradX * mapGradX;
                  tempCovar[0][1] += mapGradX * mapGradY;
                  tempCovar[0][2] += mapGradX * temp;
                  tempCovar[1][0] += mapGradX * mapGradY;
                  tempCovar[1][1] += mapGradY * mapGradY;
                  tempCovar[1][2] += mapGradY * temp;
                  tempCovar[2][0] += mapGradX * temp;
                  tempCovar[2][1] += mapGradY * temp;
                  tempCovar[2][2] += temp * temp;
                  count++;
               }
            }
         }
      }
   }
   if (localMaps[currentLocalMap].scans.size() > 0) {
      for (i = 0; i < 3; i++) {
         for(j = 0; j < 3; j++) {
            tempCovar[i][j] /= count;
            //cout << tempCovar[i][j] << " ";
         }
      }
      //cout << endl << "* ";
      invert3x3Matrix(tempCovar, newScan->covar);
      for (i = 0 ; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            newScan->covar[i][j] /= PerScanInfoScaleFactor;
            //cout << newScan->covar[i][j] << " ";
            if (i == j && newScan->covar[i][i] < 0) {
               cout << "This shouldn't happen " << newScan->covar[i][i] << " "
                  << i << endl;
            }
            localMaps[currentLocalMap].internalCovar[i][j] += newScan->covar[i][j];
         }
      }
      //cout << endl;
   }

   for (i = 0; i < scanPoints.size(); ++i) {

      int pre = i - 5;
      if (pre < 1) { pre = 1;}
      double preP = (scanPoints[pre - 1].x + scanPoints[pre].x + scanPoints[pre + 1].x) / 3.0;
      int nxt = i + 5;
      if (nxt > (int) scanPoints.size() - 2) { nxt = scanPoints.size() - 2; }
      double nxtP = (scanPoints[nxt - 1].x + scanPoints[nxt].x + scanPoints[nxt + 1].x) / 3.0;
      double mapGradX = nxtP - preP;
      preP = (scanPoints[pre - 1].y + scanPoints[pre].y + scanPoints[pre + 1].y) / 3.0;
      nxtP = (scanPoints[nxt - 1].y + scanPoints[nxt].y + scanPoints[nxt + 1].y) / 3.0;
      double mapGradY = nxtP - preP;

      int ogIndex = getLocalOGIndex(scanPoints[i].x, scanPoints[i].y);
      if (ogIndex >= 0 && mapGradX < GradientDistanceThreshold && mapGradY < GradientDistanceThreshold) {
         common->grid[ogIndex].p.z = std::max(common->grid[ogIndex].p.z, scanPoints[i].z);
         common->grid[ogIndex].p.x += scanPoints[i].x;
         common->grid[ogIndex].p.y += scanPoints[i].y;
         common->grid[ogIndex].count++;
         common->grid[ogIndex].gradX += mapGradY; //Note these are swapped
         common->grid[ogIndex].gradY += mapGradX;

         //Update the histograms
         double orien = atan2(mapGradY, mapGradX);
         orien += M_PI/2;
         if (orien >= M_PI) {
            orien -= 2 * M_PI;
         }
         int orienIndex = (orien + M_PI) * NUM_ORIENTATION_BINS / (2*M_PI);
         localMaps[currentLocalMap].orientationHist[orienIndex]++;
         double mapSize = DimLocalOG * CellSize;
         for(j = 0; j < NUM_ORIENTATION_BINS; j++) {
            double dist = scanPoints[i].x * common->histCos[j] + 
                              scanPoints[i].y * common->histSin[j];
            int projIndex = (dist + mapSize / 2.0f) / 
                              (mapSize / (double) NUM_PROJECTION_BINS);
            if (projIndex < 0 || projIndex >= NUM_PROJECTION_BINS) {
               continue;
            }
            double normalX = -mapGradY;
            double normalY = mapGradX;
            double normalise = sqrt(normalX * normalX + normalY * normalY);
            double weight = normalX / normalise * common->histCos[j] +
                                normalY / normalise * common->histSin[j];
            localMaps[currentLocalMap].projectionHist[j][projIndex] += weight;
         }

         if (common->grid[ogIndex].count == MinObservationCount && 
               localMaps[currentLocalMap].numPoints < MAX_LOCAL_POINTS) {
            common->activeCells[localMaps[currentLocalMap].numPoints] = ogIndex;
            common->localOG[ogIndex] = localMaps[currentLocalMap].numPoints;
         
            if (scanPoints[i].x < common->minMapRangeX) {
               common->minMapRangeX = scanPoints[i].x;
            }
            if (scanPoints[i].x > common->maxMapRangeX) {
               common->maxMapRangeX = scanPoints[i].x;
            }
            if (scanPoints[i].y < common->minMapRangeY) {
               common->minMapRangeY = scanPoints[i].y;
            }
            if (scanPoints[i].y > common->maxMapRangeY) {
               common->maxMapRangeY = scanPoints[i].y;
            }
            addToFreeArea(scanPoints[i].x, scanPoints[i].y);

            //Add the point to the global map
            int globalIndex = convertToGlobalPosition(scanPoints[i].x, scanPoints[i].y, 
                           currentLocalMap, cosThG, sinThG);
            if (numGlobalPoints + localMaps[currentLocalMap].numPoints < globalMap.size()) {
               globalMap[numGlobalPoints + localMaps[currentLocalMap].numPoints] = globalIndex;
               globalMapHeights[numGlobalPoints + common->localOG[ogIndex]] = 
                  common->grid[ogIndex].p.z;
            } else {
               globalMap.push_back(globalIndex);
               globalMapHeights.push_back(common->grid[ogIndex].p.z);
            }
            localMaps[currentLocalMap].numPoints++;
            localMaps[currentLocalMap].numWarpPoints++;
         }
         if (common->localOG[ogIndex] > -1) {
            localMaps[currentLocalMap].lastObserved[common->localOG[ogIndex]] =
               localMaps[currentLocalMap].scans.size();
         }
      }
   }
   if (!LocalMapWarp) {
      newScan->points.clear();
   }
   localMaps[currentLocalMap].scans.push_back(newScan);
   //updateTestMap();
   activeMapIndex = currentLocalMap;

   double temp = sqrt(offsetFromParentX * offsetFromParentX +
         offsetFromParentY * offsetFromParentY);

   //Different estimation of new local maps
   double edgeDist = (LocalMapSize / 2.0) - 1.0;
   //TODO: fix up the angular covariance calculations
   //TODO: using distance to edges is disabled at the moment. Should it be put back in?
   double sumX = localMaps[currentLocalMap].internalCovar[0][0] + 
      fabs(localMaps[currentLocalMap].internalCovar[0][1]) /*+ fabs(localMaps[currentLocalMap].internalCovar[0][2])*/;
   double sumY = fabs(localMaps[currentLocalMap].internalCovar[1][0]) + 
      localMaps[currentLocalMap].internalCovar[1][1] /*+ fabs(localMaps[currentLocalMap].internalCovar[1][2])*/;
   double sumTh = 0/*fabs(localMaps[currentLocalMap].internalCovar[2][0]) + 
      fabs(localMaps[currentLocalMap].internalCovar[2][1]) + localMaps[currentLocalMap].internalCovar[2][2]*/;

   if (sumX > LocalMapCovarianceThreshold || sumY > LocalMapCovarianceThreshold 
         || sumTh > LocalMapCovarianceThreshold) {
      cout << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^New local map because of covariances" << endl;
      localMaps[currentLocalMap].isFeatureless = true;
      finishMap(angleError, yi, icpPose);
   } else if (temp > LocalMapDistance * 1.0 /*|| fabs(offsetFromParentTh) > 3*M_PI/4.0*/) {
      cout << "New local map because of local map distance only" << endl;
      finishMap(angleError, yi, icpPose);
   } else if ((temp > LocalMapDistance) && (fabs(common->minMapRangeX) > edgeDist || fabs(common->minMapRangeY) > edgeDist ||
            fabs(common->maxMapRangeX) > edgeDist || fabs(common->maxMapRangeY) > edgeDist)) {
      cout << "New local map because of distance and edges " << temp << " " << common->minMapRangeX << " " << 
         common->minMapRangeY << " " << common->maxMapRangeX << " " << common->maxMapRangeY << endl;
      finishMap(angleError, yi, icpPose);
   }

   /*if (temp >= LocalMapDistance) {
      finishMap(angleError, yi, icpPose);
   }*/

   oldICPPose = icpPose;

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 50 == 0 || didOptimise) {
      double ys, ps, rs, yi;
      slamPose.getYPR(ys, ps, rs);
      icpPose.getYPR(yi, ps, rs);
      if (didOptimise) {
         cout << (t2 - t1).toSec() * 1000.0f << "ms total ";
      }
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms Pos: " << slamPose.position.x
        << " " << slamPose.position.y << " " << ys << " icp: " << icpPose.position.x << " " <<
       icpPose.position.y << " " << yi << endl;
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            cout << localMaps[currentLocalMap].internalCovar[i][j] << " ";
         }
      }
      cout << endl;
      cout << "Stamp: " << stamp << endl;
   }

   finishedSetup = true;


   //Temp output for seeding manual relations entry
   /*ostringstream tt;
   tt << stamp;
   const char *st = tt.str().c_str();
   double stampTime;
   sscanf(st, "%lf", &stampTime);*/
   /*if (stampTime > 1364015744 && !alreadyOutput) {
      alreadyOutput = true;
   //if (numIterations == 100) {
      cout << "OUTPUTTING DATA " << nextLocalMap << endl;
      FILE *f = fopen("/home/adrianrobolab/mapVerify/slamPos.txt", "w");

      int scanCount = 0;
      for(int mapI = 0; mapI <= currentLocalMap; mapI++) {
         for (int scanI = 0; scanI < localMaps[mapI].scans.size(); scanI++, scanCount++) {
            if (scanCount % 10 == 0) {
               double posX = localMaps[mapI].scans[scanI]->pose[0] + localMaps[mapI].scans[scanI]->correction[0];
               double posY = localMaps[mapI].scans[scanI]->pose[1] + localMaps[mapI].scans[scanI]->correction[1];
               double posTh = localMaps[mapI].scans[scanI]->pose[2] + localMaps[mapI].scans[scanI]->correction[2];

               double globPosX;
               double globPosY;
               double globPosTh;
               convertToGlobalCoord(posX, posY, localMaps[mapI].currentGlobalPosX, 
                     localMaps[mapI].currentGlobalPosY, localMaps[mapI].currentGlobalPosTh, &globPosX,
                     &globPosY);
               globPosTh = posTh + localMaps[mapI].currentGlobalPosTh;

               ostringstream ss;
               ss << localMaps[mapI].scans[scanI]->stamp;

               fprintf(f, "%s %lf %lf %lf %d", ss.str().c_str(), 
                     globPosX, globPosY, globPosTh, localMaps[mapI].scans[scanI]->points.size());
               for (int c = 0; c < localMaps[mapI].scans[scanI]->points.size(); c++) {
                  fprintf(f, " %lf %lf", localMaps[mapI].scans[scanI]->points[c].x,
                        localMaps[mapI].scans[scanI]->points[c].y);
               }
               fprintf(f, "\n");
            }
         }
      }
      fclose(f);
   }*/
   //temp output for printing slam positions
   /*if (stampTime > 1364015711.983 && !alreadyOutput) {
   //if (stampTime > 1364015663.983 && !alreadyOutput) {
      alreadyOutput = true;
   //if (numIterations == 100) {
      cout << "OUTPUTTING STATE NOW" << endl << endl;
      FILE *f = fopen("/home/adrian/slamPositions.txt", "w");

      //int scanCount = 0;
      for(int mapI = 0; mapI <= currentLocalMap; mapI++) {
         for (int scanI = 0; scanI < localMaps[mapI].scans.size(); scanI++) {
            //if (scanCount % 10 == 0) {
               double posX = localMaps[mapI].scans[scanI]->pose[0] + localMaps[mapI].scans[scanI]->correction[0];
               double posY = localMaps[mapI].scans[scanI]->pose[1] + localMaps[mapI].scans[scanI]->correction[1];
               double posTh = localMaps[mapI].scans[scanI]->pose[2] + localMaps[mapI].scans[scanI]->correction[2];

               double globPosX;
               double globPosY;
               double globPosTh;
               convertToGlobalCoord(posX, posY, localMaps[mapI].currentGlobalPosX, 
                     localMaps[mapI].currentGlobalPosY, localMaps[mapI].currentGlobalPosTh, &globPosX,
                     &globPosY);
               globPosTh = posTh + localMaps[mapI].currentGlobalPosTh;

               ostringstream ss;
               ss << localMaps[mapI].scans[scanI]->stamp;

               fprintf(f, "FLASER 0 0 0 0 %lf %lf %lf %s NAME %s\n", 
                     globPosX, globPosY, globPosTh, ss.str().c_str(), ss.str().c_str());
               //for (int c = 0; c < localMaps[mapI].scans[scanI]->points.size(); c++) {
               //   fprintf(f, " %lf %lf", localMaps[mapI].scans[scanI]->points[c].x,
               //         localMaps[mapI].scans[scanI]->points[c].y);
               //}
               //fprintf(f, "\n");
               //scanCount++;
            //}
         }
      }
      fclose(f);
   }*/
}

void GraphSlamCPU::finishMap(double angleError, double icpTh, Pose icpPose) {

{{ Lock lock(masterLockSmith);
   cout << "Creating a new local map " << currentLocalMap << " points: " 
      << localMaps[currentLocalMap].numPoints << endl;

   int numLocalPoints = localMaps[currentLocalMap].numPoints;

   int oldLocalMap = currentLocalMap;
   int numOldMapPoints = numLocalPoints;

   int i;
   for (i = 0; i < numLocalPoints; i++) {
      int ogIndex = common->activeCells[i];
      double count = (double)common->grid[ogIndex].count;
      localMaps[currentLocalMap].pointsX[i] = common->grid[ogIndex].p.x / count;
      localMaps[currentLocalMap].pointsY[i] = common->grid[ogIndex].p.y / count;
      localMaps[currentLocalMap].pointsZ[i] = common->grid[ogIndex].p.z;
      globalMapHeights[numGlobalPoints + i] = common->grid[ogIndex].p.z;
      localMaps[currentLocalMap].warpPointsX[i] = localMaps[currentLocalMap].pointsX[i];
      localMaps[currentLocalMap].warpPointsY[i] = localMaps[currentLocalMap].pointsY[i];
      localMaps[currentLocalMap].warpPointsZ[i] = localMaps[currentLocalMap].pointsZ[i];
      //TODO: Should these be / count ??
      localMaps[currentLocalMap].gradX[i] = common->grid[ogIndex].gradX;
      localMaps[currentLocalMap].gradY[i] = common->grid[ogIndex].gradY;
   }

   updateTestMap();

   if (parentLocalMap >= 0) {
      parentLocalMap = currentLocalMap;
      //getHessianMatch(-1);
      prepareLocalMap();
      cout << "Global covar of map is: ";
      int k, l;
      for (k = 0; k < 3; k++) {
         for (l = 0; l < 3; l++) {
            cout << localMaps[currentLocalMap].internalCovar[k][l] << " ";
         }
      }
      cout << endl;
      bool needOptimisation = false;
      if (combineMode > 0) {
         int combineIndex = common->combineIndex;
         if (combineIndex >= 0) {
            cout << "****Combining node: " << currentLocalMap << " with " << combineIndex << endl;
            int nI = 0;
            int matchSuccess = 0;
            while (matchSuccess == 0) {
               alignICP(combineIndex, 0, currentLocalMap);
               calculateICPMatrix(0, true, currentLocalMap);
               matchSuccess = common->matchSuccess;
               nI++;
            }
            cout << "Number of iterations: " << nI << endl;
            if (matchSuccess == 1) {
               cout << "Combining map succeeded" << endl;
               int numOtherGlobalPoints = 0;
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
      bool loopClosed = false;
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

            //Debugging map print
            //updateTestMap();

            for (i = 0; i < common->numPotentialMatches; i++) {
               int matchSuccess = 0;
               int c = 0;
               while (matchSuccess == 0) {
                  alignICP(common->potentialMatches[i], i, currentLocalMap);
                  calculateICPMatrix(i, true, currentLocalMap);
                  matchSuccess = common->matchSuccess;
                  c++;
               }
               cout << "Number of iterations in alignment: " << c << " " << matchSuccess << endl;
               if (matchSuccess == 1) {
                  //updateTestMap();
                  getHessianMatch(numConstraints);
                  finaliseInformationMatrix();
                  numConstraints++;
                  needOptimisation = true;
                  loopClosed = true;
                  parentLocalMap = common->potentialMatches[i];
                  //Only add one loop closing constraint
                  break;
               } else {
                  cout << "Alignment failed" << endl;
               }
            }
         }
         if (!loopClosed && UseTempLoopClosures) {
            bool matchMade = findTempMatches();
            if (matchMade) {
               needOptimisation = true;
            }
         }
      }
      if (needOptimisation) {
         didOptimise = true;
         cout << "Optimising graph" << endl;
         double posBeforeX, posBeforeY, posBeforeTh;
         if (loopClosed) {
            cout << "Loop was closed" << endl;
            posBeforeX = localMaps[parentLocalMap].currentGlobalPosX;
            posBeforeY = localMaps[parentLocalMap].currentGlobalPosY;
            posBeforeTh = localMaps[parentLocalMap].currentGlobalPosTh;
         } else {
            posBeforeX = localMaps[currentLocalMap].currentGlobalPosX;
            posBeforeY = localMaps[currentLocalMap].currentGlobalPosY;
            posBeforeTh = localMaps[currentLocalMap].currentGlobalPosTh;
         }

         cout << "pos before: " << posBeforeX << " " << posBeforeY << " " << posBeforeTh << endl;
         //for (int numIterations = 1; numIterations < 6; numIterations++) {
         for (int k = 0; k < nextLocalMap; k++) {
         //   cout << "Pose before of map: " << k << " is: " << localMaps[k].currentGlobalPosX << " " <<
         //      localMaps[k].currentGlobalPosY << " " << localMaps[k].currentGlobalPosTh << endl;
            localMaps[k].startingPos[0] = localMaps[k].currentGlobalPosX;
            localMaps[k].startingPos[1] = localMaps[k].currentGlobalPosY;
            localMaps[k].startingPos[2] = localMaps[k].currentGlobalPosTh;
         }

         ros::WallTime t1 = ros::WallTime::now();
         int optType = 1;
         if (!loopClosed) {
            optType = -1;
         }
         optimiseGraph(optType);
         if (loopClosed) {
            optType = 0;
            evaluateTempConstraints();
            optimiseGraph(optType);
         }
         /*for (int numIterations = 1; numIterations < 10 * 2; numIterations++) {
            getGlobalHessianMatrix();
            if (numIterations == 10 && loopClosed) {
               optType = 0;
               evaluateTempConstraints();
            }
            calculateOptimisationChange(numIterations, optType);
            updateGlobalPositions();

         }*/
         updateRobotMapCentres();
         ros::WallTime t2 = ros::WallTime::now();
         ros::WallDuration tote = t2 - t1;
         cout << "Time to optimise " << tote.toSec() * 1000.0f << "ms" << endl;
         bool foundMoreLoops = false;
         for (int k = 0; loopClosed && UseTempLoopClosures && k < nextLocalMap; k++) {
            double angleMov = localMaps[k].currentGlobalPosTh - localMaps[k].startingPos[2];
            ANGNORM(angleMov);
            if (fabs(localMaps[k].currentGlobalPosX - localMaps[k].startingPos[0]) > LargeMovementThreshold ||
                  fabs(localMaps[k].currentGlobalPosY - localMaps[k].startingPos[1]) > LargeMovementThreshold ||
                  fabs(angleMov) > LargeMovementThreshold) {
                     
               cout << "Map " << k << " moved a lot " << localMaps[k].currentGlobalPosX << " " <<
                 localMaps[k].currentGlobalPosY << " " << localMaps[k].currentGlobalPosTh << "  " <<
                 localMaps[k].startingPos[0] << " " << localMaps[k].startingPos[1] << " " <<
                 localMaps[k].startingPos[2] << endl;
               foundMoreLoops = findChangedPosMatches(k) || foundMoreLoops;
            }
         //   cout << "Pose after of map: " << k << " is: " << localMaps[k].currentGlobalPosX << " " <<
         //      localMaps[k].currentGlobalPosY << " " << localMaps[k].currentGlobalPosTh << endl;
            //cout << "   Change was: " << localMaps[k].totalChange[0] << " " << localMaps[k].totalChange[1] << " " 
            //   << localMaps[k].totalChange[2] << endl;
         }
         if (foundMoreLoops) {
            cout << "****Optimising again" << endl;
            optimiseGraph(0);
            /*for (int numIterations = 1; numIterations < 10; numIterations++) {
               getGlobalHessianMatrix();
               calculateOptimisationChange(numIterations, 0);
               updateGlobalPositions();
            }*/
            updateRobotMapCentres();
         }

         t1 = ros::WallTime::now();
         updateGlobalMap();
         t2 = ros::WallTime::now();
         tote = t2 - t1;
         cout << "Time to update global map " << tote.toSec() * 1000.0f << "ms" << endl;
         
         resetMap = true;
         lastCloudPublished = 0;
         historySlamPoses.resize(0);


         double posChangeX;
         double posChangeY;
         double posChangeTh;
         if (loopClosed) {
            posChangeTh = localMaps[parentLocalMap].currentGlobalPosTh + common->currentOffsetTh;
            convertToGlobalCoord(common->currentOffsetX, common->currentOffsetY,
                  localMaps[parentLocalMap].currentGlobalPosX, localMaps[parentLocalMap].currentGlobalPosY,
                  localMaps[parentLocalMap].currentGlobalPosTh, &posChangeX, &posChangeY);
         } else {
            posChangeTh = localMaps[currentLocalMap].currentGlobalPosTh + common->currentOffsetTh;
            convertToGlobalCoord(common->currentOffsetX, common->currentOffsetY,
                  localMaps[currentLocalMap].currentGlobalPosX, localMaps[currentLocalMap].currentGlobalPosY,
                  localMaps[currentLocalMap].currentGlobalPosTh, &posChangeX, &posChangeY);
         }
            
         cout << "pos change: " << posChangeX << " " << posChangeY << " " << posChangeTh << endl;

         slamPose.position.x = posChangeX;
         slamPose.position.y = posChangeY;
         double ys, ps, rs;
         slamPose.getYPR(ys, ps, rs);
         ys = posChangeTh;
         ANGNORM(ys);
         slamPose.setYPR(ys, ps, rs);

         if (LocalMapCombine) {
            combineMode++;
         }
      }
   } else {
      prepareLocalMap();
      parentLocalMap = currentLocalMap;
      nextLocalMap++;
   }      
   cout << "Maps are: old: " << oldLocalMap << " new: " << nextLocalMap << " parent: " 
      << parentLocalMap << endl;
   createNewLocalMap(oldLocalMap, nextLocalMap, parentLocalMap, angleError, icpTh);
   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   currentLocalMapICPPose = icpPose;
   numGlobalPoints += numLocalPoints;
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
   int numLocalPoints = localMaps[currentLocalMap].numWarpPoints;
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
         if (x >= globalMap.size()) {
            cout << "Bother. global map size " << globalMap.size() << " " << x << endl;
         }
         int index = globalMap[x];
         int yi = index / DimGlobalOG;
         int xi = index % DimGlobalOG;
         if (xi > DimGlobalOG || yi > DimGlobalOG) {
            cout << "global map indicies are out: " << xi << " " << yi << " " << index << " " << x << " " << globalMap.size() << endl;
            continue;
         }


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

               //test
               if (localMaps.size() > 3 && x > localMaps[0].numPoints + localMaps[1].numPoints && x < localMaps[0].numPoints + localMaps[1].numPoints + localMaps[2].numPoints) {
                  cellsP->current = true;
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

int GraphSlamCPU::getScanIndex(int mapIndex) {
   return localMaps[mapIndex].scans.size() - 1;
}

void GraphSlamCPU::getScanPose(int mapIndex, int scanIndex, double& px, double& py, double& pth) {
   px = localMaps[mapIndex].scans[scanIndex]->pose[0] + 
      localMaps[mapIndex].scans[scanIndex]->correction[0];
   py = localMaps[mapIndex].scans[scanIndex]->pose[1] + 
      localMaps[mapIndex].scans[scanIndex]->correction[1];
   pth = localMaps[mapIndex].scans[scanIndex]->pose[2] + 
      localMaps[mapIndex].scans[scanIndex]->correction[2];

}

void GraphSlamCPU::clearMap(int mapIndex) {
   localMaps[mapIndex].numPoints = 0;
   localMaps[mapIndex].numWarpPoints = 0;
   localMaps[mapIndex].indexNextNode = -1;
   localMaps[mapIndex].freeArea = new double[DimLocalOG*DimLocalOG];
   localMaps[mapIndex].isFeatureless = false;
   memset(localMaps[mapIndex].orientationHist, 0, sizeof(double) * NUM_ORIENTATION_BINS);
   memset(localMaps[mapIndex].entropyHist, 0, sizeof(double) * NUM_ORIENTATION_BINS);
   memset(localMaps[mapIndex].projectionHist, 0, sizeof(double) * NUM_ORIENTATION_BINS
         * NUM_PROJECTION_BINS);
   memset(localMaps[mapIndex].parentInfo, 0, sizeof(double) * 9);
   memset(localMaps[mapIndex].internalCovar, 0, sizeof(double) * 9);
   for (int i = 0; i < DimLocalOG * DimLocalOG; i++) {
      localMaps[mapIndex].freeArea[i] = -1;
   }
}

inline int GraphSlamCPU::getLocalOGIndex(double x, double y) {
   int i,j;
   double off = (DimLocalOG * CellSize) / 2.0;
   i = (x + off) / CellSize;
   j = (y + off) / CellSize;
   //check to see if the point fits inside the occupancy grid
   if (i >= 0 && i < DimLocalOG && j >= 0 && j < DimLocalOG) {
      return j * DimLocalOG + i;
   } else {
      return -1;
   }
}

inline int GraphSlamCPU::convertToGlobalPosition(double x, double y, int mapIndex, double cosTh, double sinTh) {
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
      common->grid[i].p.x = 0;
      common->grid[i].p.y = 0;
      common->grid[i].p.z = MinAddHeight;
      common->grid[i].gradX = 0;
      common->grid[i].gradY = 0;
      common->grid[i].count = 0;

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
      localMaps[0].robotMapCentreX = common->currentOffsetX/2.0;
      localMaps[0].robotMapCentreY = common->currentOffsetY/2.0;
   }
   double cosTh = cos(-common->startICPTh);
   double sinTh = sin(-common->startICPTh);
   localMaps[oldLocalMap].indexNextNode = newLocalMap;
   localMaps[oldLocalMap].nextOffsetX = cosTh * offsetFromParentX - sinTh * offsetFromParentY;
   localMaps[oldLocalMap].nextOffsetY = sinTh * offsetFromParentX + cosTh * offsetFromParentY;
   localMaps[oldLocalMap].nextOffsetTh = offsetFromParentTh;

   //localMaps[oldLocalMap].robotMapCentreX = common->currentOffsetX/2.0;
   //localMaps[oldLocalMap].robotMapCentreY = common->currentOffsetY/2.0;
   localMaps[newLocalMap].indexParentNode = parentLocalMap;
   localMaps[newLocalMap].treeLevel = localMaps[parentLocalMap].treeLevel + 1;
   localMaps[newLocalMap].minOptNode = newLocalMap;
   common->minMapRangeX = INFINITY;
   common->maxMapRangeX = 0;
   common->minMapRangeY = INFINITY;
   common->maxMapRangeY = 0;
   common->numPotentialMatches = 0;
   common->startICPTh = icpTh;


   /*double cosTh = cos(angleError);
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
   ANGNORM(localMaps[newLocalMap].currentGlobalPosTh);*/


   convertToGlobalCoord(common->currentOffsetX, common->currentOffsetY,
         localMaps[parentLocalMap].currentGlobalPosX, localMaps[parentLocalMap].currentGlobalPosY,
         localMaps[parentLocalMap].currentGlobalPosTh, &(localMaps[newLocalMap].currentGlobalPosX),
         &(localMaps[newLocalMap].currentGlobalPosY));
   localMaps[newLocalMap].currentGlobalPosTh = localMaps[parentLocalMap].currentGlobalPosTh +
      common->currentOffsetTh;
   ANGNORM(localMaps[newLocalMap].currentGlobalPosTh);

   //Make the offset relative to the parent instead of the global coord system
   /*cosTh = cos(-localMaps[parentLocalMap].currentGlobalPosTh);
   sinTh = sin(-localMaps[parentLocalMap].currentGlobalPosTh);
   localMaps[newLocalMap].parentOffsetX = cosTh * tempX - sinTh * tempY;
   localMaps[newLocalMap].parentOffsetY = sinTh * tempX + cosTh * tempY;
   localMaps[newLocalMap].parentOffsetTh = offsetFromParentTh;*/
   localMaps[newLocalMap].parentOffsetX = common->currentOffsetX;
   localMaps[newLocalMap].parentOffsetY = common->currentOffsetY;
   localMaps[newLocalMap].parentOffsetTh = common->currentOffsetTh;

   common->currentOffsetX = 0;
   common->currentOffsetY = 0;
   common->currentOffsetTh = 0;

}

void GraphSlamCPU::addToFreeArea(double px, double py) {

   int index = getLocalOGIndex(px, py);
   double origX = px;
   double origY = py;
   px /= CellSize;
   py /= CellSize;
   double currentOffsetX = common->currentOffsetX / CellSize;
   double currentOffsetY = common->currentOffsetY / CellSize;

   //Ray trace
   double dx = fabs(currentOffsetX - px);
   double dy = fabs(currentOffsetY - py);

   int x = int(floor(px));
   int y = int(floor(py));
   int n = 1;
   int xInc, yInc;
   double error;

   if (dx == 0) {
      xInc = 0;
      error = INFINITY;
   } else if (currentOffsetX > px) {
      xInc = 1;
      n += int(floor(currentOffsetX)) - x;
      error = (floor(px) + 1 - px) * dy;
   } else {
      xInc = -1;
      n += x - int(floor(currentOffsetX));
      error = (px - floor(px)) * dy;
   }
   if (dy == 0) {
      yInc = 0;
      error -= INFINITY;
   } else if (currentOffsetY > py) {
      yInc = 1;
      n += int(floor(currentOffsetY)) - y;
      error -= (floor(py) + 1 - py) * dx;
   } else {
      yInc = -1;
      n += y - int(floor(currentOffsetY));
      error -= (py - floor(py)) * dx;
   }

   //yInc *= DimLocalOG;

   //Now actually go through all the cells
   int startIndex = index;
   x = index % DimLocalOG;
   y = index / DimLocalOG;
   double off = CellSize / 2.0 - (DimLocalOG * CellSize) / 2.0;

   double denom = log(1.0 + FreeAreaDistanceThreshold);
   for (; n > 0; --n) {
      if (index < 0 || index >= DimLocalOG * DimLocalOG) {
         cout << "Something has gone wrong in addToFreeArea " << index << endl;
      }

      double xCent = ((double) x) * CellSize + off;
      double yCent = ((double) y) * CellSize + off;
      //cout << xCent << " " << yCent << " " << origX << " " << origY << " " << off << endl;
      double dist = sqrt((origX - xCent) * (origX - xCent) + (origY - yCent) * (origY - yCent));

      dist = fmin(dist, FreeAreaDistanceThreshold);
      double score = log (dist + 1.0) / denom;
      //cout << score << endl;

      if (localMaps[currentLocalMap].freeArea[index] == -1 || localMaps[currentLocalMap].freeArea[index] > score) {
         localMaps[currentLocalMap].freeArea[index] = score;
      }

      //do whatever(x,y)
      if (error > 0) {
         index += yInc * DimLocalOG;
         y += yInc;
         error -= dx;
      } else {
         index += xInc;
         x += xInc;
         error += dy;
      }
   }
   localMaps[currentLocalMap].freeArea[startIndex] = 0;
}

void GraphSlamCPU::updateRobotMapCentres() {
   for (int i = 0; i < nextLocalMap; i++) {
      convertToGlobalCoord(localMaps[i].robotMapCentreX, localMaps[i].robotMapCentreY,
            localMaps[i].currentGlobalPosX, localMaps[i].currentGlobalPosY,
            localMaps[i].currentGlobalPosTh, &(localMaps[i].globalRobotMapCentreX),
            &(localMaps[i].globalRobotMapCentreY));
   }
}

double GraphSlamCPU::evaluateMapMatch(int ref, int test, double offX, double offY, double offTh, int *overlapNum) {
   //if (tempO) {
   /*int x,y;
   for (y = 0; y < testMap->height; y++) {
      crosbot::LocalMap::Cell *cellsP = &(testMap->cells[y][0]);
      for (x = 0; x < testMap->width; x++) {
         cellsP->current = false;
         cellsP->hits = 0;
         cellsP++;
      }
   }
   int mapWidth = LocalMapSize / CellSize;
   double off = (mapWidth * CellSize) / 2.0 - CellSize / 2.0;
   int k;
   for (k = 0; k < localMaps[currentLocalMap].numPoints; k++) {
      double xd, yd;
      xd = localMaps[ref].pointsX[k];
      yd = localMaps[ref].pointsY[k];
      int i,j;
      i = testMap->width/2 + xd / testMap->resolution;
      j = testMap->height/2 - yd / testMap->resolution;
      if (i >= 0 && i < testMap->width && j >= 0 && j < testMap->height) {
         crosbot::LocalMap::Cell *cellsP = &(testMap->cells[testMap->height - j - 1][i]);
         cellsP->hits = testMap->maxHits;
         cellsP->current = true;
      }
   }*/
   //}



   double cosTh = cos(-offTh);
   double sinTh = sin(-offTh);

   double score = 0;
   int numOverlap = 0;

   int tt = 0;
   int numPoints = localMaps[test].numPoints;
   for (int i = 0; i < numPoints; i++) {
      double px, py;
      convertReferenceFrame(localMaps[test].pointsX[i], localMaps[test].pointsY[i], offX, offY,
            cosTh, sinTh, &px, &py);
      int index = getLocalOGIndex(px, py);
      if (index >= 0 && localMaps[ref].freeArea[index] >= 0) {
         score += localMaps[ref].freeArea[index];
         numOverlap++;
         if (localMaps[ref].freeArea[index] == 0) {
            tt++;
         }
      }

      //if (tempO) {
      /*int pi,pj;
      pi = testMap->width/2 + px / testMap->resolution;
      pj = testMap->height/2 - py / testMap->resolution;
      if (pi >= 0 && pi < testMap->width && pj >= 0 && pj < testMap->height) {
         crosbot::LocalMap::Cell *cellsP = &(testMap->cells[testMap->height - pj - 1][pi]);
         cellsP->hits = testMap->maxHits;
      }*/
      //}

   }

   cout << "In evaluate match " << tt << " points exactly match and " << numOverlap - tt << " points are over free areas" << endl;
   *overlapNum = numOverlap;
   if (numOverlap == 0) {
      return -1;
   } else {
      return score / (double) numOverlap;
   }
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
      matchIndex = getLocalOGIndex(transformedPointX, transformedPointY);
      
      if (matchIndex >= 0 && common->localOG[matchIndex] >= 0) {

         double mapGradX;
         double mapGradY;
         double x,y;
         mapGradX = common->grid[matchIndex].gradX;
         mapGradY = common->grid[matchIndex].gradY;
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

inline void GraphSlamCPU::convertReferenceFrame(double pX, double pY, double offsetX, double offsetY,
         double cosTh, double sinTh, double *pointX, double *pointY) {
   double tempX;
   double tempY;
   tempX = pX - offsetX;
   tempY = pY - offsetY;
   *pointX = tempX * cosTh - tempY * sinTh;
   *pointY = tempX * sinTh + tempY * cosTh;
}

int GraphSlamCPU::findMatchingPoint(double pointX, double pointY, int searchFactor, int currentMap) {
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
                  ogPointX = localMaps[currentMap].pointsX[common->localOG[ogIndex]];
                  ogPointY = localMaps[currentMap].pointsY[common->localOG[ogIndex]];
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
          (pointX * pointX + pointY * pointY + LValue * LValue);
}


void GraphSlamCPU::prepareLocalMap() {

   double cosTh = cos(localMaps[currentLocalMap].currentGlobalPosTh);
   double sinTh = sin(localMaps[currentLocalMap].currentGlobalPosTh);

   localMaps[currentLocalMap].robotMapCentreX = common->currentOffsetX/2.0;
   localMaps[currentLocalMap].robotMapCentreY = common->currentOffsetY/2.0;
   convertToGlobalCoord(localMaps[currentLocalMap].robotMapCentreX, localMaps[currentLocalMap].robotMapCentreY,
         localMaps[currentLocalMap].currentGlobalPosX, localMaps[currentLocalMap].currentGlobalPosY,
         localMaps[currentLocalMap].currentGlobalPosTh, &(localMaps[currentLocalMap].globalRobotMapCentreX),
         &(localMaps[currentLocalMap].globalRobotMapCentreY));
   
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

   /*cout << "Internal covar: ";
   int yy, xx;
   for (yy = 0; yy < 3; yy++) {
      for (xx = 0; xx < 3; xx++) {
         cout << localMaps[currentLocalMap].internalCovar[yy][xx] << " ";
      }
   }
   cout << endl;*/

   //Fix up the hessian matrixpreviously calculated
   double a[3][3];
   double b[3][3];
   double c[3][3];
   //if (currentLocalMap > 0) {
      //Need to rotate the hessian matrix as it was calculated in the opposite displacement
      //to the actual move - rotate info matrix b by RbR^T where R is the homogeneous 
      //rotation matrix of the angle of the last move
      
      int x,y;

      localMaps[currentLocalMap].mapCentreX = 
         (common->minMapRangeX + common->maxMapRangeX) / 2.0;
      localMaps[currentLocalMap].mapCentreY = 
         (common->minMapRangeY + common->maxMapRangeY) / 2.0;

      //Set the parentInfo matrix and the global covar

      int lastI = localMaps[currentLocalMap].scans.size();
      for (y = 0; y < 3; y++) {
         for (x = 0; x < 3; x++) {
            //a[y][x] = localMaps[currentLocalMap].internalCovar[y][x] / (double)lastI;
            //a[y][x] = (localMaps[currentLocalMap].scans[lastI]->covar[y][x] * 
            //   localMaps[currentLocalMap].scans[lastI]->covar[y][x]) * 1000000;
            
            //a[y][x] = ((localMaps[currentLocalMap].internalCovar[y][x] / (double) lastI) * 
            //   (localMaps[currentLocalMap].internalCovar[y][x] / (double) lastI)) * InformationScaleFactor;
            //a[y][x] = (localMaps[currentLocalMap].internalCovar[y][x] / (double) lastI) * 10000;
            a[y][x] = ((localMaps[currentLocalMap].internalCovar[y][x] / (double) lastI) * 
               (localMaps[currentLocalMap].internalCovar[y][x] / (double) lastI)) * 100000;
         }
      }

      invert3x3Matrix(a, localMaps[currentLocalMap].parentInfo);
      //invert3x3Matrix(localMaps[currentLocalMap].scans[lastI]->covar, localMaps[currentLocalMap].parentInfo);
      cout << "Info of standard constraint: ";
      for (y = 0; y < 3; y++) {
         for (x = 0; x < 3; x++) {
            cout << localMaps[currentLocalMap].parentInfo[y][x] << " ";
         }
      }
      cout << endl;
      
            
      //invert3x3Matrix(localMaps[currentLocalMap].internalCovar, localMaps[currentLocalMap].parentInfo);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
      mult3x3Matrix(a, localMaps[currentLocalMap].internalCovar, c);
      //transpose the parent rotation matrix
      double temp = a[1][0];
      a[1][0] = a[0][1];
      a[0][1] = temp;
      mult3x3Matrix(c, a, localMaps[currentLocalMap].globalCovar);

       

      /*double cosTh = cos(- localMaps[currentLocalMap].parentOffsetTh);
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
      */
      common->infoCount = 0;

   if (currentLocalMap > 0) {
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

void GraphSlamCPU::transpose3x3Matrix(double a[3][3], double res[3][3]) {
   for (int y = 0; y < 3; y++) {
      for (int x = 0; x < 3; x++) {
         res[y][x] = a[x][y];
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

inline void GraphSlamCPU::convertToGlobalCoord(double x, double y, double localPosX,
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

   if (localMaps[currentLocalMap].isFeatureless) {
      return;
   }

   for(int globalWarp = 0; globalWarp < nextLocalMap; globalWarp++) {
      if (globalWarp != currentLocalMap && globalWarp != localMaps[currentLocalMap].indexParentNode 
            && !localMaps[globalWarp].isFeatureless) {
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
         convertToGlobalCoord(localMaps[currentLocalMap].robotMapCentreX, 
                           localMaps[currentLocalMap].robotMapCentreY, 
                           localMaps[currentLocalMap].currentGlobalPosX,
                           localMaps[currentLocalMap].currentGlobalPosY,
                           localMaps[currentLocalMap].currentGlobalPosTh,
                           &mapCurPosX, &mapCurPosY);
         convertToGlobalCoord(localMaps[globalWarp].robotMapCentreX, 
                           localMaps[globalWarp].robotMapCentreY, 
                           localMaps[globalWarp].currentGlobalPosX,
                           localMaps[globalWarp].currentGlobalPosY,
                           localMaps[globalWarp].currentGlobalPosTh,
                           &mapOtherPosX, &mapOtherPosY);
         /*cout << "covar of map " << currentLocalMap << " to map " << globalWarp <<
            " is " << totalCovar[0][0] << " " << totalCovar[1][1] << " pos " <<
            mapCurPosX << " " << mapCurPosY << " " << mapOtherPosX << " " << mapOtherPosY << endl;*/
         if (fabs(mapCurPosX - mapOtherPosX) < totalCovar[0][0] + LocalMapDistance &&
               fabs(mapCurPosY - mapOtherPosY) < totalCovar[1][1] + LocalMapDistance) {
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
               double cosTh = cos(-maxTheta);
               double sinTh = sin(-maxTheta);
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

void GraphSlamCPU::alignICP(int otherMap, int mIndex, int currentMap) {
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
         searchFactor = 4;
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
      int matchIndex = findMatchingPoint(transformedPointX, transformedPointY, searchFactor, currentMap);
      //If found a matching point, calculate its contribution to the match
      if (matchIndex >= 0) {
         double x12, y12, k, temp;
         //lPoint is the point on the map ie. mapPoint
         //point is the laser point not in the map ie. transformedPoint
         double mapPointX = localMaps[currentMap].pointsX[matchIndex];
         double mapPointY = localMaps[currentMap].pointsY[matchIndex];
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

void GraphSlamCPU::calculateICPMatrix(int matchIndex, bool fullLoop, int currentMap) {
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
         cout << "Too many iterations " << common->numIterations << " " << common->goodCount << endl;
      } else if (!fullLoop && fabs(shift[0]) <= MaxErrorDisp &&
            fabs(shift[1]) <= MaxErrorDisp && fabs(shift[2]) <= MaxErrorTheta) {
         common->matchSuccess = 1;
         finished = 1;
         cout << "Finished success with temp loop " << common->goodCount << endl;
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
         double constraintTh = -common->potentialMatchTh[matchIndex];
         double cosTh = cos(-common->potentialMatchTh[matchIndex]);
         double sinTh = sin(-common->potentialMatchTh[matchIndex]);
         double constraintX = (cosTh * -common->potentialMatchX[matchIndex] -
                                                  sinTh * -common->potentialMatchY[matchIndex]);
         double constraintY = (sinTh * -common->potentialMatchX[matchIndex] +
                                                  cosTh * -common->potentialMatchY[matchIndex]);
         ANGNORM(constraintTh);
         int overlapNum;
         //if (!fullLoop) { tempO = true; }
         double score = evaluateMapMatch(currentMap, common->potentialMatches[matchIndex],
               constraintX, constraintY, constraintTh, &overlapNum);

         double angleDiff = localMaps[currentMap].currentGlobalPosTh - 
            (localMaps[common->potentialMatches[matchIndex]].currentGlobalPosTh + constraintTh);
         ANGNORM(angleDiff);
         bool stopMatch = false;
         if (PreventMatchesSymmetrical && (angleDiff < -3.0 * M_PI / 4.0 || angleDiff > 3.0 * M_PI / 4.0)) {
            //cout << "***********************************************************" << endl;
            cout << "***Skipping match " << currentMap << " " << angleDiff << endl;
            //cout << "***********************************************************" << endl;
            stopMatch = true;
         }


         //if (!fullLoop) { tempO = false; }
         cout << "The matching score is......" << score << endl;
         if (((fullLoop && score < FreeAreaThreshold) || (!fullLoop && score < previousScore && score < FreeAreaThreshold && score != -1 && overlapNum > OverlapThreshold)) && !stopMatch) {

            //If successful match, add match information to data structures
            int mIndex = common->numConstraints;
            common->numConstraints++;
            int loopIndex = common->numLoopConstraints;
            common->numLoopConstraints++;
            common->constraintType[mIndex] = 1;
            common->constraintIndex[mIndex] = loopIndex;
            common->loopConstraintI[loopIndex] = common->potentialMatches[matchIndex];
            common->loopConstraintJ[loopIndex] = currentMap;
            if (fullLoop) {
               common->loopConstraintParent[loopIndex] = common->potentialMatchParent[matchIndex];
               common->loopConstraintFull[loopIndex] = true;
               common->loopConstraintWeight[loopIndex] = 1;
            } else {
               int minLevel = min(localMaps[currentMap].treeLevel, localMaps[common->potentialMatches[matchIndex]].treeLevel);

               int mapIndex = currentMap;
               while (localMaps[mapIndex].treeLevel > minLevel) {
                  mapIndex = localMaps[mapIndex].indexParentNode;
               }
               int mapIndexGlobal = common->potentialMatches[matchIndex];
               while (localMaps[mapIndexGlobal].treeLevel > minLevel) {
                  mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
               }
               while (mapIndex != mapIndexGlobal) {
                  mapIndex = localMaps[mapIndex].indexParentNode;
                  mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
               }
               common->loopConstraintParent[loopIndex] = mapIndex;
               common->loopConstraintFull[loopIndex] = false;
               //common->loopConstraintWeight[loopIndex] = overlapNum / (double) localMaps[common->potentialMatches[matchIndex]].numPoints;
               common->loopConstraintWeight[loopIndex] = 1;

               cout << "Loop constraint weight is: " << common->loopConstraintWeight[loopIndex] << " " << 
                  localMaps[common->potentialMatches[matchIndex]].numPoints << endl;

            }

            //set the min opt level
            int parentIndex = common->loopConstraintParent[loopIndex];
            int curMinOptLevel = localMaps[parentIndex].minOptNode;
            int mapIndex = currentMap;
            while (mapIndex != parentIndex) {
               localMaps[mapIndex].minOptNode = curMinOptLevel;
               mapIndex = localMaps[mapIndex].indexParentNode;
            }
            mapIndex = common->loopConstraintI[loopIndex];
            while (mapIndex != parentIndex) {
               localMaps[mapIndex].minOptNode = curMinOptLevel;
               mapIndex = localMaps[mapIndex].indexParentNode;
            }
            //end of set min opt level

            common->loopConstraintInfo[loopIndex][0][0] = 0;
            common->loopConstraintInfo[loopIndex][0][1] = 0;
            common->loopConstraintInfo[loopIndex][0][2] = 0;
            common->loopConstraintInfo[loopIndex][1][0] = 0;
            common->loopConstraintInfo[loopIndex][1][1] = 0;
            common->loopConstraintInfo[loopIndex][1][2] = 0;
            common->loopConstraintInfo[loopIndex][2][0] = 0;
            common->loopConstraintInfo[loopIndex][2][1] = 0;
            common->loopConstraintInfo[loopIndex][2][2] = 0;
            common->loopConstraintThetaDisp[loopIndex] = constraintTh;
            common->loopConstraintXDisp[loopIndex] = constraintX;
            common->loopConstraintYDisp[loopIndex] = constraintY;

            if (fullLoop) {
               common->currentOffsetTh += common->loopConstraintThetaDisp[loopIndex];
               double tempX = common->currentOffsetX - common->potentialMatchX[matchIndex]; 
               double tempY = common->currentOffsetY - common->potentialMatchY[matchIndex];
               common->currentOffsetX = cosTh * tempX - sinTh * tempY;
               common->currentOffsetY = sinTh * tempX + cosTh * tempY;

               lastFullLoopIndex = mIndex;
               previousINode = common->loopConstraintI[loopIndex];
            }
          
            //common->loopConstraintXDisp[loopIndex] = - common->potentialMatchX[matchIndex];
            //common->loopConstraintYDisp[loopIndex] = - common->potentialMatchY[matchIndex];
            if(LocalMapCombine) {
               common->combineIndex = common->potentialMatches[matchIndex];
            }
         } else {
            common->matchSuccess = -1;
            cout << "Match failed because of free area check " << score << endl;
         }
      } else if (common->combineIndex >= 0 && common->matchSuccess < 0) {
         int cIndex = common->numConstraints;
         common->numConstraints++;
         common->constraintType[cIndex] = 0;
         common->constraintIndex[cIndex] = currentMap;
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

   /*invert3x3Matrix(common->loopConstraintInfo[cIndex], a);
   for (int y = 0; y < 3; y++) {
      for (int x = 0; x < 3; x++) {
         a[y][x] /= 10;
      }
   }
   invert3x3Matrix(a, common->loopConstraintInfo[cIndex]);*/
   cout << "the info matrix of the loop closing is: ";
   for (int y = 0; y < 3; y++) {
      for (int x = 0; x < 3; x++) {
         cout << common->loopConstraintInfo[cIndex][y][x] << " ";
      }
   }
   cout << endl;

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


   /*if (common->infoCount < InformationScaleFactor) {
      common->infoCount = InformationScaleFactor;
   }*/
   
   for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
 
         //Adjust the parent info matrix to be relative to the parent instead
         //of relative to the current node
         //b[y][x] = common->loopConstraintInfo[cIndex][y][x] /= 
         //               ((double)common->infoCount / (double)InformationScaleFactor);
         b[y][x] = common->loopConstraintInfo[cIndex][y][x];
      }
   }
   mult3x3Matrix(a, b, c);
   a[1][0] *= -1;
   a[0][1] *= -1;
   mult3x3Matrix(c, a, b);

   /*double det = b[0][0] * (b[1][1] * b[2][2] - b[1][2] * b[2][1]) -
                b[0][1] * (b[1][0] * b[2][2] - b[1][2] * b[2][0]) +
                b[0][2] * (b[1][0] * b[2][1] - b[1][1] * b[2][0]);*/
   double det = 1;

   for(int y = 0; y < 3; y++) {
      for(int x = 0; x < 3; x++) {
         common->loopConstraintInfo[cIndex][y][x] = b[y][x] / det;
      }
   }
   common->infoCount = 0;


}

bool GraphSlamCPU::findTempMatches() {
   if (localMaps[currentLocalMap].isFeatureless) {
      return false;
   }

   int mIndex = 0;
   double xCur = localMaps[currentLocalMap].globalRobotMapCentreX;
   double yCur = localMaps[currentLocalMap].globalRobotMapCentreY;

   bool returnVal = false;
   double distanceThreshold = DistanceOverlapThreshold;
   for (mIndex = 0; mIndex < nextLocalMap; mIndex++) {
      if (mIndex != currentLocalMap && localMaps[currentLocalMap].indexParentNode != mIndex &&
            !localMaps[mIndex].isFeatureless && fabs(xCur - localMaps[mIndex].globalRobotMapCentreX) < distanceThreshold
            && fabs(yCur - localMaps[mIndex].globalRobotMapCentreY) < distanceThreshold) {
         //Map is in the right area
         if (performTempMatch(currentLocalMap, mIndex)) {
            returnVal = true;
         }
      }
   }
   return returnVal;
}

bool GraphSlamCPU::performTempMatch(int currentMap, int testMap) {
   int overlapNum;
   //want to transform old match points into current map
   double globalOffTh = localMaps[currentMap].currentGlobalPosTh - localMaps[testMap].currentGlobalPosTh;
   double globalOffX = localMaps[currentMap].currentGlobalPosX - localMaps[testMap].currentGlobalPosX;
   double globalOffY = localMaps[currentMap].currentGlobalPosY - localMaps[testMap].currentGlobalPosY;
   double cosTh = cos(-localMaps[testMap].currentGlobalPosTh);
   double sinTh = sin(-localMaps[testMap].currentGlobalPosTh);
   double offX = cosTh * globalOffX - sinTh * globalOffY;
   double offY = sinTh * globalOffX + cosTh * globalOffY;

   double score = evaluateMapMatch(currentMap, testMap, offX, offY, globalOffTh, &overlapNum);
   cout << "Score for map " << currentMap << " to " << testMap << " is " << score << " with overlap " << overlapNum << endl;

   if (overlapNum > OverlapThreshold) {
      previousScore = score;
      double cosM = cos(-globalOffTh);
      double sinM = sin(-globalOffTh);
      common->potentialMatchTh[0] = -globalOffTh;
      common->potentialMatchX[0] = cosM * - offX - sinM * - offY;
      common->potentialMatchY[0] = sinM * - offX + cosM * - offY;
      common->potentialMatches[0] = testMap;
      
      ros::WallTime t1 = ros::WallTime::now();
      int matchSuccess = 0;
      int c = 0;
      while (matchSuccess == 0) {
         alignICP(testMap, 0, currentMap);
         calculateICPMatrix(0, false, currentMap);
         matchSuccess = common->matchSuccess;
         c++;
      }
      ros::WallTime t2 = ros::WallTime::now();
      ros::WallDuration tote = t2 - t1;
      cout << "Number of iterations in alignment: " << c << " " << matchSuccess << " time: " << tote.toSec() * 1000.0f << "ms" << endl;
      if (matchSuccess == 1) {
         //Alignment was successful
         getHessianMatch(numConstraints);
         finaliseInformationMatrix();
         numConstraints++;
         return true;
      }
   }
   return false;
}

void GraphSlamCPU::evaluateTempConstraints() {

   double dispThresh = TempConstraintMovementXY;
   double thThresh = TempConstraintMovementTh;

   for (int i = 0; i < common->numLoopConstraints; i++) {
      if (!common->loopConstraintFull[i]) {
         int iNode = common->loopConstraintI[i];
         int jNode = common->loopConstraintJ[i];
         double cosTh = cos(localMaps[iNode].currentGlobalPosTh);
         double sinTh = sin(localMaps[iNode].currentGlobalPosTh);
         double tempX = cosTh * common->loopConstraintXDisp[i] - sinTh * common->loopConstraintYDisp[i] 
            + localMaps[iNode].currentGlobalPosX;
         double tempY = sinTh * common->loopConstraintXDisp[i] + cosTh * common->loopConstraintYDisp[i]
            + localMaps[iNode].currentGlobalPosY;
         double tempTh = common->loopConstraintThetaDisp[i] + localMaps[iNode].currentGlobalPosTh;
         ANGNORM(tempTh);
         if (fabs(tempX - localMaps[jNode].currentGlobalPosX) > dispThresh ||
               fabs(tempY - localMaps[jNode].currentGlobalPosY) > dispThresh ||
               fabs(tempTh - localMaps[jNode].currentGlobalPosTh) > thThresh) {
            cout << "Removing link " << jNode << " " << iNode << " " << tempX << " " << tempY << " " << tempTh << " " <<
              localMaps[jNode].currentGlobalPosX << " " << localMaps[jNode].currentGlobalPosY << " " <<
              localMaps[jNode].currentGlobalPosTh << endl;
            common->loopConstraintWeight[i] = 0;
         }
      }
   }
}

bool GraphSlamCPU::findChangedPosMatches(int mapNum) {
   if (localMaps[mapNum].isFeatureless) {
      return false;
   }

   int mIndex = 0;
   double xCur = localMaps[mapNum].globalRobotMapCentreX;
   double yCur = localMaps[mapNum].globalRobotMapCentreY;

   bool returnVal = false;
   double distanceThreshold = DistanceOverlapThreshold;
   for (mIndex = 0; mIndex < mapNum; mIndex++) {
      if (mIndex != currentLocalMap && localMaps[mapNum].indexParentNode != mIndex &&
            !localMaps[mIndex].isFeatureless && fabs(xCur - localMaps[mIndex].globalRobotMapCentreX) < distanceThreshold
            && fabs(yCur - localMaps[mIndex].globalRobotMapCentreY) < distanceThreshold) {
         //Time to be super dodgy
         for(int i = 0; i < DimLocalOG * DimLocalOG; i++) {
            common->localOG[i] = -1;
         }
         for (int i = 0; i < localMaps[mapNum].numPoints; i++) {
            int index = getLocalOGIndex(localMaps[mapNum].pointsX[i], localMaps[mapNum].pointsY[i]);
            if (index >= 0) {
               common->localOG[index] = i;
               common->grid[index].gradX = localMaps[mapNum].gradX[i];
               common->grid[index].gradY = localMaps[mapNum].gradY[i];
            }
         }

         //Map is in the right area
         if (performTempMatch(mapNum, mIndex)) {
            returnVal = true;
            cout << "****Found a match by going back through map. Maps: " << mapNum << " to " << mIndex << endl;
         }
      }
   }
   if (returnVal) {
      lastFullLoopIndex = common->numConstraints - 1;
   }
   return returnVal;
}

void GraphSlamCPU::getGlobalHessianMatrix() {
   //Reset the changeInPos variables before the optimise step
   for (int index = 0; index < nextLocalMap; index++) {
      localMaps[index].changeInPos[0] = 0;
      localMaps[index].changeInPos[1] = 0;
      localMaps[index].changeInPos[2] = 0;
      localMaps[index].numConstraints = 0;
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
         continue;
      }
      
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
               //b[y][x] = localMaps[constraintIndex].parentInfo[y][x];
               b[y][x] = localMaps[iNode].parentInfo[y][x];
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

void GraphSlamCPU::calculateOptimisationChange(int numIterations, int type) {

   double a[3][3];
   double b[3][3];
   double c[3][3];
   double constraint[3];
   double residual[3];

   /*int order[10];
   int parentLevel[10];
   int num = 0;
   for (int i = 0; i < common->numConstraints; i++) {
      if (common->constraintType[i] == 1) {
         int j, k;
         int par = localMaps[common->loopConstraintParent[common->constraintIndex[i]]].treeLevel;
         for (j = num; j > 0 && parentLevel[j-1] > par; j--) {
            parentLevel[j] = parentLevel[j-1];
            order[j] = order[j-1];
         }
         parentLevel[j] = par;
         order[j] = i;
         num++;
      }
   }
         
   for (int kk = 0; kk < num; kk++) {
      int globalWarp = order[kk];*/
   int startIndex = 0;
   if (type == -1) {
      startIndex = lastFullLoopIndex + 1;
   }
   for (int globalWarp = startIndex; globalWarp < common->numConstraints; globalWarp++) {

      int constraintType = common->constraintType[globalWarp];
      int constraintIndex = common->constraintIndex[globalWarp];
      int iNode;
      int jNode;
      int parentIndex;
      int x,y;
      double weight;

      if (constraintType != 1) {
         continue;
      }
      
      if (constraintType == 1) {
         if (type == 1 && !common->loopConstraintFull[constraintIndex]) {
            continue;
         }
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
         weight = common->loopConstraintWeight[constraintIndex];
         if (weight == 0) {
            continue;
         }
         
      } else {
         iNode = localMaps[constraintIndex].indexParentNode;
         parentIndex = iNode;
         jNode = constraintIndex;
         for (y = 0; y < 3; y++) {
            for (x = 0; x < 3; x++) {
               b[y][x] = localMaps[iNode].parentInfo[y][x];
            }
         }
         constraint[0] = localMaps[constraintIndex].parentOffsetX;
         constraint[1] = localMaps[constraintIndex].parentOffsetY;
         constraint[2] = localMaps[constraintIndex].parentOffsetTh;
         weight = 1;
      }
      int pathLength = (localMaps[iNode].treeLevel - localMaps[parentIndex].treeLevel) +
                       (localMaps[jNode].treeLevel - localMaps[parentIndex].treeLevel);

      if (type == -1) {
         pathLength = localMaps[jNode].treeLevel - localMaps[previousINode].treeLevel;
      }

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
         dm[warpIndex] = 0;
         residual[warpIndex] = 0;
         tempNode = iNode;
         while (tempNode != parentIndex) {
            tempPos = getGlobalPosIndex(tempNode, warpIndex);
            if (type != -1) {
               dm[warpIndex] += 1 / common->graphHessian[tempNode][warpIndex];
            }
            tempNode = localMaps[tempNode].indexParentNode;
            residual[warpIndex] += getGlobalPosIndex(
                  tempNode, warpIndex) - tempPos;
         }
         tempNode = jNode;
         bool reachedEnd = false;
         while (tempNode != parentIndex) {
            if (type == -1 && tempNode == previousINode) {
               reachedEnd = true;
            }
            if (!reachedEnd) {
               dm[warpIndex] += 1 / common->graphHessian[tempNode][warpIndex];
            }
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

      if (PreventMatchesSymmetrical && (residual[2] > 3.0 * M_PI / 4.0 || residual[2] < -3.0 * M_PI / 4.0)) {
         cout << "Fixing residual angles " << residual[2] << " " << iNode << " " << jNode << endl;
         common->loopConstraintWeight[constraintIndex] = 0;
         continue;
         //residual[2] -= M_PI;
      } /*else if (residual[2] < - 3.0 * M_PI / 4.0) {
         cout << "Fixing residual angles" << endl;
         continue;
         //residual[2] += M_PI;
      }*/
      if (numIterations <= 2) {cout << iNode << " " << jNode << " Residual is: " << residual[0] << " " << residual[1] << " " << residual[2] << endl; }
      //cout << "Constraint is: " << constraint[0] << " " << constraint[1] << " " << constraint[2] << endl;
      mult3x3Matrix(a, b, c);
      a[1][0] *= -1;
      a[0][1] *= -1;
      mult3x3Matrix(c, a, b);

      for (int warpIndex = 0; warpIndex < 3; warpIndex++) {
         commonValue[warpIndex] = b[warpIndex][0] * residual[0] +
                       b[warpIndex][1] * residual[1] +
                       b[warpIndex][2] * residual[2];

         scaleFactor[warpIndex] = 1 / ((double) numIterations * 
               common->scaleFactor[warpIndex]/* * ((double) nextLocalMap - 1)*/);
         /*if (constraintType != 1) {
            scaleFactor[warpIndex] /= 100;
         }*/

         //cout << "Vals are: " << commonValue[warpIndex] << " " << scaleFactor[warpIndex] << " " <<
         //   dm[warpIndex] << endl;
      
         //Now do the calculations for each node in the constraint
         double adjust = scaleFactor[warpIndex] * pathLength * commonValue[warpIndex];
         if (warpIndex == 2) {
            ANGNORM(adjust);
         }
         if (fabs(adjust) > fabs(residual[warpIndex])) {
            //cout << "Throttle on " << globalWarp << " " << warpIndex << " " << adjust << " " << residual[warpIndex] << endl;
            adjust = residual[warpIndex];
         }
         tempNode = iNode;

         if (type != -1) {
            while (tempNode != parentIndex) {
               //tempNode = localMaps[tempNode].indexParentNode;
               double value = weight * adjust * dm[warpIndex] * 
                             1/common->graphHessian[tempNode][warpIndex] * -1;
               localMaps[tempNode].changeInPos[warpIndex] += value;
               if (warpIndex == 0) {
                  localMaps[tempNode].numConstraints++;
               }
               tempNode = localMaps[tempNode].indexParentNode;
            }
         }
         tempNode = jNode;
         while (tempNode != parentIndex) {
            if (type == -1 && tempNode == previousINode) {
               break;
            }
            double value = weight * adjust * dm[warpIndex] * 
                          1/common->graphHessian[tempNode][warpIndex];
            //cout << "Fraction for node " << tempNode << " is " << dm[warpIndex] * 1/common->graphHessian[tempNode][warpIndex] << 
            //   " " << dm[warpIndex] << " " << warpIndex << " " << localMaps[tempNode].parentInfo[warpIndex][warpIndex] << endl;
            localMaps[tempNode].changeInPos[warpIndex] += value;
            if (warpIndex == 0) {
               localMaps[tempNode].numConstraints++;
            }
            tempNode = localMaps[tempNode].indexParentNode;
         }
      }
      //updateGlobalPositions();
   }
}

//Type = 1 is only optimsing from parent of the full loop closure
//Type = -1 is for partial loop closures only - Only since last full loop closure
//Type = 0 is everything
void GraphSlamCPU::optimiseGraph(int type) {


   double previousError = INFINITY;
   double lambda = 1.0;
   double currentError = 0;
   bool rewind = false;


   //Calculate the starting node
   int startingNode;
   int startingIndex = 0;

   //type = 0;
   
   if (type == -1) {
      //startingNode = previousINode;
      int constraintIndex = common->constraintIndex[lastFullLoopIndex + 1];
      //startingNode = common->loopConstraintJ[constraintIndex];
      if (common->constraintType[lastFullLoopIndex + 1] == 1) {
         cout << "Error: constraint after loop cloosing constraint is another loop closing one" << endl;
      }
      startingNode = constraintIndex - 1;

      //if (common->loopConstraintI[constraintIndex] != localMaps[startingNode].indexParentNode) {
         cout << "Last full loop constraint index " << previousINode <<
            " " << startingNode << " " << lastFullLoopIndex << endl;
      //}

      startingIndex = lastFullLoopIndex + 1;
      if (lastFullLoopIndex < 0) {
         startingNode = 0;
         startingIndex = 0;
      }
   } else if (type == 1) {
      int constraintIndex = common->constraintIndex[common->numConstraints - 1];
      //startingNode = common->loopConstraintParent[constraintIndex];
      startingNode = localMaps[common->loopConstraintParent[constraintIndex]].minOptNode;
      cout << "Full loop: " << startingNode << " " << common->loopConstraintI[constraintIndex] <<
         " " << common->loopConstraintJ[constraintIndex] << endl;

      for (int i = 0; i < common->numConstraints; i++) {
         int constraintType = common->constraintType[i];
         constraintIndex = common->constraintIndex[i];
         if (constraintType != 1 && localMaps[constraintIndex].indexParentNode == startingNode) {
            startingIndex = i;
            break;
         }
      }
   } else {
      startingNode = 0;
      startingIndex = 0;
   }
   cout << "starting node is : " << startingNode << " " << startingIndex << " " << type << endl;
         
   //Set the number of rows in the matrix (3 * the number of maps
   //the will be optimising)
   int numMaps = nextLocalMap - startingNode;
   int nrows = 3 * numMaps;
   //Overestimate of number of cells that will be stored in the map
   int nzmax = (numMaps + (common->numConstraints - startingNode) * 2) * 9;

   //Allocate the dense b array
   double *b = (double *) malloc(sizeof(double) * nrows);
   //Allocate the sparse hessian matrix
   cs *sparseH = cs_spalloc(nrows, nrows, nzmax, 1, 1);

   double *oldB = (double *) malloc(sizeof(double) * nrows);

   double startX, startY, startTh;
   if (type == -1) {
      startX = localMaps[previousINode].currentGlobalPosX;
      startY = localMaps[previousINode].currentGlobalPosY;
      startTh = localMaps[previousINode].currentGlobalPosTh;
   } else {
      startX = localMaps[startingNode].currentGlobalPosX;
      startY = localMaps[startingNode].currentGlobalPosY;
      startTh = localMaps[startingNode].currentGlobalPosTh;
   }  

   for (int numIterations = 0; numIterations < MaxNumOfOptimisationIts; numIterations++) {

      //Initialise areas of Hessian and b where more than one constraint
      //will write
      memset(b, 0, sizeof(double) * nrows);
      int count = 0;
      for (int x = startingNode; x < nextLocalMap; x++) {
         int off = (x - startingNode) * 3;
         for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 3; i++, count++) {
               sparseH->i[count] = off + i;
               sparseH->p[count] = off + j;
               sparseH->x[count] = 0;
               if (i == j) {
                  sparseH->x[count] = lambda;
               }
            }
         }
      }

      double maxX = 0;
      double maxY = 0;
      double maxTh = 0;
      currentError = 0;

      //Set the values in H and b
      for (int constraintI = startingIndex; constraintI < common->numConstraints; constraintI++) {

         int constraintType = common->constraintType[constraintI];
         int constraintIndex = common->constraintIndex[constraintI];
         int iNode;
         int jNode;

         double weight;
         //Information matrix for constraint
         double info[3][3];
         //Actual constraint and the error in the constraint
         double constraint[3];
         double error[3];
         //The two blocks of a constraint's jacobian matrix
         double A[3][3];
         double B[3][3];
         //Transposes of the jacobians
         double AT[3][3];
         double BT[3][3];
         //Temp stores
         double temp[3][3];
         
         double ATA[3][3];
         double ATB[3][3];
         double BTA[3][3];
         double BTB[3][3];

         if (constraintType == 1) { //Loop closing constraint
            //Just doing full loop closures, so skip partial closures
            if (type == 1 && !common->loopConstraintFull[constraintIndex]) {
               continue;
            }
            iNode = common->loopConstraintI[constraintIndex];
            jNode = common->loopConstraintJ[constraintIndex];
            for (int y = 0; y < 3; y++) {
               for (int x = 0; x < 3; x++) {
                  info[y][x] = common->loopConstraintInfo[constraintIndex][y][x];
               }
            }
            constraint[0] = common->loopConstraintXDisp[constraintIndex];
            constraint[1] = common->loopConstraintYDisp[constraintIndex];
            constraint[2] = common->loopConstraintThetaDisp[constraintIndex];
         
            weight = common->loopConstraintWeight[constraintIndex];
            if (weight == 0) {
               continue;
            }
            //Doing partial loop closure
            if (type == -1) {
               int mapIndex;
               int parentIndex = common->loopConstraintParent[constraintIndex];
               for (mapIndex = jNode; mapIndex != previousINode && mapIndex != parentIndex;
                     mapIndex = localMaps[mapIndex].indexParentNode);

               if (mapIndex != iNode) {
                  //Transform constraint to be from mapIndex instead of iNode
                  if (numIterations == 0) {
                     cout << "Moving iNode of partial constraint. Orig iNode is " << iNode << 
                        " new is " << mapIndex << endl;
                  }

                  double tempX, tempY;
                  convertToGlobalCoord(constraint[0], constraint[1], 
                        localMaps[iNode].currentGlobalPosX, localMaps[iNode].currentGlobalPosY,
                        localMaps[iNode].currentGlobalPosTh, &tempX, &tempY);
                  tempX -= startX;
                  tempY -= startY;
                  double cosTh = cos(-startTh);
                  double sinTh = sin(-startTh);
                  constraint[0] = tempX * cosTh - tempY * sinTh;
                  constraint[1] = tempX * sinTh + tempY * cosTh;
                  double rotateAngle = localMaps[iNode].currentGlobalPosTh - startTh;
                  constraint[2] += rotateAngle;

                 
                  /*double rotateAngle = localMaps[iNode].currentGlobalPosTh -
                     localMaps[mapIndex].currentGlobalPosTh;
                  double cosTh = cos(rotateAngle);
                  double sinTh = sin(rotateAngle);

                  double tempX = cosTh * constraint[0] - sinTh * constraint[1];
                  double tempY = sinTh * constraint[0] + cosTh * constraint[1];
                  constraint[0] = tempX + localMaps[mapIndex].currentGlobalPosX - 
                     localMaps[iNode].currentGlobalPosX;
                  constraint[1] = tempY + localMaps[mapIndex].currentGlobalPosY -
                     localMaps[iNode].currentGlobalPosY;
                  constraint[2] += rotateAngle;*/

                  //cosTh = cos(startTh);
                  //sinTh = sin(startTh);

                  double tempRot[3][3];
                  double temp[3][3];
                  tempRot[0][0] = cosTh;
                  tempRot[1][1] = cosTh;
                  tempRot[1][0] = sinTh;
                  tempRot[0][1] = -sinTh;
                  tempRot[2][2] = 1;
                  tempRot[0][2] = 0;
                  tempRot[1][2] = 0;
                  tempRot[2][0] = 0;
                  tempRot[2][1] = 0;

                  mult3x3Matrix(tempRot, info, temp);
                  tempRot[0][1] *= -1;
                  tempRot[1][0] *= -1;
                  mult3x3Matrix(temp, tempRot, info);

                  iNode = mapIndex;
               }

            }
         } else {
            iNode = localMaps[constraintIndex].indexParentNode;
            jNode = constraintIndex;
            for (int y = 0; y < 3; y++) {
               for (int x = 0; x < 3; x++) {
                  info[y][x] = localMaps[iNode].parentInfo[y][x];
               }
            }
            constraint[0] = localMaps[constraintIndex].parentOffsetX;
            constraint[1] = localMaps[constraintIndex].parentOffsetY;
            constraint[2] = localMaps[constraintIndex].parentOffsetTh;
            weight = 1;
         }
         double sinI = sin(localMaps[iNode].currentGlobalPosTh);
         double cosI = cos(localMaps[iNode].currentGlobalPosTh);

         A[0][0] = - cosI;
         A[1][0] = sinI;
         A[2][0] = 0;
         A[0][1] = -sinI;
         A[1][1] = -cosI;
         A[2][1] = 0;
         A[0][2] = - localMaps[jNode].currentGlobalPosX * sinI +
                     localMaps[iNode].currentGlobalPosX * sinI +
                     localMaps[jNode].currentGlobalPosY * cosI -
                     localMaps[iNode].currentGlobalPosY * cosI;
         A[1][2] = - localMaps[jNode].currentGlobalPosX * cosI +
                     localMaps[iNode].currentGlobalPosX * cosI -
                     localMaps[jNode].currentGlobalPosY * sinI +
                     localMaps[iNode].currentGlobalPosY * sinI;
         A[2][2] = -1;

         B[0][0] = cosI;
         B[1][0] = -sinI;
         B[2][0] = 0;
         B[0][1] = sinI;
         B[1][1] = cosI;
         B[2][1] = 0;
         B[0][2] = 0;
         B[1][2] = 0;
         B[2][2] = 1;

         error[0] = (localMaps[jNode].currentGlobalPosX - localMaps[iNode].currentGlobalPosX) * cosI
                  + (localMaps[jNode].currentGlobalPosY - localMaps[iNode].currentGlobalPosY) * sinI
                  - constraint[0];
         error[1] = - (localMaps[jNode].currentGlobalPosX - localMaps[iNode].currentGlobalPosX) * sinI
                  + (localMaps[jNode].currentGlobalPosY - localMaps[iNode].currentGlobalPosY) * cosI
                  - constraint[1];
         error[2] = localMaps[jNode].currentGlobalPosTh - localMaps[iNode].currentGlobalPosTh
                  - constraint[2];
         ANGNORM(error[2]);
      
         if (PreventMatchesSymmetrical && (error[2] > 3.0 * M_PI / 4.0 || 
                  error[2] < -3.0 * M_PI / 4.0)) {
            cout << "Fixing residual angles " << error[2] << " " << iNode << " " << jNode << endl;
            common->loopConstraintWeight[constraintIndex] = 0;
            continue;
         }

         currentError += fabs((error[0]  * (error[0] * info[0][0] + error[1] * info[1][0] + error[2] * info[2][0]) +
                         error[1]  * (error[0] * info[0][1] + error[1] * info[1][1] + error[2] * info[2][1]) +
                         error[2]  * (error[0] * info[0][2] + error[1] * info[1][2] + error[2] * info[2][2])) *
                         weight);

                        

         if (numIterations == 0 || numIterations == MaxNumOfOptimisationIts - 1) {
            cout << "Error : " << iNode << " " << jNode << " is: " << error[0] << " " << error[1]
               << " " << error[2];
            if (fabs(error[0]) > 0.4 || fabs(error[1]) > 0.4 || fabs(error[2]) > 0.4) {
                  cout << " *****";
            }
            cout << endl;
         }

         transpose3x3Matrix(A, AT);
         transpose3x3Matrix(B, BT);

         if (type == -1 && iNode < startingNode) {
            iNode = startingNode;
         }

         mult3x3Matrix(AT, info, temp);
         //-= because b is really -b, as solving Hx = -b
         b[(iNode - startingNode) * 3] -= (temp[0][0] * error[0] + 
               temp[0][1] * error[1] + temp[0][2] * error[2]) * weight;
         b[(iNode - startingNode) * 3 + 1] -= (temp[1][0] * error[0] + 
               temp[1][1] * error[1] + temp[1][2] * error[2]) * weight;
         b[(iNode - startingNode) * 3 + 2] -= (temp[2][0] * error[0] + 
               temp[2][1] * error[1] + temp[2][2] * error[2]) * weight;

         mult3x3Matrix(temp, A, ATA);
         mult3x3Matrix(temp, B, ATB);

         mult3x3Matrix(BT, info, temp);
         b[(jNode - startingNode) * 3] -= (temp[0][0] * error[0] + 
               temp[0][1] * error[1] + temp[0][2] * error[2]) * weight;
         b[(jNode - startingNode) * 3 + 1] -= (temp[1][0] * error[0] + 
               temp[1][1] * error[1] + temp[1][2] * error[2]) * weight;
         b[(jNode - startingNode) * 3 + 2] -= (temp[2][0] * error[0] + 
               temp[2][1] * error[1] + temp[2][2] * error[2]) * weight;

         mult3x3Matrix(temp, A, BTA);
         mult3x3Matrix(temp, B, BTB);

         if (iNode - startingNode < 0 || jNode - startingNode < 0) {
            cout << "### We have a problem " << iNode << " " << jNode << " " << startingNode << endl;
         }

         for (int j = 0; j < 3; j++) {
            for (int i = 0; i < 3; i++) {
               sparseH->x[(iNode - startingNode) * 9 + j * 3 + i] += weight * ATA[i][j];
               sparseH->x[(jNode - startingNode) * 9 + j * 3 + i] += weight * BTB[i][j];

               sparseH->i[count] = (iNode - startingNode) * 3 + i;
               sparseH->p[count] = (jNode - startingNode) * 3 + j;
               sparseH->x[count] = weight * ATB[i][j];
               count++;
               sparseH->i[count] = (jNode - startingNode) * 3 + i;
               sparseH->p[count] = (iNode - startingNode) * 3 + j;
               sparseH->x[count] = weight * BTA[i][j];
               count++;
            }
         }
         //Set the number of entries in the sparse matrix
         sparseH->nz = count;

         //row index is T->i
         //column index is T->p
         //actual value is T->X
      }

      currentError /= numMaps;
      if (currentError > previousError && !rewind) {
         cout << "Unwinding change because bad " << currentError << " " << previousError << endl;
         rewind = true;
         for (int x = 0; x< numMaps; x++) {
            if (type == -1 && x == 0) {
               localMaps[previousINode].currentGlobalPosX -= oldB[x * 3];
               localMaps[previousINode].currentGlobalPosY -= oldB[x * 3 + 1];
               localMaps[previousINode].currentGlobalPosTh -= oldB[x * 3 + 2];
               ANGNORM(localMaps[previousINode].currentGlobalPosTh);
            } else {
               localMaps[x + startingNode].currentGlobalPosX -= oldB[x * 3];
               localMaps[x + startingNode].currentGlobalPosY -= oldB[x * 3 + 1];
               localMaps[x + startingNode].currentGlobalPosTh -= oldB[x * 3 + 2];
               ANGNORM(localMaps[x + startingNode].currentGlobalPosTh);
            }
            //oldB[x * 3] = 0;
            //oldB[x * 3 + 1] = 0;
            //oldB[x * 3 + 2] = 0;
         }
         numIterations--;
         lambda *= 2.0;
         continue; 
      } else {
         if (lambda > 1.0) {
            lambda /= 2.0;
         }
         previousError = currentError;
      }
      rewind = false;


      /*cout << "Count is: " << count << " nzmax is " << nzmax << endl;
      cout << "First square of matrix is: " << endl;
      for (int j = 0; j < 3; j++) {
         for(int i = 0; i < 3; i++) {
            cout << sparseH->x[j * 3 + i] << " ";
         }
      }
      cout << endl;
      cs_print(sparseH, 0);
      cout << "b is: " << endl;
      for (int i = 0; i < nextLocalMap; i++) {
         cout << b[i * 3] << " " << b[i*3 + 1] << " " << b[i*3 + 2] << endl;
      }*/

      //Compress H into sparse column form
      cs *compH = cs_compress(sparseH);
      
      //find the solution of the linear system
      //int retVal = cs_cholsol(0, compH, b);
      int retVal = cs_lusol(0, compH, b, 1);
      cs_spfree(compH);
      if (retVal != 1) {
         cout << "Colesky failed. Can't optimise " << retVal << endl;
         break;
         /*for (int x = 0; x < numMaps; x++) {
            b[x] = 0;
         }*/
      }

      //b has the solution!!
      for (int x = 0; x < numMaps; x++) {
         if (b[x * 3 + 2] > 100) {
            cout << "AAAAAAAAAAAARGGGG angle is big: " << b[x * 3 + 2] << endl;
         }
         ANGNORM(b[x * 3 + 2]);
         oldB[x * 3] = b[x* 3];
         oldB[x * 3 + 1] = b[x* 3 + 1];
         oldB[x * 3 + 2] = b[x* 3 + 2];
         //cout << "Map " << x << " before: " << localMaps[x].currentGlobalPosX << " " <<
         //   localMaps[x].currentGlobalPosY << " " << localMaps[x].currentGlobalPosTh << 
         //   " change: " << b[x*3] << " " << b[x * 3 + 1] << " " << b[x * 3 + 2] << endl;
         /*double cosTh = cos(b[x * 3 + 2]);
         double sinTh = sin(b[x * 3 + 2]);
         double oldX = localMaps[x].currentGlobalPosX;
         double oldY = localMaps[x].currentGlobalPosY;
         localMaps[x].currentGlobalPosX = oldX * cosTh - oldY * sinTh + b[x*3];
         localMaps[x].currentGlobalPosY = oldX * sinTh + oldY * cosTh + b[x*3 + 1];
         localMaps[x].currentGlobalPosTh += b[x * 3 + 2];*/

         if (type == -1 && x == 0) {
            localMaps[previousINode].currentGlobalPosX += b[x * 3];
            localMaps[previousINode].currentGlobalPosY += b[x * 3 + 1];
            localMaps[previousINode].currentGlobalPosTh += b[x * 3 + 2];
            ANGNORM(localMaps[previousINode].currentGlobalPosTh);
         } else {
            localMaps[x + startingNode].currentGlobalPosX += b[x * 3];
            localMaps[x + startingNode].currentGlobalPosY += b[x * 3 + 1];
            localMaps[x + startingNode].currentGlobalPosTh += b[x * 3 + 2];
            ANGNORM(localMaps[x + startingNode].currentGlobalPosTh);
         }

         if (fabs(b[x*3]) > maxX) {
            maxX = fabs(b[x * 3]);
         }
         if (fabs(b[x*3 + 1]) > maxY) {
            maxY = fabs(b[x * 3 + 1]);
         }
         if (fabs(b[x*3 + 2]) > maxTh) {
            maxTh = fabs(b[x * 3 + 2]);
         }
      }
      cout << "Finished iteration " << maxX << " " << maxY << " " << maxTh << " " << currentError << endl;
      if (maxX < MaxOptMoveXY && maxY < MaxOptMoveXY && maxTh < MaxOptMoveTh) {
         cout << "Finished optimising. Took " << (numIterations + 1) << " iterations" << endl;
         break;
      }
   }
   cs_spfree(sparseH);
   free(b);
   free(oldB);

   double offX, offY, offTh;
   if (type == -1) {
      offX = localMaps[previousINode].currentGlobalPosX - startX;
      offY = localMaps[previousINode].currentGlobalPosY - startY;
      offTh = localMaps[previousINode].currentGlobalPosTh - startTh;
   } else {
      offX = localMaps[startingNode].currentGlobalPosX - startX;
      offY = localMaps[startingNode].currentGlobalPosY - startY;
      offTh = localMaps[startingNode].currentGlobalPosTh - startTh;
      cout << "map " << startingNode << " is at: " << localMaps[startingNode].currentGlobalPosX << " " << 
         localMaps[startingNode].currentGlobalPosY << " " << localMaps[startingNode].currentGlobalPosTh << endl;
   }


   for(int i = startingNode; i < nextLocalMap; i++) {
      /*localMaps[i].currentGlobalPosX -= offX;
      localMaps[i].currentGlobalPosY -= offY;
      localMaps[i].currentGlobalPosTh -= offTh;*/
      if (type == -1 && i == startingNode) {
         continue;
      }
      double errX = localMaps[i].currentGlobalPosX - offX;
      double errY = localMaps[i].currentGlobalPosY - offY;
      double errTh = localMaps[i].currentGlobalPosTh - offTh;
      double sinTh = sin(-offTh);
      double cosTh = cos(-offTh);
      localMaps[i].currentGlobalPosX = cosTh * errX - sinTh * errY;
      localMaps[i].currentGlobalPosY = sinTh * errX + cosTh * errY;
      localMaps[i].currentGlobalPosTh = errTh;

      ANGNORM(localMaps[i].currentGlobalPosTh);
   }
   if (type == -1) {
      localMaps[previousINode].currentGlobalPosX = startX;
      localMaps[previousINode].currentGlobalPosY = startY;
      localMaps[previousINode].currentGlobalPosTh = startTh;
      ANGNORM(localMaps[previousINode].currentGlobalPosTh);
   }
   //cout << "map " << startingNode << " is now at: " << localMaps[startingNode].currentGlobalPosX << " " << 
   //   localMaps[startingNode].currentGlobalPosY << " " << localMaps[startingNode].currentGlobalPosTh << endl;

      /*cout << "Finished iteration " << maxX << " " << maxY << " " << maxTh << endl;
      if (maxX < MaxOptMoveXY && maxY < MaxOptMoveXY && maxTh < MaxOptMoveTh) {
         cout << "Finished optimising. Took " << (numIterations + 1) << " iterations" << endl;
         break;
      }

      if (retVal != 1) {
         break;
      }

   }
   cs_spfree(sparseH);
   free(b);*/
}

inline double GraphSlamCPU::getGlobalPosIndex(int node, int index) {
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

      /*if (localMaps[index].numConstraints > 0) {
         localMaps[index].totalChange[0] += localMaps[index].changeInPos[0] / localMaps[index].numConstraints;
         localMaps[index].totalChange[1] += localMaps[index].changeInPos[1] / localMaps[index].numConstraints;
         localMaps[index].totalChange[2] += localMaps[index].changeInPos[2] / localMaps[index].numConstraints;
      }*/

      while (curMap >= 0) {
         if (localMaps[curMap].numConstraints > 0) {
            posChangeX += localMaps[curMap].changeInPos[0] / localMaps[curMap].numConstraints;
            posChangeY += localMaps[curMap].changeInPos[1] / localMaps[curMap].numConstraints;
            posChangeTh += localMaps[curMap].changeInPos[2] / localMaps[curMap].numConstraints;
         }
         //posChangeX += localMaps[curMap].changeInPos[0] / common->numLoopConstraints;
         //posChangeY += localMaps[curMap].changeInPos[1] / common->numLoopConstraints;
         //posChangeTh += localMaps[curMap].changeInPos[2] / common->numLoopConstraints;
         curMap = localMaps[curMap].indexParentNode;
      }
      localMaps[index].currentGlobalPosX += posChangeX;
      localMaps[index].currentGlobalPosY += posChangeY;
      localMaps[index].currentGlobalPosTh += posChangeTh;
      ANGNORM(localMaps[index].currentGlobalPosTh);
      //cout << "Map: " << index << " change in pos: " << posChangeX << " " << posChangeY << " " << posChangeTh << endl;
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
      double cosTh = cos(localMaps[i].currentGlobalPosTh);
      double sinTh = sin(localMaps[i].currentGlobalPosTh);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
      mult3x3Matrix(a, localMaps[i].internalCovar, c);
      a[1][0] *= -1;
      a[0][1] *= -1;
      mult3x3Matrix(c, a, localMaps[i].globalCovar);

      /*convertToGlobalCoord(localMaps[currentLocalMap].robotMapCentreX, localMaps[currentLocalMap].robotMapCentreY,
            localMaps[currentLocalMap].currentGlobalPosX, localMaps[currentLocalMap].currentGlobalPosY,
            localMaps[currentLocalMap].currentGlobalPosTh, &(localMaps[currentLocalMap].globalRobotMapCentreX),
            &(localMaps[currentLocalMap].globalRobotMapCentreY));*/


      /*double parentAngle = localMaps[localMaps[i].indexParentNode]
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
      covarFiddle(localMaps[i].globalCovar);*/
   }
      
   //Local map warping
   if (LocalMapWarp) {
      int j;
      for (i = 0; i < nextLocalMap; i++) {
         j = localMaps[i].indexNextNode;
         if (j > -1) {
            double errX = localMaps[j].currentGlobalPosX - localMaps[i].currentGlobalPosX;
            double errY = localMaps[j].currentGlobalPosY - localMaps[i].currentGlobalPosY;
            double diffTh = localMaps[j].currentGlobalPosTh - localMaps[i].currentGlobalPosTh;
            ANGNORM(diffTh);

            //transform global diffs to be relative to the local map
            double cosTh = cos(- localMaps[i].currentGlobalPosTh);
            double sinTh = sin(- localMaps[i].currentGlobalPosTh);
            double diffX = cosTh * errX - sinTh * errY;
            double diffY = sinTh * errX + cosTh * errY;

            errX = diffX - localMaps[i].nextOffsetX;
            errY = diffY - localMaps[i].nextOffsetY;
            double errTh = diffTh - localMaps[i].nextOffsetTh;
            ANGNORM(errTh);

            diffX = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[0] - errX;
            diffY = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[1] - errY;
            diffTh = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[2] - errTh;
            ANGNORM(diffTh);

            //cout << "**Looking at map: " << i << ": " << diffX << " " << diffY << " " << diffTh << ": " << errX << " " << errY
            //  << " " << errTh << endl; 

            if (fabs(diffX) > LocalMapWarpThreshXY || fabs(diffY) > LocalMapWarpThreshXY || 
                  fabs(diffTh) > LocalMapWarpThreshTh) {
               cout << "Warping map: " << i << " " << diffX << " " << diffY <<  " " << diffTh << endl;
               warpLocalMap(i, errX, errY, errTh);
            }
         }
      }
      numGlobalPoints = 0;
      for (i = 0; i < nextLocalMap; i++) {
         numGlobalPoints += localMaps[i].numWarpPoints;
      }
      globalMap.resize(numGlobalPoints); 
      globalMapHeights.resize(numGlobalPoints); 
      numGlobalPoints -= localMaps[currentLocalMap].numWarpPoints;
   }
   //updateTestMap();


   //Now update the global map
   int globalOffset = 0;
   for(i = 0; i < nextLocalMap; i++) {
      double cosTh = cos(localMaps[i].currentGlobalPosTh);
      double sinTh = sin(localMaps[i].currentGlobalPosTh);

      for (int index = 0; index < localMaps[i].numWarpPoints; index++) {
         int globalIndex = convertToGlobalPosition(localMaps[i].warpPointsX[index], 
                           localMaps[i].warpPointsY[index], i, cosTh, sinTh);
         globalMap[globalOffset + index] = globalIndex;
         globalMapHeights[globalOffset + index] = localMaps[i].warpPointsZ[index];
      }
      globalOffset += localMaps[i].numWarpPoints;
   }
}

void GraphSlamCPU::warpLocalMap(int mapIndex, double errX, double errY, double errTh) {
   int i, j, k;
   //Warp each scan
   double tempCovar[3][3];
   double den[3];
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         tempCovar[i][j] = 0;
      }
      /*den[i] = errX * localMaps[mapIndex].internalCovar[i][0] +
         errY * localMaps[mapIndex].internalCovar[i][1] +
         errTh * localMaps[mapIndex].internalCovar[i][2];*/
      den[i] = localMaps[mapIndex].internalCovar[i][i];
   }
      
   double err[3];
   err[0] = errX;
   err[1] = errY;
   err[2] = errTh;
   double prevFrac[3] = {0, 0, 0};
   for (i = 0; i < localMaps[mapIndex].scans.size(); i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            tempCovar[j][k] += localMaps[mapIndex].scans[i]->covar[j][k];
         }
      }
      for (j = 0; j < 3; j++) {
         /*double num = errX * tempCovar[j][0] + errY * tempCovar[j][1] +
            errTh * tempCovar[j][2];*/
         double num = tempCovar[j][j];
         localMaps[mapIndex].scans[i]->correction[j] = fabs(num / den[j]) * err[j];
         if (fabs(num / den[j]) < prevFrac[j]) {
            cout << "^^^^^^^^^ " << j << " " << num << " " << den[j] << " " << mapIndex << endl;
         }
         prevFrac[j] = fabs(num / den[j]);
      }
   }
   int tt = localMaps[mapIndex].scans.size() - 1;
   cout << "Correction of final scan is: " << localMaps[mapIndex].scans[tt]->correction[0] << " " <<
      localMaps[mapIndex].scans[tt]->correction[1] << " " << localMaps[mapIndex].scans[tt]->correction[2] << endl;
   cout << "Num warp points before is: " << localMaps[mapIndex].numWarpPoints;
   //Now actually warp the points
   
   //Method 1: warp the already selected points
   /*for (i = 0; i < localMaps[mapIndex].numWarpPoints; i++) {
      int scanIndex = localMaps[mapIndex].lastObserved[i];
      double tempX = localMaps[mapIndex].pointsX[i] - localMaps[mapIndex].scans[scanIndex]->pose[0];
      double tempY = localMaps[mapIndex].pointsY[i] - localMaps[mapIndex].scans[scanIndex]->pose[1];
      double cosTh = cos(-localMaps[mapIndex].scans[scanIndex]->pose[2]);
      double sinTh = sin(-localMaps[mapIndex].scans[scanIndex]->pose[2]);
      double relX = cosTh * tempX - sinTh * tempY;
      double relY = sinTh * tempX + cosTh * tempY;
      cosTh = cos(localMaps[mapIndex].scans[scanIndex]->pose[2] + localMaps[mapIndex].scans[scanIndex]->correction[2]);
      sinTh = sin(localMaps[mapIndex].scans[scanIndex]->pose[2] + localMaps[mapIndex].scans[scanIndex]->correction[2]);
      localMaps[mapIndex].warpPointsX[i] = cosTh * relX - sinTh * relY
         + localMaps[mapIndex].scans[scanIndex]->pose[0] + localMaps[mapIndex].scans[scanIndex]->correction[0];
      localMaps[mapIndex].warpPointsY[i] = sinTh * relX + cosTh * relY
         + localMaps[mapIndex].scans[scanIndex]->pose[1] + localMaps[mapIndex].scans[scanIndex]->correction[1];
   }*/

   //Method 2: Warp all the points
   ////TODO: note this doesn't handle Z properly
   double z = (MinAddHeight + MaxAddHeight) / 2.0;
   int *ogMap = new int[DimLocalOG * DimLocalOG];
   memset(ogMap, 0, sizeof(int) * DimLocalOG * DimLocalOG);
   localMaps[mapIndex].numWarpPoints = 0;
   for (i = 0; i < localMaps[mapIndex].scans.size(); i++) {
      double posX = localMaps[mapIndex].scans[i]->pose[0] + localMaps[mapIndex].scans[i]->correction[0];
      double posY = localMaps[mapIndex].scans[i]->pose[1] + localMaps[mapIndex].scans[i]->correction[1];
      double posTh = localMaps[mapIndex].scans[i]->pose[2] + localMaps[mapIndex].scans[i]->correction[2];
      ANGNORM(posTh);
      double cosTh = cos(posTh);
      double sinTh = sin(posTh);
      for (j = 0; j < localMaps[mapIndex].scans[i]->points.size(); j++) {
         double tempX = localMaps[mapIndex].scans[i]->points[j].x;
         double tempY = localMaps[mapIndex].scans[i]->points[j].y;
         double x = tempX * cosTh - tempY * sinTh + posX;
         double y = tempX * sinTh + tempY * cosTh + posY;
         int index = getLocalOGIndex(x, y);
         if (index >= 0) {
            ogMap[index]++;
            if (ogMap[index] == MinObservationCount && localMaps[mapIndex].numWarpPoints < MAX_LOCAL_POINTS) {
            //if (ogMap[index] == MinObservationCount && localMaps[mapIndex].numWarpPoints < localMaps[mapIndex].numPoints) {
               localMaps[mapIndex].warpPointsX[localMaps[mapIndex].numWarpPoints] = x;
               localMaps[mapIndex].warpPointsY[localMaps[mapIndex].numWarpPoints] = y;
               localMaps[mapIndex].warpPointsZ[localMaps[mapIndex].numWarpPoints] = z;
               localMaps[mapIndex].numWarpPoints++;
            }
         }
      }
   }
   cout << " and after is: " << localMaps[mapIndex].numWarpPoints << endl;
   delete[] ogMap;
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

void GraphSlamCPU::updateTestMap() {

   int x,y;
   for (y = 0; y < testMap->height; y++) {
      crosbot::LocalMap::Cell *cellsP = &(testMap->cells[y][0]);
      for (x = 0; x < testMap->width; x++) {
         cellsP->current = false;
         cellsP->hits = 0;
         /*if (localMaps[currentLocalMap].freeArea[y * DimLocalOG + x] > 0) {
            cellsP->hits = localMaps[currentLocalMap].freeArea[y *DimLocalOG + x] * testMap->maxHits;
         } else if (localMaps[currentLocalMap].freeArea[y * DimLocalOG + x] == 0) {
            cellsP->hits = testMap->maxHits;
            cellsP->current = true;
         }*/
         cellsP++;
      }
   }
   int mapWidth = LocalMapSize / CellSize;
   double off = (mapWidth * CellSize) / 2.0 - CellSize / 2.0;
   int k;
   for (k = 0; k < localMaps[currentLocalMap].numPoints; k++) {
      double xd, yd;
      xd = localMaps[currentLocalMap].pointsX[k];
      yd = localMaps[currentLocalMap].pointsY[k];
      int i,j;
      i = testMap->width/2 + xd / testMap->resolution;
      j = testMap->height/2 - yd / testMap->resolution;
      if (i >= 0 && i < testMap->width && j >= 0 && j < testMap->height) {
         crosbot::LocalMap::Cell *cellsP = &(testMap->cells[testMap->height - j - 1][i]);

         cellsP->hits = testMap->maxHits;
         //cellsP->current = true;
      }
   }

   /*int otherMap = common->potentialMatches[0];
   double offsetX = common->potentialMatchX[0];
   double offsetY = common->potentialMatchY[0];
   double offsetTh = common->potentialMatchTh[0];
   double cosTh = cos(offsetTh);
   double sinTh = sin(offsetTh);
   cout << "***** Updating test map with maps " << currentLocalMap << " (red) and " << otherMap << endl;
   cout << "Offsets from histogram are: " << offsetX << " " << offsetY << " " << offsetTh << endl;
   //cout << "number of points displayed: " << count << "/" << localMaps[currentLocalMap].numPoints << endl;
   for (k = 0; k < localMaps[otherMap].numPoints; k++) {
      //Transform the points. Offsets are currently relative to the new map, so shouldn't use
      //convertReferenceFrame
      double xd, yd;
      double pX = localMaps[otherMap].pointsX[k];
      double pY = localMaps[otherMap].pointsY[k];
      xd = (pX * cosTh - pY * sinTh) + offsetX;
      yd = (pX * sinTh + pY * cosTh) + offsetY;
      //double xd2, yd2;
      //xd2 = (pX * cosTh - pY * sinTh);
      //yd2 = (pX * sinTh + pY * cosTh);

      int i,j;
      i = testMap->width/2 + xd / testMap->resolution;
      j = testMap->height/2 - yd / testMap->resolution;
      //int i2,j2;
      //i2 = testMap->width/2 + xd2 / testMap->resolution;
      //j2 = testMap->height/2 - yd2 / testMap->resolution;

      if (i >= 0 && i < testMap->width && j >= 0 && j < testMap->height) {
         crosbot::LocalMap::Cell *cellsP = &(testMap->cells[testMap->height - j - 1][i]);
         cellsP->hits = testMap->maxHits;

         //cellsP = &(testMap->cells[testMap->height - j2 - 1][i2]);
         //cellsP->hits = testMap->maxHits / 3.0;
         
      }
      
   }*/
}
/*void GraphSlamCPU::updateTestMap() {

   int x, y;
   crosbot::LocalMap::Cell *cellsP;
   for(y = 0; y < testMap->height; y++) {
      cellsP = &(testMap->cells[y][0]);
      for(x = 0; x< testMap->width; x++) {
         cellsP->current = false;
         cellsP->hits = 0;
         cellsP++;
      }
   }
   int i, j;
   for (i = 0; i < nextLocalMap; i++) {
      double cosTh = cos(localMaps[i].currentGlobalPosTh);
      double sinTh = sin(localMaps[i].currentGlobalPosTh);
      for (j = 0; j < localMaps[i].numPoints; j++) {
         int index = convertToGlobalPosition(localMaps[i].pointsX[j], localMaps[i].pointsY[j], 
                           i, cosTh, sinTh);
         int yi = index / DimGlobalOG;
         int xi = index % DimGlobalOG;
         cellsP = &(testMap->cells[yi][xi]);
         cellsP->hits = testMap->maxHits;
      }
   }
}*/


