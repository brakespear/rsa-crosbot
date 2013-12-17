/*
 * OgmbicpCPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 *
 * CPU version of ogmbicp
 */

#include <newmat/newmat.h>
#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

using namespace NEWMAT;

OgmbicpCPU::OgmbicpCPU() {
   failCount = 0;
   scanSkip = 0;
   addSkip = 0;
   finishedSetup = false;

   //imuYaw = 0;
   //isOrientationValid = false;
   discardScan = false;

   numIterations = 0;
   avNumIts = 0;
}

void OgmbicpCPU::initialise(ros::NodeHandle &nh) {

   Ogmbicp::initialise(nh);

   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LaserSkip", LaserSkip, 4);
   paramNH.param<int>("CellSearchDistance", CellSearchDistance, 3);

   localMap = new PointMap3D(MapSize, CellSize, CellHeight);

}

void OgmbicpCPU::start() {
}

void OgmbicpCPU::stop() {
}

void OgmbicpCPU::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   curPose.position.z = InitHeight;
   px = py = pz = pth = 0;
}

void OgmbicpCPU::updateTrack(Pose sensorPose, PointCloudPtr cloud) {

   if (discardScan) {
      cout << "Ignoring scan" << endl;
      return;
   }

   ros::WallTime t1 = ros::WallTime::now();

   //Filter out points that are too close to the robot
   vector<Point> newCloud;
   int i;
   for (i = 0; i < cloud->cloud.size(); i++) {
      if (cloud->cloud[i].x * cloud->cloud[i].x + 
            cloud->cloud[i].y * cloud->cloud[i].y > LaserMinDist * LaserMinDist) {
         newCloud.push_back(cloud->cloud[i]);
      }
   }
   cloud->cloud = newCloud;

   PointCloudPtr worldPoints = localMap->centerPointCloud(*cloud, curPose, sensorPose, &laserOffset);
   laserPose = sensorPose;

   LaserPoints scan = new _LaserPoints(worldPoints, MaxSegLen, IgnoreZValues,
         FloorHeight, MinAddHeight, MaxAddHeight);

   if (InitialScans > 0) {
      localMap->addScan(scan, MaxObservations, LifeRatio, true);
      InitialScans--;
      return;
   }

   //Updates each iteration
   double dx, dy, dz, dth;
   dx = px;
   dy = py;
   dz = pz;
   dth = pth;
   /*if (isOrientationValid) {
      double roll, pitch, yaw;
      curPose.getYPR(yaw, pitch, roll);
      cout << imuYaw << " " << yaw << " " << dth << endl;
      dth = imuYaw - yaw;
      ANGNORM(dth);
   }*/
   //Total move for the laser scan
   double gx, gy, gz, gth;
   gx = px;
   gy = py;
   gz = pz;
   gth = pth;


   //TODO: put the initial transform stuff here
   
   scan->transformPoints(dx, dy, dz, dth, laserOffset);

   int iterCount = 0;
   bool alignedScan = false;
   while (iterCount < MaxIterations && getOffset(scan, dx, dy, dz, dth)) {
      scan->transformPoints(dx, dy, dz, dth, laserOffset);

      dz = 0; //TODO: fix this

      gx += dx;
      gy += dy;
      gz += dz;
      gth += dth;
      ANGNORM(gth);

      avNumIts++;

      if (fabs(dx) < MaxErrorXY && fabs(dy) < MaxErrorXY && fabs(dz) < MaxErrorZ
            && fabs(dth) < MaxErrorTh) {
         alignedScan = true;
         break;
      }
      iterCount++;
   }
   //cout << "iter count is " << iterCount << endl;

   if (isnan(gx) || isnan(gy) || isnan(gz) || isnan(gth) || fabs(gx) > MaxMoveXYZ ||
         fabs(gy) > MaxMoveXYZ || fabs(gz) > MaxMoveXYZ || fabs(gth) > MaxMoveTh) {
      failCount++;
      cout << "Failed scan" << endl;
      if (failCount < MaxFail) {
         return;
      } else {
         cout << "Setting move to 0" << endl;
         gx = gy = gz = gth = 0;
         addSkip = AddSkipCount + 1;
      }
   }
   failCount = 0;

   transformToRobot(gx, gy, gz, gth);
   ANGNORM(gth);

   if (alignedScan && scanSkip >= MaxScanSkip) {
      scanSkip = 0;
      localMap->updateActiveCells();
      if (addSkip > AddSkipCount) {
         addSkip = 0;
         localMap->addScan(scan, MaxObservations, LifeRatio, true);
      } else {
         localMap->addScan(scan, MaxObservations, LifeRatio, false);
      }
      addSkip++;
   }
   scanSkip++;
   localMap->shift(gx,gy,gz);

   curPose.position.x += gx;
   curPose.position.y += gy;
   curPose.position.z = InitHeight + gz;

   double roll, pitch, yaw;
   curPose.getYPR(yaw, pitch, roll);
   yaw += gth;
   ANGNORM(yaw);
   curPose.setYPR(yaw, pitch, roll);

   if (UsePriorMove) {
      px = gx;
      py = gy;
      pz = 0;
      pth = gth;
   } else {
      px = py = pz = pth = 0;
   }

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms "
         << curPose.position.x << " " << curPose.position.y << " " <<
         yaw << " " << (float) avNumIts / numIterations << endl;
   }

   finishedSetup = true;
   Pose3D newPose;
   newPose = curPose;
   PointCloudPtr hist = new PointCloud("/icp", *cloud, newPose);
   hist->timestamp = ros::Time::now();
   recentScans.push_back(hist);
   while((recentScans.back()->timestamp - 
            recentScans.front()->timestamp).toSec() > ScanListTime) {
      recentScans.pop_front();
   }
}

bool OgmbicpCPU::getOffset(LaserPoints scan, double &dx, double &dy, double &dz, double &dth) {

   Point scanPoint;
   Point mPoint;
   int i;
   int count = 0;
   Matrix A(4,4);
   Matrix ASimple(3,3);
   ColumnVector B(4);
   ColumnVector BSimple(3);
   ColumnVector Q(4);
   ColumnVector QSimple(3);

   double a11, a12, a13, a14, a22, a23, a24, a33, a34, a44;
   double b1, b2, b3, b4;
   a11 = a12 = a13 = a14 = a22 = a23 = a24 = a33 = a34 = a44 = 0.0;
   b1 = b2 = b3 = b4 = 0;


   for (i = 0; i < scan->points.size(); i += LaserSkip) {
      //TODO: deal with z values properly
      if (scan->points[i].point.z < MinAddHeight || scan->points[i].point.z > MaxAddHeight) {
         //cout << "points have the wrong height" << endl;
         continue;
      }
      if (scan->points[i].pointNxt.hasNAN()) {
         //cout << "points have nan" << endl;
         continue;
      }
      scanPoint = removeLaserOffset(scan->points[i].point);

      double d = sqrt(SQ(scanPoint.x) + SQ(scanPoint.y));
      if (LaserMaxAlign > 0 && d > LaserMaxAlign) {
         //cout << "laser max align failing" << endl;
         continue;
      }
      double lValue2 = LValue;
      if (UseVariableL) {
         lValue2 *= (AlignmentDFix - d) / AlignmentDFix;
         lValue2 *= lValue2;
      } else {
         lValue2 *= LValue;
      }
      double h = findMatchingPoint(scanPoint, mPoint, lValue2);
      if (h == INFINITY) {
         //cout << "no matching point found" << endl;
         continue;
      }
      //cout << "matching point found" << endl;
      Point3D mPointMap = addLaserOffset(mPoint);

      Cell3DColumn *colCell = localMap->columnAtXY(mPointMap.x, mPointMap.y);
      if (colCell == NULL) {
         continue;
      }
      if (colCell->obsCount < MinCellCount) {
         //cout << "min cell count failed" << endl;
         continue;
      }
      double fac = 1;
      if (UseFactor) {
         fac = (d / LaserMaxDistance) * ((double) colCell->obsCount) / MaxObservations;
         if (fac < MinFactor) {
            fac = MinFactor;
         }
      }
      if (UseSimpleH) {
         double x12 = SQ(mPoint.x);
         double y12 = SQ(mPoint.y);
         double k = 1.0 / (x12 + y12 + LValue * LValue);

         a11 += fac * (1.0 - y12 * k);
         a12 += fac * mPoint.x * mPoint.y * k;
         a13 += fac * (-scanPoint.y + mPoint.y * k * 
               (mPoint.y * scanPoint.y + mPoint.x * scanPoint.x));
         a22 += fac * (1.0 - x12 * k);
         a23 += fac * (scanPoint.x - mPoint.x * k *
               (mPoint.y * scanPoint.y + mPoint.x * scanPoint.x));
         a33 += fac * (x12 + y12 - k *
               SQ(mPoint.y * scanPoint.y + mPoint.x * scanPoint.x));

         b1 += fac * (scanPoint.x - mPoint.x - mPoint.y * k *
               (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y));
         b2 += fac * (scanPoint.y - mPoint.y + mPoint.x * k *
               (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y));
         b3 += fac * (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y) * 
               (-1.0 + k * (mPoint.y * scanPoint.y + mPoint.x * scanPoint.x));

      } else {
         double x12 = SQ(mPoint.x);
         double y12 = SQ(mPoint.y);
         double z12 = SQ(mPoint.z);
         double k = 1.0 / (x12 + y12 + z12 + LValue * LValue);

         a11 += fac * (1.0 - (z12 + y12) * k);
         a12 += fac * (mPoint.x * mPoint.y * k);
         a13 += fac * (mPoint.x * mPoint.z * k);
         a14 += fac * (((scanPoint.y * (z12 + y12) + mPoint.x * mPoint.y * scanPoint.x)
                  * k) - scanPoint.y);
         a22 += fac * (1.0 - (z12 + x12) * k);
         a23 += fac * (mPoint.y * mPoint.z * k);
         a24 += fac * (scanPoint.x - (scanPoint.x * z12 + 
                  mPoint.x * mPoint.y * scanPoint.y + x12 * scanPoint.x) * k);
         a33 += fac * (1.0 - (x12 + y12) * k);
         a34 += fac * ((mPoint.y * mPoint.z * scanPoint.x - 
                  mPoint.x * mPoint.z * scanPoint.y) * k);
         a44 += fac * (x12 + y12 + k * (SQ(scanPoint.y) * (y12 + z12) + 
                  SQ(scanPoint.x) * (x12 + z12) + 
                  mPoint.x * mPoint.y * scanPoint.x * scanPoint.y));

         b1 += fac * (scanPoint.x - mPoint.x + k * (mPoint.x * 
                  (mPoint.z * scanPoint.z + mPoint.y * scanPoint.y) - 
                  scanPoint.x * (y12 + z12)));
         b2 += fac * (scanPoint.y - mPoint.y + k * (mPoint.y * 
                  (mPoint.z * scanPoint.z + mPoint.x * scanPoint.x) - 
                  scanPoint.y * (x12 + z12)));
         b3 += fac * (scanPoint.z - mPoint.z + k * (mPoint.z * 
                  (mPoint.x * scanPoint.x + mPoint.y * scanPoint.y) - 
                  scanPoint.z * (y12 + x12)));
         b4 += fac * (mPoint.x * scanPoint.y - mPoint.y * scanPoint.x + k *
                  (mPoint.z * scanPoint.z * (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y) + 
                   mPoint.y * scanPoint.y * (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y) +
                   mPoint.x * scanPoint.x * (mPoint.y * scanPoint.x - mPoint.x * scanPoint.y)));
      }
      count++;
   }
   if (count < MinGoodCount) {
      dx = dy = dz = dth = 0.0;
      return false;
   }

   if (UseSimpleH) {
      ASimple(1,1) = a11;
      ASimple(2,1) = a12;
      ASimple(3,1) = a13;
      ASimple(1,2) = a12;
      ASimple(2,2) = a22;
      ASimple(3,2) = a23;
      ASimple(1,3) = a13;
      ASimple(2,3) = a23;
      ASimple(3,3) = a33;

      BSimple(1) = b1;
      BSimple(2) = b2;
      BSimple(3) = b3;

      ASimple = ASimple.i();
      QSimple = -ASimple * BSimple;
      dx = QSimple(1);
      dy = QSimple(2);
      dz = 0;
      dth = QSimple(3);

   } else {
      A(1,1) = a11;
      A(2,1) = a12;
	   A(3,1) = a13;
   	A(4,1) = a14;
   	A(1,2) = a12;
	   A(2,2) = a22;
   	A(3,2) = a23;
	   A(4,2) = a24;
   	A(1,3) = a13;
	   A(2,3) = a23;
   	A(3,3) = a33;
	   A(4,3) = a34;
   	A(1,4) = a14;
	   A(2,4) = a24;
   	A(3,4) = a34;
	   A(4,4) = a44;

   	B(1) = b1;
	   B(2) = b2;
   	B(3) = b3;
	   B(4) = b4;

   	A = A.i();
   	Q = -A * B;

	   dx = Q(1);
   	dy = Q(2);
	   dz = Q(3);
   	dth = Q(4);
   }
   return true;
}

double OgmbicpCPU::findMatchingPoint(Point scanPoint, Point &mPoint, double lVal) {
   int besti[FullSearchSize];
   int bestj[FullSearchSize];
   double besth[FullSearchSize];
   Cell3D *bestCell[FullSearchSize];

   double centerX, centerY;

   int i,j,k, m, n;
   for(k = 0; k < FullSearchSize; k++) {
      besti[k] = 0;
      bestj[k] = 0;
      besth[k] = INFINITY;
      bestCell[k] = NULL;
   }
   Point3D scanPointOnGrid = addLaserOffset(scanPoint);
   //cout << "scan point: " << scanPoint.z << " on grid: " << scanPointOnGrid.z << endl;
   localMap->getIJ(scanPointOnGrid.x, scanPointOnGrid.y, &i, &j);
   Cell3DColumn *col = localMap->columnAtIJ(i, j);
   if (col == NULL) {
      return INFINITY;
   }

   for (m = -CellSearchDistance; m < CellSearchDistance; m++) {
      for (n = -CellSearchDistance; n < CellSearchDistance; n++) {
         col = localMap->columnAtIJ(i+m, j+n);
         if (col == NULL) {
            //cout << "null ";
            continue;
         }
         if (col->obsCount < MinCellCount) {
            //cout << "cellCount" << col->obsCount << " ";
            continue;
         }
         Cell3D *cell = col->getNearestCell(scanPointOnGrid.z);
         if (cell == NULL) {
            //cout << "cellNull ";
            continue;
         }
         if (cell->points.size() == 0) {
            //cout << "points ";
            continue;
         }
         localMap->getXY(i+m, j+n, &centerX, &centerY);
         Point centerPoint(centerX, centerY, cell->zVal);
         //Calculate the match score of the scan point to the center of the cell
         Point centerPointRem = removeLaserOffset(centerPoint);
         double h = calculateHValue(scanPoint, centerPointRem, lVal);
         //cout << "found a possibility " << h << endl;
         if (NearestAlgorithm == 1) {
            h = h * ((double) MaxObservations / (double)col->obsCount);
         } else if (NearestAlgorithm == 2) {
            h = h * (1.1 - ((double)col->obsCount / (double) MaxObservations));
         }
         k = FullSearchSize - 1;
         if (h < besth[k]) {
            besth[k] = h;
            besti[k] = i+m;
            bestj[k] = j+n;
            bestCell[k] = cell;
         }
         k -= 1;
         while (k >= 0 && besth[k] > besth[k+1]) {
            double htmp = besth[k];
            int itmp = besti[k];
            int jtmp = bestj[k];
            Cell3D *celltmp = bestCell[k];
            besth[k] = besth[k+1];
            besti[k] = besti[k+1];
            bestj[k] = bestj[k+1];
            bestCell[k] = bestCell[k+1];
            besth[k+1] = htmp;
            besti[k+1] = itmp;
            bestj[k+1] = jtmp;
            bestCell[k+1] = celltmp;
            k -= 1;
         }
      }
   }
   //cout << endl;
   double hMin = INFINITY;
   for(i = 0; i < FullSearchSize; i++) {
      if (besth[i] == INFINITY) {
         continue;
      }
      localMap->getXY(besti[i], bestj[i], &centerX, &centerY);
      Cell3D *cell = bestCell[i];
      Point tmpPoint;
      for(k = 0; k < cell->points.size(); k++) {
         double h = getHValue(scanPoint, cell->points.at(k), centerX, centerY, tmpPoint, lVal);
         if (h < hMin) {
            hMin = h;
            mPoint = tmpPoint;
         }
      }
   }

   return hMin;
}

double OgmbicpCPU::getHValue(Point scanPoint, LaserPoint mapPoint, double centerX,
      double centerY, Point &matchPoint, double lVal) {
   Point p1 = removeLaserOffset(mapPoint.point);
   p1.x += centerX;
   p1.y += centerY;
   Point p2 = removeLaserOffset(mapPoint.pointNxt);
   p2.x += centerX;
   p2.y += centerY;

   if (isnan(p2.x)) {
      matchPoint = p1;
      return calculateHValue(scanPoint, p1, lVal);
   } else if (isnan(p1.x)) {
      matchPoint = p2;
      return calculateHValue(scanPoint, p2, lVal);
   }

   double dx, dy, ux, uy, a, b, c, d, lambda;
   ux = p1.x - p2.x;
   uy = p1.y - p2.y;
   dx = p1.x - scanPoint.x;
   dy = p1.y - scanPoint.y;
   a = 1/(SQ(scanPoint.x) + SQ(scanPoint.y)+lVal);
   b = 1 - a * SQ(scanPoint.y);
   c = 1 - a * SQ(scanPoint.x);
   d = a*scanPoint.x*scanPoint.y;
   lambda = (d*(ux*dy + uy*dx) + b*ux*dx + c * uy * dy) / (b*ux*ux + c*uy*uy + 2.0*c*ux*uy);
   if (lambda <= 0.0) {
      matchPoint = p1;
   } else if (lambda >= 1.0) {
      matchPoint = p2;
   } else {
      matchPoint = p1;
      matchPoint.x = p1.x + lambda * ux;
      matchPoint.y = p1.y + lambda * uy;
   }
   return calculateHValue(scanPoint, matchPoint, lVal);
}


double OgmbicpCPU::calculateHValue(Point scanPoint, Point mPoint, double lVal) {
   double dx, dy, dz;

   dx = mPoint.x - scanPoint.x;
   dy = mPoint.y - scanPoint.y;
   dz = mPoint.z - scanPoint.z;

   if (SQ(dx) + SQ(dy) + SQ(dz) > MaxAlignDistance) {
      //cout << mPoint.z << " " << scanPoint.z << endl;
      return INFINITY;
   }

   if (UseSimpleH) {
      return dx*dx + dy*dy - (SQ(dx*scanPoint.y - dy*scanPoint.x)) / 
            (SQ(scanPoint.x) + SQ(scanPoint.y) + lVal);
   } else {
      return dx*dx + dy*dy + dz*dz - (SQ(dy*scanPoint.z - dz*scanPoint.y) +
            SQ(dz*scanPoint.x - dx*scanPoint.z) + SQ(dx*scanPoint.y - dy*scanPoint.x)) /
            (SQ(scanPoint.x) + SQ(scanPoint.y) + SQ(scanPoint.z) + lVal);
   }
}

Point3D OgmbicpCPU::removeLaserOffset(Point3D p1) {
   Point3D p = p1;
   p.x -= laserOffset.position.x;
   p.y -= laserOffset.position.y;
   p.z -= laserOffset.position.z;
   return p;
}

Point3D OgmbicpCPU::addLaserOffset(Point3D p1) {
   Point3D p = p1;
   p.x += laserOffset.position.x;
   p.y += laserOffset.position.y;
   p.z += laserOffset.position.z;
   return p;
}

OgmbicpCPU::~OgmbicpCPU() {
   delete localMap;
}

void OgmbicpCPU::getLocalMap(LocalMapPtr curMap) {

   //cout << "in get local map " << curMap->height << " " << curMap->width << " " << localMap->activeColumns.size() << endl;

   double lifeScale = curMap->maxHits / (LifeRatio * MaxObservations);
   int x,y;
   for (y = 0; y < curMap->height; y++) {
      LocalMap::Cell *cellsP = &(curMap->cells[y][0]);
      for (x = 0; x < curMap->width; x++) {
         cellsP->current = false;
         cellsP->hits = 0;
         cellsP++;
      }
   }
   curMap->origin.position.x = curPose.position.x - 
      (curMap->width * curMap->resolution) / 2;
   curMap->origin.position.y = curPose.position.y - 
      (curMap->height * curMap->resolution) / 2;
   int mapWidth = MapSize / CellSize;
   double off = (mapWidth * CellSize) / 2.0 - CellSize / 2.0;
   int k;
   for (k = 0; k < localMap->activeColumns.size(); k++) {
      double xd, yd;
      xd = localMap->activeColumns[k].i * CellSize - off;
      yd = localMap->activeColumns[k].j * CellSize - off;
      int i,j;
      i = curMap->width/2 - xd / curMap->resolution;
      j = curMap->height/2 - yd / curMap->resolution;
      if (i >= 0 && i < curMap->width && j >= 0 && j < curMap->height) {
         LocalMap::Cell *cellsP = &(curMap->cells[curMap->height - j - 1][i]);

         Cell3DColumn *col = localMap->columnAtIJ(localMap->activeColumns[k].i,
               localMap->activeColumns[k].j);
         if (col->current) {
            cellsP->hits = curMap->maxHits;
            cellsP->current = true;
         } else {
            //cellsP->hits = lifeScale * col->lifeCount;
            cellsP->hits = curMap->maxHits;
         }
         //TODO: put in the flobsticle stuff here as well
      }
   }
}
