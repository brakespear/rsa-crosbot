/*
 * OgmbicpCPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <newmat/newmat.h>
#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

using namespace NEWMAT;

OgmbicpCPU::OgmbicpCPU() {
}

void OgmbicpCPU::initialise(ros::NodeHandle &nh) {

   Ogmbicp::initialise(nh);

   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LaserSkip", LaserSkip, 4);

   localMap = new PointMap3D(MapSize, CellSize, CellHeight);

}

void OgmbicpCPU::start() {
   cout << "starting ogmbicp" << endl;
}

void OgmbicpCPU::stop() {
   cout << "stopping ogmbicp" << endl;

}

void OgmbicpCPU::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp: initialise track" << endl;
}

void OgmbicpCPU::updateTrack(Pose sensorPose, PointCloudPtr cloud) {

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

   //Updates each iteration
   double dx, dy, dz, dth;
   dx = 0;
   dy = 0;
   dz = 0;
   dth = 0;
   //Total move for the laser scan
   double gx, gy, gz, gth;
   gx = 0;
   gy = 0;
   gz = 0;
   gth = 0;
   //TODO: put the initial transform stuff here
   
   scan->transformPoints(dx, dy, dz, dth, laserOffset);

   int iterCount = 0;
   while (iterCount < MaxIterations && getOffset(scan, dx, dy, dz, dth)) {
      scan->transformPoints(dx, dy, dz, dth, laserOffset);

      dz = 0; //TODO: fix this

      gx += dx;
      gy += dy;
      gz += dz;
      gth += dth;
      ANGNORM(gth);

      if (fabs(dx) < MaxErrorXY && fabs(dy) < MaxErrorXY && fabs(dz) < MaxErrorZ
            && fabs(dth) < MaxErrorTh) {
         break;
      }
      iterCount++;
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
   a11 = a12 = a13 = a14 = a22 = a23 = a24 = a33 = a34 = a44 = 0;.0;
   b1 = b2 = b3 = b4 = 0;


   for (i = 0; i < scan->points.size(); i += LaserSkip) {
      if (scan->points[i].point.z < MinAddHeight || scan->points[i].point.z > MaxAddHeight) {
         continue;
      }
      if (scan->points[i].pointNxt.hasNAN()) {
         continue;
      }
      scanPoint = removeLaserOffset(scan->points[i].point);
      double d = sqrt(SQ(scanPoint.x) + SQ(scanPoint.y));
      if (LaserMaxAlign > 0 && d > LaserMaxAlign) {
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
         continue;
      }
      Point3D mPointMap = addLaserOffset(mPoint);

      Cell3DColumn *colCell = localMap->columnAtXY(mPointMap.x, mPointMap.y);
      if (colCell->obsCount < MinCellCount) {
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
   return 0;
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
