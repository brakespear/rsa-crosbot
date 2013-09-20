/*
 * OgmbicpCPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

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
   int i;
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
      ///////Up to here - looking at UseVariableL and AlignmentDFix

   }
   return false;
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
