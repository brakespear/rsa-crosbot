/*
 * OgmbicpCPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

OgmbicpCPU::OgmbicpCPU() {
}

void OgmbicpCPU::initialise(ros::NodeHandle &nh) {

   Ogmbicp::initialise(nh);

   //ros::NodeHandle paramNH("~");

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
   PointCloudPtr worldPoints = localMap->centerPointCloud(*cloud, curPose, sensorPose, &laserOffset);
   laserPose = sensorPose;

   LaserPoints scan = new _LaserPoints(worldPoints, MaxSegLen);

   double dx, dy, dz, dth;
   dx = 0;
   dy = 0;
   dz = 0;
   dth = 0;
   //TODO: put the initial transform stuff here
   
   scan->transformPoints(dx, dy, dz, dth, laserOffset);


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
