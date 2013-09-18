/*
 * OgmbicpCPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/OgmbicpCPU.hpp>

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
   cout << "ogmbicp: update track" << endl;
}
