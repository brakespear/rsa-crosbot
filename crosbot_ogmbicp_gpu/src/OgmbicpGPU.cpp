/*
 * OgmbicpGPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp_gpu/OgmbicpGPU.hpp>

OgmbicpGPU::OgmbicpGPU() {
}

OgmbicpGPU::~OgmbicpGPU() {
}

void OgmbicpGPU::initialise(ros::NodeHandle &nh) {
}

void OgmbicpGPU::start() {
   cout << "starting ogmbicp_gpu" << endl;
}

void OgmbicpGPU::stop() {
   cout << "stopping ogmbicp_gpu" << endl;

}

void OgmbicpGPU::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp_gpu: initialise track" << endl;
}

void OgmbicpGPU::updateTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp_gpu: update track" << endl;
}

void OgmbicpGPU::getLocalMap(LocalMapPtr curMap) {
}
