/*
 * OgmbicpGPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/Ogmbicp.hpp>

void Ogmbicp::initialise(ros::NodeHandle &nh) {
}

void Ogmbicp::start() {
   cout << "starting ogmbicp_gpu" << endl;
}

void Ogmbicp::stop() {
   cout << "stopping ogmbicp_gpu" << endl;

}

void Ogmbicp::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp_gpu: initialise track" << endl;
}

void Ogmbicp::updateTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp_gpu: update track" << endl;
}
