/*
 * Ogmbicp.cpp
 *
 * Created on: 16/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/Ogmbicp.hpp>

void Ogmbicp::initialise(ros::NodeHandle &nh) {
}

void Ogmbicp::start() {
   cout << "starting ogmbicp" << endl;
}

void Ogmbicp::stop() {
   cout << "stopping ogmbicp" << endl;

}

void Ogmbicp::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp: initialise track" << endl;
}

void Ogmbicp::updateTrack(Pose sensorPose, PointCloudPtr cloud) {
   cout << "ogmbicp: update track" << endl;
}
