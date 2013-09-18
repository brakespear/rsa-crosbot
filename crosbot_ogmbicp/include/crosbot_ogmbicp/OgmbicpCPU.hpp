/*
 * OgmbicpCPU.hpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#ifndef OGMBICP_CPU_HPP_
#define OGMBICP_CPU_HPP_

#include <crosbot_ogmbicp/Ogmbicp.hpp>
#include <crosbot_ogmbicp/PointMap3D.hpp>

class OgmbicpCPU : public Ogmbicp {
public:
   //Inherited methods from ogmbicp.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);
private:
   PointMap3D *localMap;

};

#endif
