/*
 * OgmbicpGPU.hpp
 *
 * Created on: 13/12/2013
 *     Author: adrianr
 */

#ifndef OGMBICP_GPU_HPP_
#define OGMBICP_GPU_HPP_

#include <crosbot_ogmbicp/Ogmbicp.hpp>

class OgmbicpGPU : public Ogmbicp {
public:
   /*
    * Config attributes for GPU version of ogmbicp
    */

   //Inherited methods from ogmbicp.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);

   OgmbicpGPU();
   ~OgmbicpGPU();
protected:
   void getLocalMap(LocalMapPtr curMap);
private:

   //Offset of laser from center of point cloud
   Pose laserOffset;
};

#endif
