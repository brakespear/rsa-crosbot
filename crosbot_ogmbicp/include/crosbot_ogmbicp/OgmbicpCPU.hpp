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
   /*
    * Config attributes for CPU version of ogmbicp
    */
   //Every LaserSkip(th) laser point will be used in the alignment
   //eg. LaserSkip = 1 will use every laser point from a scan
   int LaserSkip;

   //Inherited methods from ogmbicp.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);

   OgmbicpCPU();
   ~OgmbicpCPU();
private:
   PointMap3D *localMap;

   //Offset of laser from center of point cloud
   Pose laserOffset;

   //Get the offset from an iteration of ogmbicp
   bool getOffset(LaserPoints scan, double &dx, double &dy, double &dz, double &dth);
   //Find the best matching point (mPoint) to scanPoint
   //Returns the matching score. INFINITY if no matching point was found
   double findMatchingPoint(Point scanPoint, Point &mPoint, double lVal);
   //Remove the laser offset from a point
   Point3D removeLaserOffset(Point3D p1);
   //Put the laser offset back onto the value of a point
   Point3D addLaserOffset(Point3D p1);

};

#endif
