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
   //Number of cells around the laser point that will be searched
   // to find the matching point
   int CellSearchDistance;

   //Inherited methods from ogmbicp.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);

   OgmbicpCPU();
   ~OgmbicpCPU();
protected:
   void getLocalMap(LocalMapPtr curMap);
private:
   PointMap3D *localMap;

   //Offset of laser from center of point cloud
   Pose laserOffset;

   int failCount;
   int scanSkip;
   int addSkip;

   //Previous move
   double px, py, pz, pth;

   //Get the offset from an iteration of ogmbicp
   bool getOffset(LaserPoints scan, double &dx, double &dy, double &dz, double &dth);
   //Find the best matching point (mPoint) to scanPoint
   //Returns the matching score. INFINITY if no matching point was found
   double findMatchingPoint(Point scanPoint, Point &mPoint, double lVal);
   //Calculates the match score between scan point and a laser point. 
   //Laser points are stored relative to the center of their map cell,
   //so centerX, centerY offsets them by the center of the cell.
   //matchPoint is the matching point as scanPoint is matched to the line
   //between mapPoint.point and mapPoint.pointNxt
   double getHValue(Point scanPoint, LaserPoint mapPoint, double centerX, 
         double centerY, Point &matchPoint, double lVale);
   //Calculate the match score for scanPoint to mPoint
   double calculateHValue(Point scanPoint, Point mPoint, double lVal);
   //Remove the laser offset from a point
   Point3D removeLaserOffset(Point3D p1);
   //Put the laser offset back onto the value of a point
   Point3D addLaserOffset(Point3D p1);

};

#endif
