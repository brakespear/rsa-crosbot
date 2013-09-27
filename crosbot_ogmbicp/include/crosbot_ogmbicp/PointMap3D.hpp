/*
 * PointMap3D.hpp
 *
 * Local map used for the CPU version of ogmbicp
 *
 * Created on: 18/9/2013
 *     Author: adrianr
 */

#ifndef POINT_MAP_3D_HPP_
#define POINT_MAP_3D_HPP_

#include <vector>
#include <list>
#include <deque>

#include <crosbot/data.hpp>
#include <crosbot/geometry.hpp>

namespace std {
using namespace crosbot;
/*
 * Wrapper for Point3D to alos store the adjacent point in the 
 * laser scan as needed for the point matching in ogmbicp
 */
class LaserPoint {
public:
   LaserPoint();

   Point3D point;
   //The following point on the laser scan the point originated from
   Point3D pointNxt;
};

/*
 * A list of laser points stored during an iteration of ogmbicp
 */
class _LaserPoints : public HandledObject {
public:
   _LaserPoints(PointCloudPtr p, double MaxSegLen, bool IgnoreZValues,
         double FloorHeight, double MinAddHeight, double MaxAddHeight);

   //Transform the points by dx, dy, dth, dz
   void transformPoints(double dx, double dy, double dz, double dth, Pose offset);

   vector<LaserPoint> points;
};
typedef Handle<_LaserPoints> LaserPoints;

/*
 * A single 3D cell in the local map
 */
class Cell3D {
public:
   deque<LaserPoint> points;
};

/*
 * A column of 3d cells (essentially a 2D cell)
 */
class Cell3DColumn {
public:
   list<Cell3D *> cells;

   //The number of times a laser point in the column has been observed
   int obsCount;
};

/*
 * The local map used by ogmbicp
 */
class PointMap3D {
public:
   PointMap3D(double mapSize, double cellSize, double cellHeight);

   PointCloudPtr centerPointCloud(PointCloud &p, Pose curPose, Pose sensorPose, Pose *laserOffset);

   //Returns the Cell3DColumn at x,y metres from the robot
   Cell3DColumn *columnAtXY(double x, double y);

private:
   double const MapSize;
   double const CellSize;
   double const CellHeight;

   //Number of cells in a row and column of the map
   int numWidth;
   //current x and y pose of the robot rounded to the nearest cell
   double pos_x;
   double pos_y;

   deque<deque<Cell3DColumn *> *> grid;
};



}

#endif




