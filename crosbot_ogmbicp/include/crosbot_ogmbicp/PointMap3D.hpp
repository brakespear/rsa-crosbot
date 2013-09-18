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
};

/*
 * The local map used by ogmbicp
 */
class PointMap3D {
public:
   PointMap3D(double mapSize, double cellSize, double cellHeight);   

private:
   double const MapSize;
   double const CellSize;
   double const CellHeight;

   int numWidth;

   deque<deque<Cell3DColumn *> *> grid;
};



}

#endif




