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
 * Wrapper for Point3D to also store the adjacent point in the 
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
   Cell3D(double z);

   //The list of points stored by the cell
   deque<LaserPoint> points;

   //The centre z value of the cell
   double zVal;
   //If the cell has had points added to it this iteration
   bool mark;

   //Marks the cell
   void markCell();
   //Clears the points stored in the cell
   void reset();

   //Makes sure all cells are unmarked
   static void unmarkCells();
private:
   //Stores a list of marked cells. A marked cell is one that has
   //had a point added to it this iteration
   static deque<Cell3D *> markedCells;
};

/*
 * A column of 3d cells (essentially a 2D cell)
 */
class Cell3DColumn {
public:
   Cell3DColumn(double cellHeight);

   list<Cell3D *> cells;

   //The number of times a laser point in the column has been observed
   int obsCount;
   //The life count of the cell - the number of scans that can be processed without any
   //laser points being observed in the column until it is deleted
   int lifeCount;
   //True if the column is currently being observed
   bool current;

   //Returns the nearest 3D cell in height to the given z value
   //if nearest is true, return the nearest cell. otherwise, only return the
   //cell if z actually lies within the cell
   Cell3D *getNearestCell(double z, bool nearest=true);
   //Adds a laser point to the column
   void addLaserPoint(LaserPoint point, int maxObservations, double lifeRatio, bool resetCell);
   //Deletes all the information about the column so that it can be reused
   void reset();
private:
   double const CellHeight;

   //Last cell that was observed to speed up searching for nearest cells
   list<Cell3D *>::iterator lastIndex;

   //Adds a new cell to the column in the correct position
   Cell3D *addNewCell(double zCent);

};

/*
 * Stores the coordinates of a Cell3Dcolumn that is currently active
 */
class ActiveColumn {
public:
   int i;
   int j;
   ActiveColumn(int ix, int jy) {
      i = ix;
      j = jy;
   }
};

/*
 * The local map used by ogmbicp
 */
class PointMap3D {
public:
   //Stores the columns that are currently active
   deque<ActiveColumn> activeColumns;


   PointMap3D(double mapSize, double cellSize, double cellHeight);

   PointCloudPtr centerPointCloud(PointCloud &p, Pose curPose, Pose sensorPose, Pose *laserOffset);

   //Returns the Cell3DColumn at x,y metres from the robot
   Cell3DColumn *columnAtXY(double x, double y);
   //Returns the Cell3DColumn at index i,j
   Cell3DColumn *columnAtIJ(int i, int j);
   //Gets the index i,j corresponding to the coordinates x,y
   void getIJ(double x, double y, int *i, int *j);
   //Returns the x,y coords of the center of the column at index i,j
   void getXY(int i, int j, double *x, double *y);

   //Adds a scan to the map
   void addScan(LaserPoints scan, int maxObservations, double lifeRatio, bool resetCells);
   //Update the life count of active cells
   void updateActiveCells();
   //Shift the map by the robot's movement
   void shift(double gx, double gy, double gz);

private:
   double const MapSize;
   double const CellSize;
   double const CellHeight;

   //Number of cells in a row and column of the map
   int numWidth;
   double mapOffset;
   //current x and y pose of the robot rounded to the nearest cell
   double pos_x;
   double pos_y;
   //Offset of robot in current cell
   double offsetX;
   double offsetY;

   //Stores the 2D grid of columns
   deque<deque<Cell3DColumn *> *> grid;

   //Shift the map columns
   void shiftX(int xMove);
   void shiftY(int xMove);
   
};



}

#endif




