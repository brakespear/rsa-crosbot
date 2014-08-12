/*
 * local3DMap.hpp
 *
 * Created on: 05/08/2014
 *     Author: adrianr
 */

#ifndef LOCAL3DMAP_HPP_
#define LOCAL3DMAP_HPP_

#include <crosbot/data.hpp>
#include <crosbot_3d_graphslam/depthPoints.hpp>

struct avColour {
   int r;
   int g;
   int b;

   inline void add(Colour& c) {
      r += c.r;
      b += c.b;
      g += c.g;
   }

   inline Colour getColour(int numObs) {
      return Colour(r/numObs, g/numObs, b/numObs);
   }
};
typedef struct avColour AvColour;

class VoxelCell {
public:
   Point point;
   AvColour colour;
   int numObs;
   //Some kind of probability of being occupied
};

class VoxelColumn {
public:
   VoxelColumn(double height, double res);
   ~VoxelColumn();

   inline void add (Point point);
   inline VoxelCell *get(int z);
   
   
   VoxelCell *cells;
   int i, j;
};


/*
 * Hi-resolution voxel grid for the currently active local map
 */
class VoxelGrid {
public:
   VoxelGrid(double width, double height, double res);
   ~VoxelGrid();

   void clearGrid();

   void addScan(DepthPointsPtr points);

   PointCloudPtr extractPoints();

private:
   double width;
   double height;
   double res;
   int dimWidth;
   int dimHeight;

   int offXY;
   int offZ;

   VoxelColumn ***grid;

};

/*
 * Information about each local map that is permanently stored
 */
class Local3DMap {
public:
   Local3DMap(Pose pose);
   
   Pose getPose();

private:

   Pose globalPose;

   //Tree information here
   //List of points

};

#endif
