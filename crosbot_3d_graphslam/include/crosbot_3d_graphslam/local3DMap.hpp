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
   uint32_t r;
   uint32_t g;
   uint32_t b;

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
   AvColour avColour;
   int numObs;
   //Some kind of probability of being occupied
   
   VoxelCell();
   
   inline void addPoint(Point& point, Colour& colour);

   inline void getCellInfo(Point& point, Colour& colour);
};

class VoxelColumn {
public:
   VoxelColumn(double height, double res);
   ~VoxelColumn();

   inline void add (Point& point, Colour& colour);
   inline VoxelCell *get(double z);
   void extractCells(PointCloudPtr cloud, int obsThresh);
   
   
   VoxelCell *cells;
   int i, j;
private:
   double height;
   double res;
   int dimHeight;
   double offZ;
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

   inline int getIndex(double x, double y);

   PointCloudPtr extractPoints(int obsThresh);

private:
   double width;
   double height;
   double res;
   int dimWidth;
   //int dimHeight;

   double offXY;
   //double offZ;

   VoxelColumn **grid;

};

/*
 * Information about each local map that is permanently stored
 */
class Local3DMap {
public:
   Local3DMap(Pose pose);
   
   Pose getPose();

   void updatePose(Pose pose);

   bool poseChanged;

private:

   Pose globalPose;
   
   PointCloudPtr cloud;
   //Tree information here
   //List of points

};

#endif
