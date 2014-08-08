/*
 * local3DMap.hpp
 *
 * Created on: 05/08/2014
 *     Author: adrianr
 */

#ifndef LOCAL3DMAP_HPP_
#define LOCAL3DMAP_HPP_

struct avColour {
   int r;
   int g;
   int b;

   inline void add(uint8_t rN, uint8_t bN, uint8_t gN) {
      r += rN;
      b += bN;
      g += gN;
   }

   inline Colour getColour(int numObs) {
      return Colour(r/numObs, g/numObs, b/numObs);
   }
};
typedef struct avColour AvColour;

class VoxelCell {
   Point point;
   AvColour colour;
   int numObs;
   //Some kind of probability of being occupied
};

class VoxelColumn {
public:
   VoxelColumn(double height, double res);
   ~VoxelColumn();

   void add (Point point);
   VoxelCell *get(int z);
   
   
   VoxelCells *cells;
   int i, j;
};


/*
 * Hi-resolution voxel grid for the currently active local map
 */
class VoxelGrid {
public:
   OccupancyGrid(double width, double height, double res);

   void clearGrid();

   addScan(DepthPoints& points);

   PointCloudPtr extractPoints();

private:
   VoxelColumn ***grid;

};

/*
 * Information about each local map that is permanently stored
 */
class Local3DMap {
public:

   

private:

   Pose globalPose;

   //Tree information here
   //List of points

};

#endif
