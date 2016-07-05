/*
 * local3DMap.cpp
 *
 * Created on: 05/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/local3DMap.hpp>

using namespace std;

VoxelCell::VoxelCell() {
   numObs = 0;
   avColour.b = 0;
   avColour.g = 0;
   avColour.r = 0;
}

inline void VoxelCell::addPoint(Point& point, Colour& colour) {
   numObs++;
   this->point += point;
   avColour.add(colour);
}

inline void VoxelCell::getCellInfo(Point& point, Colour& colour) {
   point = this->point / (double) numObs;
   colour = avColour.getColour(numObs);
}

VoxelColumn::VoxelColumn(double height, double res): height(height), res(res) {
   dimHeight = height / res;
   offZ = (dimHeight * res) / 2.0;

   //cells = (VoxelCell *) malloc(sizeof(VoxelCell) * dimHeight);
   cells = new VoxelCell[dimHeight];
}

VoxelColumn::~VoxelColumn() {
   delete [] cells;
}

inline void VoxelColumn::add(Point& point, Colour& colour) {
   int index = (point.z + offZ)  / res;
   if (index >= 0 && index < dimHeight) {
      cells[index].addPoint(point, colour);
   }
}

inline VoxelCell *VoxelColumn::get(double z) {
   int index = (z + offZ) / res;
   if (index >= 0 && index < dimHeight) {
      return &(cells[index]);
   } else {
      return NULL;
   }
}

void VoxelColumn::extractCells(PointCloudPtr cloud, int obsThresh) {
   int i;
   for (i = 0; i < dimHeight; i++) {
      if (cells[i].numObs > obsThresh) {
         Point p;
         Colour c;
         cells[i].getCellInfo(p, c);
         cloud->cloud.push_back(p);
         cloud->colours.push_back(c);
      }
   }
}

VoxelGrid::VoxelGrid(double width, double height, double res): 
         width(width), height(height), res(res) {
   dimWidth = width / res;
   //dimHeight = height / res;

   offXY = (dimWidth * res) / 2.0;
   //offZ = dimHeight / 2;

   grid = (VoxelColumn **) malloc(sizeof(VoxelColumn *) * dimWidth * dimWidth);
   int x;
   for (x = 0; x < dimWidth * dimWidth; ++x) {
      grid[x] = NULL;
   }
}

VoxelGrid::~VoxelGrid() {
   clearGrid();
   free(grid);
}

void VoxelGrid::clearGrid() {
   int x;
   for (x = 0; x < dimWidth * dimWidth; x++) {
      if (grid[x] != NULL) {
         delete grid[x];
         grid[x] = NULL;
      } 
   }
}

inline int VoxelGrid::getIndex(double x, double y) {
   int i, j;
   i = (x + offXY) / res;
   j = (y + offXY) / res;
   if (i >= 0 && i < dimWidth && j >= 0 && j < dimWidth) {
      return j * dimWidth + i;
   } else {
      return -1;
   }
   
}

void VoxelGrid::addScan(DepthPointsPtr points) {
   int x;
   for (x = 0; x < points->cloud.size(); x++) {
      int index = getIndex(points->cloud[x].x, points->cloud[x].y);
      if (index >= 0) {
         if (grid[index] == NULL) {
            grid[index] = new VoxelColumn(height, res);
         }
         grid[index]->add(points->cloud[x], points->colours[x]);
      }
   }
}

PointCloudPtr VoxelGrid::extractPoints(int obsThresh) {
   int i;
   PointCloudPtr cloud = new PointCloud();
   for (i = 0; i < dimWidth * dimWidth; i++) {
      if (grid[i] != NULL) {
         grid[i]->extractCells(cloud, obsThresh);
      }
   }
   cout << "Number of points extracted is: " << cloud->cloud.size() << " " << cloud->colours.size() << endl;
   return cloud;
}

Local3DMap::Local3DMap(Pose pose): globalPose(pose) {
}

Pose Local3DMap::getPose() {
   return globalPose;
}

void Local3DMap::updatePose(Pose pose) {
   globalPose = pose;
}




//Think about:
//How can make it easy to put onto GPU as will need to do so
//What data structure for CPU version I should use - octree or just a grid (depending on memory
//requirements of a local map) - calculate this!!
//Think of differerence between active map and non active maps
//Think of the various steps and methods that will need to perform
//- filter points
//- transform from kinect to robot_base (think of how to do for existing code as well as not doing it!!)
//- transform from robot_base to local map
//- add points
//- get points out
//- create new local map
//- change positions of local maps
//
//How coarse to fine will be approached - from the map storage end or the points in the new
//scan end?
//
//Most important:
//feedback: How to communicate position updates - kinect pos track will run slower than laser
//don't want to delay laser - have to work out something.

//For kinect points, need to:
//- Skip points
//- transform (as point cloud does in its constructor
//- get normals?
//- filter?

