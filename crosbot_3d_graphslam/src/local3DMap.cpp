/*
 * local3DMap.cpp
 *
 * Created on: 05/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam/local3DMap.hpp>


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

