/*
 * OgmbicpGPU.hpp
 *
 * Created on: 13/12/2013
 *     Author: adrianr
 */

#ifndef OGMBICP_GPU_HPP_
#define OGMBICP_GPU_HPP_

#include <crosbot_ogmbicp/Ogmbicp.hpp>
#include <crosbot_gpu/openCL.hpp>
#include <crosbot_ogmbicp_gpu/openclCommon.h>

class OgmbicpGPU : public Ogmbicp {
public:
   /*
    * Config attributes for GPU version of ogmbicp
    */

   //Inherited methods from ogmbicp.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose sensorPose, PointCloudPtr cloud);
   void updateTrack(Pose sensorPose, PointCloudPtr cloud);

   OgmbicpGPU();
   ~OgmbicpGPU();
protected:

   OpenCLManager *opencl_manager;
   OpenCLTask *opencl_task;

   void getLocalMap(LocalMapPtr curMap);
private:

   /*
    * Config attributes for gpu version of ogmbicp
    */
   //The size of an opencl work group
   int LocalSize;
   //Most number of laser points that can be stored in each 3D cell
   int MaxLaserPointsInCell;
   //Maximum number of active cells at any time
   int MaxActiveCells;

   //Current x and y pose of the robot rounded to the nearest cell
   double pos_x;
   double pos_y;
   //Number of laser points in first scan
   int numLaserPoints;
   //Number of laser points in the previous iteration
   int oldNumPoints;
   //Number of 2d cells in the map
   int totalMapSize;
   //number of voxels in all the active cells
   int numActiveCellVoxels;
   //Number of laser points that can be stored in all the voxels
   int numLaserPointCoords;
   //size of the points arrays in the clPoints struct
   size_t arrayPointsSize;
   //size of the rest of the clPoints struct
   size_t restPointsSize;

   /* 
    * Opencl data structures
    */
   //Results from ogmbicp
   oclResults *results;
   cl_mem clResults;
   //Config params for ogmbicp
   configValues config_values;
   cl_mem clConfig;
   //partial results from position tracking to report if another ineration is needed
   oclPartialResults *partialResults;
   cl_mem clPartialResults;
   //Map structure that gets given to the cpu for viewing
   oclSharedMap *sharedMap;
   cl_mem clSharedMap;
   //Laser points struct for opencl
   oclLaserPoints *points;
   cl_mem clPoints;
   //The local map structure stored on the gpu
   cl_mem clLocalMap3D;

   //Debugging memory structure
   cl_mem clRes;


   //Constants for opencl code
   static const string file_names[];
   static const int num_files;
   static const string header_file;
   static const string kernel_names[];
   static const int num_kernels;

   //Root directory of the package, because ati doesn't understand the meaning of relative paths
   string rootDir;

   //debugging for timings
   ros::WallDuration totalTime;
   int numIterations;
   int avNumIts;

   //center point cloud around the robot's current position
   PointCloudPtr centerPointCloud(PointCloud &p, Pose curPose, Pose sensorPose);
   //Initialise results struct for opencl
   void initialiseResults();
   //Initialise map for opencl
   void initialiseMap();
   //Initialise points struct for opencl
   void initialisePoints();
   //Set the common arguments of the opencl kernels
   void setKernelArgs();
   //Copy laser points across to gpu data structure
   int prepareLaserPoints(PointCloudPtr p, int numPoints, Pose sensorPose);
   //Set the current number of laser points as kernel args for the appriopriate kernels
   void setNumPointsKernelArgs(int numPoints);
   //Returns the global work size needed if there are numThreads for the kernel
   //(global work size needs to be a multiple of LocalSize)
   inline int getGlobalWorkSize(int numThreads);
   void queueMapUpdateKernels(int numThreads);

};

#endif
