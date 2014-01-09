/*
 * GraphSlamGPU.hpp
 *
 * Created on: 8/1/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAM_GPU_HPP_
#define GRAPHSLAM_GPU_HPP_

#include <crosbot_graphslam/graphSlam.hpp>
#include <crosbot_gpu/openCL.hpp>
#include <crosbot_graphslam_gpu/openclCommon.h>

class GraphSlamGPU : public GraphSlam {
public:

   //Inherited methods from graphSlam.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose icpPose, PointCloudPtr cloud);
   void updateTrack(Pose icpPose, PointCloudPtr cloud);

   GraphSlamGPU();
   ~GraphSlamGPU();
protected:
   OpenCLManager *opencl_manager;
   OpenCLTask *opencl_task;

   void getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices);
   void getGlobalMapPosition(int mapIndex, double& gx, double& gy, 
         double& gth);
private:
   /*
    * Config attributes for GPU version of graph slam
    */
   int LocalSize;

   /*
    * Opencl data structures
    */
   //Config params for slam
   slamConfig slam_config;
   cl_mem clSlamConfig;
   //Laser points
   oclLaserPoints *points;
   cl_mem clPoints;
   //Data required for the entire slam algorithm on the gpu
   cl_mem clSlamCommon;
   //Data structure holding the array of local maps
   cl_mem clSlamLocalMap;
   //Data structure holding the global map
   cl_mem clGlobalMap;
   int *globalMap;
   //Data structure holding the heights of each global map points
   cl_mem clGlobalMapHeights;
   ocl_float *globalMapHeights;
   //Storage for global positions of each map for snaps
   cl_mem clGlobalMapPositions;
   ocl_float *globalMapPositions;

   //debugging output
   cl_mem clRes;

   //Constants for opencl code
   static const string file_names[];
   static const int num_files;
   static const string header_file;
   static const string kernel_names[];
   static const int num_kernels;
   //Root directory of the package, because ati doesn't understand the meaning of relative paths
   string rootDir;
   
   //Should the map be totally redrawn?
   bool resetMap;
   //Offset of the robot in the current local map relative to ICP frame 
   double offsetFromParentX;
   double offsetFromParentY;
   double offsetFromParentTh;
   //Current offset of robot in map
   ocl_float4 currentOffset;
   //The starting ICP angle for the current local map
   double startICPTheta;
   //Total number of cells in the occupancy grid of a local mao
   int localOGSize;
   //Number of laser points in the first scan
   int numLaserPoints;
   //Size of the array of points copied to the gpu
   size_t arrayPointsSize;
   //current maximum number of global points
   int totalGlobalPoints;
   //current maximum number of local maps
   int totalLocalMaps;
   //Number of laser points in each local map
   int *numLocalMapPoints;
   //The most laser points in a single local map
   int maxNumLocalPoints;


   //debugging for timings
   ros::WallDuration totalTime;
   int numIterations;

   /*
    * Private methods
    */

   //Performs loop closing tests and sets up a new local map if needed
   void finishMap(double angleError, double icpTh, Pose icpPose);
   //Initialise points struct for opencl
   void initialisePoints();
   //Initialise the structures used for slam
   void initialiseSlamStructures();
   //Set the common arguments of the opencl kernels
   void setKernelArgs();
   //Returns the global work size needed if there are numThreads for the
   //kernel (global work size needs to be a multiple of LocalSize)
   inline int getGlobalWorkSize(int numThreads);
   //Write memory to the GPU. Wrapes clEnqueueWriteBuffer
   void writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
         size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg);
   //read memory from the GPU. Wraps clEnqueueReadBuffer
   void readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
         size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg);


};

#endif
