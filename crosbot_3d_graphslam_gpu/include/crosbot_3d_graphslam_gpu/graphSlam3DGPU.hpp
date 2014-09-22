/*
 * graphSlam3DGPU.hpp
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAM3DGPU_HPP_
#define GRAPHSLAM3DGPU_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot_gpu/openCL.hpp>
#include <crosbot_3d_graphslam_gpu/openclCommon.h>
#include <crosbot_3d_graphslam/graphSlam3D.hpp>

using namespace std;
using namespace crosbot;

class GraphSlam3DGPU : public GraphSlam3D {
public:
   //Inherited methods from graphSlam3D.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose);
   void newLocalMap(LocalMapInfoPtr localMapInfo);
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions);

   GraphSlam3DGPU();
   ~GraphSlam3DGPU();

private:
   /*
    * GPU config params
    */
   int LocalSize;
   int NumBlocksAllocated;
   int MaxNumActiveBlocks;

   //Params that can probably move to general code
   //Size of a block - should be a multiple of CellSize
   double BlockSize;
   double TruncNeg;
   double TruncPos;

   //Derived params
   int NumBlocksTotal;
   int NumBlocksWidth;
   int NumBlocksHeight;
   //Cell counts for each block
   int NumCellsTotal;
   int NumCellsWidth;

   //GPU fields
   OpenCLManager *opencl_manager;
   OpenCLTask *opencl_task;

   //Constants for opencl
   static const string file_names[];
   static const int num_files;
   static const string header_file;
   static const string kernel_names[];
   static const int num_kernels;
   string rootDir;

   /*
    * Fields
    */
   //The number of depth points each frame
   int numDepthPoints;
   int numActiveBlocks;

   /*
    * GPU data structures
    */

   //depth points for a frame
   oclDepthPoints *points;
   cl_mem clPoints;
   size_t pointsSize;
   size_t coloursSize;
   //config attributes
   oclGraphSlam3DConfig graphSlam3DConfig;
   cl_mem clGraphSlam3DConfig;
   //local map structures
   cl_mem clLocalMapBlocks;
   cl_mem clLocalMapCells;
   cl_mem clLocalMapCommon;
   size_t numActiveBlocksOffset;
   size_t numPointsOffset;

   vector<Local3DMap *> maps;
   int currentMap;

   bool hasInitialised;

   /*
    * GPU structure methods
    */
   void initialiseDepthPoints();
   void initialiseLocalMap();
   void initialiseGraphSlam(DepthPointsPtr depthPoints);
   /*
    * GPU wrapper methods
    */
   void clearLocalMap(int numBlocks);
   void checkBlocksExist(int numPoints, tf::Transform trans);
   void addRequiredBlocks();
   void addFrame(tf::Transform trans);
   void extractPoints(int numBlocks, cl_mem &clPointCloud, cl_mem &clColours);
   PointCloudPtr copyPoints(int numPoints, cl_mem &clPointCloud, cl_mem &clColours);


   /*
    * GPU helper methods
    */
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


