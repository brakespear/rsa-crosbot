/*
 * positionTrack3D.hpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#ifndef POSITIONTRACK3D_HPP_
#define POSITIONTRACK3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot_3d_graphslam/depthPoints.hpp>

#include <crosbot_gpu/openCL.hpp>
#include <crosbot_3d_position_track/openclCommon.h>

using namespace std;
using namespace crosbot;

class PositionTrack3D {
public:

   PositionTrack3D();
   ~PositionTrack3D();

   /*
    * Initialise parameters
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Start 3d position tracking
    */
   void start();

   /*
    * Shutdown node
    */
   void stop();

   /*
    * Called for the first frame received
    */
   void initialiseFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose);

   /*
    * Process a kinect frame
    */
   Pose processFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose);

private:
   /*
    * Config attributes
    */


   //GPU configs
   int LocalSize;

   //Map dimensions
   double MapWidth;
   double MapHeight;
   double CellSize;
   //Maximum number of ICP iterations
   int MaxIterations;
   //If move more than max move, a mistake has been made so don't move
   double MaxMove;
   //ICP stops when movement in in iteration is less than this threshold
   double MoveThresh;
   //Min number of pixels used in alignment
   int MinCount;
   //Number of times an alignment can fail before a scan is added anyway
   int MaxFail;
   //Number of scans to just add to the map before aligning
   int BeginScans;
   //The meaximum number of cells in the map to search in one direction
   //before stopping
   int MaxSearchCells;
   //Initial height of robot
   double InitZ

   //derived configs
   int NumCells;
   int NumCellsWidth;
   int NumCellsHeight;

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
   int failCount;
   int beginCount;

   //Current height of the robot
   double z;

   int mapCentreX;
   int mapCentreY;
   int mapCentreZ;

   /*
    * GPU data structures
    */

   //depth points for a frame
   oclDepthPoints *points;
   cl_mem clPoints;
   cl_mem clNormals;
   size_t pointsSize;
   //config attributes
   oclPositionTrackConfig positionTrackConfig;
   cl_mem clPositionTrackConfig;
   //the local map
   cl_mem clLocalMap;
   //general information
   cl_mem clCommon;


   /*
    * GPU structure methods
    */
   void initialiseDepthPoints();
   void initialiseMap();

   void transform3D(Pose sensorPose, Pose icpPose);
   void calculateNormals();
   bool alignZ(float *zChange);
   void addScan(float zChange);
   void clearCells(int newMapX, int newMapY, int newMapZ);

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
