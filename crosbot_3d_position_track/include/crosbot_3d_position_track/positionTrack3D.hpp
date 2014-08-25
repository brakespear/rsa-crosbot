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
   void processFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose);

private:
   /*
    * Config attributes
    */


   //GPU configs
   int LocalSize;

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

   /*
    * GPU data structures
    */
   oclDepthPoints *points;
   cl_mem clPoints;
   size_t pointsSize;

   /*
    * GPU structure methods
    */
   void initialiseDepthPoints();

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
