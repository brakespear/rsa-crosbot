/*
 * positionTrackFull3D.hpp
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#ifndef POSITIONTRACKFULL3D_HPP_
#define POSITIONTRACKFULL3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>

#include <crosbot_gpu/openCL.hpp>
#include <crosbot_3d_position_track_full/openclCommon.h>

using namespace std;
using namespace crosbot;

class PositionTrackFull3D {
public:

   PositionTrackFull3D();
   ~PositionTrackFull3D();

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
   void initialiseFrame(const sensor_msgs::ImageConstPtr& depthImage, 
         const sensor_msgs::ImageConstPtr& rgbImage, Pose sensorPose, Pose icpPose);

   /*
    * Process a kinect frame
    */
   Pose processFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage, Pose sensorPose, Pose icpPose, 
         float *floorHeight);

   /*
    * Set the params of the registered depth camera
    */
   void setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty);

private:
   /*
    * Config attributes
    */
   //Number of kinect points to skip
   //int SkipPoints;
   //Dimensions in metres of the local map
   double LocalMapWidth;
   double LocalMapHeight;
   //Total number of blocks that can be used
   int NumBlocksAllocated;
   //Number of blocks that can be active in an iteration
   int MaxNumActiveBlocks;
   //Size of a single cell in the tsdf
   double CellSize;
   //Size of a block - should be a multiple of CellSize
   double BlockSize;
   //truncation distances for the tsdf
   double TruncNeg;
   double TruncPos;

   //GPU configs
   int LocalSize;

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

   //Camera params:
   double fx;
   double fy;
   double cx;
   double cy;
   double tx;
   double ty;

   /*
    * Fields
    */
   int imageWidth;
   int imageHeight;
   //The number of depth points in each frame
   int numDepthPoints;


   /*
    * GPU data structures
    */
   //config attributes
   oclPositionTrackConfig positionTrackConfig;
   cl_mem clPositionTrackConfig;
   float *depthFrame;
   cl_mem clDepthFrame;
   size_t sizeDepthPoints;

   void initialiseImages();
   void convertFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage);

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
