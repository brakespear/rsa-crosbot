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
#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_gpu/openCL.hpp>
#include <crosbot_3d_position_track_full/openclCommon.h>
#include <crosbot_3d_position_track_full/positionTrackFull3DNode.hpp>

using namespace std;
using namespace crosbot;

#define DOF 6

class PositionTrackFull3DNode;
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
         float *floorHeight, bool outputMapPoints, vector<uint8_t>& allPoints);

   /*
    * Set the params of the registered depth camera
    */
   void setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty);

   /*
    * Process a new local map
    */
   void newLocalMap(LocalMapInfoPtr localM);

   PositionTrackFull3DNode *positionTrack3DNode;
   bool UseLocalMaps;

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
   //Only extract surface points of tsdf if the cell is occupied 
   //by a depth reading
   bool UseOccupancyForSurface;
   //Max distance of a cell to the camera to be used in the tsdf
   double MaxDistance;
   //The maximum number of points that can be extracted from a block can be
   //slice mult*number of cells in a 2D slice of the block. Normally 2, but
   //can be decreased to 1 to allow higher res blocks without running out of
   //memory
   int SliceMult;
   //Number of times the absolute maximum number of points that are 
   //possible is reduced by. Normally 1, but can be increased to allow
   //higher res blocks
   int MaxPointsFrac;
   //Extract blocks if they are seen again after they have been extracted into a
   //previous local map
   bool ReExtractBlocks;
   //When downsampling won't include points 3*this distance away from this point
   //Also used during bilateral filter
   double DepthDistThreshold;
   //Window size of the bilateral filter
   int FilterWindowSize;
   //Scaling for pixel position used in the bilateral filter
   double FilterScalePixel;
   //How many pixels will be skipped in the checkBlocks kernel
   int SkipNumCheckBlocks;
   //When the movement in an iteration falls below this, it will stop
   double MoveThresh;
   //Should odometry be used as a guess?
   bool UseOdometry;
   //Should the points just be added to the map or should they be aligned first?
   bool UseICP;
   //Only align in Z
   bool UseICPZOnly;

   double DistThresh;
   double DotThresh;
   double DistThresh2;
   double DotThresh2;
   double DistThresh4;
   double DotThresh4;

   //GPU configs
   int LocalSize;

   //Derived params
   
   int NumBlocksTotal;
   int NumBlocksWidth;
   int NumBlocksHeight;
   //Cell counts for each block
   int NumCellsTotal;
   int NumCellsWidth;
   //Maximum number of points that can be extracted
   int MaxPoints;

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
   cl_mem clFiltDepthFrame;
   size_t sizeDepthPoints;
   oclColourPoints *colourFrame;
   cl_mem clColourFrame;
   size_t sizeColourPoints;
   cl_mem clDepthFrameXYZ;
   cl_mem clNormalsFrame;
   size_t sizeDepthFrameXYZ;
   cl_mem clLocalMapBlocks;
   cl_mem clLocalMapCells;
   cl_mem clLocalMapCommon;
   size_t numActiveBlocksOffset;
   size_t numBlocksToExtractOffset;
   size_t numPointsOffset;
   size_t numBlocksToDeleteOffset;
   size_t highestBlockNumOffset;
   size_t icpResultsOffset;
   size_t icpScaleResultsOffset;
   cl_mem clPointCloud; //Also predPoints
   cl_mem clColours;  //Also tempStore
   cl_mem clNormals; //Also predNormals

   cl_mem clDepthFrameXYZ2;
   cl_mem clNormalsFrame2;
   cl_mem clDepthFrameXYZ4;
   cl_mem clNormalsFrame4;

   //The current icp pose output by this node
   Pose icpFullPose;
   //Index of the cell in the local map where the robot currently is
   ocl_int3 mapCentre;
   //Current number of active blocks
   int numActiveBlocks;
   //Current highest block num
   int highestBlockNum;
   //The ICP pose from the previous frame
   tf::Transform oldICP;

   //local map infos
   LocalMapInfoPtr newLocalMapInfo;
   LocalMapInfoPtr currentLocalMapInfo;
   Pose currentLocalMapICPPose;
   Pose newLocalMapICPPose;


   FILE *f;

   void setupGPU();
   void initialiseImages();
   void initialiseLocalMap();
   void convertFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage);

   void clearLocalMap();
   void checkBlocksExist(int numDepthPoints, tf::Transform trans);
   void addRequiredBlocks();
   void addFrame(tf::Transform trans);
   void markForExtraction(ocl_int3 newMapCentre, bool outputLocalMap);
   void extractPoints(int numBlocksToExtract, bool extractNorms);
   void transformPoints(int numPoints, bool transformNorms, tf::Transform trans);
   void addPointsToLocalMap(int numPoints);
   void clearBlocks();
   void markAllForExtraction();
   void outputAllPoints(int numPoints, vector<uint8_t>& allPoints);
   void calculateNormals();
   void alignICP(tf::Transform sensorPose, tf::Transform newPose);
   void alignRayTraceICP(tf::Transform sensorPose, tf::Transform newPose);
   void predictSurface(tf::Transform trans);
   void downsampleDepth();
   void alignZOnlyICP(tf::Transform sensorPose, tf::Transform newPose);

   void outputDebuggingImage(tf::Transform trans);

   void bilateralFilter();
   void combineICPResults(int numGroups, int numResults);


   void scaleICP(int numGroups, tf::Transform trans);
   void scaleRayTraceICP(int numGroups, float scale[6]);

   //Solve Ax = b. Requires A to be symmetric. Only looks at bottom left of A
   void solveCholesky(float A[DOF][DOF], float b[DOF], float x[DOF]);

   void mult6x6Vector(float A[DOF][DOF], float b[DOF], float res[DOF]);
   void multVectorTrans(float vec[DOF], float res[DOF][DOF]);

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
