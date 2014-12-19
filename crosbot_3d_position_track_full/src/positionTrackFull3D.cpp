/*
 * positionTrackFull3D.cpp
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#include <crosbot_3d_position_track_full/positionTrackFull3D.hpp>


using namespace std;
using namespace crosbot;

const string PositionTrackFull3D::file_names[] = {
   "/src/opencl/positionTrackFull3D.cl"
};
const int PositionTrackFull3D::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string PositionTrackFull3D::header_file = "/include/crosbot_3d_position_track_full/openclCommon.h";
const string PositionTrackFull3D::kernel_names[] = {
};
const int PositionTrackFull3D::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
//#define INITIALISE_MAP 0

//The maximum number of points extracted from a block can be SLICE_MULT*the number of 
//cells in a slice of a block
#define SLICE_MULT 2

PositionTrackFull3D::PositionTrackFull3D() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   FILE *file = popen("rospack find crosbot_3d_position_track_full", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;

}

void PositionTrackFull3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");

   paramNH.param<int>("LocalSize", LocalSize, 256);
   //paramNH.param<int>("SkipPoints", SkipPoints, 1);
   paramNH.param<double>("LocalMapWidth", LocalMapWidth, 12);
   paramNH.param<double>("LocalMapHeight", LocalMapHeight, 4);
   paramNH.param<int>("NumBlocksAllocated", NumBlocksAllocated, 10000);
   paramNH.param<int>("MaxNumActiveBlocks", MaxNumActiveBlocks, 2500);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("BlockSize", BlockSize, 0.2);
   paramNH.param<double>("TruncNeg", TruncNeg, 0.2);
   paramNH.param<double>("TruncPos", TruncPos, 0.3);

   NumBlocksWidth = (LocalMapWidth + 0.00001) / BlockSize;
   NumBlocksHeight = (LocalMapHeight + 0.00001) / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = (BlockSize + 0.00001) / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;

}

void PositionTrackFull3D::start() {
}

PositionTrackFull3D::~PositionTrackFull3D() {
   delete opencl_task;
   delete opencl_manager;
}

void PositionTrackFull3D::stop() {
}

void PositionTrackFull3D::initialiseFrame(const sensor_msgs::ImageConstPtr& depthImage, 
         const sensor_msgs::ImageConstPtr& rgbImage, Pose sensorPose, Pose icpPose) {
   cout << "Position track full 3D: starting compile" << endl;

   imageWidth = depthImage->width;
   imageHeight = depthImage->height;

   //set config options
   positionTrackConfig.ImageWidth = imageWidth;
   positionTrackConfig.ImageHeight = imageHeight;
   positionTrackConfig.LocalMapWidth = LocalMapWidth;
   positionTrackConfig.LocalMapHeight = LocalMapHeight;
   positionTrackConfig.BlockSize = BlockSize;
   positionTrackConfig.NumBlocksTotal = NumBlocksTotal;
   positionTrackConfig.NumBlocksWidth = NumBlocksWidth;
   positionTrackConfig.NumBlocksHeight = NumBlocksHeight;
   positionTrackConfig.CellSize = CellSize;
   positionTrackConfig.NumCellsTotal = NumCellsTotal;
   positionTrackConfig.NumCellsWidth = NumCellsWidth;
   positionTrackConfig.TruncPos = TruncPos;
   positionTrackConfig.TruncNeg = TruncNeg;
   positionTrackConfig.fx = fx;
   positionTrackConfig.fx = fy;
   positionTrackConfig.fx = cx;
   positionTrackConfig.fx = cy;
   positionTrackConfig.fx = tx;
   positionTrackConfig.fx = ty;

   clPositionTrackConfig = opencl_manager->deviceAlloc(sizeof(oclPositionTrackConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &positionTrackConfig);
   
   numDepthPoints = imageWidth * imageHeight;
   //Round up to the nearest 16 for memory coalescing
   numDepthPoints = ((numDepthPoints + 15) / 16) * 16;

   stringstream ss;
   ss << "-D NUM_DEPTH_POINTS=" << numDepthPoints <<
         " -D LOCAL_SIZE=" << LocalSize <<
         " -D NUM_CELLS=" << NumCellsTotal <<
         " -D MAX_NUM_ACTIVE_BLOCKS=" << MaxNumActiveBlocks << 
         " -D MAX_POINTS_GROUP=" << LocalSize * SLICE_MULT <<
         " -D BLOCKS_PER_GROUP=" << LocalSize / (NumCellsWidth * NumCellsWidth);
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels,
         ss.str(), header_file);

   initialiseImages();

   cout << "Depth image: " << depthImage->encoding << " " << depthImage->step << " " <<
      depthImage->width << " " << depthImage->height << " " << depthImage->header.frame_id << endl;
   cout << "RGB image: " << rgbImage->encoding << " " << rgbImage->step << " " <<
      rgbImage->width << " " << rgbImage->height << " " << rgbImage->header.frame_id << endl;

   cout << "Position track full 3D: finished compile" << endl;
}

Pose PositionTrackFull3D::processFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage, Pose sensorPose, Pose icpPose, 
         float *floorHeight) {
   convertFrame(depthImage, rgbImage);
   //preprocessing of points (only used for icp) - normals, filtering and different res's


   //icp itself
   

   //add frame
   

   //extract points??
   
   //questions:
   //- how to extract points? - for graph slam and for next iteration of position tracking?
   //- how to do the icp? directly use tdsf or use extracted points?
   //- what preprocessing do I want to do?
   return icpPose;
}

void PositionTrackFull3D::initialiseImages() {
   depthFrame = new float[imageWidth * imageHeight];
   sizeDepthPoints = sizeof(ocl_float) * numDepthPoints;
   clDepthFrame = opencl_manager->deviceAlloc(sizeDepthPoints, CL_MEM_READ_WRITE, NULL);

}

void PositionTrackFull3D::convertFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage) {
   if (depthImage->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
      cout << "ERROR: unexpected encoding of depth image" << endl;
      return;
   }

   //Extract the depth data from the image message and convert it to
   //metres in floats
   const uint16_t *rawData = reinterpret_cast<const uint16_t *>(&depthImage->data[0]);
   for (int i = 0; i < numDepthPoints; ++i) {
      uint16_t p = rawData[i];
      depthFrame[i] = (p == 0) ? NAN : (float)p * 0.001f;
   }

   writeBuffer(clDepthFrame, CL_FALSE, 0, sizeDepthPoints, depthFrame, 0, 0, 0, 
         "Copying depth points to the GPU");
}

void PositionTrackFull3D::setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty) {

   this->fx = fx;
   this->fy = fy;
   this->cx = cx;
   this->cy = cy;
   this->tx = tx;
   this->ty = ty;
}

inline int PositionTrackFull3D::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
}

void PositionTrackFull3D::writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
      size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), buffer, blocking_write,
        offset, cb, ptr, num_events_in_wait_list , event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error write buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void PositionTrackFull3D::readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
      size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), buffer, blocking_read,
         offset, cb, ptr, num_events_in_wait_list, event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error read buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}


