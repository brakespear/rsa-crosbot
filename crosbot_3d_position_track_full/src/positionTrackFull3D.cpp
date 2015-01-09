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
   "clearLocalMap",
   "checkBlocksExist",
   "addRequiredBlocks",
   "addFrame"
};
const int PositionTrackFull3D::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
#define CLEAR_LOCAL_MAP 0
#define CHECK_BLOCKS_EXIST 1
#define ADD_RREQUIRED_BLOCKS 2
#define ADD_FRAME 3

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

   mapCentre.x = 0;
   mapCentre.y = 0;
   mapCentre.z = 0;

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
   paramNH.param<bool>("UseOccupancyForSurface", UseOccupancyForSurface, true);
   paramNH.param<double>("MaxDistance", MaxDistancem 5.0);

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
   positionTrackConfig.NumBlocksAllocated = NumBlocksAllocated;
   positionTrackConfig.TruncPos = TruncPos;
   positionTrackConfig.TruncNeg = TruncNeg;
   positionTrackConfig.fx = fx;
   positionTrackConfig.fx = fy;
   positionTrackConfig.fx = cx;
   positionTrackConfig.fx = cy;
   positionTrackConfig.fx = tx;
   positionTrackConfig.fx = ty;
   positionTrackConfig.UseOccupancyForSurface = UseOccupancyForSurface;
   positionTrackConfig.MaxDistance = MaxDistance;

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
         " -D BLOCKS_PER_GROUP=" << LocalSize / (NumCellsWidth * NumCellsWidth) <<
         " -D NUM_BLOCKS_ALLOCATED=" << NumBlocksAllocated;
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels,
         ss.str(), header_file);

   initialiseImages();
   initialiseLocalMap();
   clearLocalMap();

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
  
   //TODO: do this properly when have icp working
   icpFullPose = icpPose;

   //preprocessing of points (only used for icp) - normals, filtering and different res's


   //icp itself
   

   //add frame
   tf::Transform offset = icpFullPose.toTF() * sensorPose.toTF();

   checkBlocksExist(numDepthPoints, offset);
   readBuffer(clLocalMapCommon, CL_TRUE, numActiveBlocksOffset, 
         sizeof(int), &numActiveBlocks, 0, 0, 0, "Reading num active blocks");
   if (numActiveBlocks > MaxNumActiveBlocks) {
      numActiveBlocks = MaxNumActiveBlocks;
   }
   addRequiredBlocks();
   addFrame(offset);

   //extract points
   
   mapCentre.x = icpFullPose.position.x / BlockSize;
   mapCentre.y = icpFullPose.position.y / BlockSize;
   mapCentre.z = icpFullPose.position.z / BlockSize;
   
   //questions:
   //- how to extract points? - for graph slam and for next iteration of position tracking?
   //- how to do the icp? directly use tdsf or use extracted points?
   //- what preprocessing do I want to do?
   return icpFullPose;
}

void PositionTrackFull3D::initialiseImages() {
   depthFrame = new float[imageWidth * imageHeight];
   sizeDepthPoints = sizeof(ocl_float) * numDepthPoints;
   clDepthFrame = opencl_manager->deviceAlloc(sizeDepthPoints, CL_MEM_READ_WRITE, NULL);

   colourFrame = new oclColourPoints;
   colourFrame->r = new unsigned char[numDepthPoints * 3];
   colourFrame->g = &(colourFrame->r[numDepthPoints]);
   colourFrame->b = &(colourFrame->r[numDepthPoints * 2]);
   sizeColourPoints = sizeof(unsigned char) * numDepthPoints * 3;
   clColourFrame = opencl_manager->deviceAlloc(sizeColourPoints, CL_MEM_READ_WRITE, NULL);

}

void PositionTrackFull3D::initialiseLocalMap() {
   clLocalMapBlocks = opencl_manager->deviceAlloc(sizeof(ocl_int) * NumBlocksTotal, 
         CL_MEM_READ_WRITE, NULL);

   oclLocalBlock localBlock;
   size_t localBlockSize = (sizeof(*(localBlock.distance)) +
      sizeof(*(localBlock.weight)) + sizeof(*(localBlock.pI)) + sizeof(*(localBlock.r)) * 4) 
      * NumCellsTotal + sizeof(localBlock.blockIndex);
   clLocalMapCells = opencl_manager->deviceAlloc(localBlockSize * NumBlocksAllocated, 
         CL_MEM_READ_WRITE, NULL);

   size_t commonSize = sizeof(ocl_int) * (5 + MaxNumActiveBlocks + 2*NumBlocksAllocated);
   clLocalMapCommon = opencl_manager->deviceAlloc(commonSize,
         CL_MEM_READ_WRITE, NULL);

   oclLocalMapCommon com;
   numActiveBlocksOffset = (unsigned char *)&(com.numActiveBlocks) - (unsigned char *)&(com);
   numBlocksToExtractOffset = (unsigned char *)&(com.numBlocksToExtract) - 
      (unsigned char *)&(com);
   numPointsOffset = (unsigned char *)&(com.numPoints) - (unsigned char *)&(com);

}

//TODO: make processing of rgb images more flexible (more encodings, cope with
//different width of rgb and depth images
void PositionTrackFull3D::convertFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage) {
   if (depthImage->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
      cout << "ERROR: unexpected encoding of depth image" << endl;
      return;
   }
   if (depthImage->header.frame_id != rgbImage->header.frame_id) {
      cout << "ERROR: depth and rgb images should be in the same frame!" << endl;
      return;
   }

   if (depthImage->width != rgbImage->width) {
      cout << "ERROR: width of rgb and depth images are different" << endl;
      return;
   }
   if (rgbImage->encoding != sensor_msgs::image_encodings::RGB8) {
      cout << "ERROR: unexpected encoding of rgb image" << endl;
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

   //Assume rgb8 encoding
   int colourStep = 3;
   int rOff = 0;
   int gOff = 1;
   int bOff = 2;
   //Extract the colour data from the image message
   int colourId = 0;
   for (int i = 0; i < numDepthPoints; ++i, colourId += colourStep) {
      colourFrame->r[numDepthPoints] = rgbImage->data[colourId + rOff];
      colourFrame->g[numDepthPoints] = rgbImage->data[colourId + gOff];
      colourFrame->b[numDepthPoints] = rgbImage->data[colourId + bOff];
   }
   writeBuffer(clColourFrame, CL_FALSE, 0, sizeColourPoints, colourFrame->r, 0, 0, 0, 
         "Copying colour points to the GPU");
}

void PositionTrackFull3D::clearLocalMap() {
   int kernelI = CLEAR_LOCAL_MAP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   int globalSize = LocalSize * 20;
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::checkBlocksExist(int numPoints, tf::transform trans) {
   tf::Matrix3x3 basis = trans.getBasis();
   tf::Vector3 origin = trans.getOrigin();

   ocl_float3 clBasis[3];
   ocl_float3 clOrigin;
   for (int j = 0; j < 3; j++) {
      clBasis[j].x = basis[j][0];
      clBasis[j].y = basis[j][1];
      clBasis[j].z = basis[j][2];
   }
   clOrigin.x = origin[0];
   clOrigin.y = origin[1];
   clOrigin.z = origin[2];
   int globalSize = getGlobalWorkSize(numPoints);
   int kernelI = CHECK_BLOCKS_EXIST;

   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrame);
   opencl_task->setArg(5, kernelI, sizeof(int), &numPoints);
   opencl_task->setArg(6, kernelI, sizeof(ocl_int3), &mapCentre);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clBasis[2]);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::addRequiredBlocks() {
   int kernelI = ADD_REQUIRED_BLOCKS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);

   int globalSize = getGlobalWorkSize(numActiveBlocks);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::addFrame(tf::Transform trans) {
   //trans takes point from sensor frame to icp frame.
   //We need icp frame to sensor frame, so we take the inverse
   tf::Transform offset = trans.inverse();
   tf::Matrix3x3 basis = offset.getBasis();
   tf::Vector3 origin = offset.getOrigin();

   ocl_float3 clBasis[3];
   ocl_float3 clOrigin;
   for (int j = 0; j < 3; j++) {
      clBasis[j].x = basis[j][0];
      clBasis[j].y = basis[j][1];
      clBasis[j].z = basis[j][2];
   }
   clOrigin.x = origin[0];
   clOrigin.y = origin[1];
   clOrigin.z = origin[2];
   
   int kernelI = ADD_FRAME;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrame);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clColourFrame);
   opencl_task->setArg(6, kernelI, sizeof(int), &numActiveBlocks);
   opencl_task->setArg(7, kernelI, sizeof(ocl_int3), &mapCentre);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(11, kernelI, sizeof(ocl_float3), &clBasis[2]);

   //If there are more cells in a block than threads in a group,
   //have one block per group. Otherwise have multiple blocks
   //per group
   int globalSize;
   if (NumCellsTotal >= LocalSize) {
      globalSize = numActiveBlocks * LocalSize;
   } else {
      int numBlocksPerGroup = LocalSize / NumCellsTotal;
      globalSize = (numActiveBlocks / numBlocksPerGroup) * LocalSize;
      if (numActiveBlocks % numBlocksPerGroup != 0) {
         globalSize += LocalSize;
      }
   }
   
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
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


