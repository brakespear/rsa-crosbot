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
   "addFrame",
   "markForExtraction",
   "markAllForExtraction",
   "extractPoints",
   "transformPoints",
   "clearBlocks"
};
const int PositionTrackFull3D::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
#define CLEAR_LOCAL_MAP 0
#define CHECK_BLOCKS_EXIST 1
#define ADD_REQUIRED_BLOCKS 2
#define ADD_FRAME 3
#define MARK_FOR_EXTRACTION 4
#define MARK_ALL_FOR_EXTRACTION 5
#define EXTRACT_POINTS 6
#define TRANSFORM_POINTS 7
#define CLEAR_BLOCKS 8

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

   UseLocalMaps = false;
   newLocalMapInfo = NULL;
   currentLocalMapInfo = NULL;

}

void PositionTrackFull3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");

   paramNH.param<int>("LocalSize", LocalSize, 256);
   //paramNH.param<int>("SkipPoints", SkipPoints, 1);
   paramNH.param<double>("LocalMapWidth", LocalMapWidth, 8);
   paramNH.param<double>("LocalMapHeight", LocalMapHeight, 4);
   paramNH.param<int>("NumBlocksAllocated", NumBlocksAllocated, 10000);
   paramNH.param<int>("MaxNumActiveBlocks", MaxNumActiveBlocks, 2500);
   paramNH.param<double>("CellSize", CellSize, 0.025);
   paramNH.param<double>("BlockSize", BlockSize, 0.2);
   paramNH.param<double>("TruncNeg", TruncNeg, 0.2);
   paramNH.param<double>("TruncPos", TruncPos, 0.3);
   paramNH.param<bool>("UseOccupancyForSurface", UseOccupancyForSurface, true);
   paramNH.param<double>("MaxDistance", MaxDistance, 8.0);
   //If have cell size of 0.0125, use SliceMult = 1, MaxPointsFrac = 2,
   //NumBlocksAlloctacated = 5000.
   //Otherwise can use NumBlocksAllocated = 10000, SliceMult = 2, MaxPointsFrac = 1
   paramNH.param<int>("SliceMult", SliceMult, 2);
   paramNH.param<int>("MaxPointsFrac", MaxPointsFrac, 1);

   NumBlocksWidth = (LocalMapWidth + 0.00001) / BlockSize;
   NumBlocksHeight = (LocalMapHeight + 0.00001) / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = (BlockSize + 0.00001) / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;
   
   int maxPointsPerBlock = SliceMult * NumCellsWidth * NumCellsWidth;
   MaxPoints = maxPointsPerBlock * NumBlocksAllocated / MaxPointsFrac; //numBlocks

}

void PositionTrackFull3D::start() {
}

PositionTrackFull3D::~PositionTrackFull3D() {
   opencl_manager->deviceRelease(clPositionTrackConfig);
   opencl_manager->deviceRelease(clDepthFrame);
   opencl_manager->deviceRelease(clColourFrame);
   opencl_manager->deviceRelease(clLocalMapBlocks);
   opencl_manager->deviceRelease(clLocalMapCells);
   opencl_manager->deviceRelease(clLocalMapCommon);
   opencl_manager->deviceRelease(clPointCloud);
   opencl_manager->deviceRelease(clColours);
   opencl_manager->deviceRelease(clNormals);
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
   positionTrackConfig.fy = fy;
   positionTrackConfig.cx = cx;
   positionTrackConfig.cy = cy;
   positionTrackConfig.tx = tx;
   positionTrackConfig.ty = ty;
   positionTrackConfig.UseOccupancyForSurface = UseOccupancyForSurface;
   positionTrackConfig.MaxDistance = MaxDistance;
   positionTrackConfig.MaxPoints = MaxPoints;

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
         " -D MAX_POINTS_GROUP=" << LocalSize * SliceMult <<
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
         float *floorHeight, bool outputMapPoints, vector<uint8_t>& allPoints) {

   bool outputLocalMap = false;
   if (UseLocalMaps && currentLocalMapInfo != NULL && newLocalMapInfo != NULL &&
         currentLocalMapInfo->index != newLocalMapInfo->index) {
      outputLocalMap = true;
   }     

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
   //cout << "numActiveBlocks are: " << numActiveBlocks << endl;
   addRequiredBlocks();
   addFrame(offset);

   readBuffer(clLocalMapCommon, CL_TRUE, highestBlockNumOffset,
         sizeof(int), &highestBlockNum, 0, 0, 0, "Reading highest block num");
   //cout << "Highest block num is: " << highestBlockNum << " cent is: " << mapCentre.x <<
   //  " " << mapCentre.y << " " << mapCentre.z << endl;
   //cout << "pose is: " << icpPose.position.x << " " << icpPose.position.y << " " <<
   //   icpPose.position.z << endl;

   //extract points
   ocl_int3 newMapCentre;
   newMapCentre.x = icpFullPose.position.x / BlockSize;
   newMapCentre.y = icpFullPose.position.y / BlockSize;
   newMapCentre.z = icpFullPose.position.z / BlockSize;

   if (newMapCentre.x != mapCentre.x || newMapCentre.y != mapCentre.y ||
         newMapCentre.z != mapCentre.z || outputLocalMap) {
      markForExtraction(newMapCentre, outputLocalMap);
      int numBlocksToExtract;
      readBuffer(clLocalMapCommon, CL_TRUE, numBlocksToExtractOffset,
            sizeof(int), &numBlocksToExtract, 0, 0, 0, "reading num of blocks to extract");
      if (UseLocalMaps && numBlocksToExtract > 0) {
         extractPoints(numBlocksToExtract, true);

         int numPoints;
         readBuffer(clLocalMapCommon, CL_TRUE, numPointsOffset,
            sizeof(int), &numPoints, 0, 0, 0, "reading num of points extracted");
         if (numPoints > MaxPoints) {
            numPoints = MaxPoints;
         }

         tf::Transform trans = currentLocalMapICPPose.toTF().inverse();
         transformPoints(numPoints, true, trans);
         addPointsToLocalMap(numPoints);
      }
   }

   if (outputMapPoints) {
      cout << "outputting map points now" << endl;
      int temp = 0;
      writeBuffer(clLocalMapCommon, CL_FALSE, numBlocksToExtractOffset, 
            sizeof(int), &temp, 0, 0, 0, "reseting num blocks to extract offset");
      writeBuffer(clLocalMapCommon, CL_FALSE, numPointsOffset, 
            sizeof(int), &temp, 0, 0, 0, "reseting num points offset");

      markAllForExtraction();
      int numBlocksToExtract;
      readBuffer(clLocalMapCommon, CL_TRUE, numBlocksToExtractOffset,
            sizeof(int), &numBlocksToExtract, 0, 0, 0, "reading num of blocks to extract");
      cout << numBlocksToExtract << " blocks to extract" << endl;

      extractPoints(numBlocksToExtract, false);

         
      int numPoints;
      readBuffer(clLocalMapCommon, CL_TRUE, numPointsOffset,
         sizeof(int), &numPoints, 0, 0, 0, "reading num of points extracted");
      cout << "Number of points that were extracted: " << numPoints << " " << MaxPoints << endl;
      if (numPoints > MaxPoints) {
         numPoints = MaxPoints;
      }

      //Should the points be transformed to be relative to the robot?
      //tf::Transform trans = icpFullPose.toTF().inverse();
      //transformPoints(numPoints, false, trans);

      outputAllPoints(numPoints, allPoints);               
   }
   
   if (newMapCentre.x != mapCentre.x || newMapCentre.y != mapCentre.y ||
         newMapCentre.z != mapCentre.z) {
      clearBlocks();
   }

   if (outputLocalMap) {
      //transform to be relative to currentLocalMapICPPose
      //copy extracted points
      //add points and normals to currentLocalMapInfo

      positionTrack3DNode->publishLocalMap(currentLocalMapInfo);
      currentLocalMapInfo = newLocalMapInfo;
      currentLocalMapICPPose = newLocalMapICPPose;
   }
   mapCentre = newMapCentre;

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

   clPointCloud = opencl_manager->deviceAlloc(sizeof(ocl_float) * MaxPoints * 3, 
         CL_MEM_READ_WRITE, NULL);
   clColours = opencl_manager->deviceAlloc(sizeof(unsigned char) * MaxPoints * 3,
         CL_MEM_READ_WRITE, NULL);
   clNormals = opencl_manager->deviceAlloc(sizeof(ocl_float) * MaxPoints * 3,
         CL_MEM_READ_WRITE, NULL);
}

void PositionTrackFull3D::initialiseLocalMap() {
   clLocalMapBlocks = opencl_manager->deviceAlloc(sizeof(ocl_int) * NumBlocksTotal, 
         CL_MEM_READ_WRITE, NULL);

   oclLocalBlock localBlock;
   size_t localBlockSize = (sizeof(*(localBlock.distance)) +
      sizeof(*(localBlock.weight)) + sizeof(*(localBlock.pI)) + sizeof(*(localBlock.r)) * 4) 
      * NumCellsTotal + sizeof(localBlock.blockIndex) + sizeof(localBlock.haveExtracted);
   clLocalMapCells = opencl_manager->deviceAlloc(localBlockSize * NumBlocksAllocated, 
         CL_MEM_READ_WRITE, NULL);

   size_t commonSize = sizeof(ocl_int) * (6 + MaxNumActiveBlocks + 3*NumBlocksAllocated);
   clLocalMapCommon = opencl_manager->deviceAlloc(commonSize,
         CL_MEM_READ_WRITE, NULL);

   oclLocalMapCommon com;
   numActiveBlocksOffset = (unsigned char *)&(com.numActiveBlocks) - (unsigned char *)&(com);
   numBlocksToExtractOffset = (unsigned char *)&(com.numBlocksToExtract) - 
      (unsigned char *)&(com);
   numPointsOffset = (unsigned char *)&(com.numPoints) - (unsigned char *)&(com);
   numBlocksToDeleteOffset = (unsigned char *)&(com.numBlocksToDelete) - 
      (unsigned char *)&(com);
   highestBlockNumOffset = (unsigned char *)&(com.highestBlockNum) - (unsigned char *)&(com);

}

//TODO: make processing of rgb images more flexible (more encodings, cope with
//different width of rgb and depth images
void PositionTrackFull3D::convertFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage) {
   bool isDepthMM = false;
   if (depthImage->encoding == sensor_msgs::image_encodings::TYPE_16UC1) {
      isDepthMM = true;
   } else if (depthImage->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
      isDepthMM = false;
   } else { 
      cout << "ERROR: unexpected encoding of depth image" << endl;
      cout << depthImage->encoding << endl;
      return;
   }

   /*if (depthImage->encoding != sensor_msgs::image_encodings::TYPE_16UC1) {
      cout << "ERROR: unexpected encoding of depth image" << endl;
      cout << depthImage->encoding << endl;
      return;
   }*/
   if (depthImage->header.frame_id != rgbImage->header.frame_id) {
      cout << "ERROR: depth and rgb images should be in the same frame!" << endl;
      return;
   }

   if (depthImage->width != rgbImage->width) {
      cout << "ERROR: width of rgb and depth images are different" << endl;
      return;
   }
   //Assume rgb8 encoding
   int colourStep = 0;
   int rOff = 0;
   int gOff = 0;
   int bOff = 0;
   if (rgbImage->encoding == sensor_msgs::image_encodings::RGB8) {
      colourStep = 3;
      rOff = 0;
      gOff = 1;
      bOff = 2;
   } else if (rgbImage->encoding == sensor_msgs::image_encodings::BGR8) {
      colourStep = 3;
      rOff = 2;
      gOff = 1;
      bOff = 0;
   } else if (rgbImage->encoding == sensor_msgs::image_encodings::MONO8) {
      colourStep = 1;
      rOff = 0;
      gOff = 0;
      bOff = 0;
   } else {
      cout << "ERROR: unexpected encoding of rgb image" << endl;
      cout << rgbImage->encoding << endl;
      return;
   }

   if (isDepthMM) {
      //Extract the depth data from the image message and convert it to
      //metres in floats
      const uint16_t *rawData = reinterpret_cast<const uint16_t *>(&depthImage->data[0]);
      for (int i = 0; i < numDepthPoints; ++i) {
         uint16_t p = rawData[i];
         depthFrame[i] = (p == 0) ? NAN : (float)p * 0.001f;
      }
   } else {
      const float *rawData = reinterpret_cast<const float *>(&depthImage->data[0]);
      for (int i = 0; i < numDepthPoints; ++i) {
         depthFrame[i] = rawData[i];
      }
   }
   writeBuffer(clDepthFrame, CL_FALSE, 0, sizeDepthPoints, depthFrame, 0, 0, 0, 
         "Copying depth points to the GPU");

   //Extract the colour data from the image message
   int colourId = 0;
   for (int i = 0; i < numDepthPoints; ++i, colourId += colourStep) {
      colourFrame->r[i] = rgbImage->data[colourId + rOff];
      colourFrame->g[i] = rgbImage->data[colourId + gOff];
      colourFrame->b[i] = rgbImage->data[colourId + bOff];
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

void PositionTrackFull3D::checkBlocksExist(int numDepthPoints, tf::Transform trans) {
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
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = CHECK_BLOCKS_EXIST;

   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrame);
   opencl_task->setArg(5, kernelI, sizeof(int), &numDepthPoints);
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

void PositionTrackFull3D::markForExtraction(ocl_int3 newMapCentre, bool outputLocalMap) {
   int kernelI = MARK_FOR_EXTRACTION;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(ocl_int3), &mapCentre);
   opencl_task->setArg(5, kernelI, sizeof(ocl_int3), &newMapCentre);
   int isFullExtract = outputLocalMap;
   opencl_task->setArg(6, kernelI, sizeof(int), &isFullExtract);
   ocl_float3 position;
   position.x = icpFullPose.position.x;
   position.y = icpFullPose.position.y;
   double yy, pp, rr;
   icpFullPose.getYPR(yy, pp, rr);
   position.z = yy;
   opencl_task->setArg(7, kernelI, sizeof(cl_float3), &position);
   
   int globalSize = getGlobalWorkSize(highestBlockNum);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::extractPoints(int numBlocksToExtract, bool extractNorms) {
   int kernelI = EXTRACT_POINTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPointCloud);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clColours);
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clNormals);
   opencl_task->setArg(7, kernelI, sizeof(ocl_int3), &mapCentre);
   int exNorms = extractNorms;
   opencl_task->setArg(8, kernelI, sizeof(int), &exNorms);

   int numBlocksPerGroup = LocalSize / (NumCellsWidth * NumCellsWidth);
   int numGroups = numBlocksToExtract / numBlocksPerGroup;
   if (numBlocksToExtract % numBlocksPerGroup != 0) {
      numGroups++;
   }
   int globalSize = numGroups * LocalSize;
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

}

void PositionTrackFull3D::transformPoints(int numPoints, bool transformNorms,
      tf::Transform trans) {
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
   int kernelI = TRANSFORM_POINTS;

   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPointCloud);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormals);
   int transNorms = transformNorms;
   opencl_task->setArg(3, kernelI, sizeof(int), &transNorms);
   opencl_task->setArg(4, kernelI, sizeof(int), &numPoints);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(6, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[2]);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::addPointsToLocalMap(int numPoints) {
   size_t pointsSize = sizeof(float) * numPoints * 3;
   size_t coloursSize = sizeof(unsigned char) * numPoints * 3;
   float *ps = (float *) malloc(pointsSize);
   unsigned char *cols = (unsigned char *) malloc(coloursSize);
   float *norms = (float *) malloc(pointsSize);
   if (ps == NULL || cols == NULL || norms == NULL) {
      cout << "ERROR: malloc for extracting points failed" << endl;
   }
   readBuffer(clPointCloud, CL_FALSE, 0, pointsSize, ps, 0, 0, 0, "Reading points");
   readBuffer(clColours, CL_FALSE, 0, coloursSize, cols, 0, 0, 0, "Reading colours");
   readBuffer(clNormals, CL_TRUE, 0, pointsSize, norms, 0, 0, 0, "Reading normals");

   if (currentLocalMapInfo->cloud == NULL) {
      currentLocalMapInfo->cloud = new PointCloud();
      currentLocalMapInfo->normals = new PointCloud();
   }
   int curSize = currentLocalMapInfo->cloud->cloud.size();
   currentLocalMapInfo->cloud->cloud.resize(curSize + numPoints);
   currentLocalMapInfo->cloud->colours.resize(curSize + numPoints);
   currentLocalMapInfo->normals->cloud.resize(curSize + numPoints);

   int rIndex = 0;
   int added = 0;
   for (int i = 0; i < numPoints; i++, rIndex = rIndex + 3) {
      Point p;
      Colour c;
      if (!isnan(ps[rIndex]) && !isnan(norms[rIndex])) {
         p.x = ps[rIndex];
         p.y = ps[rIndex + 1];
         p.z = ps[rIndex + 2];
         c.r = cols[rIndex];
         c.g = cols[rIndex + 1];
         c.b = cols[rIndex + 2];
         currentLocalMapInfo->cloud->cloud[curSize + added] = p;
         currentLocalMapInfo->cloud->colours[curSize + added] = c;

         p.x = norms[rIndex];
         p.y = norms[rIndex + 1];
         p.z = norms[rIndex + 2];
         currentLocalMapInfo->normals->cloud[curSize + added] = p;
         added++;
      }
   }
   currentLocalMapInfo->cloud->cloud.resize(curSize + added);
   currentLocalMapInfo->cloud->colours.resize(curSize + added);
   currentLocalMapInfo->normals->cloud.resize(curSize + added);
   free(ps);
   free(cols);
   free(norms);
}

void PositionTrackFull3D::clearBlocks() {
   int numBlocksToDelete;
   readBuffer(clLocalMapCommon, CL_TRUE, numBlocksToDeleteOffset,
         sizeof(int), &numBlocksToDelete, 0, 0, 0, "Reading num blocks to delete");
   cout << "Clearing part map. Deleting: " << numBlocksToDelete << " blocks" << endl;

   int kernelI = CLEAR_BLOCKS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(int), &numBlocksToDelete);

   int globalSize = getGlobalWorkSize(numBlocksToDelete);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::markAllForExtraction() {
   int kernelI = MARK_ALL_FOR_EXTRACTION;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   int globalSize = getGlobalWorkSize(highestBlockNum);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::outputAllPoints(int numPoints, vector<uint8_t>& allPoints) {
   size_t pointsSize = sizeof(float) * numPoints * 3;
   size_t coloursSize = sizeof(unsigned char) * numPoints * 3;
   float *ps = (float *) malloc(pointsSize);
   unsigned char *cols = (unsigned char *) malloc(coloursSize);
   if (ps == NULL || cols == NULL) {
      cout << "ERROR: malloc for extracting all points failed" << endl;
   }
   readBuffer(clPointCloud, CL_FALSE, 0, pointsSize, ps, 0, 0, 0, "Reading all points");
   readBuffer(clColours, CL_TRUE, 0, coloursSize, cols, 0, 0, 0, "Reading all colours");

   allPoints.resize(numPoints * 32);
   int rIndex = 0;
   int added = 0;
   int curSize = 0;
   for (int i = 0; i < numPoints; i++, rIndex = rIndex + 3, curSize += 32) {
      if (!isnan(ps[rIndex])) {
         float *arr = (float *) &(allPoints[curSize]);
         arr[0] = ps[rIndex];
         arr[1] = ps[rIndex + 1];
         arr[2] = ps[rIndex + 2];
         
         arr[3] = 0;
         arr[4] = 0;
         arr[5] = 0;

         allPoints[curSize + 16] = cols[rIndex + 2];
         allPoints[curSize + 17] = cols[rIndex + 1];
         allPoints[curSize + 18] = cols[rIndex];

         added++;
      }
   }
   allPoints.resize(added * 32);
   cout << "Number of points published: " << added << endl;
   cout << "centre: " << mapCentre.x << " " << mapCentre.y << " " << mapCentre.z << endl;
   free(ps);
   free(cols);

   positionTrack3DNode->publishAllPoints();
}

void PositionTrackFull3D::setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty) {

   this->fx = fx;
   this->fy = fy;
   this->cx = cx;
   this->cy = cy;
   this->tx = tx;
   this->ty = ty;
   //cout << fx << " " << fy << " " << cx << " " << cy << " " << tx << " " << ty << endl;
}

void PositionTrackFull3D::newLocalMap(LocalMapInfoPtr localM) {
   newLocalMapInfo = localM;
   newLocalMapICPPose = icpFullPose;
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


