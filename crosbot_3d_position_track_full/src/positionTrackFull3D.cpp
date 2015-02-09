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
   "clearBlocks",
   "calculateNormals",
   "fastICP",
   "bilateralFilter",
   "combineICPResults",
   "scaleICP",
   "combineScaleICPResults"
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
#define CALCULATE_NORMALS 9
#define FAST_ICP 10
#define BILATERAL_FILTER 11
#define COMBINE_ICP_RESULTS 12
#define SCALE_ICP 13
#define COMBINE_SCALE_ICP_RESULTS 14

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
   paramNH.param<double>("TruncNeg", TruncNeg, 0.7);
   paramNH.param<double>("TruncPos", TruncPos, 0.7);
   paramNH.param<bool>("UseOccupancyForSurface", UseOccupancyForSurface, true);
   paramNH.param<double>("MaxDistance", MaxDistance, 8.0);
   //If have cell size of 0.0125, use SliceMult = 1, MaxPointsFrac = 2,
   //NumBlocksAlloctacated = 5000.
   //Otherwise can use NumBlocksAllocated = 10000, SliceMult = 2, MaxPointsFrac = 1
   paramNH.param<int>("SliceMult", SliceMult, 2);
   paramNH.param<int>("MaxPointsFrac", MaxPointsFrac, 1);
   paramNH.param<bool>("ReExtractBlocks", ReExtractBlocks, false);

   paramNH.param<int>("ImageWidth", imageWidth, -1);
   paramNH.param<int>("ImageHeight", imageHeight, -1);

   NumBlocksWidth = (LocalMapWidth + 0.00001) / BlockSize;
   NumBlocksHeight = (LocalMapHeight + 0.00001) / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = (BlockSize + 0.00001) / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;
   
   int maxPointsPerBlock = SliceMult * NumCellsWidth * NumCellsWidth;
   MaxPoints = maxPointsPerBlock * NumBlocksAllocated / MaxPointsFrac; //numBlocks

}

void PositionTrackFull3D::start() {
   if (imageWidth > 0 && imageHeight > 0) {
      setupGPU();
   }
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

   if (imageWidth == -1 || imageHeight == -1) {
      imageWidth = depthImage->width;
      imageHeight = depthImage->height;
      setupGPU();
   }
   //Update configs with camera params
   opencl_manager->deviceRelease(clPositionTrackConfig);
   positionTrackConfig.fx = fx;
   positionTrackConfig.fy = fy;
   positionTrackConfig.cx = cx;
   positionTrackConfig.cy = cy;
   positionTrackConfig.tx = tx;
   positionTrackConfig.ty = ty;
   clPositionTrackConfig = opencl_manager->deviceAlloc(sizeof(oclPositionTrackConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &positionTrackConfig);
   
   oldICP = icpPose.toTF();
   icpFullPose = icpPose;
}

void PositionTrackFull3D::setupGPU() {
   cout << "Position track full 3D: starting compile" << endl;

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
   positionTrackConfig.UseOccupancyForSurface = UseOccupancyForSurface;
   positionTrackConfig.MaxDistance = MaxDistance;
   positionTrackConfig.MaxPoints = MaxPoints;
   positionTrackConfig.ReExtractBlocks = ReExtractBlocks;
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

   /*cout << "Depth image: " << depthImage->encoding << " " << depthImage->step << " " <<
      depthImage->width << " " << depthImage->height << " " << depthImage->header.frame_id << endl;
   cout << "RGB image: " << rgbImage->encoding << " " << rgbImage->step << " " <<
      rgbImage->width << " " << rgbImage->height << " " << rgbImage->header.frame_id << endl;*/

   cout << "Position track full 3D: finished compile" << endl;

}

Pose PositionTrackFull3D::processFrame(const sensor_msgs::ImageConstPtr& depthImage,
         const sensor_msgs::ImageConstPtr& rgbImage, Pose sensorPose, Pose icpPose, 
         float *floorHeight, bool outputMapPoints, vector<uint8_t>& allPoints) {

   bool outputLocalMap = false;
   if (UseLocalMaps && currentLocalMapInfo != NULL && newLocalMapInfo != NULL &&
         currentLocalMapInfo->index != newLocalMapInfo->index) {
      cout << "Outputting the local map this iteration!!" << endl;
      outputLocalMap = true;
   }     

   convertFrame(depthImage, rgbImage);

   /*tf::Transform temp = icpFullPose.toTF() * oldICP.inverse();
   oldICP = icpPose.toTF();
   tf::Transform newFullPose = temp * oldICP;
   icpFullPose = newFullPose;*/
   tf::Transform newFullPose = icpFullPose.toTF();

   double y,p,r;
   icpPose.getYPR(y,p,r);
   cout << "icpPose: " << icpPose.position.x << " " << icpPose.position.y << " " <<
      icpPose.position.z << " : " << y << " " << p << " " << r << endl;
   icpFullPose.getYPR(y,p,r);
   cout << "fullIcpPose: " << icpFullPose.position.x << " " << icpFullPose.position.y << " " <<
      icpFullPose.position.z << " " << y << " " << p << " " << r << endl;

   //preprocessing of points (only used for icp) - normals, filtering and different res's
   //bilateralFilter();
   calculateNormals();

   //icp itself
   ros::WallTime t1 = ros::WallTime::now();
   alignICP(sensorPose.toTF(), newFullPose);
   ros::WallTime t2 = ros::WallTime::now();
   ros::WallDuration totalTime = t2 - t1;
   cout << "Time of align icp: " << totalTime.toSec() * 1000.0f << endl;


   t1 = ros::WallTime::now();
   //add frame
   tf::Transform offset = newFullPose * sensorPose.toTF();
   checkBlocksExist(numDepthPoints, offset);
   readBuffer(clLocalMapCommon, CL_TRUE, numActiveBlocksOffset, 
         sizeof(int), &numActiveBlocks, 0, 0, 0, "Reading num active blocks");
   //cout << "Number of blocks are: " << numActiveBlocks << endl;
   if (numActiveBlocks > MaxNumActiveBlocks) {
      numActiveBlocks = MaxNumActiveBlocks;
   }

   t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   cout << "Time of half adding points: " << totalTime.toSec() * 1000.0f << endl;
   t1 = ros::WallTime::now();

   //cout << "numActiveBlocks are: " << numActiveBlocks << endl;
   addRequiredBlocks();

   clFinish(opencl_manager->getCommandQueue());
   t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   cout << "Time of adding blocks: " << totalTime.toSec() * 1000.0f << endl;
   t1 = ros::WallTime::now();

   addFrame(offset);

   
   readBuffer(clLocalMapCommon, CL_TRUE, highestBlockNumOffset,
         sizeof(int), &highestBlockNum, 0, 0, 0, "Reading highest block num");
   t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   cout << "Time of adding points: " << totalTime.toSec() * 1000.0f << endl;
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
         cout << "Saving points to the local map. Extracting " << numBlocksToExtract <<
           " blocks" << endl;
         extractPoints(numBlocksToExtract, true);

         int numPoints;
         readBuffer(clLocalMapCommon, CL_TRUE, numPointsOffset,
            sizeof(int), &numPoints, 0, 0, 0, "reading num of points extracted");
         cout << numPoints << " were extracted. max: " << MaxPoints << endl;
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

      t1 = ros::WallTime::now();
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
   
      t2 = ros::WallTime::now();
      totalTime = t2 - t1;
      cout << "Time of extracting points: " << totalTime.toSec() * 1000.0f << endl;
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
   clFiltDepthFrame = opencl_manager->deviceAlloc(sizeDepthPoints, CL_MEM_READ_WRITE, NULL);

   colourFrame = new oclColourPoints;
   colourFrame->r = new unsigned char[numDepthPoints * 3];
   colourFrame->g = &(colourFrame->r[numDepthPoints]);
   colourFrame->b = &(colourFrame->r[numDepthPoints * 2]);
   sizeColourPoints = sizeof(unsigned char) * numDepthPoints * 3;
   clColourFrame = opencl_manager->deviceAlloc(sizeColourPoints, CL_MEM_READ_WRITE, NULL);

   sizeDepthFrameXYZ = sizeof(ocl_float) * numDepthPoints * 3;
   clDepthFrameXYZ = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);
   clNormalsFrame = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);

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

   size_t commonSize = sizeof(ocl_int) * (6 + MaxNumActiveBlocks + 3*NumBlocksAllocated) +
      sizeof(ocl_float) * (NUM_RESULTS + 6);
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
   icpResultsOffset = (unsigned char *)&(com.icpResults) - (unsigned char *)&(com);

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
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(7, kernelI, sizeof(int), &numActiveBlocks);
   opencl_task->setArg(8, kernelI, sizeof(ocl_int3), &mapCentre);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(11, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(12, kernelI, sizeof(ocl_float3), &clBasis[2]);

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
   if (currentLocalMapInfo == NULL) {
      //Haven't received the first local map yet, so ignore points
      return;
   }
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
   if (numBlocksToDelete == 0) {
      return;
   }

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

void PositionTrackFull3D::calculateNormals() {

   int kernelI = CALCULATE_NORMALS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clDepthFrame);
   //opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clFiltDepthFrame);
   opencl_task->setArg(4, kernelI, sizeof(int), &numDepthPoints);
   int globalSize = getGlobalWorkSize(numDepthPoints);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::alignICP(tf::Transform sensorPose, tf::Transform newPose) {

   int globalSize = getGlobalWorkSize(numDepthPoints);
   int numGroups = globalSize / LocalSize;

   int kernelI = FAST_ICP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clPointCloud); //tempStore
   opencl_task->setArg(7, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(9, kernelI, sizeof(ocl_int3), &mapCentre);

   ocl_float zero[NUM_RESULTS];
   memset(zero, 0, sizeof(ocl_float) * NUM_RESULTS);
   ocl_float rawResults[NUM_RESULTS];

   float A[DOF][DOF];
   float b[DOF];
   float x[DOF];

   tf::Transform start = newPose * sensorPose;
   tf::Transform curTrans = start;
   
   scaleICP(numGroups, curTrans);
   readBuffer(clLocalMapCommon, CL_TRUE, icpResultsOffset + sizeof(float) * NUM_RESULTS, sizeof(ocl_float) * 6, 
            rawResults, 0, 0, 0, "Reading the icp scale results");
   cout << "Scale is: " << rawResults[0] << " " << rawResults[1] << " " << rawResults[2] << " " <<
      rawResults[3] << " " << rawResults[4] << " " << rawResults[5] << endl;

   Pose startPose = curTrans;
   double y,p,r;
   startPose.getYPR(y,p,r);

   cout << "Start pose is: " << startPose.position.x << " " << startPose.position.y << " " <<
         startPose.position.z << " " << y << " " << p << " " << r << endl;

   int i;
   for (i = 0; i < 5; i++) {

      //writeBuffer(clLocalMapCommon, CL_FALSE, icpResultsOffset, sizeof(ocl_float) * NUM_RESULTS, 
      //      zero, 0, 0, 0, "Zeroing the icp results array");
      /*Pose curPose = curTrans;
      cout << "Now pose is: " << curPose.position.x << " " << curPose.position.y << " " <<
         curPose.position.z << endl;*/

      tf::Matrix3x3 basis = curTrans.getBasis();
      tf::Vector3 origin = curTrans.getOrigin();

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
   
      opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clOrigin);
      opencl_task->setArg(11, kernelI, sizeof(ocl_float3), &clBasis[0]);
      opencl_task->setArg(12, kernelI, sizeof(ocl_float3), &clBasis[1]);
      opencl_task->setArg(13, kernelI, sizeof(ocl_float3), &clBasis[2]);

      opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
      combineICPResults(numGroups);
   
      readBuffer(clLocalMapCommon, CL_TRUE, icpResultsOffset, sizeof(ocl_float) * NUM_RESULTS, 
            rawResults, 0, 0, 0, "Reading the icp results");

      A[0][0] = rawResults[0];
      A[1][0] = rawResults[1];
      A[1][1] = rawResults[2];
      A[2][0] = rawResults[3];
      A[2][1] = rawResults[4];
      A[2][2] = rawResults[5];
      A[3][0] = rawResults[6];
      A[3][1] = rawResults[7];
      A[3][2] = rawResults[8];
      A[3][3] = rawResults[9];
      A[4][0] = rawResults[10];
      A[4][1] = rawResults[11];
      A[4][2] = rawResults[12];
      A[4][3] = rawResults[13];
      A[4][4] = rawResults[14];
      A[5][0] = rawResults[15];
      A[5][1] = rawResults[16];
      A[5][2] = rawResults[17];
      A[5][3] = rawResults[18];
      A[5][4] = rawResults[19];
      A[5][5] = rawResults[20];
      b[0] = rawResults[21];
      b[1] = rawResults[22];
      b[2] = rawResults[23];
      b[3] = rawResults[24];
      b[4] = rawResults[25];
      b[5] = rawResults[26];

      cout << "Raw results: ";
      for (int j = 0; j < 27; j++) {
         cout << rawResults[j] << " ";
      }
      cout << endl;

      //cout << "Num points used: " << rawResults[28] << " average distance:" << rawResults[27]/rawResults[28] << endl; 

      solveCholesky(A, b, x);

      /*float check[6];
      check[0] = rawResults[0] * x[0] + rawResults[1] * x[1] + rawResults[3] * x[2] +
         rawResults[6] * x[3] + rawResults[10] * x[4] + rawResults[15] * x[5];
      check[1] = rawResults[1] * x[0] + rawResults[2] * x[1] + rawResults[4] * x[2] +
         rawResults[7] * x[3] + rawResults[11] * x[4] + rawResults[16] * x[5];
      check[2] = rawResults[3] * x[0] + rawResults[4] * x[1] + rawResults[5] * x[2] +
         rawResults[8] * x[3] + rawResults[12] * x[4] + rawResults[17] * x[5];
      check[3] = rawResults[6] * x[0] + rawResults[7] * x[1] + rawResults[8] * x[2] +
         rawResults[9] * x[3] + rawResults[13] * x[4] + rawResults[18] * x[5];
      check[4] = rawResults[10] * x[0] + rawResults[11] * x[1] + rawResults[12] * x[2] +
         rawResults[13] * x[3] + rawResults[14] * x[4] + rawResults[19] * x[5];
      check[5] = rawResults[15] * x[0] + rawResults[16] * x[1] + rawResults[17] * x[2] +
         rawResults[18] * x[3] + rawResults[19] * x[4] + rawResults[20] * x[5];
      cout << "The results are: " << endl;
      for (int j = 0; j < DOF; j++) {
         cout << check[j] << " " << b[j] << endl;
      }*/

      /*cout << "Results: ";
      for (int j = 0; j < DOF; j++) {
         cout << x[j] << " ";
      }
      cout << endl;*/

      bool valid = true;
      for (int j = 0; j < DOF; j++) {
         if (isnan(x[j])) {
            valid = false;
         }
         //x[j] /= (float)(i + 1);
      }
      if (!valid) {
         cout << "Alignment failed" << endl;
         break;
      }

      tf::Vector3 incVec(x[3], x[4], x[5]);
      //tf::Matrix3x3 incMat(1, x[2], -x[1], -x[2], 1, x[0], x[1], -x[0], 1);
      //tf::Matrix3x3 incMat(1, -x[2], x[1], x[2], 1, -x[0], -x[1], x[0], 1);
      tf::Matrix3x3 incMat;
      incMat.setEulerYPR(x[2], x[1], x[0]);
      tf::Transform inc(incMat, incVec);
      Pose incPose = inc;
      //double r,p,y;
      incPose.getYPR(y,p,r);
      cout << incPose.position.x << " " << incPose.position.y << " " << incPose.position.z << " : " 
         << y << " " << p << " " << r << " : " << rawResults[28] << " " << 
         rawResults[27]/rawResults[28] << endl;

      //cout << y << " " << p << " " << r << "  " << x[0] << " " << x[1] << " " << x[2] << endl;
      //y = 0, p = 0, r = 0;
      //y = 0;
      //incPose.setYPR(y,p,r);
      //incPose.position.x = 0;
      //incPose.position.y = 0;
      //incPose.position.z = 0;
      //inc = incPose.toTF();
      curTrans = inc * curTrans;
   }
   Pose endPose = curTrans;
   endPose.getYPR(y,p,r);
   cout << "End pose is: " << endPose.position.x << " " << endPose.position.y << " " <<
       endPose.position.z << " " << y << " " << p << " " << r << endl;
   icpFullPose = curTrans * sensorPose.inverse();

}

void PositionTrackFull3D::solveCholesky(float A[DOF][DOF], float b[DOF], float x[DOF]) {

   int col, row, i;
   float buf[DOF];
   float sum;

   //Calculate LDL^T factorisation of A
   for (row = 0; row < DOF; row++) {
      //Calculate L
      for (col = 0; col < row; col++) {
         sum = 0;
         for (i = 0; i < col; i++) {
            sum += A[row][i] * A[col][i] * A[i][i];
         }
         A[row][col] = (1 / A[col][col]) * (A[row][col] - sum);
      }

      //Calculate D
      for (i = row - 1; i >= 0; i--) {
         A[row][row] -= (A[row][i] * A[row][i] * A[i][i]);
      }
   }

   //Calculate Lz = b
   for (row = 0; row < DOF; row++) {
      buf[row] = b[row];
      for (i = 0; i < row; i++) {
         buf[row] -= A[row][i] * buf[i];
      }
   }

   //Calculate Dy = z
   for (i = 0; i < DOF; i++) {
      buf[i] = buf[i] / A[i][i];
   }

   //Calculate L^T x = y
   for (row = DOF - 1; row >= 0; row--) {
      x[row] = buf[row];
      for (i = row + 1; i < DOF; i++) {
         x[row] -= A[i][row] * x[i];
      }
   }
}

void PositionTrackFull3D::bilateralFilter() {
   int kernelI = BILATERAL_FILTER;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clDepthFrame);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clFiltDepthFrame);
   opencl_task->setArg(3, kernelI, sizeof(int), &numDepthPoints);
   int globalSize = getGlobalWorkSize(numDepthPoints);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::combineICPResults(int numGroups) {
   int kernelI = COMBINE_ICP_RESULTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPointCloud); //tempStore
   opencl_task->setArg(2, kernelI, sizeof(int), &numGroups);
   opencl_task->queueKernel(kernelI, 1, LocalSize, LocalSize, 0, NULL, NULL, false);

}

void PositionTrackFull3D::scaleICP(int numGroups, tf::Transform trans) {
   tf::Matrix3x3 basis = trans.getBasis();

   ocl_float3 clBasis[3];
   for (int j = 0; j < 3; j++) {
      clBasis[j].x = basis[j][0];
      clBasis[j].y = basis[j][1];
      clBasis[j].z = basis[j][2];
   }
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = SCALE_ICP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPointCloud); //tempStore
   opencl_task->setArg(5, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(6, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[2]);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   kernelI = COMBINE_SCALE_ICP_RESULTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPointCloud); //tempStore
   opencl_task->setArg(2, kernelI, sizeof(int), &numGroups);
   opencl_task->queueKernel(kernelI, 1, LocalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::setCameraParams(double fx, double fy, double cx, double cy, double tx, double ty) {

   this->fx = fx;
   this->fy = fy;
   this->cx = cx;
   this->cy = cy;
   this->tx = tx;
   this->ty = ty;
   /*this->fx = 517.3;
   this->fy = 516.5;
   this->cx = 318.6;
   this->cy = 255.3;
   this->tx = tx;
   this->ty = ty;*/
   //cout << fx << " " << fy << " " << cx << " " << cy << " " << tx << " " << ty << endl;
}

void PositionTrackFull3D::newLocalMap(LocalMapInfoPtr localM) {
   cout << "Received info about new local map" << endl;
   if (currentLocalMapInfo == NULL) {
      currentLocalMapInfo = localM;
      currentLocalMapICPPose = icpFullPose;
   } else {
      newLocalMapInfo = localM;
      newLocalMapICPPose = icpFullPose;
   }
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


