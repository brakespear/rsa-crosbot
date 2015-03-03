/*
 * positionTrackFull3D.cpp
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#include <crosbot_3d_position_track_full/positionTrackFull3D.hpp>

#include <sstream>


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
   "combineScaleICPResults",
   "predictSurface",
   "rayTraceICP",
   "downsampleDepth",
   "zOnlyICP",
   "scaleRayTraceICP",
   "outputDebuggingImage"
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
#define PREDICT_SURFACE 15
#define RAY_TRACE_ICP 16
#define DOWNSAMPLE_DEPTH 17
#define Z_ONLY_ICP 18
#define SCALE_RAY_TRACE_ICP 19

#define OUTPUT_DEBUGGING_IMAGE 20

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

   //f = fopen("frames.txt", "w");
   //fprintf(f, "160 120\n");

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
   paramNH.param<double>("DepthDistThreshold", DepthDistThreshold, 0.05);
   paramNH.param<int>("FilterWindowSize", FilterWindowSize, 4);
   paramNH.param<double>("FilterScalePixel", FilterScalePixel, 0.5);
   paramNH.param<int>("SkipNumCheckBlocks", SkipNumCheckBlocks, 4);
   paramNH.param<int>("MaxRayTrace", MaxRayTrace, 20);
   paramNH.param<double>("MoveThresh", MoveThresh, 0.001);
   paramNH.param<bool>("UseOdometry", UseOdometry, true);
   paramNH.param<bool>("UseICP", UseICP, true);
   paramNH.param<bool>("UseICPZOnly", UseICPZOnly, false);
   paramNH.param<int>("MinICPCount", MinICPCount, 100);
   paramNH.param<double>("MaxMoveXYZ", MaxMoveXYZ, 0.3);
   paramNH.param<double>("MaxMoveRPY", MaxMoveRPY, 0.3);
   paramNH.param<int>("FailCount", FailCount, 50);
   paramNH.param<double>("MinScale", MinScale, 8000);
   paramNH.param<int>("MaxICPIterations", MaxICPIterations, 10);
   paramNH.param<int>("MinICPIterations", MinICPIterations, 5);
   paramNH.param<int>("NumICPIterations4", NumICPIterations4, 4);
   paramNH.param<int>("NumICPIterations2", NumICPIterations2, 7);
   paramNH.param<double>("ScaleRegularisation", ScaleRegularisation, 1);


   paramNH.param<double>("DistThresh", DistThresh, 0.0005);
   paramNH.param<double>("DotThresh", DotThresh, 0.7);
   paramNH.param<double>("DistThresh2", DistThresh2, 0.05);
   paramNH.param<double>("DotThresh2", DotThresh2, 0.6);
   paramNH.param<double>("DistThresh4", DistThresh4, 0.2);
   paramNH.param<double>("DotThresh4", DotThresh4, 0.5);

   paramNH.param<int>("ImageWidth", imageWidth, -1);
   paramNH.param<int>("ImageHeight", imageHeight, -1);

   NumBlocksWidth = (LocalMapWidth + 0.00001) / BlockSize;
   NumBlocksHeight = (LocalMapHeight + 0.00001) / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = (BlockSize + 0.00001) / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;
   
   int maxPointsPerBlock = SliceMult * NumCellsWidth * NumCellsWidth;
   MaxPoints = maxPointsPerBlock * NumBlocksAllocated / MaxPointsFrac; //numBlocks
   
   numFails = FailCount + 1;

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
   //fclose(f);
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
   positionTrackConfig.DepthDistThreshold = DepthDistThreshold;
   positionTrackConfig.FilterWindowSize = FilterWindowSize;
   positionTrackConfig.FilterScalePixel = FilterScalePixel;
   positionTrackConfig.SkipNumCheckBlocks = SkipNumCheckBlocks;
   positionTrackConfig.MaxRayTrace = MaxRayTrace;
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

   if (UseOdometry) {
      tf::Transform temp = icpFullPose.toTF() * oldICP.inverse();
      oldICP = icpPose.toTF();
      tf::Transform newFullPose = temp * oldICP;
      icpFullPose = newFullPose;
   }
   //tf::Transform newFullPose = icpFullPose.toTF();

   double y,p,r;
   icpPose.getYPR(y,p,r);
   cout << "icpPose: " << icpPose.position.x << " " << icpPose.position.y << " " <<
      icpPose.position.z << " : " << y << " " << p << " " << r << endl;
   icpFullPose.getYPR(y,p,r);
   cout << "fullIcpPose: " << icpFullPose.position.x << " " << icpFullPose.position.y << " " <<
      icpFullPose.position.z << " " << y << " " << p << " " << r << endl;

   //preprocessing of points (only used for icp) - normals, filtering and different res's
   //bilateralFilter();
   downsampleDepth();
   calculateNormals();

//Debugging only!!
   //printFrames(depthImage->header.stamp, sensorPose.toTF());

   //icp itself
   clFlush(opencl_manager->getCommandQueue());
   ros::WallTime t1 = ros::WallTime::now();
   if (UseICP) {
      if (UseICPZOnly) {
         alignZOnlyICP(sensorPose.toTF(), icpFullPose.toTF());
      } else {
         //alignICP(sensorPose.toTF(), icpFullPose.toTF());
         alignRayTraceICP(sensorPose.toTF(), icpFullPose.toTF());
      }
   }

   poseHistory.push_back(icpFullPose);
   timeHistory.push_back(depthImage->header.stamp);

   ros::WallTime t2 = ros::WallTime::now();
   ros::WallDuration totalTime = t2 - t1;
   cout << "Total time of align icp: " << totalTime.toSec() * 1000.0f << endl;

   if (numFails == 0 || numFails > FailCount) {
      t1 = ros::WallTime::now();
      //add frame
      tf::Transform offset = icpFullPose.toTF() * sensorPose.toTF();
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
   }

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

         if (numPoints > 0) {
            tf::Transform trans;
            if (currentLocalMapInfo == NULL) {
               Pose zero;
               trans = zero.toTF();
            } else {
               trans = currentLocalMapInfo->icpPose.toTF().inverse();
            }

            //tf::Transform trans = currentLocalMapICPPose.toTF().inverse();
            transformPoints(numPoints, true, trans);
            addPointsToLocalMap(numPoints);
         }
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
      //cout << numBlocksToExtract << " blocks to extract" << endl;

      extractPoints(numBlocksToExtract, false);

         
      int numPoints;
      readBuffer(clLocalMapCommon, CL_TRUE, numPointsOffset,
         sizeof(int), &numPoints, 0, 0, 0, "reading num of points extracted");
      cout << "Number of points that were extracted: " << numPoints << " " << MaxPoints << endl;
      if (numPoints > MaxPoints) {
         numPoints = MaxPoints;
      }

      if (numPoints > 0) {
         //Should the points be transformed to be relative to the robot?
         //tf::Transform trans = icpFullPose.toTF().inverse();
         //transformPoints(numPoints, false, trans);

         outputAllPoints(numPoints, allPoints);               
      }
   
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

      //currentLocalMapInfo->pose = currentLocalMapICPPose; //Test this

      currentLocalMapInfo->poseHistory.resize(poseHistory.size());
      //tf::Transform correction = currentLocalMapICPPose.toTF().inverse();
      tf::Transform correction = currentLocalMapInfo->icpPose.toTF().inverse();

      for (int i = 0; i < poseHistory.size(); i++) {
         currentLocalMapInfo->poseHistory[i] = correction * poseHistory[i]. toTF();
      }
      currentLocalMapInfo->timeHistory = timeHistory;
      timeHistory.clear();
      poseHistory.clear();

      //Just in case no points were extracted, make sure here is a (empty) cloud
      if (currentLocalMapInfo->cloud == NULL) {
         currentLocalMapInfo->cloud = new PointCloud();
         currentLocalMapInfo->normals = new PointCloud();
      }

      positionTrack3DNode->publishLocalMap(currentLocalMapInfo);
      currentLocalMapInfo = newLocalMapInfo;
      //currentLocalMapICPPose = newLocalMapICPPose;
   }
   mapCentre = newMapCentre;


   //Extract pose
   //ostringstream tt;
   //tt << depthImage->header.stamp;
   //const char *st = tt.str().c_str();
   //tf::Transform tTrans = icpFullPose.toTF();
   //tf::Vector3 tVec = tTrans.getOrigin();
   //tf::Quaternion tQuat = tTrans.getRotation();
   //fprintf(f, "%s %lf %lf %lf %lf %lf %lf %lf\n", st, tVec[0], tVec[1], tVec[2], tQuat.x(),
   //      tQuat.y(), tQuat.z(), tQuat.w());

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

   size_t sizeOutPoints = sizeof(ocl_float) * MaxPoints * 3;
   //clPointCloud and clNormals are reused during icp
   if (sizeOutPoints < sizeDepthFrameXYZ) {
      sizeOutPoints = sizeDepthFrameXYZ;
   }
   clPointCloud = opencl_manager->deviceAlloc(sizeOutPoints, 
         CL_MEM_READ_WRITE, NULL);
   clNormals = opencl_manager->deviceAlloc(sizeOutPoints,
         CL_MEM_READ_WRITE, NULL);

   //clColours is reused during ICP
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int numGroups = globalSize / LocalSize;
   size_t sizeTempStore = numGroups * NUM_RESULTS;
   size_t sizeOutColours = sizeof(unsigned char) * MaxPoints * 3;
   if (sizeOutColours < sizeTempStore) {
      sizeOutColours = sizeTempStore;
   }
   clColours = opencl_manager->deviceAlloc(sizeOutColours,
         CL_MEM_READ_WRITE, NULL);

   /*int widthDown = imageWidth / 2;
   int heightDown = imageHeight / 2;
   size_t sizeDownPoints = sizeof(ocl_float) * widthDown * heightDown * 3;
   clDepthFrameXYZ2 = opencl_manager->deviceAlloc(sizeDownPoints, CL_MEM_READ_WRITE, NULL);
   clNormalsFrame2 = opencl_manager->deviceAlloc(sizeDownPoints, CL_MEM_READ_WRITE, NULL);
   widthDown /= 2;
   heightDown /= 2;
   sizeDownPoints = sizeof(ocl_float) * widthDown * heightDown * 3;
   clDepthFrameXYZ4 = opencl_manager->deviceAlloc(sizeDownPoints, CL_MEM_READ_WRITE, NULL);
   clNormalsFrame4 = opencl_manager->deviceAlloc(sizeDownPoints, CL_MEM_READ_WRITE, NULL);*/
   clDepthFrameXYZ2 = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);
   clNormalsFrame2 = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);
   clDepthFrameXYZ4 = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);
   clNormalsFrame4 = opencl_manager->deviceAlloc(sizeDepthFrameXYZ, CL_MEM_READ_WRITE, NULL);

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
   icpScaleResultsOffset = (unsigned char *)&(com.icpScale) - (unsigned char *)&(com);


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
      //return;
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
   int numPoints = (imageWidth / SkipNumCheckBlocks) * (imageHeight / SkipNumCheckBlocks);
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
   int factor = 1;
   opencl_task->setArg(5, kernelI, sizeof(int), &factor);
   int globalSize = getGlobalWorkSize(numDepthPoints);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

   int width = imageWidth / 2;
   int height = imageHeight / 2;
   int numDownPoints = width * height;
   factor = 2;
   globalSize = getGlobalWorkSize(numDownPoints);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clDepthFrameXYZ2);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormalsFrame2);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clPointCloud);
   opencl_task->setArg(4, kernelI, sizeof(int), &numDownPoints);
   opencl_task->setArg(5, kernelI, sizeof(int), &factor);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   width /= 2;
   height /= 2;
   numDownPoints = width * height;
   factor = 4;
   globalSize = getGlobalWorkSize(numDownPoints);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clDepthFrameXYZ4);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormalsFrame4);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormals);
   opencl_task->setArg(4, kernelI, sizeof(int), &numDownPoints);
   opencl_task->setArg(5, kernelI, sizeof(int), &factor);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

}

void PositionTrackFull3D::alignICP(tf::Transform sensorPose, tf::Transform newPose) {
   ros::WallTime t1 = ros::WallTime::now();
   tf::Transform start = newPose * sensorPose;
   int nGroups = getGlobalWorkSize(numDepthPoints) / LocalSize;
   float scale[6];
   scaleICP(nGroups, start, scale);
   
   ros::WallTime t2 = ros::WallTime::now();
   ros::WallDuration totalTime = t2 - t1;
   cout << "Total time of scale: " << totalTime.toSec() * 1000.0f << endl;
   t1 = ros::WallTime::now();
   
   int kernelI = FAST_ICP;
   
   int numPoints = (imageWidth / 4) * (imageHeight / 4);
   int globalSize = getGlobalWorkSize(numPoints);
   int numGroups = globalSize / LocalSize;
  
   float distThresh = DistThresh4;
   float dotThresh = DotThresh4;
   
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrameXYZ4);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormalsFrame4);
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(7, kernelI, sizeof(int), &numPoints);
   opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(9, kernelI, sizeof(ocl_int3), &mapCentre);
   
   opencl_task->setArg(14, kernelI, sizeof(float), &dotThresh);
   opencl_task->setArg(15, kernelI, sizeof(float), &distThresh);
   
   ocl_float rawResults[NUM_RESULTS];

   float A[DOF][DOF];
   float b[DOF];
   float x[DOF];
   float reg[DOF][DOF];

   start = newPose;
   tf::Transform curTrans = start;
   Pose startPose = curTrans;
   double y,p,r;
   startPose.getYPR(y,p,r);
   cout << "Start pose is: " << startPose.position.x << " " << startPose.position.y << " " <<
         startPose.position.z << "  " << y << " " << p << " " << r << endl;

   float totalMovement[6];
   for (int i = 0; i < 6; i++) {
      totalMovement[i] = 0;
   }

   
   bool limit = false;
   if (scale[3] < MinScale || scale[4] < MinScale || scale[5] < MinScale) {
      limit = true;
   }

   int i;
   bool cont = true;
   bool failed = false;
   for (i = 0; i < 10 && cont; i++) {

      if (i == 4) {
         numPoints = (imageWidth / 2) * (imageHeight / 2);
         globalSize = getGlobalWorkSize(numPoints);
         numGroups = globalSize / LocalSize;
         distThresh = DistThresh2;
         dotThresh = DotThresh2;
         
         opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrameXYZ2);
         opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormalsFrame2);
         opencl_task->setArg(7, kernelI, sizeof(int), &numPoints);
         opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
         opencl_task->setArg(14, kernelI, sizeof(float), &dotThresh);
         opencl_task->setArg(15, kernelI, sizeof(float), &distThresh);
      } else if (i == 7) {
         globalSize = getGlobalWorkSize(numDepthPoints);
         numGroups = globalSize / LocalSize;
         distThresh = DistThresh;
         dotThresh = DotThresh;
         
         opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
         opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormalsFrame);
         opencl_task->setArg(7, kernelI, sizeof(int), &numDepthPoints);
         opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
         opencl_task->setArg(14, kernelI, sizeof(float), &dotThresh);
         opencl_task->setArg(15, kernelI, sizeof(float), &distThresh);
      }

      tf::Transform temp = curTrans * sensorPose;
      tf::Matrix3x3 basis = temp.getBasis();
      tf::Vector3 origin = temp.getOrigin();

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
      combineICPResults(numGroups, NUM_RESULTS);
   
      readBuffer(clLocalMapCommon, CL_TRUE, icpResultsOffset, sizeof(ocl_float) * NUM_RESULTS, 
            rawResults, 0, 0, 0, "Reading the icp results");

      /*A[0][0] = rawResults[0];
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

      solveCholesky(A, b, x);*/

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

      memset(reg, 0, sizeof(float) * 36);
      float tempScale[6];
      float max = 0;
      for (int j = 0; j < 6; j++) {
         if (scale[j] > max) {
            max = scale[j];
         }

      }
      for (int j = 0; j < 6; j++) {
         //tempScale[j] = ((scale[j]) / numDepthPoints) * rawResults[27];
         //tempScale[j] = 1 - (scale[j] / numDepthPoints);
         tempScale[j] = 1.0f / (scale[j] / numDepthPoints);
         tempScale[j] = pow(tempScale[j], 2);
         tempScale[j] *= 1.0f;
      }
      cout << "Temp scale: " << tempScale[0] << " " << tempScale[1] << " " << tempScale[2]
         << " " << tempScale[3] << " " << tempScale[4] << " " << tempScale[5] << endl;
      multVectorTrans(tempScale, reg);
      //reg[0][0] = reg[1][1] = reg[2][2] = reg[3][3] = reg[4][4] = reg[5][5] = 1;


      A[0][0] = rawResults[0] + reg[0][0];
      A[1][0] = rawResults[1] + reg[1][0];
      A[1][1] = rawResults[2] + reg[1][1];
      A[2][0] = rawResults[3] + reg[2][0];
      A[2][1] = rawResults[4] + reg[2][1];
      A[2][2] = rawResults[5] + reg[2][2];
      A[3][0] = rawResults[6] + reg[3][0];
      A[3][1] = rawResults[7] + reg[3][1];
      A[3][2] = rawResults[8] + reg[3][2];
      A[3][3] = rawResults[9] + reg[3][3];
      A[4][0] = rawResults[10] + reg[4][0];
      A[4][1] = rawResults[11] + reg[4][1];
      A[4][2] = rawResults[12] + reg[4][2];
      A[4][3] = rawResults[13] + reg[4][3];
      A[4][4] = rawResults[14] + reg[4][4];
      A[5][0] = rawResults[15] + reg[5][0];
      A[5][1] = rawResults[16] + reg[5][1];
      A[5][2] = rawResults[17] + reg[5][2];
      A[5][3] = rawResults[18] + reg[5][3];
      A[5][4] = rawResults[19] + reg[5][4];
      A[5][5] = rawResults[20] + reg[5][5];

      float offset[6];
      mult6x6Vector(reg, totalMovement, offset);
      offset[0] = offset[1] = offset[2] = offset[3] = offset[4] = offset[5] = 0;
      b[0] = rawResults[21] - offset[0];
      b[1] = rawResults[22] - offset[1];
      b[2] = rawResults[23] - offset[3];
      b[3] = rawResults[24] - offset[4];
      b[4] = rawResults[25] - offset[5];
      b[5] = rawResults[26] - offset[6];

      int count = rawResults[27];

      /*cout << "Raw results: ";
      for (int j = 0; j < NUM_RESULTS; j++) {
         cout << rawResults[j] << " ";
      }
      cout << endl;*/

      solveCholesky(A, b, x);
      cout << "Modified results: " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << " " << x[4]
         << " " << x[5] << endl;

      cout << "Total movement: " << totalMovement[0] << " " << totalMovement[1] << " " << totalMovement[2] << " "
          << totalMovement[3] << " " << totalMovement[4] << " " << totalMovement[5] << endl;

      /*cout << "Results: ";
      for (int j = 0; j < DOF; j++) {
         cout << x[j] << " ";
      }
      cout << endl;*/

      bool valid = true;
      cont = false;
      for (int j = 0; j < DOF; j++) {
         if (isnan(x[j])) {
            valid = false;
         }
         if (fabs(x[j]) > MoveThresh) {
            cont = true;
         }
         totalMovement[j] += x[j];
         //if (j >= 3) {
         //   x[j] = 0;
         //}
         if ((j < 3 && fabs(totalMovement[j]) > MaxMoveRPY) || (j >= 3 && fabs(totalMovement[j]) > MaxMoveXYZ)) {
            cout << "******Moved too far" << endl;
            valid = false;
            break;
         }
      }
      if (i < 5) {
         cont = true;
      }
      if (!valid) {
         cout << "********Alignment failed" << endl;
         failed = true;
         break;
      }
      if (count < MinICPCount) {
         cout << "***Alignment failed: " << count << endl;
         failed = true;
         break;
      }

      //x[5] = 0;
      //x[0] = 0;
      //x[1] = 0;

      tf::Vector3 incVec(x[3], x[4], x[5]);
      tf::Matrix3x3 incMat(1, x[2], -x[1], -x[2], 1, x[0], x[1], -x[0], 1);
      //tf::Matrix3x3 incMat;
      //incMat.setEulerYPR(x[2], x[1], x[0]);
      tf::Transform inc(incMat, incVec);
      
      Pose incPose = inc;
      incPose.getYPR(y,p,r);

      //cout << incPose.position.x << " " << incPose.position.y << " " << incPose.position.z << " : " 
      //   << y << " " << p << " " << r << " : " << rawResults[27] << " " << endl;
      
      //tf::Vector3 tt = curTrans.getOrigin();
      //curTrans = inc * curTrans;
      //curTrans.setOrigin(tt);

      //tf::Transform before = curTrans;
      //curTrans = inc * curTrans;
      //curTrans.setOrigin(before.getOrigin());

      //Pose pos = inc;
      //pos.getYPR(y,p,r);
      //p = 0;
      //r = 0;
      //pos.setYPR(y,p,r);
      //pos.position.z = 0;
      //inc = pos.toTF();


      curTrans = inc * curTrans;
   }
   Pose endPose = curTrans;
   endPose.getYPR(y,p,r);
   cout << "End pose is: " << endPose.position.x << " " << endPose.position.y << " " <<
       endPose.position.z << "  " << y << " " << p << " " << r << endl;
   if (!failed) {
      if (limit) {
         cout << "^^^^Uncertain of x,y,y, limiting movement" << endl;
         double yn, rn, pn, yo, ro, po;
         Pose newPose = curTrans;
         newPose.getYPR(yn,pn,rn);
         icpFullPose.getYPR(yo,po,ro);
         icpFullPose.position.z = newPose.position.z;
         icpFullPose.setYPR(yo,pn,rn);
      } else {
         icpFullPose = curTrans;
      }
      numFails = 0;
   } else {
      numFails++;
   }
   t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   cout << "Total time of icp aligning (" << i << "its): " << totalTime.toSec() * 1000.0f << endl;
}





void PositionTrackFull3D::alignRayTraceICP(tf::Transform sensorPose, tf::Transform newPose) {
   tf::Transform start = newPose * sensorPose;

   ros::WallTime t1 = ros::WallTime::now();

   predictSurface(start);
   outputDebuggingImage(start);
   int nGroups = getGlobalWorkSize(numDepthPoints) / LocalSize;
   float scale[6];
   scaleRayTraceICP(nGroups, scale);


   start = newPose;
   
   ros::WallTime t2 = ros::WallTime::now();
   ros::WallDuration totalTime = t2 - t1;
   cout << "Total time of predict surface: " << totalTime.toSec() * 1000.0f << endl;
   t1 = ros::WallTime::now();

   int kernelI = RAY_TRACE_ICP;

   //int numPoints = numDepthPoints;
   int numPoints = (imageWidth / 4) * (imageHeight / 4);
   int globalSize = getGlobalWorkSize(numPoints);
   int numGroups = globalSize / LocalSize;
  
   float distThresh = DistThresh4;
   float dotThresh = DotThresh4;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   //opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   //opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clDepthFrameXYZ4);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormalsFrame4);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPointCloud); //predPoints
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormals); //predNormals
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(7, kernelI, sizeof(int), &numPoints);
   opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
   
   opencl_task->setArg(17, kernelI, sizeof(float), &dotThresh);
   opencl_task->setArg(18, kernelI, sizeof(float), &distThresh);

   ocl_float rawResults[NUM_RESULTS];
   float A[DOF][DOF];
   float b[DOF];
   float x[DOF];

   float reg[DOF][DOF];

   tf::Transform curTrans = start;
   
   Pose startPose = curTrans;
   double y,p,r;
   startPose.getYPR(y,p,r);
   cout << "Start pose is: " << startPose.position.x << " " << startPose.position.y << " " <<
         startPose.position.z << "  " << y << " " << p << " " << r << endl;

   float totalMovement[6];
   for (int i = 0; i < 6; i++) {
      totalMovement[i] = 0;
   }

   
   bool limit = false;
   if (scale[3] < MinScale || scale[4] < MinScale || scale[5] < MinScale) {
      limit = true;
   }
   double scaleOff = 16;

   int i;
   bool cont = true;
   bool failed = false;
   for (i = 0; i < MaxICPIterations && cont; i++) {

      if (i == NumICPIterations4) {
         numPoints = (imageWidth / 2) * (imageHeight / 2);
         globalSize = getGlobalWorkSize(numPoints);
         numGroups = globalSize / LocalSize;
         distThresh = DistThresh2;
         dotThresh = DotThresh2;
         scaleOff = 4;
         
         opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clDepthFrameXYZ2);
         opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormalsFrame2);
         opencl_task->setArg(7, kernelI, sizeof(int), &numPoints);
         opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
         opencl_task->setArg(17, kernelI, sizeof(float), &dotThresh);
         opencl_task->setArg(18, kernelI, sizeof(float), &distThresh);
      } else if (i == NumICPIterations2) {
         globalSize = getGlobalWorkSize(numDepthPoints);
         numGroups = globalSize / LocalSize;
         distThresh = DistThresh;
         dotThresh = DotThresh;
         scaleOff = 1;
         
         opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
         opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clNormalsFrame);
         opencl_task->setArg(7, kernelI, sizeof(int), &numDepthPoints);
         opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
         opencl_task->setArg(17, kernelI, sizeof(float), &dotThresh);
         opencl_task->setArg(18, kernelI, sizeof(float), &distThresh);
      }


      tf::Transform temp = curTrans * sensorPose;
      tf::Matrix3x3 basis = temp.getBasis();
      tf::Vector3 origin = temp.getOrigin();

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
      opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clOrigin);
      opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clBasis[0]);
      opencl_task->setArg(11, kernelI, sizeof(ocl_float3), &clBasis[1]);
      opencl_task->setArg(12, kernelI, sizeof(ocl_float3), &clBasis[2]);
      //tf::Transform adjust = start.inverse() * curTrans;
      tf::Transform adjust = (start * sensorPose).inverse() * (curTrans * sensorPose);
      basis = adjust.getBasis();
      origin = adjust.getOrigin();
      ocl_float3 frBasis[3];
      ocl_float3 frOrigin;
      for (int j = 0; j < 3; j++) {
         frBasis[j].x = basis[j][0];
         frBasis[j].y = basis[j][1];
         frBasis[j].z = basis[j][2];
      }
      frOrigin.x = origin[0];
      frOrigin.y = origin[1];
      frOrigin.z = origin[2];
      opencl_task->setArg(13, kernelI, sizeof(ocl_float3), &frOrigin);
      opencl_task->setArg(14, kernelI, sizeof(ocl_float3), &frBasis[0]);
      opencl_task->setArg(15, kernelI, sizeof(ocl_float3), &frBasis[1]);
      opencl_task->setArg(16, kernelI, sizeof(ocl_float3), &frBasis[2]);

      opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

      combineICPResults(numGroups, NUM_RESULTS);

      readBuffer(clLocalMapCommon, CL_TRUE, icpResultsOffset, sizeof(ocl_float) * NUM_RESULTS, 
            rawResults, 0, 0, 0, "Reading the icp results");
      /*A[0][0] = rawResults[0];
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

      solveCholesky(A, b, x);

      cout << "Standard results: " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << " " << x[4]
         << " " << x[5] << endl;*/

      ///////
      memset(reg, 0, sizeof(float) * 36);
      float tempScale[6];
      float max = 0;
      for (int j = 0; j < 6; j++) {
         if (scale[j] > max) {
            max = scale[j];
         }

      }
      for (int j = 0; j < 6; j++) {
         //tempScale[j] = ((scale[j]) / numDepthPoints) * rawResults[27];
         //tempScale[j] = 1 - (scale[j] / numDepthPoints);
         tempScale[j] = 1.0f / (scale[j] / numDepthPoints);
         tempScale[j] = pow(tempScale[j], 2);
         tempScale[j] *= (ScaleRegularisation / scaleOff);
      }
      cout << "Temp scale: " << tempScale[0] << " " << tempScale[1] << " " << tempScale[2]
         << " " << tempScale[3] << " " << tempScale[4] << " " << tempScale[5] << endl;
      multVectorTrans(tempScale, reg);
      //reg[0][0] = reg[1][1] = reg[2][2] = reg[3][3] = reg[4][4] = reg[5][5] = 0;


      A[0][0] = rawResults[0] + reg[0][0];
      A[1][0] = rawResults[1] + reg[1][0];
      A[1][1] = rawResults[2] + reg[1][1];
      A[2][0] = rawResults[3] + reg[2][0];
      A[2][1] = rawResults[4] + reg[2][1];
      A[2][2] = rawResults[5] + reg[2][2];
      A[3][0] = rawResults[6] + reg[3][0];
      A[3][1] = rawResults[7] + reg[3][1];
      A[3][2] = rawResults[8] + reg[3][2];
      A[3][3] = rawResults[9] + reg[3][3];
      A[4][0] = rawResults[10] + reg[4][0];
      A[4][1] = rawResults[11] + reg[4][1];
      A[4][2] = rawResults[12] + reg[4][2];
      A[4][3] = rawResults[13] + reg[4][3];
      A[4][4] = rawResults[14] + reg[4][4];
      A[5][0] = rawResults[15] + reg[5][0];
      A[5][1] = rawResults[16] + reg[5][1];
      A[5][2] = rawResults[17] + reg[5][2];
      A[5][3] = rawResults[18] + reg[5][3];
      A[5][4] = rawResults[19] + reg[5][4];
      A[5][5] = rawResults[20] + reg[5][5];

      float offset[6];
      mult6x6Vector(reg, totalMovement, offset);
      offset[0] = offset[1] = offset[2] = offset[3] = offset[4] = offset[5] = 0;
      b[0] = rawResults[21] - offset[0];
      b[1] = rawResults[22] - offset[1];
      b[2] = rawResults[23] - offset[2];
      b[3] = rawResults[24] - offset[3];
      b[4] = rawResults[25] - offset[4];
      b[5] = rawResults[26] - offset[5];

      int count = rawResults[27];

      /*cout << "Raw results: ";
      for (int j = 0; j < NUM_RESULTS; j++) {
         cout << rawResults[j] << " ";
      }
      cout << endl;*/

      solveCholesky(A, b, x);
      cout << "Modified results: " << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << " " << x[4]
         << " " << x[5] << endl;

      cout << "Total movement: " << totalMovement[0] << " " << totalMovement[1] << " " << totalMovement[2] << " "
          << totalMovement[3] << " " << totalMovement[4] << " " << totalMovement[5] << endl;

      /*cout << "Results: ";
      for (int j = 0; j < DOF; j++) {
         cout << x[j] << " ";
      }
      cout << endl;*/

      bool valid = true;
      cont = false;
      for (int j = 0; j < DOF; j++) {
         if (isnan(x[j])) {
            valid = false;
         }
         if (fabs(x[j]) > MoveThresh) {
            cont = true;
         }
         totalMovement[j] += x[j];
         //if (j >= 3) {
         //   x[j] = 0;
         //}
         if ((j < 3 && fabs(totalMovement[j]) > MaxMoveRPY) || (j >= 3 && fabs(totalMovement[j]) > MaxMoveXYZ)) {
            cout << "******Moved too far" << endl;
            valid = false;
            break;
         }
      }
      if (i < MinICPIterations) {
         cont = true;
      }
      if (!valid) {
         cout << "********Alignment failed" << endl;
         failed = true;
         break;
      }
      if (count < MinICPCount) {
         cout << "***Alignment failed: " << count << endl;
         failed = true;
         break;
      }

      //x[5] = 0;
      //x[0] = 0;
      //x[1] = 0;

      tf::Vector3 incVec(x[3], x[4], x[5]);
      tf::Matrix3x3 incMat(1, x[2], -x[1], -x[2], 1, x[0], x[1], -x[0], 1);
      //tf::Matrix3x3 incMat;
      //incMat.setEulerYPR(x[2], x[1], x[0]);
      tf::Transform inc(incMat, incVec);
      
      Pose incPose = inc;
      incPose.getYPR(y,p,r);

      //cout << incPose.position.x << " " << incPose.position.y << " " << incPose.position.z << " : " 
      //   << y << " " << p << " " << r << " : " << rawResults[27] << " " << endl;
      
      //tf::Vector3 tt = curTrans.getOrigin();
      //curTrans = inc * curTrans;
      //curTrans.setOrigin(tt);

      //tf::Transform before = curTrans;
      //curTrans = inc * curTrans;
      //curTrans.setOrigin(before.getOrigin());

      //Pose pos = inc;
      //pos.getYPR(y,p,r);
      //p = 0;
      //r = 0;
      //pos.setYPR(y,p,r);
      //pos.position.z = 0;
      //inc = pos.toTF();
      
      curTrans = inc * curTrans;

   }
   Pose endPose = curTrans;
   endPose.getYPR(y,p,r);
   cout << "End pose is: " << endPose.position.x << " " << endPose.position.y << " " <<
       endPose.position.z << "  " << y << " " << p << " " << r << endl;
   if (!failed) {
      if (limit) {
         cout << "^^^^Uncertain of x,y,y, limiting movement" << endl;
         double yn, rn, pn, yo, ro, po;
         Pose newPose = curTrans;
         newPose.getYPR(yn,pn,rn);
         icpFullPose.getYPR(yo,po,ro);
         icpFullPose.position.z = newPose.position.z;
         icpFullPose.setYPR(yo,pn,rn);
      } else {
         icpFullPose = curTrans;
      }
      numFails = 0;
   } else {
      numFails++;
   }
   t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   cout << "Total time of icp aligning (" << i << "its): " << totalTime.toSec() * 1000.0f << endl;
}


void PositionTrackFull3D::predictSurface(tf::Transform trans) {
   int kernelI = PREDICT_SURFACE;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPointCloud); //predPoints
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormals); //predNormals
   opencl_task->setArg(6, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(7, kernelI, sizeof(ocl_int3), &mapCentre);
   
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
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(11, kernelI, sizeof(ocl_float3), &clBasis[2]);
   int globalSize = getGlobalWorkSize(numDepthPoints);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrackFull3D::scaleRayTraceICP(int numGroups, float scale[6]) {
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = SCALE_RAY_TRACE_ICP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clNormals); //predNormals
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(3, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(4, kernelI, sizeof(int), &numGroups);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   kernelI = COMBINE_SCALE_ICP_RESULTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(2, kernelI, sizeof(int), &numGroups);
   opencl_task->queueKernel(kernelI, 1, LocalSize, LocalSize, 0, NULL, NULL, false);
   readBuffer(clLocalMapCommon, CL_TRUE, icpScaleResultsOffset, sizeof(ocl_float) * 6, 
         scale, 0, 0, 0, "Reading the scale icp results");
   cout << "Scale results are: " << scale[0] << " " << scale[1] << " " << scale[2] << " " 
      << scale[3] << " " << scale[4] << " " << scale[5] << endl;

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
   
   //Debugging
   /*vector<uint8_t> data;
   data.resize(640 * 480 * sizeof(float));
  
   readBuffer(clFiltDepthFrame, CL_TRUE, 0, sizeDepthPoints, &data[0], 0, 0, 0, 
         "Reading debugging image from the GPU");
   positionTrack3DNode->outputImage(data);*/
}

void PositionTrackFull3D::downsampleDepth() {
   int kernelI = DOWNSAMPLE_DEPTH;
   int widthOut = imageWidth / 2;
   int heightOut = imageHeight / 2;

   int numPoints = widthOut * heightOut;
   int globalSize = getGlobalWorkSize(numPoints);
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clDepthFrame);
   //opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clFiltDepthFrame);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clPointCloud); //image out
   opencl_task->setArg(3, kernelI, sizeof(int), &widthOut);
   opencl_task->setArg(4, kernelI, sizeof(int), &heightOut);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   widthOut /= 2;
   heightOut /= 2;
   numPoints = widthOut * heightOut;
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPointCloud); //image in
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormals); //image out
   opencl_task->setArg(3, kernelI, sizeof(int), &widthOut);
   opencl_task->setArg(4, kernelI, sizeof(int), &heightOut);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   /*vector<uint8_t> data;
   data.resize(160 * 120 * sizeof(float));
  
   readBuffer(clNormals, CL_TRUE, 0, sizeof(float) * 160 * 120, &data[0], 0, 0, 0, 
         "Reading debugging image from the GPU");
   positionTrack3DNode->outputImage(data);*/
}


void PositionTrackFull3D::combineICPResults(int numGroups, int numResults) {
   int kernelI = COMBINE_ICP_RESULTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(2, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(3, kernelI, sizeof(int), &numResults);
   opencl_task->queueKernel(kernelI, 1, LocalSize, LocalSize, 0, NULL, NULL, false);

}

void PositionTrackFull3D::scaleICP(int numGroups, tf::Transform trans, float scale[6]) {
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
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(5, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(6, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[2]);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   kernelI = COMBINE_SCALE_ICP_RESULTS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(2, kernelI, sizeof(int), &numGroups);
   opencl_task->queueKernel(kernelI, 1, LocalSize, LocalSize, 0, NULL, NULL, false);
   readBuffer(clLocalMapCommon, CL_TRUE, icpScaleResultsOffset, sizeof(ocl_float) * 6, 
         scale, 0, 0, 0, "Reading the scale icp results");
   cout << "Scale results are: " << scale[0] << " " << scale[1] << " " << scale[2] << " " 
      << scale[3] << " " << scale[4] << " " << scale[5] << endl;
}

void PositionTrackFull3D::alignZOnlyICP(tf::Transform sensorPose, tf::Transform newPose) {
   tf::Transform curTrans = newPose * sensorPose;

   int kernelI = Z_ONLY_ICP;
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int numGroups = globalSize / LocalSize;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clDepthFrameXYZ);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clNormalsFrame);
   opencl_task->setArg(6, kernelI, sizeof(cl_mem), &clColours); //tempStore
   opencl_task->setArg(7, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(8, kernelI, sizeof(int), &numGroups);
   opencl_task->setArg(9, kernelI, sizeof(ocl_int3), &mapCentre);
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
   float dotThresh = DotThresh;
   float normalThresh = DotThresh4;
   opencl_task->setArg(14, kernelI, sizeof(float), &dotThresh);
   opencl_task->setArg(15, kernelI, sizeof(float), &normalThresh);

   bool success = true;
   int i;
   float zInc = 0;
   float results[2];
   for (i = 0; i < 10; i++) {

      opencl_task->setArg(16, kernelI, sizeof(float), &zInc);
      opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

      combineICPResults(numGroups, 2);

      readBuffer(clLocalMapCommon, CL_TRUE, icpResultsOffset, sizeof(ocl_float) * 2, 
            results, 0, 0, 0, "Reading the icp results");
      float distance = results[0];
      float count = results[1];
      float zAl = distance / count;

      cout << "Count is: " << count << endl;
      int cc = (int)count;
      //if (count < MinCount) {
      if (cc == 0) {
         zInc = 0;
         success = false;
         break;
      }
      zInc += zAl;
      if (fabs(zAl) < MoveThresh) {
         break;
      }
   }
   if (success /*&& fabs(zInc) < MaxMove*/) {
      cout << "Moved: " << zInc << endl;
      Pose endPose = newPose;
      endPose.position.z += zInc;
      icpFullPose = endPose.toTF();
   }
}

void PositionTrackFull3D::outputDebuggingImage(tf::Transform trans) {
   int kernelI = OUTPUT_DEBUGGING_IMAGE;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPointCloud); //predPoints
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clFiltDepthFrame); //depthPoints
   opencl_task->setArg(3, kernelI, sizeof(int), &numDepthPoints);
  
   trans = trans.inverse(); 
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
   opencl_task->setArg(4, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(6, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[2]);
   int globalSize = getGlobalWorkSize(numDepthPoints);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

   vector<uint8_t> data;
   data.resize(640 * 480 * sizeof(float));
  
   readBuffer(clFiltDepthFrame, CL_TRUE, 0, sizeDepthPoints, &data[0], 0, 0, 0, 
         "Reading debugging image from the GPU");
   positionTrack3DNode->outputImage(data);
}

void PositionTrackFull3D::mult6x6Vector(float A[DOF][DOF], float b[DOF], float res[DOF]) {
   for (int y = 0; y < 6; y++) {
      res[y] = 0;
      for (int x = 0; x < 6; x++) {
         res[y] += A[y][x] * b[x];
      }
   }
}
void PositionTrackFull3D::multVectorTrans(float vec[DOF], float res[DOF][DOF]) {
   for (int y = 0; y < 6; y++) {
      /*for (int x = 0; x < 6; x++) {
         res[y][x] = vec[y] * vec[x];
      }*/
      res[y][y] = vec[y];
   }
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
      //currentLocalMapICPPose = icpFullPose;
   } else {
      newLocalMapInfo = localM;
      //newLocalMapICPPose = icpFullPose;
   }
}

void PositionTrackFull3D::forceMapPub() {
   markAllForExtraction();
   int numBlocksToExtract;

   //tf::Transform trans = currentLocalMapICPPose.toTF().inverse();
   tf::Transform trans;
   if (currentLocalMapInfo == NULL) {
      Pose zero;
      currentLocalMapInfo = new LocalMapInfo(zero, 0);
      trans = zero.toTF();
   } else {
      //currentLocalMapInfo->pose = currentLocalMapICPPose; //Test this
      trans = currentLocalMapInfo->icpPose.toTF().inverse();
   }

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

      if (numPoints > 0) {
         transformPoints(numPoints, true, trans);
         addPointsToLocalMap(numPoints);
      }
   }
   if (currentLocalMapInfo->cloud == NULL) {
      currentLocalMapInfo->cloud = new PointCloud();
      currentLocalMapInfo->normals = new PointCloud();
   }
   currentLocalMapInfo->poseHistory.resize(poseHistory.size());
   for (int i = 0; i < poseHistory.size(); i++) {
      currentLocalMapInfo->poseHistory[i] = trans * poseHistory[i]. toTF();
   }
   currentLocalMapInfo->timeHistory = timeHistory;

   positionTrack3DNode->publishLocalMap(currentLocalMapInfo);
}

void PositionTrackFull3D::printFrames(ros::Time stamp, tf::Transform sensorPose) {
   
   tf::Transform rot = sensorPose;
   rot.setOrigin(tf::Vector3(0,0,0));
   int maxPoints = 160 * 120;
   int numPoints = 0;

   if ((stamp - lastTime).toSec() > 1.0) {

      oclDepthPoints points;
      points.x = (ocl_float *) malloc(sizeof(float) * numDepthPoints * 3);
      points.y = &(points.x[numDepthPoints]);
      points.z = &(points.x[numDepthPoints * 2]);
      oclDepthPoints normals;
      normals.x = (ocl_float *) malloc(sizeof(float) * 640 * 480 * 3);
      normals.y = &(normals.x[numDepthPoints]);
      normals.z = &(normals.x[numDepthPoints * 2]);
      readBuffer(clDepthFrameXYZ4, CL_FALSE, 0, sizeDepthFrameXYZ, points.x, 0, 0, 0, 
         "Copying downsampled points to CPU");
      readBuffer(clNormalsFrame4, CL_TRUE, 0, sizeDepthFrameXYZ, normals.x, 0, 0, 0, 
         "Copying downsampled normals to CPU");

      for (int i = 0; i < maxPoints; i++) {
         if (!isnan(points.x[i])) {
            numPoints++;
         }
      }
      ostringstream tt;
      tt << stamp;
      const char *st = tt.str().c_str();
      fprintf(f, "%s %d", st, numPoints);
      for (int i = 0; i < maxPoints; i++) {
         if (!isnan(points.x[i])) {
            tf::Vector3 p(points.x[i], points.y[i], points.z[i]);
            tf::Vector3 n(normals.x[i], normals.y[i], normals.z[i]);
            p = sensorPose * p;
            n = rot * n;
            fprintf(f, " %lf %lf %lf %lf %lf %lf", p.x(), p.y(), p.z(), n.x(), n.y(), n.z());
         }
      }
      fprintf(f, "\n");

      free(points.x);
      free(normals.x);
      lastTime = stamp;
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


