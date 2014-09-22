/*
 * graphSlam3DGPU.cpp
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_gpu/graphSlam3DGPU.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

const string GraphSlam3DGPU::file_names[] = {
   "/src/opencl/graphSlam3d.cl"
};
const int GraphSlam3DGPU::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string GraphSlam3DGPU::header_file = "/include/crosbot_3d_graphslam_gpu/openclCommon.h";
const string GraphSlam3DGPU::kernel_names[] = {
   "clearLocalMap",
   "checkBlocksExist",
   "addRequiredBlocks",
   "addFrame",
   "extractPoints"
};
const int GraphSlam3DGPU::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
#define CLEAR_LOCAL_MAP 0
#define CHECK_BLOCKS_EXIST 1
#define ADD_REQUIRED_BLOCKS 2
#define ADD_FRAME 3
#define EXTRACT_POINTS 4

//The maximum number of points extracted from a block can be SLICE_MULT*the number of 
//cells in a slice of a block
#define SLICE_MULT 2

GraphSlam3DGPU::GraphSlam3DGPU() : GraphSlam3D() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   FILE *file = popen("rospack find crosbot_3d_graphslam_gpu", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;

   hasInitialised = false;

}

GraphSlam3DGPU::~GraphSlam3DGPU() {
   opencl_manager->deviceRelease(clPoints);
   opencl_manager->deviceRelease(clGraphSlam3DConfig);
   opencl_manager->deviceRelease(clLocalMapBlocks);
   opencl_manager->deviceRelease(clLocalMapCells);
   opencl_manager->deviceRelease(clLocalMapCommon);
   delete opencl_task;
   delete opencl_manager;
}

void GraphSlam3DGPU::initialise(ros::NodeHandle &nh) {
   GraphSlam3D::initialise(nh);
   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LocalSize", LocalSize, 256);
   paramNH.param<int>("NumBlocksAllocated", NumBlocksAllocated, 7000);
   paramNH.param<int>("MaxNumActiveBlocks", MaxNumActiveBlocks, 1500);

   //Params that can probably move to general code
   paramNH.param<double>("BlockSize", BlockSize, 0.2);
   paramNH.param<double>("TruncNeg", TruncNeg, 0.3);
   paramNH.param<double>("TrunkPos", TruncPos, 0.7);
   paramNH.param<double>("MaxDistance", MaxDistance, 3.0);

   NumBlocksWidth = (LocalMapWidth + 0.00001) / BlockSize;
   NumBlocksHeight = (LocalMapHeight + 0.00001) / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = (BlockSize + 0.00001) / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;
   cout << "NumCell widths: " << NumCellsWidth << " " << BlockSize << " " << CellSize << endl;

}

void GraphSlam3DGPU::start() {
}

void GraphSlam3DGPU::stop() {
}

void GraphSlam3DGPU::initialiseGraphSlam(DepthPointsPtr depthPoints) {

   cout << "Graph slam 3D: starting compile" << endl;

   //Set config options
   graphSlam3DConfig.ScanWidth = depthPoints->width;
   graphSlam3DConfig.ScanHeight = depthPoints->height;
   graphSlam3DConfig.LocalMapWidth = LocalMapWidth;
   graphSlam3DConfig.LocalMapHeight = LocalMapHeight;
   graphSlam3DConfig.BlockSize = BlockSize;
   graphSlam3DConfig.NumBlocksTotal = NumBlocksTotal;
   graphSlam3DConfig.NumBlocksWidth = NumBlocksWidth;
   graphSlam3DConfig.NumBlocksHeight = NumBlocksHeight;
   graphSlam3DConfig.CellSize = CellSize;
   graphSlam3DConfig.NumCellsTotal = NumCellsTotal;
   graphSlam3DConfig.NumCellsWidth = NumCellsWidth;
   graphSlam3DConfig.NumBlocksAllocated = NumBlocksAllocated;
   graphSlam3DConfig.TruncNeg = TruncNeg;
   graphSlam3DConfig.TruncPos = TruncPos;
   graphSlam3DConfig.fx = fx;
   graphSlam3DConfig.fy = fy;
   graphSlam3DConfig.cx = cx;
   graphSlam3DConfig.cy = cy;
   graphSlam3DConfig.tx = tx;
   graphSlam3DConfig.ty = ty;
   graphSlam3DConfig.MaxDistance = MaxDistance;

   clGraphSlam3DConfig = opencl_manager->deviceAlloc(sizeof(oclGraphSlam3DConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &graphSlam3DConfig);

   numDepthPoints = depthPoints->width * depthPoints->height;
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

   initialiseDepthPoints();
   initialiseLocalMap();
   clearLocalMap(NumBlocksAllocated);

   cout << "Graph slam 3D: finished compile" << endl;
   hasInitialised = true;
}

void GraphSlam3DGPU::addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose) {

   if (finishedSetup && hasInitialised) {

      if (done) {
         return;
      }
      //Calculate offset of robot inside current local map
      tf::Transform robotPose =  (maps[currentMap]->getPose().toTF().inverse()) * slamPose.toTF();
      //Transform points from kinect frame of reference to be relative to local map
      tf::Transform offset = robotPose * sensorPose.toTF();
     
      //Copy the new points to gpu
      int i;
      for (i = 0; i < numDepthPoints; ++i) {
         points->pointX[i] = depthPoints->cloud[i].x;
         points->pointY[i] = depthPoints->cloud[i].y;
         points->pointZ[i] = depthPoints->cloud[i].z;
         points->r[i] = depthPoints->colours[i].r;
         points->g[i] = depthPoints->colours[i].g;
         points->b[i] = depthPoints->colours[i].b;
         //cout << points->pointX[i] << "  " << points->pointY[i] << " " << points->pointZ[i] << endl;

      }
      writeBuffer(clPoints, CL_TRUE, 0, pointsSize, points->pointX, 0, 0, 0,
            "Copying depth points to GPU");
      writeBuffer(clPoints, CL_TRUE, pointsSize, coloursSize, points->r, 0, 0, 0,
            "Copying point colours to GPU");

      {{ Lock lock(masterLock);
      ros::WallTime t1 = ros::WallTime::now();

      //Check that the required blocks in the local map exist
      checkBlocksExist(numDepthPoints, offset);
      if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) 
         cout << "error check blocks exist" << endl;

      /*int tempA[NumBlocksTotal];
      readBuffer(clLocalMapBlocks, CL_TRUE, 0, 
         sizeof(int) * NumBlocksTotal, tempA, 0, 0, 0, "Reading blocks");
      for (int i = 0; i < NumBlocksTotal; i++) {
         if (tempA[i] != -1) {
            cout << tempA[i] << " ";
         }
      }
      cout << endl;*/
      
      ros::WallTime t2 = ros::WallTime::now();
      ros::WallDuration totalTime = t2 - t1;
      cout << "Time of check blocks: " << totalTime.toSec() * 1000.0f << endl;
      /*int temp; 
      readBuffer(clLocalMapCommon, CL_TRUE, 8, 
         sizeof(int), &temp, 0, 0, 0, "Reading num squares set");
      cout << "Squares set are: " << temp << endl;*/

      readBuffer(clLocalMapCommon, CL_TRUE, numActiveBlocksOffset, 
         sizeof(int), &numActiveBlocks, 0, 0, 0, "Reading num active blocks");
      if (numActiveBlocks > MaxNumActiveBlocks) {
         numActiveBlocks = MaxNumActiveBlocks;
      }
      cout << "Num active blocks: " << numActiveBlocks << endl;
      
      t1 = ros::WallTime::now();
      addRequiredBlocks();

      /*int temp[NumBlocksTotal];
      readBuffer(clLocalMapBlocks, CL_TRUE, 0, 
         sizeof(int) * NumBlocksTotal, temp, 0, 0, 0, "Reading blocks");
      for (int i = 0; i < NumBlocksTotal; i++) {
         if (temp[i] >= 0) {
            int z = i / (NumBlocksWidth * NumBlocksWidth);
            double offZ = (BlockSize * NumBlocksHeight) / 2.0;
            double zVal = z * BlockSize - offZ;
            cout << "zval is: " << zVal << " " << offZ << endl;
         }
         if (temp[i] < -1) {
            cout << "Problem..." << endl;
         }
      }*/


      if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) 
         cout << "error required blocks" << endl;
      t2 = ros::WallTime::now();
      totalTime = t2 - t1;
      cout << "Time of add blocks: " << totalTime.toSec() * 1000.0f << endl;
      t1 = ros::WallTime::now();

      addFrame(offset);
      
      if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) 
         cout << "error add frame" << endl;
      t2 = ros::WallTime::now();
      totalTime = t2 - t1;
      cout << "Time of add frame: " << totalTime.toSec() * 1000.0f << endl;
     

      //done = true;

      }}
   } else if (!hasInitialised && receivedCameraParams) {
      initialiseGraphSlam(depthPoints);

      done = false;
   }
}

void GraphSlam3DGPU::newLocalMap(LocalMapInfoPtr localMapInfo) {

   {{ Lock lock(masterLock);

   if (finishedSetup) {
      done = false;
      int numBlocks;
      readBuffer(clLocalMapCommon, CL_TRUE, 0, 
         sizeof(int), &numBlocks, 0, 0, 0, "Reading total number of blocks");
      cout << "***new map. Number of blocks is: " << numBlocks << endl;
      int maxPointsPerBlock = SLICE_MULT * NumCellsWidth * NumCellsWidth;
      int maxPoints = maxPointsPerBlock * numBlocks;
      cl_mem clPointCloud = opencl_manager->deviceAlloc(sizeof(ocl_float) * maxPoints * 3, 
            CL_MEM_READ_WRITE, NULL);
      cl_mem clColours = opencl_manager->deviceAlloc(sizeof(unsigned char) * maxPoints * 3,
            CL_MEM_READ_WRITE, NULL);
      if (clPointCloud == NULL || clColours == NULL) {
         cout << "Error allocated point storage" << endl;
      }

      ros::WallTime t1 = ros::WallTime::now();
      extractPoints(numBlocks, clPointCloud, clColours);
      if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) 
         cout << "error extract points " << maxPoints << endl;
      ros::WallTime t2 = ros::WallTime::now();
      ros::WallDuration totalTime = t2 - t1;
      cout << "Time of extracting points: " << totalTime.toSec() * 1000.0f << endl;
      
      int numPoints;
      readBuffer(clLocalMapCommon, CL_TRUE, numPointsOffset, 
         sizeof(int), &numPoints, 0, 0, 0, "Reading total number of points");
      cout << "Number of points is: " << numPoints << endl;
      
      PointCloudPtr cloud = copyPoints(numPoints, clPointCloud, clColours);

      opencl_manager->deviceRelease(clPointCloud);
      opencl_manager->deviceRelease(clColours);

      clearLocalMap(numBlocks);
      if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) 
         cout << "error clear map" << endl;

      LocalMapInfoPtr oldLocalMap = new LocalMapInfo(maps[currentMap]->getPose(), currentMap,
            cloud);
      graphSlam3DNode->publishLocalMap(oldLocalMap);
   }
   currentMap = localMapInfo->index;
   if (maps.size() == currentMap) {
      maps.push_back(new Local3DMap(localMapInfo->pose));
   } else {
      //At the moment graph slam creates new local maps in existing order, so just adding
      //a map onto the end should work
      cout << "Indexes don't match. ERROR" << endl;
   }

   }}
   finishedSetup = true;
}

void GraphSlam3DGPU::haveOptimised(vector<LocalMapInfoPtr> newMapPositions) {
   if (finishedSetup) {

      {{ Lock lock(masterLock); 

          //Changes to local maps to be sent to viewer
          vector<LocalMapInfoPtr> changes;

          int i;
          for (i = 0; i < newMapPositions.size(); i++) {
             int index = newMapPositions[i]->index;
             maps[index]->updatePose(newMapPositions[i]->pose);
             changes.push_back(new LocalMapInfo(maps[index]->getPose(), index));
          }
          //TODO: Do 3D loop closing here
          
         graphSlam3DNode->publishOptimisedMapPositions(changes);

      }}

   }
}

void GraphSlam3DGPU::initialiseDepthPoints() {
   points = new oclDepthPoints;
   points->pointX = new ocl_float[numDepthPoints * 3];
   points->pointY = &(points->pointX[numDepthPoints]);
   points->pointZ = &(points->pointX[numDepthPoints * 2]);
   points->r = new unsigned char[numDepthPoints * 3];
   points->g = &(points->r[numDepthPoints]);
   points->b = &(points->r[numDepthPoints * 2]);
   pointsSize = sizeof(ocl_float) * numDepthPoints * 3;
   coloursSize = sizeof(unsigned char) * numDepthPoints * 3;
   clPoints = opencl_manager->deviceAlloc(pointsSize + coloursSize, CL_MEM_READ_WRITE, NULL);
}

void GraphSlam3DGPU::initialiseLocalMap() {
   clLocalMapBlocks = opencl_manager->deviceAlloc(sizeof(ocl_int) * NumBlocksTotal, 
         CL_MEM_READ_WRITE, NULL);

   oclLocalBlock localBlock;
   size_t localBlockSize = (sizeof(*(localBlock.distance)) +
      sizeof(*(localBlock.weight)) + sizeof(*(localBlock.pI)) +
      sizeof(*(localBlock.r)) * 3 + 4) * NumCellsTotal + sizeof(localBlock.blockIndex);
   clLocalMapCells = opencl_manager->deviceAlloc(localBlockSize * NumBlocksAllocated, 
         CL_MEM_READ_WRITE, NULL);

   size_t commonSize = sizeof(ocl_int) * (3 + MaxNumActiveBlocks);
   clLocalMapCommon = opencl_manager->deviceAlloc(commonSize,
         CL_MEM_READ_WRITE, NULL);

   oclLocalMapCommon com;
   numActiveBlocksOffset = (unsigned char *)&(com.numActiveBlocks) - (unsigned char *)&(com);
   numPointsOffset = (unsigned char *)&(com.numPoints) - (unsigned char *)&(com);
   
}

void GraphSlam3DGPU::clearLocalMap(int numBlocks) {
   int kernelI = CLEAR_LOCAL_MAP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(int), &numBlocks);
   int globalSize = LocalSize * 20;
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}


void GraphSlam3DGPU::checkBlocksExist(int numPoints, tf::Transform trans) {
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

   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(4, kernelI, sizeof(int), &numPoints);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(6, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[2]);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void GraphSlam3DGPU::addRequiredBlocks() {

   int kernelI = ADD_REQUIRED_BLOCKS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);

   int globalSize = getGlobalWorkSize(numActiveBlocks);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void GraphSlam3DGPU::addFrame(tf::Transform trans) {
   //trans takes point from sensor frame to local map frame.
   tf::Vector3 localMapPose = trans.getOrigin();
   //We need local map frame to snesor frame, so we take the inverse
   tf::Transform offset = trans.inverse();
   tf::Matrix3x3 basis = offset.getBasis();
   tf::Vector3 origin = offset.getOrigin();

   ocl_float3 clLocalMapPose;
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
   clLocalMapPose.x = localMapPose[0];
   clLocalMapPose.y = localMapPose[1];
   clLocalMapPose.z = localMapPose[2];
   
   int kernelI = ADD_FRAME;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(5, kernelI, sizeof(int), &numActiveBlocks);
   opencl_task->setArg(6, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(9, kernelI, sizeof(ocl_float3), &clBasis[2]);
   opencl_task->setArg(10, kernelI, sizeof(ocl_float3), &clLocalMapPose);

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

void GraphSlam3DGPU::extractPoints(int numBlocks, cl_mem &clPointCloud, cl_mem &clColours) {

   int kernelI = EXTRACT_POINTS;

   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPointCloud);
   opencl_task->setArg(5, kernelI, sizeof(cl_mem), &clColours);

   int numBlocksPerGroup = LocalSize / (NumCellsWidth * NumCellsWidth);
   int numGroups = numBlocks / numBlocksPerGroup;
   if (numBlocks % numBlocksPerGroup != 0) {
      numGroups++;
   }
   int globalSize = numGroups * LocalSize;
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

}

PointCloudPtr GraphSlam3DGPU::copyPoints(int numPoints, cl_mem &clPointCloud, cl_mem &clColours) {

   size_t pointsSize = sizeof(float) * numPoints * 3;
   size_t coloursSize = sizeof(unsigned char) * numPoints * 3;
   float *ps = (float *) malloc(pointsSize);
   unsigned char *cols = (unsigned char *) malloc(coloursSize);
   readBuffer(clPointCloud, CL_TRUE, 0, 
         pointsSize, ps, 0, 0, 0, "Reading points");
   readBuffer(clColours, CL_TRUE, 0, 
         coloursSize, cols, 0, 0, 0, "Reading colours");
   
   PointCloudPtr cloud = new PointCloud();
   cloud->cloud.resize(numPoints);
   cloud->colours.resize(numPoints);
   int rIndex = 0;
   int added = 0;
   for (int i = 0; i < numPoints; i++, rIndex = rIndex + 3) {
      Point p;
      Colour c;
      if (!isnan(ps[rIndex])) {
         p.x = ps[rIndex];
         p.y = ps[rIndex + 1];
         p.z = ps[rIndex + 2];
         c.r = cols[rIndex];
         c.g = cols[rIndex + 1];
         c.b = cols[rIndex + 2];
         cloud->cloud[added] = p;
         cloud->colours[added] = c;
         added++;
         //cout << p.x << " " << p.y << " " << p.z << endl;
      } else {
         //cout << "bbb" << endl;
      }
   }
   cout << "Publishing " << added << " points" << endl;
   cloud->cloud.resize(added);
   cloud->colours.resize(added);
   free(ps);
   free(cols);
   return cloud;
}

inline int GraphSlam3DGPU::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
}

void GraphSlam3DGPU::writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
      size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), buffer, blocking_write,
        offset, cb, ptr, num_events_in_wait_list , event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error write buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void GraphSlam3DGPU::readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
      size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), buffer, blocking_read,
         offset, cb, ptr, num_events_in_wait_list, event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error read buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}



