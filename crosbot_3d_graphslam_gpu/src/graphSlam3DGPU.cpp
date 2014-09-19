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
   "addFrame"
};
const int GraphSlam3DGPU::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
#define CLEAR_LOCAL_MAP 0
#define CHECK_REQUIRED_BLOCKS_EXIST 1
#define ADD_REQUIRED_BLOCKS 2
#define ADD_FRAME 3


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
   delete opencl_task;
   delete opencl_manager;
}

void GraphSlam3DGPU::initialise(ros::NodeHandle &nh) {
   GraphSlam3D::initialise(nh);
   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LocalSize", LocalSize, 256);
   paramNH.param<int>("NumBlocksAllocated", NumBlocksAllocated, 1000);

   //Params that can probably move to general code
   paramNH.param<double>("BlockSize", BlockSize, 0.2);

   NumBlocksWidth = LocalMapWidth / BlockSize;
   NumBlocksHeight = LocalMapHeight / BlockSize;
   NumBlocksTotal = NumBlocksWidth * NumBlocksWidth * NumBlocksHeight;
   NumCellsWidth = BlockSize / CellSize;
   NumCellsTotal = NumCellsWidth * NumCellsWidth * NumCellsWidth;


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
   graphSlam3DConfig.fx = fx;
   graphSlam3DConfig.fy = fy;
   graphSlam3DConfig.cx = cx;
   graphSlam3DConfig.cy = cy;
   graphSlam3DConfig.tx = tx;
   graphSlam3DConfig.ty = ty;

   clGraphSlam3DConfig = opencl_manager->deviceAlloc(sizeof(oclGraphSlam3DConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &graphSlam3DConfig);

   numDepthPoints = depthPoints->width * depthPoints->height;
   //Round up to the nearest 16 for memory coalescing
   numDepthPoints = ((numDepthPoints + 15) / 16) * 16;
   stringstream ss;
   ss << "-D NUM_DEPTH_POINTS=" << numDepthPoints <<
         " -D LOCAL_SIZE=" << LocalSize <<
         " -D NUM_CELLS=" << NumCellsTotal;
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
      }
      writeBuffer(clPoints, CL_TRUE, 0, pointsSize, points->pointX, 0, 0, 0,
            "Copying depth points to GPU");

      {{ Lock lock(masterLock);

      //Check that the required blocks in the local map exist
      checkBlocksExist(numDepthPoints, offset);
      addRequiredBlocks();
      addFrame(numDepthPoints, offset);

      }}
   } else if (!hasInitialised && receivedCameraParams) {
      initialiseGraphSlam(depthPoints);
   }
}

void GraphSlam3DGPU::newLocalMap(LocalMapInfoPtr localMapInfo) {

   {{ Lock lock(masterLock);

   if (finishedSetup) {
      /*PointCloudPtr cloud = localMap->extractPoints(ObsThresh);
      LocalMapInfoPtr oldLocalMap = new LocalMapInfo(maps[currentMap]->getPose(), currentMap,
            cloud);
      graphSlam3DNode->publishLocalMap(oldLocalMap);
      localMap->clearGrid();*/
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
   pointsSize = sizeof(ocl_float) * numDepthPoints * 3;
   clPoints = opencl_manager->deviceAlloc(pointsSize, CL_MEM_READ_WRITE, NULL);
}

void GraphSlam3DGPU::initialiseLocalMap() {
   clLocalMapBlocks = opencl_manager->deviceAlloc(sizeof(ocl_int) * NumBlocksTotal, 
         CL_MEM_READ_WRITE, NULL);

   oclLocalBlock localBlock;
   size_t localBlockSize = (sizeof(*(localBlock.distance)) +
      sizeof(*(localBlock.weight)) +
      sizeof(*(localBlock.r)) * 3) * NumCellsTotal;
   clLocalMapCells = opencl_manager->deviceAlloc(localBlockSize * NumBlocksAllocated, 
         CL_MEM_READ_WRITE, NULL);

   clLocalMapCommon = opencl_manager->deviceAlloc(sizeof(oclLocalMapCommon),
         CL_MEM_READ_WRITE, NULL);
   
}

void GraphSlam3DGPU::clearLocalMap(int numBlocks) {
   int kernelI = CLEAR_LOCAL_MAP
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
      clOrigin.x = origin[0];
   }
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
}

void GraphSlam3DGPU::addFrame(int numPoints, tf::Transform trans) {
   //trans takes point from sensor frame to local map frame.
   //We need local map frame to snesor frame, so we take the inverse
   tf::Transform offset = trans.inverse();
   tf::Matrix3x3 basis = offset.getBasis();
   tf::Vector3 origin = offset.getOrigin();

   ocl_float3 clBasis[3];
   ocl_float3 clOrigin;
   for (int j = 0; j < 3; j++) {
      clBasis[j].x = basis[j][0];
      clBasis[j].y = basis[j][1];
      clBasis[j].z = basis[j][2];
      clOrigin.x = origin[0];
   }
   
   int kernelI = ADD_FRAME;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clGraphSlam3DConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMapBlocks);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clLocalMapCells);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMapCommon);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(6, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(7, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(8, kernelI, sizeof(ocl_float3), &clBasis[2]);

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



