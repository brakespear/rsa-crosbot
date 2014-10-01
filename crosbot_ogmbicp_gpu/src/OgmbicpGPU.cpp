/*
 * OgmbicpGPU.cpp
 *
 * Created on: 17/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp_gpu/OgmbicpGPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI

const string OgmbicpGPU::file_names[] = {
   "/src/opencl/pogmbicp.cl"
};
const int OgmbicpGPU::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string OgmbicpGPU::header_file = "/include/crosbot_ogmbicp_gpu/openclCommon.h";
const string OgmbicpGPU::kernel_names[] = {
   "initialiseMap", //0
   "update2DMap", //1
   "transform", //2      
   "getNearestByH", //3
   "calcOffsetParams", //4
   "calcMatrix", //5
   "initialUpdatePoints", //6
   "updateActiveCells", //7
   "finalUpdatePoints", //8
};
const int OgmbicpGPU::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
   

OgmbicpGPU::OgmbicpGPU() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   finishedSetup = false;

   discardScan = false;
   numIterations = 0;
   avNumIts = 0;
   pos_x = 0;
   pos_y = 0;
   oldNumPoints = 0;

   FILE *file = popen("rospack find crosbot_ogmbicp_gpu", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;

   f = fopen("/home/adrianr/timing2DICPGPU.txt", "w");
}

OgmbicpGPU::~OgmbicpGPU() {
   delete [] points->pointX;
   delete results;
   delete partialResults;
   delete sharedMap->cellStatus;
   delete sharedMap;
   opencl_manager->deviceRelease(clPoints);
   opencl_manager->deviceRelease(clLocalMap3D);
   opencl_manager->deviceRelease(clResults);


   delete opencl_task;
   delete opencl_manager;
   fclose(f);
}

void OgmbicpGPU::initialise(ros::NodeHandle &nh) {
   Ogmbicp::initialise(nh);
   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LocalSize", LocalSize, 256);
   paramNH.param<int>("MaxLaserPointsInCell", MaxLaserPointsInCell, 4);
   paramNH.param<int>("MaxActiveCells", MaxActiveCells, 4000);
   
}

void OgmbicpGPU::start() {
   config_values.MaxLaserPointsInCell = MaxLaserPointsInCell;
   config_values.MapMaxHeight = MaxAddHeight;
   config_values.MapMinHeight = MinAddHeight;
   config_values.MapDimWidth = MapSize / CellSize;
   double mapHeight = MaxAddHeight - MinAddHeight;
   config_values.MapDimHeight = mapHeight / CellHeight;
   config_values.MapCellWidth = CellSize;
   config_values.MapCellHeight = CellHeight;
   config_values.LaserMaxAlign = LaserMaxAlign;
   config_values.MaxAlignDist = MaxAlignDistance;
   config_values.UseSimpleH = UseSimpleH ? 1 : 0;
   config_values.UseVariableL = UseVariableL ? 1 : 0;
   config_values.UseFactor = UseFactor ? 1 : 0;
   config_values.l = LValue;
   config_values.AlignmentDFix = AlignmentDFix;
   config_values.NearestAlgorithm = NearestAlgorithm;
   config_values.MaxObservations = MaxObservations;
   config_values.FullSearchSize = FullSearchSize;
   config_values.MinFactor = MinFactor;
   config_values.MinGoodCount = MinGoodCount;
   config_values.MaxIterations = MaxIterations;
   config_values.MaxErrX = MaxErrorXY;
   config_values.MaxErrY = MaxErrorXY;
   config_values.MaxErrTh = MaxErrorTh;
   config_values.MaxMoveX = MaxMoveXYZ;
   config_values.MaxMoveY = MaxMoveXYZ;
   config_values.MaxMoveTh = MaxMoveTh;
   config_values.MaxScanSkip = MaxScanSkip;
   config_values.AddSkipCount = AddSkipCount;
   config_values.MaxFail = MaxFail;
   config_values.LifeRatio = LifeRatio;
   config_values.UsePriorMove = UsePriorMove/* ? 1 : 0*/;
   //TODO: set this to be the correct values
   //config_values.FlobsticleHeight = FlobsticleHeight;
   config_values.FlobsticleHeight = 0.05;

   config_values.FloorHeight = FloorHeight;
   config_values.LaserMinDist = LaserMinDist;
   config_values.MaxLaserDist = LaserMaxDistance;

   clConfig = opencl_manager->deviceAlloc(sizeof(configValues), 
                       CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &config_values);
   
   totalMapSize = config_values.MapDimWidth * config_values.MapDimWidth;
   numActiveCellVoxels = MaxActiveCells * config_values.MapDimHeight;
   numLaserPointCoords = numActiveCellVoxels * MaxLaserPointsInCell;
}

void OgmbicpGPU::stop() {
   cout << "stopping ogmbicp_gpu" << endl;

}

void OgmbicpGPU::initialiseTrack(Pose sensorPose, PointCloudPtr cloud) {
   curPose.position.z = zOffset;

   cout << "Starting compile" << endl;

   numLaserPoints = cloud->cloud.size();
   //Round up to the nearest 16 for memory coalescing
   numLaserPoints = ((numLaserPoints + 15) / 16) * 16;
   stringstream ss;
   ss << "-D NUM_LASER_POINTS=" << numLaserPoints << 
         " -D MAP_SIZE=" << totalMapSize << 
         " -D MAX_ACTIVE_CELLS=" << MaxActiveCells << 
         " -D NUM_LASER_POINTS_CELL=" << numActiveCellVoxels <<
         " -D NUM_LASER_POINT_COORDS=" << numLaserPointCoords <<
         " -D LOCAL_SIZE=" << LocalSize;
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels, 
         ss.str(), header_file);
   
   initialiseResults();
   initialiseMap();
   initialisePoints();
   setKernelArgs();

   cout << "ending compile" << endl;
   laserPose = sensorPose;
}

void OgmbicpGPU::initialiseResults() {
   results = new oclResults;
   results->finalOffset.x = 0;
   results->finalOffset.y = 0;
   results->finalOffset.z = 0;
   results->finalOffset.w = 0;
   results->cellShift.x = 0;
   results->cellShift.y = 0;
   results->largestActiveCell = 0;

   clResults = opencl_manager->deviceAlloc(sizeof(oclResults), CL_MEM_READ_WRITE, NULL);
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), clResults, CL_TRUE, 
         0, sizeof(oclResults), results, 0, 0, 0);
   if (ret != CL_SUCCESS) {
      cout << "Error copying initialised results, code: " << ret << endl;
      exit(0);
   }

   partialResults = new oclPartialResults;
   clPartialResults = opencl_manager->deviceAlloc(sizeof(oclPartialResults), CL_MEM_READ_WRITE, NULL);

   sharedMap = new oclSharedMap;
   sharedMap->cellStatus = new ocl_int[MaxActiveCells * 2];
   sharedMap->mapIndex = &(sharedMap->cellStatus[MaxActiveCells]);
   clSharedMap = opencl_manager->deviceAlloc(sizeof(ocl_int) * MaxActiveCells * 2,
         CL_MEM_READ_WRITE, NULL);

   clRes = opencl_manager->deviceAlloc(sizeof(ocl_float) * 1000, CL_MEM_READ_WRITE, NULL);

}

void OgmbicpGPU::initialiseMap() {
   int mapNumInts = MaxActiveCells * 6 + 
                    numLaserPoints +
                    totalMapSize +
                    numActiveCellVoxels + 7;
   int mapNumFloats = 6 * numLaserPointCoords + 3 * numLaserPoints +
                      2 * MaxActiveCells;
   size_t mapSize = sizeof(ocl_int) * mapNumInts + sizeof(ocl_float) * mapNumFloats +
                    sizeof(ocl_float) * 20 + sizeof(ocl_float2);
   clLocalMap3D = opencl_manager->deviceAlloc(mapSize, CL_MEM_READ_WRITE, NULL);
   
   opencl_task->setArg(0, 0, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(1, 0, sizeof(int), &totalMapSize);
   opencl_task->setArg(2, 0, sizeof(int), &MaxActiveCells);
   opencl_task->setArg(3, 0, sizeof(int), &numActiveCellVoxels);
   opencl_task->queueKernel(0, 1, MaxActiveCells, 0, 0, NULL, NULL, false);

}

void OgmbicpGPU::initialisePoints() {
   points = new oclLaserPoints;
   points->pointX = new ocl_float[numLaserPoints * 6];
   points->pointY = &(points->pointX[numLaserPoints]);
   points->pointZ = &(points->pointX[numLaserPoints * 2]);
   points->pointNextX = &(points->pointX[numLaserPoints * 3]);
   points->pointNextY = &(points->pointX[numLaserPoints * 4]);
   points->pointNextZ = &(points->pointX[numLaserPoints * 5]);
   arrayPointsSize = sizeof(ocl_float) * numLaserPoints * 6;
   restPointsSize = sizeof(ocl_float4) + sizeof(ocl_float3);
   clPoints = opencl_manager->deviceAlloc(restPointsSize + arrayPointsSize, CL_MEM_READ_WRITE, NULL);
   points->offset.x = 0;
   points->offset.y = 0;
   points->offset.z = 0;
   points->offset.w = 0;

}

void OgmbicpGPU::setKernelArgs() {
   //kernel 1: updateLocalMap
   //kernel 2: transform
   //kernel 3: getNearestByH
   //kernel 4: calcOffsetParams
   //kernel 5: calcMatrix
   //kernel 6: initialUpdatePoint
   //kernel 7: updateActiveCells
   //kernel 8: finalUpdatePoints
   
   //kernel 1
   opencl_task->setArg(0, 1, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 1, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 1, sizeof(cl_mem), &clResults);

   //kernel 2
   opencl_task->setArg(0, 2, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 2, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(2, 2, sizeof(cl_mem), &clLocalMap3D);
   //opencl_task->setArg(3, 2, sizeof(int), &numPoints);
   //opencl_task->setArg(4, 2, sizeof(int), &oldNumPoints);
   //opencl_task->setArg(5, 2, sizeof(int), 0);
   opencl_task->setArg(6, 2, sizeof(cl_mem), &clRes);
   opencl_task->setArg(8, 2, sizeof(cl_mem), &clResults);

   //kernel 3
   opencl_task->setArg(0, 3, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 3, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 3, sizeof(cl_mem), &clPoints);
   //opencl_task->setArg(3, 3, sizeof(int), &numPoints);
   opencl_task->setArg(4, 3, sizeof(cl_mem), &clRes);

   //kernel 4
   opencl_task->setArg(0, 4, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 4, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 4, sizeof(cl_mem), &clPoints);
   //opencl_task->setArg(3, 4, sizeof(int), &numPoints);
   opencl_task->setArg(4, 4, sizeof(cl_mem), &clRes);

   //kernel 5
   opencl_task->setArg(0, 5, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 5, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 5, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(3, 5, sizeof(cl_mem), &clResults);
   opencl_task->setArg(4, 5, sizeof(cl_mem), &clPartialResults);
   opencl_task->setArg(5, 5, sizeof(cl_mem), &clRes);

   //kernel 6
   opencl_task->setArg(0, 6, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 6, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 6, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(3, 6, sizeof(cl_mem), &clResults);
   //opencl_task->setArg(4, 6, sizeof(int), &numPoints);
   opencl_task->setArg(5, 6, sizeof(cl_mem), &clRes);

   //kernel 7
   opencl_task->setArg(0, 7, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 7, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 7, sizeof(cl_mem), &clResults);
   opencl_task->setArg(3, 7, sizeof(cl_mem), &clSharedMap);
   opencl_task->setArg(4, 7, sizeof(cl_mem), &clRes);

   //kernel 8
   opencl_task->setArg(0, 8, sizeof(cl_mem), &clConfig);
   opencl_task->setArg(1, 8, sizeof(cl_mem), &clLocalMap3D);
   opencl_task->setArg(2, 8, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(3, 8, sizeof(cl_mem), &clResults);
   //opencl_task->setArg(4, 8, sizeof(int), &numPoints);

}

void OgmbicpGPU::updateTrack(Pose sensorPose, PointCloudPtr cloud) {
   curPose.position.z = zOffset;
   if (discardScan) {
      cout << "Ignoring scan" << endl;
   }
   ros::WallTime t1 = ros::WallTime::now();

   //Filter out points that are too close to the robot
   vector<Point> newCloud;
   int i;
   for (i = 0; i < cloud->cloud.size(); i++) {
      if (cloud->cloud[i].x * cloud->cloud[i].x + 
            cloud->cloud[i].y * cloud->cloud[i].y > LaserMinDist * LaserMinDist) {
         newCloud.push_back(cloud->cloud[i]);
      }
   }
   cloud->cloud = newCloud;

   PointCloudPtr worldPoints = centerPointCloud(*cloud, curPose, sensorPose);
   laserPose = sensorPose;
   //Old place timer t1 went

   int numPoints = (int)cloud->cloud.size() < numLaserPoints ?
                   (int)cloud->cloud.size() : numLaserPoints;
   numPoints = prepareLaserPoints(worldPoints, numPoints, sensorPose);
   setNumPointsKernelArgs(numPoints);
   int globalSizePoints = getGlobalWorkSize(numPoints);
   if (oldNumPoints > numPoints) {
      globalSizePoints = getGlobalWorkSize(oldNumPoints);
   }

   if (InitialScans > 0) {
      queueMapUpdateKernels(numPoints);
      InitialScans--;
      return;
   }

   partialResults->proceed = 1;

   //debugging array
   //float outP[1000];

   while (partialResults->proceed == 1) {
      //transform
      opencl_task->queueKernel(2, 1, globalSizePoints, LocalSize, 0, NULL, NULL, false);
      int globalSizeCalcH = getGlobalWorkSize(numPoints * 32);
      //getNearestByH
      opencl_task->queueKernel(3, 1, globalSizeCalcH, LocalSize, 0, NULL, NULL, false);
      //calcOffsetParams
      opencl_task->queueKernel(4, 1, globalSizePoints, LocalSize, 0, NULL, NULL, false);
      //calcMatrix
      opencl_task->queueKernel(5, 1, 32, 0, 0, NULL, NULL, false);
      //copy partial results
      int retVal = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), clPartialResults, CL_TRUE,
            0, sizeof(oclPartialResults), partialResults, 0, 0, 0);
      if (retVal != CL_SUCCESS) {
         cout << "Copying partial results failed, code: " << retVal << endl;
         exit(0);
      }
      int yy = 0;
      opencl_task->setArg(5, 2, sizeof(int), &yy);
      avNumIts++;
      /*retVal = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), 
               clRes, CL_TRUE, 0, sizeof(float) * 500, outP, 0, 0, 0);
      if (retVal != CL_SUCCESS) {
         cout << "Copy of debug data failed " << retVal << " dumb de dumb" << endl;
         if (retVal == CL_INVALID_COMMAND_QUEUE) {
            cout << "command queue" << endl;
         } else if (retVal == CL_INVALID_CONTEXT) {
            cout << "invalud context" << endl;
         } else if (retVal == CL_INVALID_MEM_OBJECT) {
            cout << "invalid mem object" << endl;
         } else if (retVal == CL_OUT_OF_HOST_MEMORY) {
            cout << "out of mem" << endl;
         } else if (retVal == CL_INVALID_VALUE) {
            cout << "invalid value" << endl;
         } else if (retVal == CL_INVALID_EVENT_WAIT_LIST) {
            cout << "invalid event wait list" << endl;
         } else if (retVal == CL_MEM_OBJECT_ALLOCATION_FAILURE) {
            cout << "mem obj failure" << endl;
         } else {
            cout << "ret val is " << retVal << endl;
         }
         exit(0);
      }*/

   }
   queueMapUpdateKernels(numPoints);
   
   double gx, gy, gz, gth;
   gx = results->finalOffset.x;
   gy = results->finalOffset.y;
   gz = results->finalOffset.z;
   gth = results->finalOffset.w;

   //transformToRobot(gx, gy, gz, gth); 

   curPose.position.x += gx; 
   curPose.position.y += gy;
   curPose.position.z = zOffset + gz;
   
   double roll, pitch, yaw;
   curPose.getYPR(yaw, pitch, roll);
   yaw += gth;
   ANGNORM(yaw);
   curPose.setYPR(yaw,pitch,roll);


   ros::WallTime t2 = ros::WallTime::now();
   totalTime = t2 - t1;
   fprintf(f, "%lf\n", totalTime.toSec() * 1000.0f);

   //totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms "
         << curPose.position.x << " " << curPose.position.y << " " <<
         yaw << " " << (float) avNumIts / numIterations << endl;
   }

   oldNumPoints = numPoints;
   finishedSetup = true;
   
   Pose3D newPose;
   newPose = curPose;
   PointCloudPtr hist = new PointCloud("/icp", *cloud, newPose);
   hist->timestamp = ros::Time::now();
   recentScans.push_back(hist);
   while((recentScans.back()->timestamp - 
            recentScans.front()->timestamp).toSec() > ScanListTime) {
      recentScans.pop_front();
   }   
}

int OgmbicpGPU::prepareLaserPoints(PointCloudPtr p, int numPoints, Pose sensorPose) {
   int i;
   double dx, dy, dz;
   int numAdded = 0;

   /*double minZ = 999;
   double maxZ = -999;
   for (i = 0; i < numPoints - 1; i++) {
         if (p->cloud[i].z > maxZ) {
            maxZ = p->cloud[i].z;
         }
         if (p->cloud[i].z < minZ) {
            minZ = p->cloud[i].z;
         }
   }
   int removed = 0;*/
   for (i = 0; i < numPoints - 1; i++) {
      dx = p->cloud[i+1].x - p->cloud[i].x;
      dy = p->cloud[i+1].y - p->cloud[i].y;
      dz = p->cloud[i+1].z - p->cloud[i].z;
      if (IgnoreZValues) { 
         dz = 0;
      }
      dx = dx * dx + dy * dy + dz * dz;
      if (dx < MaxSegLen) {
         if (IgnoreZValues && (p->cloud[i].z < MinAddHeight || p->cloud[i].z > MaxAddHeight)) {
            continue;
         }
         if (p->cloud[i].z < FloorHeight) {
            continue;
         }
         if ((!isnan(floorHeight) && p->cloud[i].z <= floorHeight) /*|| (maxZ - minZ > 0.5 && p->cloud[i].z < minZ + 0.2)*/) {
            //removed++;
            continue;
         }
         points->pointX[numAdded] = p->cloud[i].x;
         points->pointY[numAdded] = p->cloud[i].y;
         points->pointZ[numAdded] = p->cloud[i].z;
         points->pointNextX[numAdded] = p->cloud[i + 1].x;
         points->pointNextY[numAdded] = p->cloud[i + 1].y;
         points->pointNextZ[numAdded] = p->cloud[i + 1].z;

         if (IgnoreZValues) {
            points->pointZ[numAdded] = (float)(MinAddHeight + MaxAddHeight) / 2.0f;
            points->pointNextZ[numAdded] = (float) (MinAddHeight + MaxAddHeight) / 2.0f;
         }
         ++numAdded;         
      }
   }
   //cout << "Z's are: " << minZ << " " << maxZ << " " << floorHeight << " " << zOffset << " " << removed << " " << numAdded << endl;
   
   if (!UsePriorMove) {
      memset(&(points->offset), 0, 16);
   } else {
      points->offset.x = results->finalOffset.x;
      points->offset.y = results->finalOffset.y;
      points->offset.z = results->finalOffset.z;
      //points->offset.z = 0;
      points->offset.w = results->finalOffset.w;
   }

   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), clPoints, CL_TRUE, 
         0, arrayPointsSize, points->pointX, 0, 0, 0);
   if (ret != CL_SUCCESS) {
      cout << "Error copying points to GPU, code: " << ret << endl;
      exit(0);
   }
   ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), clPoints, CL_TRUE, 
         arrayPointsSize, restPointsSize, &(points->offset), 0, 0, 0);
   if (ret != CL_SUCCESS) {
      cout << "Error copying offset vals to GPU, code: " << ret << endl;
      exit(0);
   }
   return numAdded;
}

void OgmbicpGPU::setNumPointsKernelArgs(int numPoints) {
   int num = 1;
   opencl_task->setArg(3, 2, sizeof(int), &numPoints);
   opencl_task->setArg(4, 2, sizeof(int), &oldNumPoints);
   opencl_task->setArg(5, 2, sizeof(int), &num);
   opencl_task->setArg(3, 3, sizeof(int), &numPoints);
   opencl_task->setArg(3, 4, sizeof(int), &numPoints);
   opencl_task->setArg(4, 6, sizeof(int), &numPoints);
   opencl_task->setArg(4, 8, sizeof(int), &numPoints);
   int isFinal = 0;
   opencl_task->setArg(7, 2, sizeof(int), &isFinal);

}

inline int OgmbicpGPU::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                  ((numThreads / LocalSize) + 1) * LocalSize;

}

void OgmbicpGPU::queueMapUpdateKernels(int numThreads) {
   int globalSizePoints = getGlobalWorkSize(numThreads);
   if (oldNumPoints > numThreads) {
      globalSizePoints = getGlobalWorkSize(oldNumPoints);
   }



   int isFinal = 1;
   opencl_task->setArg(7, 2, sizeof(int), &isFinal);
   clEnqueueReadBuffer(opencl_manager->getCommandQueue(), clResults, CL_TRUE, 0, 
         sizeof(oclResults), results, 0, 0, 0);
   double gx, gy, gz, gth;
   gx = results->finalOffset.x;
   gy = results->finalOffset.y;
   gz = results->finalOffset.z;
   gth = results->finalOffset.w;
   transformToRobot(gx, gy, gz, gth);
   results->finalOffset.x = gx; 
   results->finalOffset.y = gy; 
   results->finalOffset.z = gz; 
   results->finalOffset.w = gth;
   clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), clResults, CL_TRUE, 
         0, sizeof(oclResults), results, 0, 0, 0);
   



   opencl_task->queueKernel(2, 1, globalSizePoints, LocalSize, 0, NULL, NULL, false);
   opencl_task->queueKernel(6, 1, globalSizePoints, LocalSize, 0, NULL, NULL, false);
   int res = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), clResults, CL_TRUE, 0, 
         sizeof(oclResults), results, 0, 0, 0);
   if (res != CL_SUCCESS) {
      cout << "Error copying map update data, code: " << res << endl;
      exit(0);
   }
   int globalSizeActiveCells = getGlobalWorkSize(results->largestActiveCell);
   opencl_task->queueKernel(7, 1, globalSizeActiveCells, LocalSize, 0, NULL, NULL, false);
   opencl_task->queueKernel(8, 1, globalSizePoints, LocalSize, 0, NULL, NULL, false);
   opencl_task->queueKernel(1, 1, globalSizeActiveCells, LocalSize, 0, NULL, NULL, false);
   
}

PointCloudPtr OgmbicpGPU::centerPointCloud(PointCloud &p, Pose curPose, Pose sensorPose) {

   pos_x += results->cellShift.x * CellSize;
   pos_y += results->cellShift.y * CellSize;

   Pose3D newPose = curPose;
   newPose.position.x = newPose.position.x - pos_x;
   newPose.position.y = newPose.position.y - pos_y;

   PointCloudPtr rval = new PointCloud("/world", p, newPose);

   Pose absSensorPose = newPose.toTF() * sensorPose.toTF();
   points->laserOffset.x = absSensorPose.position.x;
   points->laserOffset.y = absSensorPose.position.y;
   points->laserOffset.z = absSensorPose.position.z;

   return rval;
}

void OgmbicpGPU::getLocalMap(LocalMapPtr curMap) {
   int res = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), clSharedMap, CL_TRUE, 0, 
         sizeof(ocl_int) * MaxActiveCells * 2, sharedMap->cellStatus, 0, 0, 0);
   if (res != CL_SUCCESS) {
      cout << "Copy of map drawing info failed, code: " << res << endl;
      exit(0);
   }

   double lifeScale = curMap->maxHits / (LifeRatio * MaxObservations);
   int x,y;
   for (y = 0; y < curMap->height; y++) {
      LocalMap::Cell *cellsP = &(curMap->cells[y][0]);
      for (x = 0; x < curMap->width; x++) {
         cellsP->current = false;
         cellsP->hits = 0;
         cellsP++;
      }
   }
   curMap->origin.position.x = curPose.position.x - 
      (curMap->width * curMap->resolution) / 2;
   curMap->origin.position.y = curPose.position.y - 
      (curMap->height * curMap->resolution) / 2;
   int mapWidth = MapSize / CellSize;
   double off = (mapWidth * CellSize) / 2.0 - CellSize / 2.0;
   for(x = 0; x < results->largestActiveCell; x++) {
      if (sharedMap->cellStatus[x] >= 0 || sharedMap->cellStatus[x] == -2) {
         int index = sharedMap->mapIndex[x];
         int yi = index / mapWidth;
         int xi = index % mapWidth;
         double yd = yi * CellSize - off;
         double xd = xi * CellSize - off;
         yi = curMap->height/2 - yd / curMap->resolution;
         xi = curMap->width/2 + xd / curMap->resolution;
         if (xi >= 0 && xi < curMap->width && yi >= 0 && yi < curMap->height) {
            LocalMap::Cell *cellsP = &(curMap->cells[curMap->height - yi - 1][xi]);
            if (sharedMap->cellStatus[x] == 0) {
               //cellsP->hits = lifeScale * sharedMap->cellStatus[x];
               cellsP->hits = curMap->maxHits;
               cellsP->current = true;
            } else if (sharedMap->cellStatus[x] == -2) {
               cellsP->hits = curMap->maxHits - 15;
            } else {
               //cellsP->hits = lifeScale * sharedMap->cellStatus[x];
               cellsP->hits = curMap->maxHits;
            }
         } 
      }
   }
}
