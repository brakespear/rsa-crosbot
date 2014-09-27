/*
 * positionTrack3D.cpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_position_track/positionTrack3D.hpp>


using namespace std;
using namespace crosbot;

const string PositionTrack3D::file_names[] = {
   "/src/opencl/positionTrack3D.cl"
};
const int PositionTrack3D::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string PositionTrack3D::header_file = "/include/crosbot_3d_position_track/openclCommon.h";
const string PositionTrack3D::kernel_names[] = {
   "initialiseMap",
   "transform3D",
   "calculateNormals",
   "alignZ",
   "addScan",
   "clearCells"
};
const int PositionTrack3D::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);
#define INITIALISE_MAP 0
#define TRANSFORM_3D 1
#define CALCULATE_NORMALS 2
#define ALIGN_Z 3
#define ADD_SCAN 4
#define CLEAR_CELLS 5

PositionTrack3D::PositionTrack3D() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   FILE *file = popen("rospack find crosbot_3d_position_track", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;

   failCount = 0;
}

void PositionTrack3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");

   paramNH.param<int>("LocalSize", LocalSize, 256);
   paramNH.param<double>("MapWidth", MapWidth, 10.0);
   paramNH.param<double>("MapHeight", MapHeight, 4.0);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<int>("MaxIterations", MaxIterations, 10);
   paramNH.param<double>("MaxMove", MaxMove, 0.5);
   paramNH.param<double>("MoveThresh", MoveThresh, 0.01);
   paramNH.param<int>("MinCount", MinCount, 1000);
   paramNH.param<int>("MaxFail", MaxFail, 100);
   paramNH.param<int>("BeginScans", BeginScans, 5);
   paramNH.param<int>("MaxSearchCells", MaxSearchCells, 5);
   paramNH.param<double>("InitZ", InitZ, 0.5);
   paramNH.param<int>("MinObsCount", MinObsCount, 2);

   NumCellsWidth = (MapWidth+0.00001) / CellSize;
   NumCellsHeight = (MapHeight+0.00001) / CellSize;
   NumCells = NumCellsWidth * NumCellsWidth * NumCellsHeight;
   beginCount = BeginScans;

   z = InitZ;
   mapCentreX = 0;
   mapCentreY = 0;
   mapCentreZ = (InitZ+0.000001) / CellSize;
   cout << mapCentreZ << endl;

}

void PositionTrack3D::start() {
}

PositionTrack3D::~PositionTrack3D() {
   delete opencl_task;
   delete opencl_manager;
}

void PositionTrack3D::stop() {
}

void PositionTrack3D::initialiseFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose) {
   cout << "Position track 3D: starting compile" << endl;

   //Set config options
   positionTrackConfig.ScanWidth = depthPoints->width;
   positionTrackConfig.ScanHeight = depthPoints->height;
   positionTrackConfig.MapWidth = MapWidth;
   positionTrackConfig.MapHeight = MapHeight;
   positionTrackConfig.CellSize = CellSize;
   positionTrackConfig.NumCellsWidth = NumCellsWidth;
   positionTrackConfig.NumCellsHeight = NumCellsHeight;
   positionTrackConfig.MaxSearchCells = MaxSearchCells;
   positionTrackConfig.MinObsCount = MinObsCount;

   clPositionTrackConfig = opencl_manager->deviceAlloc(sizeof(oclPositionTrackConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &positionTrackConfig);

   numDepthPoints = depthPoints->width * depthPoints->height;
   //Round up to the nearest 16 for memory coalescing
   numDepthPoints = ((numDepthPoints + 15) / 16) * 16;
   stringstream ss;
   ss << "-D NUM_DEPTH_POINTS=" << numDepthPoints <<
         " -D LOCAL_SIZE=" << LocalSize <<
         " -D NUM_CELLS=" << NumCells; 
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels,
         ss.str(), header_file);

   initialiseDepthPoints();
   initialiseMap();

   cout << "Position track 3D: finished compile" << endl;

}

void PositionTrack3D::initialiseDepthPoints() {
   points = new oclDepthPoints;
   points->pointX = new ocl_float[numDepthPoints * 3];
   points->pointY = &(points->pointX[numDepthPoints]);
   points->pointZ = &(points->pointX[numDepthPoints * 2]);
   pointsSize = sizeof(ocl_float) * numDepthPoints * 3;
   clPoints = opencl_manager->deviceAlloc(pointsSize, CL_MEM_READ_WRITE, NULL);
   clNormals = opencl_manager->deviceAlloc(pointsSize, CL_MEM_READ_WRITE, NULL);
}

void PositionTrack3D::initialiseMap() {
   size_t sizeLocalMap = (sizeof(ocl_float) * 4 + sizeof(ocl_int)) * NumCells +
      sizeof(ocl_int) * numDepthPoints;
   clLocalMap = opencl_manager->deviceAlloc(sizeLocalMap, CL_MEM_READ_WRITE, NULL);
   
   size_t sizeCommon = sizeof(ocl_int) + sizeof(ocl_float);
   clCommon = opencl_manager->deviceAlloc(sizeCommon, CL_MEM_READ_WRITE, NULL);

   int kernelI = INITIALISE_MAP;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMap);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clCommon);
   int globalSize = LocalSize * 40;
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) {
      cout << "Error initialising" << endl;
   }
}

Pose PositionTrack3D::processFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose) {
   icpPose.position.z = z;

   int i;
   for (i = 0; i < numDepthPoints; ++i) {
      points->pointX[i] = depthPoints->cloud[i].x;
      points->pointY[i] = depthPoints->cloud[i].y;
      points->pointZ[i] = depthPoints->cloud[i].z;
   }

   //ros::WallTime t1 = ros::WallTime::now();

   writeBuffer(clPoints, CL_TRUE, 0, pointsSize, points->pointX, 0, 0, 0,
         "Copying depth points to GPU");

   transform3D(sensorPose, icpPose);
   calculateNormals();
   if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) {
      cout << "Error normals" << endl;
   }
   //ros::WallTime t2 = ros::WallTime::now();
   //ros::WallDuration totalTime = t2 - t1;
   //cout << "Time to transform and normalise: " << totalTime.toSec() * 1000.0f << endl;
   bool success = true;
   float zChange = 0;
   
   if (beginCount < 0) {
      //t1 = ros::WallTime::now();
      bool success = alignZ(&zChange);
      //t2 = ros::WallTime::now();
      //totalTime = t2 - t1;
      //cout << "Time to align: " << totalTime.toSec() * 1000.0f << endl;
   } else {
      beginCount--;
   }
   if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) {
      cout << "alignZ" << endl;
   }

   if (success) {
      //t1 = ros::WallTime::now();
      addScan(zChange);
      bool success = alignZ(&zChange);
      //t2 = ros::WallTime::now();
      //totalTime = t2 - t1;
      //cout << "Time to add scan: " << totalTime.toSec() * 1000.0f << endl;
      failCount = 0;
   } else {
      failCount++;
      if (failCount >= MaxFail) {
         //TODO: should the map be totally cleared now?
         beginCount = BeginScans;
         cout << "Many scans failed. Adding anyway" << endl;
      }
   }
   if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) {
      cout << "Error addScan" << endl;
   }

   z += zChange;
   icpPose.position.z = z;
   int newMapCentreX = icpPose.position.x / CellSize;
   int newMapCentreY = icpPose.position.y / CellSize;
   int newMapCentreZ = icpPose.position.z / CellSize;

   cout << zChange << " " << icpPose.position.z << endl;

   //t1 = ros::WallTime::now();
   clearCells(newMapCentreX, newMapCentreY, newMapCentreZ);
   if (clFinish(opencl_manager->getCommandQueue()) != CL_SUCCESS) {
      cout << "Error clear cells" << endl;
   }
   //t2 = ros::WallTime::now();
   //totalTime = t2 - t1;
   //cout << "Time to clear cells: " << totalTime.toSec() * 1000.0f << endl;


   mapCentreX = newMapCentreX;
   mapCentreY = newMapCentreY;
   mapCentreZ = newMapCentreZ;
   return icpPose;
}

void PositionTrack3D::transform3D(Pose sensorPose, Pose icpPose) {
   tf::Transform trans = icpPose.toTF() * sensorPose.toTF();
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
   int kernelI = TRANSFORM_3D;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(1, kernelI, sizeof(int), &numDepthPoints);
   opencl_task->setArg(2, kernelI, sizeof(ocl_float3), &clOrigin);
   opencl_task->setArg(3, kernelI, sizeof(ocl_float3), &clBasis[0]);
   opencl_task->setArg(4, kernelI, sizeof(ocl_float3), &clBasis[1]);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float3), &clBasis[2]);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrack3D::calculateNormals() {
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = CALCULATE_NORMALS;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormals);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMap);
   opencl_task->setArg(4, kernelI, sizeof(int), &numDepthPoints);
   ocl_int2 mapCent;
   mapCent.x = mapCentreX;
   mapCent.y = mapCentreY;
   opencl_task->setArg(5, kernelI, sizeof(ocl_int2), &mapCent);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

bool PositionTrack3D::alignZ(float *zChange) {
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = ALIGN_Z;
   float zInc = 0;
   
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormals);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMap);
   opencl_task->setArg(4, kernelI, sizeof(cl_mem), &clCommon);
   opencl_task->setArg(5, kernelI, sizeof(ocl_int), &numDepthPoints);
   opencl_task->setArg(6, kernelI, sizeof(ocl_int), &mapCentreZ);

   oclCommon commonInfo;
   bool success = true;   

   int numIts;
   for (numIts = 0; numIts < MaxIterations; numIts++) {
      commonInfo.numMatch = 0;
      commonInfo.distance = 0;

      writeBuffer(clCommon, CL_FALSE, 0, sizeof(oclCommon), &commonInfo, 0, 0, 0,
            "Copying Zeroing of oclCommon struct");

      opencl_task->setArg(7, kernelI, sizeof(ocl_float), &zInc);
      opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

      readBuffer(clCommon, CL_TRUE, 0, sizeof(oclCommon), &commonInfo, 0, 0, 0,
            "Reading common info after alignment");
      double zAl = commonInfo.distance / (float) commonInfo.numMatch;
      //cout << "Finished iteration " << commonInfo.numMatch << " " << commonInfo.distance 
      //   << " " << zAl << endl;
      if (commonInfo.numMatch < MinCount) {
         zInc = 0;
         success = false;
         break;
      }
      zInc += zAl;
      if (fabs(zAl) < MoveThresh) {
         break;
      }
   }
   if (success && fabs(zInc) < MaxMove) {
      *zChange = zInc;
   } else {
      success = false;
      *zChange = 0;
   }
   //cout << "Finished alignment" << endl;
}

void PositionTrack3D::addScan(float zChange) {
   int globalSize = getGlobalWorkSize(numDepthPoints);
   int kernelI = ADD_SCAN;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(2, kernelI, sizeof(cl_mem), &clNormals);
   opencl_task->setArg(3, kernelI, sizeof(cl_mem), &clLocalMap);
   opencl_task->setArg(4, kernelI, sizeof(ocl_int), &numDepthPoints);
   opencl_task->setArg(5, kernelI, sizeof(ocl_float), &zChange);
   opencl_task->setArg(6, kernelI, sizeof(ocl_int), &mapCentreZ);

   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);
}

void PositionTrack3D::clearCells(int newMapX, int newMapY, int newMapZ) {
   int kernelI = CLEAR_CELLS;
   ocl_int3 newInd, oldInd;
   newInd.x = newMapX;
   newInd.y = newMapY;
   newInd.z = newMapZ;
   oldInd.x = mapCentreX;
   oldInd.y = mapCentreY;
   oldInd.z = mapCentreZ;
   opencl_task->setArg(0, kernelI, sizeof(cl_mem), &clPositionTrackConfig);
   opencl_task->setArg(1, kernelI, sizeof(cl_mem), &clLocalMap);
   opencl_task->setArg(2, kernelI, sizeof(ocl_int3), &oldInd);
   opencl_task->setArg(3, kernelI, sizeof(ocl_int3), &newInd);

   int dimXY = NumCellsWidth * NumCellsHeight;
   int dimZ = NumCellsWidth * NumCellsWidth;
   int dim = dimXY;
   if (dimZ > dim) {
      dim = dimZ;
   }
   int globalSize = getGlobalWorkSize(dim);
   opencl_task->queueKernel(kernelI, 1, globalSize, LocalSize, 0, NULL, NULL, false);

}

inline int PositionTrack3D::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
}

void PositionTrack3D::writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
      size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), buffer, blocking_write,
        offset, cb, ptr, num_events_in_wait_list , event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error write buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void PositionTrack3D::readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
      size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), buffer, blocking_read,
         offset, cb, ptr, num_events_in_wait_list, event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error read buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}


