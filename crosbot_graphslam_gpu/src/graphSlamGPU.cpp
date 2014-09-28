/*
 * graphSlamGPU.cpp
 *
 * Created on: 8/1/2014
 *     Author: adrianr
 *
 * GPU version of graph slam
 */

#include <crosbot_graphslam_gpu/graphSlamGPU.hpp>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI
#define SQ(X) ((X)*(X))

const string GraphSlamGPU::file_names[] = {
   "/src/opencl/graphSlam.cl"
};
const int GraphSlamGPU::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string GraphSlamGPU::header_file = "/include/crosbot_graphslam_gpu/openclCommon.h";
const string GraphSlamGPU::kernel_names[] = {
   "initialiseSlam", //0
   "addScanToMap", //1
   "createNewLocalMap", //2      
   "getHessianMatch", //3
   "prepareLocalMap", //4
   "findPotentialMatches", //5
   "alignICP", //6
   "calculateICPMatrix", //7
   "finaliseInformationMatrix", //8
   "getGlobalHessianMatrix", //9
   "calculateOptimisationChange", //10
   "updateGlobalPositions", //11
   "updateGlobalMap", //12
   "combineNodes", //13
   "translateScanPoints", //14
   "setLocalMap", //15
   "evaluateMapMatch", //16
   "addLoopClosure", //17
   "evaluateTempConstraints", //18
   "getGlobalMapPositions", //19
   "resetOccupancyGrid", //20
   "setOccupancyGrid", //21
   "updateWarpPoints"  //22
};
const int GraphSlamGPU::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);


GraphSlamGPU::GraphSlamGPU() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   numIterations = 0;
   finishedSetup = false;
   currentLocalMap = 0;
   numGlobalPoints = 0;
   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   resetMap = false;
   lastDrawnGlobalPoints = 0;
   parentLocalMap = -1;
   nextLocalMap = 0;
   combineMode = 0;
   numConstraints = 0;
   numLoopConstraints = 0;
   maxNumLocalPoints = 0;
   firstScan = 1;
   numScans = 0;
   lastFullLoopIndex = -1;

   currentOffset.x = 0;
   currentOffset.y = 0;
   currentOffset.z = 0;
   currentOffset.w = 0;

   totalLocalMaps = 100;
   totalGlobalPoints = 10000;

   //kinect params
   lastCloudPublished = 0;
   messageSize = 0;
   activeMapIndex = -1;

   FILE *file = popen("rospack find crosbot_graphslam_gpu", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;
}

void GraphSlamGPU::initialise(ros::NodeHandle &nh) {
   GraphSlam::initialise(nh);

   ros::NodeHandle paramNH("~");
   paramNH.param<int>("LocalSize", LocalSize, 256);
}

void GraphSlamGPU::start() {
   slam_config.CellWidthOG = CellSize;
   slam_config.DimLocalOG = DimLocalOG;
   slam_config.DimGlobalOG = DimGlobalOG;
   slam_config.SearchSize = SearchSize;
   slam_config.MaxSlamAlignDist = MaxAlignDistance;
   slam_config.SlamL = LValue;
   slam_config.InformationScaleFactor = InformationScaleFactor;
   slam_config.MaxCovar = MaxCovar;
   slam_config.CorrelationThreshold = CorrelationThreshold;
   slam_config.MinGoodCount = MinGoodCount;
   slam_config.FinalMinGoodCount = FinalMinGoodCount;
   slam_config.MaxIterations = MaxIterations;
   slam_config.MaxErrorDisp = MaxErrorDisp;
   slam_config.MaxErrorTheta = MaxErrorTheta;
   slam_config.LocalMapDist = LocalMapDistance;
   slam_config.LocalMapCombine = LocalMapCombine;
   slam_config.MaxThetaOptimise = MaxThetaOptimise;
   slam_config.MinObservationCount = MinObservationCount;
   slam_config.PerScanInfoScaleFactor = PerScanInfoScaleFactor;
   slam_config.GradientDistanceThreshold = GradientDistanceThreshold;
   slam_config.FreeAreaDistanceThreshold = FreeAreaDistanceThreshold;
   slam_config.LocalMapCovarianceThreshold = LocalMapCovarianceThreshold;
   slam_config.FreeAreaThreshold = FreeAreaThreshold;
   slam_config.PreventMatchesSymmetrical = PreventMatchesSymmetrical;
   slam_config.TempConstraintMovementXY = TempConstraintMovementXY;
   slam_config.TempConstraintMovementTh = TempConstraintMovementTh;

   clSlamConfig = opencl_manager->deviceAlloc(sizeof(slamConfig), 
          CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &slam_config);

   localOGSize = DimLocalOG * DimLocalOG;

   //the offsets in the data structures that are read
   numPointsOffset = sizeof(ocl_float4) * 2;
   matchSuccessOffset = sizeof(ocl_float4) +
                               sizeof(ocl_float2) * 2 +
                               sizeof(ocl_int);
   combineIndexOffset = matchSuccessOffset + sizeof(int);
   numPotentialMatchesOffset = matchSuccessOffset + sizeof(int) * 3;
   evaluateOffset = sizeof(ocl_float4) + sizeof(ocl_float2) * 2 +
                           sizeof (ocl_int) * (5 + MAX_POTENTIAL_MATCHES * 2) +
                           sizeof (ocl_float) * MAX_POTENTIAL_MATCHES * 3;
   tempCovarOffset = evaluateOffset + sizeof(ocl_int) + sizeof(ocl_float);
}

void GraphSlamGPU::stop() {
   cout << "stopping graphslam_gpu" << endl;
}

GraphSlamGPU::~GraphSlamGPU() {
   opencl_manager->deviceRelease(clSlamCommon);
   opencl_manager->deviceRelease(clSlamLocalMap);
   opencl_manager->deviceRelease(clGlobalMap);
   opencl_manager->deviceRelease(clGlobalMapHeights);
   opencl_manager->deviceRelease(clSlamConfig);
   opencl_manager->deviceRelease(clFreeAreas);

   delete [] points->pointX;
   delete [] globalMap;
   delete [] numLocalMapPoints;
   delete [] globalMapHeights;
   delete [] globalMapPositions;

   delete opencl_task;
   delete opencl_manager;
}


void GraphSlamGPU::initialiseTrack(Pose icpPose, PointCloudPtr cloud) {
   currentLocalMapICPPose = icpPose;
   oldICPPose = icpPose;
   startICPTheta = 0;

   cout << "Starting compile" << endl;
   numLaserPoints = cloud->cloud.size();
   //Round up to the nearest 16 for memory coalescing
   numLaserPoints = ((numLaserPoints + 15) / 16) * 16;
   stringstream ss;
   ss << "-D NUM_LASER_POINTS=" << numLaserPoints << 
         " -D LOCAL_SIZE=" << LocalSize <<
         " -D SIZE_LOCAL_OG=" << localOGSize <<
         " -D MAX_NUM_CONSTRAINTS=" << MaxNumConstraints << 
         " -D MAX_NUM_LOOP_CONSTRAINTS=" << MaxNumLoopConstraints;
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels, 
         ss.str(), header_file);

   initialisePoints();
   initialiseSlamStructures();
   setKernelArgs();

   cout << "ending compile" << endl;

   LocalMap temp;
   temp.indexNextNode = -1;
   localMaps.push_back(temp);

   localMaps[0].globalPose = slamPose;
   LocalMapInfo newMap(slamPose, 0);
   graphSlamNode->publishLocalMapInfo(newMap);

}

void GraphSlamGPU::initialisePoints() {
   points = new oclLaserPoints;
   points->pointX = new ocl_float[numLaserPoints * 3];
   points->pointY = &(points->pointX[numLaserPoints]);
   points->pointZ = &(points->pointX[numLaserPoints * 2]);
   arrayPointsSize = sizeof(ocl_float) * numLaserPoints * 3;
   clPoints = opencl_manager->deviceAlloc(arrayPointsSize, CL_MEM_READ_WRITE, NULL);
}

void GraphSlamGPU::initialiseSlamStructures() {
   
   int numFloats = localOGSize * 5 +
                   numLaserPoints * 3 + 
                   MAX_LOCAL_POINTS * 2 +
                   NUM_ORIENTATION_BINS * 2 +
                   MAX_POTENTIAL_MATCHES * 3 +
                   MaxNumLoopConstraints * 13 +
                   (MaxNumConstraints + 1) * 3 +
                   25;
   int numInts = localOGSize * 2 + 
                 MAX_LOCAL_POINTS + 
                 MAX_POTENTIAL_MATCHES * 2 + 
                 MaxNumConstraints * 2 +
                 MaxNumLoopConstraints * 6 + 12;
   size_t size = sizeof(ocl_float) * numFloats +
                 sizeof(ocl_int) * numInts +
                 sizeof(ocl_float4) +
                 sizeof(ocl_float2) * 2;
   clSlamCommon = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);

   size = sizeof(slamLocalMap) * totalLocalMaps;
   clSlamLocalMap = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);

   size = sizeof(ocl_float) * localOGSize * totalLocalMaps;
   clFreeAreas = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);

   size = sizeof(ocl_int) * totalGlobalPoints;
   clGlobalMap = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMap = new int[totalGlobalPoints];
   numLocalMapPoints = new int[totalLocalMaps];

   size = sizeof(ocl_float) * totalGlobalPoints;
   clGlobalMapHeights = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMapHeights = new float[totalGlobalPoints];

   size = totalLocalMaps * sizeof(ocl_float) * 5;
   clGlobalMapPositions = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMapPositions = new ocl_float[totalLocalMaps * 5];
   globalMapPositions[0] = 0;
   globalMapPositions[1] = 0;
   globalMapPositions[2] = 0;

   opencl_task->setArg(0, 0, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 0, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 0, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 0, sizeof(cl_mem), &clFreeAreas);
   opencl_task->queueKernel(0, 1, MAX_LOCAL_POINTS, 0, 0, NULL, NULL, false);

   clRes = opencl_manager->deviceAlloc(sizeof(ocl_float) * 1000, CL_MEM_READ_WRITE, NULL);
   
   indexParentNode.push_back(-1);
}

void GraphSlamGPU::setKernelArgs() {
   //kernel 0: initialiseSlam
   //kernel 1: addScanToMap
   //kernel 2: createNewLocalMap
   //kernel 3: getHessianMatch
   //kernel 4: prepareLocalMap
   //kernel 5: findPotentialMatches
   //kernel 6: alginICP
   //kernel 7: calculateICPMatrix
   //kernel 8: finaliseInformationMatrix
   //kernel 9: getGlobalHessianMatrix
   //kernel 10: calculateOptimisationChange
   //kernel 11: updateGlobalPositions
   //kernel 12: updateGlobalMap 
   //kernel 13: combineNodes
   //kernel 14: translateScanPoints
   //kernel 15: setLocalMap
   //kernel 16: evaluateMapMapMatch
   //kernel 17: addLoopClosure
   //kernel 18: evaluateTempConstraints
   //kernel 19: getGlobalMapPositions
   //kernel 20: resetOccupancyGrid
   //kernel 21: setOccupancyGrid
   
   //kernel 1: addScanToMap
   opencl_task->setArg(0, 1, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 1, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 1, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 1, sizeof(cl_mem), &clGlobalMap);
   opencl_task->setArg(4, 1, sizeof(cl_mem), &clGlobalMapHeights);
   opencl_task->setArg(5, 1, sizeof(cl_mem), &clFreeAreas);
   //opencl_task->setArg(6, 1, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(7, 1, sizeof(int), &numPoints);
   //opencl_task->setArg(8, 1, sizeof(int), &numGlobalPoints);
   //opencl_task->setArg(9, 1, sizeof(int), &firstScan);
   opencl_task->setArg(10, 1, sizeof(cl_mem), &clRes);

   //kernel 2: createNewLocalMap 
   opencl_task->setArg(0, 2, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 2, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 2, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 2, sizeof(cl_mem), &clFreeAreas);
   //opencl_task->setArg(4, 2, sizeof(int), &oldLocalMap);
   //opencl_task->setArg(5, 2, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(6, 2, sizeof(int), &oldLocalMap);
   //opencl_task->setArg(7, 2, sizeof(ocl_float), &angleError);
   //opencl_task->setArg(8, 2, sizeof(int), &numOldPoints);

   //kernel 3: getHessianMatch
   opencl_task->setArg(0, 3, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 3, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 3, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 3, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(4, 3, sizeof(int), &constraintIndex);
   
   //kernel 4: prepareLocalMap
   opencl_task->setArg(0, 4, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 4, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 4, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 4, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(4, 4, sizeof(int), &numLocalMaps);
   //opencl_task->setArg(5, 4, sizeof(int), &numScans);
   opencl_task->setArg(6, 4, sizeof(cl_mem), &clRes);

   //kernel 5: findPotentialMatches
   opencl_task->setArg(0, 5, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 5, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 5, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 5, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(4, 5, sizeof(int), &numLocalMaps);
   opencl_task->setArg(5, 5, sizeof(cl_mem), &clRes);
   
   //kernel 6: alignICP
   opencl_task->setArg(0, 6, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 6, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 6, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 6, sizeof(int), &otherMap);
   //opencl_task->setArg(4, 6, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(5, 6, sizeof(int), &numPointsOtherMap);
   //opencl_task->setArg(6, 6, sizeof(int), &matchIndex);

   //kernel 7: calculateICPMatrix
   opencl_task->setArg(0, 7, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 7, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 7, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 7, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(4, 7, sizeof(int), &matchIndex);
   //opencl_task->setArg(5, 7, sizeof(int), &fullLoop);
   opencl_task->setArg(6, 7, sizeof(cl_mem), &clRes);

   //kernel 8: finaliseInformationMatrix
   opencl_task->setArg(0, 8, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 8, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(2, 8, sizeof(int), &constraintIndex);
   opencl_task->setArg(3, 8, sizeof(cl_mem), &clRes);

   //kernel 9: getGlobalHessianMatrix
   opencl_task->setArg(0, 9, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 9, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 9, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 9, sizeof(int), &numMaps);
   opencl_task->setArg(4, 9, sizeof(cl_mem), &clRes);

   //kernel 10: calculateOptimisationChange
   opencl_task->setArg(0, 10, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 10, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 10, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 10, sizeof(int), &numIterations);
   //opencl_task->setArg(4, 10, sizeof(int), &numMaps);
   //opencl_task->setArg(5, 10, sizeof(int), &type);
   //opencl_task->setArg(6, 10, sizeof(int), &lastFullLoopIndex);
   opencl_task->setArg(7, 10, sizeof(cl_mem), &clRes);

   //kernel 11: updateGlobalPositions
   opencl_task->setArg(0, 11, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 11, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 11, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 11, sizeof(int), &numLocalMaps);
   opencl_task->setArg(4, 11, sizeof(cl_mem), &clRes);
   
   //kernel 12: updateGlobalMap 
   opencl_task->setArg(0, 12, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 12, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 12, sizeof(cl_mem), &clGlobalMap);
   //opencl_task->setArg(3, 12, sizeof(int), &numLocalMaps);
   opencl_task->setArg(4, 12, sizeof(cl_mem), &clGlobalMapPositions);
   opencl_task->setArg(5, 12, sizeof(cl_mem), &clGlobalMapHeights);

   //kernel 13: combineNodes
   opencl_task->setArg(0, 13, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 13, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 13, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 13, sizeof(cl_mem), &clGlobalMap);
   //opencl_task->setArg(4, 13, sizeof(int), &currentMap);
   //opencl_task->setArg(5, 13, sizeof(float), &alignError);
   //opencl_task->setArg(6, 13, sizeof(ocl_float4), &offset);
   //opencl_task->setArg(7, 13, sizeof(int), &numGlobalPoints);
   //opencl_task->setArg(8, 13, sizeof(int), &numOtherGlobalPoints);
   opencl_task->setArg(9, 13, sizeof(cl_mem), &clGlobalMapHeights);
   
   //kernel 14: translateScanPoints
   opencl_task->setArg(0, 14, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 14, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 14, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 14, sizeof(cl_mem), &clPoints);
   //opencl_task->setArg(4, 14, sizeof(int), &numPoints);
   //opencl_task->setArg(5, 14, sizeof(ocl_float4), &currentOffset);
   opencl_task->setArg(6, 14, sizeof(cl_mem), &clRes);
   
   //kernel 15: setLocalMap
   opencl_task->setArg(0, 15, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 15, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 15, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 15, sizeof(cl_mem), &clGlobalMapHeights);
   //opencl_task->setArg(4, 15, sizeof(int), &currentMap);
   //opencl_task->setArg(5, 15, sizeof(int), &numGlobalPoints);
   
   //kernel 16: evaluateMapMatch
   opencl_task->setArg(0, 16, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 16, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 16, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 16, sizeof(cl_mem), &clFreeAreas);
   //opencl_task->setArg(4, 16, sizeof(int), &curMap);
   //opencl_task->setArg(5, 16, sizeof(int), &testMap);
   //opencl_task->setArg(6, 16, sizeof(int), &mode);

   //kernel 17: addLoopClosure
   opencl_task->setArg(0, 17, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 17, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 17, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 17, sizeof(int), &currentMap);
   //opencl_task->setArg(4, 17, sizeof(int), &matchIndex);
   //opencl_task->setArg(5, 17, sizeof(int), &fullLoop);
   //opencl_task->setArg(6, 17, sizeof(float), &previousScore);
   opencl_task->setArg(7, 17, sizeof(cl_mem), &clRes);
   
   //kernel 18: evaluateTempConstraints
   opencl_task->setArg(0, 18, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 18, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 18, sizeof(cl_mem), &clSlamCommon);

   //kernel 19: getGlobalMapPositions
   opencl_task->setArg(0, 19, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 19, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 19, sizeof(cl_mem), &clGlobalMapPositions);
   //opencl_task->setArg(3, 19, sizeof(int), &numLocalMaps);

   //kernel 20: resetOccupancyGrid
   opencl_task->setArg(0, 20, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 20, sizeof(cl_mem), &clSlamCommon);

   //kernel 21: setOccupancyGrid
   opencl_task->setArg(0, 21, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 21, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 21, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 21, sizeof(int), &mapNum);
   
   //kernel 22: updateWarpPoints
   opencl_task->setArg(0, 22, sizeof(cl_mem), &clSlamLocalMap);
   //opencl_task->setArg(1, 22, sizeof(cl_mem), &clPointsStore);
   //opencl_task->setArg(2, 22, sizeof(int), &mapIndex);
   //opencl_task->setArg(2, 22, sizeof(int), &numWarpPoints);
   
}

void GraphSlamGPU::updateTrack(Pose icpPose, PointCloudPtr cloud, ros::Time time) {

   ros::WallTime t1 = ros::WallTime::now();

   double diffX, diffY, diffTh;

   //Update the current positions
   slamPose.position.z = icpPose.position.z;

   //get the difference in icp position since the last update
   diffX = icpPose.position.x - oldICPPose.position.x; 
   diffY = icpPose.position.y - oldICPPose.position.y; 
   double yi, pi, ri;
   double yo, po, ro;
   icpPose.getYPR(yi, pi, ri);
   oldICPPose.getYPR(yo, po, ro);
   diffTh = yi - yo;
   ANGNORM(diffTh);
   //Update the slam position
   double ys, ps, rs;
   slamPose.getYPR(ys, ps, rs);
   ys += diffTh;
   ANGNORM(ys);
   double angleError = ys - yi;
   double cosTh = cos(angleError);
   double sinTh = sin(angleError);
   slamPose.position.x += cosTh * diffX - sinTh * diffY;
   slamPose.position.y += sinTh * diffX + cosTh * diffY;
   slamPose.setYPR(ys, pi, ri);
   //Update the offset in the current local map
   offsetFromParentX += diffX;
   offsetFromParentY += diffY;
   offsetFromParentTh += diffTh;
   ANGNORM(offsetFromParentTh);
   //Need to convert diffX and diffY to be relative to the current local map
   cosTh = cos(-startICPTheta);
   sinTh = sin(-startICPTheta);
   currentOffset.x += diffX * cosTh - diffY * sinTh;
   currentOffset.y += diffX * sinTh + diffY * cosTh;
   currentOffset.w += diffTh;
   ANGNORM(currentOffset.w);

   Scan *newScan = new Scan();
   memset(newScan->covar, 0, sizeof(double) * 9);
   newScan->pose[0] = currentOffset.x;
   newScan->pose[1] = currentOffset.y;
   newScan->pose[2] = currentOffset.w;
   newScan->stamp = time;
   newScan->correction[0] = 0;
   newScan->correction[1] = 0;
   newScan->correction[2] = 0;

   //Add the points to the current local map
   int i, j;
   int numAdded = 0;
   for (i = 0; i < cloud->cloud.size(); ++i) {
      double dist = cloud->cloud[i].x * cloud->cloud[i].x + cloud->cloud[i].y * cloud->cloud[i].y;
      if (dist > LaserMinDist * LaserMinDist && dist < LaserMaxDist * LaserMaxDist &&
         cloud->cloud[i].z + InitHeight > MinAddHeight && cloud->cloud[i].z + InitHeight < MaxAddHeight) {
         points->pointX[numAdded] = cloud->cloud[i].x;
         points->pointY[numAdded] = cloud->cloud[i].y;
         points->pointZ[numAdded] = cloud->cloud[i].z + InitHeight;
         numAdded++;

         newScan->points.push_back(cloud->cloud[i]);
      }
   }

   //cout << "Points: " << numAdded << " / " << numLaserPoints << endl;

   int globalSize = getGlobalWorkSize(numAdded);

   opencl_task->setArg(4, 14, sizeof(int), &numAdded);
   opencl_task->setArg(5, 14, sizeof(ocl_float4), &currentOffset);

   opencl_task->setArg(6, 1, sizeof(int), &currentLocalMap);
   opencl_task->setArg(7, 1, sizeof(int), &numAdded);
   opencl_task->setArg(8, 1, sizeof(int), &numGlobalPoints);
   opencl_task->setArg(9, 1, sizeof(int), &firstScan);

   writeBuffer(clPoints, CL_TRUE, 0, arrayPointsSize, points->pointX, 0, 0, 0,
         "Copying points to GPU");
   opencl_task->queueKernel(14, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   float tempStore[10];
   readBuffer(clSlamCommon, CL_FALSE, tempCovarOffset, sizeof(ocl_float) * 10, tempStore,
         0, 0, 0, "Copying temp covar of scan");
   opencl_task->queueKernel(1, 1, globalSize, LocalSize, 0, NULL, NULL, false);

   int isFeatureless;
   size_t offset = currentLocalMap * sizeof(slamLocalMap) +
            sizeof(ocl_float4) * 2 + sizeof(ocl_int);
   readBuffer(clSlamLocalMap, CL_TRUE, offset, sizeof(ocl_int), &isFeatureless, 0, 0, 0,
         "Copying is featureless");

   if (localMaps[currentLocalMap].scans.size() > 0) {
      double tempCovar[3][3];
      int *coP = (int *)&(tempStore[9]);
      int covarCount = *coP;
      int count = 0;

      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++, count++) {
            tempCovar[i][j] = tempStore[count] / (double) covarCount;
         }
      }
      invert3x3Matrix(tempCovar, newScan->covar);
      for (i = 0; i < 3; i++) {
         for (j = 0; j < 3; j++) {
            newScan->covar[i][j] /= PerScanInfoScaleFactor;
         }
      }
   }
   if (!LocalMapWarp) {
      newScan->points.clear();
   }
   localMaps[currentLocalMap].scans.push_back(newScan);
   activeMapIndex = currentLocalMap;


   firstScan = 0;
   numScans++;

   double temp = sqrt(offsetFromParentX * offsetFromParentX +
         offsetFromParentY * offsetFromParentY);
   //TODO: might want to add the more complicated (but currently disabled)
   //map changing stuff from the CPU code
   if (temp >= LocalMapDistance || isFeatureless > 0) {
      if (isFeatureless > 0) {
         isMapFeatureless.push_back(true);
         cout << "Featureless map: ";
      } else {
         isMapFeatureless.push_back(false);
      }
      //cout << "Creating a new local map " << currentLocalMap << endl;
      cout << "Creating a new local map " << currentLocalMap << " " << 
         slamPose.position.x << " " << slamPose.position.y << " " << offsetFromParentX <<
         " " << offsetFromParentY << endl;

      float x = currentOffset.x / 2.0;
      float y = currentOffset.y / 2.0;
      float cosGl = cos(globalMapPositions[currentLocalMap * 5 + 2]);
      float sinGl = sin(globalMapPositions[currentLocalMap * 5 + 2]);
      globalMapPositions[currentLocalMap * 5 + 3] = x * cosGl - y * sinGl
         + globalMapPositions[currentLocalMap * 5];
      globalMapPositions[currentLocalMap * 5 + 4] = x * sinGl + y * cosGl
         + globalMapPositions[currentLocalMap * 5 + 1];


      finishMap(angleError, yi, icpPose);
      firstScan = 1;
      numScans = 0;
   }

   oldICPPose = icpPose;

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms ";// << endl;
      cout << slamPose.position.x << " " << slamPose.position.y << " " << ys << endl;
   }
   
   //if (numIterations == 20) {
   //   exit(0);
   //}

   finishedSetup = true;
}

void GraphSlamGPU::finishMap(double angleError, double icpTh, Pose icpPose) {

{{ Lock lock(masterLockSmith);


   //Read the number of points in the current local map
   int numLocalPoints;
   size_t offset = currentLocalMap * sizeof(slamLocalMap) +
            numPointsOffset;
   readBuffer(clSlamLocalMap, CL_TRUE, offset, sizeof(ocl_int), &numLocalPoints, 0, 0, 0,
         "Copying number of laser points in current map");
   cout << "Local map has: " << numLocalPoints << " points" << endl;

   numLocalMapPoints[currentLocalMap] = numLocalPoints;
   localMaps[currentLocalMap].numWarpPoints = numLocalPoints;
   if (numLocalPoints > maxNumLocalPoints) {
      maxNumLocalPoints = numLocalPoints;
   } 

   int oldLocalMap = currentLocalMap;
   int numOldMapPoints = numLocalPoints;

   if (parentLocalMap >= 0) {
      parentLocalMap = currentLocalMap;
      int globalSize = getGlobalWorkSize(numLocalMapPoints[currentLocalMap]);
      int potentialMatches[MAX_POTENTIAL_MATCHES + 1];

      int noConstraint = -1;
      opencl_task->setArg(3, 4, sizeof(int), &currentLocalMap);
      opencl_task->setArg(4, 4, sizeof(int), &nextLocalMap);
      opencl_task->setArg(5, 4, sizeof(int), &numScans);
      opencl_task->setArg(4, 15, sizeof(int), &currentLocalMap);
      opencl_task->setArg(5, 15, sizeof(int), &numGlobalPoints);
      //setLocalMap and prepareLocalMap
      opencl_task->queueKernel(15, 1, globalSize, LocalSize, 
                                 0, NULL, NULL, false);
      opencl_task->queueKernel(4, 1, NUM_ORIENTATION_BINS, 0, 
                                 0, NULL, NULL, false);
      bool needOptimisation = false;

         /*int kk;
         float outPP[100];
         readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outPP, 0, 0, 0, "");
         cout << "internal covar: " << endl;
         for (kk = 0; kk < 9; kk++) {
            cout << outPP[kk] << " ";
         }
         cout << endl;*/

      if (combineMode > 0) {
         int combineIndex;
         readBuffer(clSlamCommon, CL_TRUE, combineIndexOffset, sizeof(ocl_int), &combineIndex, 
               0, 0, 0, "Copying combine index to see if should combine");

         if (combineIndex >= 0) {
            cout << "****Combining node: " << currentLocalMap << " with " 
               << combineIndex << endl;
            int mIndex = 0;
            opencl_task->setArg(3, 6, sizeof(int), &combineIndex);
            opencl_task->setArg(4, 6, sizeof(int), &currentLocalMap);
            opencl_task->setArg(5, 6, sizeof(int), &(numLocalMapPoints[combineIndex]));
            opencl_task->setArg(6, 6, sizeof(int), &mIndex);
            opencl_task->setArg(3, 7, sizeof(int), &currentLocalMap);
            opencl_task->setArg(4, 7, sizeof(int), &mIndex);
            int fullLoop = 1;
            opencl_task->setArg(5, 7, sizeof(int), &fullLoop);

            int nI = 0;
            int matchSuccess = 0;
            while (matchSuccess == 0) {
               //alignICP and calculateICPMatrix
               opencl_task->queueKernel(6, 1, globalSize, LocalSize,
                                 0, NULL, NULL, false);
               opencl_task->queueKernel(7, 1, 32, 0,
                                 0, NULL, NULL, false);
               readBuffer(clSlamCommon, CL_TRUE, matchSuccessOffset, sizeof(ocl_int), 
                     &matchSuccess, 0, 0, 0, "Error copying match success combining");
               nI++;
            }
            cout << "Number of iterations: " << nI << endl;
            // *** Debugging 
            float outP[100];
            readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outP, 0, 0, 0, "");
            int k;
            cout << " Alignment is: ";
            for (k = 0; k < 4; k++) {
               cout << outP[k] << " ";
            }
            cout << endl;
            // *** End debugging

            if (matchSuccess == 1) {
               cout << "Combining map succeeded" << endl;
               int combineSize = getGlobalWorkSize(numLocalPoints);
               opencl_task->setArg(4, 13, sizeof(int), &currentLocalMap);
               float alignError = (float) angleError;
               opencl_task->setArg(5, 13, sizeof(float), &alignError);
               ocl_float4 pOffset;
               pOffset.x = offsetFromParentX;
               pOffset.y = offsetFromParentY;
               pOffset.z = 0;
               pOffset.w = offsetFromParentTh;
               opencl_task->setArg(6, 13, sizeof(ocl_float4), &pOffset);
               opencl_task->setArg(7, 13, sizeof(int), &numGlobalPoints);
               int numOtherGlobalPoints = 0;
               int i;
               for (i = 0; i < combineIndex; i++) {
                  numOtherGlobalPoints += numLocalMapPoints[i];
               }
               opencl_task->setArg(8, 13, sizeof(int), &numOtherGlobalPoints);
               //CombineNodes
               opencl_task->queueKernel(25, 1, combineSize, LocalSize,
                            0, NULL, NULL, false);

               if (combineMode == 1) {
                  needOptimisation = true;
                  numConstraints++;
               }
               int temp = numLocalMapPoints[combineIndex];
               numLocalMapPoints[combineIndex] += numLocalPoints;
               if (numLocalMapPoints[combineIndex] > MAX_LOCAL_POINTS) {
                  numLocalMapPoints[combineIndex] = MAX_LOCAL_POINTS;
               }
               numLocalPoints = numLocalMapPoints[combineIndex] - temp;
               cout << "New num points is: " << 
                     numLocalMapPoints[combineIndex] << " " << numLocalPoints << endl;
               if (numLocalMapPoints[combineIndex] > maxNumLocalPoints) {
                  maxNumLocalPoints = numLocalMapPoints[combineIndex];
               }

               int mapOffset = currentLocalMap * sizeof(slamLocalMap);
               //Update global position of robot
               double ys, ps, rs;
               slamPose.getYPR(ys, ps, rs);
               cout << "global pos robot before: " << slamPose.position.x << " " <<
                  slamPose.position.y << " " << ys << endl;
               ocl_float4 globalPos;
               readBuffer(clSlamLocalMap, CL_TRUE, mapOffset, sizeof(ocl_float4),
                      &globalPos, 0, 0, 0, "Copying global pos info");
               cout << "global pos robot after: " << globalPos.x << " " <<
                     globalPos.y << " " << globalPos.w << endl;
               slamPose.position.x = globalPos.x;
               slamPose.position.y = globalPos.y;
               ys = globalPos.w;
               slamPose.setYPR(ys, ps, rs);
               //Update parent offset of local map
               cout << "parent offset before: " << offsetFromParentX << " " <<
                     offsetFromParentY << " " << offsetFromParentTh << endl;
               ocl_float4 tempOff;
               readBuffer(clSlamLocalMap, CL_TRUE, mapOffset + sizeof(ocl_float4), 
                      sizeof(ocl_float4), &tempOff, 0, 0, 0, "Copying parent offset info");
               offsetFromParentX = tempOff.x;
               offsetFromParentY = tempOff.y;
               offsetFromParentTh = tempOff.w;
               cout << "parent offset after: " << offsetFromParentX << " " <<
                     offsetFromParentY << " " << offsetFromParentTh << endl;

               numLocalMapPoints[currentLocalMap] = 0;

               //Need to update snaps and slam history to the combined local map
               float offset[MAX_POTENTIAL_MATCHES * 3];
               readBuffer(clSlamCommon, CL_TRUE, numPotentialMatchesOffset + sizeof(int) * 
                     (MAX_POTENTIAL_MATCHES + 1), sizeof(ocl_float) * (MAX_POTENTIAL_MATCHES*3),
                     &offset, 0, 0, 0, "Copying potential matches for fixing positions");
               double alignX = offset[0];
               double alignY = offset[MAX_POTENTIAL_MATCHES];
               double alignTh = offset[MAX_POTENTIAL_MATCHES * 2];
               fixSnapPositions(combineIndex, alignX, alignY, alignTh);
               fixSlamHistoryPositions(combineIndex, alignX, alignY, alignTh);

               currentLocalMap = combineIndex;
               resetMap = true;
            } else {
               cout << "Combing map alignment failed" << matchSuccess << endl;
               combineMode = 0;
            }
         } else {
            combineMode = 0;
         }
      }
      bool loopClosed = false;
      if (!combineMode) {
         opencl_task->setArg(3, 5, sizeof(int), &currentLocalMap);
         opencl_task->setArg(4, 5, sizeof(int), &nextLocalMap);
         globalSize = getGlobalWorkSize(nextLocalMap * 32);
         opencl_task->queueKernel(5, 1, globalSize, LocalSize,
              0, NULL, NULL, false);
         readBuffer(clSlamCommon, CL_TRUE, numPotentialMatchesOffset, 
              sizeof(ocl_int) * (MAX_POTENTIAL_MATCHES + 1), &potentialMatches, 
              0, 0, 0, "Copying potential matches");
         nextLocalMap++; //in original code this goes after the set args
         numConstraints++;
         
         if (potentialMatches[0] > 0 && !resetMap) {
            cout << "************" << endl;
            cout << "Potential Match found with map: " << potentialMatches[1] << endl;
            cout << "************" << endl;

            // *** Debugging
            float temp[MAX_POTENTIAL_MATCHES*3];
            readBuffer(clSlamCommon, CL_TRUE, numPotentialMatchesOffset 
                         + sizeof(int) * (MAX_POTENTIAL_MATCHES + 1), 
                         sizeof(ocl_float) * (MAX_POTENTIAL_MATCHES*3),
                         &temp, 0, 0, 0, "Copying potential match offsets");
            float corrScores[10];
            readBuffer(clRes, CL_TRUE, 0, sizeof(ocl_float) * 10, &corrScores,
                     0, 0, 0, "Copying correlation scores");
            int i;
            cout << "Potential Matches are: " << endl;
            for(i = 1; i <= potentialMatches[0]; i++) {
               cout << potentialMatches[i] << " " << temp[i-1] << " " << 
                    temp[MAX_POTENTIAL_MATCHES + i-1] << " " << 
                    temp[MAX_POTENTIAL_MATCHES * 2 + i - 1] << " " <<
                    " score: " << corrScores[i-1] << endl;
            }  
            cout << endl;
            // End debugging

            for (i = 1; i <= potentialMatches[0]; i++) {
               opencl_task->setArg(3, 6, sizeof(int), &(potentialMatches[i]));
               opencl_task->setArg(4, 6, sizeof(int), &currentLocalMap);
               opencl_task->setArg(5, 6, sizeof(int), &(numLocalMapPoints[potentialMatches[i]]));
               int mIndex = i - 1;
               opencl_task->setArg(6, 6, sizeof(int), &mIndex);
               opencl_task->setArg(3, 7, sizeof(int), &currentLocalMap);
               opencl_task->setArg(4, 7, sizeof(int), &mIndex);
               int fullLoop = 1;
               opencl_task->setArg(5, 7, sizeof(int), &fullLoop);

               int runSize = getGlobalWorkSize(
                        numLocalMapPoints[potentialMatches[i]]);

               int matchSuccess = 0;
               int c = 0;
               while (matchSuccess == 0) {
                  opencl_task->queueKernel(6, 1, runSize, LocalSize,
                                    0, NULL, NULL, false);
                  opencl_task->queueKernel(7, 1, 32, 0,
                                    0, NULL, NULL, false);
                  readBuffer(clSlamCommon, CL_TRUE, matchSuccessOffset, sizeof(ocl_int), 
                        &matchSuccess, 0, 0, 0, "Copying match success");
                  c++;
               }
               // *** Debugging
               float outP[100];
               readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outP, 0, 0, 0, "");
               int k;
               cout << " Alignment of " << potentialMatches[i] << " is: ";
               for (k = 0; k < 4; k++) {
                  cout << outP[k] << " ";
               }
               cout << numLocalMapPoints[potentialMatches[i]] << endl;
               // End debugging
               cout << "Number of iterations in alignment: " << c << " " << matchSuccess << endl;

               if (matchSuccess == 1) {
                  cout << "Alignment Succeeded" << endl;
                  opencl_task->setArg(4, 16, sizeof(int), &currentLocalMap);
                  opencl_task->setArg(5, 16, sizeof(int), &(potentialMatches[i]));
                  int mode = i - 1;
                  opencl_task->setArg(6, 16, sizeof(int), &mode);
                  opencl_task->queueKernel(16, 1, globalSize, LocalSize,
                     0, NULL, NULL, false);
                  opencl_task->setArg(3, 17, sizeof(int), &currentLocalMap);
                  opencl_task->setArg(4, 17, sizeof(int), &mode);
                  opencl_task->setArg(5, 17, sizeof(int), &fullLoop);
                  ocl_float prevScore = 0;
                  opencl_task->setArg(6, 17, sizeof(float), &prevScore);
                  opencl_task->queueKernel(17, 1, 32, 0,
                     0, NULL, NULL, false);
                  readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outP, 0, 0, 0, "");
                  cout << "Values from map matching are: " << outP[0] << " " <<
                     outP[1] << " " << outP[2] << endl;

                  readBuffer(clSlamCommon, CL_TRUE, matchSuccessOffset, sizeof(ocl_int), 
                             &matchSuccess, 0, 0, 0, "Copying match success 2");
                  if (matchSuccess == 1) {
                     cout << "Allignment check succeeded " << numConstraints << endl;
                     lastFullLoopIndex = numConstraints;
                     opencl_task->setArg(3, 3, sizeof(int), &currentLocalMap);
                     opencl_task->setArg(4, 3, sizeof(int), &numConstraints);
                     opencl_task->setArg(2, 8, sizeof(int), &numConstraints);
                     runSize = getGlobalWorkSize(
                              numLocalMapPoints[potentialMatches[i]]);
                     opencl_task->queueKernel(3, 1, runSize, LocalSize,
                                    0, NULL, NULL, false);
                     opencl_task->queueKernel(8, 1, 32, 0,
                                    0, NULL, NULL, false);

                     numConstraints++;
                     numLoopConstraints++;
                     needOptimisation = true;
                     loopClosed = true;
                     parentLocalMap = potentialMatches[i];

                     // *** Debugging
                     readBuffer(clSlamCommon, CL_TRUE, numPotentialMatchesOffset 
                            + sizeof(int) * (MAX_POTENTIAL_MATCHES + 1), 
                            sizeof(ocl_float) * (MAX_POTENTIAL_MATCHES*3),
                           &temp, 0, 0, 0, "Copying icp alignment");
                     int r;
                     cout << "Aligned offsets are: " << endl;
                     for(r = 1; r <= potentialMatches[0]; r++) {
                        cout << potentialMatches[r] << " " << temp[r-1] << " " << 
                            temp[MAX_POTENTIAL_MATCHES + r-1] << " " << 
                            temp[MAX_POTENTIAL_MATCHES * 2 + r - 1] << endl;
                     } 
                     // End debugging
                     //Only add one loop closing constraint
                     break;
                  } else {
                     cout << "Alignment failed due to not enough empty map matching" << endl;
                  }
               } else {
                  cout << "Alignment failed" << endl;
               }
            }
         }
         if (!loopClosed && UseTempLoopClosures) {
            bool matchMade = findTempMatches();
            if (matchMade) {
               cout << "Found a temporary loop closing match" << endl;
               needOptimisation = true;
            }
         }
      }
      if (needOptimisation) {
         cout << "Optimising graph" << endl;
         ocl_float4 posBefore;
         size_t off;
         if (loopClosed) {
            off = parentLocalMap * sizeof(slamLocalMap);
         } else {
            off = currentLocalMap * sizeof(slamLocalMap);
         }
         readBuffer(clSlamLocalMap, CL_TRUE, off, sizeof(ocl_float4),
               &posBefore, 0, 0, 0, "Copying old global position");
         cout << "pos before: " << posBefore.x << " " << posBefore.y << " " << posBefore.w << endl;
         opencl_task->setArg(3, 9, sizeof(int), &nextLocalMap);
         opencl_task->setArg(4, 10, sizeof(int), &nextLocalMap);
         opencl_task->setArg(3, 11, sizeof(int), &nextLocalMap);
         opencl_task->setArg(3, 12, sizeof(int), &nextLocalMap);
         opencl_task->setArg(6, 10, sizeof(int), &lastFullLoopIndex);

         int optType = 1;
         if (!loopClosed) {
            optType = -1;
         }
         opencl_task->setArg(5, 10, sizeof(int), &optType);
         
         int kernelSize;

         //Now actually optimise the map
         for (int numIterations = 1; numIterations < NumOfOptimisationIts * 2; 
               numIterations++) {
            opencl_task->setArg(3, 10, sizeof(int), &numIterations);
            kernelSize = getGlobalWorkSize(numConstraints * 32);
            opencl_task->queueKernel(9, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
            if (numIterations == NumOfOptimisationIts && loopClosed) {
               optType = 0;
               opencl_task->setArg(5, 10, sizeof(int), &optType);
               int kSize = getGlobalWorkSize(numLoopConstraints);
               opencl_task->queueKernel(18, 1, kSize, LocalSize, 0,
                     NULL, NULL, false);
            }
            opencl_task->queueKernel(10, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
            kernelSize = getGlobalWorkSize(nextLocalMap);
            opencl_task->queueKernel(11, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
               /*float outP[100];
               readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outP, 0, 0, 0, "");
               int k;
               for (k = 0; k < nextLocalMap; k++) {
                  cout << "map " << k << " moved " << outP[k] << endl;
               }*/
         }
         opencl_task->setArg(3, 19, sizeof(int), &nextLocalMap);
         kernelSize = getGlobalWorkSize(nextLocalMap);
         opencl_task->queueKernel(19, 1, kernelSize, LocalSize,
               0, NULL, NULL, false);
         
         bool foundMoreLoops = false;
         float *oldMapPositions = (float *)malloc(sizeof(ocl_float) * nextLocalMap * 5);
         memcpy(oldMapPositions, globalMapPositions, sizeof(ocl_float) * nextLocalMap * 5);
         readBuffer(clGlobalMapPositions, CL_TRUE, 0, sizeof(ocl_float) * nextLocalMap * 5,
                   globalMapPositions, 0, 0, 0, "Copying global map positions");
         for(int k = 0; loopClosed && UseTempLoopClosures && k < nextLocalMap; k++) {
            cout << "Map " << k << ": old: " << oldMapPositions[k*5] << " " <<
               oldMapPositions[k*5 + 1] << " " << oldMapPositions[k*5 + 2] << " new "
               << globalMapPositions[k*5] << " " <<
               globalMapPositions[k*5 + 1] << " " << globalMapPositions[k*5 + 2] << endl;
            if (fabs(globalMapPositions[k * 5] - oldMapPositions[k * 5]) > 
                     LargeMovementThreshold ||
                fabs(globalMapPositions[k * 5 + 1] - oldMapPositions[k * 5 + 1]) >
                     LargeMovementThreshold ||
                fabs(globalMapPositions[k * 5 + 2] - oldMapPositions[k * 5 + 2]) >
                     LargeMovementThreshold) {

               cout << "Map " << k << " moved a lot" << endl;
               foundMoreLoops = findChangedPosMatches(k) || foundMoreLoops;

            }
         }

         if (foundMoreLoops) {
            cout << "****Optimising again" << endl;
            optType = 0;
            opencl_task->setArg(5, 10, sizeof(int), &optType);
            for (int numIterations = 1; numIterations < NumOfOptimisationIts; 
                  numIterations++) {
               opencl_task->setArg(3, 10, sizeof(int), &numIterations);
               kernelSize = getGlobalWorkSize(numConstraints * 32);
               opencl_task->queueKernel(9, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
               opencl_task->queueKernel(10, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
               kernelSize = getGlobalWorkSize(nextLocalMap);
               opencl_task->queueKernel(11, 1, kernelSize, LocalSize,
                                 0, NULL, NULL, false);
            }
            opencl_task->setArg(3, 19, sizeof(int), &nextLocalMap);
            kernelSize = getGlobalWorkSize(nextLocalMap);
            opencl_task->queueKernel(19, 1, kernelSize, LocalSize,
                  0, NULL, NULL, false);
            readBuffer(clGlobalMapPositions, CL_TRUE, 0, sizeof(ocl_float) * nextLocalMap * 5,
                   globalMapPositions, 0, 0, 0, "Copying global map positions 2");
         }

         if (LocalMapWarp) {
            warpLocalMaps();
            if (numGlobalPoints + MAX_LOCAL_POINTS > totalGlobalPoints) {
               createExtraGlobalPointsSpace();
            }
         }

         vector<LocalMapInfoPtr> changes;
         for(int k = 0; k < nextLocalMap; k++) {
            if (oldMapPositions[k*5] != globalMapPositions[k*5] ||
                  oldMapPositions[k*5 + 1] != globalMapPositions[k*5 + 1] ||
                  oldMapPositions[k*5 + 2] != globalMapPositions[k*5 + 2]) {
               double ys, ps, rs;
               localMaps[k].globalPose.getYPR(ys, ps, rs);
               localMaps[k].globalPose.position.x = globalMapPositions[k*5];
               localMaps[k].globalPose.position.y = globalMapPositions[k*5 + 1];
               ys = globalMapPositions[k*5 + 2];
               localMaps[k].globalPose.setYPR(ys, ps, rs);

               //LocalMapInfo temp(localMaps[k].globalPose, k);
               changes.push_back(new LocalMapInfo(localMaps[k].globalPose, k));
            }
         }
         vector<int> iNodes;
         vector<int> jNodes;
         //TODO: fill out the iNodes and jNodes arrays
         graphSlamNode->publishOptimiseLocalMapInfo(changes, iNodes, jNodes, loopClosed);
         free(oldMapPositions);
        
         
         kernelSize = getGlobalWorkSize(maxNumLocalPoints);
         opencl_task->queueKernel(12, 1, kernelSize, LocalSize,
               0, NULL, NULL, false);

         resetMap = true;
         lastCloudPublished = 0;
         historySlamPoses.resize(0);

         int mapIndex;
         if (loopClosed) {
            mapIndex = parentLocalMap;
            readBuffer(clSlamCommon, CL_TRUE, 0, sizeof(ocl_float4), &currentOffset, 
               0, 0, 0, "Copying new current offset");
         } else {
            mapIndex = currentLocalMap;
         }
         double posChangeTh = globalMapPositions[mapIndex * 5 + 2] + currentOffset.w;
         double cosTh = cos(globalMapPositions[mapIndex * 5 + 2]);
         double sinTh = sin(globalMapPositions[mapIndex * 5 + 2]);
         double posChangeX = currentOffset.x * cosTh - currentOffset.y * sinTh + 
            globalMapPositions[mapIndex * 5];
         double posChangeY = currentOffset.x * sinTh + currentOffset.y * cosTh +
            globalMapPositions[mapIndex * 5 + 1];

         cout << "map index: " << mapIndex << " " << globalMapPositions[mapIndex * 5] << " " 
            << globalMapPositions[mapIndex * 5 + 1] << " " << globalMapPositions[mapIndex * 5 + 2] << endl;

         double ys, ps, rs;
         slamPose.getYPR(ys, ps, rs);

         cout << "old pose: " << slamPose.position.x << " " << slamPose.position.y << " " 
            << ys << endl;
         cout << "new pose: " << posChangeX << " " << posChangeY << " " << posChangeTh << endl;


         slamPose.position.x = posChangeX;
         slamPose.position.y = posChangeY;
         ys = posChangeTh;
         ANGNORM(ys);
         slamPose.setYPR(ys, ps, rs);

         if (LocalMapCombine) {
            combineMode++;
         }
      }
   } else {
      opencl_task->setArg(3, 4, sizeof(int), &currentLocalMap);
      opencl_task->setArg(4, 4, sizeof(int), &nextLocalMap);
      opencl_task->setArg(5, 4, sizeof(int), &numScans);
      opencl_task->setArg(4, 15, sizeof(int), &currentLocalMap);
      opencl_task->setArg(5, 15, sizeof(int), &numGlobalPoints);
      //setLocalMap and prepareLocalMap
      int globalSize = getGlobalWorkSize(numLocalMapPoints[currentLocalMap]);
      opencl_task->queueKernel(15, 1, globalSize, LocalSize, 
                                 0, NULL, NULL, false);
      //Make sure the histograms are finalised for the first map
      opencl_task->queueKernel(4, 1, NUM_ORIENTATION_BINS, 0, 
                                 0, NULL, NULL, false);
      parentLocalMap = currentLocalMap;
      nextLocalMap++;
   }

   float alignError = angleError;
   opencl_task->setArg(4, 2, sizeof(int), &oldLocalMap);
   opencl_task->setArg(5, 2, sizeof(int), &nextLocalMap);
   opencl_task->setArg(6, 2, sizeof(int), &parentLocalMap);
   opencl_task->setArg(7, 2, sizeof(ocl_float), &alignError);
   opencl_task->setArg(8, 2, sizeof(int), &numOldMapPoints);
   int globalSize = getGlobalWorkSize(numOldMapPoints);
   opencl_task->queueKernel(2, 1, globalSize, LocalSize, 
         0, NULL, NULL, false);

   if (nextLocalMap == localMaps.size()) {
      LocalMap temp;
      temp.numWarpPoints = 0;
      temp.indexNextNode = -1;
      temp.globalPose = slamPose;
      localMaps.push_back(temp);
   }
   double cosTh = cos(-startICPTheta);
   double sinTh = sin(-startICPTheta);

   localMaps[oldLocalMap].indexNextNode = nextLocalMap;
   localMaps[oldLocalMap].nextOffsetX = cosTh * offsetFromParentX - sinTh * offsetFromParentY;
   localMaps[oldLocalMap].nextOffsetY = sinTh * offsetFromParentX + cosTh * offsetFromParentY;
   localMaps[oldLocalMap].nextOffsetTh = offsetFromParentTh;

   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   double yi, pi, ri;
   icpPose.getYPR(yi, pi, ri);
   startICPTheta = yi;
   currentOffset.x = 0;
   currentOffset.y = 0;
   currentOffset.w = 0;
   indexParentNode.push_back(parentLocalMap);
   currentLocalMapICPPose = icpPose;
   numGlobalPoints += numLocalPoints;
   //parentLocalMap = currentLocalMap;
   currentLocalMap = nextLocalMap;
   globalMapPositions[currentLocalMap * 5] = slamPose.position.x;
   globalMapPositions[currentLocalMap * 5 + 1] = slamPose.position.y;
   double ys, ps, rs;
   slamPose.getYPR(ys, ps, rs);
   globalMapPositions[currentLocalMap * 5 + 2] = ys;

   LocalMapInfo newMap(slamPose, currentLocalMap);
   graphSlamNode->publishLocalMapInfo(newMap);

   //Grow data structures if needed
   if (numGlobalPoints + MAX_LOCAL_POINTS > totalGlobalPoints) {
      createExtraGlobalPointsSpace();
   }
   if (nextLocalMap + 2 >= totalLocalMaps) {
      cout << "^^^^^^^^^^^^^^^^^^^^^Run of out space to store local maps" << endl;
      cout << sizeof(ocl_float) << " " << sizeof(float) << endl;
      int increment = 20;
      //slamLocalMap maps[totalLocalMaps + increment];
      slamLocalMap *maps = new slamLocalMap[totalLocalMaps + increment];

      readBuffer(clSlamLocalMap, CL_TRUE, 0, sizeof(slamLocalMap) * 
               totalLocalMaps, maps, 0, 0, 0, "Copying local map information");
      ocl_float *tempFreeArea = new ocl_float[totalLocalMaps * localOGSize];
      readBuffer(clFreeAreas, CL_TRUE, 0, sizeof(ocl_float) * totalLocalMaps * localOGSize,
            tempFreeArea, 0, 0, 0, "Copying free areas info to main memory");

      opencl_manager->deviceRelease(clSlamLocalMap);
      opencl_manager->deviceRelease(clGlobalMapPositions);
      opencl_manager->deviceRelease(clFreeAreas);

      int *temp = new int[totalLocalMaps + increment];
      int i;
      for(i = 0; i < totalLocalMaps; i++) {
         temp[i] = numLocalMapPoints[i];
      }
      delete [] numLocalMapPoints;
      numLocalMapPoints = temp;

      ocl_float *tempF = new ocl_float[(totalLocalMaps + increment) * 5];
      for(i = 0; i < totalLocalMaps * 5; i++) {
         tempF[i] = globalMapPositions[i];
      }
      delete [] globalMapPositions;
      globalMapPositions = tempF;
         
      totalLocalMaps += increment;

      size_t size = sizeof(slamLocalMap) * totalLocalMaps;
      clSlamLocalMap = opencl_manager->deviceAlloc(size,
            CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, maps);
      size = totalLocalMaps * sizeof(ocl_float) * 5;
      clGlobalMapPositions = opencl_manager->deviceAlloc(size,
            CL_MEM_READ_WRITE, NULL);
      size = totalLocalMaps * sizeof(ocl_float) * localOGSize;
      clFreeAreas = opencl_manager->deviceAlloc(size,
            CL_MEM_READ_WRITE, NULL);
      writeBuffer(clFreeAreas, CL_TRUE, 0, (totalLocalMaps - increment) * 
            sizeof(ocl_float) * localOGSize, tempFreeArea, 0, 0, 0, 
            "Copying free areas to the gpu memory");
      delete [] tempFreeArea;
      delete [] maps;

      opencl_task->setArg(1, 1, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 2, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 3, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 4, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 5, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 6, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 7, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 9, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 10, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 11, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 12, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 13, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 14, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 15, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 16, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 17, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 18, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 19, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(1, 21, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(0, 22, sizeof(cl_mem), &clSlamLocalMap);
      opencl_task->setArg(4, 12, sizeof(cl_mem), &clGlobalMapPositions);
      opencl_task->setArg(2, 19, sizeof(cl_mem), &clGlobalMapPositions);

      opencl_task->setArg(5, 1, sizeof(cl_mem), &clFreeAreas);
      opencl_task->setArg(3, 2, sizeof(cl_mem), &clFreeAreas);
      opencl_task->setArg(3, 16, sizeof(cl_mem), &clFreeAreas);
   }

}}
}

bool GraphSlamGPU::findTempMatches() {
   if (isMapFeatureless[currentLocalMap]) {
      return false;
   }

   int mIndex = 0;

   float xCur = globalMapPositions[currentLocalMap * 5 + 3];
   float yCur = globalMapPositions[currentLocalMap * 5 + 4];

   bool returnVal = false;
   
   for (mIndex = 0; mIndex < nextLocalMap; mIndex++) {
      if (mIndex != currentLocalMap && indexParentNode[currentLocalMap] != mIndex &&
            !isMapFeatureless[mIndex] && 
            fabs(xCur - globalMapPositions[mIndex * 5 + 3]) < DistanceOverlapThreshold &&
            fabs(yCur - globalMapPositions[mIndex * 5 + 4]) < DistanceOverlapThreshold) {
         //cout << "Map: " << currentLocalMap << " p: " << 
         //   indexParentNode[currentLocalMap] << " m: " << mIndex << " " <<
         //   xCur << " " << yCur << " " << globalMapPositions[mIndex * 5 + 3] << " " <<
         //   globalMapPositions[mIndex * 5 + 4] << endl;
         if (performTempMatch(currentLocalMap, mIndex)) {
            returnVal = true;
         }
      }
   }
   return returnVal;
}

bool GraphSlamGPU::performTempMatch(int currentMap, int testMap) {

   int mode = -1; 
   opencl_task->setArg(4, 16, sizeof(int), &currentMap);
   opencl_task->setArg(5, 16, sizeof(int), &testMap);
   opencl_task->setArg(6, 16, sizeof(int), &mode);


   size_t dataSpace = sizeof(ocl_int) + sizeof(ocl_float);
   char *tempData = (char *)malloc(dataSpace);
   memset(tempData, 0, dataSpace);

   writeBuffer(clSlamCommon, CL_FALSE, evaluateOffset, dataSpace, tempData, 0, 0, 0,
         "Writing to reset temp match info");

   int globalSize = getGlobalWorkSize(numLocalMapPoints[testMap]);
   opencl_task->queueKernel(16, 1, globalSize, LocalSize,
           0, NULL, NULL, false);

   readBuffer(clSlamCommon, CL_TRUE, evaluateOffset, dataSpace, tempData, 0, 0, 0,
         "Reading evaluate map info");

   int numOverlap = *((int *)tempData);
   float score = *((float *)(tempData + sizeof(int)));
   free(tempData);

   cout << "Score for map " << currentMap << "(" << indexParentNode[currentMap] << ") to " << testMap << " is " << 
      score / (float) numOverlap << " with overlap " << numOverlap << endl;

   if (numOverlap > OverlapThreshold) {
      cout << "Trying to align possible matches" << endl;

      float previousScore = score / (float) numOverlap;
      //TODO: previousScore - do I need to set it somewhere here or in the evaluateKernel?
      opencl_task->setArg(3, 6, sizeof(int), &testMap);
      opencl_task->setArg(4, 6, sizeof(int), &currentMap);
      opencl_task->setArg(5, 6, sizeof(int), &(numLocalMapPoints[testMap]));
      int mIndex = 0;
      opencl_task->setArg(6, 6, sizeof(int), &mIndex);
      opencl_task->setArg(3, 7, sizeof(int), &currentMap);
      opencl_task->setArg(4, 7, sizeof(int), &mIndex);
      int fullLoop = 0;
      opencl_task->setArg(5, 7, sizeof(int), &fullLoop);


      int matchSuccess = 0;
      int c = 0;
      while (matchSuccess == 0) {
         opencl_task->queueKernel(6, 1, globalSize, LocalSize,
                           0, NULL, NULL, false);
         opencl_task->queueKernel(7, 1, 32, 0,
                           0, NULL, NULL, false);
         readBuffer(clSlamCommon, CL_TRUE, matchSuccessOffset, sizeof(ocl_int), 
                    &matchSuccess, 0, 0, 0, "Copying match success");
         c++;
      }
      if (matchSuccess == 1) {
         cout << "Temp match alignment success " << c << endl;
         mode = 0;
         opencl_task->setArg(6, 16, sizeof(int), &mode);
         opencl_task->queueKernel(16, 1, globalSize, LocalSize,
            0, NULL, NULL, false);
         opencl_task->setArg(3, 17, sizeof(int), &currentMap);
         opencl_task->setArg(4, 17, sizeof(int), &mIndex);
         opencl_task->setArg(5, 17, sizeof(int), &fullLoop);
         opencl_task->setArg(6, 17, sizeof(float), &previousScore);
         opencl_task->queueKernel(17, 1, 32, 0,
            0, NULL, NULL, false);

         readBuffer(clSlamCommon, CL_TRUE, matchSuccessOffset, sizeof(ocl_int), 
                    &matchSuccess, 0, 0, 0, "Copying match success 2");

         float outP[100];
         readBuffer(clRes, CL_TRUE, 0, sizeof(float) * 100, outP, 0, 0, 0, "");
         cout << "Values from map matching are: " << outP[0] << " " <<
                     outP[1] << " " << outP[2] << endl;


         if (matchSuccess == 1) {

            cout << "Temp match loop closed!! " << currentMap << " with " << testMap << endl;

            opencl_task->setArg(3, 3, sizeof(int), &currentMap);
            opencl_task->setArg(4, 3, sizeof(int), &numConstraints);
            opencl_task->setArg(2, 8, sizeof(int), &numConstraints);
            int runSize = getGlobalWorkSize(numLocalMapPoints[testMap]);
            opencl_task->queueKernel(3, 1, runSize, LocalSize,
                       0, NULL, NULL, false);
            opencl_task->queueKernel(8, 1, 32, 0,
                       0, NULL, NULL, false);
            numConstraints++;
            numLoopConstraints++;
            return true;
         } else {
            cout << "Temp match loop closure not made" << endl;
         }
      } else {
         cout << "temp match alignment failed " << c << endl;
      }
   }   
   return false;
}

bool GraphSlamGPU::findChangedPosMatches(int mapNum) {
   if (isMapFeatureless[mapNum]) {
      return false;
   }

   int mIndex = 0;
   int xCur = globalMapPositions[mapNum * 5 + 3];
   int yCur = globalMapPositions[mapNum * 5 + 4];

   bool returnVal = false;
   bool entered = false;
   for (mIndex = 0; mIndex < mapNum; mIndex++) {
      if (mIndex != currentLocalMap && indexParentNode[mapNum] != mIndex &&
            !isMapFeatureless[mIndex] && 
            fabs(xCur - globalMapPositions[mIndex * 5 + 3]) < DistanceOverlapThreshold &&
            fabs(yCur - globalMapPositions[mIndex * 5 + 4]) < DistanceOverlapThreshold) {

         if (!entered) {
            //Reset the occupancy grid cells
            int globalSize = getGlobalWorkSize(numLocalMapPoints[mapNum]);
            opencl_task->setArg(3, 21, sizeof(int), &mapNum);
            opencl_task->queueKernel(20, 1, globalSize, LocalSize,
                          0, NULL, NULL, false);
            opencl_task->queueKernel(21, 1, globalSize, LocalSize,
                          0, NULL, NULL, false);
            entered = true;
         }

         if (performTempMatch(mapNum, mIndex)) {
            returnVal = true;
            cout << "****Found a match by going back through map. Maps: " << 
               mapNum << " to " << mIndex << endl;
         }

      }
   }
   if (returnVal) {
      lastFullLoopIndex = numConstraints - 1;
   }

   return returnVal;
}

void GraphSlamGPU::warpLocalMaps() {
   int i,j;
   for (i = 0; i < nextLocalMap; i++) {
      j = localMaps[i].indexNextNode;
      if (j > -1) {
         double jX, jY, jTh, iX, iY, iTh;
         getGlobalMapPosition(j, jX, jY, jTh);
         getGlobalMapPosition(i, iX, iY, iTh);
         double errX = jX - iX;
         double errY = jY - iY;
         double diffTh = jTh - iTh;
         ANGNORM(diffTh);

         //transform global diffs to be relative to the local map
         double cosTh = cos(- iTh);
         double sinTh = sin(- iTh);
         double diffX = cosTh * errX - sinTh * errY;
         double diffY = sinTh * errX + cosTh * errY;

         errX = diffX - localMaps[i].nextOffsetX;
         errY = diffY - localMaps[i].nextOffsetY;
         double errTh = diffTh - localMaps[i].nextOffsetTh;
         ANGNORM(errTh);

         //cout << errX << " " << diffX << " " << localMaps[i].nextOffsetX << " " << iX << " " << jX << " " << i << " " << j << endl;

         diffX = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[0] - errX;
         diffY = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[1] - errY;
         diffTh = localMaps[i].scans[localMaps[i].scans.size() - 1]->correction[2] - errTh;
         ANGNORM(diffTh);

         //cout << "**Looking at map: " << i << ": " << diffX << " " << diffY << " " 
         //<< diffTh << ": " << errX << " " << errY << " " << errTh << endl; 

         if (fabs(diffX) > LocalMapWarpThreshXY || fabs(diffY) > LocalMapWarpThreshXY || 
               fabs(diffTh) > LocalMapWarpThreshTh) {
            cout << "Warping map: " << i << " " << diffX << " " << diffY <<  " " << diffTh << endl;
            warpLocalMap(i, errX, errY, errTh);
         }
      }
   }
}

void GraphSlamGPU::warpLocalMap(int mapIndex, double errX, double errY, double errTh) {
   int i, j, k;
   //Warp each scan
   double tempCovar[3][3];
   double den[3];
   for (i = 0; i < 3; i++) {
      for (j = 0; j < 3; j++) {
         tempCovar[i][j] = 0;
      }
      den[i] = 0;
      for (j = 0; j < localMaps[mapIndex].scans.size(); j++) {
         den[i] += localMaps[mapIndex].scans[j]->covar[i][i];
      }
   }
      
   double err[3];
   err[0] = errX;
   err[1] = errY;
   err[2] = errTh;
   double prevFrac[3] = {0, 0, 0};
   for (i = 0; i < localMaps[mapIndex].scans.size(); i++) {
      for (j = 0; j < 3; j++) {
         for (k = 0; k < 3; k++) {
            tempCovar[j][k] += localMaps[mapIndex].scans[i]->covar[j][k];
         }
      }
      for (j = 0; j < 3; j++) {
         /*double num = errX * tempCovar[j][0] + errY * tempCovar[j][1] +
            errTh * tempCovar[j][2];*/
         double num = tempCovar[j][j];
         localMaps[mapIndex].scans[i]->correction[j] = fabs(num / den[j]) * err[j];
         if (fabs(num / den[j]) < prevFrac[j]) {
            cout << "^^^^^^^^^ " << j << " " << num << " " << den[j] << " " << mapIndex << endl;
         }
         prevFrac[j] = fabs(num / den[j]);
      }
   }
   int tt = localMaps[mapIndex].scans.size() - 1;
   cout << "Correction of final scan is: " << localMaps[mapIndex].scans[tt]->correction[0] << " " <<
      localMaps[mapIndex].scans[tt]->correction[1] << " " << localMaps[mapIndex].scans[tt]->correction[2] << endl;
   cout << "Num warp points before is: " << localMaps[mapIndex].numWarpPoints;
   //Now actually warp the points
   
   //Method 1: warp the already selected points
   /*for (i = 0; i < localMaps[mapIndex].numWarpPoints; i++) {
      int scanIndex = localMaps[mapIndex].lastObserved[i];
      double tempX = localMaps[mapIndex].pointsX[i] - localMaps[mapIndex].scans[scanIndex]->pose[0];
      double tempY = localMaps[mapIndex].pointsY[i] - localMaps[mapIndex].scans[scanIndex]->pose[1];
      double cosTh = cos(-localMaps[mapIndex].scans[scanIndex]->pose[2]);
      double sinTh = sin(-localMaps[mapIndex].scans[scanIndex]->pose[2]);
      double relX = cosTh * tempX - sinTh * tempY;
      double relY = sinTh * tempX + cosTh * tempY;
      cosTh = cos(localMaps[mapIndex].scans[scanIndex]->pose[2] + localMaps[mapIndex].scans[scanIndex]->correction[2]);
      sinTh = sin(localMaps[mapIndex].scans[scanIndex]->pose[2] + localMaps[mapIndex].scans[scanIndex]->correction[2]);
      localMaps[mapIndex].warpPointsX[i] = cosTh * relX - sinTh * relY
         + localMaps[mapIndex].scans[scanIndex]->pose[0] + localMaps[mapIndex].scans[scanIndex]->correction[0];
      localMaps[mapIndex].warpPointsY[i] = sinTh * relX + cosTh * relY
         + localMaps[mapIndex].scans[scanIndex]->pose[1] + localMaps[mapIndex].scans[scanIndex]->correction[1];
   }*/

   //Method 2: Warp all the points
   ////TODO: note this doesn't handle Z properly
   double z = (MinAddHeight + MaxAddHeight) / 2.0;
   int *ogMap = new int[DimLocalOG * DimLocalOG];
   memset(ogMap, 0, sizeof(int) * DimLocalOG * DimLocalOG);

   int numWarpPoints = 0;
   float newWarpPoints[MAX_LOCAL_POINTS * 3];


   for (i = 0; i < localMaps[mapIndex].scans.size(); i++) {
      double posX = localMaps[mapIndex].scans[i]->pose[0] + localMaps[mapIndex].scans[i]->correction[0];
      double posY = localMaps[mapIndex].scans[i]->pose[1] + localMaps[mapIndex].scans[i]->correction[1];
      double posTh = localMaps[mapIndex].scans[i]->pose[2] + localMaps[mapIndex].scans[i]->correction[2];
      ANGNORM(posTh);
      double cosTh = cos(posTh);
      double sinTh = sin(posTh);
      for (j = 0; j < localMaps[mapIndex].scans[i]->points.size(); j++) {
         double tempX = localMaps[mapIndex].scans[i]->points[j].x;
         double tempY = localMaps[mapIndex].scans[i]->points[j].y;
         double x = tempX * cosTh - tempY * sinTh + posX;
         double y = tempX * sinTh + tempY * cosTh + posY;
         
         double off = (DimLocalOG * CellSize) / 2.0;
         int xx = (x + off) / CellSize;
         int yy = (y + off) / CellSize;
         if (xx >= 0 && xx < DimLocalOG && yy >= 0 && yy < DimLocalOG) {
            int index = yy * DimLocalOG + xx;
            ogMap[index]++;
            if (ogMap[index] == MinObservationCount && numWarpPoints < MAX_LOCAL_POINTS) {
            //if (ogMap[index] == MinObservationCount && localMaps[mapIndex].numWarpPoints < localMaps[mapIndex].numPoints) {
               newWarpPoints[numWarpPoints] = (float)x;
               newWarpPoints[MAX_LOCAL_POINTS + numWarpPoints] = (float)y;
               newWarpPoints[MAX_LOCAL_POINTS * 2 + numWarpPoints] = (float)z;
               numWarpPoints++;
            }
         }
      }
   }
   cout << " and after is: " << numWarpPoints << endl;
   delete[] ogMap;
  
   //Now have warped to map, so copy the points back to the GPU
   size_t size = sizeof(ocl_float) * MAX_LOCAL_POINTS * 3;
   cl_mem clPointsStore = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
    
   writeBuffer(clPointsStore, CL_FALSE, 0, size, newWarpPoints, 0, 0, 0,
         "Copying new warp points to GPU");
   opencl_task->setArg(1, 22, sizeof(cl_mem), &clPointsStore);
   opencl_task->setArg(2, 22, sizeof(int), &mapIndex);
   opencl_task->setArg(3, 22, sizeof(int), &numWarpPoints);

   int kernelSize = getGlobalWorkSize(numWarpPoints);
   opencl_task->queueKernel(22, 1, kernelSize, LocalSize,
           0, NULL, NULL, false);
   
   opencl_manager->deviceRelease(clPointsStore);
   
   if (numWarpPoints > maxNumLocalPoints) {
      maxNumLocalPoints = numWarpPoints;
   }
   numGlobalPoints += numWarpPoints - localMaps[mapIndex].numWarpPoints;
   localMaps[mapIndex].numWarpPoints = numWarpPoints; 
}

inline int GraphSlamGPU::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
}

void GraphSlamGPU::createExtraGlobalPointsSpace() {
   cout << "Run out of space to store global points" << endl;
   int increment = 10000;
   if (numGlobalPoints > totalGlobalPoints) {
      increment += numGlobalPoints - totalGlobalPoints;
   }
   delete []globalMap;
   delete []globalMapHeights;
   globalMap = new int[totalGlobalPoints + increment];
   globalMapHeights = new float[totalGlobalPoints + increment];
   readBuffer(clGlobalMap, CL_TRUE, 0, sizeof(ocl_int) * totalGlobalPoints,
             globalMap, 0, 0, 0, "Copying number of local points");
   readBuffer(clGlobalMapHeights, CL_TRUE, 0, sizeof(ocl_float) * totalGlobalPoints,
             globalMapHeights, 0, 0, 0, "Copying global map heights");
   opencl_manager->deviceRelease(clGlobalMap);
   opencl_manager->deviceRelease(clGlobalMapHeights);
   totalGlobalPoints+= increment;
   size_t size = sizeof(ocl_int) * totalGlobalPoints;
   clGlobalMap = opencl_manager->deviceAlloc(size, 
         CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, globalMap);
   size = sizeof(ocl_float) * totalGlobalPoints;
   clGlobalMapHeights = opencl_manager->deviceAlloc(size, 
         CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, globalMapHeights);

   opencl_task->setArg(3, 1, sizeof(cl_mem), &clGlobalMap);
   opencl_task->setArg(2, 12, sizeof(cl_mem), &clGlobalMap);
   opencl_task->setArg(3, 13, sizeof(cl_mem), &clGlobalMap);
   opencl_task->setArg(4, 1, sizeof(cl_mem), &clGlobalMapHeights);
   opencl_task->setArg(5, 12, sizeof(cl_mem), &clGlobalMapHeights);
   opencl_task->setArg(9, 13, sizeof(cl_mem), &clGlobalMapHeights);
   opencl_task->setArg(3, 15, sizeof(cl_mem), &clGlobalMapHeights);

}

void GraphSlamGPU::writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
      size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), buffer, blocking_write,
        offset, cb, ptr, num_events_in_wait_list , event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error write buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void GraphSlamGPU::readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
      size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), buffer, blocking_read,
         offset, cb, ptr, num_events_in_wait_list, event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error read buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void GraphSlamGPU::getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices) {

   int numLocalPoints;
   size_t offset = currentLocalMap * sizeof(slamLocalMap) +
                   sizeof(ocl_float4) * 2;
   readBuffer(clSlamLocalMap, CL_TRUE, offset, sizeof(ocl_int), &numLocalPoints, 0, 0, 0,
         "Copying number of local points for map drawing");

   int x, y, mapNum;
   crosbot::LocalMap::Cell *cellsP;
   if (resetMap) {
      resetMap = false;
      cout << "Reseting map" << endl;
      lastDrawnGlobalPoints = 0;
      for(mapNum = 0; mapNum < curMap.size(); mapNum++) {
         for(y = 0; y < curMap[mapNum]->height; y++) {
            cellsP = &(curMap[mapNum]->cells[y][0]);
            for(x = 0; x< curMap[mapNum]->width; x++) {
               cellsP->current = false;
               cellsP->hits = 0;
               cellsP++;
            }
         }
      }
   }
   //cout << numLocalPoints << " " << numGlobalPoints << " " << lastDrawnGlobalPoints << endl;
   if (numGlobalPoints + numLocalPoints > lastDrawnGlobalPoints) {
      size_t size = sizeof(ocl_int) * (numGlobalPoints + numLocalPoints - 
         lastDrawnGlobalPoints);
      readBuffer(clGlobalMap, CL_TRUE, sizeof(ocl_int) * lastDrawnGlobalPoints, size,
            &(globalMap[lastDrawnGlobalPoints]), 0, 0, 0, "Copy of map drawing info");
      readBuffer(clGlobalMapHeights, CL_TRUE, sizeof(ocl_int) * lastDrawnGlobalPoints, 
            size, &(globalMapHeights[lastDrawnGlobalPoints]), 0, 0, 0,
            "Copy of map height drawing info");

      double off = (DimGlobalOG * CellSize) / 2.0 
                 - CellSize / 2.0;
      x = numGlobalPoints - MAX_LOCAL_POINTS > lastDrawnGlobalPoints ?
           lastDrawnGlobalPoints : numGlobalPoints - MAX_LOCAL_POINTS;
      if (x < 0) {
         x = 0;
      }
      for(; x < numGlobalPoints + numLocalPoints; x++) {
         int index = globalMap[x];
         int yi = index / DimGlobalOG;
         int xi = index % DimGlobalOG;
         for(mapNum = 0; mapNum < curMap.size(); mapNum++) {
            cellsP = &(curMap[mapNum]->cells[yi][xi]);
            if (globalMapHeights[x] >= mapSlices[mapNum]) {
               if (x < numGlobalPoints) {
                  cellsP->current = false;
                  cellsP->hits = curMap[mapNum]->maxHits;
               } else {
                  cellsP->current = true;
                  cellsP->hits = curMap[mapNum]->maxHits;
               }
            }
         }
      }
      lastDrawnGlobalPoints = numGlobalPoints + numLocalPoints;
   }
}

void GraphSlamGPU::getGlobalMapPosition(int mapIndex, double& gx, double& gy,
      double& gth) {
   gx = globalMapPositions[mapIndex * 5];
   gy = globalMapPositions[mapIndex * 5 + 1];
   gth = globalMapPositions[mapIndex * 5 + 2];
}

int GraphSlamGPU::getScanIndex(int mapIndex) {
   return localMaps[mapIndex].scans.size() - 1;
}

void GraphSlamGPU::getScanPose(int mapIndex, int scanIndex, double& px, double& py, double& pth) {
   px = localMaps[mapIndex].scans[scanIndex]->pose[0] + 
      localMaps[mapIndex].scans[scanIndex]->correction[0];
   py = localMaps[mapIndex].scans[scanIndex]->pose[1] + 
      localMaps[mapIndex].scans[scanIndex]->correction[1];
   pth = localMaps[mapIndex].scans[scanIndex]->pose[2] + 
      localMaps[mapIndex].scans[scanIndex]->correction[2];
}

