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
   "add3DToMap", //14
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
   maxNumLocalPoints = 0;

   totalLocalMaps = 100;
   totalGlobalPoints = 10000;

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

   clSlamConfig = opencl_manager->deviceAlloc(sizeof(slamConfig), 
          CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &slam_config);

   localOGSize = DimLocalOG * DimLocalOG;

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
}

void GraphSlamGPU::initialisePoints() {
   points = new oclLaserPoints;
   points->pointX = new ocl_float[numLaserPoints * 6];
   points->pointY = &(points->pointX[numLaserPoints]);
   points->pointZ = &(points->pointX[numLaserPoints * 2]);
   points->pointNextX = &(points->pointX[numLaserPoints * 3]);
   points->pointNextY = &(points->pointX[numLaserPoints * 4]);
   points->pointNextZ = &(points->pointX[numLaserPoints * 5]);
   arrayPointsSize = sizeof(ocl_float) * numLaserPoints * 6;
   clPoints = opencl_manager->deviceAlloc(arrayPointsSize, CL_MEM_READ_WRITE, NULL);
}

void GraphSlamGPU::initialiseSlamStructures() {
   
   int numFloats = localOGSize +
                   MAX_LOCAL_POINTS * 2 +
                   NUM_ORIENTATION_BINS * 2 +
                   MAX_POTENTIAL_MATCHES * 3 +
                   MaxNumLoopConstraints * 12 +
                   (MaxNumConstraints + 1) * 3 +
                   15;
   int numInts = localOGSize * 2 + 
                 MAX_POTENTIAL_MATCHES * 2 + 
                 MaxNumConstraints * 2 +
                 MaxNumLoopConstraints * 5 + 9;
   size_t size = sizeof(ocl_float) * numFloats +
                 sizeof(ocl_int) * numInts +
                 sizeof(ocl_float4) +
                 sizeof(ocl_float2) * 2;
   clSlamCommon = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);

   size = sizeof(slamLocalMap) * totalLocalMaps;
   clSlamLocalMap = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);

   size = sizeof(ocl_int) * totalGlobalPoints;
   clGlobalMap = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMap = new int[totalGlobalPoints];
   numLocalMapPoints = new int[totalLocalMaps];

   size = sizeof(ocl_float) * totalGlobalPoints;
   clGlobalMapHeights = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMapHeights = new float[totalGlobalPoints];

   size = totalLocalMaps * sizeof(ocl_float) * 3;
   clGlobalMapPositions = opencl_manager->deviceAlloc(size, CL_MEM_READ_WRITE, NULL);
   globalMapPositions = new ocl_float[totalLocalMaps * 3];
   globalMapPositions[0] = 0;
   globalMapPositions[1] = 0;
   globalMapPositions[2] = 0;

   opencl_task->setArg(0, 0, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 0, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 0, sizeof(cl_mem), &clSlamCommon);
   opencl_task->queueKernel(0, 1, MAX_LOCAL_POINTS, 0, 0, NULL, NULL, false);

   clRes = opencl_manager->deviceAlloc(sizeof(ocl_float) * 1000, CL_MEM_READ_WRITE, NULL);
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
   //kernel 14: add3DToMap
   
   //kernel 1: addScanToMap
   opencl_task->setArg(0, 1, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 1, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 1, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 1, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(4, 1, sizeof(cl_mem), &clGlobalMap);
   opencl_task->setArg(5, 1, sizeof(cl_mem), &clGlobalMapHeights);
   //opencl_task->setArg(6, 1, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(7, 1, sizeof(int), &numPoints);
   //opencl_task->setArg(8, 1, sizeof(int), &numGlobalPoints);
   //opencl_task->setArg(9, 1, sizeof(ocl_float4), &currentoffset);
   opencl_task->setArg(10, 1, sizeof(cl_mem), &clRes);

   //kernel 2: createNewLocalMap 
   opencl_task->setArg(0, 2, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 2, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 2, sizeof(cl_mem), &clSlamCommon);
   //opencl_task->setArg(3, 2, sizeof(int), &oldLocalMap);
   //opencl_task->setArg(4, 2, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(5, 2, sizeof(int), &oldLocalMap);
   //opencl_task->setArg(6, 2, sizeof(ocl_float4), &offsetFromParent);
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
   opencl_task->setArg(5, 4, sizeof(cl_mem), &clRes);

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
   opencl_task->setArg(5, 7, sizeof(cl_mem), &clRes);

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
   opencl_task->setArg(5, 10, sizeof(cl_mem), &clRes);

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
   
   //kernel 14: add3DToMap
   opencl_task->setArg(0, 14, sizeof(cl_mem), &clSlamConfig);
   opencl_task->setArg(1, 14, sizeof(cl_mem), &clSlamLocalMap);
   opencl_task->setArg(2, 14, sizeof(cl_mem), &clSlamCommon);
   opencl_task->setArg(3, 14, sizeof(cl_mem), &clPoints);
   opencl_task->setArg(4, 14, sizeof(cl_mem), &clGlobalMapHeights);
   //opencl_task->setArg(5, 14, sizeof(int), &currentLocalMap);
   //opencl_task->setArg(6, 14, sizeof(int), &numPoints);
   //opencl_task->setArg(7, 14, sizeof(int), &numGlobalPoints);
}

void GraphSlamGPU::updateTrack(Pose icpPose, PointCloudPtr cloud) {

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

   //Add the points to the current local map
   int i, j;
   int numAdded = 0;
   for (i = 0; i < cloud->cloud.size() - 1; ++i) {
      double dist = cloud->cloud[i].x * cloud->cloud[i].x + cloud->cloud[i].y * cloud->cloud[i].y;
      if (dist > LaserMinDist * LaserMinDist && dist < LaserMaxDist * LaserMaxDist &&
         cloud->cloud[i].z + InitHeight > MinAddHeight && cloud->cloud[i].z + InitHeight < MaxAddHeight) {
         points->pointX[numAdded] = cloud->cloud[i].x;
         points->pointY[numAdded] = cloud->cloud[i].y;
         points->pointZ[numAdded] = cloud->cloud[i].z + InitHeight;
         points->pointNextX[numAdded] = cloud->cloud[i + 1].x;
         points->pointNextY[numAdded] = cloud->cloud[i + 1].y;
         points->pointNextZ[numAdded] = cloud->cloud[i + 1].z = InitHeight;
         numAdded++;
      }
   }

   int globalSize = getGlobalWorkSize(numAdded);

   opencl_task->setArg(6, 1, sizeof(int), &currentLocalMap);
   opencl_task->setArg(7, 1, sizeof(int), &numAdded);
   opencl_task->setArg(8, 1, sizeof(int), &numGlobalPoints);
   opencl_task->setArg(9, 1, sizeof(ocl_float4), &currentOffset);

   opencl_task->setArg(5, 14, sizeof(int), &currentLocalMap);
   opencl_task->setArg(6, 14, sizeof(int), &numAdded);
   opencl_task->setArg(7, 14, sizeof(int), &numGlobalPoints);

   writeBuffer(clPoints, CL_TRUE, 0, arrayPointsSize, points->pointX, 0, 0, 0,
         "Copying points to GPU");
   opencl_task->queueKernel(1, 1, globalSize, LocalSize, 0, NULL, NULL, false);
   opencl_task->queueKernel(14, 1, globalSize, LocalSize, 0, NULL, NULL, false);

   double temp = sqrt(offsetFromParentX * offsetFromParentX +
         offsetFromParentY * offsetFromParentY);
   if (temp >= LocalMapDistance) {
      cout << "Creating a new local map " << currentLocalMap << endl;
      finishMap(angleError, yi, icpPose);
   }

   oldICPPose = icpPose;

   ros::WallTime t2 = ros::WallTime::now();
   totalTime += t2 - t1;
   numIterations++;
   if (numIterations % 100 == 0) {
      cout << totalTime.toSec() * 1000.0f / (double) numIterations << "ms " << endl;
   }

   finishedSetup = true;
}

void GraphSlamGPU::finishMap(double angleError, double icpTh, Pose icpPose) {

{{ Lock lock(masterLockSmith);

   //the offsets in the data structures that are read
   size_t numPointsOffset = sizeof(ocl_float4) * 2;
   size_t matchSuccessOffset = sizeof(ocl_float4) +
                               sizeof(ocl_float2) * 2 +
                               sizeof(ocl_int);
   size_t combineIndexOffset = matchSuccessOffset + sizeof(int);
   size_t numPotentialMatchesOffset = matchSuccessOffset + sizeof(int) * 3;

   //Read the number of points in the current local map
   int numLocalPoints;
   size_t offset = currentLocalMap * sizeof(slamLocalMap) +
            numPointsOffset;
   readBuffer(clSlamLocalMap, CL_TRUE, offset, sizeof(ocl_int), &numLocalPoints, 0, 0, 0,
         "Copying number of laser points in current map");
   cout << "Local map has: " << numLocalPoints << " points" << endl;

   numLocalMapPoints[currentLocalMap] = numLocalPoints;
   if (numLocalPoints > maxNumLocalPoints) {
      maxNumLocalPoints = numLocalPoints;
   } 

   int oldLocalMap = currentLocalMap;
   int numOldMapPoints = numLocalPoints;

   if (parentLocalMap >= 0) {
      int globalSize = getGlobalWorkSize(numLocalMapPoints[parentLocalMap]);
      int potentialMatches[MAX_POTENTIAL_MATCHES + 1];

      int noConstraint = -1;
      opencl_task->setArg(3, 3, sizeof(int), &currentLocalMap);
      opencl_task->setArg(4, 3, sizeof(int), &noConstraint);
      opencl_task->setArg(3, 4, sizeof(int), &currentLocalMap);
      opencl_task->setArg(4, 4, sizeof(int), &nextLocalMap);
      //getHessianmatch and prepareLocalMap
      opencl_task->queueKernel(3, 1, globalSize, LocalSize, 
                                 0, NULL, NULL, false);
      opencl_task->queueKernel(4, 1, NUM_ORIENTATION_BINS, 0, 
                                 0, NULL, NULL, false);
      bool needOptimisation = false;

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
               cout << " Alignment is: ";
               for (k = 0; k < 4; k++) {
                  cout << outP[k] << " ";
               }
               cout << endl;
               // End debugging
               cout << "Number of iterations in alignment: " << c << " " << matchSuccess << endl;

               if (matchSuccess == 1) {
                  cout << "Alignment Succeeded" << endl;
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
                  needOptimisation = true;

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
                  cout << endl;
                  // End debugging
                  //Only add one loop closing constraint
                  break;
               } else {
                  cout << "Alignment failed" << endl;
               }
            }
         }
      }
      if (needOptimisation) {
         cout << "Optimising graph" << endl;
         //Get the current global pos of the current map
         size_t off = currentLocalMap * sizeof(slamLocalMap);
         ocl_float4 posBefore;
         readBuffer(clSlamLocalMap, CL_TRUE, off, sizeof(ocl_float4),
               &posBefore, 0, 0, 0, "Copying old global position");
         cout << "pos before: " << posBefore.x << " " << posBefore.y << " " << posBefore.w << endl;
         opencl_task->setArg(3, 9, sizeof(int), &nextLocalMap);
         opencl_task->setArg(4, 10, sizeof(int), &nextLocalMap);
         opencl_task->setArg(3, 11, sizeof(int), &nextLocalMap);
         opencl_task->setArg(3, 12, sizeof(int), &nextLocalMap);
         
         int kernelSize;
         for (int numIterations = 1; numIterations < 6; numIterations++) {
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
         kernelSize = getGlobalWorkSize(maxNumLocalPoints);
         opencl_task->queueKernel(12, 1, kernelSize, LocalSize,
               0, NULL, NULL, false);

         resetMap = true;
         historySlamPoses.resize(0);

         ocl_float4 posChange;
         readBuffer(clSlamLocalMap, CL_TRUE, off, sizeof(ocl_float4), &posChange, 
               0, 0, 0, "Copying new global position");
         cout << "Pos after: " << posChange.x << " " << posChange.y <<
               " " << posChange.w << endl;
         double posChangeX = posChange.x - posBefore.x;
         double posChangeY = posChange.y - posBefore.y;
         double posChangeTh = posChange.w - posBefore.w;
         cout << "pos change: " << posChangeX << " " << posChangeY << " " << posChangeTh << endl;

         slamPose.position.x += posChangeX;
         slamPose.position.y += posChangeY;
         double ys, ps, rs;
         slamPose.getYPR(ys, ps, rs);
         ys += posChangeTh;
         ANGNORM(ys);
         slamPose.setYPR(ys, ps, rs);

         if (LocalMapCombine) {
            combineMode++;
         }
         readBuffer(clGlobalMapPositions, CL_TRUE, 0, sizeof(ocl_float) * nextLocalMap * 3,
                   globalMapPositions, 0, 0, 0, "Copying global map positions");
      }
   } else {
      opencl_task->setArg(3, 4, sizeof(int), &currentLocalMap);
      opencl_task->setArg(4, 4, sizeof(int), &nextLocalMap);
      //Make sure the histograms are finalised for the first map
       opencl_task->queueKernel(4, 1, NUM_ORIENTATION_BINS, 0, 
                                 0, NULL, NULL, false);
      nextLocalMap++;
   }

   float alignError = angleError;
   opencl_task->setArg(3, 2, sizeof(int), &oldLocalMap);
   opencl_task->setArg(4, 2, sizeof(int), &nextLocalMap);
   opencl_task->setArg(5, 2, sizeof(int), &currentLocalMap);
   ocl_float4 pOffset;
   pOffset.x = offsetFromParentX;
   pOffset.y = offsetFromParentY;
   pOffset.z = 0;
   pOffset.w = offsetFromParentTh;
   opencl_task->setArg(6, 2, sizeof(ocl_float4), &pOffset);
   opencl_task->setArg(7, 2, sizeof(ocl_float), &alignError);
   opencl_task->setArg(8, 2, sizeof(int), &numOldMapPoints);
   int globalSize = getGlobalWorkSize(numOldMapPoints);
   opencl_task->queueKernel(2, 1, globalSize, LocalSize, 
         0, NULL, NULL, false);

   offsetFromParentX = 0;
   offsetFromParentY = 0;
   offsetFromParentTh = 0;
   double yi, pi, ri;
   icpPose.getYPR(yi, pi, ri);
   startICPTheta = yi;
   currentOffset.x = 0;
   currentOffset.y = 0;
   currentOffset.w = 0;
   currentLocalMapICPPose = icpPose;
   numGlobalPoints += numLocalPoints;
   parentLocalMap = currentLocalMap;
   currentLocalMap = nextLocalMap;
   globalMapPositions[currentLocalMap * 3] = slamPose.position.x;
   globalMapPositions[currentLocalMap * 3 + 1] = slamPose.position.y;
   double ys, ps, rs;
   slamPose.getYPR(ys, ps, rs);
   globalMapPositions[currentLocalMap * 3 + 2] = ys;

   //Grow data structures if needed
   if (numGlobalPoints + MAX_LOCAL_POINTS > totalGlobalPoints) {
      cout << "Run out of space to store global points" << endl;
      int increment = 10000;
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

      opencl_task->setArg(4, 1, sizeof(cl_mem), &clGlobalMap);
      opencl_task->setArg(2, 12, sizeof(cl_mem), &clGlobalMap);
      opencl_task->setArg(3, 13, sizeof(cl_mem), &clGlobalMap);
      opencl_task->setArg(5, 1, sizeof(cl_mem), &clGlobalMapHeights);
      opencl_task->setArg(5, 12, sizeof(cl_mem), &clGlobalMapHeights);
      opencl_task->setArg(9, 13, sizeof(cl_mem), &clGlobalMapHeights);
      opencl_task->setArg(4, 14, sizeof(cl_mem), &clGlobalMapHeights);
   }
   if (nextLocalMap + 2 >= totalLocalMaps) {
      cout << "Run of out space to store local maps" << endl;
      int increment = 20;
      slamLocalMap maps[totalLocalMaps + increment];
      readBuffer(clSlamLocalMap, CL_TRUE, 0, sizeof(slamLocalMap) * 
               totalLocalMaps, maps, 0, 0, 0, "Copying local map information");
      opencl_manager->deviceRelease(clSlamLocalMap);
      opencl_manager->deviceRelease(clGlobalMapPositions);
      int *temp = new int[totalLocalMaps + increment];
      int i;
      for(i = 0; i < totalLocalMaps; i++) {
         temp[i] = numLocalMapPoints[i];
      }
      delete [] numLocalMapPoints;
      numLocalMapPoints = temp;

      ocl_float *tempF = new ocl_float[(totalLocalMaps + increment) * 3];
      for(i = 0; i < totalLocalMaps * 3; i++) {
         temp[i] = globalMapPositions[i];
      }
      delete [] globalMapPositions;
      globalMapPositions = tempF;
         
      totalLocalMaps += increment;

      size_t size = sizeof(slamLocalMap) * totalLocalMaps;
      clSlamLocalMap = opencl_manager->deviceAlloc(size,
            CL_MEM_READ_WRITE | CL_MEM_COPY_HOST_PTR, maps);
      size = totalLocalMaps * sizeof(ocl_float) * 3;
      clGlobalMapPositions = opencl_manager->deviceAlloc(size,
            CL_MEM_READ_WRITE, NULL);

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
      opencl_task->setArg(4, 12, sizeof(cl_mem), &clGlobalMapPositions);
   }

}}
}

inline int GraphSlamGPU::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
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
      //cout << "Reseting map" << endl;
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
   gx = globalMapPositions[mapIndex * 3];
   gy = globalMapPositions[mapIndex * 3 + 1];
   gth = globalMapPositions[mapIndex * 3 + 2];
}

