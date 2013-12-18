
//Data types shared between standard casrobot code and opencl code
//
#ifndef OPENCL_COMMON_H_
#define OPENCL_COMMON_H_

//Vectors have different names in host code and kernel code. Kernel code
//will be compiled with CL_RUNTIME flag set
#ifdef CL_RUNTIME
typedef float2 ocl_float2;
typedef float3 ocl_float3;
typedef float4 ocl_float4;
typedef float ocl_float;
typedef int ocl_int;
#else
typedef cl_float2 ocl_float2;
typedef cl_float3 ocl_float3;
typedef cl_float4 ocl_float4;
typedef cl_float ocl_float;
typedef cl_int ocl_int;
#endif

//Stores information about laser points for each scan
typedef struct {
#ifdef CL_RUNTIME
   ocl_float pointX[NUM_LASER_POINTS];
   ocl_float pointY[NUM_LASER_POINTS];
   ocl_float pointZ[NUM_LASER_POINTS];
   ocl_float pointNextX[NUM_LASER_POINTS];
   ocl_float pointNextY[NUM_LASER_POINTS];
   ocl_float pointNextZ[NUM_LASER_POINTS];
#else
   ocl_float *pointX;
   ocl_float *pointY;
   ocl_float *pointZ;
   ocl_float *pointNextX;
   ocl_float *pointNextY;
   ocl_float *pointNextZ;
#endif
   ocl_float4 offset;
   ocl_float3 laserOffset /*__attribute__((aligned(16)))*/;
} oclLaserPoints;

//Configuration parameters used on the gpu
typedef struct {
   //The most number of laser points that can be stored in each 3D cell
   ocl_int MaxLaserPointsInCell;
   //The number of 2D cells across the map
   ocl_int MapDimWidth;
   //The number of cells in the z axis of the map
   ocl_int MapDimHeight;
   //The width of each 3D cell
   ocl_float MapCellWidth;
   //The height of each 3D cell
   ocl_float MapCellHeight;
   //The maximum height of a laser point to be in the map
   ocl_float MapMaxHeight;
   //The minimum height of a laser point to be in the map
   ocl_float MapMinHeight;
   //The maximum distance to a laser point
   ocl_float MaxLaserDist;
   //The minimum distance a point can be to the laser
   ocl_float LaserMinDist;
   //The furthest away a laser point will be aligned
   ocl_float LaserMaxAlign;
   //maximum distance two points can be away from each other to calculate H
   //Note that this value is maxAlignDist^2 to avoid needing sqr roots
   ocl_float MaxAlignDist;
   //Should simple H calculations be used?
   ocl_int UseSimpleH;
   //Should a variable value of l be used?
   ocl_int UseVariableL;
   //Factor least suqares calculations by distance away and number of obs
   ocl_int UseFactor;
   //The l value used for position tracking
   ocl_float l;
   //Factor used for variable l
   ocl_float AlignmentDFix;
   //Selects which algorithm to use for adjusting h val
   ocl_int NearestAlgorithm;
   //The maximum number of laser points that can be stored in a 2D cell
   ocl_int MaxObservations;
   //The number of 3D cells that will have their points searched to
   //find the best h match
   ocl_int FullSearchSize;
   //Minimum factor allowed
   ocl_float MinFactor;
   //The minimum number of matching points accepted for an
   //alignment to continue
   ocl_int MinGoodCount;
   //Maximum number of iterations before giving up
   ocl_int MaxIterations;
   //Maximum error in match to proceed
   ocl_float MaxErrX;
   ocl_float MaxErrY;
   ocl_float MaxErrTh;
   //The maximum move allowed in a single run
   ocl_float MaxMoveX;
   ocl_float MaxMoveY;
   ocl_float MaxMoveTh;
   //Number of scans to not update or add points to the map
   //Default is one, so it does nothing by default
   ocl_int MaxScanSkip;
   //The number of frames skipped when adding new points to the map
   ocl_int AddSkipCount;
   //Maximum number of frames that can fail to be matched in a row
   //before points are added to map anyway
   ocl_int MaxFail;
   //The effect of an observation of a laser point on the life time of the
   //cell
   ocl_float LifeRatio;
   ocl_int UsePriorMove;
   ocl_float FlobsticleHeight;
   ocl_float FloorHeight;
} configValues;


typedef struct {
   ocl_float A[4][4];
   ocl_float B[4];
   ocl_float2 robotOff;   //robot_x_off
   ocl_int goodCount;
   ocl_int failCount;
   ocl_int scanSkip;
   ocl_int addSkip;
   ocl_int badScan;
   ocl_int numIterations;
   ocl_int nextEmptyCell;
#ifdef CL_RUNTIME
   ocl_int pointsMask[NUM_LASER_POINTS];
   ocl_float pointMatchX[NUM_LASER_POINTS];
   ocl_float pointMatchY[NUM_LASER_POINTS];
   ocl_float pointMatchZ[NUM_LASER_POINTS];

   ocl_int activeCellsMap[MAP_SIZE];
   ocl_int indexEmptyCells[MAX_ACTIVE_CELLS];
   //Active Cell values
   ocl_int xIndex[MAX_ACTIVE_CELLS];
   ocl_int yIndex[MAX_ACTIVE_CELLS];
   ocl_int obsCount[MAX_ACTIVE_CELLS];
   ocl_int prevObsCount[MAX_ACTIVE_CELLS];
   ocl_int lifeCount[MAX_ACTIVE_CELLS];
   ocl_float cellMinZ[MAX_ACTIVE_CELLS];
   ocl_float cellMaxZ[MAX_ACTIVE_CELLS];
   ocl_int numLaserPoints[NUM_LASER_POINTS_CELL];
   ocl_float cellLaserPointsX[NUM_LASER_POINT_COORDS];
   ocl_float cellLaserPointsY[NUM_LASER_POINT_COORDS];
   ocl_float cellLaserPointsZ[NUM_LASER_POINT_COORDS];
   ocl_float cellLaserPointsNxtX[NUM_LASER_POINT_COORDS];
   ocl_float cellLaserPointsNxtY[NUM_LASER_POINT_COORDS];
   ocl_float cellLaserPointsNxtZ[NUM_LASER_POINT_COORDS];
#else
   ocl_int *pointsMask;
   ocl_float *pointMatchX;
   ocl_float *pointMatchY;
   ocl_float *pointMatchZ;
   ocl_int *activeCellsMap;
   ocl_int *indexEmptyCells;
   //Active Cell values
   ocl_int *xIndex;
   ocl_int *yIndex;
   ocl_int *obsCount;
   ocl_int *prevObsCount;
   ocl_int *lifeCount;
   ocl_float *cellMinZ;
   ocl_float *cellMaxZ;
   ocl_int *numLaserPoints;
   ocl_float *cellLaserPointsX;
   ocl_float *cellLaserPointsY;
   ocl_float *cellLaserPointsZ;
   ocl_float *cellLaserPointsNxtX;
   ocl_float *cellLaserPointsNxtY;
   ocl_float *cellLaserPointsNxtZ;
#endif
} oclLocalMap3D;

typedef struct {
   ocl_float4 finalOffset;  //gx, gy, gz, gth
   ocl_float2 cellShift;  //pos_x, pos_y (xch, ych)
   ocl_int largestActiveCell;
} oclResults;

typedef struct {
   ocl_int proceed;
} oclPartialResults;

typedef struct {
   //-2 flobsticle cell
   //-1 inactive cell
   //0 currently detected cell
   //otherwise lifecount of cell
#ifdef CL_RUNTIME
   ocl_int cellStatus[MAX_ACTIVE_CELLS];
   ocl_int mapIndex[MAX_ACTIVE_CELLS];
#else
   ocl_int *cellStatus;
   ocl_int *mapIndex;
#endif
} oclSharedMap;

#endif
