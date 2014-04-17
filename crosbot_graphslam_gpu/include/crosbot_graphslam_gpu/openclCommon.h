
//Data types shared between standard ros code and opencl code
//
#ifndef OPENCL_COMMON_H_
#define OPENCL_COMMON_H_

#define MAX_LOCAL_POINTS 3000
#define MAX_POTENTIAL_MATCHES 50
#define NUM_ORIENTATION_BINS 64
#define NUM_PROJECTION_BINS 100

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
#else
   ocl_float *pointX;
   ocl_float *pointY;
   ocl_float *pointZ;
#endif
} oclLaserPoints;

/*
 * SLAM data structures
 */
/*
 * Configuration parameters for graph SLAM
 */
typedef struct {
   //dimension of a side of the occupancy grid for a local map
   ocl_int DimLocalOG;
   //the width in metres of a single cell on the local occupancy grid
   ocl_float CellWidthOG;
   //dimension of a side of the occupancy grid for the global map
   ocl_int DimGlobalOG;
   //The number of cells away from the currently aligned cell
   //that will be examined for an icp match
   ocl_int SearchSize;
   //The maximum distance points can be apart to be aligned by icp
   ocl_float MaxSlamAlignDist;
   //The L value used for the icp in slam
   ocl_float SlamL;
   //Scale factor for the information matrix to make the values
   //represent an appropriate level of uncertainty
   ocl_int InformationScaleFactor;
   //The maximum covariance between local frames permitted
   ocl_float MaxCovar;
   //The threshold used to consider a histogram match
   ocl_float CorrelationThreshold;
   //The minimum number of matches needed for each match of icp
   ocl_int MinGoodCount;
   //The minimum number of good matches needed to consider scans
   //successfully aligned
   ocl_int FinalMinGoodCount;
   //The maximum number of ICP iterations in local map alignment
   ocl_int MaxIterations;
   //An alginment is finished when the movement in an iteration
   //falls below this threshold
   ocl_float MaxErrorDisp;
   ocl_float MaxErrorTheta;
   //The distance the robot can travel before a new local map is
   //created
   ocl_float LocalMapDist;
   //Allow local maps to be combined after a loop closure
   ocl_int LocalMapCombine;
   //Maximum difference locla maps can move when finding a loop closing constraint
   ocl_float MaxThetaOptimise;
   //Minimum number of times a point has to be observed before being
   //added to the map
   ocl_int MinObservationCount;
//new slam configs
   ocl_float PerScanInfoScaleFactor;
   ocl_float GradientDistanceThreshold;
   ocl_float FreeAreaDistanceThreshold;
   ocl_float LocalMapCovarianceThreshold;
   ocl_float FreeAreaThreshold;
   ocl_int PreventMatchSymmetrical;
} slamConfig;

/*
 * the data stored in each local map
 */
/*
 * notes:
 * MAX_SLAM_LOCAL_POINTS needs to be set in a file somewhere
 * as need this to be present at compile time
 */
typedef struct {
   ocl_float4 currentGlobalPos;
   ocl_float4 parentOffset;

   //The number of laser points in the local map
   //NOTE THIS IS READ DIRECTLY IN CPU CODE
   ocl_int numPoints;
   ocl_int isFeatureless;
   ocl_int indexParentNode;
   //The level of the node in the tree
   ocl_int treeLevel;
//new bit
   ocl_float internalCovar[3][3];

   ocl_float parentInfo[3][3];

   //The change in position during an optimisation run
   ocl_float changeInPos[3];
   //The current global covar - obtained from rotating the 
   //parent information matrix by the global rotation
   //matrix of the *****PARENT***** and inverting
   ocl_float globalCovar[3][3];
   //The centre coord of the local map. Relative to the local map
   ocl_float2 mapCentre;
   ocl_float2 robotMapCentre;
   ocl_float2 globalRobotMapCentre;
   ocl_float orientationHist[NUM_ORIENTATION_BINS];
   ocl_float projectionHist[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS];
   ocl_float entropyHist[NUM_ORIENTATION_BINS];
   //Want these to be in the struct on the CPU as well
   ocl_float pointsX[MAX_LOCAL_POINTS];
   ocl_float pointsY[MAX_LOCAL_POINTS];
   ocl_float pointsZ[MAX_LOCAL_POINTS];
   ocl_float gradX[MAX_LOCAL_POINTS];
   ocl_float gradY[MAX_LOCAL_POINTS];
} slamLocalMap;

/*
 * Any data required for the entire graph SLAM algorithm
 * or data required temporairily as a local map is being
 * constructed
 */
/*
 * notes:
 * SIZE_LOCAL_OG: size of occupancy grid of local node
 *          set by c++ code. should be the square of
 *          DimLocalOG in config struct
 */
typedef struct {
   //offset so far in the local map
   ocl_float4 currentOffset;
   //Minimum and maximum extent of the current local map
   ocl_float2 minMapRange;
   ocl_float2 maxMapRange;
   //Number of points used to calculate the information matrix
   ocl_int infoCount;
   //NOTE THESE ARE READ DIRECTLY IN CPU CODE
   ocl_int matchSuccess; 
   ocl_int combineIndex;
   ocl_int combineMode;
   ocl_int numPotentialMatches;
   ocl_int potentialMatches[MAX_POTENTIAL_MATCHES];
   ocl_float potentialMatchX[MAX_POTENTIAL_MATCHES];
   ocl_float potentialMatchY[MAX_POTENTIAL_MATCHES];
   ocl_float potentialMatchTheta[MAX_POTENTIAL_MATCHES];
   ocl_int potentialMatchParent[MAX_POTENTIAL_MATCHES];
//newbies read in by cpu code
   ocl_int covarCount;
   ocl_int evaluateOverlap;
   ocl_float evaluateScore;
   ocl_float tempCovar[3][3];
   ocl_int activeCells[MAX_LOCAL_POINTS];
   ocl_int previousINode;
//end newbies
   //Variables used for the ICP alignment
   ocl_int numIterations;
   ocl_float A[3][3];
   ocl_float B[3];
   ocl_int goodCount;
   ocl_int numConstraints;
   ocl_int numLoopConstraints;
   //Precomputed values of sin and cos theta for each of the
   //orientation histogram bins
   ocl_float histCos[NUM_ORIENTATION_BINS];
   ocl_float histSin[NUM_ORIENTATION_BINS];
   ocl_float scaleFactor[3];
#ifdef CL_RUNTIME
   ocl_int localOG[SIZE_LOCAL_OG];
   ocl_int localOGCount[SIZE_LOCAL_OG];
   ocl_float localOGZ[SIZE_LOCAL_OG];
//start newbies - first two just renamed, third one is new
   ocl_float localOGX[SIZE_LOCAL_OG];
   ocl_float localOGY[SIZE_LOCAL_OG];
   ocl_float localOGGradX[SIZE_LOCAL_OG];
   ocl_float localOGGradY[SIZE_LOCAL_OG];   
   ocl_float pointsTempX[NUM_LASER_POINTS];
   ocl_float pointsTempY[NUM_LASER_POINTS];
   ocl_float pointsTempZ[NUM_LASER_POINTS];
//end newbies
   //0 if parent constraint, 1 if loop constraint
   ocl_int constraintType[MAX_NUM_CONSTRAINTS];
   ocl_int constraintIndex[MAX_NUM_CONSTRAINTS];
   ocl_int loopConstraintParent[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_int loopConstraintI[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_int loopConstraintJ[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_float loopConstraintXDisp[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_float loopConstraintYDisp[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_float loopConstraintThetaDisp[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_float loopConstraintWeight[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_int loopConstraintFull[MAX_NUM_LOOP_CONSTRAINTS];
   ocl_float loopConstraintInfo[MAX_NUM_LOOP_CONSTRAINTS][3][3];
   //Will never have more nodes that number of constraints + 1
   ocl_float graphHessian[MAX_NUM_CONSTRAINTS + 1][3];
#else
   ocl_int *localOG;
   ocl_int *localOGCount;
   ocl_float *localOGZ;
   ocl_float *localOGX;
   ocl_float *localOGY;
   ocl_float *localOGGradX;
   ocl_float *localOGGradY;
   ocl_float *pointsTempX;
   ocl_float *pointsTempY;
   ocl_float *pointsTempZ;
   ocl_int *constraintType;
   ocl_int *constraintIndex;
   ocl_int *loopConstraintParent;
   ocl_int *loopConstraintI;
   ocl_int *loopConstraintJ;
   ocl_float *loopConstraintXDisp;
   ocl_float *loopConstraintYDisp;
   ocl_float *loopConstraintThetaDisp;
   ocl_float *loopConstraintWeight;
   ocl_int *loopConstraintFull;
   ocl_float *loopConstraintInfo[3][3];
   ocl_float *graphHessian[3];
#endif
} slamCommon;

#endif
