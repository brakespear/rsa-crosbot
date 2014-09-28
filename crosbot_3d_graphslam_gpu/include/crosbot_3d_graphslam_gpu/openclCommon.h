/*
 * openclCommon.h
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

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
typedef short ocl_short;
#else
typedef cl_float2 ocl_float2;
typedef cl_float3 ocl_float3;
typedef cl_float4 ocl_float4;
typedef cl_float ocl_float;
typedef cl_int ocl_int;
typedef cl_short ocl_short;
#endif

//Stores kinect information for each scan
typedef struct {
#ifdef CL_RUNTIME
   ocl_float pointX[NUM_DEPTH_POINTS];
   ocl_float pointY[NUM_DEPTH_POINTS];
   ocl_float pointZ[NUM_DEPTH_POINTS];
   unsigned char r[NUM_DEPTH_POINTS];
   unsigned char g[NUM_DEPTH_POINTS];
   unsigned char b[NUM_DEPTH_POINTS];
#else
   ocl_float *pointX;
   ocl_float *pointY;
   ocl_float *pointZ;
   unsigned char *r;
   unsigned char *g;
   unsigned char *b;
#endif
} oclDepthPoints;

typedef struct {
   //width and height of a single scan
   ocl_int ScanWidth;
   ocl_int ScanHeight;
   //Dimensions of local map
   ocl_float LocalMapWidth;
   ocl_float LocalMapHeight;
   //Dimensions of a block of the local map
   ocl_float BlockSize;
   ocl_int NumBlocksTotal;
   ocl_int NumBlocksWidth;
   ocl_int NumBlocksHeight;
   //Dimensions of a cell in a block of the local map
   ocl_float CellSize;
   ocl_int NumCellsTotal;
   ocl_int NumCellsWidth;
   ocl_int NumBlocksAllocated;
   //Truncation values for tsdf
   ocl_float TruncNeg;
   ocl_float TruncPos;
   //camera params
   ocl_float fx;
   ocl_float fy;
   ocl_float cx;
   ocl_float cy;
   ocl_float tx;
   ocl_float ty;
   //Max distance of a cell to the camera that will be used in the tsdf
   ocl_float MaxDistance;
   //Max distance to search during map alignment
   ocl_int MaxSearchCells;

} oclGraphSlam3DConfig;

typedef struct {
   int blockIndex;
#ifdef CL_RUNTIME
   ocl_float distance[NUM_CELLS];
   ocl_float weight[NUM_CELLS];
   ocl_int pI[NUM_CELLS];
   unsigned char r[NUM_CELLS];
   unsigned char g[NUM_CELLS];
   unsigned char b[NUM_CELLS];
#else
   ocl_float *distance;
   ocl_float *weight;
   ocl_int *pI;
   unsigned char *r;
   unsigned char *g;
   unsigned char *b;
#endif
}  oclLocalBlock;

typedef struct {
   ocl_int numBlocks;
   ocl_int numActiveBlocks;
   ocl_int numPoints;
   ocl_int numMatch;
   ocl_float distance;
#ifdef CL_RUNTIME
   ocl_int activeBlocks[MAX_NUM_ACTIVE_BLOCKS];
#else
   ocl_int *activeBlocks;
#endif
} oclLocalMapCommon;

#endif
