/*
 * openclCommon.h
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#ifndef OPENCL_COMMON_H_
#define OPENCL_COMMON_H_

#define NUM_RESULTS 29

//Vectors have different names in host code and kernel code. Kernel code
//will be compiled with CL_RUNTIME flag set
#ifdef CL_RUNTIME
typedef float2 ocl_float2;
typedef int2 ocl_int2;
typedef int3 ocl_int3;
typedef float3 ocl_float3;
typedef float4 ocl_float4;
typedef float ocl_float;
typedef int ocl_int;
typedef short ocl_short;
#else
typedef cl_float2 ocl_float2;
typedef cl_int2 ocl_int2;
typedef cl_int3 ocl_int3;
typedef cl_float3 ocl_float3;
typedef cl_float4 ocl_float4;
typedef cl_float ocl_float;
typedef cl_int ocl_int;
typedef cl_short ocl_short;
#endif

typedef struct {
   ocl_int ImageWidth;
   ocl_int ImageHeight;
   ocl_float LocalMapWidth;
   ocl_float LocalMapHeight;
   ocl_float BlockSize;
   ocl_int NumBlocksTotal;
   ocl_int NumBlocksWidth;
   ocl_int NumBlocksHeight;
   ocl_float CellSize;
   ocl_int NumCellsTotal;
   ocl_int NumCellsWidth;
   ocl_int NumBlocksAllocated;
   ocl_float TruncPos;
   ocl_float TruncNeg;
   ocl_float fx;
   ocl_float fy;
   ocl_float cx;
   ocl_float cy;
   ocl_float tx;
   ocl_float ty;
   ocl_int UseOccupancyForSurface;
   ocl_float MaxDistance;
   ocl_int MaxPoints;
   ocl_int ReExtractBlocks;
} oclPositionTrackConfig;

typedef struct {
#ifdef CL_RUNTIME
   ocl_float x[NUM_DEPTH_POINTS];
   ocl_float y[NUM_DEPTH_POINTS];
   ocl_float z[NUM_DEPTH_POINTS];
#else
   ocl_float *x;
   ocl_float *y;
   ocl_float *z;
#endif
} oclDepthPoints;


typedef struct {
#ifdef CL_RUNTIME
   unsigned char r[NUM_DEPTH_POINTS];
   unsigned char g[NUM_DEPTH_POINTS];
   unsigned char b[NUM_DEPTH_POINTS];
#else
   unsigned char *r;
   unsigned char *g;
   unsigned char *b;
#endif
} oclColourPoints;

typedef struct {
   int blockIndex;
   int haveExtracted;
#ifdef CL_RUNTIME
   ocl_float distance[NUM_CELLS];
   ocl_float weight[NUM_CELLS];
   ocl_int pI[NUM_CELLS];
   unsigned char r[NUM_CELLS];
   unsigned char g[NUM_CELLS];
   unsigned char b[NUM_CELLS];
   unsigned char occupied[NUM_CELLS];
#else
   ocl_float *distance;
   ocl_float *weight;
   ocl_int *pI;
   unsigned char *r;
   unsigned char *g;
   unsigned char *b;
   unsigned char *occupied;
#endif
} oclLocalBlock;

typedef struct {
   ocl_int highestBlockNum; //maxBlock needed
   ocl_int nextEmptyBlock;
   ocl_int numActiveBlocks;
   ocl_int numBlocksToExtract;
   ocl_int numPoints;
   ocl_int numBlocksToDelete;
   ocl_float icpResults[NUM_RESULTS];
#ifdef CL_RUNTIME
   ocl_int activeBlocks[MAX_NUM_ACTIVE_BLOCKS];
   ocl_int emptyBlocks[NUM_BLOCKS_ALLOCATED];
   ocl_int blocksToExtract[NUM_BLOCKS_ALLOCATED];
   ocl_int blocksToDelete[NUM_BLOCKS_ALLOCATED];
#else
   ocl_int *activeBlocks;
   ocl_int *emptyBlocks;
   ocl_int *blocksToExtract;
   ocl_int *blocksToDelete;
#endif
} oclLocalMapCommon;

#endif
