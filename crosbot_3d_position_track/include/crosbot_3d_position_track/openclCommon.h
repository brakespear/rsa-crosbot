/*
 * openclCommon.h
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#ifndef OPENCL_COMMON_H_
#define OPENCL_COMMON_H_

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

//Stores kinect information for each scan
typedef struct {
#ifdef CL_RUNTIME
   ocl_float pointX[NUM_DEPTH_POINTS];
   ocl_float pointY[NUM_DEPTH_POINTS];
   ocl_float pointZ[NUM_DEPTH_POINTS];
#else
   ocl_float *pointX;
   ocl_float *pointY;
   ocl_float *pointZ;
#endif
} oclDepthPoints;

typedef struct {
   //width and height of a single scan
   ocl_int ScanWidth;
   ocl_int ScanHeight;
   ocl_float MapWidth;
   ocl_float MapHeight;
   ocl_float CellSize;
   ocl_int NumCellsWidth;
   ocl_int NumCellsHeight;
   ocl_int MaxSearchCells;
} oclPositionTrackConfig;

typedef struct {
#ifdef CL_RUNTIME
   ocl_float z[NUM_CELLS];
   ocl_float normX[NUM_CELLS];
   ocl_float normY[NUM_CELLS];
   ocl_float normZ[NUM_CELLS];
   ocl_int count[NUM_CELLS];
   ocl_int cellXY[NUM_DEPTH_POINTS];
#else
   ocl_float *z;
   ocl_float *normX;
   ocl_float *normY;
   ocl_float *normZ;
   ocl_int *count;
   ocl_int *cellXY;
#endif
} oclLocalMap;

typedef struct {
   ocl_int numMatch;
   ocl_float distance;
} oclCommon;


#endif
