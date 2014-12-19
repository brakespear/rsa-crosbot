/*
 * openclCommon.h
 *
 * Created on: 12/12/2014
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
   ocl_float TruncPos;
   ocl_float TruncNeg;
   ocl_float fx;
   ocl_float fy;
   ocl_float cx;
   ocl_float cy;
   ocl_float tx;
   ocl_float ty;
} oclPositionTrackConfig;


#endif
