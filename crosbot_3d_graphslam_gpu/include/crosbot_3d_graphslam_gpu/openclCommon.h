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
#else
typedef cl_float2 ocl_float2;
typedef cl_float3 ocl_float3;
typedef cl_float4 ocl_float4;
typedef cl_float ocl_float;
typedef cl_int ocl_int;
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

} ocl3DGraphSlamConfig;


#endif
