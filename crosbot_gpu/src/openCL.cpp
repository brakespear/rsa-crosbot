

#include <sstream>
#include <fstream>
#include <iostream>
#include <string>
#include <string.h>
#include <stdlib.h>

//#include <casrobot/logger.h>

#include "graphSlam/openCL.h"
//#include "exception.h"

using namespace std;

/*
 * callback function for any error's that occur during the execution
 * of a context
 */
void CL_CALLBACK contextCallBack(const char *errInfo, const void *privInfo,
      size_t cb, void *userData) {
   stringstream ss;
   ss << "Error during context use: " << errInfo << endl;
   //throw OpenCLException(ss.str(), "openCL.cpp");
   cout << ss.str();
}


OpenCLManager::OpenCLManager() {
   init();
}

OpenCLManager::~OpenCLManager() {
   clReleaseCommandQueue(gpuCommandQueue);
   clReleaseContext(gpuContext);
}

//void _OpenCLManager::configure(ConfigElement config) {
   //No configs yet
//}

void OpenCLManager::addTask(OpenCLTask *task) {

}

void OpenCLManager::removeTask(OpenCLTask *task) {
}

void OpenCLManager::init() {
   //Tell the opencl compiler not to cache header files!!
   //TODO: Make this a config option
   setenv("CUDA_CACHE_DISABLE", "1", 1);
   //Get the number of openCL platforms
   cl_uint numPlatforms;
   clGetPlatformIDs (0, NULL, &numPlatforms);
   if (numPlatforms == 0) {
      stringstream ss;
      ss << "Didn't find any OpenCL capable platforms\n";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }

   //Get the openCL platform IDs
   cl_platform_id *platformIDs;
   platformIDs = (cl_platform_id *) malloc(sizeof(cl_platform_id) * numPlatforms);
   clGetPlatformIDs(numPlatforms, platformIDs, NULL);
   
   //Get a GPU device
   //TODO: change this to make it more general
   int i;
   for (i = 0; i < (int) numPlatforms; i++) {
      cl_int err = clGetDeviceIDs(platformIDs[i], CL_DEVICE_TYPE_GPU, 1, &deviceId, NULL);
      if (err == CL_SUCCESS) {
         break;
      }
   }
   if (i == (int)numPlatforms) {
      stringstream ss;
      ss << "Couldn't find an openCL GPU\n";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }

   //Create a context
   gpuContext = clCreateContext(0, 1, &deviceId, NULL, NULL, NULL);
   if (!gpuContext) {
      stringstream ss;
      ss << "Failed to create openCL context\n";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }

   //Create a command queue for the GPU context
   gpuCommandQueue = clCreateCommandQueue(gpuContext, deviceId, 0, NULL);
   //gpuCommandQueue = clCreateCommandQueue(gpuContext, deviceId, CL_QUEUE_PROFILING_ENABLE, NULL);
   if (!gpuCommandQueue) {
      stringstream ss;
      ss << "Failed to create command queue for gpu\n";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }

   free(platformIDs);
}

cl_context OpenCLManager::getContext() const {
   return gpuContext;
}

cl_device_id OpenCLManager::getDeviceId() const {
   return deviceId;
}

cl_command_queue OpenCLManager::getCommandQueue() const {
   return gpuCommandQueue;
}

cl_mem OpenCLManager::deviceAlloc(size_t size, int flags, void *hostPtr) {
   cl_mem memObj = clCreateBuffer(gpuContext, flags, size, hostPtr, 0);
   if (!memObj) {
      stringstream ss;
      ss << "buffer creation failed";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }
   return memObj;
}

void OpenCLManager::deviceRelease(cl_mem memObj) {
   clReleaseMemObject(memObj);
}





OpenCLTask::OpenCLTask(OpenCLManager *openCLManager) {
   manager = openCLManager;
   manager->addTask(this);
}

OpenCLTask::~OpenCLTask() {
   manager->removeTask(this);
   int i;
   for(i = 0; i < nKernels; i++) {
      clReleaseKernel(kernels[i]);
   }
   free(kernels);
   clReleaseProgram(program);
}

void OpenCLTask::compileProgram(string rootDir, const string *fileNames, int numFiles, const string *kernelNames, int numKernels,
      string buildOptions) {
   //Read the files
   int i;
   char *programSource[numFiles];
   for (i = 0; i < numFiles; i++) {
      programSource[i] = readFile(rootDir, fileNames[i]);
   }

   //Create the opencl program
   program = clCreateProgramWithSource(manager->getContext(), numFiles, (const char **)programSource, 0, 0);
   if (!program) {
      stringstream ss;
      ss << "Couldn't create opencl program";
      //throw OpenCLException(ss.str(), "openCL.cpp");
      cout << ss.str();
   }

   //Build the opencl program
   //int err = clBuildProgram(program, 0, 0, (buildOptions + " -D CL_RUNTIME -Werror -cl-opt-disable").c_str(), 0, 0);
   int err = clBuildProgram(program, 0, 0, (buildOptions + " -D CL_RUNTIME -Werror").c_str(), 0, 0);
   if (err != CL_SUCCESS) {
      compileError();
   }

   for (i = 0; i < numFiles; i++) {
      free(programSource[i]);
   }
 
   nKernels = numKernels;
   kernels = (cl_kernel *) malloc(sizeof(cl_kernel) * numKernels);
   for(i = 0; i < numKernels; i++) {
      kernels[i] = clCreateKernel(program, kernelNames[i].c_str(), &err);
      if (kernels[i] == NULL) {
         stringstream ss;
         ss << kernelNames[i] << ": Couldn't create kernel "<< err << endl;
         //throw OpenCLException(ss.str(), "openCL.cpp");
         cout << ss.str();

      }
   }

}

char *OpenCLTask::readFile(string rootDir, string fileName) {
   string file = rootDir + fileName;
   ifstream kernelFile(file.c_str(), ios::in);
   if (!kernelFile.is_open()) {
      stringstream ss;
      ss << "Failed to open file " << fileName << endl;
      //throw OpenCLException(ss.str(), "openCl.cpp");
      cout << ss.str();
   }
   //Get the file in a C type string
   ostringstream oss;
   oss << "#include \"" << rootDir << "/include/graphSlam/openclCommon.h\"\n";
   oss << kernelFile.rdbuf();
   string srcString = oss.str();
   char *cString = (char *) malloc(sizeof(char) * (srcString.size() + 1));
   strcpy(cString, srcString.c_str());
   return cString;
}

void OpenCLTask::compileError() {
   size_t len;
   char buff[100000];
   clGetProgramBuildInfo(program, manager->getDeviceId(), CL_PROGRAM_BUILD_LOG, sizeof(char) * 100000, buff, &len);
   //char *errors = (char *)malloc(sizeof(char) * len + 3);
   //clGetProgramBuildInfo(program, manager->getDeviceId(), CL_PROGRAM_BUILD_LOG, sizeof(char) * len + 3, errors, &len);
   stringstream ss;
   ss << buff;
   //throw OpenCLException(ss.str(), "openCL.cpp");
   cout << ss.str();
}


void OpenCLTask::setArg(int kernelArgNum, int kernelNum, size_t size, void *ptr) {
   int err = clSetKernelArg(kernels[kernelNum], kernelArgNum, size, ptr);
   if (err != CL_SUCCESS) {
      stringstream ss;
      ss << "Error Setting kernel arguements. Kernel: " << kernelNum << " Arg Num: " << kernelArgNum << 
         " Error code: " << err << endl;
      //throw OpenCLException(ss.str(), "openCl.cpp");
      cout << ss.str();
   }

}


int OpenCLTask::queueKernel(int kernelNum, int workDim, int totalNumWorkItems, 
      int workGroupSize, int numEvtsInList, cl_event *waitList, cl_event *evt, bool profile) {
   if (evt == NULL && profile) {
      cl_event prof_event;
      evt = &prof_event;
   }
   if (!kernels[kernelNum]) {
      cout << "something wrong " << this << endl;
   }
   size_t totalWorkItems = totalNumWorkItems;
   size_t sizeWorkGroup = workGroupSize;

   if (workGroupSize == 0) {
      int ret = clEnqueueNDRangeKernel(manager->getCommandQueue(), kernels[kernelNum], workDim, 0, &totalWorkItems,
            0, numEvtsInList, waitList, evt);
      if (ret != CL_SUCCESS) {
         cout << "Error queueing kernel " << kernelNum << " error " << ret << endl;
         exit(0);
      }
   } else {
      int ret = clEnqueueNDRangeKernel(manager->getCommandQueue(), kernels[kernelNum], workDim, 0, &totalWorkItems,
            &sizeWorkGroup, numEvtsInList, waitList, evt);
      if (ret != CL_SUCCESS) {
         cout << "Error queueing kernel " << kernelNum << " error " << ret << endl;
         exit(0);
      }
   }
   //Debugging only - delete when properly time code
   /*int res = clFinish(manager->getCommandQueue());
   if (res != CL_SUCCESS) {
      cout << "Error during kernel, code: " << res << " kernel num: " << kernelNum << " global size " << totalNumWorkItems << endl;
      exit(0);
   }*/
   //End debugging only part

   if (profile) {
      clWaitForEvents(1, evt);
      cl_ulong start = 0;
      cl_ulong end = 0;
      size_t ret;
      clGetEventProfilingInfo(*evt, CL_PROFILING_COMMAND_START, sizeof(cl_ulong), &start, &ret);
      clGetEventProfilingInfo(*evt, CL_PROFILING_COMMAND_END, sizeof(cl_ulong), &end, &ret);
      int time = (end - start);
      return time;
      
   }
   return 0;
}


