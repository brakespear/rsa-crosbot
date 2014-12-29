
#ifndef OPENCL_HPP_
#define OPENCL_HPP_

//This is a bad idea but....#YOLO
//Undefining strict ansi before include cl.h means
//opencl vector elements can be accessed by x,y,z,w.
#undef __STRICT_ANSI__
#include <string>
#include <CL/cl.h>

//#include <casrobot/config.h>


namespace std {

/**
 * Callback used by opencl context if there are run time errors during the
 * life of the context
 */
void CL_CALLBACK contextCallBack(const char *errInfo, const void *privInfo,
      size_t cb, void *userData);

class OpenCLTask;

//General class for setting up and managing an opencl environment
//At the moment only works for one GPU
class OpenCLManager {
public:

   OpenCLManager();
   ~OpenCLManager();

   //void configure(ConfigElement config);

   void addTask(OpenCLTask *task);

   void removeTask(OpenCLTask *task);
  
   /*
    * Allocate a buffer in the manager's context
    */ 
   cl_mem deviceAlloc(size_t size, int flags, void *hostPtr);
   void deviceRelease(cl_mem memObj);

   cl_context getContext() const;

   cl_device_id getDeviceId() const;

   cl_command_queue getCommandQueue() const;

private:

   cl_context gpuContext;
   cl_command_queue gpuCommandQueue;
   cl_device_id deviceId;

   void init();
};


/* A single openCL task. Can consist of multiple kernels
 * 
 */
class OpenCLTask {
public:
   OpenCLTask(OpenCLManager *openCLManager);
   ~OpenCLTask();
   /*
    * Reads in the files contained in the fileNames array,c ompiles them, and creates the
    * kernels specified in the kernelNames array
    */
   void compileProgram(std::string rootDir,
                       const std::string *fileNames, int numFiles,
                       const std::string *kernelNames, int numKernels, 
                       std::string buildOptions, std::string headerFile);
   
   /* set ptr as a kernel arg
    */
   void setArg(int kernelArgNum, int kernelNum, size_t size, void *ptr);
   /*
    * Enqueues the kernel onto the context.
    */
   int queueKernel(int kernelNum, int workDim, int totalNumWorkItems, 
      int workGroupSize, int numEvtsInList, cl_event *waitList, 
      cl_event *evt, bool profile);


   cl_kernel *kernels;

private:
   OpenCLManager *manager;
   cl_program program;
   int nKernels;

   static const string common_opencl_file;

   char *readFile(std::string rootDir, std::string fileName, std::string headerFile);
   void compileError();

};


} //namespace casrobot

#endif
