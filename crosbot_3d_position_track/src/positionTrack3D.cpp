/*
 * positionTrack3D.cpp
 *
 * Created on: 25/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_position_track/positionTrack3D.hpp>


using namespace std;
using namespace crosbot;

const string PositionTrack3D::file_names[] = {
   "/src/opencl/positionTrack3D.cl"
};
const int PositionTrack3D::num_files = sizeof(file_names) / sizeof(file_names[0]);
const string PositionTrack3D::header_file = "/include/crosbot_3d_position_track/openclCommon.h";
const string PositionTrack3D::kernel_names[] = {
   "transform3D"
};
const int PositionTrack3D::num_kernels = sizeof(kernel_names) / sizeof(kernel_names[0]);


PositionTrack3D::PositionTrack3D() {
   opencl_manager = new OpenCLManager();
   opencl_task = new OpenCLTask(opencl_manager);

   FILE *file = popen("rospack find crosbot_3d_position_track", "r");
   char buffer[200];
   fscanf(file, "%199s", buffer);
   pclose(file);
   rootDir = buffer;
}

void PositionTrack3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");

   paramNH.param<int>("LocalSize", LocalSize, 256);
   paramNH.param<double>("MapWidth", MapWidth, 8.0);
   paramNH.param<double>("MapHeight", MapWidth, 4.0);
   paramNH.param<double>("CellSize", CellSize, 0.05);
}

void PositionTrack3D::start() {
}

PositionTrack3D::~PositionTrack3D() {
   delete opencl_task;
   delete opencl_manager;
}

void PositionTrack3D::stop() {
}

void PositionTrack3D::initialiseFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose) {
   cout << "Position track 3D: starting compile" << endl;

   //Set config options
   positionTrackConfig.ScanWidth = depthPoints->width;
   positionTrackConfig.ScanHeight = depthPoints->height;

   clPositionTrackConfig = opencl_manager->deviceAlloc(sizeof(oclPositionTrackConfig),
         CL_MEM_READ_ONLY | CL_MEM_COPY_HOST_PTR, &positionTrackConfig);

   numDepthPoints = depthPoints->width * depthPoints->height;
   //Round up to the nearest 16 for memory coalescing
   numDepthPoints = ((numDepthPoints + 15) / 16) * 16;
   stringstream ss;
   ss << "-D NUM_DEPTH_POINTS=" << numDepthPoints <<
         "-D LOCAL_SIZE=" << LocalSize;
   opencl_task->compileProgram(rootDir, file_names, num_files, kernel_names, num_kernels,
         ss.str(), header_file);

   initialiseDepthPoints();

   cout << "Position track 3D: finished compile" << endl;

}

void PositionTrack3D::initialiseDepthPoints() {
   points = new oclDepthPoints;
   points->pointX = new ocl_float[numDepthPoints * 3];
   points->pointY = &(points->pointX[numDepthPoints]);
   points->pointZ = &(points->pointX[numDepthPoints * 2]);
   pointsSize = sizeof(ocl_float) * numDepthPoints * 3;
   clPoints = opencl_manager->deviceAlloc(pointsSize, CL_MEM_READ_WRITE, NULL);
}

void PositionTrack3D::processFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose icpPose) {

   int i;
   for (i = 0; i < numDepthPoints; ++i) {
      points->pointX[i] = depthPoints->cloud[i].x;
      points->pointY[i] = depthPoints->cloud[i].y;
      points->pointZ[i] = depthPoints->cloud[i].z;
   }

   int globalSize = getGlobalWorkSize(numDepthPoints);

   writeBuffer(clPoints, CL_TRUE, 0, pointsSize, points->pointX, 0, 0, 0,
         "Copying depth points to GPU");

   //Pose has a Point3D position field and a quaternion orientation field
   //Point3D has x,y,z fields, quaternion has x,y,z,w fields
   
   //Can do something like:
   tf::Quaternion quat = sensorPose.orientation.toTF();
   tf::Matrix3x3 basis(quat);
   tf::Vector3 origin = sensorPose.position.toTF();
   //or:
   //tf::transfrom trans = ....
   //tf::Matrix3x3 basis = trans.getBasis();
   //tf::Vector3 origin = trans.getOrigin();
   
   ocl_float3 clBasis[3];
   ocl_float3 clOrigin;
   for (int j = 0; j < 3; j++) {
      clBasis[j].x = basis[j][0];
      clBasis[j].y = basis[j][1];
      clBasis[j].z = basis[j][2];
      clOrigin.x = origin[0];
   }
   //then can pass clBasis and clOrigin into kernel
}

inline int PositionTrack3D::getGlobalWorkSize(int numThreads) {
   return numThreads % LocalSize == 0 ? numThreads :
                        ((numThreads / LocalSize) + 1) * LocalSize;
}

void PositionTrack3D::writeBuffer(cl_mem buffer, cl_bool blocking_write, size_t offset, 
      size_t cb, const void *ptr, cl_uint num_events_in_wait_list, 
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueWriteBuffer(opencl_manager->getCommandQueue(), buffer, blocking_write,
        offset, cb, ptr, num_events_in_wait_list , event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error write buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}

void PositionTrack3D::readBuffer(cl_mem buffer, cl_bool blocking_read, size_t offset, 
      size_t cb, void *ptr, cl_uint num_events_in_wait_list,
         const cl_event *event_wait_list, cl_event *event, string errorMsg) {
   int ret = clEnqueueReadBuffer(opencl_manager->getCommandQueue(), buffer, blocking_read,
         offset, cb, ptr, num_events_in_wait_list, event_wait_list, event);
   if (ret != CL_SUCCESS) {
      cout << "Error read buffer: " << ret << " : " << errorMsg << endl;
      exit(0);
   }
}


