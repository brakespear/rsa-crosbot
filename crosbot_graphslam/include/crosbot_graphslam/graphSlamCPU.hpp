/*
 * GraphSlamCPU.hpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */

#ifndef GRAPHSLAM_CPU_HPP_
#define GRAPHSLAM_CPU_HPP_

#include <crosbot_graphslam/graphSlam.hpp>

class GraphSlamCPU : public GraphSlam {
public:
   /*
    * Config attributes for CPU version of graph slam
    */

   //Inherited methods from graphSlam.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void initialiseTrack(Pose icpPose, PointCloudPtr cloud);
   void updateTrack(Pose icpPose, PointCloudPtr cloud);

   GraphSlamCPU();
   ~GraphSlamCPU();
protected:
   void getGlobalMap(LocalMapPtr curMap);
private:

   //debugging for timings
   ros::WallDuration totalTime;
   int numIterations;

};

#endif
