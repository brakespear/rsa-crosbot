/*
 * graphSlam3DCPU.hpp
 *
 * Created on: 17/09/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAM3DCPU_HPP_
#define GRAPHSLAM3DCPU_HPP_

#include <crosbot_3d_graphslam/graphSlam3D.hpp>

using namespace std;
using namespace crosbot;

class GraphSlam3DCPU : public GraphSlam3D {
public:
   //Inherited methods from graphSlam3D.hpp
   void initialise(ros::NodeHandle &nh);
   void start();
   void stop();
   void addFrame(DepthPointsPtr depthPoints, Pose sensorPose, Pose slamPose);
   void newLocalMap(LocalMapInfoPtr localMapInfo);
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions);

   GraphSlam3DCPU();
   ~GraphSlam3DCPU();

private:
   /*
    * CPU config params
    */
   //Minimum number of observations in a cell needed before considered occupied
   int ObsThresh;


   VoxelGrid *localMap;
   vector<Local3DMap *> maps;
   int currentMap;

   bool done;

};

#endif


