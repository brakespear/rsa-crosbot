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
   void getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices);
   void getGlobalMapPosition(int mapIndex, double& gx, double& gy, 
         double& gth);
private:
   //Slam data structures:

   /*
    * Information stored for each local map
    */
   typedef struct {
      double currentGlobalPosX;
      double currentGlobalPosY;
      double currentGlobalPosTh;

      double parentOffsetX;
      double parentOffsetY;
      double parentOffsetTh;

      int numPoints;
      int indexParentNode;
      int treeLevel;
      double parentInfo[3][3];
      double changeInPos[3];
      double globalCovar[3][3];

      double mapCentreX;
      double mapCentreY;
      double robotMapCentreX;
      double robotMapCentreY;

      double orientationHist[NUM_ORIENTATION_BINS];
      double projectionHist[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS];
      double entropyHist[NUM_ORIENTATION_BINS];

      double pointsX[MAX_LOCAL_POINTS];
      double pointsY[MAX_LOCAL_POINTS];
      double pointsZ[MAX_LOCAL_POINTS];
   } LocalMap;
   vector<LocalMap> localMaps;

   /*
    * General information needed for graph slam
    */
   typedef struct {
      double currentOffsetX;
      double currentOffsetY;
      double currentOffsetTh;
      double startICPTh;

      double minMapRangeX;
      double minMapRangeY;
      double maxMapRangeX;
      double maxMapRangeY;
      int infoCount;
      int matchSuccess;
      int combineIndex;
      int combineMode;
      int numPotentialMatches;
      int *potentialMatches;
      double *potentialMatchX;
      double *potentialMatchY;
      double *potentialMatchTh;
      int *potentialMatchParent;
      int numIterations;
      double A[3][3];
      double B[3];
      int goodCount;
      int numConstraints;
      int numLoopConstraints;
      double histCos[NUM_ORIENTATION_BINS];
      double histSin[NUM_ORIENTATION_BINS];
      double scaleFactor[3];

      int *localOG;
      int *localOGCount;
      double *localOGZ;
      double pointsNxtX[MAX_LOCAL_POINTS];
      double pointsNxtY[MAX_LOCAL_POINTS];

      vector<int> constraintType;
      vector<int> constraintIndex;
      vector<int> loopConstraintParent;
      vector<int> loopConstraintI;
      vector<int> loopConstraintJ;
      vector<double> loopConstraintXDisp;
      vector<double> loopConstraintYDisp;
      vector<double> loopConstraintThetaDisp;
      vector<double[3][3]> loopConstraintInfo;
      vector<double[3]> graphHessian;
   } SlamCommon;
   SlamCommon *common;

   //The current global map
   vector<int> globalMap;
   vector<double> globalMapHeights;
   //Should the map be totally redrawn?
   bool resetMap;
   //Offset of the robot in the current local map relative to ICP frame 
   double offsetFromParentX;
   double offsetFromParentY;
   double offsetFromParentTh;


   //debugging for timings
   ros::WallDuration totalTime;
   int numIterations;


   /*
    * Private methods
    */

   //Performs loop closing tests and sets up a new local map if needed
   void finishMap(double angleError, double icpTh, Pose icpPose);
   //Clears a local map for use
   void clearMap(int mapIndex);
   //returns the index of a 2D point in a local map, or -1 if the point does not fit inside the map
   int getLocalOGIndex(double x, double y);
   //converts to x and y coord of a local map to its global index
   int convertToGlobalPosition(double x, double y, int mapIndex, double cosTh, double sinTh);
   void createNewLocalMap(int oldLocalMap, int newLocalMap, int parentLocalMap, double angleError, double icpTh);

};

#endif
