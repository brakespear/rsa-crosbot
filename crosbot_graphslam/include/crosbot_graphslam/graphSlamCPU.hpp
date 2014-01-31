/*
 * GraphSlamCPU.hpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */

#ifndef GRAPHSLAM_CPU_HPP_
#define GRAPHSLAM_CPU_HPP_

#include <crosbot_graphslam/graphSlam.hpp>

#define NUM_ORIENTATION_BINS 64
#define NUM_PROJECTION_BINS 100
#define MAX_LOCAL_POINTS 2000
#define MAX_POTENTIAL_MATCHES 50

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
   void updateTrack(Pose icpPose, PointCloudPtr cloud, ros::Time stamp);

   GraphSlamCPU();
   ~GraphSlamCPU();
protected:
   void getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices);
   void getGlobalMapPosition(int mapIndex, double& gx, double& gy, 
         double& gth);
private:
   //Kinect data structure
   typedef struct {
      vector<Point> points;
      vector<float> rgb;
      int localMapIndex;
      int scanIndex;
   } KinectScan;

   //Slam data structures:
   typedef struct {
      vector<Point> points;
      double covar[3][3];
      double pose[3];
      double correction[3];
      ros::Time stamp;
   } Scan;

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
      double internalCovar[3][3];

      int numConstraints;
      int indexNextNode;
      double nextOffsetX;
      double nextOffsetY;
      double nextOffsetTh;

      double mapCentreX;
      double mapCentreY;
      double robotMapCentreX;
      double robotMapCentreY;

      double orientationHist[NUM_ORIENTATION_BINS];
      double projectionHist[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS];
      double entropyHist[NUM_ORIENTATION_BINS];

      double *freeArea;

      double pointsX[MAX_LOCAL_POINTS];
      double pointsY[MAX_LOCAL_POINTS];
      double pointsZ[MAX_LOCAL_POINTS];

      double warpPointsX[MAX_LOCAL_POINTS];
      double warpPointsY[MAX_LOCAL_POINTS];
      double warpPointsZ[MAX_LOCAL_POINTS];
      int lastObserved[MAX_LOCAL_POINTS];
      int numWarpPoints;

      vector<Scan *> scans;
   } LocalMap;
   vector<LocalMap> localMaps;

   typedef struct {
      Point p;
      double gradX;
      double gradY;
      int count;
   } GridCell;

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
      int potentialMatches[MAX_POTENTIAL_MATCHES];
      double potentialMatchX[MAX_POTENTIAL_MATCHES];
      double potentialMatchY[MAX_POTENTIAL_MATCHES];
      double potentialMatchTh[MAX_POTENTIAL_MATCHES];
      int potentialMatchParent[MAX_POTENTIAL_MATCHES];
      int numIterations;
      double A[3][3];
      double B[3];
      int goodCount;
      int numConstraints;
      int numLoopConstraints;
      double histCos[NUM_ORIENTATION_BINS];
      double histSin[NUM_ORIENTATION_BINS];
      double scaleFactor[3];

      int activeCells[MAX_LOCAL_POINTS];
      int *localOG;
      GridCell *grid;

      int *constraintType;
      int *constraintIndex;
      int *loopConstraintParent;
      int *loopConstraintI;
      int *loopConstraintJ;
      double *loopConstraintXDisp;
      double *loopConstraintYDisp;
      double *loopConstraintThetaDisp;
      double (*loopConstraintInfo)[3][3];
      double (*graphHessian)[3];
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
   //Creates and sets up a new local map
   void createNewLocalMap(int oldLocalMap, int newLocalMap, int parentLocalMap, double angleError, double icpTh);
   //Converts point pX, pY to the reference frame of another local map
   void convertReferenceFrame(double pX, double pY, double offsetX, double offsetY,
         double cosTh, double sinTh, double *pointX, double *pointY);
   //Finds the best matching point in the occupancy grid of the current local map
   int findMatchingPoint(double pointX, double pointY, int searchFactor);
   //Gets the mbicp metric vallue for the match between laser point and og point
   double getMetricValue(double pointX, double pointY, double ogPointX, double ogPointY);
   //Multiply two 3x3 matrices
   void mult3x3Matrix(double a[3][3], double b[3][3], double res[3][3]);
   //Calculate the inverse of a 3x3 matrix
   void invert3x3Matrix(double m[3][3], double res[3][3]);
   // Nasty hack function to make covariance matrices acceptable in the occassional 
   // case where not enough matching points were found when creating the 
   // information matrix
   void covarFiddle(double m[3][3]);
   //Converts a point from local map coords to global coords  
   void convertToGlobalCoord(double x, double y, double localPosX,
      double localPosY, double localPosTh, double *resX, double *resY);
   //Finds if an index in a histogram correlation is a peak
   bool findIfPeak(double *corr, int i);
   //Correlates two projection histogram sets
   double correlateProjection(double proj1[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS],
         double proj2[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS], int startIndex,
         int offset, int *maxIndex);
   //gets part of the global position of node
   double getGlobalPosIndex(int node, int index);

   /*
    * Loop closing methods
    */
   //Calculates the information matrix between the current local map
   //and its parent
   void getHessianMatch(int constraintIndex);
   //Finishes a local map - finalises the histograms and hessian matrices
   void prepareLocalMap();
   //Finds potentially matching local maps by position and histogram correlation
   void findPotentialMatches();
   //Perform an icp match between the current local map and otherMap
   void alignICP(int otherMap, int matchIndex);
   //Calculates the overall icp alignment matrix from alignICP
   void calculateICPMatrix(int matchIndex);
   //Finalises the information matrix of a loop constraint
   void finaliseInformationMatrix();
   //Calculates the global hessian matrix for the map optimisation
   void getGlobalHessianMatrix();
   //Performs the optimisation of the graph
   void calculateOptimisationChange(int numIterations);
   //Updates the global positions of all local maps
   void updateGlobalPositions();
   //Updates the global covariances of each local map and updates
   //the global positions of all laser points
   void updateGlobalMap();
   //Combines two local maps
   void combineNodes(double alignError, int numOtherGlobalPoints);

   void warpLocalMap(int mapIndex, double errX, double errY, double errTh);

   void addToFreeArea(double px, double py);


   //Debugging publisher
   void updateTestMap();

   //Kinect bit
   vector<KinectScan *> kinectScans;
   int lastCloudPublished;
   int messageSize;
   int activeMapIndex;
   void getPoints(vector<uint8_t>& points);
   void captureScan(const vector<uint8_t>& points, Pose correction);

};

#endif
