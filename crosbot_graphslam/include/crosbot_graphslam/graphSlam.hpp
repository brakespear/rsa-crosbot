/*
 * graphSlam.hpp
 *
 * Created on: 24/12/2013
 *     Author: adrianr
 */

#ifndef GRAPHSLAM_HPP_
#define GRAPHSLAM_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot/thread.hpp>
#include <crosbot_map/localmap.hpp>
#include <crosbot_map/tag.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <crosbot_graphslam/graphSlamNode.hpp>
#include <crosbot_graphslam/localMap.hpp>

using namespace crosbot;
using namespace std;

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI

class SlamSnap {
public:
   SnapPtr snap;
   int localMapIndex;
   Pose localOffset;
};

class SlamHistory {
public:
   Pose localPose;
   int localMapIndex;
   ros::Time stamp;
   Pose currentSlamPose;
};


class GraphSlamNode;
class GraphSlam {

public:

   /*
    * Config attributes for graph slam
    */
   //total width and length of the global map in metres
   double GlobalMapSize;
   //width and length of a map cell in metres
   double CellSize;
   //Minimum and maximum heights a laser point can have
   double MinAddHeight;
   double MaxAddHeight;
protected:
   //Time in us between transfers of global map images
   int ImgTransmitTime;
   //Maximum and minimum distance laser points can have
   double LaserMinDist;
   double LaserMaxDist;
   //Width and height of a local map in metres
   double LocalMapSize;
   //Distance the robot can travel inside a local map
   double LocalMapDistance;
   //Number of local map grid squares searched in each direction for
   //an ICP match
   int SearchSize;
   //Maximum distance points can be apart to be considered matching
   //in the ICP process
   double MaxAlignDistance;
   //L value for ICP
   double LValue;
   //Fudge factor used to scale information matrix of constraint into
   //covariance matrix that corresponds to distance between scans
   int InformationScaleFactor;
   //Maximum covariance constraints can have. Should roughly correspond
   //to maximum error position tracker is likely to habe inside a local  map
   double MaxCovar;
   //Threshold used for histogram correlation in loop closing
   //4 is a perfect match. 3.3 - 3.6 is normally good
   double CorrelationThreshold;
   //Number of matching points in the icp of loop closing needed for
   //each iteration of icp
   int MinGoodCount;
   //Number of matching points in the icp of loop closing needed for
   //maps to be considered matched
   int FinalMinGoodCount;
   //Max number of iterations used in the ICP alignment of local maps
   int MaxIterations;
   //Max errors in an iteration of ICP for the match to be considered
   //matched
   double MaxErrorTheta;
   double MaxErrorDisp;
   //Maximum number of constraints possible
   int MaxNumConstraints;
   //Maximum number of loop closing constraints possible
   int MaxNumLoopConstraints;
   //If should turn combining of local maps on
   bool LocalMapCombine;
   //Maximum value theta can change during a loop closure.
   //A negative value is no maximum
   double MaxThetaOptimise;
   //Time in us between robot poses being added to the history
   int HistoryTime;
   //Number of times a cell has to be observed before being added to the local map
   int MinObservationCount;
   //Initial height of the robot
   double InitHeight;
   //Should local maps be warped?
   bool LocalMapWarp;
   //Threshold for which scoring a match against free areas is considered
   //acceptable
   double FreeAreaThreshold;
   //Should temp loop closures be used
   bool UseTempLoopClosures;
   //Distance away from occupied cell that the probbability of the cell being free is 1
   double FreeAreaDistanceThreshold;
   //Stop loop closures with a residual close to 180 degrees in case the
   //operating environment has sections which are symetrical
   bool PreventMatchesSymmetrical;

   //Amount the info matrix of each scan should be scaled by
   double PerScanInfoScaleFactor;
   //Threshold that points are skipped if their x or y partial gradients are too large
   double GradientDistanceThreshold;
   //Total covariance at which a new local map will be created
   double LocalMapCovarianceThreshold;
   //Number of iterations of the graph optimiser
   int MaxNumOfOptimisationIts;
   //Amount a local maps moves before it is evaluated for temp loop closures
   double LargeMovementThreshold;
   //amount two local maps have to overlap to be considered potentially matching
   int OverlapThreshold;
   //Amount of movement away from a temp constraint for it to be deleted
   double TempConstraintMovementXY;
   double TempConstraintMovementTh;
   //Maximum distance apart the centre of two maps can be to be examined for a temp loop
   //closure
   double DistanceOverlapThreshold;
   //Amount the last scan of a local map needs to be corrected before the local map is warped
   double LocalMapWarpThreshXY;
   double LocalMapWarpThreshTh;
   //Maximum movement of a map for the graph optimsation to be considered
   //to converge
   double MaxOptMoveXY;
   double MaxOptMoveTh;
   //If should use a least squares optimiser. If false, will use gradient descent
   bool UseLeastSquaresOptimisation;


   //Kinect Params
   
   //Width and height of RGBD sensor
   int RGBDWidth;
   int RGBDHeight;
   //Number of pixels to skip when saving scans
   int SkipVal;
   //Minimum and maximum heights of pixels to include in 3D map
   double RGBDMinHeight;
   double RGBDMaxHeight;
   //Maximum and minimum distance of pixels from rgbd sensor to include in 3D map
   double RGBDMaxDistance;
   double RGBDMinDistance;

   //Kinect data structure
   typedef struct {
      vector<Point> points;
      vector<float> rgb;
      int localMapIndex;
      int scanIndex;
   } KinectScan;

   /*
    * Other fields
    */
   /*
    * The time the last image was grabbed from slam
    */
   Time lastImgTime;

   //Lock to stop snaps interferring with local map changes
   ReadWriteLock masterLockSmith;

   //Index of current local map
   int currentLocalMap;
   //the parent of the current local map in the tree
   int parentLocalMap;
   //Index of next local map that will be used
   int nextLocalMap;
   //ICP pose of current local map
   Pose currentLocalMapICPPose;
   //ICP pose of last iteration
   Pose oldICPPose;
   //List of all stored snaps
   vector<SlamSnap*> snaps;
   //Time the last pose history was stored
   Time lastHistoryTime;
   //History of slam poses relative to their local maps
   vector<SlamHistory> historySlam;
   //History of actual slam poses
   vector<geometry_msgs::PoseStamped> historySlamPoses;
   //Number of points that were used the last time the global map was drawn
   int lastDrawnGlobalPoints;
   //Number of points in the global map not counting the points in the current
   //local map
   int numGlobalPoints;
   //Status of local maps being combined
   int combineMode;
   //Total number of constraints in the graph
   int numConstraints;

   //Kinect variables
   vector<KinectScan *> kinectScans;
   int lastCloudPublished;
   int messageSize;
   int activeMapIndex;

   /*
    * Gets the global map in the standard crosbot format
    */
   virtual void getGlobalMap(vector<LocalMapPtr> curMap, vector<double> mapSlices) = 0;

   /*
    * Gets the current global x,y,theta of a local map
    */
   virtual void getGlobalMapPosition(int mapIndex, double& gx, double& gy,
         double& gth) = 0;

   /*
    * Gets the index of the scan inside the active local map
    */
   virtual int getScanIndex(int mapIndex) = 0;

   /*
    * Gets the pose of the scan relative to the origian of the associated local map
    */
   virtual void getScanPose(int mapIndex, int scanIndex, double& px, 
         double& py, double& pth) = 0;

   /*
    * Adds the current pose to the history
    */
   void addPoseHistory(Pose icpPose);

   /*
    * After maps have been combined, the snaps and slam history nodes from the map being
    * deleted need to be moved to the local map being combined
    */
   void fixSnapPositions(int combineIndex, double alignX, double alignY, double alignTh);
   void fixSlamHistoryPositions(int combineIndex, double alignX, double alignY, double alignTh);

public:
   GraphSlamNode *graphSlamNode;

   Pose slamPose;
   /*
    * Has graph slam finished setting up?
    */
   bool finishedSetup;
   //Number of cells across in local and global occupancy grids
   int DimLocalOG;
   int DimGlobalOG;


   /*
    * Initialise the position tracker
    */
   virtual void initialise(ros::NodeHandle &nh);

   /*
    * Start graph slam. After this is called, graph slam
    * is ready to take scans
    */
   virtual void start() = 0;

   /*
    * Stop graph slam
    */
   virtual void stop() = 0;

   /*
    * Called for processing the first scan
    */
   virtual void initialiseTrack(Pose icpPose, PointCloudPtr cloud) = 0;

   /*
    * Update graph slam with the lastest scan
    */
   virtual void updateTrack(Pose icpPose, PointCloudPtr cloud, ros::Time stamp) = 0;

   /*
    * Grabs the current map
    */
   crosbot::ImagePtr drawMap(vector<LocalMapPtr> globalMaps, Pose icpPose, vector<double> mapSlices);
   /*
    * Adds a snap to the list
    */
   void addSnap(SnapPtr newSnap);

   /*
    * Gets the current list of snaps
    */
   void getSnaps(vector<SnapPtr>& list);

   /*
    * Updates a snap
    */
   bool updateSnap(uint32_t id, uint8_t type, string description, uint8_t status);

   /*
    * Gets a particular snap
    */
   bool getSnap(uint32_t id, uint8_t type, SnapPtr& snap);

   /*
    * Gets a history of past slam poses
    */
   vector<geometry_msgs::PoseStamped>& getSlamHistory();

   /*
    * Adds the track the robot took to the image displayed
    */
   void addSlamTrack(ImagePtr mapImage);

   /*
    * Invet a 3x3 matrix
    */
   void invert3x3Matrix(double m[3][3], double res[3][3]);

   //Kinect added
   //Updates the position of kinect points and adds them to the point cloud
   void getPoints(vector<uint8_t>& points);
   //Stores the points from a kinect scan
   void captureScan(const vector<uint8_t>& points, Pose correction);

   //Debugging functions
   LocalMapPtr testMap;

private:

};

#endif
