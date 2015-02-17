/*
 * graphSlamFull3D.hpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#ifndef GRAPHSLAMFULL3D_HPP_
#define GRAPHSLAMFULL3D_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot/thread.hpp>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <crosbot_graphslam/localMap.hpp>
#include <crosbot_3d_graphslam_full/graphSlamFull3DNode.hpp>

using namespace std;
using namespace crosbot;

class GraphSlamFull3DNode;

class LocalMaps {
public:

   tf::Transform pose;
   pcl::PointCloud<pcl::PointNormal>::Ptr cloud;

   int parentIndex;
   double parentInfo[6][6];
   tf::Transform parentOffset;


   LocalMaps(LocalMapInfoPtr inMap, int pIndex, pcl::PointCloud<pcl::PointNormal>::Ptr pts);
};

class LoopConstraint {
public:
   int i;
   int j;
   tf::Transform offset;
   double info[6][6];
};

class GraphSlamFull3D {
public:
   GraphSlamFull3D();

   /*
    * Initialise parameters
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Start 3d graph slam
    */
   void start();

   /*
    * Shutdown node
    */
   void stop();

   /*
    * Create a new local map
    */
   void newLocalMap(LocalMapInfoPtr localMapInfo);

   /*
    * A list of new positions of local maps
    */
   void haveOptimised(vector<LocalMapInfoPtr> newMapPositions, vector<int> iNodes, vector<int> jNodes,
         bool wasFullLoop);

   GraphSlamFull3DNode *graphSlamFull3DNode;
private:

   /*
    * Configuration parameters
    */
   //Maximum corresponding distance allowed squared
   double MaxCorrespDist;
   //Only will use every skipPoints when calculating info matrixes and doing icp
   int SkipPoints;
   //Search precision for nearest neighbour searches
   double KDTreeEpsilon;
   //Max iterations in ICP
   int MaxIterations;
   //Minimum number of points needed for ICP to be successful
   int MinCount;
   //Thresholds for accepting point correspondances in ICP
   double StartDistThresh;
   double StartDotThresh;
   double EndDistThresh;
   double EndDotThresh;
   //When movements falls below this, ICP will stop
   double MoveThresh;

   //ReadWriteLock masterLock;
   bool receivedOptimisationRequest;
   int parentIndex;
   int currentIndex;
   vector<LocalMaps *> localMaps;


   //Temp stores
   vector<int> nearestPoint;
   vector<float> nearestDist;
  
   //Add the points and normals to a pcl point cloud 
   pcl::PointCloud<pcl::PointNormal>::Ptr addPointsToCloud(
         PointCloudPtr cloud, PointCloudPtr normal);

   inline pcl::PointNormal getClosestPoint(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree, 
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, tf::Vector3 p);

   void calculateInfo(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform diff,
         double info[6][6]);

   tf::Transform performICP(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform initTrans);

   int performICPIteration(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform trans,
         double A[6][6], double b[6], double distThresh, double dotThresh, int numSkip);

   void solveCholesky(double A[6][6], double b[6], double x[6]);

};
   

#endif
