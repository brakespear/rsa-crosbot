/*
 * graphSlamDisplay.hpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#ifndef GRAPHSLAMDISPLAY_HPP_
#define GRAPHSLAMDISPLAY_HPP_

#include <ros/ros.h>
#include <crosbot/utils.hpp>
#include <crosbot/data.hpp>
#include <crosbot/thread.hpp>
#include <crosbot_graphslam/localMap.hpp>

#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/vtk_io.h>
//#include <pcl/surface/mls.h>
//#include <pcl/surface/impl/mls.hpp>

using namespace std;
using namespace crosbot;

class GraphSlamDisplay : public Thread {
public:
   /*
    * Configuration parameters
    */
   //Should the full point cloud be published?
   bool PublishPointCloud;
   //Reconstruct surfaces from the point cloud
   bool CreateMesh;


   GraphSlamDisplay();

   /*
    * Initialise parameters
    */
   void initialise(ros::NodeHandle &nh);

   /*
    * Shutdown node
    */
   void stop();

   /*
    * Adds a new local map with its points
    */
   void addMap(LocalMapInfoPtr localMapPoints);

   /*
    * Corrects positions of local maps when the map has been optimised
    */
   void correctMap(vector<LocalMapInfoPtr> newMapPositions);

   /*
    * Returns the point cloud of entire map
    */
   PointCloud &getPointCloud();

private:

   /*
    * Total world points
    */
   PointCloud points;

   /*
    * Mesh vector
    */
   vector<pcl::PolygonMesh *> meshes;

   pcl::visualization::PCLVisualizer *viewer;


   Mutex viewerLock;
   bool viewerUpdate;
   pcl::PolygonMesh *currentMesh;
   int currentMeshIndex;
   /*
    * Runs the pcl visualiser in its own thread
    */
   void run();
};
   

#endif
