/*
 * graphSlamDisplay.cpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>
#include <crosbot/utils.hpp>
#include <sstream>

using namespace std;
using namespace crosbot;

GraphSlamDisplay::GraphSlamDisplay() {

}

void GraphSlamDisplay::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   
   paramNH.param<bool>("PublishPointCloud", PublishPointCloud, false);
   paramNH.param<bool>("CreateMesh", CreateMesh, true);

   viewerUpdate = false;
   //Only strat the pcl viewer if want to reconstruct the surface
   if (CreateMesh) {
      start(); //starts the visualiser thread
   }
}

void GraphSlamDisplay::stop() {
}

void GraphSlamDisplay::addMap(LocalMapInfoPtr localMapPoints) {
   points.timestamp = localMapPoints->timestamp;
   points.frameID = localMapPoints->cloud->frameID;

   int startIndex = points.cloud.size();
   if (PublishPointCloud) {
      points.cloud.resize(startIndex + localMapPoints->cloud->cloud.size());
      points.colours.resize(startIndex + localMapPoints->cloud->colours.size());
   }

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

   tf::Transform mapPose = localMapPoints->pose.toTF();

   for (int i = 0; i < localMapPoints->cloud->cloud.size(); i++) {
      Point point = mapPose * localMapPoints->cloud->cloud[i].toTF();
      if (PublishPointCloud) {
         point.z -= 1.0;
         points.cloud[startIndex + i] = point;

         points.colours[startIndex + i] = localMapPoints->cloud->colours[i];
      }

      if (CreateMesh) {
         pcl::PointXYZRGB p(localMapPoints->cloud->colours[i].r, localMapPoints->cloud->colours[i].g, localMapPoints->cloud->colours[i].b);
         p.x = point.x;
         p.y = point.y;
         p.z = point.z;

         cloud->push_back(p);
      }
   }

   if (CreateMesh) {

      //TODO: if store normals in existing code, consider passing these in the message to avoid having to calculate them
      //again (as well as more efifcient, will also probably be more accurate)

      //Test stuff.....
      //Normal estimation
      pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
      pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);
      //* normals should not contain the point normals + surface curvatures

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      //* cloud_with_normals = cloud + normals

      // Create search tree*
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
      pcl::PolygonMesh *triangles = new pcl::PolygonMesh();

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (2); //0.025 in example code

      // Set typical values for the parameters
      //maximum acceptabole distance to search for neighbours relative to closest neighbour
      gp3.setMu (3);  //2.5 in example code
      //How many neighbours are searched for
      gp3.setMaximumNearestNeighbors (100); 
      //Min and max allowable angle in each triangle
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      //Deals with sharp edges or corners. Points are not connected if normals deviate by more than
      //the max surface angle. normal consistency to false means this ignores normal direction
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (*triangles);

      meshes.push_back(triangles);

      viewerLock.lock();
      viewerUpdate = true;
      currentMesh = triangles;
      currentMeshIndex = localMapPoints->index;
      viewerLock.unlock();

      //TODO: implement saving the entire mesh to file if selected by a parameter
      //Can view files made by the following command by using pcd_viewer
      //pcl::io::saveVTKFile ("/home/adrianrobolab/groovy_workspace/crosbot/src/crosbot_3d_graphslam_display/mesh.vtk", triangles);
   }
}

void GraphSlamDisplay::correctMap(vector<LocalMapInfoPtr> newMapPositions) {
   //TODO: move the map positions and do warping
}

PointCloud &GraphSlamDisplay::getPointCloud() {
   return points;
}

void GraphSlamDisplay::run() {
   viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
   viewer->setBackgroundColor(0.0, 0.0, 0.0);
   viewer->initCameraParameters ();
   while(!viewer->wasStopped()) {
      viewer->spinOnce(1000);
      viewerLock.lock();
      if (viewerUpdate) {
         ostringstream ss;
         ss << currentMeshIndex;
         viewer->addPolygonMesh(*currentMesh, ss.str());
         viewerUpdate = false;
      }
      viewerLock.unlock();
   }
}

