/*
 * graphSlamDisplay.cpp
 *
 * Created on: 13/08/2014
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_display/graphSlamDisplay.hpp>
#include <crosbot/utils.hpp>
#include <sstream>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI

using namespace std;
using namespace crosbot;

GraphSlamDisplay::GraphSlamDisplay() {

}

void GraphSlamDisplay::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   
   paramNH.param<bool>("PublishPointCloud", PublishPointCloud, false);
   paramNH.param<bool>("CreateMesh", CreateMesh, true);
   paramNH.param<bool>("WarpMaps", WarpMaps, true);

   viewerUpdate = false;
   viewerUpdateOptimise = false;
   //Only start the pcl viewer if want to reconstruct the surface
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
      //again (as well as more efficient, will also probably be more accurate)
      
      /*pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      //pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_with_normals;
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> mls;
      mls.setComputeNormals (true);
      // Set parameters
      mls.setInputCloud (cloud);
      mls.setPolynomialFit (false);
      mls.setSearchMethod (tree);
      mls.setSearchRadius (0.3);
      // Reconstruct
      mls.process (*cloud_with_normals);*/

      
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
      // normals should not contain the point normals + surface curvatures

      // Concatenate the XYZ and normal fields*
      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
      pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
      // cloud_with_normals = cloud + normals
      

      // Create search tree*
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      tree2->setInputCloud (cloud_with_normals);

      // Initialize objects
      pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
      pcl::PolygonMesh *triangles = new pcl::PolygonMesh();

      // Set the maximum distance between connected points (maximum edge length)
      gp3.setSearchRadius (0.1); //0.025 in example code //was 2

      // Set typical values for the parameters
      //maximum acceptabole distance to search for neighbours relative to closest neighbour
      gp3.setMu (2);  //2.5 in example code  //was 2.5
      //How many neighbours are searched for
      gp3.setMaximumNearestNeighbors (100); 
      //Min and max allowable angle in each triangle
      gp3.setMinimumAngle(M_PI/18); // 10 degrees
      gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
      //Deals with sharp edges or corners. Points are not connected if normals deviate by more than
      //the max surface angle. normal consistency to false means this ignores normal direction
      gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees. was pi/4
      gp3.setNormalConsistency(false);

      // Get result
      gp3.setInputCloud (cloud_with_normals);
      gp3.setSearchMethod (tree2);
      gp3.reconstruct (*triangles);

      viewerLock.lock();
      LocalMap newMap;
      newMap.pose = localMapPoints->pose;
      newMap.mesh = triangles;
      maps.push_back(newMap);

      viewerUpdate = true;
      currentMesh = triangles;
      currentMeshIndex = localMapPoints->index;
      viewerLock.unlock();

      //testing
      if(maps.size() == 2) {
         cout << "Dummy move now" << endl;
         Pose newPose2 = maps[1].pose;
         double y,p,r;
         newPose2.getYPR(y,p,r);
         newPose2.setYPR(y + M_PI/4.0, p, r);
         //newPose2.position.x += 0.5;
         vector<LocalMapInfoPtr> newMapPos;
         newMapPos.push_back(new LocalMapInfo(newPose2, 1));
         correctMap(newMapPos);
      }

      //TODO: implement saving the entire mesh to file if selected by a parameter
      //Can view files made by the following command by using pcd_viewer
      //pcl::io::saveVTKFile ("/home/adrianrobolab/groovy_workspace/crosbot/src/crosbot_3d_graphslam_display/mesh.vtk", triangles);
   }
}

void GraphSlamDisplay::correctMap(vector<LocalMapInfoPtr> newMapPositions) {

   cout << "Correcting map" << endl;

   int i;
   int maxI = -1;
   vector<bool> newPos;
   vector<int> newMapPosIndex;
   vector<bool> poseChanged;
   newPos.resize(maps.size());
   newMapPosIndex.resize(maps.size());
   poseChanged.resize(maps.size());
   for (int i = 0; i < newPos.size(); i++) {
      newPos[i] = false;
      newMapPosIndex[i] = -1;
      poseChanged[i] = false;
   }
   for (int i = 0; i < newMapPositions.size(); i++) {
      if (newMapPositions[i]->index < newPos.size()) {
         newPos[newMapPositions[i]->index] = true;
         newMapPosIndex[newMapPositions[i]->index] = i;
         poseChanged[newMapPositions[i]->index] = hasPositionChanged(
               maps[newMapPositions[i]->index].pose, newMapPositions[i]->pose);
         if (newMapPositions[i]->index > maxI) {
            maxI = newMapPositions[i]->index;
         }
      } else {
         cout << "Map in optimise list hasn't been received yet" << endl;
      }
   }
   vector<bool> hasChanged;
   hasChanged.resize(maps.size());
   for (int i = 0; i < newPos.size(); i++) {
      hasChanged[i] = false;
      if ((i == newPos.size() - 1 || i == maxI) && poseChanged[i]) {
         Pose newStart = newMapPositions[newMapPosIndex[i]]->pose;
         repositionMap(i, newStart, newStart, false);
         hasChanged[i] = true;
      } else if (i < newPos.size() - 1) {
         if (!WarpMaps && poseChanged[i]) {
            Pose newStart = newMapPositions[newMapPosIndex[i]]->pose;
            repositionMap(i, newStart, newStart, false);
            hasChanged[i] = true;
         } else if (WarpMaps && (poseChanged[i] || poseChanged[i+1])) {
            Pose newStart;
            if (newPos[i]) {
               newStart = newMapPositions[newMapPosIndex[i]]->pose;
            } else {
               newStart = maps[i].pose;
            }
            Pose newEnd;
            if (newPos[i+1]) {
               newEnd = newMapPositions[newMapPosIndex[i+1]]->pose;
            } else {
               newEnd = maps[i+1].pose;
            }
            repositionMap(i, newStart, newEnd, true);
            hasChanged[i] = true;
         }
      }
   } 
   
   viewerLock.lock();
   for(i = 0; i < hasChanged.size(); i++) {
      if (hasChanged[i]) {
         cout << "Map: " << i << " has changed" << endl;
         if (newPos[i]) {
            maps[i].pose = newMapPositions[newMapPosIndex[i]]->pose;
         }
         mapsChanged.push_back(i);
      }
   }
   viewerUpdateOptimise = true;
   viewerLock.unlock();
}

bool GraphSlamDisplay::hasPositionChanged(Pose oldPose, Pose newPose) {
   double yo, po, ro;
   oldPose.getYPR(yo, po, ro);
   double yn, pn, rn;
   newPose.getYPR(yn, pn, rn);
   double distThresh = 0.01;
   double rotThresh = 0.01;

   double yc = fabs(yn - yo);
   double pc = fabs(pn - po);
   double rc = fabs(rn - ro);
   ANGNORM(yc);
   ANGNORM(pc);
   ANGNORM(rc);

   if (fabs(newPose.position.x - oldPose.position.x) > distThresh ||
      fabs(newPose.position.y - oldPose.position.y) > distThresh ||
      fabs(newPose.position.z - oldPose.position.z) > distThresh ||
      yc > rotThresh ||
      pc > rotThresh ||
      rc > rotThresh) {

      return true;
   } else {
      return true;
   }
}

inline void GraphSlamDisplay::rotPoseToVector(Pose &pose, tf::Vector3 &vec) {
   double y,p,r;
   pose.getYPR(y,p,r);
   vec[0] = y;
   vec[1] = p;
   vec[2] = r;
}

void GraphSlamDisplay::repositionMap(int index, Pose newStart, Pose newEnd, bool warpMap) {
   tf::Matrix3x3 rotM;
   tf::Vector3 transM;
   tf::Vector3 gM;

   tf::Vector3 newStartRot, newEndRot, oldStartRot, oldEndRot;
   tf::Vector3 newStartPos, newEndPos, oldStartPos, oldEndPos;
   rotPoseToVector(newStart, newStartRot);
   newStartPos = newStart.position.toTF();
   rotPoseToVector(newEnd, newEndRot);
   newEndPos = newEnd.position.toTF();
   rotPoseToVector(maps[index].pose, oldStartRot);
   oldStartPos = maps[index].pose.position.toTF();
   if (index + 1 < maps.size()) {
      rotPoseToVector(maps[index + 1].pose, oldEndRot);
      oldEndPos = maps[index + 1].pose.position.toTF();
   }
   cout << "new start " << newStartRot[0] << " " << newStartRot[1] << " " << newStartRot[2] << endl;
   cout << "new end " << newEndRot[0] << " " << newEndRot[1] << " " << newEndRot[2] << endl;
   cout << "old start " << oldStartRot[0] << " " << oldStartRot[1] << " " << oldStartRot[2] << endl;
   cout << "old end " << oldEndRot[0] << " " << oldEndRot[1] << " " << oldEndRot[2] << endl;

   if (!warpMap) {
      //Just transform the points according to the pose at the start of the map
      tf::Transform newTrans = newStart.toTF();
      tf::Transform oldTrans = maps[index].pose.toTF();
      gM = oldTrans.getOrigin();
      tf::Transform diff = oldTrans.inverseTimes(newTrans);
      transM = diff.getOrigin();
      rotM = diff.getBasis();
   }
   pcl::PolygonMesh *mesh = maps[index].mesh;
   int sizeCloud = mesh->cloud.width * mesh->cloud.height;
   for (int i = 0; i < sizeCloud; i++) {
      const uint8_t *p = &(mesh->cloud.data[i * mesh->cloud.point_step]);
      float x,y,z;
      x = *(float*)(p);
      y = *(float*)(p + 4);
      z = *(float*)(p + 8);
      tf::Vector3 point;
      point[0] = x;
      point[1] = y;
      point[2] = z;

      if (warpMap) {
         tf::Vector3 a = point - oldStartPos;
         tf::Vector3 u = oldEndPos - oldStartPos;
         double l = u.length2();
         //fraction along the line
         double frac = a.dot(u) / l;
         /*if (frac < 0.0) frac = 0.0;
         if (frac > 1.0) frac = 1.0;*/

         gM = oldStartPos + frac * u;

         tf::Vector3 posNew = newStartPos + frac * (newEndPos - newStartPos);
         tf::Vector3 rot = newStartRot + frac * (newEndRot - newStartRot) - 
                           (oldStartRot + frac * (oldEndRot - oldStartRot));
         transM = posNew - gM;
         //cout << rot[2] << " " << rot[1] << " " << rot[0] << endl;
         rotM.setRPY(rot[2], rot[1], rot[0]);
      }

      //Convert the point to the new position
      tf::Vector3 newPoint = (rotM * (point - gM)) + gM + transM;

      x = newPoint[0];
      y = newPoint[1];
      z = newPoint[2];
      *(float*)(p) = x;
      *(float*)(p+4) = y;
      *(float*)(p+8) = z;
   }

   /*int sizeCloud = triangles->cloud.width * triangles->cloud.height;
   for (int j = 0; j < triangles->cloud.fields.size(); j++) {
      cout << "PCL fields: " << triangles->cloud.fields[j].offset << " " << 
      (triangles->cloud.fields[j].datatype == 7)
         << " " << triangles->cloud.fields[j].name << " " << triangles->cloud.fields[j].count << endl;
   }*/
   /*
    * PointCloud2 fields are all float32 and are:
    * x, off 0
    * y, off 4
    * z, off 8
    * rgb, off 32
    * normal x, off 16
    * normal y, off 20
    * normal z, off 24
    * curvature, off 36
    */
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
      if (viewerUpdateOptimise) {
         for (int i = 0; i < mapsChanged.size(); i++) {
            ostringstream ss;
            ss << mapsChanged[i];
            //viewer->updatePolygonMesh(*(maps[mapsChanged[i]].mesh), ss.str());
            viewer->removePolygonMesh(ss.str());
            viewer->addPolygonMesh(*(maps[mapsChanged[i]].mesh), ss.str());
         }
         mapsChanged.resize(0);
         viewerUpdateOptimise = false;
         viewerUpdate = false;
      }
      if (viewerUpdate) {
         /*viewer->removePolygonMesh();

         for (int i = 0; i < meshes.size() - 1; i++) {
            ostringstream ss;
            ss << i;
            viewer->removePolygonMesh(ss.str());
         }
         for (int i = meshes.size() - 1; i >= 0; i--) {
            ostringstream ss;
            ss << i;
            viewer->addPolygonMesh(*meshes[i], ss.str());
         }*/

         ostringstream ss;
         ss << currentMeshIndex;
         cout << "Adding the mesh " << ss.str() << endl;
         viewer->addPolygonMesh(*currentMesh, ss.str());
         viewerUpdate = false;
      }
      viewerLock.unlock();
   }
}

