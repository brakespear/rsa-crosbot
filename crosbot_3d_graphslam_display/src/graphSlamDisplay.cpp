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
   paramNH.param<bool>("UseVisualiser", UseVisualiser, true);

   viewerUpdate = false;
   viewerUpdateOptimise = false;
   //Only start the pcl viewer if want to reconstruct the surface
   if (UseVisualiser) {
      start(); //starts the visualiser thread
   }
}

void GraphSlamDisplay::stop() {

}

void GraphSlamDisplay::addMap(LocalMapInfoPtr localMapPoints) {
   points.timestamp = localMapPoints->timestamp;
   points.frameID = localMapPoints->cloud->frameID;
   //viewerLock.lock();
   int startIndex = points.cloud.size();
   if (PublishPointCloud) {
      points.cloud.resize(startIndex + localMapPoints->cloud->cloud.size());
      points.colours.resize(startIndex + localMapPoints->cloud->colours.size());
   }

   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
   pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);

   cloud->resize(localMapPoints->cloud->cloud.size());
   normals->resize(localMapPoints->normals->cloud.size());

   tf::Transform mapPose;
   if (maps.size() == 0) {
      mapPose = localMapPoints->pose.toTF();
   } else {
      tf::Transform changePose = prevIcpPose.inverse() * localMapPoints->icpPose.toTF();
      mapPose = maps[maps.size()-1].pose.toTF() * changePose;
   }
   prevIcpPose = localMapPoints->icpPose.toTF();


   for (int i = 0; i < localMapPoints->cloud->cloud.size(); i++) {
      Point point = mapPose * localMapPoints->cloud->cloud[i].toTF();
      Point normal = mapPose.getBasis() * localMapPoints->normals->cloud[i].toTF();
      if (PublishPointCloud) {
         point.z -= 1.0;
         points.cloud[startIndex + i] = point;

         points.colours[startIndex + i] = localMapPoints->cloud->colours[i];
      }

      //if (CreateMesh) {
         pcl::PointXYZRGB p(localMapPoints->cloud->colours[i].r, localMapPoints->cloud->colours[i].g, localMapPoints->cloud->colours[i].b);
         p.x = point.x;
         p.y = point.y;
         p.z = point.z;
         pcl::Normal n(normal.x, normal.y, normal.z);

         //cloud->push_back(p);
         (*cloud)[i] = p;
         (*normals)[i] = n;
      //}
   }

   if (/*CreateMesh && */cloud->size() > 0) {
      ros::WallTime t1 = ros::WallTime::now();

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
      /*pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> n;
      //pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
      pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
      tree->setInputCloud (cloud);
      n.setInputCloud (cloud);
      n.setSearchMethod (tree);
      n.setKSearch (20);
      n.compute (*normals);*/
      // normals should not contain the point normals + surface curvatures

      pcl::PolygonMesh *triangles = new pcl::PolygonMesh();
      
      if (CreateMesh) {
         // Concatenate the XYZ and normal fields*
         pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
         pcl::concatenateFields (*cloud, *normals, *cloud_with_normals);
         // cloud_with_normals = cloud + normals
         // Create search tree*
         pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
         tree2->setInputCloud (cloud_with_normals);

         // Initialize objects
         pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;

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
      } //else {
         //toPCLPointCloud2(*cloud_with_normals, triangles->cloud);
         
      //}

         /*delete *normals;
         delete *tree;
         delete *cloud_with_normals;
         delete *tree2;
         delete *cloud;*/

      if (localMapPoints->index != maps.size()) {
         cout << "We have a problem. Missing a map" << endl;
      }

      viewerLock.lock();
      LocalMap newMap;
      newMap.pose = mapPose;
      newMap.poseHistory = localMapPoints->poseHistory;
      newMap.timeHistory = localMapPoints->timeHistory;
      cout << "sizes are: " << localMapPoints->poseHistory.size() << " " << newMap.poseHistory.size() << endl;
      if (CreateMesh) {
         newMap.mesh = triangles;
         currentMesh = triangles;
      } else {
         newMap.cloud = cloud;
         currentCloud = cloud;
      }
      maps.push_back(newMap);

      viewerUpdate = true;

      currentMeshIndex = localMapPoints->index;
      viewerLock.unlock();
      
      ros::WallTime t2 = ros::WallTime::now();
      ros::WallDuration totalTime = t2 - t1;
      cout << "Time to create mesh: " << totalTime.toSec() * 1000.0f << endl;

      //testing
      /*if(maps.size() == 2) {
         cout << "Dummy move now" << endl;
         Pose newPose2 = maps[1].pose;
         //newPose2.position.z += 1.0;
         double y,p,r;
         newPose2.getYPR(y,p,r);
         newPose2.setYPR(y + M_PI/4.0, p, r);
         //newPose2.position.x += 0.5;
         vector<LocalMapInfoPtr> newMapPos;
         newMapPos.push_back(new LocalMapInfo(newPose2, 1));
         correctMap(newMapPos);
      }*/

      //TODO: implement saving the entire mesh to file if selected by a parameter
      //Can view files made by the following command by using pcd_viewer
      //pcl::io::saveVTKFile ("/home/adrianrobolab/groovy_workspace/crosbot/src/crosbot_3d_graphslam_display/mesh.vtk", triangles);
      
      //outputMapToFile("/home/adrianrobolab/mesh.vtk");
   } else {
      cout << "ERROR: Creating an empty mesh" << endl;
      LocalMap newMap;
      newMap.pose = mapPose;
      newMap.poseHistory = localMapPoints->poseHistory;
      newMap.timeHistory = localMapPoints->timeHistory;
      if (CreateMesh) {
         newMap.mesh = new pcl::PolygonMesh();
      } else {
         newMap.cloud = cloud;
      }
      maps.push_back(newMap);
   }
}

void GraphSlamDisplay::correctMap(vector<LocalMapInfoPtr> newMapPositions) {

   cout << "Correcting map" << endl;
   //viewerLock.lock();

   ros::WallTime t1 = ros::WallTime::now();
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
         repositionMap(i, newStart, newStart, newStart, false);
         hasChanged[i] = true;
      } else if (i < newPos.size() - 1) {
         if (!WarpMaps && poseChanged[i]) {
            Pose newStart = newMapPositions[newMapPosIndex[i]]->pose;
            repositionMap(i, newStart, newStart, newStart, false);
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
            Pose newEndNext;
            if (i < newPos.size() - 2) {
               if (newPos[i+2]) {
                  newEndNext = newMapPositions[newMapPosIndex[i+2]]->pose;
               } else {
                  newEndNext = maps[i+2].pose;
               }
            } else {
               newEndNext = newEnd;
            }
            repositionMap(i, newStart, newEnd, newEndNext, true);
            hasChanged[i] = true;
         }
      }
   } 
   ros::WallTime t2 = ros::WallTime::now();
   ros::WallDuration totalTime = t2 - t1;
   cout << "Time to create mesh: " << totalTime.toSec() * 1000.0f << endl;
   
   viewerLock.lock();
   for(i = 0; i < hasChanged.size(); i++) {
      if (hasChanged[i]) {
         //cout << "Map: " << i << " has changed" << endl;
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
   double distThresh = 0.03;
   double rotThresh = 0.03;

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

void GraphSlamDisplay::repositionMap(int index, Pose newStart, Pose newEnd, Pose newEndNext, bool warpMap) {
   tf::Matrix3x3 rotM;
   tf::Vector3 transM;
   tf::Vector3 gM;

   tf::Vector3 newStartRot, newEndRot, newEndNextRot, oldStartRot, oldEndRot, oldEndNextRot;
   tf::Vector3 newStartPos, newEndPos, newEndNextPos, oldStartPos, oldEndPos, oldEndNextPos;
   rotPoseToVector(newStart, newStartRot);
   newStartPos = newStart.position.toTF();
   rotPoseToVector(newEnd, newEndRot);
   newEndPos = newEnd.position.toTF();
   rotPoseToVector(newEndNext, newEndNextRot);
   newEndNextPos =  newEndNext.position.toTF();
   rotPoseToVector(maps[index].pose, oldStartRot);
   oldStartPos = maps[index].pose.position.toTF();
   if (index + 1 < maps.size()) {
      rotPoseToVector(maps[index + 1].pose, oldEndRot);
      oldEndPos = maps[index + 1].pose.position.toTF();
   }
   if (index + 2 < maps.size()) {
      rotPoseToVector(maps[index + 2].pose, oldEndNextRot);
      oldEndNextPos = maps[index + 2].pose.position.toTF();
   }

   if (!warpMap) {
      //Just transform the points according to the pose at the start of the map
      gM = oldStartPos;
      transM = newStartPos - oldStartPos;
      tf::Vector3 rot = newStartRot - oldStartRot;
      rotM.setRPY(rot[2], rot[1], rot[0]);
      /*tf::Transform newTrans = newStart.toTF();
      tf::Transform oldTrans = maps[index].pose.toTF();
      gM = oldTrans.getOrigin();
      tf::Transform diff = oldTrans.inverseTimes(newTrans);
      transM = diff.getOrigin();
      rotM = diff.getBasis();*/
   }
   int sizeCloud;
   //pcl::PolygonMesh *mesh = maps[index].mesh;
   if (CreateMesh) {
      sizeCloud = maps[index].mesh->cloud.width * maps[index].mesh->cloud.height;
   } else {
      sizeCloud = maps[index].cloud->size();
   }
   for (int i = 0; i < sizeCloud; i++) {
      const uint8_t *p;
      tf::Vector3 point;
      if (CreateMesh) {
         p = &(maps[index].mesh->cloud.data[i * maps[index].mesh->cloud.point_step]);
         float x,y,z;
         x = *(float*)(p);
         y = *(float*)(p + 4);
         z = *(float*)(p + 8);
         point[0] = x;
         point[1] = y;
         point[2] = z;
      } else {
         point[0] = (*(maps[index].cloud))[i].x;
         point[1] = (*(maps[index].cloud))[i].y;
         point[2] = (*(maps[index].cloud))[i].z;
      }

      if (warpMap) {
         tf::Vector3 a = point - oldStartPos;
         tf::Vector3 u = oldEndPos - oldStartPos;
         double l = u.length2();
         //fraction along the line
         double frac = a.dot(u) / l;

         if (frac < 0.0) {
            frac = 0.0;
         } else if (frac > 1.0 && index + 2 >= maps.size()) {
            frac = 1.0;
         } 
         
         tf::Vector3 posNew;
         tf::Vector3 rot;
         if (frac > 1.0) {
            a = point - oldEndPos;
            u = oldEndNextPos - oldEndPos;
            l = u.length2();
            frac = a.dot(u) / l;
            if (frac > 1.0) {
               frac = 1.0;
            }
            gM = oldEndPos + frac * u;
            posNew = newEndPos + frac * (newEndNextPos - newEndPos);
            tf::Vector3 rotDifNew = newEndNextRot - newEndRot;
            ANGNORM(rotDifNew[0]);
            ANGNORM(rotDifNew[1]);
            ANGNORM(rotDifNew[2]);
            tf::Vector3 rotDifOld = oldEndNextRot - oldEndRot;
            ANGNORM(rotDifOld[0]);
            ANGNORM(rotDifOld[1]);
            ANGNORM(rotDifOld[2]);
            rotDifNew = newEndRot + frac * rotDifNew;
            //ANGNORM(rotDifNew[0]);
            //ANGNORM(rotDifNew[1]);
            //ANGNORM(rotDifNew[2]);
            rotDifOld = oldEndRot + frac * rotDifOld;
            //ANGNORM(rotDifOld[0]);
            //ANGNORM(rotDifOld[1]);
            //ANGNORM(rotDifOld[2]);
            rot = rotDifNew - rotDifOld;
         } else {
            gM = oldStartPos + frac * u;

            posNew = newStartPos + frac * (newEndPos - newStartPos);
            tf::Vector3 rotDifNew = newEndRot - newStartRot;
            ANGNORM(rotDifNew[0]);
            ANGNORM(rotDifNew[1]);
            ANGNORM(rotDifNew[2]);
            tf::Vector3 rotDifOld = oldEndRot - oldStartRot;
            ANGNORM(rotDifOld[0]);
            ANGNORM(rotDifOld[1]);
            ANGNORM(rotDifOld[2]);
            rotDifNew = newStartRot + frac * rotDifNew;
            //ANGNORM(rotDifNew[0]);
            //ANGNORM(rotDifNew[1]);
            //ANGNORM(rotDifNew[2]);
            rotDifOld = oldStartRot + frac * rotDifOld;
            //ANGNORM(rotDifOld[0]);
            //ANGNORM(rotDifOld[1]);
            //ANGNORM(rotDifOld[2]);
            rot = rotDifNew - rotDifOld;
         }
         transM = posNew - gM;
         rotM.setRPY(rot[2], rot[1], rot[0]);
      }

      //Convert the point to the new position
      tf::Vector3 newPoint = (rotM * (point - gM)) + gM + transM;

      if (CreateMesh) {
         float x,y,z;
         x = newPoint[0];
         y = newPoint[1];
         z = newPoint[2];
         *(float*)(p) = x;
         *(float*)(p+4) = y;
         *(float*)(p+8) = z;
      } else {
         (*(maps[index].cloud))[i].x = newPoint[0];
         (*(maps[index].cloud))[i].y = newPoint[1];
         (*(maps[index].cloud))[i].z = newPoint[2];
      }
         
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

void GraphSlamDisplay::outputMapToFile(string fileName) {
   
   if (maps.size() == 0) {
      cout << "No map to save yet!" << endl;
      return;
   }

   if (CreateMesh) {
      //cout << "before" << maps[0].mesh->cloud.height << " " << maps[0].mesh->cloud.width << endl;
      pcl::PolygonMesh fullMesh = *(maps[0].mesh);
      pcl::uint32_t pointStep = maps[0].mesh->cloud.point_step;

      int numPoints = maps[0].mesh->cloud.width;
      for (int i = 1; i < maps.size(); i++) {
         pcl::uint32_t oldDataSize = fullMesh.cloud.data.size();
         pcl::uint32_t cloudDataSize = maps[i].mesh->cloud.data.size();
         fullMesh.cloud.data.resize(oldDataSize + cloudDataSize);
         for (unsigned int j = 0; j < cloudDataSize; j++) {
            fullMesh.cloud.data[j + oldDataSize] = maps[i].mesh->cloud.data[j];
         }  
         fullMesh.cloud.width += maps[i].mesh->cloud.width;
      
         int oldPolygons = fullMesh.polygons.size();
         int numNewPolygons = maps[i].mesh->polygons.size();
         fullMesh.polygons.resize(oldPolygons + numNewPolygons);
         for (unsigned int j = 0; j < numNewPolygons; j++) {
            fullMesh.polygons[j + oldPolygons].vertices.resize(3);
            fullMesh.polygons[j + oldPolygons].vertices[0] = maps[i].mesh->polygons[j].vertices[0] +
               numPoints;
            fullMesh.polygons[j + oldPolygons].vertices[1] = maps[i].mesh->polygons[j].vertices[1] +
               numPoints;
            fullMesh.polygons[j + oldPolygons].vertices[2] = maps[i].mesh->polygons[j].vertices[2] +
               numPoints;
         }
         numPoints += maps[i].mesh->cloud.width;
      }

      fullMesh.cloud.row_step = fullMesh.cloud.data.size();
   
      cout << "Saving map!!" << endl;
      pcl::io::saveVTKFile (fileName, fullMesh);
   } else {
      pcl::PointCloud<pcl::PointXYZRGB> fullCloud;
      for (int i = 0; i < maps.size(); i++) {
         fullCloud += *(maps[i].cloud);
      }
      int numPoints = fullCloud.points.size();
      fullCloud.width = numPoints;
      fullCloud.height = 1;
      fullCloud.is_dense = true;
      pcl::io::savePCDFileASCII (fileName, fullCloud);
      cout << "Finished saving mpa to file!!" << endl;
   }
}

void GraphSlamDisplay::outputPoseToFile(string fileName) {
   FILE *f = fopen(fileName.c_str(), "w");
   for (int i = 0; i < maps.size(); i++) {
      tf::Transform mapTrans = maps[i].pose.toTF();
      for (int j = 0; j < maps[i].poseHistory.size(); j++) {
         tf::Transform point = mapTrans * maps[i].poseHistory[j].toTF();
         tf::Vector3 pVec = point.getOrigin();
         tf::Quaternion pQuat = point.getRotation();
         pQuat.normalize();
         ostringstream tt;
         tt << maps[i].timeHistory[j].toROS();
         const char *st = tt.str().c_str();
         fprintf(f, "%s %lf %lf %lf %lf %lf %lf %lf\n", st, pVec[0], pVec[1], pVec[2], pQuat.x(),
            pQuat.y(), pQuat.z(), pQuat.w());
      }
   }
   fclose(f);
   cout << "Finished saving pose history to file!" << endl;
}

void GraphSlamDisplay::run() {
   viewer = new pcl::visualization::PCLVisualizer ("3D Viewer");
   viewer->setBackgroundColor(1.0, 1.0, 1.0);
   viewer->initCameraParameters ();
   while(!viewer->wasStopped()) {
      viewer->spinOnce(1000);
      viewerLock.lock();
      if (viewerUpdateOptimise) {
         for (int i = 0; i < mapsChanged.size(); i++) {
            ostringstream ss;
            ss << mapsChanged[i];
            //viewer->updatePolygonMesh(*(maps[mapsChanged[i]].mesh), ss.str());
            if (CreateMesh) {
               viewer->removePolygonMesh(ss.str());
               viewer->addPolygonMesh(*(maps[mapsChanged[i]].mesh), ss.str());
            } else {
               viewer->removePointCloud(ss.str());
               viewer->addPointCloud(maps[mapsChanged[i]].cloud, ss.str());
               viewer->setPointCloudRenderingProperties (
                     pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
            }
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
         if (CreateMesh) {
            viewer->addPolygonMesh(*currentMesh, ss.str());
         } else {
            viewer->addPointCloud(currentCloud, ss.str());
            viewer->setPointCloudRenderingProperties (
                  pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, ss.str());
         }
         viewerUpdate = false;
      }
      viewerLock.unlock();
   }
}

