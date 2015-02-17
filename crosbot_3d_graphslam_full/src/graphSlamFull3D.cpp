/*
 * graphslamFull3D.cpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_full/graphSlamFull3D.hpp>
#include <crosbot/utils.hpp>

using namespace std;
using namespace crosbot;

LocalMaps::LocalMaps(LocalMapInfoPtr inMap, int pIndex, pcl::PointCloud<pcl::PointNormal>::Ptr pts) {
   pose = inMap->pose.toTF();
   cloud = pts;
   parentIndex = pIndex;
}

GraphSlamFull3D::GraphSlamFull3D() {
   parentIndex = -1;
   currentIndex = -1;
   receivedOptimisationRequest = false;
   nearestPoint.resize(1);
   nearestDist.resize(1);
}

void GraphSlamFull3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
}

void GraphSlamFull3D::start() {

}

void GraphSlamFull3D::stop() {

}

void GraphSlamFull3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

   if (localMapInfo->index != currentIndex + 1) {
      cout << "ERROR: indexes dont match!!!!" << endl;
   }
   currentIndex = localMapInfo->index;

   pcl::PointCloud<pcl::PointNormal>::Ptr cloud = addPointsToCloud(localMapInfo->cloud, localMapInfo->normals);
   localMaps.push_back(new LocalMaps(localMapInfo, parentIndex, cloud));

   if (parentIndex != -1) {
   
      pcl::KdTreeFLANN<pcl::PointNormal> kdTree;
      kdTree.setEpsilon(KDTreeEpsilon);
      kdTree.setInputCloud(cloud);

      tf::Transform diff = localMaps[currentIndex]->pose.inverse() * localMaps[parentIndex]->pose;
      calculateInfo(kdTree, cloud, parentIndex, diff, localMaps[currentIndex]->parentInfo);
      localMaps[currentIndex]->parentOffset = localMaps[parentIndex]->pose.inverse() * localMaps[currentIndex]->pose;


      if (receivedOptimisationRequest) {

         //map message contains all loop closure constraints
         //need loopconstraint vector, and to add contraints, and here to deal with them properly
         //and all config params
         
         //otherI, diff, info
         //tf::Transform change = performICP(kdTree, cloud, otherI, diff);
         //calculateInfo(kdTree, cloud, otherI, change, info);



         receivedOptimisationRequest = false;
         //parentIndex = 
      } else {
         parentIndex = currentIndex;
      }
   } else {
      parentIndex = 0;
   }


   /*int numPoints = localMapInfo->cloud->cloud.size();
   cout << "outputting current map: " << endl;
   FILE *f = fopen("/home/adrianrobolab/curMap.pcd", "w");
   fprintf(f, "VERSION .7\nFIELDS x y z normal_x normal_y normal_z\nSIZE 4 4 4 4 4 4 \nTYPE F F F F F F\nCOUNT 1 1 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", numPoints, numPoints);
   for (int i = 0; i < numPoints;i++) {
      fprintf(f, "%f %f %f %f %f %f\n", localMapInfo->cloud->cloud[i].x, localMapInfo->cloud->cloud[i].y, 
            localMapInfo->cloud->cloud[i].z, localMapInfo->normals->cloud[i].x, 
            localMapInfo->normals->cloud[i].y, localMapInfo->normals->cloud[i].z);
   }
   fclose(f);
   cout << "done" << endl;*/


}

void GraphSlamFull3D::haveOptimised(vector<LocalMapInfoPtr> newMapPositions,
      vector<int> iNodes, vector<int> jNodes, bool wasFullLoop) {

}

pcl::PointCloud<pcl::PointNormal>::Ptr GraphSlamFull3D::addPointsToCloud(
      PointCloudPtr cloud, PointCloudPtr normal) {
   pcl::PointCloud<pcl::PointNormal>::Ptr out(new pcl::PointCloud<pcl::PointNormal>);
   int size = cloud->cloud.size();
   out->resize(size);

   for (int i = 0; i < size; i++) {
      pcl::PointNormal p;
      p.x = cloud->cloud[i].x;
      p.y = cloud->cloud[i].y;
      p.z = cloud->cloud[i].z;
      p.normal_x = normal->cloud[i].x;
      p.normal_y = normal->cloud[i].y;
      p.normal_z = normal->cloud[i].z;
      (*out)[i] = p;
   }
   return out;
}

inline pcl::PointNormal GraphSlamFull3D::getClosestPoint(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree, 
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, tf::Vector3 p) {
   pcl::PointNormal point;
   point.x = /*(float)*/p.x();
   point.y = (float)p.y();
   point.z = (float)p.z();

   kdTree.nearestKSearch(point, 1, nearestPoint, nearestDist);

   if (nearestDist[0] < MaxCorrespDist) {
      pcl::PointNormal ret((*cloud)[nearestPoint[0]]);
      return ret;
   } else {
      pcl::PointNormal ret;
      ret.x = NAN;
      return ret;
   }
}

void GraphSlamFull3D::calculateInfo(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
      pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform diff,
      double info[6][6]) {

   for (int j = 0; j < 6; j++) {
      for(int i = 0; i < 6; i++) {
         info[j][i] = 0;
      }
   }

   pcl::PointCloud<pcl::PointNormal>::Ptr otherCloud = localMaps[otherI]->cloud;
   int size = otherCloud->size();
   for (int i = 0; i < size; i += SkipPoints) {
      tf::Vector3 p((*otherCloud)[i].x, (*otherCloud)[i].y, (*otherCloud)[i].z);
      p = diff * p;

      pcl::PointNormal mapP = getClosestPoint(kdTree, cloud, p);
      if (!isnan(mapP.x)) {
         double normX = fabs(mapP.normal_x);
         double normY = fabs(mapP.normal_y);
         double normZ = fabs(mapP.normal_z);
         double temp[6];
         temp[0] = normY + normZ;
         temp[1] = normX + normZ;
         temp[2] = normX + normY;
         temp[3] = normX;
         temp[4] = normY;
         temp[5] = normZ;

         for (int y = 0; y < 6; y++) {
            for (int x = 0; x < 6; x++) {
               info[y][x] = temp[y] * temp[x];
            }
         }
      }
   }
}

tf::Transform GraphSlamFull3D::performICP(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform initTrans) {

   bool cont = true;
   bool failed = false;
   tf::Transform trans = initTrans;
   double A[6][6];
   double b[6];
   double x[6];

   double distThresh = StartDistThresh;
   double dotThresh = StartDotThresh;
   int numSkip = SkipPoints * 4;

   for (int i = 0; i < MaxIterations && cont; i++) {

      if (i < MaxIterations / 3) {
         distThresh = EndDistThresh;
         dotThresh = EndDotThresh;
         numSkip = SkipPoints;
      }

      int goodCount = performICPIteration(kdTree, cloud, otherI, trans, A, b, distThresh, dotThresh,
            numSkip);

      solveCholesky(A, b, x);

      bool valid = true;
      cont = false;
      for (int j = 0; j < 6; j++) {
         if(std::isnan(x[j])) {
            valid = false;
         }
         if (fabs(x[j]) > MoveThresh) {
            cont = true;
         }
      }
      if (goodCount < MinCount) {
         valid = false;
      }
      if (!valid) {
         cout << "ERROR: Alignment failed" << endl;
         failed = true;
         break;
      }
      tf::Vector3 incVec(x[3], x[4], x[5]);
      tf::Matrix3x3 incMat;
      incMat.setEulerYPR(x[2], x[1], x[0]);
      tf::Transform inc(incMat, incVec);
      trans = inc * trans;
   }
   if (failed) {
      return initTrans;
   } else {
     return trans;
   } 
}

int GraphSlamFull3D::performICPIteration(pcl::KdTreeFLANN<pcl::PointNormal> &kdTree,
         pcl::PointCloud<pcl::PointNormal>::Ptr cloud, int otherI, tf::Transform trans,
         double AOut[6][6], double bOut[6], double distThresh, double dotThresh, int numSkip) {

   for (int y = 0; y < 6; y++) {
      for(int x = 0; x < 6; x++) {
         AOut[y][x] = 0;
      }
      bOut[y] = 0;
   }

   int goodCount = 0;
   pcl::PointCloud<pcl::PointNormal>::Ptr otherCloud = localMaps[otherI]->cloud;
   int size = otherCloud->size();
   for (int i = 0; i < size; i += numSkip) {
      tf::Vector3 p((*otherCloud)[i].x, (*otherCloud)[i].y, (*otherCloud)[i].z);
      p = trans * p;

      pcl::PointNormal mapP = getClosestPoint(kdTree, cloud, p);

      if (!isnan(mapP.x)) {

         tf::Vector3 n((*otherCloud)[i].normal_x, (*otherCloud)[i].normal_y, (*otherCloud)[i].normal_z);
         tf::Vector3 zero(0,0,0);
         tf::Transform newTrans(trans.getBasis(), zero);
         n = newTrans * n;
         //normalise?

         double dot = n.x() * mapP.normal_x + n.y() * mapP.normal_y + n.z() * mapP.normal_z;
         double dist2 = (p.x() - mapP.x) * (p.x() - mapP.x) +
                        (p.y() - mapP.y) * (p.y() - mapP.y) +
                        (p.z() - mapP.z) * (p.z() - mapP.z);
      
         //TODO: add conditions to accepting pair
         if (dot > dotThresh && dist2 < distThresh) {
            int scale = dot;
            double a = (mapP.normal_z * p.y() - mapP.normal_y * p.z()) * scale;
            double b = (mapP.normal_x * p.z() - mapP.normal_z * p.x()) * scale;
            double c = (mapP.normal_y * p.x() - mapP.normal_x * p.y()) * scale;
            double d = mapP.normal_x * scale;
            double e = mapP.normal_y * scale;
            double f = mapP.normal_z * scale;

            double normScale = (mapP.normal_x * (mapP.x - p.x()) +
                                mapP.normal_y * (mapP.y - p.y()) +
                                mapP.normal_z * (mapP.z - p.z())) * scale;

            AOut[0][0] += a*a;
            AOut[1][0] += a*b;
            AOut[1][1] += b*b;
            AOut[2][0] += a*c;
            AOut[2][1] += b*c;
            AOut[2][2] += c*c;
            AOut[3][0] += a*d;
            AOut[3][1] += b*d;
            AOut[3][2] += c*d;
            AOut[3][3] += d*d;
            AOut[4][0] += a*e;
            AOut[4][1] += b*e;
            AOut[4][2] += c*e;
            AOut[4][3] += d*e;
            AOut[4][4] += e*e;
            AOut[5][0] += a*f;
            AOut[5][1] += b*f;
            AOut[5][2] += c*f;
            AOut[5][3] += d*f;
            AOut[5][4] += e*f;
            AOut[5][5] += f*f;
            bOut[0] += normScale * a;
            bOut[1] += normScale * b;
            bOut[2] += normScale * c;
            bOut[3] += normScale * d;
            bOut[4] += normScale * e;
            bOut[5] += normScale * f;
            goodCount++;
            
         }
      }
   }
   return goodCount;
}

void GraphSlamFull3D::solveCholesky(double A[6][6], double b[6], double x[6]) {

   int col, row, i;
   double buf[6];
   double sum;

   //Calculate LDL^T factorisation of A
   for (row = 0; row < 6; row++) {
      //Calculate L
      for (col = 0; col < row; col++) {
         sum = 0;
         for (i = 0; i < col; i++) {
            sum += A[row][i] * A[col][i] * A[i][i];
         }
         A[row][col] = (1 / A[col][col]) * (A[row][col] - sum);
      }

      //Calculate D
      for (i = row - 1; i >= 0; i--) {
         A[row][row] -= (A[row][i] * A[row][i] * A[i][i]);
      }
   }

   //Calculate Lz = b
   for (row = 0; row < 6; row++) {
      buf[row] = b[row];
      for (i = 0; i < row; i++) {
         buf[row] -= A[row][i] * buf[i];
      }
   }

   //Calculate Dy = z
   for (i = 0; i < 6; i++) {
      buf[i] = buf[i] / A[i][i];
   }

   //Calculate L^T x = y
   for (row = 5; row >= 0; row--) {
      x[row] = buf[row];
      for (i = row + 1; i < 6; i++) {
         x[row] -= A[i][row] * x[i];
      }
   }
}

