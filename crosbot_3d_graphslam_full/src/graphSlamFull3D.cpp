/*
 * graphslamFull3D.cpp
 *
 * Created on: 16/01/2015
 *     Author: adrianr
 */

#include <crosbot_3d_graphslam_full/graphSlamFull3D.hpp>
#include <crosbot/utils.hpp>
#include <suitesparse/cs.h>

#define ANGNORM(X) while (X < -M_PI) X += 2.0*M_PI;while (X > M_PI) X -= 2.0*M_PI

using namespace std;
using namespace crosbot;

LocalMaps::LocalMaps(LocalMapInfoPtr inMap, int pIndex, pcl::PointCloud<pcl::PointNormal>::Ptr pts) {
   pose = inMap->pose.toTF();
   cloud = pts;
   parentIndex = pIndex;
   initialPose = pose;
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
   paramNH.param<double>("MaxCorrespDist", MaxCorrespDist, 0.2);
   paramNH.param<int>("SkipPoints", SkipPoints, 8);
   paramNH.param<double>("KDTreeEpsilon", KDTreeEpsilon, 0.03);
   paramNH.param<int>("MaxIterations", MaxIterations, 10);
   paramNH.param<int>("MinCount", MinCount, 10000);
   paramNH.param<double>("StartDistThresh", StartDistThresh, 0.2);
   paramNH.param<double>("StartDotThresh", StartDotThresh, 0.6);
   paramNH.param<double>("EndDistThresh", EndDistThresh, 0.03);
   paramNH.param<double>("EndDotThresh", EndDotThresh, 0.8);
   paramNH.param<double>("MoveThresh", MoveThresh, 0.01);
   paramNH.param<int>("MaxNumOfOptimisationIts", MaxNumOfOptimisationIts, 10);
   paramNH.param<double>("MaxOptMoveXYZ", MaxOptMoveXYZ, 0.01);
   paramNH.param<double>("MaxOptMoveYPR", MaxOptMoveYPR, 0.01);


   double y1 = 0.2;
   double r1 = -0.3;
   double p1 = 1.3;
   double y2 = 1.6;
   double r2 = 0.6;
   double p2 = -0.4;
   y1 = r1 = y2 = r2 = 0;
   tf::Matrix3x3 mat1, mat2, matAdd, matRes;
   mat1.setEulerYPR(y1, p1, r1);
   mat2.setEulerYPR(y2, p2, r2);
   matAdd.setEulerYPR(y1 + y2, p1 + p2, r1 + r2);
   matRes = mat2 * mat1;

   double yA,pA,rA, yR,pR,rR;
   matAdd.getEulerYPR(yA, pA, rA);
   matRes.getEulerYPR(yR, pR, rR);
   cout << "Result of adding first: " << yA << " " << pA << " " << rA << endl;
   cout << "Result of multiplying : " << yR << " " << pR << " " << rR << endl;



   /*tf::Vector3 incVec(b[x*6 + 3], b[x*6 + 4], b[x*6 + 5]);
   tf::Matrix3x3 incMat;
   incMat.setEulerYPR(b[x*6 + 2], b[x*6 + 1], b[x*6]);
   tf::Transform inc(incMat, incVec);
   localMaps[x]->pose = inc * localMaps[x]->pose;*/

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

   cout << "Received new local map " << currentIndex << endl;

   pcl::PointCloud<pcl::PointNormal>::Ptr cloud = addPointsToCloud(localMapInfo->cloud, localMapInfo->normals);
   localMaps.push_back(new LocalMaps(localMapInfo, parentIndex, cloud));

   if (parentIndex != -1) {
   
      pcl::KdTreeFLANN<pcl::PointNormal> kdTree;
      kdTree.setEpsilon(KDTreeEpsilon);
      kdTree.setInputCloud(cloud);

      tf::Transform diff = localMaps[currentIndex]->pose.inverse() * localMaps[parentIndex]->pose;
      calculateInfo(kdTree, cloud, parentIndex, diff, localMaps[currentIndex]->parentInfo);
      localMaps[currentIndex]->parentOffset = localMaps[parentIndex]->initialPose.inverse() * localMaps[currentIndex]->pose;
      localMaps[currentIndex]->pose = localMaps[parentIndex]->pose * localMaps[currentIndex]->parentOffset;


      if (receivedOptimisationRequest) {

         cout << "processing optimisation request" << endl;
         for (int x = 0; x < newMapPos.size(); x++) {
      
            if (newMapPos[x]->index < localMaps.size()) {
               updateMapPosition(newMapPos[x]->index, newMapPos[x]->pose);
            } else {
               cout << "ERROR: received map index that is too high" << endl;
            }
         }

         /*if (haveNewMapPosition) {
            updateMapPosition(currentIndex, newMapPosition);
         }*/
         if (startNewConstraints != loopConstraints.size() - 1) {
            cout << "ERROR: Correcting more than one loop closure per map is not supported yet " << startNewConstraints << " "
              << loopConstraints.size() << endl;
         }
         if (loopConstraints[startNewConstraints]->j != currentIndex) {
            cout << "ERROR: Loop constraints are not in expected format" << endl;
         }
         int mapI = loopConstraints[startNewConstraints]->i;
         int mapJ = loopConstraints[startNewConstraints]->j;
         //TODO: get transform from loop close service??
         tf::Transform diff = localMaps[mapJ]->pose.inverse() * localMaps[mapI]->pose;
         cout << "Matching maps: " << mapI << " " << mapJ << endl;
         double y,p,r;
         Pose pp = diff;
         pp.getYPR(y,p,r);
         cout << "Pose initial: " << pp.position.x << " " << pp.position.y << " " << pp.position.z << 
            " " << y << " " << p << " " << r << endl;


         
   int numPoints = localMapInfo->cloud->cloud.size();
   cout << "outputting current map: " << endl;
   FILE *f = fopen("curMap.pcd", "w");
   fprintf(f, "VERSION .7\nFIELDS x y z\nSIZE 4 4 4 \nTYPE F F F \nCOUNT 1 1 1 \nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", numPoints, numPoints);
   for (int i = 0; i < numPoints;i++) {
      fprintf(f, "%f %f %f\n", localMapInfo->cloud->cloud[i].x, localMapInfo->cloud->cloud[i].y, 
            localMapInfo->cloud->cloud[i].z);
   }
   fclose(f);
   cout << "done" << endl;
   numPoints = localMaps[mapI]->cloud->size();
   f = fopen("otherMap.pcd", "w");
   fprintf(f, "VERSION .7\nFIELDS x y z\nSIZE 4 4 4 \nTYPE F F F \nCOUNT 1 1 1 \nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", numPoints, numPoints);
   for (int i = 0; i < numPoints;i++) {
      tf::Vector3 p;
      p[0] = (*(localMaps[mapI]->cloud))[i].x;
      p[1] = (*(localMaps[mapI]->cloud))[i].y;
      p[2] = (*(localMaps[mapI]->cloud))[i].z;
      p = diff * p;
      fprintf(f, "%f %f %f\n", p.x(), p.y(), p.z());
   }
   fclose(f);




         
         cout << "About to perform ICP" << endl; 
         tf::Transform change = performICP(kdTree, cloud, mapI, diff);
         cout << "Performed ICP" << endl;
         calculateInfo(kdTree, cloud, mapI, change, loopConstraints[startNewConstraints]->info);
         //loopConstraints[startNewConstraints]->offset = localMaps[mapI]->pose.inverse() *
         //   localMaps[mapJ]->pose * change;
         loopConstraints[startNewConstraints]->offset = change.inverse();
  
         
         pp = change;
         pp.getYPR(y,p,r);
         cout << "Pose change: " << pp.position.x << " " << pp.position.y << " " << pp.position.z <<
            " " << y << " " << p << " " << r << endl;
         r = 0; p = 0;
         pp.setYPR(y,p,r);
         //pp.position.z = 0;
         loopConstraints[startNewConstraints]->offset = pp.toTF().inverse();
  
         
      
   
   numPoints = localMaps[mapI]->cloud->size();
   f = fopen("otherMapAl.pcd", "w");
   fprintf(f, "VERSION .7\nFIELDS x y z\nSIZE 4 4 4 \nTYPE F F F \nCOUNT 1 1 1 \nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA ascii\n", numPoints, numPoints);
   for (int i = 0; i < numPoints;i++) {
      tf::Vector3 p;
      p[0] = (*(localMaps[mapI]->cloud))[i].x;
      p[1] = (*(localMaps[mapI]->cloud))[i].y;
      p[2] = (*(localMaps[mapI]->cloud))[i].z;
      p = change * p;
      fprintf(f, "%f %f %f\n", p.x(), p.y(), p.z());
   }
   fclose(f);
         
         cout << "About to optimise" << endl;
         optimiseGlobalMap();
         cout << "Finished optimising" << endl;
         vector<LocalMapInfoPtr> newPositions = getNewMapPositions();
         graphSlamFull3DNode->publishOptimisedMapPositions(newPositions);

         receivedOptimisationRequest = false;
         parentIndex = mapI;
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

   cout << "Received optimisation request" << endl;
   startNewConstraints = loopConstraints.size();
   for (int x = startNewConstraints; x < iNodes.size(); x++) {
      int i = iNodes[x];
      int j = jNodes[x];
      //tf::Transform offset = newMapPositions[j].pose.toTF().inverse() * newMapPositions[i].pose.toTF();
      LoopConstraint *loop = new LoopConstraint(i, j);
      loopConstraints.push_back(loop);
   }
   newMapPos.clear();
   newMapPos.insert(newMapPos.end(), newMapPositions.begin(), newMapPositions.end());
   //haveNewMapPosition = false;
   /*for (int x = 0; x < newMapPositions.size(); x++) {
      
      if (newMapPositions[x]->index < localMaps.size()) {
         updateMapPosition(newMapPositions[x]->index, newMapPositions[x]->pose);
      } else if (newMapPositions[x]->index == localMaps.size()) {
         haveNewMapPosition = true;
         newMapPosition = newMapPositions[x]->pose;
      } else {
         cout << "ERROR: received map index that is too high" << endl;
      }
   }*/
   receivedOptimisationRequest = true;
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
   point.x = p.x();
   point.y = p.y();
   point.z = p.z();

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
         if (i == j) {
            info[i][j] = 1;
         }
      }
   }

   /*int count = 0;
   pcl::PointCloud<pcl::PointNormal>::Ptr otherCloud = localMaps[otherI]->cloud;
   int size = otherCloud->size();
   for (int i = 0; i < size; i += SkipPoints) {
      tf::Vector3 p((*otherCloud)[i].x, (*otherCloud)[i].y, (*otherCloud)[i].z);
      p = diff * p;

      pcl::PointNormal mapP = getClosestPoint(kdTree, cloud, p);
      if (!isnan(mapP.x)) {
         count++;
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
               info[y][x] += temp[y] * temp[x];
            }
         }
      }
   }
   cout << "Used: " << count << " points for info matrix" << endl;*/
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
            cout << "Found a NAN value" << endl;
         }
         if (fabs(x[j]) > MoveThresh) {
            cont = true;
         }
      }
      //if (goodCount < MinCount) {
      //   valid = false;
      //}
      if (!valid) {
         cout << "ERROR: Alignment failed " << goodCount << endl;
         failed = true;
         break;
      }
      cout << "Success!: " << x[3] << " " << x[4] << " " << x[5] << " " << x[0] << " " << x[1] << " " << x[2] << endl;
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
            double scale = dot;
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

void GraphSlamFull3D::updateMapPosition(int i, Pose pose) {
   Pose current = localMaps[i]->pose;
   double y,p,r;
   current.getYPR(y,p,r);
   current.position.x = pose.position.x;
   current.position.y = pose.position.y;
   double yp, pp, rp;
   pose.getYPR(yp,pp, rp);
   y = yp;
   current.setYPR(y,p,r);
   localMaps[i]->pose = current.toTF();

}

void GraphSlamFull3D::optimiseGlobalMap() {
   int numMaps = localMaps.size();
   int nrows = 6 * numMaps;
   int nzmax = (numMaps + (numMaps + loopConstraints.size()) * 2) * 36;

   double *b = (double *) malloc(sizeof(double) * nrows);
   cs *sparseH = cs_spalloc(nrows, nrows, nzmax, 1, 1);

   //TODO: Only optimise part of map
   tf::Transform startPose = localMaps[0]->pose;

   for (int i = 0; i < numMaps; i++) {
      tfToArray(localMaps[i]->pose, localMaps[i]->startPose);
      localMaps[i]->poseChanged = false;
   }

   for (int numI = 0; numI < MaxNumOfOptimisationIts; numI++) {

      memset(b, 0, sizeof(double) * nrows);
      int count = 0;

      for (int x = 0; x <= currentIndex; x++) {
         int off = x * 6;
         for (int j = 0; j < 6; j++) {
            for (int i = 0; i < 6; i++, count++) {
               sparseH->i[count] = off + i;
               sparseH->p[count] = off + j;
               sparseH->x[count] = 0;
            }
         }
         double temp[6];
         tfToArray(localMaps[x]->pose, temp);
         cout << "Pose of map: " << x << " is: " << temp[3] << " " << temp[4] << " " << temp[5]
            << "  " << temp[2] << " " << temp[1] << " " << temp[0] << endl;
      }

      double maxMove[6];
      memset(maxMove, 0, sizeof(double) * 6);

      int numConstraints = localMaps.size() - 1 + loopConstraints.size();
      int startLoopConstraint = localMaps.size() - 1;
      for (int cI = 0; cI < numConstraints; cI++) {

         int iNode;
         int jNode;
         double constraint[6];
         //Information matrix of the constraint
         double info[6][6];
         //The two blocks of the Jacobian
         double A[6][6];
         double B[6][6];
         //Positions of the two maps
         double posI[6];
         double posJ[6];
         //error in the constraint
         double error[6];
         //Transposes of the Jacobians
         double AT[6][6];
         double BT[6][6];
         //Temp stores
         double temp[6][6];
         double ATA[6][6];
         double ATB[6][6];
         double BTA[6][6];
         double BTB[6][6];


         if (cI < startLoopConstraint) {
            //Constraint is a movement constraint
            int mapI = cI + 1;
            iNode = localMaps[mapI]->parentIndex;
            jNode = mapI;
            tfToArray(localMaps[mapI]->parentOffset, constraint);
            copyM(localMaps[mapI]->parentInfo, info);
            /*cout << "Info for loop " << iNode << " " << jNode << ": ";
            for (int y = 0; y < 6; y++) {
               for (int x = 0; x < 6; x++) {
                  cout << info[y][x] << " ";
               }
            }
            cout << endl;*/


         } else {
            //Constraint is a loop closing constraint
            int conI = cI - startLoopConstraint;
            iNode = loopConstraints[conI]->i;
            jNode = loopConstraints[conI]->j;
            tfToArray(loopConstraints[conI]->offset, constraint);
            copyM(loopConstraints[conI]->info, info);
            /*cout << "Info for constraint " << iNode << " " << jNode << ": ";
            for (int y = 0; y < 6; y++) {
               for (int x = 0; x < 6; x++) {
                  cout << info[y][x] << " ";
               }
            }
            cout << endl;*/
         }
         if (numI == 0) {
            cout << "^^^^^ Maps: " << iNode << " " << jNode << ": " << constraint[3] << " " << constraint[4] <<
               " " << constraint[5] << "  " << constraint[2] << " " << constraint[1] << " " << constraint[0] << endl;
         }
         tfToArray(localMaps[iNode]->pose, posI);
         tfToArray(localMaps[jNode]->pose, posJ);

         double cr = cos(posI[0]);
         double cp = cos(posI[1]);
         double cy = cos(posI[2]);
         double sr = sin(posI[0]);
         double sp = sin(posI[1]);
         double sy = sin(posI[2]);

         double xd = posJ[3] - posI[3];
         double yd = posJ[4] - posI[4];
         double zd = posJ[5] - posI[5];

         memset(A, 0, sizeof(double) * 36);
         memset(B, 0, sizeof(double) * 36);
         A[0][0] = -1;
         A[1][1] = -1;
         A[2][2] = -1;
         
         A[3][0] = (cr*sp*cy-sr*sy)*yd + (cr*sy+sp*sr*cy)*zd;
         A[3][1] = -sp*cy*xd + cp*sr*cy*yd - cp*cr*cy*zd;
         A[3][2] = -cp*sy*xd + (cr*cy-sp*sr*sy)*yd + (sr*cy+sp*cr*sy)*zd;
         //A[3][1] = -sp*xd - cp*zd;
         //A[3][2] = -sy*xd + cy*yd;
         
         A[3][3] = -cp*cy;
         A[3][4] = -sp*sr*cy - cr*sy;
         A[3][5] = sp*cr*cy - sr*sy;
         
         A[4][0] = (-sr*cy-sp*cr*sy)*yd + (cr*cy-sp*sr*sy)*zd;
         A[4][1] = sp*sy*xd - cp*sr*sy*yd + cp*cr*sy*zd;
         A[4][2] = -cp*cy*xd + (-cr*sy-sp*sr*cy)*yd + (sp*cr*cy-sr*sy)*zd;
         //A[4][0] = (-sr)*yd + (cr)*zd;
         //A[4][2] = -cy*xd + (-sy)*yd;
         
         A[4][3] = cp*sy;
         A[4][4] = sp*sr*sy-cr*cy;
         A[4][5] = -sp*cr*sy-sr*cy;
         
         A[5][0] = -cp*cr*yd - cp*sr*zd;
         A[5][1] = cp*xd + sp*sr*yd - sp*cr*zd;
         //A[5][0] = -cr*yd - sr*zd;
         //A[5][1] = cp*xd - sp*zd;
         
         A[5][3] = -sp;
         A[5][4] = cp*sr;
         A[5][5] = -cp*cr;

         B[0][0] = 1;
         B[1][1] = 1;
         B[2][2] = 1;
         B[3][3] = cp*cy;
         B[3][4] = sp*sr*cy + cr*sy;
         B[3][5] = sr*sy - sp*cr*cy;
         B[4][3] = -cp*sy;
         B[4][4] = cr*cy - sp*sr*sy;
         B[4][5] = sp*cr*sy + sr*cy;
         B[5][3] = sp;
         B[5][4] = -cp*sr;
         B[5][5] = cp*cr;

         error[0] = posJ[0] - posI[0] - constraint[0];
         ANGNORM(error[0]);
         error[1] = posJ[1] - posI[1] - constraint[1];
         ANGNORM(error[1]);
         error[2] = posJ[2] - posI[2] - constraint[2];
         ANGNORM(error[2]);
         error[3] = cp*cy*xd + (sp*sr*cy+cr*sy)*yd + (sr*sy-sp*cr*cy)*zd - constraint[3];
         error[4] = -cp*sy*xd + (cr*cy - sp*sr*sy)*yd + (sp*cr*sy+sr*cy)*zd - constraint[4];
         error[5] = sp*xd - cp*sr*yd + cp*cr*zd - constraint[5];
         //if (numI == 0) {
            cout << "Error: " << error[3] << " " << error[4] <<
               " " << error[5] << "  " << error[2] << " " << error[1] << " " << error[0] << endl;
         //}

         transpose6x6Matrix(A, AT);
         transpose6x6Matrix(B, BT);

         double tempVecA[6];
         double tempVecB[6];
         mult6x6Matrix(AT, info, temp);
         mult6x6Vector(temp, error, tempVecA);

         mult6x6Matrix(temp, A, ATA);
         mult6x6Matrix(temp, B, ATB);

         mult6x6Matrix(BT, info, temp);
         mult6x6Vector(temp, error, tempVecB);

         mult6x6Matrix(temp, A, BTA);
         mult6x6Matrix(temp, B, BTB);

         for (int i = 0; i < 6; i++) {
            b[iNode * 6 + i] -= tempVecA[i];
            b[jNode * 6 + i] -= tempVecB[i];
         }

         //row is i, column is p, value is x
         for (int j = 0; j < 6; j++) {
            for (int i = 0; i < 6; i++) {
               sparseH->x[iNode * 36 + j * 6 + i] += ATA[i][j];
               sparseH->x[jNode * 36 + j * 6 + i] += BTB[i][j];

               sparseH->i[count] = iNode * 6 + i;
               sparseH->p[count] = jNode * 6 + j;
               sparseH->x[count] = ATB[i][j];
               count++;
               sparseH->i[count] = jNode * 6 + i;
               sparseH->p[count] = iNode * 6 + j;
               sparseH->x[count] = BTA[i][j];
               count++;
            }
         }
      }
      sparseH->nz = count;
   
      //Compress H into sparse column form
      cs *compH = cs_compress(sparseH);
      //Solve the linear system
      int retVal = cs_lusol(0, compH, b, 1);
      if (retVal != 1) {
         cout << "Solving matrix failed " << retVal << endl;
         break;
      }

      for (int x = 0; x < numMaps; x++) {
         cout << "Map " << x << " moved: " << b[x*6+3] << " " << b[x*6+4] << " " << b[x*6+5] << "  " 
            << b[x*6+2] << " " << b[x*6+1] << " " << b[x*6+0] << endl;

         if (b[x * 6] > 100) {
            cout << "ERROR: BIIIGGGG Angle " << b[x * 6] << endl;
            return;
         }
         ANGNORM(b[x*6]);
         ANGNORM(b[x*6 + 1]);
         ANGNORM(b[x*6 + 2]);

         double tPos[6];
         tfToArray(localMaps[x]->pose, tPos);
         for (int y = 0; y < 6; y++) {
            tPos[y] += b[x*6 + y];
            if (y < 3) {
               ANGNORM(tPos[y]);
            }

            if (fabs(b[x*6 + y]) > maxMove[y]) {
               maxMove[y] = fabs(b[x*6 + y]);
            }
         }
         Pose p;
         p.position.x = tPos[3];
         p.position.y = tPos[4];
         p.position.z = tPos[5];
         p.setYPR(tPos[2], tPos[1], tPos[0]);
         localMaps[x]->pose = p.toTF();

         /*tf::Vector3 incVec(b[x*6 + 3], b[x*6 + 4], b[x*6 + 5]);
         tf::Matrix3x3 incMat;
         incMat.setEulerYPR(b[x*6 + 2], b[x*6 + 1], b[x*6]);
         tf::Transform inc(incMat, incVec);
         localMaps[x]->pose = inc * localMaps[x]->pose;*/
      }
      if (maxMove[0] < MaxOptMoveYPR && maxMove[1] < MaxOptMoveYPR && maxMove[2] < MaxOptMoveYPR &&
          maxMove[3] < MaxOptMoveXYZ && maxMove[4] < MaxOptMoveXYZ && maxMove[5] < MaxOptMoveXYZ) {
         cout << "Finished Optimising" << endl;
         break;
      }
   }
   cs_spfree(sparseH);
   free(b);

   //Recenter the map on 0
   tf::Transform off = localMaps[0]->pose.inverse();

   double thresh = 0.005;
   for (int i = 0; i <= currentIndex; i++) {
      localMaps[i]->pose = off * localMaps[i]->pose;
      double tempPos[6];
      tfToArray(localMaps[i]->pose, tempPos);
      cout << "End pose of map: " << i << " is: " << tempPos[3] << " " << tempPos[4] << " " << tempPos[5]
         << "  " << tempPos[2] << " " << tempPos[1] << " " << tempPos[0] << endl;
      for (int x = 0; x < 6; x++) {
         if (fabs(tempPos[x] - localMaps[i]->startPose[x]) > thresh) {
            localMaps[i]->poseChanged = true;
         }
      }
   }
}

//ri and rj are in r,p,y order!!
/*void GraphSlamFull3D::numericPartJacobian(double ri[3], double rj[3], double A[3][3], double B[3][3]) {

   double h = 0.001;
   tf::Matrix3x3 matI, matJ;
   tf::Matrix3x3 res;
   matJ.setEulerYPR(rj[2], rj[1], rj[0]);
   matI.setEulerYPR(ri[2], ri[1], ri[0]);

}*/

vector<LocalMapInfoPtr> GraphSlamFull3D::getNewMapPositions() {
   vector<LocalMapInfoPtr> newPositions;
   for (int i = 0; i < localMaps.size(); i++) {
      if (localMaps[i]->poseChanged) {
         Pose p = localMaps[i]->pose;
         newPositions.push_back(new LocalMapInfo(p, i));
      }
   }
   return newPositions;
}

inline void GraphSlamFull3D::tfToArray(tf::Transform trans, double arr[6]) {
   Pose p = trans;
   arr[3] = p.position.x;
   arr[4] = p.position.y;
   arr[5] = p.position.z;
   p.getYPR(arr[2], arr[1], arr[0]);
}

inline void GraphSlamFull3D::copyM(double in[6][6], double out[6][6]) {
   for (int j = 0; j < 6; j++) {
      for (int i = 0; i < 6; i++) {
         out[j][i] = in[j][i];
      }
   }
}

inline void GraphSlamFull3D::mult6x6Matrix(double a[6][6], double b[6][6], double c[6][6]) {
   for (int y = 0; y < 6; y++) {
      for (int x = 0; x < 6; x++) {
         c[y][x] = 0;
         for(int i = 0; i < 6; i++) {
            c[y][x] += a[y][i] * b[i][x];
         }
      }
   }
}

inline void GraphSlamFull3D::transpose6x6Matrix(double in[6][6], double out[6][6]) {
   for (int y = 0; y < 6; y++) {
      for (int x = 0; x < 6; x++) {
         out[x][y] = in[y][x];
      }
   }
}

inline void GraphSlamFull3D::mult6x6Vector(double a[6][6], double b[6], double *res) {
   for (int y = 0; y < 6; y++) {
      res[y] = 0;
      for (int x = 0; x < 6; x++) {
         res[y] += a[y][x] * b[x];
      }
   }
}

