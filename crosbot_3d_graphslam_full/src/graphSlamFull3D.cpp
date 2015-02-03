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

GraphSlamFull3D::GraphSlamFull3D() {
}

void GraphSlamFull3D::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
}

void GraphSlamFull3D::start() {

}

void GraphSlamFull3D::stop() {

}

void GraphSlamFull3D::newLocalMap(LocalMapInfoPtr localMapInfo) {

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



