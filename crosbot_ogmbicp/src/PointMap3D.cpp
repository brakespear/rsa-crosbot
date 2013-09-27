
#include <crosbot_ogmbicp/PointMap3D.hpp>

using namespace crosbot;
using namespace std;

LaserPoint::LaserPoint() {
   point = Point3D(NAN, NAN, NAN);
   pointNxt = Point3D(NAN, NAN, NAN);
}

_LaserPoints::_LaserPoints(PointCloudPtr p, double MaxSegLen, bool IgnoreZValues,
      double FloorHeight, double MinAddHeight, double MaxAddHeight) {
   int i, j;
   double dx, dy, dz;
   points.resize(p->cloud.size());
   for (i = 0; i < p->cloud.size(); i++) {
      if (IgnoreZValues && (p->cloud[i].z < MinAddHeight || p->cloud[i].z > MaxAddHeight)) {
         continue;
      } else if (p->cloud[i].z < FloorHeight) {
         continue;
      }
      points[j].point = p->cloud[i];
      if (IgnoreZValues) {
         points[j].point.z = (MaxAddHeight + MinAddHeight)/ 2.0;
      }
      if (i < p->cloud.size() - 1) {
         dx = p->cloud[i + 1].x - p->cloud[i].x;
         dy = p->cloud[i + 1].y - p->cloud[i].y;
         dz = p->cloud[i + 1].z - p->cloud[i].z;
         if(IgnoreZValues) {
            dz = 0;
         }
         dx = dx * dx + dy * dy + dz * dz;
         if (dx < MaxSegLen) {
            points[j].pointNxt = p->cloud[i + 1];
            if (IgnoreZValues) {
               points[j].pointNxt.z = (MaxAddHeight + MinAddHeight) / 2.0;
            }
         }
      }
      j++;
   }
   points.resize(j);
}

void _LaserPoints::transformPoints(double dx, double dy, double dz, double dth, Pose offset) {
   double cth = cos(dth);
   double sth = sin(dth);
   Point3D p1;
   int i;
   for(i = 0; i < points.size(); i++) {
      p1 = points[i].point;
      p1.x -= offset.position.x;
      p1.y -= offset.position.y;
      points[i].point.x = (p1.x * cth - p1.y * sth + dx) + offset.position.x;
      points[i].point.y = (p1.x * sth + p1.y * cth + dx) + offset.position.y;
      points[i].point.z = p1.z + dz;
      if (points[i].pointNxt.x != NAN) {
         p1 = points[i].pointNxt;
         p1.x -= offset.position.x;
         p1.y -= offset.position.y;
         points[i].pointNxt.x = (p1.x * cth - p1.y * sth + dx) + offset.position.x;
         points[i].pointNxt.y = (p1.x * sth + p1.y * cth + dx) + offset.position.y;
         points[i].pointNxt.z = p1.z + dz;
      }
   }
}

PointMap3D::PointMap3D(double mapSize, double cellSize, double cellHeight): 
   MapSize(mapSize), CellSize(cellSize), CellHeight(cellHeight)
{
   pos_x = 0;
   pos_y = 0;  
   numWidth = mapSize / cellSize;

   grid.resize(numWidth);
   int i, j;
   for(i = 0; i < numWidth; i++) {
      grid[i] = new deque<Cell3DColumn *>();
      grid[i]->resize(numWidth);
      for (j = 0; j < numWidth; j++) {
         (*grid[i])[j] = new Cell3DColumn();
      }
   }
}

PointCloudPtr PointMap3D::centerPointCloud(PointCloud &p, Pose curPose, Pose sensorPose, Pose *laserOffset) {

   Pose3D newPose = curPose;
   newPose.position.x = newPose.position.x - pos_x;
   newPose.position.y = newPose.position.y - pos_y;

   PointCloudPtr rval = new PointCloud("/world", p, newPose);

   Pose absSensorPose = newPose.getTransform() * sensorPose.getTransform();
   laserOffset->position.x = absSensorPose.position.x;
   laserOffset->position.y = absSensorPose.position.y;
   laserOffset->position.z = absSensorPose.position.z;

   return rval;

}

Cell3DColumn *PointMap3D::columnAtXY(double x, double y) {
   return NULL;
}




