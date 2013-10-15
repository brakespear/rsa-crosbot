
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

deque<Cell3D *> Cell3D::markedCells;

Cell3D::Cell3D(double z): zVal(z) {
   mark = false;
}

void Cell3D::reset() {
   points.clear();
   mark = false;
}

void Cell3D::markCell() {
   if (!mark) {
      mark = true;
      markedCells.push_back(this);
   }
}

void Cell3D::unmarkCells() {
   int i;
   for(i = 0; i < markedCells.size(); i++) {
      markedCells[i]->mark = false;
   }
   markedCells.clear();
}

Cell3DColumn::Cell3DColumn(double cellHeight): CellHeight(cellHeight) {
   lastIndex = cells.end();
   obsCount = 0;
   lifeCount = 0;
   current = false;
}

Cell3D *Cell3DColumn::getNearestCell(double z, bool nearest) {
   if (cells.size() == 0) {
      return NULL;
   }
   list<Cell3D *>::iterator prev = lastIndex;
   if (fabs(z - (*lastIndex)->zVal) <= CellHeight) {
      //Already at correct height
      return *lastIndex;
   } else if (z > (*lastIndex)->zVal) {
      while ((lastIndex != cells.begin()) && z > (*lastIndex)->zVal + CellHeight) {
         prev = lastIndex;
         lastIndex--;
      }
      if (fabs(z - (*lastIndex)->zVal) <= CellHeight) {
         return *lastIndex;
      } 
   } else {  //Cell is too high
      while (lastIndex != cells.end() && z < (*lastIndex)->zVal - CellHeight) {
         prev = lastIndex;
         lastIndex++;
      }
      if (lastIndex == cells.end()) {
         lastIndex--;
         if (nearest) {
            return *lastIndex;
         } else {
            return NULL;
         }
      } else if (fabs(z - (*lastIndex)->zVal) <= CellHeight) {
         return *lastIndex;
      }
   }
   if (nearest) {
      if (fabs(z - (*lastIndex)->zVal) < fabs(z - (*prev)->zVal)) {
         return *lastIndex;
      } else {
         lastIndex = prev;
         return *lastIndex;
      }
   }
   return NULL;
}

void Cell3DColumn::addLaserPoint(LaserPoint point, int maxObservations, double lifeRatio, 
      bool resetCell) {
   Cell3D *cell;
   cell = getNearestCell(point.point.z, false);
   if (cell == NULL) {
      cell = addNewCell(point.point.z);
      (*lastIndex)->points.push_back(point);
      cell->markCell();
   } else if (cell->mark) {
      (*lastIndex)->points.push_back(point);
   } else if (resetCell) {
      cell->reset();
      (*lastIndex)->points.push_back(point);
      cell->markCell();
   }

   current = true;
   obsCount++;
   if (obsCount > maxObservations) {
      obsCount = maxObservations;
   }
   lifeCount = (int) lifeRatio * obsCount;

}

Cell3D *Cell3DColumn::addNewCell(double zCent) {
   Cell3D *newCell = new Cell3D(zCent);
   if (cells.size() > 0 && zCent < (*lastIndex)->zVal) {
      lastIndex++;
   }
   lastIndex = cells.insert(lastIndex, newCell);
   return newCell;
}

void Cell3DColumn::reset() {
   current = false;
   lifeCount = 0;
   obsCount = 0;
   cells.clear();
}

PointMap3D::PointMap3D(double mapSize, double cellSize, double cellHeight): 
   MapSize(mapSize), CellSize(cellSize), CellHeight(cellHeight)
{
   pos_x = 0;
   pos_y = 0;
   offsetX = 0;
   offsetY = 0;
   numWidth = mapSize / cellSize;
   mapOffset = -(numWidth * mapSize) / 2.0;

   grid.resize(numWidth);
   int i, j;
   for(i = 0; i < numWidth; i++) {
      grid[i] = new deque<Cell3DColumn *>();
      grid[i]->resize(numWidth);
      for (j = 0; j < numWidth; j++) {
         (*grid[i])[j] = new Cell3DColumn(CellHeight);
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
   int i,j;
   getIJ(x, y, &i, &j);
   return columnAtIJ(i,j);
}

Cell3DColumn *PointMap3D::columnAtIJ(int i, int j) {
   if (i < 0 || i >= numWidth || j < 0 || j >= numWidth) {
      return NULL;
   } else {
      return (*grid[j])[i];
   }
}

void PointMap3D::getIJ(double x, double y, int *i, int *j) {
   *i = (x - mapOffset) / CellSize;
   *j = (y - mapOffset) / CellSize;
}

void PointMap3D::getXY(int i, int j, double *x, double *y) {
   *x = ((double) i) * CellSize + mapOffset + (CellSize / 2.0);
   *y = ((double) j) * CellSize + mapOffset + (CellSize / 2.0);
}

void PointMap3D::addScan(LaserPoints scan, int maxObservations, double lifeRatio,
      bool resetCells) {
   int k;
   for(k = 0; k < scan->points.size(); k++) {
      LaserPoint point = scan->points[k];
      if (point.pointNxt.hasNAN()) {
         continue;
      }
      //TODO: deal with z values properly
      //See update and add functions in localmap3d

      int i,j;
      getIJ(point.point.x, point.point.y, &i, &j);
      double centerX, centerY;
      getXY(i, j, &centerX, &centerY);
      point.point.x -= centerX;
      point.point.y -= centerY;
      point.pointNxt.x -= centerX;
      point.pointNxt.y -= centerY;
      Cell3DColumn *col = columnAtIJ(i,j);
      if (col == NULL) {
         continue;
      }
      if (col->lifeCount == 0) {
         activeColumns.push_back(ActiveColumn(i, j));
      }
      col->addLaserPoint(point, maxObservations, lifeRatio, resetCells);
   }

   Cell3D::unmarkCells();
}

void PointMap3D::updateActiveCells() {
   deque<ActiveColumn>::iterator it;

   for (it = activeColumns.begin(); it != activeColumns.end();) {
      Cell3DColumn *col = columnAtIJ(it->i, it->j);
      if (col == NULL) {
         it = activeColumns.erase(it);
         continue;
      }
      col->lifeCount--;
      col->current = false;

      if (col->lifeCount <= 0) {
         col->reset();
         it = activeColumns.erase(it);
         continue;
      }
      it++;
   }
}

void PointMap3D::shift(double gx, double gy, double gz) {
   int xMove, yMove;
   double cellMoveX, cellMoveY;
   offsetX += gx;
   offsetY += gy;
   xMove = (int)(offsetX / CellSize);
   yMove = (int)(offsetY / CellSize);
   cellMoveX = ((double)xMove) * CellSize;
   cellMoveY = ((double)yMove) * CellSize;
   offsetX -= cellMoveX;
   offsetY -= cellMoveY;

   pos_x += cellMoveX;
   pos_y += cellMoveY;

   if (xMove != 0) {
      shiftX(xMove);
   }
   if (yMove != 0) {
      shiftY(yMove);
   }
   if (xMove != 0 || yMove != 0) {
      deque<ActiveColumn>::iterator it = activeColumns.begin();
      while (it != activeColumns.end()) {
         it->i -= xMove;
         it->j -= yMove;
         if (it->i < 0 || it->i >=numWidth || it->j < 0 || it->j >= numWidth) {
            it = activeColumns.erase(it);
         } else {
            it++;
         }
      }
   }
}

void PointMap3D::shiftX(int xMove) {
   int i,j;
   int cnt = xMove > 0 ? xMove : -xMove;
   Cell3DColumn *tmp;
   for (i = 0; i < numWidth; i++) {
      for (j = 0; j < cnt; j++) {
         if (xMove > 0) {
            tmp = grid[i]->front();
            grid[i]->pop_front();
            tmp->reset();
            grid[i]->push_back(tmp);
         } else {
            tmp = grid[i]->back();
            grid[i]->pop_back();
            tmp->reset();
            grid[i]->push_front(tmp);
         }
      }
   }
}

void PointMap3D::shiftY(int yMove) {
   int i, j;
   int cnt = yMove > 0 ? yMove : -yMove;
   deque<Cell3DColumn *> *tmp;
   for (i = 0; i < cnt; i++) {
      if (yMove > 0) {
         tmp = grid.front();
         grid.pop_front();
      } else {
         tmp = grid.back();
         grid.pop_back();
      }
      for(j = 0; j < tmp->size(); j++) {
         if ((*tmp)[j] != NULL) {
            (*tmp)[j]->reset();
         }
      }
      if (yMove > 0) {
         grid.push_back(tmp);
      } else {
         grid.push_front(tmp);
      }
   }
}


