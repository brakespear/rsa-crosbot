

//#include "casrobot/include/tasks/positiontrack/openclCommon.h"
//#include "/home/adrianrobolab/groovy_workspace/ratter_graphslam/include/ratter_graphslam/openclCommon.h"
//#include "/home/rescue/groovy_workspace/graphSlam/include/graphSlam/openclCommon.h"
//#include "/home/timothyw/workspace/ros/graphSlam/include/graphSlam/openclCommon.h"

#ifndef M_PI
#define M_PI 3.14159f
#endif
#define WARP_SIZE 32
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

int getMapIndex(float x, float y, int *i, int *j, constant configValues *config) {
   float off = (config->MapCellWidth * config->MapDimWidth) / 2.0f;
   *i = (x + off) / config->MapCellWidth;
   *j = (y + off) / config->MapCellWidth;
   if (*i < 0 || *i >= config->MapDimWidth || *j < 0 || *j >= config->MapDimWidth) {
      return -1;
   } else {
      return *j * config->MapDimWidth + *i;
   }
}


void getCellCentre(int i, int j, float *x, float *y, constant configValues *config) {
   float off = (config->MapCellWidth * config->MapDimWidth) / 2.0f - config->MapCellWidth / 2.0f;
   *x = i * config->MapCellWidth - off;
   *y = j * config->MapCellWidth - off;
}

float getCellZCentre(int index, constant configValues *config) {
   return ((index % config->MapDimHeight) + 0.5f) * config->MapCellHeight + config->MapMinHeight;
}

int getZIndex(int cellIndex, float z, constant configValues *config) {
   int index = cellIndex * config->MapDimHeight;
   int zIndex = (int)((z - config->MapMinHeight) / config->MapCellHeight);
   if (zIndex < config->MapDimHeight && z >= config->MapMinHeight)  {
      return index + zIndex;
   } else {
      return -1;
   }
}

float getHValue(float3 p1, float3 p2, float l2, constant configValues *config) {
   float dx = p2.x - p1.x;
   float dy = p2.y - p1.y;
   float dz = p2.z - p1.z;
   if (dx * dx + dy * dy + dz * dz > config->MaxAlignDist) {
      return INFINITY;
   } else if (config->UseSimpleH) {
      return dx * dx + dy * dy - pown(dx * p1.y - dy * p1.x, 2) / (p1.x * p1.x + p1.y * p1.y + l2);
   } else {
      return dx * dx + dy * dy + dz * dz - 
             (pown(dy * p1.z - dz * p1.y, 2) + pown(dz * p1.x - dx * p1.z, 2) + 
              pown(dx * p1.y - dy * p1.x, 2)) / (p1.x * p1.x + p1.y * p1.y + p1.z * p1.z + l2);
   }
}

void swap(local float mins[LOCAL_SIZE], local int minIdx[LOCAL_SIZE],
      int i1, int i2) {
   float tempf = mins[i1];
   mins[i1] = mins[i2];
   mins[i2] = tempf;
   int tempi = minIdx[i1];
   minIdx[i1] = minIdx[i2];
   minIdx[i2] = tempi;
}

void findMin(local float mins[LOCAL_SIZE], local int minIdx[LOCAL_SIZE],
      int indx, int warpi, int off) {
   if (warpi + off < 16 && mins[indx] > mins[indx + 16]) {
      swap(mins, minIdx, indx, indx + 16);
   }
   if (warpi < 8 && mins[indx] > mins[indx + 8]) {
      swap(mins, minIdx, indx, indx + 8);
   }
   if (warpi < 4 && mins[indx] > mins[indx + 4]) {
      swap(mins, minIdx, indx, indx + 4);
   }
   if (warpi < 2 && mins[indx] > mins[indx + 2]) {
      swap(mins, minIdx, indx, indx + 2);
   }
   if (warpi < 1 && mins[indx] > mins[indx + 1]) {
      swap(mins, minIdx, indx, indx + 1);
   }
}
   
/*
 * Initialises the map structure used on the gpu. This kernel should only be used when the
 * position tracking is initialising
 */
__kernel void initialiseMap(global oclLocalMap3D *map, const int mapSize, const int numActiveCells,
      const int numLaserPointsCell) {
   int index = get_global_id(0);
   if (index == 0) {
      map->nextEmptyCell = 0;
      map->robotOff.x = 0;
      map->robotOff.y = 0;
      map->failCount = 0;
      map->scanSkip = -1;
      map->addSkip = 0;
      map->goodCount = 0;
      map->numIterations = 0;
      map->badScan = 0;

   }
   if (index < 4) {
      map->A[index][0] = 0;
      map->A[index][1] = 0;
      map->A[index][2] = 0;
      map->A[index][3] = 0;
      map->B[index] = 0;
   }
   if (index < numActiveCells) {
      map->indexEmptyCells[index] = index;
      map->xIndex[index] = -1;
      map->yIndex[index] = -1;
      map->obsCount[index] = 0;
      map->prevObsCount[index] = 0;
      map->lifeCount[index] = 0;
      map->cellMinZ[index] = INFINITY;
      map->cellMaxZ[index] = -INFINITY;
      int i;
      for(i = index; i < mapSize; i += numActiveCells) {
         map->activeCellsMap[i] = -1;
      }
      for(i = index; i < numLaserPointsCell; i += numActiveCells) {
         map->numLaserPoints[i] = 0;
      }
      for(i = index; i < NUM_LASER_POINTS; i += numActiveCells) {
         map->pointsMask[i] = 0;
      }
   }
}

/*
 * Adds all the active cell locations to the 2D map.
 */
__kernel void update2DMap(constant configValues *config, global oclLocalMap3D *map, 
                        global oclResults *results) {
   int index = get_global_id(0);
   if (index == 0) {
      if (!config->UsePriorMove) {
         results->finalOffset.x = 0;
         results->finalOffset.y = 0;
         results->finalOffset.z = 0;
         results->finalOffset.w = 0;
      }
      results->cellShift.x = 0;
      results->cellShift.y = 0;
      if (map->scanSkip > config->MaxScanSkip) {
         map->scanSkip = 0;
         if (map->addSkip > config->AddSkipCount + 1) {
            map->addSkip = 0;
         }
      }
   }
   if (index < results->largestActiveCell && map->failCount == 0) {
      if (map->prevObsCount[index] < map->obsCount[index] || map->lifeCount[index] == -2) {
         map->lifeCount[index] = (int) config->LifeRatio * map->obsCount[index];
         if (map->cellMaxZ[index] < config->MapMinHeight) {
            map->lifeCount[index] *= 2;
         }
      }
      if (map->obsCount[index] > config->MaxObservations) {
         map->obsCount[index] = config->MaxObservations;
      }
      if (map->xIndex[index] >= 0) {
         int i = map->yIndex[index] * config->MapDimWidth + map->xIndex[index];
         map->activeCellsMap[i] = index;
      }
   }
}

/*
 * Transforms every laser point according to the offset values given in points->offset
 * It then checks to make sure the point can be used in the position tracking
 */
__kernel void transform(constant configValues *config, global oclLaserPoints *points, 
      global oclLocalMap3D *map, const int numPoints, const int oldNumPoints, 
      const int initialTransform, global float *out, const int isFinal, global oclResults *results) {
   int index = get_global_id(0);
   if (index < oldNumPoints) {
      //The map won't have been reset properly for active cells that were added in the 
      //last iteration, so reset it properly now unless the map value has already been
      //overwritten by update2DMap
      if(map->pointsMask[index] >= MAX_ACTIVE_CELLS) {
         int temp = map->pointsMask[index] - MAX_ACTIVE_CELLS;
         if (map->activeCellsMap[temp] >= MAX_ACTIVE_CELLS) {
            map->activeCellsMap[temp] = -1;
         }
      }
   }
   if (index < numPoints) {

      float cosTh = cos(points->offset.w);
      float sinTh = sin(points->offset.w);

      float2 tempP;
      tempP.x = points->pointX[index] - points->laserOffset.x;
      tempP.y = points->pointY[index] - points->laserOffset.y;
      points->pointX[index] = (tempP.x * cosTh - tempP.y * sinTh + points->offset.x)
                              + points->laserOffset.x;
      points->pointY[index] = (tempP.x * sinTh + tempP.y * cosTh + points->offset.y)
                              + points->laserOffset.y;
      points->pointZ[index] += points->offset.z;

      //next points
      tempP.x = points->pointNextX[index] - points->laserOffset.x; 
      tempP.y = points->pointNextY[index] - points->laserOffset.y; 
      points->pointNextX[index] = (tempP.x * cosTh - tempP.y * sinTh + points->offset.x)
                              + points->laserOffset.x;
      points->pointNextY[index] = (tempP.x * sinTh + tempP.y * cosTh + points->offset.y)
                              + points->laserOffset.y;
      points->pointNextZ[index] += points->offset.z;

      //Check to see if the point is valid to use for the position tracking
      //this cycle
      float x = points->pointX[index] - points->laserOffset.x;
      float y = points->pointY[index] - points->laserOffset.y;
      float d = x * x + y * y;
      //TODO: check the height is working correctly
      float partThrough = (float) index / (float) numPoints;
      if (points->pointZ[index] >= config->MapMaxHeight ||
            //points->pointZ[index] < config->MapMinHeight ||
            (config->LaserMaxAlign > 0.0f && d > config->LaserMaxAlign * config->LaserMaxAlign) 
            || (/*(partThrough < 0.35f || partThrough > 0.9f)  &&*/ d < config->LaserMinDist * config->LaserMinDist) || d < 0.01) {
         map->pointsMask[index] = -1;
      } else {
         map->pointsMask[index] = 1;
      }

   }

   if (index == 0 && isFinal == 1 && map->failCount == 0) {
      map->robotOff.x += results->finalOffset.x;
      map->robotOff.y += results->finalOffset.y;
      while (map->robotOff.x > config->MapCellWidth) {
         results->cellShift.x += 1;
         map->robotOff.x -= config->MapCellWidth;
      }
      while (map->robotOff.y > config->MapCellWidth) {
         results->cellShift.y += 1;
         map->robotOff.y -= config->MapCellWidth;
      }
      while (map->robotOff.x < -config->MapCellWidth) {
         results->cellShift.x -= 1;
         map->robotOff.x += config->MapCellWidth;
      }
      while (map->robotOff.y < -config->MapCellWidth) {
         results->cellShift.y -= 1;
         map->robotOff.y += config->MapCellWidth;
      }
   }
}

//TODO: look at including hcell propogation
__kernel void getNearestByH(constant configValues *config, global oclLocalMap3D *map,
      global oclLaserPoints *points, const int numPoints, global float *out) {
   int localId = get_local_id(0);
   int groupNum = get_group_id(0);
   int groupSize = get_local_size(0);
   int index = (groupNum * groupSize + localId)/WARP_SIZE;
   int warpIndex = localId % WARP_SIZE;
   int warpNum = localId / WARP_SIZE;

   float min = INFINITY;
   float x,y;

   //testing only
   int globalId = get_global_id(0);

   local int cellIndex[LOCAL_SIZE];
   local float mins[LOCAL_SIZE];
   local int minCells[LOCAL_SIZE];
   //out[globalId] = -2;
   if (index < numPoints && map->pointsMask[index] > 0 && 
         points->pointZ[index] >= config->MapMinHeight) {
      float3 laserPoint;
      float3 laserPointAdj;
      int cellX;
      int cellY;
      //One laser point for each warp
      laserPoint.x = points->pointX[index];
      laserPoint.y = points->pointY[index];
      laserPoint.z = points->pointZ[index];
      laserPointAdj = laserPoint - points->laserOffset;
      int mapIndex = getMapIndex(laserPoint.x, laserPoint.y, &cellX, &cellY, config);
      if (mapIndex >= 0) {
         float l2 = config->l;
         if (config->UseVariableL) {
            float d = pown(laserPointAdj.x, 2) +
                      pown(laserPointAdj.y, 2);
            d = sqrt(d);
            l2 = l2 * (config->AlignmentDFix - d)/config->AlignmentDFix;
         }
         l2 = l2 * l2;

         cellX += warpIndex % 6 - 2;
         cellY += warpIndex / 6 - 2;
         if (cellX < 0 || cellX >= config->MapDimWidth || cellY < 0 ||
               cellY >= config->MapDimWidth) {
            cellIndex[localId] = -1;
            mins[localId] = INFINITY;
         } else {
            mapIndex = cellY * config->MapDimWidth + cellX;
            cellIndex[localId] = map->activeCellsMap[mapIndex];
            if (cellIndex[localId] > 0 && (map->obsCount[cellIndex[localId]] < 10 || 
                     map->cellMaxZ[cellIndex[localId]] <= config->MapMinHeight)) {
               cellIndex[localId] = -1;
            }
            int zIndex = getZIndex(cellIndex[localId], laserPoint.z, config);
            float zVal = getCellZCentre(zIndex, config);
            int minIndex = -1;
            min = INFINITY;
            getCellCentre(cellX, cellY, &x, &y, config);
            float3 cellMidPoint = (float3) (x, y, zVal) - points->laserOffset;
            if (zIndex >= 0) {
               if (map->numLaserPoints[zIndex] > 0) {
                  min = getHValue(laserPointAdj, cellMidPoint, l2, config);
                  minIndex = zIndex;
               }
               if (zVal - config->MapCellHeight > config->MapMinHeight &&
                     map->numLaserPoints[zIndex - 1] > 0) {
                  cellMidPoint.z -= config->MapCellHeight;
                  float retVal = getHValue(laserPointAdj, cellMidPoint, l2, config);
                  cellMidPoint.z += config->MapCellHeight;
                  if (retVal < min) {
                     min = retVal;
                     minIndex = zIndex - 1;
                  }
               }
               if (zVal + config->MapCellHeight < config->MapMinHeight 
                     + config->MapCellHeight * config->MapDimHeight &&
                     map->numLaserPoints[zIndex + 1] > 0) {
                  cellMidPoint.z += config->MapCellHeight;
                  float retVal = getHValue(laserPointAdj, cellMidPoint, l2, config);
                  cellMidPoint.z -= config->MapCellHeight;
                  if (retVal < min) {
                     min = retVal;
                     minIndex = zIndex + 1;
                  }
               }
            }
            if (minIndex != -1) {
               int activeCellIndex = cellIndex[localId];
               cellIndex[localId] = minIndex;
               if (config->NearestAlgorithm == 1) {
                     mins[localId] = min * 
                     config->MaxObservations / (float)map->obsCount[activeCellIndex];
               } else if (config->NearestAlgorithm == 2) {
                  mins[localId] = min * (1.1f - 
                        (float) map->obsCount[activeCellIndex]) / (float) config->MaxObservations;
               }
            } else {
               mins[localId] = INFINITY;
            }
         }
         //Find the best several matching cells
         int i;
         for(i = 0; i < config->FullSearchSize; i++) {
            findMin(mins, cellIndex, localId + i, warpIndex, i);
         }
         //Search all the points stored in each of the best matching 3D cells
         float a, b, c, d;
         a = 1 / (laserPointAdj.x * laserPointAdj.x + laserPointAdj.y * laserPointAdj.y + l2);
         b = 1 - a * laserPointAdj.y * laserPointAdj.y;
         c = 1 - a * laserPointAdj.x * laserPointAdj.x;
         d = a * laserPointAdj.x * laserPointAdj.y;
         min = INFINITY;
         float3 minPoint = (float3) (0.0f, 0.0f, 0.0f);
         int minIndex = -1;
         for(i = warpIndex; i < config->FullSearchSize * config->MaxLaserPointsInCell; i+= WARP_SIZE) {
            int cellI = i / config->MaxLaserPointsInCell + warpNum * WARP_SIZE;
            int pointsI = i % config->MaxLaserPointsInCell;
            if (cellIndex[cellI] >= 0 && mins[cellI] != INFINITY &&
                 pointsI < map->numLaserPoints[cellIndex[cellI]]) {
               int activeCellIndex = cellIndex[cellI] / config->MapDimHeight;
               getCellCentre(map->xIndex[activeCellIndex], map->yIndex[activeCellIndex],
                     &x, &y, config);
               pointsI += cellIndex[cellI] * config->MaxLaserPointsInCell;
               float3 lPoint = (float3) (map->cellLaserPointsX[pointsI] + x,
                                         map->cellLaserPointsY[pointsI] + y,
                                         map->cellLaserPointsZ[pointsI]);
               float3 lPointNxt = (float3) (map->cellLaserPointsNxtX[pointsI] + x,
                                         map->cellLaserPointsNxtY[pointsI] + y,
                                         map->cellLaserPointsNxtZ[pointsI]);
               lPoint = lPoint - points->laserOffset;
               lPointNxt = lPointNxt - points->laserOffset;
               float ux, uy, dx, dy, lambda;
               ux = lPoint.x - lPointNxt.x;
               uy = lPoint.y - lPointNxt.y;
               dx = lPoint.x - laserPointAdj.x;
               dy = lPoint.y - laserPointAdj.y;
               lambda = (d * (ux * dy + uy * dx) + b * ux * dx + c * uy * dy) /
                        (b * ux * ux + c * uy * uy + 2.0f * c * ux * uy);
               if (lambda >= 1.0f) {
                  lPoint = lPointNxt;
               } else if (lambda >= 0.0f) {
                  lPoint.x = lPoint.x + lambda * ux;
                  lPoint.y = lPoint.y + lambda * uy;
               }
               lambda = getHValue(laserPointAdj, lPoint, l2, config);
               if (lambda < min) {
                  min = lambda;
                  minPoint = lPoint;
                  minIndex = pointsI;
               }
            }
         }
         mins[localId] = min;
         minCells[localId] = warpIndex;
         //Now parallel reduce in each warp to find the best lPoint for each laserPoint
         findMin(mins, minCells, localId, warpIndex, 0);
         //Get the thread that calculated the best matching point to
         //write it to the map
         if (warpIndex == minCells[warpNum * WARP_SIZE]) {
            if(mins[warpNum * WARP_SIZE] == INFINITY) {
               map->pointsMask[index] = -1;
            } else {
               map->pointMatchX[index] = minPoint.x;
               map->pointMatchY[index] = minPoint.y;
               map->pointMatchZ[index] = minPoint.z;

            }
         }
      } else if (warpIndex == 0) {
         map->pointsMask[index] = -1;
      }   
   }
}
/*__kernel void getNearestByH(constant configValues *config, global oclLocalMap3D *map,
      global oclLaserPoints *points, const int numPoints, global float *out) {
   int localId = get_local_id(0);
   int groupNum = get_group_id(0);
   int groupSize = get_local_size(0);
   int index = (groupNum * groupSize + localId)/WARP_SIZE;
   int warpIndex = localId % WARP_SIZE;
   int warpNum = localId / WARP_SIZE;

   //testing only
   int globalId = get_global_id(0);

   local int cellIndex[LOCAL_SIZE];
   local float mins[LOCAL_SIZE];
   local int minCells[LOCAL_SIZE];
   //out[globalId] = -2;
   if (index < numPoints && map->pointsMask[index] > 0) {
      float3 laserPoint;
      float3 laserPointAdj;
      int cellX;
      int cellY;
      //One laser point for each warp
      laserPoint.x = points->pointX[index];
      laserPoint.y = points->pointY[index];
      laserPoint.z = points->pointZ[index];
      laserPointAdj = laserPoint - points->laserOffset;
      int mapIndex = getMapIndex(laserPoint.x, laserPoint.y, &cellX, &cellY, config);
      if (mapIndex >= 0) {
         float l2 = config->l;
         if (config->UseVariableL) {
            float d = pown(laserPointAdj.x, 2) +
                      pown(laserPointAdj.y, 2);
            d = sqrt(d);
            l2 = l2 * (config->AlignmentDFix - d)/config->AlignmentDFix;
         }
         l2 = l2 * l2;

         cellX += warpIndex % 6 - 2;
         cellY += warpIndex / 6 - 2;
         if (cellX < 0 || cellX >= config->MapDimWidth || cellY < 0 ||
               cellY >= config->MapDimWidth) {
            cellIndex[localId] = -1;
         } else {
            mapIndex = cellY * config->MapDimWidth + cellX;
            cellIndex[localId] = map->activeCellsMap[mapIndex];
            if (cellIndex[localId] > 0 && map->obsCount[cellIndex[localId]] < 3) {
               cellIndex[localId] = -1;
            }
         }
         //Don't need a barrier here because only relies on each thread in the warp to
         //have written to cellIndex
         int i, j;
         float min = INFINITY;
         int minCell = -1;
         for (i = 0; i < WARP_SIZE; ++i) {
            if (cellIndex[warpNum * WARP_SIZE + i] >= 0) {
               int start = cellIndex[warpNum * WARP_SIZE + i] * config->MapDimHeight;
               min = INFINITY;
               minCell = -1;
               for(j = start + warpIndex; j < start + config->MapDimHeight; j += WARP_SIZE) {
                  if (map->numLaserPoints[j] > 0) {
                     float dist = fabs(((j - start + 0.5) * config->MapCellHeight) + 
                                         config->MapMinHeight - laserPoint.z);
                     if (dist < min) {
                        min = dist;
                        minCell = j;
                     }
                  }
               }
               mins[localId] = min;
               minCells[localId] = minCell;
               //Now parallel reduce down to get the min for a 2D cell
               if (warpIndex < 16 && mins[localId + 16] < mins[localId]) {
                  mins[localId] = mins[localId + 16];
                  minCells[localId] = minCells[localId + 16];
               }
               if (warpIndex < 8 && mins[localId + 8] < mins[localId]) {
                  mins[localId] = mins[localId + 8];
                  minCells[localId] = minCells[localId + 8];
               }
               if (warpIndex < 4 && mins[localId + 4] < mins[localId]) {
                  mins[localId] = mins[localId + 4];
                  minCells[localId] = minCells[localId + 4];
               }
               if (warpIndex < 2 && mins[localId + 2] < mins[localId]) {
                  mins[localId] = mins[localId + 2];
                  minCells[localId] = minCells[localId + 2];
               }
               if (warpIndex == 0 && mins[localId + 1] < mins[localId]) {
                  mins[localId] = mins[localId + 1];
                  minCells[localId] = minCells[localId + 1];
               }
               if (warpIndex == 0) {
                  cellIndex[warpNum * WARP_SIZE + i] = minCells[localId];
               }
            }
         }
         //At this point cellIndex stores the closest 3d cell. Each thread now calculates
         //h for the 3d cell
         float x, y;
         if (cellIndex[localId] >= 0) {
            getCellCentre(cellX, cellY, &x, &y, config);
            float3 cellMidPoint = (float3) (x, y, getCellZCentre(cellIndex[localId], config));
            cellMidPoint = cellMidPoint - points->laserOffset;
            mins[localId] = getHValue(laserPointAdj, cellMidPoint, l2, config);
            int activeCellIndex = cellIndex[localId] / config->MapDimHeight;
            if (config->NearestAlgorithm == 1) {
               mins[localId] = mins[localId] * 
                  config->MaxObservations / (float)map->obsCount[activeCellIndex];
            } else if (config->NearestAlgorithm == 2) {
               mins[localId] = mins[localId] * (1.1 - 
                     (float) map->obsCount[activeCellIndex]) / (float) config->MaxObservations;
            }
         } else {
            mins[localId] = INFINITY;
         }
         //Find the best several matching cells
         for(i = 0; i < config->FullSearchSize; i++) {
            findMin(mins, cellIndex, localId + i, warpIndex, i);
         }
         //Search all the points stored in each of the best matching 3D cells
         float a, b, c, d;
         a = 1 / (laserPointAdj.x * laserPointAdj.x + laserPointAdj.y * laserPointAdj.y + l2);
         b = 1 - a * laserPointAdj.y * laserPointAdj.y;
         c = 1 - a * laserPointAdj.x * laserPointAdj.x;
         d = a * laserPointAdj.x * laserPointAdj.y;
         min = INFINITY;
         float3 minPoint = (float3) (0.0f, 0.0f, 0.0f);
         for(i = warpIndex; i < config->FullSearchSize * config->MaxLaserPointsInCell; i+= WARP_SIZE) {
            int cellI = i / config->MaxLaserPointsInCell + warpNum * WARP_SIZE;
            int pointsI = i % config->MaxLaserPointsInCell;
            if (cellIndex[cellI] >= 0 && mins[cellI] != INFINITY &&
                 pointsI < map->numLaserPoints[cellIndex[cellI]]) {
               int activeCellIndex = cellIndex[cellI] / config->MapDimHeight;
               getCellCentre(map->xIndex[activeCellIndex], map->yIndex[activeCellIndex],
                     &x, &y, config);
               pointsI += cellIndex[cellI] * config->MaxLaserPointsInCell;
               float3 lPoint = (float3) (map->cellLaserPointsX[pointsI] + x,
                                         map->cellLaserPointsY[pointsI] + y,
                                         map->cellLaserPointsZ[pointsI]);
               float3 lPointNxt = (float3) (map->cellLaserPointsNxtX[pointsI] + x,
                                         map->cellLaserPointsNxtY[pointsI] + y,
                                         map->cellLaserPointsNxtZ[pointsI]);
               lPoint = lPoint - points->laserOffset;
               lPointNxt = lPointNxt - points->laserOffset;
               float ux, uy, dx, dy, lambda;
               ux = lPoint.x - lPointNxt.x;
               uy = lPoint.y - lPointNxt.y;
               dx = lPoint.x - laserPointAdj.x;
               dy = lPoint.y - laserPointAdj.y;
               lambda = (d * (ux * dy + uy * dx) + b * ux * dx + c * uy * dy) /
                        (b * ux * ux + c * uy * uy + 2.0f * c * ux * uy);
               if (lambda >= 1.0f) {
                  lPoint = lPointNxt;
               } else if (lambda >= 0.0f) {
                  lPoint.x = lPoint.x + lambda * ux;
                  lPoint.y = lPoint.y + lambda * uy;
               }
               lambda = getHValue(laserPointAdj, lPoint, l2, config);
               if (lambda < min) {
                  min = lambda;
                  minPoint = lPoint;
               }
            }
         }
         mins[localId] = min;
         minCells[localId] = warpIndex;
         //Now parallel reduce in each warp to find the best lPoint for each laserPoint
         findMin(mins, minCells, localId, warpIndex, 0);
         //Get the thread that calculated the best matching point to
         //write it to the map
         if (warpIndex == minCells[warpNum * WARP_SIZE]) {
            if(mins[warpNum * WARP_SIZE] == INFINITY) {
               map->pointsMask[index] = -1;
            } else {
               map->pointMatchX[index] = minPoint.x;
               map->pointMatchY[index] = minPoint.y;
               map->pointMatchZ[index] = minPoint.z;
            }
         }
      } else if (warpIndex == 0) {
         map->pointsMask[index] = -1;
      }   
   }
}*/

__kernel void calcOffsetParams(constant configValues *config, global oclLocalMap3D *map,
      global oclLaserPoints *points, const int numPoints, global float *out) {
   int index = get_global_id(0);
   int localId = get_local_id(0);
   int groupSize = get_local_size(0);
   int warpIndex = localId % WARP_SIZE;
   int numWarps = LOCAL_SIZE / WARP_SIZE;

   local float A[WARP_SIZE][4][4];
   local float4 B[LOCAL_SIZE];
   local int goodCount[LOCAL_SIZE];

   if (localId < WARP_SIZE) {
      A[localId][0][0] = 0.0f;
      A[localId][1][0] = 0.0f;
      A[localId][2][0] = 0.0f;
      A[localId][3][0] = 0.0f;
      A[localId][0][1] = 0.0f;
      A[localId][1][1] = 0.0f;
      A[localId][2][1] = 0.0f;
      A[localId][3][1] = 0.0f;
      A[localId][0][2] = 0.0f;
      A[localId][1][2] = 0.0f;
      A[localId][2][2] = 0.0f;
      A[localId][3][2] = 0.0f;
      A[localId][0][3] = 0.0f;
      A[localId][1][3] = 0.0f;
      A[localId][2][3] = 0.0f;
      A[localId][3][3] = 0.0f;
   }
   B[localId] = 0.0f;
   goodCount[localId] = 0;
   barrier(CLK_LOCAL_MEM_FENCE);
   if (index < numPoints && map->pointsMask[index] >= 0 
         && points->pointZ[index] >= config->MapMinHeight) {
      float3 lPoint = (float3) (map->pointMatchX[index], map->pointMatchY[index], map->pointMatchZ[index]);
      float3 lPointAdj = lPoint + points->laserOffset;
      float3 point = (float3) (points->pointX[index], points->pointY[index], points->pointZ[index]);
      point = point - points->laserOffset;
      int i,j;
      int cellIndex = getMapIndex(lPointAdj.x, lPointAdj.y, &i, &j, config);
      if (cellIndex >= 0) {
         cellIndex = map->activeCellsMap[cellIndex];
         int obsCount = map->obsCount[cellIndex];
         if (obsCount >= 3) {
            float fac = 1.0f;
            if (config->UseFactor) {
               fac = sqrt(point.x * point.x + point.y * point.y);
               fac = (fac / config->MaxLaserDist) * (map->obsCount[cellIndex] 
                     / (float)config->MaxObservations);
               if (fac < config->MinFactor) {
                  fac = config->MinFactor;
               }
            }
            float x12, y12, z12, k;
            float temp;
            if (config->UseSimpleH) {
               x12 = lPoint.x * lPoint.x;
               y12 = lPoint.y * lPoint.y;
               k = 1.0f / (x12 + y12 + config->l * config->l);

               temp = fac * (1.0f - y12 * k);
               atomicFloatAddLocal(&(A[warpIndex][0][0]), temp);
               temp = fac * lPoint.x * lPoint.y * k;
               atomicFloatAddLocal(&(A[warpIndex][0][1]), temp);
               temp = fac * (-point.y + lPoint.y * (point.y * lPoint.y + point.x * lPoint.x) * k);
               atomicFloatAddLocal(&(A[warpIndex][0][2]), temp);

               temp = fac * (1.0f - x12 * k);
               atomicFloatAddLocal(&(A[warpIndex][1][1]), temp);
               temp = fac * (point.x - lPoint.x * (point.y * lPoint.y + point.x * lPoint.x) * k);
               atomicFloatAddLocal(&(A[warpIndex][1][2]), temp);

               temp = fac * (x12 + y12 - pown(point.y * lPoint.y + point.x * lPoint.x, 2) * k);
               atomicFloatAddLocal(&(A[warpIndex][2][2]), temp);

               B[localId].s0 = fac * (point.x - lPoint.x - lPoint.y * 
                     (lPoint.y * point.x - lPoint.x * point.y) * k);
               B[localId].s1 = fac * (point.y - lPoint.y + lPoint.x * 
                     (lPoint.y * point.x - lPoint.x * point.y) * k);
               B[localId].s2 = fac * (lPoint.y * point.x - lPoint.x * point.y) * (-1.0f +
                     (lPoint.y * point.y + lPoint.x * point.x) * k);
            } else {
               //if (index % 4 == 0) { out[index/4] = point.z; }
               x12 = lPoint.x * lPoint.x;
               y12 = lPoint.y * lPoint.y;
               z12 = lPoint.z * lPoint.z;
               k = 1.0f / (x12 + y12 + z12 + config->l * config->l);

               temp = fac * (1.0f - (z12 + y12) * k);
               atomicFloatAddLocal(&(A[warpIndex][0][0]), temp);
               temp = fac * (lPoint.x * lPoint.y * k);
               atomicFloatAddLocal(&(A[warpIndex][0][1]), temp);
               temp = fac * (lPoint.x * lPoint.z * k);
               atomicFloatAddLocal(&(A[warpIndex][0][2]), temp);
               temp = fac * (((point.y * (z12 + y12) + 
                                lPoint.x * lPoint.y * point.x) * k) - point.y);
               atomicFloatAddLocal(&(A[warpIndex][0][3]), temp);

               temp = fac * (1.0f - (z12 + x12) * k);
               atomicFloatAddLocal(&(A[warpIndex][1][1]), temp);
               temp = fac * (lPoint.y * lPoint.z * k);
               atomicFloatAddLocal(&(A[warpIndex][1][2]), temp);
               temp = fac * (point.x - (point.x * z12 + lPoint.x  * lPoint.y * point.y
                        + x12 * point.x) * k);
               atomicFloatAddLocal(&(A[warpIndex][1][3]), temp);

               temp = fac * (1.0f - (x12 + y12) * k);
               atomicFloatAddLocal(&(A[warpIndex][2][2]), temp);
               temp = fac * ((lPoint.y * lPoint.z * point.x - lPoint.x * lPoint.z * point.y) * k);
               atomicFloatAddLocal(&(A[warpIndex][2][3]), temp);
               temp = fac * (x12 + y12 + k * (point.y * point.y * (y12 + z12) + 
                               point.x * point.x * (x12 + z12) + point.x * point.y * lPoint.x * lPoint.y));
               atomicFloatAddLocal(&(A[warpIndex][3][3]), temp);

               B[localId].s0 = fac * (point.x - lPoint.x + k * 
                     (lPoint.x * (lPoint.z * point.z + lPoint.y * point.y)
                             - point.x * (y12 + z12)));
               B[localId].s1 = fac * (point.y - lPoint.y + k * (lPoint.y * 
                             (lPoint.z * point.z + lPoint.x * point.x)
                             - point.y * (x12 + z12)));
               B[localId].s2 = fac * (point.z - lPoint.z + k * (lPoint.z * 
                             (lPoint.x * point.x + lPoint.y * point.y)
                             - point.z * (x12 + y12)));
               B[localId].s3 = fac * (lPoint.x * point.y - lPoint.y * point.x + k * (
                          lPoint.z * point.z * (lPoint.y * point.x - lPoint.x * point.y) +
                          lPoint.y * point.y * (lPoint.y * point.x - lPoint.x * point.y) +
                          lPoint.x * point.x * (lPoint.y * point.x - lPoint.x * point.y)));
            }
            goodCount[localId]++;
         }
      }
   }
   if (warpIndex < 16) {
      B[localId] += B[localId + 16];
      goodCount[localId] += goodCount[localId + 16];
   }
   if (warpIndex < 8) {
      B[localId] += B[localId + 8];
      goodCount[localId] += goodCount[localId + 8];
   }
   if (warpIndex < 4) {
      B[localId] += B[localId + 4];
      goodCount[localId] += goodCount[localId + 4];
   }
   if (warpIndex < 2) {
      B[localId] += B[localId + 2];
      goodCount[localId] += goodCount[localId + 2];
   }
   if (warpIndex < 1) {
      B[localId] += B[localId + 1];
      goodCount[localId] += goodCount[localId + 1];
   }
   //Need a barrier here as reading values written to local memory by threads in
   //other warps - at this point, the first thread in each warp stores the
   //sum of all the threads in the warp
   barrier(CLK_LOCAL_MEM_FENCE);
   //LOCAL_SIZE max is 512 so there are max 16 warps
   if (localId < 16 && localId < numWarps) {
      B[localId] = B[localId * WARP_SIZE];
      goodCount[localId] = goodCount[localId * WARP_SIZE];
   } else if (localId < 16) {
      B[localId] = 0.0f;
      goodCount[localId] = 0;
   }
   if(localId < 16) {
      A[localId][0][0] += A[localId + 16][0][0];
      A[localId][1][0] += A[localId + 16][1][0];
      A[localId][2][0] += A[localId + 16][2][0];
      A[localId][3][0] += A[localId + 16][3][0];
      A[localId][0][1] += A[localId + 16][0][1];
      A[localId][1][1] += A[localId + 16][1][1];
      A[localId][2][1] += A[localId + 16][2][1];
      A[localId][3][1] += A[localId + 16][3][1];
      A[localId][0][2] += A[localId + 16][0][2];
      A[localId][1][2] += A[localId + 16][1][2];
      A[localId][2][2] += A[localId + 16][2][2];
      A[localId][3][2] += A[localId + 16][3][2];
      A[localId][0][3] += A[localId + 16][0][3];
      A[localId][1][3] += A[localId + 16][1][3];
      A[localId][2][3] += A[localId + 16][2][3];
      A[localId][3][3] += A[localId + 16][3][3];
   }
   if (localId < 8) {
      A[localId][0][0] += A[localId + 8][0][0];
      A[localId][1][0] += A[localId + 8][1][0];
      A[localId][2][0] += A[localId + 8][2][0];
      A[localId][3][0] += A[localId + 8][3][0];
      A[localId][0][1] += A[localId + 8][0][1];
      A[localId][1][1] += A[localId + 8][1][1];
      A[localId][2][1] += A[localId + 8][2][1];
      A[localId][3][1] += A[localId + 8][3][1];
      A[localId][0][2] += A[localId + 8][0][2];
      A[localId][1][2] += A[localId + 8][1][2];
      A[localId][2][2] += A[localId + 8][2][2];
      A[localId][3][2] += A[localId + 8][3][2];
      A[localId][0][3] += A[localId + 8][0][3];
      A[localId][1][3] += A[localId + 8][1][3];
      A[localId][2][3] += A[localId + 8][2][3];
      A[localId][3][3] += A[localId + 8][3][3];
      B[localId] += B[localId + 8];
      goodCount[localId] += goodCount[localId + 8];
   }
   if (localId < 4) {
      A[localId][0][0] += A[localId + 4][0][0];
      A[localId][1][0] += A[localId + 4][1][0];
      A[localId][2][0] += A[localId + 4][2][0];
      A[localId][3][0] += A[localId + 4][3][0];
      A[localId][0][1] += A[localId + 4][0][1];
      A[localId][1][1] += A[localId + 4][1][1];
      A[localId][2][1] += A[localId + 4][2][1];
      A[localId][3][1] += A[localId + 4][3][1];
      A[localId][0][2] += A[localId + 4][0][2];
      A[localId][1][2] += A[localId + 4][1][2];
      A[localId][2][2] += A[localId + 4][2][2];
      A[localId][3][2] += A[localId + 4][3][2];
      A[localId][0][3] += A[localId + 4][0][3];
      A[localId][1][3] += A[localId + 4][1][3];
      A[localId][2][3] += A[localId + 4][2][3];
      A[localId][3][3] += A[localId + 4][3][3];
      B[localId] += B[localId + 4];
      goodCount[localId] += goodCount[localId + 4];
   }
   if (localId < 2) {
      A[localId][0][0] += A[localId + 2][0][0];
      A[localId][1][0] += A[localId + 2][1][0];
      A[localId][2][0] += A[localId + 2][2][0];
      A[localId][3][0] += A[localId + 2][3][0];
      A[localId][0][1] += A[localId + 2][0][1];
      A[localId][1][1] += A[localId + 2][1][1];
      A[localId][2][1] += A[localId + 2][2][1];
      A[localId][3][1] += A[localId + 2][3][1];
      A[localId][0][2] += A[localId + 2][0][2];
      A[localId][1][2] += A[localId + 2][1][2];
      A[localId][2][2] += A[localId + 2][2][2];
      A[localId][3][2] += A[localId + 2][3][2];
      A[localId][0][3] += A[localId + 2][0][3];
      A[localId][1][3] += A[localId + 2][1][3];
      A[localId][2][3] += A[localId + 2][2][3];
      A[localId][3][3] += A[localId + 2][3][3];
      B[localId] += B[localId + 2];
      goodCount[localId] += goodCount[localId + 2];
   }
   if (localId == 0) {
      A[localId][0][0] += A[localId + 1][0][0];
      A[localId][1][0] += A[localId + 1][1][0];
      A[localId][2][0] += A[localId + 1][2][0];
      A[localId][3][0] += A[localId + 1][3][0];
      A[localId][0][1] += A[localId + 1][0][1];
      A[localId][1][1] += A[localId + 1][1][1];
      A[localId][2][1] += A[localId + 1][2][1];
      A[localId][3][1] += A[localId + 1][3][1];
      A[localId][0][2] += A[localId + 1][0][2];
      A[localId][1][2] += A[localId + 1][1][2];
      A[localId][2][2] += A[localId + 1][2][2];
      A[localId][3][2] += A[localId + 1][3][2];
      A[localId][0][3] += A[localId + 1][0][3];
      A[localId][1][3] += A[localId + 1][1][3];
      A[localId][2][3] += A[localId + 1][2][3];
      A[localId][3][3] += A[localId + 1][3][3];
      B[localId] += B[localId + 1];

      /*B[localId] /= 4;
      A[0][0][0] /= 4;
      A[0][1][0] /= 4;
      A[0][2][0] /= 4;
      A[0][3][0] /= 4;
      A[0][0][1] /= 4;
      A[0][1][1] /= 4;
      A[0][2][1] /= 4;
      A[0][3][1] /= 4;
      A[0][0][2] /= 4;
      A[0][1][2] /= 4;
      A[0][2][2] /= 4;
      A[0][3][2] /= 4;
      A[0][0][3] /= 4;
      A[0][1][3] /= 4;
      A[0][2][3] /= 4;
      A[0][3][3] /= 4;*/

      goodCount[localId] += goodCount[localId + 1];
      atomicFloatAdd(&(map->B[0]), B[0].s0);
      atomicFloatAdd(&(map->B[1]), B[0].s1);
      atomicFloatAdd(&(map->B[2]), B[0].s2);
      atomicFloatAdd(&(map->B[3]), B[0].s3);
      atomic_add(&(map->goodCount), goodCount[0]);
   }
   if (localId < 4) {
      atomicFloatAdd(&(map->A[localId][0]), A[0][localId][0]);
      atomicFloatAdd(&(map->A[localId][1]), A[0][localId][1]);
      atomicFloatAdd(&(map->A[localId][2]), A[0][localId][2]);
      atomicFloatAdd(&(map->A[localId][3]), A[0][localId][3]);
   }
}

__kernel void calcMatrix(constant configValues *config, global oclLocalMap3D *map,
      global oclLaserPoints *points, global oclResults *results, global oclPartialResults *partialResults, global float *out) {

   local float a[4][4];
   local float b[4];

   event_t evt = async_work_group_copy((local float *)a, (global float *)map->A, 16, 0);
   wait_group_events(1, &evt);
   evt = async_work_group_copy((local float *)b, (global float *)map->B, 4, 0);
   wait_group_events(1, &evt);
   

   local float shift[4];
   int index = get_global_id(0);

   if (index == 0) {
      //out[0] = map->goodCount;
      /*out[0] = a[0][0];
      out[1] = a[0][1];
      out[2] = a[0][2];
      out[3] = a[0][3];
      out[4] = a[1][0];
      out[5] = a[1][1];
      out[6] = a[1][2];
      out[7] = a[1][3];
      out[8] = a[2][0];
      out[9] = a[2][1];
      out[10] = a[2][2];
      out[11] = a[2][3];
      out[12] = a[3][0];
      out[13] = a[3][1];
      out[14] = a[3][2];
      out[15] = a[3][3];
      out[16] = b[0];
      out[17] = b[1];
      out[18] = b[2];
      out[19] = b[3];*/
   }

   //Do inverse(a) * b. Calculations are simplfied as A is an upper triangular matrix
   if (config->UseSimpleH) {
      if (index < 3) {
         shift[index] = - b[index] / a[index][index];
      }
      if (index < 2) {
         shift[index] += (a[index][index + 1] * b[index + 1]) / (a[index][index] * a[index + 1][index + 1]);
      }
      if (index == 0) {
         shift[index] += b[2] * (a[0][2] * a[1][1] - a[0][1] * a[1][2]) / (a[0][0] * a[1][1] * a[2][2]);
         shift[3] = shift[2];
         shift[2] = 0;
      }
   } else {
      if (index < 4) {
         shift[index] = - b[index] / a[index][index];
      }
      if (index < 3) {
         shift[index] += (a[index][index + 1] * b[index + 1]) / (a[index][index] * a[index + 1][index + 1]);
      }
      if (index < 2) {
         shift[index] += b[index + 2] * (a[index][index + 2] * a[index +1 ][index + 1] - 
                    a[index][index + 1] *  a[index + 1][index + 2]) /
                    (a[index][index] * a[index + 1][index + 1] * a[index + 2][index + 2]);
      }
      if (index == 0) {
         shift[index] += b[3] * (a[0][1] * a[1][2] * a[2][3] + a[0][3] * a[1][1] * a[2][2] -
               a[0][1] * a[1][3] * a[2][2] - a[0][2] * a[1][1] * a[2][3]) /
               (a[0][0] * a[1][1] * a[2][2] * a[3][3]);
      }
   }

   if (index < 4) {
      map->A[index][0] = 0;
      map->A[index][1] = 0;
      map->A[index][2] = 0;
      map->A[index][3] = 0;
      map->B[index] = 0;
   }
   if (index == 0) {
      /*out[0] = shift[0];
      out[1] = shift[1];
      out[2] = shift[2];
      out[3] = shift[3];*/
      //out[0] = map->numIterations;
      partialResults->proceed = 1;
      if (map->goodCount >= config->MinGoodCount) {
         map->numIterations++;
         //out[0] = -3;
         points->offset = vload4(0, shift);
         points->offset.z = 0;
         results->finalOffset += points->offset;
         ANGNORM(results->finalOffset.w);
         if ((fabs(shift[0]) <= config->MaxErrX && fabs(shift[1]) <= config->MaxErrY &&
               fabs(shift[3]) <= config->MaxErrTh) || map->numIterations == config->MaxIterations) {
            //Loop is finished
            partialResults->proceed = 0;
         }
         map->badScan = 0;
      } else {
         //out[0] = -6;
         points->offset.x = 0;
         points->offset.y = 0;
         points->offset.z = 0;
         points->offset.w = 0;
         partialResults->proceed = 0;
         map->badScan = 1;

         results->finalOffset.x = 0;
         results->finalOffset.y = 0;
         results->finalOffset.w = 0;
      }
      map->goodCount = 0;
      /*out[24] = results->finalOffset.x;
      out[25] = results->finalOffset.y;
      out[26] = results->finalOffset.z;
      out[27] = results->finalOffset.w;*/

      out[0] = 1;
      if (partialResults->proceed == 0) {
         out[0] = 0;
         map->numIterations = 0;
         //Test if should update the map
         if (isnan(results->finalOffset.x) || isnan(results->finalOffset.y) ||
             isnan(results->finalOffset.w) || fabs(results->finalOffset.x) > config->MaxMoveX ||
             fabs(results->finalOffset.y) > config->MaxMoveY || 
             fabs(results->finalOffset.w) > config->MaxMoveTh) {
            map->failCount++;
            if (map->failCount > config->MaxFail) {
               map->failCount = 0;
               map->addSkip = config->AddSkipCount + 1;
            }

            results->finalOffset.x = 0;
            results->finalOffset.y = 0;
            results->finalOffset.w = 0;
         } else {
            out[0] = 1; //Shows successful alignment
            map->failCount = 0;
         }
         if (map->scanSkip >= config->MaxScanSkip) {
            map->addSkip++;
         }
         map->scanSkip++;
      }

      /*if (partialResults->proceed == 0 && map->failCount == 0) {
         map->robotOff.x += results->finalOffset.x;
         map->robotOff.y += results->finalOffset.y;
         while (map->robotOff.x > config->MapCellWidth) {
            results->cellShift.x += 1;
            map->robotOff.x -= config->MapCellWidth;
         }
         while (map->robotOff.y > config->MapCellWidth) {
            results->cellShift.y += 1;
            map->robotOff.y -= config->MapCellWidth;
         }
         while (map->robotOff.x < -config->MapCellWidth) {
            results->cellShift.x -= 1;
            map->robotOff.x += config->MapCellWidth;
         }
         while (map->robotOff.y < -config->MapCellWidth) {
            results->cellShift.y -= 1;
            map->robotOff.y += config->MapCellWidth;
         }
      }*/
   }
}

kernel void initialUpdatePoints(constant configValues *config, global oclLocalMap3D *map,
      global oclLaserPoints *points, global oclResults *results, const int numPoints, global float *out) {

   int threadIndex = get_global_id(0);
   int cellIndex;
   if (threadIndex < numPoints && map->pointsMask[threadIndex] >= 0 && map->failCount == 0 &&
         map->badScan == 0 && (map->scanSkip > config->MaxScanSkip || map->scanSkip == -1)) {
      map->pointMatchX[threadIndex] = -1.0f;
      int i,j;
      int mapIndex = getMapIndex(points->pointX[threadIndex], points->pointY[threadIndex], &i, &j, config);
      int shiftX = i - results->cellShift.x;
      int shiftY = j - results->cellShift.y;   
      //Ensure that the point fits on the map and will still be on the map after it is shifted
      if (mapIndex >= 0 && shiftX >= 0 && shiftX < config->MapDimWidth && shiftY >= 0 &&
            shiftY < config->MapDimWidth) {
         //Check to see if there is an existing active cell at the location. Note that several points
         //could potentially be looking at the same cell
         if (map->activeCellsMap[mapIndex] == -1) {
            //There isn't an active cell, so add MAX_ACTIVE_CELLS to the map location so other threads
            //know this is a new cell.
            int res = atomic_add(&(map->activeCellsMap[mapIndex]), MAX_ACTIVE_CELLS + 1);
            //Only create a new active cell if this thread was the one that changed the value in the map
            //from -1 to make sure only one thread creates the new active cell
            if (res == -1) {
               cellIndex = atomic_inc(&(map->nextEmptyCell));
               if (cellIndex < MAX_ACTIVE_CELLS) {
                  atomic_max(&(results->largestActiveCell), cellIndex + 1);
                  cellIndex = map->indexEmptyCells[cellIndex];
                  map->xIndex[cellIndex] = i;
                  map->yIndex[cellIndex] = j;
                  map->cellMaxZ[cellIndex] = points->pointZ[threadIndex];
                  map->cellMinZ[cellIndex] = points->pointZ[threadIndex];
               } else {
                  atomic_dec(&(map->nextEmptyCell));
                  //An atomic exchange here to set map index back to -1
                  atomic_xchg(&(map->activeCellsMap[mapIndex]), -1);
               }
            }
            //pointsMask is used to inform later kernels which active cell the point belongs in
            //but if the cell is new, set it to the index in the activeCellsMap instead,
            //offset by MAX_ACTIVE_CELLS (so the next kernel can tell the difference between the
            //two situations)
            map->pointsMask[threadIndex] = MAX_ACTIVE_CELLS + mapIndex;
         } else if (map->activeCellsMap[mapIndex] >= MAX_ACTIVE_CELLS) {
            //Another thread already created a new active cell for this location. We don't know the
            //active cell index for it, so we can't do anything further
            map->pointsMask[threadIndex] = MAX_ACTIVE_CELLS + mapIndex;
         } else {
            //There is already an active cell at this location
            cellIndex = map->activeCellsMap[mapIndex];
            map->pointsMask[threadIndex] = cellIndex;
            int cell3DIndex = getZIndex(cellIndex, points->pointZ[threadIndex], config);
            if (cell3DIndex >= 0) {
               if (map->addSkip > config->AddSkipCount + 1) {
                  //Replacing all points in the 3D cell, so need to set numLaserPoints to 0 for the 3D
                  //cell. As only setting to 0, don't need to worry about doing it atomically
                  map->numLaserPoints[cell3DIndex] = 0;
               } else if (map->numLaserPoints[cell3DIndex] == 0) {
                  //updating and 3D cell is empty. Use pointMatchX to "mark" the point so it will
                  //be added to the map
                  map->pointMatchX[threadIndex] = 1.0f;
               } else {
                  //updating but the 3D cell is not empty
                  map->pointMatchX[threadIndex] = -1.0f;
               }
               //Add something to the lifeCount to make sure that the cell isn't deleted.
               //Don't need atomic add as don't care about result - we only care that at least
               //one add will take place.
               map->lifeCount[cellIndex] = -2;
            } else if (points->pointZ[threadIndex] >= config->MapMaxHeight) {
               //The point is too high so don't store it
               map->pointsMask[threadIndex] = -1;
            }
         }
      } else {
         map->pointsMask[threadIndex] = -1;
      }
   }
}

//COmmunicate with CPU to make sure largestActiveCell is correct

/*__kernel void updateActiveCells(constant configValues *config, global oclLocalMap3D *map,
     global oclResults *results) {
   int index = get_global_id(0);
   if (index < results->largestActiveCell && map->failCount == 0) {
      sharedMap->cellStatus[index] = -1;
      if (map->xIndex[index] >= 0) {
         int i = map->yIndex[index] * config->MapDimWidth + map->xIndex[index];
         map->xIndex[index] -= results->cellShift.x;
         map->yIndex[index] -= results->cellShift.y;
         sharedMap->mapIndex[index] = map->yIndex[index] * config->MapDimWidth + map->xIndex[index];
         int activeCellsMapVal = map->activeCellsMap[i];
         map->activeCellsMap[i] = -1;
         if ((map->scanSkip > config->MaxScanSkip || map->scanSkip == -1) && map->badScan == 0) {

            if (activeCellsMapVal >= MAX_ACTIVE_CELLS) {
               //It is a new cell
               map->activeCellsMap[i] = MAX_ACTIVE_CELLS + index;
               sharedMap->cellStatus[index] = 0;
            } else if (map->scanSkip != -1) {
               map->prevObsCount[index] = map->obsCount[index];
               map->lifeCount[index]--;
               //TODO: finish code for deleting a cell
               //TODO: make this more parallel
               if (map->lifeCount[index] <= 0 || map->xIndex[index] < 0 || map->yIndex[index] < 0
                     || map->xIndex[index] >= config->MapDimWidth || 
                     map->yIndex[index] >= config->MapDimWidth) {
                  map->xIndex[index] = -1;
                  map->obsCount[index] = 0;
                  map->prevObsCount[index] = 0;
                  int start = index * config->MapDimHeight;
                  //TODO: really make this parallel
                  for(i = start; i < start + config->MapDimHeight; ++i) {
                     map->numLaserPoints[i] = 0;
                  }
                  int freeIndex = atomic_dec(&(map->nextEmptyCell)) - 1;
                  map->indexEmptyCells[freeIndex] = index;
               } else {
                  sharedMap->cellStatus[index] = map->lifeCount[index];
               }
            }
         }
      }
   }
}*/

/*__kernel void updateActiveCells(constant configValues *config, global oclLocalMap3D *map,
     global oclResults *results, global oclSharedMap *sharedMap, global float *out) {
   int index = get_global_id(0);

   int j;

   if (get_global_id(0) == 0) {
      for(j = 0; j < results->largestActiveCell; j++) {
            index = j;
   if (index < results->largestActiveCell) {
      sharedMap->cellStatus[index] = -9;
   }
   }
   }
   
   if (get_global_id(0) == 0) {
      for (j = 0; j < results->largestActiveCell; j++) {
         index = j;
         if (index < results->largestActiveCell) {
            //sharedMap->cellStatus[index] = -9;
            int i = map->yIndex[index] * config->MapDimWidth + map->xIndex[index];
            if (i >= 0 && i < MAP_SIZE) {
               map->activeCellsMap[i] = -1;
            }
            //out[index] = sharedMap->cellStatus[index];
         }
      }
   }
   if (get_global_id(0) == 0) {
      for(j = 0; j < results->largestActiveCell; j++) {
            index = j;
   if (index < results->largestActiveCell && index < 500) {
      out[index] = sharedMap->cellStatus[index];
   }
   }
   }
}*/

__kernel void updateActiveCells(constant configValues *config, global oclLocalMap3D *map,
     global oclResults *results, global oclSharedMap *sharedMap, global float *out) {
   int index = get_global_id(0);
   int localSize = get_local_size(0);
   int localIndex = get_local_id(0);
   local int toDelete[LOCAL_SIZE];
   local int toDeleteIndex;
   if (localIndex == 0) {
      toDeleteIndex = 0;
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (index < results->largestActiveCell && map->failCount == 0) {
      if (map->xIndex[index] >= 0) {
         int i = map->yIndex[index] * config->MapDimWidth + map->xIndex[index];
         map->xIndex[index] -= results->cellShift.x;
         map->yIndex[index] -= results->cellShift.y;
         int activeCellsMapVal = map->activeCellsMap[i];
	      //Doing this here breaks things at home
         //map->activeCellsMap[i] = -1;
         int pointStatus = -1;
         if ((map->scanSkip > config->MaxScanSkip || map->scanSkip == -1) && 
                  map->badScan == 0) {
            pointStatus = 1;
            if (activeCellsMapVal >= MAX_ACTIVE_CELLS) {
               //It is a new cell
               map->activeCellsMap[i] = MAX_ACTIVE_CELLS + index;
               pointStatus = 0;
            } else if (map->lifeCount[index] == -2) {
               map->activeCellsMap[i] = -1;
               pointStatus = 0;
            } else if (map->scanSkip != -1) {
               map->activeCellsMap[i] = -1;
               map->lifeCount[index]--;
               map->prevObsCount[index] = map->obsCount[index];
               if (map->lifeCount[index] <= 0 || map->xIndex[index] < 0 || map->yIndex[index] < 0
                     || map->xIndex[index] >= config->MapDimWidth ||
                     map->yIndex[index] >= config->MapDimWidth) {
                  toDelete[atomic_inc(&toDeleteIndex)] = index;
                  pointStatus = -1;
               }
            } 
            if (map->cellMaxZ[index] < config->MapMinHeight && 
                  map->cellMaxZ[index] - map->cellMinZ[index] >= 0 &&
                  map->cellMaxZ[index] - map->cellMinZ[index] > config->FlobsticleHeight) {
               pointStatus = -2;
            } else if (map->cellMaxZ[index] < config->MapMinHeight) {
               pointStatus = -1;
            }
            
         } else {
            map->activeCellsMap[i] = -1;
         }
         if (map->scanSkip > config->MaxScanSkip || map->scanSkip == -1) {
            sharedMap->mapIndex[index] = map->yIndex[index] * config->MapDimWidth + 
                                       map->xIndex[index];
            if (pointStatus == 1) {
               sharedMap->cellStatus[index] = map->lifeCount[index];
            } else if (pointStatus == 0) {
               sharedMap->cellStatus[index] = 0;
            } else if (pointStatus == -2) {
               sharedMap->cellStatus[index] = -2;
            } else {
               sharedMap->cellStatus[index] = -1;
            }
         }
      } else if (map->scanSkip > config->MaxScanSkip || map->scanSkip == -1) {
         sharedMap->cellStatus[index] = -1;
      }
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if(localIndex < toDeleteIndex) {
      int i = toDelete[localIndex];
      map->xIndex[i] = -1;
      map->obsCount[i] = 0;
      map->prevObsCount[i] = 0;
      map->cellMinZ[i] = INFINITY;
      map->cellMaxZ[i] = -INFINITY;
      int freeIndex = atomic_dec(&(map->nextEmptyCell)) - 1;
      map->indexEmptyCells[freeIndex] = i;
   }
   int numRuns = localSize/config->MapDimHeight;
   if (localIndex < config->MapDimHeight * numRuns) {
      int i;
      for (i = localIndex; i < config->MapDimHeight * toDeleteIndex; 
            i+=numRuns * config->MapDimHeight) {
         int start = toDelete[i / config->MapDimHeight] * config->MapDimHeight;
         map->numLaserPoints[start + i % config->MapDimHeight] = 0;
      }
   }
}

kernel void finalUpdatePoints(constant configValues *config, global oclLocalMap3D *map,
         global oclLaserPoints *points, global oclResults *results, const int numPoints) {

   int threadIndex = get_global_id(0);
   int cellIndex;
   if (threadIndex < numPoints && map->pointsMask[threadIndex] >= 0 && map->failCount == 0 &&
         map->badScan == 0 && (map->scanSkip > config->MaxScanSkip || map->scanSkip == -1)) {
      cellIndex = map->pointsMask[threadIndex];
      if (cellIndex >= MAX_ACTIVE_CELLS) {
         //Point is being added to a new cell, so still need to find the cellIndex of the cell
         cellIndex -= MAX_ACTIVE_CELLS;
         if (map->activeCellsMap[cellIndex] < MAX_ACTIVE_CELLS) {
            cellIndex = -1;
         } else {
            cellIndex = map->activeCellsMap[cellIndex] - MAX_ACTIVE_CELLS;
         }
      }
      int cell3DIndex;
      if (cellIndex >= 0) {
         cell3DIndex = getZIndex(cellIndex, points->pointZ[threadIndex], config);
      } else {
         cell3DIndex = -1;
      }
      if (cell3DIndex >= 0 || points->pointZ[threadIndex] < config->MapMinHeight) {

         if ((map->addSkip > config->AddSkipCount + 1 || map->pointMatchX[threadIndex] > 0.0f ||
               map->pointsMask[threadIndex] >= MAX_ACTIVE_CELLS) && cell3DIndex >= 0) {
            int pointIndex = atomic_inc(&(map->numLaserPoints[cell3DIndex]));
            if (pointIndex < config->MaxLaserPointsInCell) {
               pointIndex += cell3DIndex * config->MaxLaserPointsInCell;
               //Add the point and pointnxt to cellLaserPoints arrays
               float x, y;
               //Get the x and y index of the matching cell. Note that need to undo the shift
               //from the previous kernel to get the right coordinates
               int i = map->xIndex[cellIndex] + results->cellShift.x;
               int j = map->yIndex[cellIndex] + results->cellShift.y;
               getCellCentre(i, j, &x, &y, config);
               map->cellLaserPointsX[pointIndex] = points->pointX[threadIndex] - x;
               map->cellLaserPointsY[pointIndex] = points->pointY[threadIndex] - y;
               map->cellLaserPointsZ[pointIndex] = points->pointZ[threadIndex];
               map->cellLaserPointsNxtX[pointIndex] = points->pointNextX[threadIndex] - x;
               map->cellLaserPointsNxtY[pointIndex] = points->pointNextY[threadIndex] - y;
               map->cellLaserPointsNxtZ[pointIndex] = points->pointNextZ[threadIndex];
            }
         }
         atomicFloatMax(&(map->cellMaxZ[cellIndex]), points->pointZ[threadIndex]);
         atomicFloatMin(&(map->cellMinZ[cellIndex]), points->pointZ[threadIndex]);
         atomic_inc(&(map->obsCount[cellIndex]));
      }
   }
}

