
#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

__kernel void initialiseMap(global oclPositionTrackConfig *config, global oclLocalMap *map,
      global oclCommon *common) {

   int index = get_global_id(0);
   int globalSize = get_global_size(0);

   for(int i = index; i < NUM_CELLS; i+= globalSize) {
      map->z[i] = 0;
      map->normX[i] = 0;
      map->normY[i] = 0;
      map->normZ[i] = 0;
      map->count[i] = 0;
   }

   if (index == 0) {
      common->numMatch = 0;
      common->distance = 0;
   }
}

int calculateCellXY(global oclPositionTrackConfig *config, float2 point, int2 cent) {
   int2 pointInd;
   pointInd.x = point.x / config->CellSize;
   pointInd.y = point.y / config->CellSize;
   pointInd -= cent;

   if (abs(pointInd.x) < config->NumCellsWidth / 2 - 1 
         && abs(pointInd.y) < config->NumCellsWidth / 2 - 1) {
      cent = cent % config->NumCellsWidth;
      pointInd += cent;
      if (pointInd.x < 0) {
         pointInd.x += config->NumCellsWidth;
      } else if (pointInd.x >= config->NumCellsWidth) {
         pointInd.x -= config->NumCellsWidth;
      }
      if (pointInd.y < 0) {
         pointInd.y += config->NumCellsWidth;
      } else if (pointInd.y >= config->NumCellsWidth) {
         pointInd.y -= config->NumCellsWidth;
      }
      return pointInd.y * config->NumCellsWidth + pointInd.x;

   } else {
      return -1;
   }
}

int calculateCellZ(global oclPositionTrackConfig *config, float point, int cent, int *diffInd) {
   int pointInd = point / config->CellSize;
   pointInd -= cent;
   *diffInd = pointInd;

   if (abs(pointInd) < config->NumCellsHeight / 2 - 1) {
      cent = cent % config->NumCellsHeight;
      pointInd += cent;
      if (pointInd < 0) {
         pointInd += config->NumCellsHeight;
      } else if (pointInd >= config->NumCellsHeight) {
         pointInd -= config->NumCellsHeight;
      }
      return pointInd;
   } else {
      return -1;
   }
}

int getCellIndex(global oclPositionTrackConfig *config, int cellXY, int cellZ) {
   return cellZ * config->NumCellsWidth * config->NumCellsWidth + cellXY;
}

/*__kernel void filterPoints(global oclPositionTrackConfig *config, global oclDepthPoints *points, 
      global oclDepthPoint *filteredPoints, const int numPoints) {

}*/

//Each float3 of rotation is a row of the rotation matrix
__kernel void transform3D(global oclDepthPoints *points, const int numPoints, float3 origin,
      float3 rotation0, float3 rotation1, float3 rotation2) {
   int index = get_global_id(0);

   if (index < numPoints && !isnan(points->pointX[index])) {
      float3 point = (float3)(points->pointX[index], points->pointY[index], points->pointZ[index]);
      float3 temp = point * rotation0;
      points->pointX[index] = temp.x + temp.y + temp.z + origin.x;
      temp = point * rotation1;
      points->pointY[index] = temp.x + temp.y + temp.z + origin.y;
      temp = point * rotation2;
      points->pointZ[index] = temp.x + temp.y + temp.z + origin.z;

   }
}

__kernel void calculateNormals(global oclPositionTrackConfig *config, global oclDepthPoints *points,
      global oclDepthPoints *normals, global oclLocalMap *map, const int numPoints, const int2 cent) {
   int index = get_global_id(0);

   if (index < numPoints && !isnan(points->pointX[index])) {

      float2 p = (float2)(points->pointX[index], points->pointY[index]);
      map->cellXY[index] = calculateCellXY(config, p, cent);
   } else if (index < numPoints) {
      map->cellXY[index] = -1;
   }

}

__kernel void alignZ(global oclPositionTrackConfig *config, global oclDepthPoints *points,
      global oclDepthPoints *normals, global oclLocalMap *map, global oclCommon *common, 
      const int numPoints, const int zCent, const float zInc) {
   int index = get_global_id(0);
   int lIndex = get_local_id(0);

   local int count[LOCAL_SIZE];
   local float distance[LOCAL_SIZE];

   count[lIndex] = 0;

   distance[lIndex] = 0;

   if (index < numPoints && !isnan(points->pointX[index])) {
      int diffZ;
      float zVal = points->pointZ[index] + zInc;
      int zCell = calculateCellZ(config, zVal, zCent, &diffZ);
      int xyCell = map->cellXY[index];
      if (zCell >= 0 && xyCell >= 0) {
         int maxTravelPos = config->MaxSearchCells;
         int maxTravelNeg = config->MaxSearchCells;
         if (diffZ > 0 && config->NumCellsHeight/2 - 1 - diffZ < maxTravelPos) {
            maxTravelPos = config->NumCellsHeight/2 -1 - diffZ;
         } else if (diffZ < 0 && config->NumCellsHeight/2 - 1 + diffZ < maxTravelNeg) {
            maxTravelNeg = config->NumCellsHeight/2 -1 + diffZ;
         }
         int cellI = getCellIndex(config, xyCell, zCell);
         float minDist = INFINITY;
         if (map->count[cellI] >= config->MinObsCount) {
            //float zCellVal = map->z[cellI] / (float)map->count[cellI];
            float zCellVal = map->z[cellI];
            minDist = zCellVal - zVal;
            maxTravelNeg = 1;
            maxTravelPos = 1;
         }
         int travel;
         for (travel = 1; travel <= maxTravelPos; travel++) {
            int z = (zCell + travel) % config->NumCellsHeight;
            int cell = getCellIndex(config, xyCell, z);
            if (map->count[cell] >= config->MinObsCount) {
               //float zCellVal = map->z[cell] / (float)map->count[cell];
               float zCellVal = map->z[cell];
               if (zCellVal - zVal < fabs(minDist)) {
                  minDist = zCellVal - zVal;
               }
               break;
            }
         }
         maxTravelNeg = travel;
         for (travel = 1; travel  <= maxTravelNeg; travel++) {
            int z = (zCell - travel);
            if (z < 0) {
               z += config->NumCellsHeight;
            }
            int cell = getCellIndex(config, xyCell, z);
            if (map->count[cell] >= config->MinObsCount) {
               //float zCellVal = map->z[cell] / (float) map->count[cell];
               float zCellVal = map->z[cell];
               if (zVal - zCellVal < fabs(minDist)) {
                  minDist = zCellVal - zVal;
               }
            }
            break;
         }
         if (minDist < INFINITY) {
            count[lIndex] = 1;
            distance[lIndex] = minDist;
            //atomic_inc(&(common->numMatch));
         }
      }
   }

   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < 128 && LOCAL_SIZE >= 256) {
      count[lIndex] += count[lIndex + 128];
      distance[lIndex] += distance[lIndex + 128];
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < 64) {
      count[lIndex] += count[lIndex + 64];
      distance[lIndex] += distance[lIndex + 64];
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < 32) {
      count[lIndex] += count[lIndex + 32];
      distance[lIndex] += distance[lIndex + 32];
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < 16) {
      count[lIndex] += count[lIndex + 16];
      distance[lIndex] += distance[lIndex + 16];
   }
   if (lIndex < 8) {
      count[lIndex] += count[lIndex + 8];
      distance[lIndex] += distance[lIndex + 8];
   }
   if (lIndex < 4) {
      count[lIndex] += count[lIndex + 4];
      distance[lIndex] += distance[lIndex + 4];
   }
   if (lIndex < 2) {
      count[lIndex] += count[lIndex + 2];
      distance[lIndex] += distance[lIndex + 2];
   }
   if (lIndex == 0) {
      count[lIndex] += count[lIndex + 1];
      distance[lIndex] += distance[lIndex + 1];
      atomic_add(&(common->numMatch), count[0]);
      atomicFloatAdd(&(common->distance), distance[0]);
   }
}

__kernel void addScan(global oclPositionTrackConfig *config, global oclDepthPoints *points,
      global oclDepthPoints *normals, global oclLocalMap *map, const int numPoints, 
      const float zChange, const float zCent) {
   int index = get_global_id(0);

   if (index < numPoints && !isnan(points->pointX[index])) {
      int diffZ;
      int zCell = calculateCellZ(config, points->pointZ[index] + zChange, zCent, &diffZ);
      int xyCell = map->cellXY[index];
      if (zCell >= 0 && xyCell >= 0) {
         int cellI = getCellIndex(config, xyCell, zCell);
         int retVal = atomic_inc(&(map->count[cellI]));
         //Todo: should this be the average? (would have to watch out for speed slowdown)
         
         if (retVal == 0) {
            atomicFloatAdd(&(map->z[cellI]), points->pointZ[index] + zChange);
         }
      }
   }
}

__kernel void clearCells(global oclPositionTrackConfig *config, global oclLocalMap *map, ocl_int3 oldPos,
      ocl_int3 newPos) {
   int index = get_global_id(0);

   //realign x and y
   if (index < config->NumCellsHeight * config->NumCellsWidth) {
      int diff = newPos.x - oldPos.x;
      int increment = 0;
      int start = 0;
      int posI = oldPos.x % config->NumCellsWidth;
      if (diff < 0) {
         increment = -1;
         start = posI - config->NumCellsWidth/2;
         diff *= -1;
      } else if (diff > 0) {
         increment = 1;
         start = posI + config->NumCellsWidth/2 + 1;
      }
      int z = index / config->NumCellsWidth;
      int y = index % config->NumCellsWidth;
      int cellI = z * config->NumCellsWidth * config->NumCellsWidth + y * config->NumCellsWidth;
      for (int i = 0; i < diff; i++, start += increment) {
         if (start >= config->NumCellsWidth) {
            start -= config->NumCellsWidth;
         } else if (start < 0) {
            start += config->NumCellsWidth;
         }
         int c = cellI + start;
         map->count[c] = 0;
         map->z[c] = 0;
         map->normX[c] = 0;
         map->normY[c] = 0;
         map->normZ[c] = 0;
      }

      //realign y
      diff = newPos.y - oldPos.y;
      increment = 0;
      start = 0;
      posI = oldPos.y % config->NumCellsWidth;
      if (diff < 0) {
         increment = -1;
         start = posI - config->NumCellsWidth/2;
         diff *= -1;
      } else if (diff > 0) {
         increment = 1;
         start = posI + config->NumCellsWidth/2 + 1;
      }
      int x = index % config->NumCellsWidth;
      z = index / config->NumCellsWidth;
      cellI = z * config->NumCellsWidth * config->NumCellsWidth + x;
      for (int i = 0; i < diff; i++, start += increment) {
         if (start >= config->NumCellsWidth) {
            start -= config->NumCellsWidth;
         } else if (start < 0) {
            start += config->NumCellsWidth;
         }
         int c = cellI + start * config->NumCellsWidth;
         map->count[c] = 0;
         map->z[c] = 0;
         map->normX[c] = 0;
         map->normY[c] = 0;
         map->normZ[c] = 0;
      }
   }
   if (index < config->NumCellsWidth * config->NumCellsWidth) {
      int diff = newPos.z - oldPos.z;
      int increment = 0;
      int start = 0;
      int posI = oldPos.z % config->NumCellsHeight;
      if (diff < 0) {
         increment = -1;
         start = posI - config->NumCellsHeight/2;
         diff *= -1;
      } else if (diff > 0) {
         increment = 1;
         start = posI + config->NumCellsHeight/2 + 1;
      }
      int x = index % config->NumCellsWidth;
      int y = index / config->NumCellsWidth;
      int cellI = y * config->NumCellsWidth + x;
      for (int i = 0; i < diff; i++, start += increment) {
         if (start >= config->NumCellsHeight) {
            start -= config->NumCellsHeight;
         } else if (start < 0) {
            start += config->NumCellsHeight;
         }
         int c = start * config->NumCellsWidth * config->NumCellsWidth + cellI;
         map->count[c] = 0;
         map->z[c] = 0;
         map->normX[c] = 0;
         map->normY[c] = 0;
         map->normZ[c] = 0;
      }
   }
}

__kernel void calculateFloorHeight(global oclPositionTrackConfig *config, global oclDepthPoints *points,
      global int *cellCounts, const int numPoints, const int skip, const float zRob) {
   int index = get_global_id(0) * skip;

   if (index < numPoints && !isnan(points->pointX[index])) {
      float zH = points->pointZ[index] - zRob;
      int zInd = zH / config->CellSize + config->NumCellsHeight / 2;
      if (zInd >= 0 && zInd < config->NumCellsHeight / 2) {
         atomic_inc(&(cellCounts[zInd]));
      } 
   }
}


