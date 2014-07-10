
#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

float2 convertReferenceFrame(float2 p, float4 offset) {
   float cosTh = cos(-offset.w);
   float sinTh = sin(-offset.w);
   float2 point;
   /*point.x = (p.x * cosTh - p.y * sinTh) + offset.x;
   point.y = (p.x * sinTh + p.y * cosTh) + offset.y;*/

   /*float2 rotOffset;
   rotOffset.x = cosTh * offset.x - sinTh * offset.y;
   rotOffset.y = sinTh * offset.x + cosTh * offset.y;

   point.x = p.x * cosTh - p.y * sinTh - rotOffset.x;
   point.y = p.x * sinTh + p.y * cosTh - rotOffset.y;*/

   float2 temp;
   temp.x = p.x - offset.x;
   temp.y = p.y - offset.y;
   point.x = temp.x * cosTh - temp.y * sinTh;
   point.y = temp.x * sinTh + temp.y * cosTh;

   return point;
}

/*
 * Converts a point relative to  a local map to its global position
 */
int convertToGlobalPosition(constant slamConfig *config, float x, float y, ocl_float4 localPos) {
   float cosTh = cos(localPos.w);
   float sinTh = sin(localPos.w);
   float gx = (x * cosTh - y * sinTh) + localPos.x;
   float gy = (x * sinTh + y * cosTh) + localPos.y;
   float off = (config->DimGlobalOG * config->CellWidthOG) / 2.0f;
   int i = (gx + off) / config->CellWidthOG;
   int j = (gy + off) / config->CellWidthOG;
   return j * config->DimGlobalOG + i;
}

float2 convertToGlobalCoord(float x, float y, ocl_float4 localPos) {
   float cosTh = cos(localPos.w);
   float sinTh = sin(localPos.w);
   float2 res;
   res.x = (x * cosTh - y * sinTh) + localPos.x;
   res.y = (x * sinTh + y * cosTh) + localPos.y;
   return res;
}


/*
 * converts the x,y location of a point into its occupancy grid
 * coordinate in the local map
 */
int getLocalOGIndex(constant slamConfig *config, float x, float y) {
   int i,j;
   float off = (config->DimLocalOG * config->CellWidthOG) / 2.0f;
   i = (x + off) / config->CellWidthOG;
   j = (y + off) / config->CellWidthOG;
   //check to see if the point fits inside the occupancy grid
   if (i >= 0 && i < config->DimLocalOG && j >= 0 && j < config->DimLocalOG) {
      return j * config->DimLocalOG + i;
   } else {
      return -1;
   }
}

__kernel void initialiseSlam(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global float *freeAreas) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);
   int i,j;
   for (i = index; i < SIZE_LOCAL_OG; i += globalSize) {
      common->localOG[i] = -1;
      common->localOGCount[i] = 0;
      common->localOGZ[i] = 0;
      common->localOGX[i] = 0;
      common->localOGY[i] = 0;
      common->localOGGradX[i] = 0;
      common->localOGGradY[i] = 0;
      freeAreas[i] = INFINITY;
   }
   for (i = index; i < MAX_NUM_CONSTRAINTS + 1; i += globalSize) {
      common->graphHessian[i][0] = 0;
      common->graphHessian[i][1] = 0;
      common->graphHessian[i][2] = 0;
   }
   for(i = index; i < NUM_ORIENTATION_BINS; i += globalSize) {
      localMaps[0].orientationHist[i] = 0;
      localMaps[0].entropyHist[i] = 0;
      for(j = 0; j < NUM_PROJECTION_BINS; j++) {
         localMaps[0].projectionHist[i][j] = 0;
      }
      float theta = (2 * M_PI * i / (float) NUM_ORIENTATION_BINS) - 
                    (M_PI * (2.0f * NUM_ORIENTATION_BINS - 1) / (float) (2 * NUM_ORIENTATION_BINS));
      common->histCos[i] = cos(theta);
      common->histSin[i] = sin(theta);
   }
   if (index < 3) {
      localMaps[0].globalCovar[index][0] = 0;
      localMaps[0].globalCovar[index][1] = 0;
      localMaps[0].globalCovar[index][2] = 0;
      localMaps[0].parentInfo[index][0] = 0;
      localMaps[0].parentInfo[index][1] = 0;
      localMaps[0].parentInfo[index][2] = 0;
      localMaps[0].internalCovar[index][0] = 0;
      localMaps[0].internalCovar[index][1] = 0;
      localMaps[0].internalCovar[index][2] = 0;
      common->A[index][0] = 0;
      common->A[index][1] = 0;
      common->A[index][2] = 0;
      common->B[index] = 0;
      common->goodCount = 0;
      common->numIterations = 0;
      common->previousINode = 0;
      common->scaleFactor[index] = INFINITY;
      common->tempCovar[index][0] = 0;
      common->tempCovar[index][1] = 0;
      common->tempCovar[index][2] = 0;
      //common->scaleFactor[index] = 0;
   }
   if (index == 0) {
      common->currentOffset = (float4) (0.0f, 0.0f, 0.0f, 0.0f);
      common->infoCount = 0;
      common->numConstraints = 0;
      common->numLoopConstraints = 0;
      common->minMapRange.x = INFINITY;
      common->maxMapRange.x = 0;
      common->minMapRange.y = INFINITY;
      common->maxMapRange.y = 0;
      common->combineIndex = -1;
      common->combineMode = 0;
      common->covarCount = 0;
      localMaps[0].numPoints = 0;
      localMaps[0].indexParentNode = -1;
      localMaps[0].currentGlobalPos = (float4) (0.0f, 0.0f, 0.0f, 0.0f);
      localMaps[0].parentOffset = (float4) (0.0f, 0.0f, 0.0f, 0.0f);
      localMaps[0].treeLevel = 0;
      localMaps[0].isFeatureless = 0;
   }
}

__kernel void translateScanPoints(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global oclLaserPoints *points, const int numPoints, 
      const ocl_float4 currentOffset, global float *out) {
   
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpIndex = localIndex % WARP_SIZE;
   
   local float tempInfo[WARP_SIZE][3][3];
   local int infoCount[WARP_SIZE];

   if (index == 0) {
      common->currentOffset = currentOffset;
   }
   if (localIndex < WARP_SIZE) {
      tempInfo[localIndex][0][0] = 0;
      tempInfo[localIndex][0][1] = 0;
      tempInfo[localIndex][0][2] = 0;
      tempInfo[localIndex][1][0] = 0;
      tempInfo[localIndex][1][1] = 0;
      tempInfo[localIndex][1][2] = 0;
      tempInfo[localIndex][2][0] = 0;
      tempInfo[localIndex][2][1] = 0;
      tempInfo[localIndex][2][2] = 0;
      infoCount[localIndex] = 0;
   }
   barrier(CLK_LOCAL_MEM_FENCE);

   float cosTh = cos(currentOffset.w);
   float sinTh = sin(currentOffset.w);
   if (index < numPoints) {
      common->pointsTempX[index] = points->pointX[index] * cosTh - points->pointY[index] * sinTh
         + currentOffset.x;
      common->pointsTempY[index] = points->pointX[index] * sinTh + points->pointY[index] * cosTh
         + currentOffset.y;
      common->pointsTempZ[index] = points->pointZ[index];

      int ogIndex = getLocalOGIndex(config, common->pointsTempX[index], common->pointsTempY[index]);
      if (ogIndex >= 0) {
         float x,y;
         x = -sinTh * common->pointsTempX[index] - cosTh * common->pointsTempY[index];
         y = cosTh * common->pointsTempX[index] - sinTh * common->pointsTempY[index];
         float length = sqrt(common->localOGGradX[ogIndex] * common->localOGGradX[ogIndex] +
               common->localOGGradY[ogIndex] * common->localOGGradY[ogIndex]);
         if (length > 0) {
            float mapGradX = common->localOGGradX[ogIndex] / length;
            float mapGradY = common->localOGGradY[ogIndex] / length;

            //TODO: find a better way to estimate the angular covar of a scan
            float temp = 0.5f;
            atomicFloatAddLocal(&(tempInfo[warpIndex][0][0]), mapGradX * mapGradX);
            atomicFloatAddLocal(&(tempInfo[warpIndex][0][1]), mapGradX * mapGradY);
            atomicFloatAddLocal(&(tempInfo[warpIndex][0][2]), mapGradX * temp);
            atomicFloatAddLocal(&(tempInfo[warpIndex][1][0]), mapGradX * mapGradY);
            atomicFloatAddLocal(&(tempInfo[warpIndex][1][1]), mapGradY * mapGradY);
            atomicFloatAddLocal(&(tempInfo[warpIndex][1][2]), mapGradY * temp);
            atomicFloatAddLocal(&(tempInfo[warpIndex][2][0]), mapGradX * temp);
            atomicFloatAddLocal(&(tempInfo[warpIndex][2][1]), mapGradY * temp);
            atomicFloatAddLocal(&(tempInfo[warpIndex][2][2]), temp * temp);
            atomic_inc(&(infoCount[warpIndex]));
         }
      }
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   parallelReduce3x3(tempInfo, localIndex, common->tempCovar);
   parallelReduceInt(infoCount, localIndex, &(common->covarCount));

}

/*
 * numGlobalPoints is the number of points in the global map not counting this current
 * local map
 */
__kernel void addScanToMap(constant slamConfig *config, global slamLocalMap *localMaps, 
      global slamCommon *common, global int *globalMap, global float *globalMapHeights, 
      global float *freeAreas, const int currentMap, const int numPoints, 
      const int numGlobalPoints, const int firstScan, global float *out) {

   int globalId = get_global_id(0);
   int localId = get_local_id(0);
   int groupId = get_group_id(0);

   local int rayCount;
   local float rayX[LOCAL_SIZE];
   local float rayY[LOCAL_SIZE];

   local float normalX[LOCAL_SIZE];
   local float normalY[LOCAL_SIZE];
   local int orienHist[NUM_ORIENTATION_BINS];

   if (localId > LOCAL_SIZE) {
      return;
   }

   if (localId == 0) {
      rayCount = 0;
   }
   if (localId < NUM_ORIENTATION_BINS) {
      orienHist[localId] = 0;
   }
   normalX[localId] = NAN;
   normalY[localId] = NAN;

   barrier(CLK_LOCAL_MEM_FENCE);
  
   int x = globalId % 3;
   int y = globalId / 3;

   local float a[3][3];
   local float b[3][3];
   if (firstScan == 0) {
      if (globalId < 9) {
         a[x][y] = common->tempCovar[x][y] / (float)common->covarCount;
         
      }
      invert3x3MatrixLocal(a, b, globalId);
      if (globalId < 9) {
         b[x][y] /= config->PerScanInfoScaleFactor;
         localMaps[currentMap].internalCovar[x][y] += b[x][y];
      }
   }
   if (globalId < 9) {
      common->tempCovar[x][y] = 0;
   }
   if (globalId == 0) {

      common->covarCount = 0;
      float sumX = localMaps[currentMap].internalCovar[0][0] + 
         fabs(localMaps[currentMap].internalCovar[0][1])/* + 
         fabs(localMaps[currentMap].internalCovar[0][2])*/;
      float sumY = localMaps[currentMap].internalCovar[1][0] + 
         fabs(localMaps[currentMap].internalCovar[1][1])/* + 
         fabs(localMaps[currentMap].internalCovar[0][2])*/;
      float sumTh = 0 /*localMaps[currentMap].internalCovar[0][0] + 
         fabs(localMaps[currentMap].internalCovar[0][1]) + 
         fabs(localMaps[currentMap].internalCovar[0][2])*/;
      if (sumX > config->LocalMapCovarianceThreshold ||
            sumY > config->LocalMapCovarianceThreshold ||
            sumTh > config->LocalMapCovarianceThreshold) {
         localMaps[currentMap].isFeatureless = 1;
      }
   }

   int ogIndex = -1;
   float curPointX = 0;
   float curPointY = 0;
   float curPointZ = 0;
   if (globalId < numPoints) {
      int pre = globalId - 5;
      if (pre < 1) { pre = 1;}
      float preP = (common->pointsTempX[pre - 1] + common->pointsTempX[pre] + 
            common->pointsTempX[pre + 1]) / 3.0;
      int nxt = globalId + 5;
      if (nxt > numPoints - 2) { nxt = numPoints - 2; }
      float nxtP = (common->pointsTempX[nxt - 1] + common->pointsTempX[nxt] + 
            common->pointsTempX[nxt + 1]) / 3.0;
      float mapGradX = nxtP - preP;
      preP = (common->pointsTempY[pre - 1] + common->pointsTempY[pre] + 
            common->pointsTempY[pre + 1]) / 3.0;
      nxtP = (common->pointsTempY[nxt - 1] + common->pointsTempY[nxt] + 
            common->pointsTempY[nxt + 1]) / 3.0;
      float mapGradY = nxtP - preP;

      ogIndex = getLocalOGIndex(config, common->pointsTempX[globalId], 
            common->pointsTempY[globalId]);

      if (ogIndex >= 0 && mapGradX < config->GradientDistanceThreshold &&
            mapGradY < config->GradientDistanceThreshold) {
         curPointX = common->pointsTempX[globalId];
         curPointY = common->pointsTempY[globalId];
         curPointZ = common->pointsTempZ[globalId];

         //Update the histograms
         float orien = atan2(mapGradY, mapGradX);
         orien += M_PI/2;
         if (orien >= M_PI) {
            orien -= 2 * M_PI;
         }
         int orienIndex = (orien + M_PI) * NUM_ORIENTATION_BINS / (2 * M_PI);
         //atomicFloatAdd(&(localMaps[currentMap].orientationHist[orienIndex]), 1);
         atomic_inc(&(orienHist[orienIndex]));

         normalX[localId] = -mapGradY;
         normalY[localId] = mapGradX;
         
         /*int i;
         float mapSize = config->DimLocalOG * config->CellWidthOG;
         for(i = 0; i < NUM_ORIENTATION_BINS; i++) {
            float dist = curPointX * common->histCos[i] + 
                     curPointY * common->histSin[i];
            int projIndex = (dist + mapSize / 2.0f) / 
                     (mapSize / (float) NUM_PROJECTION_BINS);
            if (projIndex < 0 || projIndex >= NUM_PROJECTION_BINS) {
               continue;
            }
            float normalX = -mapGradY;
            float normalY = mapGradX;
            float normalise = sqrt(normalX * normalX + normalY * normalY);
            float weight = normalX / normalise * common->histCos[i] +
                        normalY / normalise * common->histSin[i];
            atomicFloatAdd(&(localMaps[currentMap].projectionHist[i][projIndex]), weight);
         }*/
      }
   }


   barrier(CLK_LOCAL_MEM_FENCE);

   if (!isnan(normalX[localId]) && !isnan(normalY[localId])) {

      int ogIndexPrev = - 1;
      if (globalId > 0) {
         ogIndexPrev = getLocalOGIndex(config, common->pointsTempX[globalId - 1], 
            common->pointsTempY[globalId - 1]);
      }
      if ((ogIndex != ogIndexPrev || localId == 0)) {
         float pX = curPointX;
         float pY = curPointY;
         float pZ = curPointZ;
         float gradX = -normalX[localId];
         float gradY = normalY[localId];
         int count = 1;

         int pIndex = globalId + 1;
         int lIndex = localId + 1;
         while (pIndex < numPoints && lIndex < LOCAL_SIZE) {
            float tempX = common->pointsTempX[pIndex];
            float tempY = common->pointsTempY[pIndex];
            float tempZ = common->pointsTempZ[pIndex];
            int ogNext = getLocalOGIndex(config, tempX, tempY);
            if (ogNext != ogIndex) {
               break;
            }
            if (!isnan(normalX[lIndex]) && !isnan(normalY[lIndex])) {
               pX += tempX;
               pY += tempY;
               pZ = fmax(pZ, tempZ);
               gradX += -normalX[lIndex];
               gradY += normalY[lIndex];
               count++;
            }

            pIndex++;
            lIndex++;

         }

         atomicFloatMax(&(common->localOGZ[ogIndex]), pZ);
         atomicFloatAdd(&(common->localOGX[ogIndex]), pX);
         atomicFloatAdd(&(common->localOGY[ogIndex]), pY);
         atomicFloatAdd(&(common->localOGGradX[ogIndex]), gradX);
         atomicFloatAdd(&(common->localOGGradY[ogIndex]), gradY);
         int obsCount = atomic_add(&(common->localOGCount[ogIndex]), count);
         if (obsCount <= config->MinObservationCount && obsCount + count > 
               config->MinObservationCount) {
         
         //atomicFloatMax(&(common->localOGZ[ogIndex]), curPointZ);
         //atomicFloatAdd(&(common->localOGX[ogIndex]), curPointX);
         //atomicFloatAdd(&(common->localOGY[ogIndex]), curPointY);
         //These are swapped deliberately
         //atomicFloatAdd(&(common->localOGGradX[ogIndex]), mapGradY);
         //atomicFloatAdd(&(common->localOGGradY[ogIndex]), mapGradX);
         //int obsCount = atomic_inc(&(common->localOGCount[ogIndex]));
         //if (obsCount == config->MinObservationCount) {
            int index = atomic_inc(&(localMaps[currentMap].numPoints));
            if (index < MAX_LOCAL_POINTS) {
               common->activeCells[index] = ogIndex;
               common->localOG[ogIndex] = index;
               if (curPointX < common->minMapRange.x) {
                  common->minMapRange.x = curPointX;
               }
               if (curPointX > common->maxMapRange.x) {
                  common->maxMapRange.x = curPointX;
               }
               if (curPointY < common->minMapRange.y) {
                  common->minMapRange.y = curPointY;
               }
               if (curPointY > common->maxMapRange.y) {
                  common->maxMapRange.y = curPointY;
               }
               //addToFreeArea(curPointX, curPointY);
               int rayIndex = atomic_inc(&rayCount);
               rayX[rayIndex] = curPointX;
               rayY[rayIndex] = curPointY;

               //Add the point to the global map
               int globalIndex = convertToGlobalPosition(config, curPointX, curPointY, 
                      localMaps[currentMap].currentGlobalPos);
               globalMap[numGlobalPoints + index] = globalIndex;
               globalMapHeights[numGlobalPoints + index] = common->localOGZ[ogIndex];
            } else {
               atomic_dec(&(localMaps[currentMap].numPoints));
            }
         }
      }
      //}
   }

   //barrier(CLK_LOCAL_MEM_FENCE);

   if (localId < NUM_ORIENTATION_BINS && orienHist[localId] > 0) {
      atomicFloatAdd(&(localMaps[currentMap].orientationHist[localId]), orienHist[localId]);
   }

   //Add information to the projection histograms
   float mapSize = config->DimLocalOG * config->CellWidthOG;
   int globalOff = groupId * LOCAL_SIZE;
   float projSum[NUM_PROJECTION_BINS];
   /*if (localId < NUM_ORIENTATION_BINS) {
      int i;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         projSum[i] = 0;
      }
      for (i = 0; i < LOCAL_SIZE; i++) {
         int pointI = globalOff + i;
         if (pointI < numPoints && !isnan(normalX[i]) && !isnan(normalY[i])) {
            
            float dist = common->pointsTempX[pointI] * common->histCos[localId] + 
                     common->pointsTempY[pointI] * common->histSin[localId];
            int projIndex = (dist + mapSize / 2.0f) / 
                     (mapSize / (float) NUM_PROJECTION_BINS);
            if (projIndex < 0 || projIndex >= NUM_PROJECTION_BINS) {
               continue;
            }
            float normalise = sqrt(normalX[i] * normalX[i] + normalY[i] * normalY[i]);
            float weight = normalX[i] / normalise * common->histCos[localId] +
                        normalY[i] / normalise * common->histSin[localId];
            projSum[projIndex] += weight;
         }
      }
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         if (projSum[i] != 0) {
            atomicFloatAdd(&(localMaps[currentMap].projectionHist[localId][i]), projSum[i]);
         }
      }
   }*/
   int splitNum = localId / NUM_ORIENTATION_BINS;
   int splitIndex = localId % NUM_ORIENTATION_BINS;
   int numSplits = LOCAL_SIZE / NUM_ORIENTATION_BINS;
   int splitSize = ceil(LOCAL_SIZE / (float) numSplits);
   if (splitNum < numSplits) {
      int i;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         projSum[i] = 0;
      }
      int endPoint = splitSize * (splitNum + 1);
      if (endPoint > LOCAL_SIZE) {
         endPoint = LOCAL_SIZE;
      }
      for (i = splitSize * splitNum; i < endPoint; i++) {
         int pointI = globalOff + i;
         if (pointI >= numPoints) {
            break;
         }
         if (!isnan(normalX[i]) && !isnan(normalY[i])) {
            
            float dist = common->pointsTempX[pointI] * common->histCos[splitIndex] + 
                     common->pointsTempY[pointI] * common->histSin[splitIndex];
            int projIndex = (dist + mapSize / 2.0f) / 
                     (mapSize / (float) NUM_PROJECTION_BINS);
            if (projIndex < 0 || projIndex >= NUM_PROJECTION_BINS) {
               continue;
            }
            float normalise = sqrt(normalX[i] * normalX[i] + normalY[i] * normalY[i]);
            float weight = normalX[i] / normalise * common->histCos[splitIndex] +
                        normalY[i] / normalise * common->histSin[splitIndex];
            projSum[projIndex] += weight;
         }
      }
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         if (projSum[i] != 0) {
            atomicFloatAdd(&(localMaps[currentMap].projectionHist[splitIndex][i]), projSum[i]);
         }
      }
   }

   //add the newly added points to the free area map   
   if (localId < rayCount) {
      int ogIndex = getLocalOGIndex(config, rayX[localId], rayY[localId]);
      float origX = rayX[localId];
      float origY = rayY[localId];
      float px = origX / config->CellWidthOG;
      float py = origY / config->CellWidthOG;

      float currentOffsetX = common->currentOffset.x / config->CellWidthOG;
      float currentOffsetY = common->currentOffset.y / config->CellWidthOG;

      float dx = fabs(common->currentOffset.x - px);
      float dy = fabs(common->currentOffset.y - py);
      int x = (int)(floor(px));
      int y = (int)(floor(py));
      int n = 1;
      int xInc, yInc;
      float error;
      if (dx == 0) {
         xInc = 0;
         error = INFINITY;
      } else if (currentOffsetX > px) {
         xInc = 1;
         n += (int)(floor(currentOffsetX)) - x;
         error = (floor(px) + 1 - px) * dy;
      } else {
         xInc = -1;
         n += x - (int)(floor(currentOffsetX));
         error = (px - floor(px)) * dy;
      }
      if (dy == 0) {
         yInc = 0;
         error -= INFINITY;
      } else if (currentOffsetY > py) {
         yInc = 1;
         n += (int)(floor(currentOffsetY)) - y;
         error -= (floor(py) + 1 - py) * dx;
      } else {
         yInc = -1;
         n += y - (int)(floor(currentOffsetY));
         error -= (py - floor(py)) * dx;
      }

      int startIndex = ogIndex;
      x = ogIndex % config->DimLocalOG;
      y = ogIndex / config->DimLocalOG;
      float off = config->CellWidthOG / 2.0f - (config->DimLocalOG * config->CellWidthOG) / 2.0f;

      float denom = log(1.0f + config->FreeAreaDistanceThreshold);
      int freeAreasOffset = currentMap * config->DimLocalOG * config->DimLocalOG;
      for (; n > 0; --n) {
         float xCent = ((float) x) * config->CellWidthOG + off;
         float yCent = ((float) y) * config->CellWidthOG + off;
         float dist = sqrt((origX - xCent) * (origX - xCent) + (origY - yCent) * (origY - yCent));

         dist = fmin(dist, config->FreeAreaDistanceThreshold);
         float score = log (dist + 1.0) / denom;

         //update the free map area bit
         atomicFloatMin(&(freeAreas[freeAreasOffset + ogIndex]), score);

         if (error > 0) {
            ogIndex += yInc * config->DimLocalOG;
            y += yInc;
            error -= dx;
         } else {
            ogIndex += xInc;
            x += xInc;
            error += dy;
         }
      }
      atomic_xchg(&(freeAreas[freeAreasOffset + startIndex]), 0);
   }   
}

__kernel void setLocalMap(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global float *globalMapHeights, const int currentMap,
      const int numGlobalPoints) {
   int index = get_global_id(0);
   if (index == 0) {
      localMaps[currentMap].numWarpPoints = localMaps[currentMap].numPoints;
   }
   if (index < localMaps[currentMap].numPoints) {
      int ogIndex = common->activeCells[index];
      float count = (float)common->localOGCount[ogIndex];
      localMaps[currentMap].pointsX[index] = common->localOGX[ogIndex] / count;
      localMaps[currentMap].pointsY[index] = common->localOGY[ogIndex] / count;
      localMaps[currentMap].pointsZ[index] = common->localOGZ[ogIndex];
      globalMapHeights[numGlobalPoints + index] = common->localOGZ[ogIndex];
      localMaps[currentMap].warpPointsX[index] = localMaps[currentMap].pointsX[index];
      localMaps[currentMap].warpPointsY[index] = localMaps[currentMap].pointsY[index];
      localMaps[currentMap].warpPointsZ[index] = localMaps[currentMap].pointsZ[index];
      localMaps[currentMap].gradX[index] = common->localOGGradX[ogIndex];
      localMaps[currentMap].gradY[index] = common->localOGGradY[ogIndex];
   }
}

/*
 * Gets the mbicp metric value for the match between point (a laser point)
 * and ogPoint (laser point in the occupancy grid)
 */
//point is p1, ogPoint is p2
float getMetricValue(constant slamConfig *config, float2 point, float2 ogPoint) {
   float dx = ogPoint.x - point.x;
   float dy = ogPoint.y - point.y;
   if (dx * dx + dy * dy > config->MaxSlamAlignDist) {
      return INFINITY;
   }
   return dx * dx + dy * dy - 
          (pown(dx * point.y - dy * point.x, 2)) / 
          (point.x * point.x + point.y * point.y + config->SlamL * config->SlamL);
}

/*
 * Finds the closest matching point in the current local map's occupancy grid to a 
 * point in a previous local map.
 * offset is the vector to convert points in the previous local map to the reference
 * frame of the current local map
 *
 * Returns the index of the matching point in the current local map
 */ 
int findMatchingPoint(constant slamConfig *config, global slamLocalMap *localMaps, 
       global slamCommon *common, float2 point, int currentMapIndex,
       float4 offset, int searchFactor) {

   //Get the cell in the occupancy grid the point lies on
   int ogIndex = getLocalOGIndex(config, point.x, point.y);
   if (ogIndex >= 0) {
      int ogX = ogIndex % config->DimLocalOG;
      int ogY = ogIndex / config->DimLocalOG;
      int i,j;
      float minWeight = INFINITY;
      int minOGIndex = -1;
      int searchSize = config->SearchSize * searchFactor;
      for (i = ogX - searchSize; i <= ogX + searchSize; i++) {
         for (j = ogY - searchSize; j <= ogY + searchSize; j++) {
            if (i >= 0 && i < config->DimLocalOG && j >= 0 && j < config->DimLocalOG) {
               ogIndex = j * config->DimLocalOG + i;
               if (common->localOG[ogIndex] >= 0) {
                  float2 ogPoint;
                  ogPoint.x = localMaps[currentMapIndex].pointsX[common->localOG[ogIndex]];
                  ogPoint.y = localMaps[currentMapIndex].pointsY[common->localOG[ogIndex]];
                  float min = getMetricValue(config, point, ogPoint);
                  if (min < minWeight) {
                     minWeight = min;
                     minOGIndex = common->localOG[ogIndex];
                  }
               }
            }
         }
      }
      return minOGIndex;
   } else {
      //The point does not fit in the current local map, so can't be used for alignment
      return -1;
   }
}

//TODO: make this work for estimating the information matrices for the loop closing
//as well. When do this, see if convert referenceframe should be called again
//when aligning the parent as it is now
__kernel void getHessianMatch(constant slamConfig *config, global slamLocalMap *localMaps, 
      global slamCommon *common, const int currentMap, const int constraintIndex) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpIndex = localIndex % WARP_SIZE;

   local float info[WARP_SIZE][3][3];
   local int goodCount[WARP_SIZE];

   if (localIndex < WARP_SIZE) {
      info[localIndex][0][0] = 0.0f;
      info[localIndex][0][1] = 0.0f;
      info[localIndex][0][2] = 0.0f;
      info[localIndex][1][0] = 0.0f;
      info[localIndex][1][1] = 0.0f;
      info[localIndex][1][2] = 0.0f;
      info[localIndex][2][0] = 0.0f;
      info[localIndex][2][1] = 0.0f;
      info[localIndex][2][2] = 0.0f;
      goodCount[localIndex] = 0;
   }
   barrier(CLK_LOCAL_MEM_FENCE);

   int otherMap = -1;
   float4 offset;
   if (constraintIndex < 0) {
      offset = localMaps[currentMap].parentOffset;
      otherMap = localMaps[currentMap].indexParentNode;
   } else {
      int cIndex = common->constraintIndex[constraintIndex];
      offset.x = common->loopConstraintXDisp[cIndex];
      offset.y = common->loopConstraintYDisp[cIndex];
      offset.z = 0;
      offset.w = common->loopConstraintThetaDisp[cIndex];
      otherMap = common->loopConstraintI[cIndex];
   }

   if (index < localMaps[otherMap].numPoints) {
      //If it is the parent node, have to find the matching points
      int matchIndex = -1;
      float2 transformedPoint;
      //Transform the point from a previous local map into the reference frame of the
      //current local map so that it can be matched to the occupancy grid
      transformedPoint = convertReferenceFrame((float2) (localMaps[otherMap].pointsX[index], 
                  localMaps[otherMap].pointsY[index]), offset);
      //matchIndex = findMatchingPoint(config, localMaps, common, transformedPoint, 
      //      currentMap, offset, 1);
      matchIndex = getLocalOGIndex(config, transformedPoint.x, transformedPoint.y);
      
      if (matchIndex >= 0 && common->localOG[matchIndex] >= 0) {
         float cosTh = cos(offset.w);
         float sinTh = sin(offset.w);

         float2 mapGrad;
         float x,y;
         mapGrad.x = common->localOGGradX[matchIndex];
         mapGrad.y = common->localOGGradY[matchIndex];
         float length = sqrt(mapGrad.x * mapGrad.x + mapGrad.y * mapGrad.y);
         mapGrad.x /= length;
         mapGrad.y /= length;
         if (length > 0) {
            x = -sinTh * transformedPoint.x - cosTh * transformedPoint.y;
            y = cosTh * transformedPoint.x - sinTh * transformedPoint.y;

            //normalise the movement theta differential???
            /*length = sqrt(transformedPoint.x * transformedPoint.x + transformedPoint.y *
                  transformedPoint.y);
            x /= length;
            y /= length;*/

            float temp = x * mapGrad.x + y * mapGrad.y;
            atomicFloatAddLocal(&(info[warpIndex][0][0]), mapGrad.x * mapGrad.x);
            atomicFloatAddLocal(&(info[warpIndex][0][1]), mapGrad.x * mapGrad.y);
            atomicFloatAddLocal(&(info[warpIndex][0][2]), mapGrad.x * temp);
            atomicFloatAddLocal(&(info[warpIndex][1][0]), mapGrad.x * mapGrad.y);
            atomicFloatAddLocal(&(info[warpIndex][1][1]), mapGrad.y * mapGrad.y);
            atomicFloatAddLocal(&(info[warpIndex][1][2]), mapGrad.y * temp);
            atomicFloatAddLocal(&(info[warpIndex][2][0]), mapGrad.x * temp);
            atomicFloatAddLocal(&(info[warpIndex][2][1]), mapGrad.y * temp);
            atomicFloatAddLocal(&(info[warpIndex][2][2]), temp * temp);
            atomic_inc(&(goodCount[warpIndex]));
         }
      }
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (constraintIndex < 0) {
      parallelReduce3x3(info, localIndex, localMaps[currentMap].parentInfo);
   } else {
      int cIndex = common->constraintIndex[constraintIndex];
      parallelReduce3x3(info, localIndex, common->loopConstraintInfo[cIndex]);
   }
   parallelReduceInt(goodCount, localIndex, &(common->infoCount));
}

/*
 * Nasty hack function to make covariance matrices acceptable in the occassional 
 * case where not enough matching points were found when creating the 
 * information matrix
 */
void covarFiddle(global float m[3][3], int index, float maxCovar, local float *max) {
   int y = index / 3;
   int x = index % 3;
   if (index < 3) {
      if (m[index][index] < 0) {
         m[index][index] *= -1;
      }
   }
   //local float max;
   if(index == 0) {
      *max = maxCovar;
      if (m[0][0] > *max) {
         *max = m[0][0];
      }
      if (m[1][1] > *max) {
         *max = m[1][1];
      }
      if (m[2][2] > *max) {
         *max = m[2][2];
      }
   }
   if (index < 9) {
      if(*max > maxCovar) {
         m[y][x] = m[y][x] * maxCovar/ *max;
      }
   }
}
      
/*
 * Performs the caluclations needed to setup the finalising of a local map
 *
 * This kernel should be called with the local size at least as large as
 * NUM_ORIENTATION_BINS
 *
 */
__kernel void prepareLocalMap(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int currentMap, const int numMaps, const int numScans, 
      global float *out) {
   int index = get_global_id(0);

   if (index == 0) {
      localMaps[currentMap].robotMapCentre.x = common->currentOffset.x / 2.0f;
      localMaps[currentMap].robotMapCentre.y = common->currentOffset.y / 2.0f;
      localMaps[currentMap].globalRobotMapCentre = convertToGlobalCoord(
            localMaps[currentMap].robotMapCentre.x, localMaps[currentMap].robotMapCentre.y,
            localMaps[currentMap].currentGlobalPos);
   }

   if (index < 9) {
      int x = index / 3;
      int y = index % 3;
      //out[index] = localMaps[currentMap].internalCovar[x][y];
   }

   //Finalise the histograms
   int i;
   if (index < NUM_ORIENTATION_BINS) {
      float sum = 0;
      float entropySum = 0;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         sum += localMaps[currentMap].projectionHist[index][i] * 
                localMaps[currentMap].projectionHist[index][i];
      }
      sum = sqrt(sum);
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         localMaps[currentMap].projectionHist[index][i] /= sum;
         entropySum += fabs(localMaps[currentMap].projectionHist[index][i]);
      }
      sum = 0;
      float temp;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         temp = fabs(localMaps[currentMap].projectionHist[index][i]) / entropySum;
         if (temp > 0) {
            sum += temp * log(temp);
         }
      }
      sum *= -1;
      sum = pow(2, sum);
      localMaps[currentMap].entropyHist[index] = sum;
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   local float max;
   local float finalSum;
   local float orienSum;
   if (index == 0) {
      max = 0;
      finalSum = 0;
      orienSum = 0;
      for (i = 0; i < NUM_ORIENTATION_BINS; i++) {
         if (localMaps[currentMap].entropyHist[i] > max) {
            max = localMaps[currentMap].entropyHist[i];
         }
      }
      for(i = 0; i < NUM_ORIENTATION_BINS; i++) {
         finalSum += (localMaps[currentMap].entropyHist[i] - max) * 
                     (localMaps[currentMap].entropyHist[i] - max);
         orienSum += localMaps[currentMap].orientationHist[i] *
                     localMaps[currentMap].orientationHist[i];
      }
      finalSum = sqrt(finalSum);
      orienSum = sqrt(orienSum);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (index < NUM_ORIENTATION_BINS) {
      localMaps[currentMap].entropyHist[index] = 
               -(localMaps[currentMap].entropyHist[index] - max) / finalSum;
      localMaps[currentMap].orientationHist[index] /= orienSum;
   }

   local float a[3][3];
   local float b[3][3];
   local float c[3][3];
   //if (currentMap > 0) {
      //Need to rotate the hessian matrix as it was calculated in the opposite displacement
      //to the actual move - rotate info matrix b by RbR^T where R is the homogeneous 
      //rotation matrix of the angle of the last move

   int x = index % 3;
   int y = index / 3;

   if (index == 0) {
      localMaps[currentMap].mapCentre.x = (common->minMapRange.x + common->maxMapRange.x) / 2;
      localMaps[currentMap].mapCentre.y = (common->minMapRange.y + common->maxMapRange.y) / 2;
   }
   if (index < 9) {
      a[y][x] = localMaps[currentMap].internalCovar[y][x] / (float) numScans;
      a[y][x] = a[y][x] * a[y][x] * config->InformationScaleFactor;
      invert3x3MatrixLocal(a, b, index);
      localMaps[currentMap].parentInfo[y][x] = b[y][x];

      b[y][x] = localMaps[currentMap].internalCovar[y][x];
   }
   if (index == 0) {
      //float cosTh = cos(- localMaps[currentMap].parentOffset.w);
      //float sinTh = sin(- localMaps[currentMap].parentOffset.w);
      float cosTh = cos(localMaps[currentMap].currentGlobalPos.w);
      float sinTh = sin(localMaps[currentMap].currentGlobalPos.w);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;
   }
   mult3x3MatrixLocal(a, b, c, index);
   if (index == 0) {
      float temp = a[1][0];
      a[1][0] = a[0][1];
      a[0][1] = temp;
   }
   mult3x3MatrixLocal(c, a, b, index);
   if (index < 9) {
      localMaps[currentMap].globalCovar[y][x] = b[y][x];
   }

      /*int x = index % 3;
      int y = index / 3;
      if (index == 0) {
         localMaps[currentMap].mapCentre.x = (common->minMapRange.x + common->maxMapRange.x) / 2;
         localMaps[currentMap].mapCentre.y = (common->minMapRange.y + common->maxMapRange.y) / 2;

         float cosTh = cos(- localMaps[currentMap].parentOffset.w);
         float sinTh = sin(- localMaps[currentMap].parentOffset.w);
         a[0][0] = cosTh;
         a[1][1] = cosTh;
         a[1][0] = sinTh;
         a[0][1] = -sinTh;
         a[2][2] = 1;
         a[0][2] = 0;
         a[1][2] = 0;
         a[2][0] = 0;
         a[2][1] = 0;

         if (common->infoCount < config->InformationScaleFactor) {
            common->infoCount = config->InformationScaleFactor;
         }
      }
      local float max;
      if (index < 9) {
  
         //Adjust the parent info matrix to be relative to the parent instead
         //of relative to the current node
         b[y][x] = localMaps[currentMap].parentInfo[y][x] / 
                           ((float)common->infoCount / (float)config->InformationScaleFactor);
         mult3x3MatrixLocal(a, b, c, index);
         if (index == 0) {
            a[1][0] *= -1;
            a[0][1] *= -1;
         }
         mult3x3MatrixLocal(c, a, b, index);
         localMaps[currentMap].parentInfo[y][x] = b[y][x];
      
         //Now calculate the global covar matrix for the node in its
         //current position - b currently holds the parent info matrix
         if (index == 0) {
            float cosTh = cos(localMaps[localMaps[currentMap].indexParentNode].currentGlobalPos.w);
            float sinTh = sin(localMaps[localMaps[currentMap].indexParentNode].currentGlobalPos.w);
            a[0][0] = cosTh;
            a[1][1] = cosTh;
            a[1][0] = sinTh;
            a[0][1] = -sinTh;
            a[2][2] = 1;
            a[0][2] = 0;
            a[1][2] = 0;
            a[2][0] = 0;
            a[2][1] = 0;
         }
         mult3x3MatrixLocal(a, b, c, index);
         //transpose the parent rotation matrix
         if (index == 0) {
            float temp = a[1][0];
            a[1][0] = a[0][1];
            a[0][1] = temp;
         }
         mult3x3MatrixLocal(c, a, b, index);
         //now invert it to get the covar matrix
         localMaps[currentMap].globalCovar[y][x] = b[y][x];
         invert3x3Matrix(localMaps[currentMap].globalCovar, a, index);
         localMaps[currentMap].globalCovar[y][x] = a[y][x];
         //Fiddle with the covar to make it nicer
         covarFiddle(localMaps[currentMap].globalCovar, index, config->MaxCovar, &max);
      }

      //if (index < 3) {
      //   localMaps[currentMap].parentInfo[index][0] /= common->infoCount;
      //   localMaps[currentMap].parentInfo[index][1] /= common->infoCount;
      //   localMaps[currentMap].parentInfo[index][2] /= common->infoCount;
      //}

      if (index < 3) {
         out[index] = localMaps[currentMap].globalCovar[0][index];
         out[index + 3] = localMaps[currentMap].globalCovar[1][index];
         out[index + 6] = localMaps[currentMap].globalCovar[2][index];
         out[9 + index] = localMaps[currentMap].parentInfo[0][index];
         out[9 + index + 3] = localMaps[currentMap].parentInfo[1][index];
         out[9 + index + 6] = localMaps[currentMap].parentInfo[2][index];
         if (index == 0) {
            float cosTh = cos(localMaps[localMaps[currentMap].indexParentNode].currentGlobalPos.w);
            float sinTh = sin(localMaps[localMaps[currentMap].indexParentNode].currentGlobalPos.w);
            out[18] = cosTh;
            out[19] = -sinTh;
            out[20] = 0;
            out[21] = sinTh;
            out[22] = cosTh;
            out[23] = 0;
            out[24] = 0;
            out[25] = 0;
            out[26] = 1;
         }
         out[28] = common->infoCount;
         //out[29] = det;
      }*/
   if (currentMap > 0) {
      if (index == 0) {
         common->infoCount = 0;

         //Start combining nodes
         int combined = 0;
         if (common->combineIndex >= 0) {
            /*float2 mapCurPos = convertToGlobalCoord(localMaps[currentMap].mapCentre.x, 
                                 localMaps[currentMap].mapCentre.y, 
                                 localMaps[currentMap].currentGlobalPos);*/
            float2 mapCurPos = convertToGlobalCoord(localMaps[currentMap].robotMapCentre.x, 
                                 localMaps[currentMap].robotMapCentre.y, 
                                 localMaps[currentMap].currentGlobalPos);
            for(i = -1; i <= numMaps && !combined; i++) {
               int pIndex = -1;
               if (i == -1) {
                  pIndex = localMaps[common->combineIndex].indexParentNode;
               } else if (i < numMaps && localMaps[i].indexParentNode == common->combineIndex) {
                  pIndex = i;
               } else if (i == numMaps) {
                  pIndex = common->combineIndex;
               }
               if (pIndex >= 0) {
                  float2 mapOtherPos = convertToGlobalCoord(localMaps[pIndex].robotMapCentre.x, 
                        localMaps[pIndex].robotMapCentre.y, 
                        localMaps[pIndex].currentGlobalPos);
                  if (fabs(mapOtherPos.x - mapCurPos.x) < config->LocalMapDist/* * 1.5f */&&
                        fabs(mapOtherPos.y - mapCurPos.y) < config->LocalMapDist/* * 1.5f*/) {
                     combined = 1;
                     common->combineIndex = pIndex;
                     float4 temp = localMaps[currentMap].currentGlobalPos - localMaps[pIndex].currentGlobalPos;
                     float cosTh = cos(localMaps[currentMap].currentGlobalPos.w);
                     float sinTh = cos(localMaps[currentMap].currentGlobalPos.w);
                     common->potentialMatchX[0] = temp.x * cosTh - temp.y * sinTh;
                     common->potentialMatchY[0] = temp.x * sinTh + temp.y * cosTh;
                     common->potentialMatchTheta[0] = temp.w;
                     ANGNORM(common->potentialMatchTheta[0]);
                  }
               }
            }
            if (!combined) {
               common->combineIndex = -1;
               common->combineMode = 0;
            }
         }
         if (combined == 0) {
            int cIndex = common->numConstraints;
            common->numConstraints++;
            common->constraintType[cIndex] = 0;
            common->constraintIndex[cIndex] = currentMap;
         }
      }
   }
}

int findIfPeak(local float *corr, int offset, int i) {
   int isPeak = 1;
   int j;
   if (corr[i + offset] > 0.8f) {
      for(j = -2; j < 3; j++) {
         int adjust = (i + j) % NUM_ORIENTATION_BINS;
         if (adjust < 0) {
            adjust = NUM_ORIENTATION_BINS + adjust;
         }
         if (corr[offset + adjust] > corr[offset + i]) {
            isPeak = 0;
         }
      }
   } else {
      isPeak = 0;
   }
   return isPeak;
}

float correlateProjection(global float proj1[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS],
                          global float proj2[NUM_ORIENTATION_BINS][NUM_PROJECTION_BINS],
                          int startIndex, int offset, int warpIndex, int *maxIndex,
                          local float tempCorr[LOCAL_SIZE], local int tempIndex[LOCAL_SIZE], 
                          int warpStart) {
   int sec = (startIndex + offset) % NUM_ORIENTATION_BINS;
   tempCorr[warpStart + warpIndex] = -INFINITY;
   tempIndex[warpStart + warpIndex] = -1;
   int i, j;
   int off;
   int index;
   float out;
   for (i = warpIndex; i < NUM_PROJECTION_BINS * 2; i += WARP_SIZE) {
      off = i - NUM_PROJECTION_BINS;
      out = 0;
      for (j = 0; j < NUM_PROJECTION_BINS; j++) {
         index = off + j;
         if (index >= 0 && index < NUM_PROJECTION_BINS) {
            out += proj1[startIndex][j] * proj2[sec][index];
         }
      }
      if (out > tempCorr[warpStart + warpIndex]) {
         tempCorr[warpStart + warpIndex] = out;
         tempIndex[warpStart + warpIndex] = i;
      }
   }
   if (warpIndex < 16 && tempCorr[warpStart + warpIndex + 16] > tempCorr[warpStart + warpIndex]) {
      tempCorr[warpStart + warpIndex] = tempCorr[warpStart + warpIndex + 16];
      tempIndex[warpStart + warpIndex] = tempIndex[warpStart + warpIndex + 16];
   }
   if (warpIndex < 8 && tempCorr[warpStart + warpIndex + 8] > tempCorr[warpStart + warpIndex]) {
      tempCorr[warpStart + warpIndex] = tempCorr[warpStart + warpIndex + 8];
      tempIndex[warpStart + warpIndex] = tempIndex[warpStart + warpIndex + 8];
   }
   if (warpIndex < 4 && tempCorr[warpStart + warpIndex + 4] > tempCorr[warpStart + warpIndex]) {
      tempCorr[warpStart + warpIndex] = tempCorr[warpStart + warpIndex + 4];
      tempIndex[warpStart + warpIndex] = tempIndex[warpStart + warpIndex + 4];
   }
   if (warpIndex < 2 && tempCorr[warpStart + warpIndex + 2] > tempCorr[warpStart + warpIndex]) {
      tempCorr[warpStart + warpIndex] = tempCorr[warpStart + warpIndex + 2];
      tempIndex[warpStart + warpIndex] = tempIndex[warpStart + warpIndex + 2];
   }
   if (warpIndex < 1 && tempCorr[warpStart + warpIndex + 1] > tempCorr[warpStart + warpIndex]) {
      tempCorr[warpStart + warpIndex] = tempCorr[warpStart + warpIndex + 1];
      tempIndex[warpStart + warpIndex] = tempIndex[warpStart + warpIndex + 1];      
   }
   *maxIndex = tempIndex[warpStart];
   return tempCorr[warpStart];
      
}


__kernel void findPotentialMatches(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int currentLocalMap, const int numLocalMaps, 
      global float *out) {

   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpNum = localIndex / WARP_SIZE;
   int warpIndex = localIndex % WARP_SIZE;
   int groupSize = get_local_size(0);
   int groupNum = get_group_id(0);
   int globalWarp = (groupNum * groupSize + localIndex)/ WARP_SIZE;

   local int tempCorrIndex[LOCAL_SIZE];
   local float totalCovar[LOCAL_SIZE / WARP_SIZE][3][3];

   //historgram local variables
   local int peaks[LOCAL_SIZE];
   local float tempCorr[LOCAL_SIZE];
   local float orienCorr[NUM_ORIENTATION_BINS * LOCAL_SIZE / WARP_SIZE];
   local float entCorr[NUM_ORIENTATION_BINS * LOCAL_SIZE / WARP_SIZE];
   local int numPeaks[LOCAL_SIZE / WARP_SIZE];
   local int maxPeak[LOCAL_SIZE / WARP_SIZE];
   if (warpIndex == 0) {
      numPeaks[warpNum] = 0;
   }
   int parentIndex = -1;

   if (localMaps[currentLocalMap].isFeatureless > 0) {
      return;
   }

   //globalWarp is the index of the localMap that are examining
   //warpIndex is the index in the warp
   if (globalWarp < numLocalMaps && globalWarp != currentLocalMap &&
         globalWarp != localMaps[currentLocalMap].indexParentNode &&
         localMaps[globalWarp].isFeatureless == 0) {
      //Calculate the total covar from currentlocalmap to the local map given by
      //globalWarp. Adds covar contributions of two localmaps at once
      int y = warpIndex / 3;
      int x = warpIndex % 3;
      if (warpIndex < 9) {
         totalCovar[warpNum][y][x] = 0;
      
         int minLevel = min(localMaps[currentLocalMap].treeLevel, localMaps[globalWarp].treeLevel);
         int mapIndex = currentLocalMap;

         while (localMaps[mapIndex].treeLevel > minLevel) {
            totalCovar[warpNum][y][x] += localMaps[mapIndex].globalCovar[y][x];
            mapIndex = localMaps[mapIndex].indexParentNode;
            /*if (mapIndex == localMaps[mapIndex].indexParentNode) {
               break;
            }*/
         }
         int mapIndexGlobal = globalWarp;
         while (localMaps[mapIndexGlobal].treeLevel > minLevel) {
            totalCovar[warpNum][y][x] += localMaps[mapIndexGlobal].globalCovar[y][x];
            mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
            /*if (mapIndexGlobal == localMaps[mapIndexGlobal].indexParentNode) {
               break;
            }*/
         }
         while (mapIndex != mapIndexGlobal) {
            totalCovar[warpNum][y][x] += localMaps[mapIndex].globalCovar[y][x];
            totalCovar[warpNum][y][x] += localMaps[mapIndexGlobal].globalCovar[y][x];
            mapIndex = localMaps[mapIndex].indexParentNode;
            mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
         }
         parentIndex = mapIndex;
      }
      //Now have calculated the total covar. Need to now see if a potential match, and 
      //if so, perform a histogram correlation
      float2 mapCurPos = convertToGlobalCoord(localMaps[currentLocalMap].robotMapCentre.x, 
                        localMaps[currentLocalMap].robotMapCentre.y, 
                        localMaps[currentLocalMap].currentGlobalPos);
      float2 mapOtherPos = convertToGlobalCoord(localMaps[globalWarp].robotMapCentre.x, 
                        localMaps[globalWarp].robotMapCentre.y, 
                        localMaps[globalWarp].currentGlobalPos);
      /*float2 mapCurPos = (float2) (localMaps[currentLocalMap].currentGlobalPos.x + 
                        localMaps[currentLocalMap].mapCentre.x,
                        localMaps[currentLocalMap].currentGlobalPos.y +
                        localMaps[currentLocalMap].mapCentre.y);
      float2 mapOtherPos = (float2) (localMaps[globalWarp].currentGlobalPos.x + 
                        localMaps[globalWarp].mapCentre.x,
                        localMaps[globalWarp].currentGlobalPos.y +
                        localMaps[globalWarp].mapCentre.y);*/

      if (fabs(mapCurPos.x - mapOtherPos.x) < totalCovar[warpNum][0][0] + config->LocalMapDist &&
            fabs(mapCurPos.y - mapOtherPos.y) < totalCovar[warpNum][1][1] + config->LocalMapDist) {
         //In the right area for a match, so do histogram correlation
         int i, j;
         int offset = NUM_ORIENTATION_BINS * warpNum;
         //Perform the correlations for the entropy and orientation histograms
         for (i = warpIndex; i < NUM_ORIENTATION_BINS; i += WARP_SIZE) {
            orienCorr[i + offset] = 0;
            entCorr[i + offset] = 0;
            for(j = 0; j < NUM_ORIENTATION_BINS; j++) {
               int temp = i + j;
               if(temp >= NUM_ORIENTATION_BINS) {
                  temp = temp - NUM_ORIENTATION_BINS;
               }
               orienCorr[i + offset] += localMaps[currentLocalMap].orientationHist[j] * 
                                         localMaps[globalWarp].orientationHist[temp];
               entCorr[i + offset] += localMaps[currentLocalMap].entropyHist[j] *
                                          localMaps[globalWarp].entropyHist[temp];
            }
         }
         //Find the peaks in the histograms
         for (i = warpIndex; i < NUM_ORIENTATION_BINS; i += WARP_SIZE) {
            int isPeak = findIfPeak(orienCorr, offset, i);
            if (!isPeak) {
               isPeak = findIfPeak(entCorr, offset, i);
            }
            if (isPeak) {
               int ret = atomic_inc(&(numPeaks[warpNum]));
               /*if (ret > 31) {
                  atomic_dec(&(numPeaks[warpNum]));
               } else {*/
               peaks[warpNum * WARP_SIZE + ret] = i;
               //}
            }
         }
         //Find the index of the largest peak in the orientation histogram
         if (warpIndex == 0) {
            float max = 0;
            int maxIndex = -1;
            for (i = 0; i < numPeaks[warpNum] && i < 32; i++) {
               if (orienCorr[offset + peaks[warpNum * WARP_SIZE + i]] > max) {
                  max = orienCorr[offset + peaks[warpNum * WARP_SIZE + i]];
                  maxIndex = peaks[warpNum * WARP_SIZE + i];
               }
            }
            maxPeak[warpNum] = maxIndex;
         }
         //Go through the peaks and perform the projection correlations as well
         float maxCorrScore = 0;
         int maxOrien = 0;
         int maxX = 0;
         int maxY = 0;

         //float t1 = 0.0, t2 = 0.0, t3 = 0.0, t4 = 0.0;

         for (i = 0; i < numPeaks[warpNum] && i < 32; i++) {
            float maxValue;
            int maxIndex;
            float maxValue90;
            int maxIndex90;
            maxValue = correlateProjection(localMaps[currentLocalMap].projectionHist,
                localMaps[globalWarp].projectionHist, maxPeak[warpNum],
                peaks[warpNum * WARP_SIZE + i], warpIndex, &maxIndex, tempCorr, 
                tempCorrIndex, warpNum * WARP_SIZE);
            maxValue90 = correlateProjection(localMaps[currentLocalMap].projectionHist,
                localMaps[globalWarp].projectionHist, (maxPeak[warpNum] + 
                NUM_ORIENTATION_BINS/4)%NUM_ORIENTATION_BINS,
                peaks[warpNum * WARP_SIZE + i], warpIndex, &maxIndex90, tempCorr, 
                tempCorrIndex, warpNum * WARP_SIZE);
            float score = maxValue + maxValue90 + orienCorr[offset + peaks[warpNum * WARP_SIZE + i]] +
                         entCorr[offset + peaks[warpNum * WARP_SIZE + i]];
            if (score > maxCorrScore) {
               maxCorrScore = score;
               maxOrien = peaks[warpNum * WARP_SIZE + i];
               maxX = maxIndex;
               maxY = maxIndex90;

            }            
         }
         if (warpIndex == 0 && maxCorrScore > config->CorrelationThreshold) {
         //if (warpIndex == 0) {
            int res = atomic_inc(&(common->numPotentialMatches));
            common->potentialMatches[res] = globalWarp;
            //convert to x,y,theta for the next stage of alignment
            float mapSize = config->DimLocalOG * config->CellWidthOG;
            float tempX = (maxX - NUM_PROJECTION_BINS) * mapSize / (float) NUM_PROJECTION_BINS;
            float tempY = (maxY - NUM_PROJECTION_BINS) * mapSize / (float) NUM_PROJECTION_BINS;
            float maxTheta = (2 * M_PI * maxPeak[warpNum]) / NUM_ORIENTATION_BINS - 
                             (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
                              / (float) (2 * NUM_ORIENTATION_BINS));
            float cosTh = cos(-maxTheta);
            float sinTh = sin(-maxTheta);
            common->potentialMatchX[res] = (tempX + sinTh * tempY / cosTh) / 
                                           (cosTh + sinTh * sinTh / cosTh);
            common->potentialMatchY[res] = (cosTh * common->potentialMatchX[res] - tempX) / sinTh;
            //This is what it should be:
            //common->potentialMatchTheta[res] = (2 * M_PI * maxOrien) / NUM_ORIENTATION_BINS - 
            //                                    (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
            //                                    / (float) (2 * NUM_ORIENTATION_BINS));
            //This is what works:
            common->potentialMatchTheta[res] = ((2 * M_PI * maxOrien) / NUM_ORIENTATION_BINS - 
                                                (M_PI * (float)(2 * NUM_ORIENTATION_BINS - 1) 
                                                / (float) (2 * NUM_ORIENTATION_BINS)) + M_PI) * -1;
            common->potentialMatchX[res] *= -1;
            common->potentialMatchY[res] *= -1;
            common->potentialMatchParent[res] = parentIndex;
            ANGNORM(common->potentialMatchTheta[res]);
            out[res] = maxCorrScore;
         }
      }
   }
}

__kernel void alignICP(constant slamConfig *config, global slamLocalMap *localMaps,
        global slamCommon *common, const int otherMap, const int currentMap,
        const int numPoints, const int matchIndex) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int groupSize = get_local_size(0);
   int warpIndex = localIndex % WARP_SIZE;
   int numWarps = groupSize / WARP_SIZE;
   
   local float A[WARP_SIZE][3][3];
   local float4 B[LOCAL_SIZE];
   local int goodCount[LOCAL_SIZE];
   
   if (localIndex < WARP_SIZE) {
      A[localIndex][0][0] = 0.0f;
      A[localIndex][0][1] = 0.0f;
      A[localIndex][0][2] = 0.0f;
      A[localIndex][1][0] = 0.0f;
      A[localIndex][1][1] = 0.0f;
      A[localIndex][1][2] = 0.0f;
      A[localIndex][2][0] = 0.0f;
      A[localIndex][2][1] = 0.0f;
      A[localIndex][2][2] = 0.0f;
   }
   B[localIndex] = 0.0f;
   goodCount[localIndex] = 0;
   barrier(CLK_LOCAL_MEM_FENCE);

   if (index < numPoints) {
      float4 offset = (float4) (common->potentialMatchX[matchIndex], 
                      common->potentialMatchY[matchIndex], 0, common->potentialMatchTheta[matchIndex]);
      int searchFactor = 1;
      if (common->numIterations < 2) {
         searchFactor = 4;
      } else if (common->numIterations < 4) {
         searchFactor = 2;
      }
      //Transform the points. Offsets are currently relative to the new map, so shouldn't use
      //convertReferenceFrame
      float cosTh = cos(offset.w);
      float sinTh = sin(offset.w);
      float2 transformedPoint;
      float2 p = (float2)(localMaps[otherMap].pointsX[index], localMaps[otherMap].pointsY[index]);
      transformedPoint.x = (p.x * cosTh - p.y * sinTh) + offset.x;
      transformedPoint.y = (p.x * sinTh + p.y * cosTh) + offset.y;
      /*float2 transformedPoint = convertReferenceFrame((float2) (localMaps[otherMap].pointsX[index], 
                       localMaps[otherMap].pointsY[index]), offset);*/
      //TODO: looks at the number of beighbouring cells examined to find a matching point
      int matchIndex = findMatchingPoint(config, localMaps, common, transformedPoint, 
                  currentMap, offset, searchFactor);
      //If found a matching point, calculate its contribution to the match
      if (matchIndex >= 0) {
         float x12, y12, k, temp;
         //lPoint is the point on the map ie. mapPoint
         //point is the laser point not in the map ie. transformedPoint
         float2 mapPoint = (float2) (localMaps[currentMap].pointsX[matchIndex],
                                     localMaps[currentMap].pointsY[matchIndex]);
         //Doing this as per UseSimpleH in pogmbicp code as no 3d yet
         x12 = mapPoint.x * mapPoint.x;
         y12 = mapPoint.y * mapPoint.y;
         k = 1.0f / (x12 + y12 + config->SlamL * config->SlamL);
         temp = 1.0f - y12 * k;
         atomicFloatAddLocal(&(A[warpIndex][0][0]), temp);
         temp = mapPoint.x * mapPoint.y * k;
         atomicFloatAddLocal(&(A[warpIndex][0][1]), temp);
         temp = - transformedPoint.y + mapPoint.y * (transformedPoint.y * mapPoint.y +
                transformedPoint.x * mapPoint.x) * k;
         atomicFloatAddLocal(&(A[warpIndex][0][2]), temp);
         temp = 1.0f - x12 * k;
         atomicFloatAddLocal(&(A[warpIndex][1][1]), temp);
         temp = transformedPoint.x - mapPoint.x * (transformedPoint.y * mapPoint.y +
                transformedPoint.x * mapPoint.x) * k;
         atomicFloatAddLocal(&(A[warpIndex][1][2]), temp);
         temp = x12 + y12 - pown(transformedPoint.y * mapPoint.y +
                transformedPoint.x * mapPoint.x, 2) * k;
         atomicFloatAddLocal(&(A[warpIndex][2][2]), temp);
         B[localIndex].s0 = transformedPoint.x - mapPoint.x - mapPoint.y *
             (mapPoint.y * transformedPoint.x - mapPoint.x * transformedPoint.y) * k;
         B[localIndex].s1 = transformedPoint.y - mapPoint.y + mapPoint.x *
             (mapPoint.y * transformedPoint.x - mapPoint.x * transformedPoint.y) * k;
         B[localIndex].s2 = (mapPoint.y * transformedPoint.x - mapPoint.x *
             transformedPoint.y) * (-1.0f +
            (mapPoint.y * transformedPoint.y + mapPoint.x * transformedPoint.x) * k);
         goodCount[localIndex]++;
      }
   }

   //Now need to reduce A, B and goodCount into singular values for the group
   //and add the entries in the common data structure to store them
   if (warpIndex < 16) {
      B[localIndex] += B[localIndex + 16];
      goodCount[localIndex] += goodCount[localIndex + 16];
   }
   if (warpIndex < 8) {
      B[localIndex] += B[localIndex + 8];
      goodCount[localIndex] += goodCount[localIndex + 8];
   }
   if (warpIndex < 4) {
      B[localIndex] += B[localIndex + 4];
      goodCount[localIndex] += goodCount[localIndex + 4];
   }
   if (warpIndex < 2) {
      B[localIndex] += B[localIndex + 2];
      goodCount[localIndex] += goodCount[localIndex + 2];
   }
   if (warpIndex < 1) {
      B[localIndex] += B[localIndex + 1];
      goodCount[localIndex] += goodCount[localIndex + 1];
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (localIndex < 16 && localIndex < numWarps) {
      B[localIndex] = B[localIndex * WARP_SIZE];
      goodCount[localIndex] = goodCount[localIndex * WARP_SIZE];
   } else if (localIndex < 16) {
      B[localIndex] = 0.0f;
      goodCount[localIndex] = 0;
   }
   if (localIndex < 16) {
      A[localIndex][0][0] += A[localIndex + 16][0][0];
      A[localIndex][0][1] += A[localIndex + 16][0][1];
      A[localIndex][0][2] += A[localIndex + 16][0][2];
      A[localIndex][1][0] += A[localIndex + 16][1][0];
      A[localIndex][1][1] += A[localIndex + 16][1][1];
      A[localIndex][1][2] += A[localIndex + 16][1][2];
      A[localIndex][2][0] += A[localIndex + 16][2][0];
      A[localIndex][2][1] += A[localIndex + 16][2][1];
      A[localIndex][2][2] += A[localIndex + 16][2][2];
   }
   if (localIndex < 8) {
      A[localIndex][0][0] += A[localIndex + 8][0][0];
      A[localIndex][0][1] += A[localIndex + 8][0][1];
      A[localIndex][0][2] += A[localIndex + 8][0][2];
      A[localIndex][1][0] += A[localIndex + 8][1][0];
      A[localIndex][1][1] += A[localIndex + 8][1][1];
      A[localIndex][1][2] += A[localIndex + 8][1][2];
      A[localIndex][2][0] += A[localIndex + 8][2][0];
      A[localIndex][2][1] += A[localIndex + 8][2][1];
      A[localIndex][2][2] += A[localIndex + 8][2][2];
      B[localIndex] += B[localIndex + 8];
      goodCount[localIndex] += goodCount[localIndex + 8];
   }
   if (localIndex < 4) {
      A[localIndex][0][0] += A[localIndex + 4][0][0];
      A[localIndex][0][1] += A[localIndex + 4][0][1];
      A[localIndex][0][2] += A[localIndex + 4][0][2];
      A[localIndex][1][0] += A[localIndex + 4][1][0];
      A[localIndex][1][1] += A[localIndex + 4][1][1];
      A[localIndex][1][2] += A[localIndex + 4][1][2];
      A[localIndex][2][0] += A[localIndex + 4][2][0];
      A[localIndex][2][1] += A[localIndex + 4][2][1];
      A[localIndex][2][2] += A[localIndex + 4][2][2];
      B[localIndex] += B[localIndex + 4];
      goodCount[localIndex] += goodCount[localIndex + 4];
   }
   
   if (localIndex < 2) {
      A[localIndex][0][0] += A[localIndex + 2][0][0];
      A[localIndex][0][1] += A[localIndex + 2][0][1];
      A[localIndex][0][2] += A[localIndex + 2][0][2];
      A[localIndex][1][0] += A[localIndex + 2][1][0];
      A[localIndex][1][1] += A[localIndex + 2][1][1];
      A[localIndex][1][2] += A[localIndex + 2][1][2];
      A[localIndex][2][0] += A[localIndex + 2][2][0];
      A[localIndex][2][1] += A[localIndex + 2][2][1];
      A[localIndex][2][2] += A[localIndex + 2][2][2];
      B[localIndex] += B[localIndex + 2];
      goodCount[localIndex] += goodCount[localIndex + 2];
   }
   if (localIndex < 1) {
      A[localIndex][0][0] += A[localIndex + 1][0][0];
      A[localIndex][0][1] += A[localIndex + 1][0][1];
      A[localIndex][0][2] += A[localIndex + 1][0][2];
      A[localIndex][1][0] += A[localIndex + 1][1][0];
      A[localIndex][1][1] += A[localIndex + 1][1][1];
      A[localIndex][1][2] += A[localIndex + 1][1][2];
      A[localIndex][2][0] += A[localIndex + 1][2][0];
      A[localIndex][2][1] += A[localIndex + 1][2][1];
      A[localIndex][2][2] += A[localIndex + 1][2][2];
      B[localIndex] += B[localIndex + 1];
      goodCount[localIndex] += goodCount[localIndex + 1];

      atomicFloatAdd(&(common->B[0]), B[0].s0);
      atomicFloatAdd(&(common->B[1]), B[0].s1);
      atomicFloatAdd(&(common->B[2]), B[0].s2);
      atomic_add(&(common->goodCount), goodCount[0]);
   }
   if (localIndex < 3) {
      atomicFloatAdd(&(common->A[localIndex][0]), A[0][localIndex][0]);
      atomicFloatAdd(&(common->A[localIndex][1]), A[0][localIndex][1]);
      atomicFloatAdd(&(common->A[localIndex][2]), A[0][localIndex][2]);
   }
}

__kernel void calculateICPMatrix(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int currentMap, const int matchIndex, const int fullLoop,
      global float *out) {

   int index = get_global_id(0);

   local float shift[3];

   if (index < 3) {
      shift[index] = -common->B[index] / common->A[index][index];
   }
   if (index < 2) {
      shift[index] += (common->A[index][index + 1] * common->B[index + 1]) /
         (common->A[index][index] * common->A[index + 1][index + 1]);
   }
   if (index == 0) {
      int finished = 0;
      shift[0] += common->B[2] * (common->A[0][2] * common->A[1][1] - 
            common->A[0][1] * common->A[1][2]) / (common->A[0][0] *
               common->A[1][1] * common->A[2][2]);

      out[0] = common->goodCount;
      out[1] = shift[0];
      out[2] = shift[1];
      out[3] = shift[2];
      /*out[1] = common->potentialMatchX[0];
      out[2] = common->potentialMatchY[0];
      out[3] = common->potentialMatchTheta[0];*/

      if (common->goodCount > config->MinGoodCount) {
         //Add the amount of the shift to the move offset
         common->potentialMatchX[matchIndex] += shift[0];
         common->potentialMatchY[matchIndex] += shift[1];
         common->potentialMatchTheta[matchIndex] += shift[2];

         if (common->numIterations >= config->MaxIterations &&
               common->goodCount > config->FinalMinGoodCount) {
            //If the max number of iterations have been exceeded, but the match
            //strength is good, end it successfully
            common->matchSuccess = 1;
            finished = 1;
         } else if (fabs(shift[0]) <= config->MaxErrorDisp &&
                  fabs(shift[1]) <= config->MaxErrorDisp &&
                  fabs(shift[2]) <= config->MaxErrorTheta &&
                  common->goodCount > config->FinalMinGoodCount) {
            common->matchSuccess = 1;
            finished = 1;
         } else if (common->numIterations >= config->MaxIterations) {
            common->matchSuccess = -1;
            finished = 1;
         } else if (fullLoop == 0 && fabs(shift[0]) <= config->MaxErrorDisp &&
                   fabs(shift[1]) <= config->MaxErrorDisp && fabs(shift[2]) <= config->MaxErrorTheta) {
            common->matchSuccess = 1;
            finished = 1;
         }
      } else {
         //Match failed
         common->matchSuccess = -1;
         finished = 1;
      }
      if (finished) {
         common->numIterations = 0;
         /*if (common->combineIndex < 0 && common->matchSuccess > 0 && config->MaxThetaOptimise > 0) {
            float thetaChange = common->potentialMatchTheta[matchIndex];
            float thetaOld = localMaps[common->potentialMatches[matchIndex]].currentGlobalPos.w;
            float thetaNew = localMaps[currentMap].currentGlobalPos.w;
            //if (fabs(thetaOld + thetaChange - thetaNew) > config->MaxThetaOptimise) {
            if (fabs(thetaNew + thetaChange - thetaOld) > config->MaxThetaOptimise) {
               //common->matchSuccess = -1;
            }     
            out[0] = thetaNew;
            out[1] = thetaChange;
            out[2] = thetaOld;
            out[3] = config->MaxThetaOptimise;

         }*/
         if (common->combineIndex < 0 && common->matchSuccess > 0) {
            //If successful match, will need to run evalute map match kernel so stop now
            common->evaluateOverlap = 0;
            common->evaluateScore = 0;

         } else if (common->combineIndex >= 0 && common->matchSuccess < 0) {
            int cIndex = common->numConstraints;
            common->numConstraints++;
            common->constraintType[cIndex] = 0;
            common->constraintIndex[cIndex] = currentMap;
            common->combineIndex = -1;
            common->combineMode = 0;
         }
      } else {
         common->numIterations++;
         common->matchSuccess = 0;
      }
      //Needs to be reset every iteration
      common->goodCount = 0;
   }
   if (index < 3) {
      common->A[index][0] = 0;
      common->A[index][1] = 0;
      common->A[index][2] = 0;
      common->B[index] = 0;
   }
}

__kernel void addLoopClosure(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int currentMap, const int matchIndex, 
      const int fullLoop, const float previousScore, global float *out) {

   int index = get_global_id(0);

   if (index == 0) {

      float score = common->evaluateScore / (float) common->evaluateOverlap;
      out[0] = score;
      out[1] = common->evaluateScore;
      out[2] = common->evaluateOverlap;
      
      float constraintTh = -common->potentialMatchTheta[matchIndex];
      float angleDiff = localMaps[currentMap].currentGlobalPos.w - 
         (localMaps[common->potentialMatches[matchIndex]].currentGlobalPos.w + constraintTh);
      ANGNORM(angleDiff);
      int stopMatch = 0;
      if (config->PreventMatchesSymmetrical && 
            (angleDiff < -3.0 * M_PI / 4.0 || angleDiff > 3.0 * M_PI / 4.0)) {
         stopMatch = 1;
      }

      if (((fullLoop == 1 && score < config->FreeAreaThreshold) || 
            (fullLoop == 0 && score < previousScore && score < config->FreeAreaThreshold 
            && score != -1)) && stopMatch == 0) {
         //If successful match, add match information to data structures
         //Don't need atomic operations as only one thread
         int mIndex = common->numConstraints;
         common->numConstraints++;
         int loopIndex = common->numLoopConstraints;
         common->numLoopConstraints++;
         common->constraintType[mIndex] = 1;
         common->constraintIndex[mIndex] = loopIndex;
         common->loopConstraintI[loopIndex] = common->potentialMatches[matchIndex];
         common->loopConstraintJ[loopIndex] = currentMap;
         if (fullLoop == 1) {
            common->loopConstraintParent[loopIndex] = common->potentialMatchParent[matchIndex];
            common->loopConstraintFull[loopIndex] = 1;
            common->loopConstraintWeight[loopIndex] = 1;
         } else {
            int minLevel = min(localMaps[currentMap].treeLevel, 
                  localMaps[common->potentialMatches[matchIndex]].treeLevel);

            int mapIndex = currentMap;
            while (localMaps[mapIndex].treeLevel > minLevel) {
               mapIndex = localMaps[mapIndex].indexParentNode;
            }
            int mapIndexGlobal = common->potentialMatches[matchIndex];
            while (localMaps[mapIndexGlobal].treeLevel > minLevel) {
               mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
            }
            while (mapIndex != mapIndexGlobal) {
               mapIndex = localMaps[mapIndex].indexParentNode;
               mapIndexGlobal = localMaps[mapIndexGlobal].indexParentNode;
            }
            common->loopConstraintParent[loopIndex] = mapIndex;
            common->loopConstraintFull[loopIndex] = 0;
            //common->loopConstraintWeight[loopIndex] = common->evaluateOverlap / 
            //     (float) localMaps[common->potentialMatches[matchIndex]].numPoints;
            common->loopConstraintWeight[loopIndex] = 1;
         }

         common->loopConstraintInfo[loopIndex][0][0] = 0;
         common->loopConstraintInfo[loopIndex][0][1] = 0;
         common->loopConstraintInfo[loopIndex][0][2] = 0;
         common->loopConstraintInfo[loopIndex][1][0] = 0;
         common->loopConstraintInfo[loopIndex][1][1] = 0;
         common->loopConstraintInfo[loopIndex][1][2] = 0;
         common->loopConstraintInfo[loopIndex][2][0] = 0;
         common->loopConstraintInfo[loopIndex][2][1] = 0;
         common->loopConstraintInfo[loopIndex][2][2] = 0;

         common->loopConstraintThetaDisp[loopIndex] = - common->potentialMatchTheta[matchIndex];
         float cosTh = cos(-common->potentialMatchTheta[matchIndex]);
         float sinTh = sin(-common->potentialMatchTheta[matchIndex]);
         common->loopConstraintXDisp[loopIndex] = (cosTh * -common->potentialMatchX[matchIndex] -
                                                  sinTh * -common->potentialMatchY[matchIndex]);
         common->loopConstraintYDisp[loopIndex] = (sinTh * -common->potentialMatchX[matchIndex] +
                                                  cosTh * -common->potentialMatchY[matchIndex]);
         ANGNORM(common->loopConstraintThetaDisp[loopIndex]);
         //common->loopConstraintXDisp[loopIndex] = - common->potentialMatchX[matchIndex];
         //common->loopConstraintYDisp[loopIndex] = - common->potentialMatchY[matchIndex];

         if (fullLoop == 1) {
            common->currentOffset.w += common->loopConstraintThetaDisp[loopIndex];
            float tempX = common->currentOffset.x - common->potentialMatchX[matchIndex]; 
            float tempY = common->currentOffset.y - common->potentialMatchY[matchIndex];
            common->currentOffset.x = cosTh * tempX - sinTh * tempY;
            common->currentOffset.y = sinTh * tempX + cosTh * tempY;

            common->previousINode = common->loopConstraintI[loopIndex];
         }

         if(config->LocalMapCombine) {
            common->combineIndex = common->potentialMatches[matchIndex];
         }
      } else {
         common->matchSuccess = -1;
      }
   }
}

/*
 * Normalise the information matrix and rotate it so it is with respect to the lower
 * number local map, not the current local map
 */
__kernel void finaliseInformationMatrix(constant slamConfig *config, global slamCommon *common, 
      const int constraintIndex, global float *out) {

   int index = get_global_id(0);

   local float a[3][3];
   local float b[3][3];
   local float c[3][3];

   int cIndex = common->constraintIndex[constraintIndex];

   int x = index % 3;
   int y = index / 3;
   if (index == 0) {
      float cosTh = cos(- common->loopConstraintThetaDisp[cIndex]);
      float sinTh = sin(- common->loopConstraintThetaDisp[cIndex]);
      a[0][0] = cosTh;
      a[1][1] = cosTh;
      a[1][0] = sinTh;
      a[0][1] = -sinTh;
      a[2][2] = 1;
      a[0][2] = 0;
      a[1][2] = 0;
      a[2][0] = 0;
      a[2][1] = 0;

      /*if (common->infoCount < config->InformationScaleFactor) {
         common->infoCount = config->InformationScaleFactor;
      }*/
   }
   if (index < 9) {
 
      //Adjust the parent info matrix to be relative to the parent instead
      //of relative to the current node
      //b[y][x] = common->loopConstraintInfo[cIndex][y][x] /= 
      //                  ((float)common->infoCount / (float)config->InformationScaleFactor);
      b[y][x] = common->loopConstraintInfo[cIndex][y][x];
      mult3x3MatrixLocal(a, b, c, index);
      if (index == 0) {
         a[1][0] *= -1;
         a[0][1] *= -1;
      }
      mult3x3MatrixLocal(c, a, b, index);
      common->loopConstraintInfo[cIndex][y][x] = b[y][x];
      
      out[index] = common->loopConstraintInfo[cIndex][y][x];
   }
   if (index == 0) {
      out[9] = common->infoCount;
      common->infoCount = 0;
   }
}

/*
 * Evaluates the amount of matching overlap between maps
 *
 * mode is -1 if used to see if a temp constraint should be investigated, >= 0 if 
 * used to evaluate how good an icp alignment between maps is, where the value
 * is the index of the match in the potential matches array 
 */ 
__kernel void evaluateMapMatch(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global float *freeAreas, 
      const int curMap, const int testMap, const int mode) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpIndex = localIndex % WARP_SIZE;

   local int numOverlap[WARP_SIZE];
   local float score[WARP_SIZE];

   if (localIndex < WARP_SIZE) {
      numOverlap[localIndex] = 0;
      score[localIndex] = 0;
   }
   barrier(CLK_LOCAL_MEM_FENCE);

   if (index < localMaps[testMap].numPoints) {
      float offX, offY, offTh;
      if (mode == -1) {
         ocl_float4 gOff = localMaps[curMap].currentGlobalPos - localMaps[testMap].currentGlobalPos;
         float cosTh = cos(-localMaps[testMap].currentGlobalPos.w);
         float sinTh = sin(-localMaps[testMap].currentGlobalPos.w);
         offTh = gOff.w;
         offX = cosTh * gOff.x - sinTh * gOff.y;
         offY = sinTh * gOff.x + cosTh * gOff.y;
         if (index == 0) {
            float cosM = cos(-offTh);
            float sinM = sin(-offTh);
            common->potentialMatches[0] = testMap;
            common->potentialMatchTheta[0] = -offTh;
            common->potentialMatchX[0] = cosM * -offX - sinM * -offY;
            common->potentialMatchY[0] = sinM * -offX + cosM * -offY;
         }
      } else {
         offTh = -common->potentialMatchTheta[mode];
         float cosTh = cos(-common->potentialMatchTheta[mode]);
         float sinTh = sin(-common->potentialMatchTheta[mode]);
         offX = cosTh * -common->potentialMatchX[mode] - sinTh * -common->potentialMatchY[mode];
         offY = sinTh * -common->potentialMatchX[mode] + cosTh * -common->potentialMatchY[mode];
      }
      ANGNORM(offTh);
      float2 p = (float2) (localMaps[testMap].pointsX[index], localMaps[testMap].pointsY[index]);
      float4 off = (float4) (offX, offY, 0, offTh);
      float2 res = convertReferenceFrame(p, off);
      int index = getLocalOGIndex(config, res.x, res.y);
      int freeAreasOffset = curMap * config->DimLocalOG * config->DimLocalOG;
      if (index >= 0 && freeAreas[freeAreasOffset + index] < INFINITY) {
         atomicFloatAddLocal(&(score[warpIndex]), freeAreas[freeAreasOffset + index]);
         atomic_inc(&(numOverlap[warpIndex]));
      }
   } 
   barrier(CLK_LOCAL_MEM_FENCE);
   parallelReduceFloat(score, localIndex, &(common->evaluateScore));
   parallelReduceInt(numOverlap, localIndex, &(common->evaluateOverlap));
}

__kernel void evaluateTempConstraints(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common) {
   int index = get_global_id(0);

   if (index < common->numLoopConstraints) {
      if (common->loopConstraintFull[index] == 0) {
         int iNode = common->loopConstraintI[index];
         int jNode = common->loopConstraintJ[index];
         float cosTh = cos(localMaps[iNode].currentGlobalPos.w);
         float sinTh = sin(localMaps[iNode].currentGlobalPos.w);

         float tempX = cosTh * common->loopConstraintXDisp[index] - 
            sinTh  * common->loopConstraintYDisp[index] + localMaps[iNode].currentGlobalPos.x;
         float tempY = sinTh * common->loopConstraintXDisp[index] + 
            cosTh  * common->loopConstraintYDisp[index] + localMaps[iNode].currentGlobalPos.y;
         float tempTh = common->loopConstraintThetaDisp[index] + localMaps[iNode].currentGlobalPos.w;
         ANGNORM(tempTh);

         if (fabs(tempX - localMaps[jNode].currentGlobalPos.x) > config->TempConstraintMovementXY ||
            fabs(tempY - localMaps[jNode].currentGlobalPos.y) > config->TempConstraintMovementXY ||
            fabs(tempTh - localMaps[jNode].currentGlobalPos.w) > config->TempConstraintMovementTh) {
            common->loopConstraintWeight[index] = 0;
         }
      }
   }
}

/*
 * Calculate the global Hessian matrix for the optimisation step
 */
__kernel void getGlobalHessianMatrix(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int numMaps, global float *out) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpNum = localIndex / WARP_SIZE;
   int warpIndex = localIndex % WARP_SIZE;
   int groupSize = get_local_size(0);
   int groupNum = get_group_id(0);
   int globalWarp = (groupNum * groupSize + localIndex)/ WARP_SIZE;

   //Reset the changeInPos vairables before the optimise step
   //There should always be well over numMaps threads
   if (index < numMaps) {
      localMaps[index].changeInPos[0] = 0;
      localMaps[index].changeInPos[1] = 0;
      localMaps[index].changeInPos[2] = 0;
      localMaps[index].numConstraints = 0;
   }

   local float a[LOCAL_SIZE / WARP_SIZE][3][3];
   local float b[LOCAL_SIZE / WARP_SIZE][3][3];
   local float c[LOCAL_SIZE / WARP_SIZE][3][3];
   if (globalWarp < common->numConstraints) {
//if (common->constraintType[globalWarp] != 1 || globalWarp >= common->numConstraints - 2) {
      int x = warpIndex % 3;
      int y = warpIndex / 3;

      int constraintType = common->constraintType[globalWarp];
      int constraintIndex = common->constraintIndex[globalWarp];
      int iNode;
      int jNode;
      int parentIndex;

      if (constraintType == 1) {
         return;
      }
      
      if (constraintType == 1) {
         iNode = common->loopConstraintI[constraintIndex];
         parentIndex = common->loopConstraintParent[constraintIndex];
         jNode = common->loopConstraintJ[constraintIndex];

         if (warpIndex < 9) {
            b[warpNum][y][x] = common->loopConstraintInfo[constraintIndex][y][x];
         }
      } else {
         iNode = localMaps[constraintIndex].indexParentNode;
         parentIndex = iNode;
         jNode = constraintIndex;
         if (warpIndex < 9) {
            b[warpNum][y][x] = localMaps[constraintIndex].parentInfo[y][x];
         }
      }
      if (warpIndex == 0) {
         float cosTh = cos(localMaps[iNode].currentGlobalPos.w);
         float sinTh = sin(localMaps[iNode].currentGlobalPos.w);
         a[warpNum][0][0] = cosTh;
         a[warpNum][1][1] = cosTh;
         a[warpNum][1][0] = sinTh;
         a[warpNum][0][1] = -sinTh;
         a[warpNum][2][2] = 1;
         a[warpNum][0][2] = 0;
         a[warpNum][1][2] = 0;
         a[warpNum][2][0] = 0;
         a[warpNum][2][1] = 0;
      }
      mult3x3MatrixLocal(a[warpNum], b[warpNum], c[warpNum], warpIndex);
      if (warpIndex == 0) {
         a[warpNum][1][0] *= -1;
         a[warpNum][0][1] *= -1;
      }
      mult3x3MatrixLocal(c[warpNum], a[warpNum], b[warpNum], warpIndex);
      //Now have info matrix for constraint in global reference frame in b[warpNum]

      /*if (warpIndex < 9) {
         b[warpNum][y][x] /= 150.0f;
         if (b[warpNum][y][x] > 1) {
            b[warpNum][y][x] = 1;
         }
      }*/
      /*if (warpIndex < 9) {
         if (b[warpNum][y][x] < 0) {
            b[warpNum][y][x] = 0;
         }
      }*/

      if (warpIndex < 3) {
         int tempNode = iNode;
         while (tempNode != parentIndex) {
            tempNode = localMaps[tempNode].indexParentNode;
            atomicFloatAdd(&(common->graphHessian[tempNode][warpIndex]), 
                  b[warpNum][warpIndex][warpIndex]);
         }
         tempNode = jNode;
         while (tempNode != parentIndex) {
            atomicFloatAdd(&(common->graphHessian[tempNode][warpIndex]), 
                  b[warpNum][warpIndex][warpIndex]);
            tempNode = localMaps[tempNode].indexParentNode;
         }
         atomicFloatMin(&(common->scaleFactor[warpIndex]), b[warpNum][warpIndex][warpIndex]);
      }

      /*if (index < 60) {
        out[index] = -0.787;
      }*/
//}
   }
}

float getGlobalPosIndex(float4 v, int index) {
   if (index == 0) return v.x;
   if (index == 1) return v.y;
   if (index == 2) return v.w;
   return 0;
}

__kernel void calculateOptimisationChange(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int numIterations, const int numMaps, const int type,
      const int lastFullLoopIndex, global float *out) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpNum = localIndex / WARP_SIZE;
   int warpIndex = localIndex % WARP_SIZE;
   int groupSize = get_local_size(0);
   int groupNum = get_group_id(0);
   int globalWarp = (groupNum * groupSize + localIndex)/ WARP_SIZE;

   local float a[LOCAL_SIZE / WARP_SIZE][3][3];
   local float b[LOCAL_SIZE / WARP_SIZE][3][3];
   local float c[LOCAL_SIZE / WARP_SIZE][3][3];

   local float constraint[LOCAL_SIZE / WARP_SIZE][3];
   local float residual[LOCAL_SIZE / WARP_SIZE][3];

   int startIndex = 0;
   if (type == -1) {
      startIndex = lastFullLoopIndex + 1;
   }
   if (globalWarp < common->numConstraints && globalWarp >= startIndex) {
//if (common->constraintType[globalWarp] != 1 || globalWarp >= common->numConstraints - 2) {
      int x = warpIndex % 3;
      int y = warpIndex / 3;
      
      int constraintType = common->constraintType[globalWarp];
      int constraintIndex = common->constraintIndex[globalWarp];
      int iNode;
      int jNode;
      int parentIndex;
      float weight = 0;

      if (constraintType != 1) {
         return;
      }
      
      if (constraintType == 1) {
         if (type == 1 && common->loopConstraintFull[constraintIndex] == 0) {
            return;
         }
         iNode = common->loopConstraintI[constraintIndex];
         parentIndex = common->loopConstraintParent[constraintIndex];
         jNode = common->loopConstraintJ[constraintIndex];
         if (warpIndex < 9) {
            b[warpNum][y][x] = common->loopConstraintInfo[constraintIndex][y][x];
         }
         if (warpIndex == 0) {
            constraint[warpNum][0] = common->loopConstraintXDisp[constraintIndex];
            constraint[warpNum][1] = common->loopConstraintYDisp[constraintIndex];
            constraint[warpNum][2] = common->loopConstraintThetaDisp[constraintIndex];
         }
         weight = common->loopConstraintWeight[constraintIndex];
         if (weight == 0) {
            return;
         }
      } else {
         iNode = localMaps[constraintIndex].indexParentNode;
         parentIndex = iNode;
         jNode = constraintIndex;
         if (warpIndex < 9) {
            b[warpNum][y][x] = localMaps[constraintIndex].parentInfo[y][x];
         }
         if (warpIndex == 0) {
            constraint[warpNum][0] = localMaps[constraintIndex].parentOffset.x;
            constraint[warpNum][1] = localMaps[constraintIndex].parentOffset.y;
            constraint[warpNum][2] = localMaps[constraintIndex].parentOffset.w;
         }
         weight = 1;
         
      }
      int pathLength = (localMaps[iNode].treeLevel - localMaps[parentIndex].treeLevel) +
                       (localMaps[jNode].treeLevel - localMaps[parentIndex].treeLevel);

      if (type == -1) {
         pathLength = localMaps[jNode].treeLevel - localMaps[common->previousINode].treeLevel;
      }
      //if (pathLength > 1 && warpIndex < 9) {
      //   out[warpIndex] = common->loopConstraintInfo[constraintIndex][y][x];
      //}


      //Calculate the rotated information matrix for the constraint 
      //answer is in b - available to all nodes in the warp
      if (warpIndex == 0) {
         float cosTh = cos(localMaps[iNode].currentGlobalPos.w);
         float sinTh = sin(localMaps[iNode].currentGlobalPos.w);
         a[warpNum][0][0] = cosTh;
         a[warpNum][1][1] = cosTh;
         a[warpNum][1][0] = sinTh;
         a[warpNum][0][1] = -sinTh;
         a[warpNum][2][2] = 1;
         a[warpNum][0][2] = 0;
         a[warpNum][1][2] = 0;
         a[warpNum][2][0] = 0;
         a[warpNum][2][1] = 0;
      }

      //Calculate sum inverse dm's and the residual for the constraint
      float dm = 0;
      int tempNode = iNode;
      float tempPos;

      float commonValue;
      float scaleFactor;

      if (warpIndex < 3) {
         if (warpIndex == 2) {
            ANGNORM(constraint[warpNum][2]);
         }
         residual[warpNum][warpIndex] = 0;
         while (tempNode != parentIndex) {
            tempPos = getGlobalPosIndex(localMaps[tempNode].currentGlobalPos, warpIndex);
            if (type != -1) {
               dm += 1 / common->graphHessian[tempNode][warpIndex];
            }
            tempNode = localMaps[tempNode].indexParentNode;
            residual[warpNum][warpIndex] += getGlobalPosIndex(
                  localMaps[tempNode].currentGlobalPos, warpIndex) - tempPos;
         }
         tempNode = jNode;
         int reachedEnd = 0;
         while (tempNode != parentIndex) {
            if (type == -1 && tempNode == common->previousINode) {
               reachedEnd = 1;
            }
            if (reachedEnd == 0) {
               dm += 1 / common->graphHessian[tempNode][warpIndex];
            }
            tempPos = getGlobalPosIndex(localMaps[tempNode].currentGlobalPos, warpIndex);
            tempNode = localMaps[tempNode].indexParentNode;
            residual[warpNum][warpIndex] += tempPos - 
               getGlobalPosIndex(localMaps[tempNode].currentGlobalPos, warpIndex);
         }
         if (warpIndex == 2) {
            ANGNORM(residual[warpNum][warpIndex]);
         }
         residual[warpNum][warpIndex] -= (a[warpNum][warpIndex][0] * constraint[warpNum][0] +
                      a[warpNum][warpIndex][1] * constraint[warpNum][1] +
                      a[warpNum][warpIndex][2] * constraint[warpNum][2]);
         if (warpIndex == 2) {
            ANGNORM(residual[warpNum][warpIndex]);
         }
         //residual[warpNum][warpIndex] -= constraint[warpNum][warpIndex];
         residual[warpNum][warpIndex] *= -1;
         dm = 1/dm;

      }

      if (config->PreventMatchesSymmetrical == 1 && (residual[warpNum][2] > 3.0f * M_PI / 4.0f ||
               residual[warpNum][2] < - 3.0f * M_PI / 4.0f)) {
         if (warpIndex == 0) {
            common->loopConstraintWeight[constraintIndex] = 0;
         }
         return;
      }

      mult3x3MatrixLocal(a[warpNum], b[warpNum], c[warpNum], warpIndex);
      if (warpIndex == 0) {
         a[warpNum][1][0] *= -1;
         a[warpNum][0][1] *= -1;
      }
      mult3x3MatrixLocal(c[warpNum], a[warpNum], b[warpNum], warpIndex);

      /*if (warpIndex < 9) {
         b[warpNum][y][x] /= 150.0f;
         if (b[warpNum][y][x] > 1) {
            b[warpNum][y][x] = 1;
         }
      }*/
      /*if (warpIndex == 0) {
         for(y = 0; y < 3; y++) {
            for(x = 0; x < 3; x++) {
               if (x == 2 && y == 2) {
                  b[warpNum][y][x] /= 2;
               } else if (x != y) {
                  b[warpNum][y][x] /= 10;
               }
               if (b[warpNum][y][x] < 0) {
                  b[warpNum][y][x] = 0;
               }
               if (x == y) {
                  b[warpNum][y][x] = 1;
               } else {
                  b[warpNum][y][x] = 0;
               }
            }
         }
      }*/
      //pathLength = 1;
      /*float sum = 0;
      for(y = 0; y < 3; y++) {
         for(x = 0; x < 3; x++) {
            sum += b[warpNum][y][x];
         }
      }*/

      if (warpIndex < 3) {
         commonValue = b[warpNum][warpIndex][0] * residual[warpNum][0] +
                       b[warpNum][warpIndex][1] * residual[warpNum][1] +
                       b[warpNum][warpIndex][2] * residual[warpNum][2];

         //commonValue = b[warpNum][warpIndex][warpIndex] * residual[warpNum][warpIndex];
         //scaleFactor = 1 / ((float) numIterations * common->scaleFactor[warpIndex]);
         //if (numIterations == 1) {
         scaleFactor = 1 / ((float) numIterations * common->scaleFactor[warpIndex]
              /* * (float) numMaps*/);
         /*if (warpIndex < 2) {
            scaleFactor *= 1.3f;
         } else  {
            scaleFactor *= 0.3f;
         }*/
         //} else {
         //scaleFactor = 1 / (((float) numIterations / 1.5f) * common->scaleFactor[warpIndex] * (float) numMaps);
         //}
         /*if (constraintType == 1 && warpIndex == 2) {
            scaleFactor *= 1.4f;
         }*/
         //scaleFactor = 1;
      
         //Now do the calculations for each node in the constraint
         float adjust = scaleFactor * pathLength * commonValue;
         if (warpIndex == 2) {
         //   adjust *= 1.1f;
            ANGNORM(adjust);
         }
         if (fabs(adjust) > fabs(residual[warpNum][warpIndex])) {
            adjust = residual[warpNum][warpIndex];
         }
         /*if ((adjust < 0 && residual[warpNum][warpIndex] > 0) ||
               adjust > 0 && residual[warpNum][warpIndex] < 0) {
            adjust *= -1;
         }*/
         tempNode = iNode;
         if (type != -1) {
            while (tempNode != parentIndex) {
               //tempNode = localMaps[tempNode].indexParentNode;
               /*float value = scaleFactor * pathLength * dm * 
                             1/common->graphHessian[tempNode][warpIndex] * -1 *
                             commonValue;*/
               float value = weight * adjust * dm * 
                             1/common->graphHessian[tempNode][warpIndex] * -1;
               atomicFloatAdd(&(localMaps[tempNode].changeInPos[warpIndex]), value);
               if (warpIndex == 0) {
                  atomic_inc(&(localMaps[tempNode].numConstraints));
               }
               tempNode = localMaps[tempNode].indexParentNode;
            }
         }
         tempNode = jNode;

         /*int xx = 0;
         if (constraintType == 1 && warpIndex == 0) {
         for (xx = 0; xx < 60; xx++) {
            out[xx] = 0;
         }
         out[0] = residual[warpNum][0];
         out[1] = scaleFactor;
         out[2] = pathLength;
         out[3] = commonValue;
         }*/
         //if (warpIndex == 0) {
         //   out[globalWarp] = residual[warpNum][1];
         //}
         while (tempNode != parentIndex) {
            if (type == -1 && tempNode == common->previousINode) {
               break;
            }
            /*float value = scaleFactor * pathLength * dm * 
                          1/common->graphHessian[tempNode][warpIndex] * 
                          commonValue;*/
            float value = weight * adjust * dm * 
                          1/common->graphHessian[tempNode][warpIndex];

            /*if (constraintType == 1) {              
            value = residual[warpNum][warpIndex];
            } else {
            value = 0;
            }*/
            //if (constraintType == 1 && warpIndex == 0) {
               //out[tempNode] = dm * 1/common->graphHessian[tempNode][warpIndex];
            //}
            atomicFloatAdd(&(localMaps[tempNode].changeInPos[warpIndex]), value);
            if (warpIndex == 0) {
               atomic_inc(&(localMaps[tempNode].numConstraints));
            }
            tempNode = localMaps[tempNode].indexParentNode;


            //break;
         }
      }
//}
   }
}

__kernel void updateGlobalPositions(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, const int numLocalMaps, global float *out) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);
   int i;

   //Firstly reset the graphHessian and scale factor variables for the next optimisation run
   for (i = index; i < MAX_NUM_CONSTRAINTS + 1; i += globalSize) {
      common->graphHessian[i][0] = 0;
      common->graphHessian[i][1] = 0;
      common->graphHessian[i][2] = 0;
   }
   if (index < 3) {
      common->scaleFactor[index] = INFINITY;
      //common->scaleFactor[index] = 0;
   }

   if (index < numLocalMaps) {
      int curMap = index;
      float4 posChange = (float4) (0.0f, 0.0f, 0.0f, 0.0f);
      while (curMap >= 0) {
         if (localMaps[curMap].numConstraints > 0) {
            posChange.x += localMaps[curMap].changeInPos[0] / (float)localMaps[curMap].numConstraints;
            posChange.y += localMaps[curMap].changeInPos[1] / (float)localMaps[curMap].numConstraints;
            posChange.w += localMaps[curMap].changeInPos[2] / (float)localMaps[curMap].numConstraints;
         }
         curMap = localMaps[curMap].indexParentNode;
      }
      localMaps[index].currentGlobalPos += posChange;
      ANGNORM(localMaps[index].currentGlobalPos.w);
      //if (1) {out[index] = posChange.w;}
      //out[index] = localMaps[index].changeInPos[0];
   }
}

__kernel void combineNodes(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global int *globalMap, const int currentMap, 
      const float alignError, const ocl_float4 offCurMap, const int numGlobalPoints,
      const int numOtherGlobalPoints, global float *globalMapHeights) {
   int index = get_global_id(0);

   int numNewPoints = localMaps[currentMap].numPoints;
   int numOldPoints = localMaps[common->combineIndex].numPoints;  

   float skip = 1.0f;
   if (numNewPoints + numOldPoints > MAX_LOCAL_POINTS) {
      skip = (numNewPoints + numOldPoints) / MAX_LOCAL_POINTS;
   }
   if (index < localMaps[currentMap].numPoints) {
      float temp = (float)((int)(((float)index) / skip)) * skip;
      if (temp >= index || (int)(temp + skip) == index) {
         //Point will be combined into the old map
         //First need to transform point to reference frame of old map
         float cosTh = cos(-common->potentialMatchTheta[0]);
         float sinTh = sin(-common->potentialMatchTheta[0]);
         float2 transformedPoint;
         float2 p = (float2)(localMaps[currentMap].pointsX[index] - 
               common->potentialMatchX[0], localMaps[currentMap].pointsY[index] - 
               common->potentialMatchY[0]);
         transformedPoint.x = (p.x * cosTh - p.y * sinTh);
         transformedPoint.y = (p.x * sinTh + p.y * cosTh);

         int numPointsToAdd = (int)((float)numNewPoints / skip);
         int pointNum = ceil(index/ skip);
         int space = MAX_LOCAL_POINTS - numOldPoints;
         if (pointNum < space) {
            localMaps[common->combineIndex].pointsX[numOldPoints + pointNum] = transformedPoint.x;
            localMaps[common->combineIndex].pointsY[numOldPoints + pointNum] = transformedPoint.y;

            globalMap[numGlobalPoints + pointNum] = convertToGlobalPosition(config, 
                  transformedPoint.x, transformedPoint.y, 
                  localMaps[common->combineIndex].currentGlobalPos);
            globalMapHeights[numGlobalPoints + pointNum] = localMaps[currentMap].pointsZ[index];
         } else {
            numPointsToAdd -= space;
            float spacer = (numOldPoints / (float)numPointsToAdd);
            pointNum -= space;
            localMaps[common->combineIndex].pointsX[(int)(pointNum * spacer)] = transformedPoint.x;
            localMaps[common->combineIndex].pointsY[(int)(pointNum * spacer)] = transformedPoint.y;
            
            globalMap[numOtherGlobalPoints + (int)(pointNum * spacer)] = convertToGlobalPosition(config, 
                  transformedPoint.x, transformedPoint.y, 
                  localMaps[common->combineIndex].currentGlobalPos);
            globalMapHeights[numGlobalPoints + pointNum] = localMaps[currentMap].pointsZ[index];
         }
      }
   }
   int shift = common->potentialMatchTheta[0] * (float)NUM_ORIENTATION_BINS / (2 * M_PI);
   int i;
   if (index < NUM_ORIENTATION_BINS) {
      int toI = index + shift;
      if (toI < 0) {
         toI += NUM_ORIENTATION_BINS;
      }
      toI = toI % NUM_ORIENTATION_BINS;
      localMaps[common->combineIndex].orientationHist[index] = 
            (localMaps[common->combineIndex].orientationHist[index] + 
             localMaps[currentMap].orientationHist[toI]) / 2.0f;
      localMaps[common->combineIndex].entropyHist[index] = 
            (localMaps[common->combineIndex].entropyHist[index] + 
             localMaps[currentMap].entropyHist[toI]) / 2.0f;
      for (i = 0; i < NUM_PROJECTION_BINS; i++) {
         localMaps[common->combineIndex].projectionHist[index][i] = 
            (localMaps[common->combineIndex].projectionHist[index][i] + 
             localMaps[currentMap].projectionHist[toI][i]) / 2.0f;
      }
   }
   local float a[3][3];
   local float b[3][3];
   local float c[3][3];
   local float adjustPos[3];
   local float correctedOffset[3];

   //If are combining multiple nodes, refine the constraints
   if (index < 9 && common->combineMode > 0 && 
         localMaps[currentMap].indexParentNode != common->combineMode) {
      
      float cosTh = cos(-localMaps[currentMap].parentOffset.w);
      float sinTh = sin(-localMaps[currentMap].parentOffset.w);

      float2 temp;
      temp.x = cosTh * -common->potentialMatchX[0] - sinTh * -common->potentialMatchY[0];
      temp.y = sinTh * -common->potentialMatchX[0] + cosTh * -common->potentialMatchY[0];

      correctedOffset[0] = localMaps[currentMap].parentOffset.x - temp.x;
      correctedOffset[1] = localMaps[currentMap].parentOffset.y - temp.y;
      correctedOffset[2] = localMaps[currentMap].parentOffset.w - common->potentialMatchTheta[0];
      
      int x = index % 3;
      int y = index / 3;
      float4 existingOffset;
      if (localMaps[currentMap].indexParentNode != localMaps[common->combineIndex].indexParentNode) {
         correctedOffset[2] *= -1;
         temp.x = -correctedOffset[0];
         temp.y = -correctedOffset[1];
         cosTh = cos(correctedOffset[2]);
         sinTh = sin(correctedOffset[2]);
         correctedOffset[0] = cosTh * temp.x - sinTh * temp.y;
         correctedOffset[1] = sinTh * temp.x + cosTh * temp.y;

         if (index == 0) {
            a[0][0] = cosTh;
            a[1][1] = cosTh;
            a[1][0] = sinTh;
            a[0][1] = -sinTh;
            a[2][2] = 1;
            a[0][2] = 0;
            a[1][2] = 0;
            a[2][0] = 0;
            a[2][1] = 0;
         }
         b[y][x] = localMaps[currentMap].parentInfo[y][x];
         mult3x3MatrixLocal(a,b,c, index);
         if (index == 0) {
            a[1][0] *= -1;
            a[0][1] *= -1;
         }
         mult3x3MatrixLocal(c,a,b, index);
         localMaps[currentMap].parentInfo[y][x] = b[y][x];

         b[y][x] = localMaps[localMaps[currentMap].indexParentNode].parentInfo[y][x];
         existingOffset = localMaps[localMaps[currentMap].indexParentNode].parentOffset;
      } else {
         b[y][x] = localMaps[common->combineIndex].parentInfo[y][x];
         existingOffset = localMaps[common->combineIndex].parentOffset;
      }
      a[y][x] = b[y][x] + localMaps[currentMap].parentInfo[y][x];
      invert3x3MatrixLocal(a, c, index);
      if (index < 3) {
         adjustPos[index] = b[index][0] * existingOffset.x +
                            b[index][1] * existingOffset.y +
                            b[index][2] * existingOffset.w +
                            localMaps[currentMap].parentInfo[index][0] * correctedOffset[0] +
                            localMaps[currentMap].parentInfo[index][1] * correctedOffset[1] +
                            localMaps[currentMap].parentInfo[index][2] * correctedOffset[2];
         correctedOffset[index] = c[index][0] * adjustPos[0] +
                                  c[index][1] * adjustPos[1] +
                                  c[index][2] * adjustPos[2];
      }
      if (localMaps[currentMap].indexParentNode != localMaps[common->combineIndex].indexParentNode) {
         if (index == 0) {
            localMaps[localMaps[currentMap].indexParentNode].parentOffset.x = correctedOffset[0];
            localMaps[localMaps[currentMap].indexParentNode].parentOffset.y = correctedOffset[1];
            localMaps[localMaps[currentMap].indexParentNode].parentOffset.w = correctedOffset[2];
         } 
         localMaps[localMaps[currentMap].indexParentNode].parentInfo[y][x] = a[y][x];
      } else {
         if (index == 0) {
            localMaps[common->combineIndex].parentOffset.x = correctedOffset[0];
            localMaps[common->combineIndex].parentOffset.y = correctedOffset[1];
            localMaps[common->combineIndex].parentOffset.w = correctedOffset[2];
         }
         localMaps[common->combineIndex].parentInfo[y][x] = a[y][x];
      }
   }

   if (index == 0) {
      localMaps[common->combineIndex].numPoints += localMaps[currentMap].numPoints;
      if (localMaps[common->combineIndex].numPoints > MAX_LOCAL_POINTS) {
         localMaps[common->combineIndex].numPoints = MAX_LOCAL_POINTS;
      }
      float2 newMapCentre = (float2) (common->currentOffset.x/2.0f,
            common->currentOffset.y/2.0f);
      localMaps[common->combineIndex].robotMapCentre = 
            (localMaps[common->combineIndex].robotMapCentre + newMapCentre) / 2.0f;

      if (common->combineMode == 0 && 
            common->loopConstraintI[common->numLoopConstraints - 1] != common->combineIndex) {
         //If making an initial combining, add constraint to the parent of currentMap to combineIndex
         int mIndex = common->numConstraints;
         common->numConstraints++;
         int loopIndex = common->numLoopConstraints;
         common->numLoopConstraints++;
         common->constraintType[mIndex] = 1;
         common->constraintIndex[mIndex] = loopIndex;
         common->loopConstraintI[loopIndex] = common->combineIndex;
         common->loopConstraintJ[loopIndex] = localMaps[currentMap].indexParentNode;
         common->loopConstraintParent[loopIndex] = common->loopConstraintParent[loopIndex - 1];
         common->loopConstraintInfo[loopIndex][0][0] = localMaps[currentMap].parentInfo[0][0];
         common->loopConstraintInfo[loopIndex][0][1] = localMaps[currentMap].parentInfo[0][1];
         common->loopConstraintInfo[loopIndex][0][2] = localMaps[currentMap].parentInfo[0][2];
         common->loopConstraintInfo[loopIndex][1][0] = localMaps[currentMap].parentInfo[1][0];
         common->loopConstraintInfo[loopIndex][1][1] = localMaps[currentMap].parentInfo[1][1];
         common->loopConstraintInfo[loopIndex][1][2] = localMaps[currentMap].parentInfo[1][2];
         common->loopConstraintInfo[loopIndex][2][0] = localMaps[currentMap].parentInfo[2][0];
         common->loopConstraintInfo[loopIndex][2][1] = localMaps[currentMap].parentInfo[2][1];
         common->loopConstraintInfo[loopIndex][2][2] = localMaps[currentMap].parentInfo[2][2];

         common->loopConstraintThetaDisp[loopIndex] = -(common->potentialMatchTheta[0] 
               + localMaps[currentMap].parentOffset.w);
         float cosTh = cos(-localMaps[currentMap].parentOffset.w);
         float sinTh = sin(-localMaps[currentMap].parentOffset.w);
         float2 temp;
         temp.x = (cosTh * -localMaps[currentMap].parentOffset.x - 
                  sinTh * -localMaps[currentMap].parentOffset.y);
         temp.y = (sinTh * -localMaps[currentMap].parentOffset.x + 
                  cosTh * -localMaps[currentMap].parentOffset.y);
         temp.x -= common->potentialMatchX[0];
         temp.y -= common->potentialMatchY[0];

         cosTh = cos(-common->potentialMatchTheta[0]);
         sinTh = sin(-common->potentialMatchTheta[0]);
         common->loopConstraintXDisp[loopIndex] = -(cosTh * -temp.x -
                                                  sinTh * -temp.y);
         common->loopConstraintYDisp[loopIndex] = -(sinTh * -temp.x +
                                                  cosTh * -temp.y);
         ANGNORM(common->loopConstraintThetaDisp[loopIndex]);
      }

      //Fix up the offset from the current node so the next node starts in the right place
      
      //convert to global slam coords
      float cosTh = cos(alignError);
      float sinTh = sin(alignError);
      float2 temp;
      temp.x = cosTh * offCurMap.x - sinTh * offCurMap.y;
      temp.y = sinTh * offCurMap.x + cosTh * offCurMap.y;

      //calculate the new offset wrt global slam coords
      cosTh = cos(localMaps[currentMap].currentGlobalPos.w);
      sinTh = sin(localMaps[currentMap].currentGlobalPos.w);
      float newOffX = (cosTh * -common->potentialMatchX[0] - sinTh * -common->potentialMatchY[0]);
      float newOffY = (sinTh * -common->potentialMatchX[0] + cosTh * -common->potentialMatchY[0]);
      newOffX += temp.x;
      newOffY += temp.y;

      //convert back to pos track coords and save answer in parent offset of curNode
      localMaps[currentMap].parentOffset = offCurMap;
      localMaps[currentMap].parentOffset.w = offCurMap.w - common->potentialMatchTheta[0];
      cosTh = cos(-alignError);
      sinTh = sin(-alignError);
      localMaps[currentMap].parentOffset.x = cosTh * newOffX - sinTh * newOffY;
      localMaps[currentMap].parentOffset.y = sinTh * newOffX + cosTh * newOffY;

      //make global pos of robot correction and save result in currentglobalpos of
      //current map
      localMaps[currentMap].currentGlobalPos = localMaps[common->combineIndex].currentGlobalPos;
      localMaps[currentMap].currentGlobalPos.w += localMaps[currentMap].parentOffset.w;
      localMaps[currentMap].currentGlobalPos.x += newOffX;
      localMaps[currentMap].currentGlobalPos.y += newOffY;


      common->combineMode = 1;
   }
}

__kernel void getGlobalMapPositions(constant slamConfig *config, global slamLocalMap *localMaps,
      global ocl_float *globalMapPositions, const int numLocalMaps) {
   int i = get_global_id(0);

   if (i < numLocalMaps) {
      localMaps[i].globalRobotMapCentre = convertToGlobalCoord(localMaps[i].robotMapCentre.x, 
            localMaps[i].robotMapCentre.y, localMaps[i].currentGlobalPos);
      //Write new global map positions to array for the snaps
      globalMapPositions[i*5] = localMaps[i].currentGlobalPos.x;
      globalMapPositions[i*5 + 1] = localMaps[i].currentGlobalPos.y;
      globalMapPositions[i*5 + 2] = localMaps[i].currentGlobalPos.w;
      globalMapPositions[i*5 + 3] = localMaps[i].globalRobotMapCentre.x;
      globalMapPositions[i*5 + 4] = localMaps[i].globalRobotMapCentre.y;
   }
}
      

__kernel void updateGlobalMap(constant slamConfig *config, global slamLocalMap *localMaps,
      global int *globalMap, const int numLocalMaps, global ocl_float *globalMapPositions, global float *globalMapHeights) {
   int index = get_global_id(0);
   int localIndex = get_local_id(0);
   int warpNum = localIndex / WARP_SIZE;
   int globalSize = get_global_size(0);
   int warpIndex = localIndex % WARP_SIZE;
   int groupSize = get_local_size(0);
   int groupNum = get_group_id(0);
   int globalWarp = (groupNum * groupSize + localIndex)/ WARP_SIZE;
   int numWarps = globalSize / WARP_SIZE;

   //Firstly, update the current globalCovar for each local map
   local float a[LOCAL_SIZE / WARP_SIZE][3][3];
   local float b[LOCAL_SIZE / WARP_SIZE][3][3];
   local float c[LOCAL_SIZE / WARP_SIZE][3][3];
   int i;

   int y = warpIndex / 3;
   int x = warpIndex % 3;

   local float max;
   for (i = globalWarp; i < numLocalMaps; i += numWarps) {
      if (i != 0) {
         if (warpIndex == 0) {
            float cosTh = cos(localMaps[i].currentGlobalPos.w);
            float sinTh = sin(localMaps[i].currentGlobalPos.w);

            //float parentAngle = localMaps[localMaps[i].indexParentNode]
            //                        .currentGlobalPos.w;
            //float cosTh = cos(parentAngle);
            //float sinTh = sin(parentAngle);
            a[warpNum][0][0] = cosTh;
            a[warpNum][1][1] = cosTh;
            a[warpNum][1][0] = sinTh;
            a[warpNum][0][1] = -sinTh;
            a[warpNum][2][2] = 1;
            a[warpNum][0][2] = 0;
            a[warpNum][1][2] = 0;
            a[warpNum][2][0] = 0;
            a[warpNum][2][1] = 0;
         }
         if (warpIndex < 9) {
            //b[warpNum][y][x] = localMaps[i].parentInfo[y][x];
            b[warpNum][y][x] = localMaps[i].internalCovar[y][x];
         }
         mult3x3MatrixLocal(a[warpNum], b[warpNum], c[warpNum], warpIndex);
         if (warpIndex == 0) {
            a[warpNum][1][0] *= -1;
            a[warpNum][0][1] *= -1;
         }
         mult3x3MatrixLocal(c[warpNum], a[warpNum], b[warpNum], warpIndex);
         //now invert it to get the covar matrix
         if (warpIndex < 9) {
            localMaps[i].globalCovar[y][x] = b[warpNum][y][x];
            //invert3x3Matrix(localMaps[i].globalCovar, a[warpNum], warpIndex);
            //localMaps[i].globalCovar[y][x] = a[warpNum][y][x];
            //Fiddle with the covar to make it nicer
            //covarFiddle(localMaps[i].globalCovar, warpIndex, config->MaxCovar, &max);
         }
      }
   }

   //Now update the global map
   int globalOffset = 0;
   for(i = 0; i < numLocalMaps; i++) {
      if (index < localMaps[i].numWarpPoints) {
         int globalIndex = convertToGlobalPosition(config, localMaps[i].warpPointsX[index], 
                           localMaps[i].warpPointsY[index], localMaps[i].currentGlobalPos);
         globalMap[globalOffset + index] = globalIndex;
         globalMapHeights[globalOffset + index] = localMaps[i].warpPointsZ[index];
      }
      globalOffset += localMaps[i].numWarpPoints;
   }
}

__kernel void resetOccupancyGrid(constant slamConfig *config, global slamCommon *common) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);

   int i;
   for (i = index; i < SIZE_LOCAL_OG; i += globalSize) {
      common->localOG[i] = -1;
   }
}

__kernel void setOccupancyGrid(constant slamConfig *config, global slamLocalMap *localMaps,
   global slamCommon *common, const int mapNum) {
   int index = get_global_id(0);

   if (index < localMaps[mapNum].numPoints) {
      int i = getLocalOGIndex(config, localMaps[mapNum].pointsX[index],
         localMaps[mapNum].pointsY[index]);
      if (i >= 0) {
         common->localOG[i] = index;
         common->localOGGradX[i] = localMaps[mapNum].gradX[index];
         common->localOGGradY[i] = localMaps[mapNum].gradY[index];
      }
   }
}

__kernel void updateWarpPoints(global slamLocalMap *localMaps,
   global float *points, const int mapIndex, const int numWarpPoints) {
   int index = get_global_id(0);

   if (index == 0) {
      localMaps[mapIndex].numWarpPoints = numWarpPoints;
   }
   if (index < numWarpPoints) {
      localMaps[mapIndex].warpPointsX[index] = points[index];
      localMaps[mapIndex].warpPointsY[index] = points[MAX_LOCAL_POINTS + index];
      localMaps[mapIndex].warpPointsZ[index] = points[MAX_LOCAL_POINTS * 2 + index];
   }
}

/*
 * kernel to clear the temporary data in slamCommon, and to initialise the new map
 *
 */
__kernel void createNewLocalMap(constant slamConfig *config, global slamLocalMap *localMaps,
      global slamCommon *common, global float *freeAreas, const int oldLocalMap, const int newLocalMap,
      const int parentLocalMap, const float angleError, const int numOldPoints) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);

   int i,j;
   int offset = SIZE_LOCAL_OG * newLocalMap;
   for (i = index; i < SIZE_LOCAL_OG; i += globalSize) {
      common->localOGZ[i] = 0;
      common->localOGCount[i] = 0;
      common->localOGX[i] = 0;   
      common->localOGY[i] = 0;   
      common->localOGGradX[i] = 0;   
      common->localOGGradY[i] = 0;
      common->localOG[i] = -1;
      freeAreas[offset + i] = INFINITY;   
   }

   for(i = index; i < NUM_ORIENTATION_BINS; i += globalSize) {
      localMaps[newLocalMap].orientationHist[i] = 0;
      localMaps[newLocalMap].entropyHist[i] = 0;
      for(j = 0; j < NUM_PROJECTION_BINS; j++) {
         localMaps[newLocalMap].projectionHist[i][j] = 0;
      }
   }
   if (index < 3) {
      localMaps[newLocalMap].parentInfo[index][0] = 0;
      localMaps[newLocalMap].parentInfo[index][1] = 0;
      localMaps[newLocalMap].parentInfo[index][2] = 0;
      localMaps[newLocalMap].internalCovar[index][0] = 0;
      localMaps[newLocalMap].internalCovar[index][1] = 0;
      localMaps[newLocalMap].internalCovar[index][2] = 0;
   }
   if (index == 0) {
      //Set the map centre for the old local map. This is normally done in the prepare
      //local map kernel, but it is not called for the first local map
      if (oldLocalMap == 0) {
         localMaps[0].mapCentre.x = (common->minMapRange.x + common->maxMapRange.x) / 2.0f;
         localMaps[0].mapCentre.y = (common->minMapRange.y + common->maxMapRange.y) / 2.0f;
         localMaps[0].robotMapCentre = (float2) (common->currentOffset.x / 2.0f,
            common->currentOffset.y / 2.0f);
      }
      localMaps[newLocalMap].numPoints = 0;
      localMaps[newLocalMap].isFeatureless = 0;
      localMaps[newLocalMap].indexParentNode = parentLocalMap;
      localMaps[newLocalMap].parentOffset = common->currentOffset;
      localMaps[newLocalMap].treeLevel = localMaps[parentLocalMap].treeLevel + 1;
      common->minMapRange.x = INFINITY;
      common->maxMapRange.x = 0;
      common->minMapRange.y = INFINITY;
      common->maxMapRange.y = 0;
      common->numPotentialMatches = 0;

      float2 temp = convertToGlobalCoord(common->currentOffset.x, common->currentOffset.y, 
            localMaps[parentLocalMap].currentGlobalPos);
      localMaps[newLocalMap].currentGlobalPos.x = temp.x;
      localMaps[newLocalMap].currentGlobalPos.y = temp.y;
      localMaps[newLocalMap].currentGlobalPos.z = localMaps[parentLocalMap].currentGlobalPos.z;
      localMaps[newLocalMap].currentGlobalPos.w = localMaps[parentLocalMap].currentGlobalPos.w +
         common->currentOffset.w;
      ANGNORM(localMaps[newLocalMap].currentGlobalPos.w);

      common->currentOffset.x = 0;
      common->currentOffset.y = 0;
      common->currentOffset.w = 0;
      

      //After the map has been optimised, the global coord system of the position tracking
      //is slightly different to the global coords for slam. parentOffset from the CPU
      //is relattive to the position tracker, but need it relative to slam
      /*float cosTh = cos(angleError);
      float sinTh = sin(angleError);
      float2 temp;
      temp.x = cosTh * parentOffset.x - sinTh * parentOffset.y;
      temp.y = sinTh * parentOffset.x + cosTh * parentOffset.y;
      parentOffset.x = temp.x;
      parentOffset.y = temp.y;
      localMaps[newLocalMap].currentGlobalPos = localMaps[parentLocalMap].currentGlobalPos
         + parentOffset;
      ANGNORM(localMaps[newLocalMap].currentGlobalPos.w);*/

      //Make the offset relative to the parent instead of the global coord system
      /*cosTh = cos(-localMaps[parentLocalMap].currentGlobalPos.w);
      sinTh = sin(-localMaps[parentLocalMap].currentGlobalPos.w);
      localMaps[newLocalMap].parentOffset.x = cosTh * parentOffset.x - sinTh * parentOffset.y;
      localMaps[newLocalMap].parentOffset.y = sinTh * parentOffset.x + cosTh * parentOffset.y;*/

      /*common->currentOffset -= angleError;
      localMaps[newLocalMap].parentOffset = common->currentOffset;
      float2 temp = convertToGlobalCoord(common->currentOffset.x, common->currentOffset.y, 
            localMaps[parentLocalMap].currentGlobalPos);
      localMaps[newLocalMap].currentGlobalPos = (float4) (temp.x, temp.y, 
            localMaps[parentLocalMap].currentGlobalPos.z, 
            localMaps[parentLocalMap].currentGlobalPos.w + common->currentOffset.w);
      common->currentOffset = startingOffset;*/

   }
}



