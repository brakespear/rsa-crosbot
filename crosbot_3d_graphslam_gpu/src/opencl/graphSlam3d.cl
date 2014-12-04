
#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

__kernel void clearLocalMap(constant oclGraphSlam3DConfig *config, 
      global int *blocks, global oclLocalBlock *localMapCells, 
      global oclLocalMapCommon *common, const int numBlocksLast) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);

   for (int i = index; i < config->NumBlocksTotal; i+= globalSize) {
      blocks[i] = -1;
   }
   for (int i = index; i < numBlocksLast * config->NumCellsTotal; i+= globalSize) {
      int blockIndex = i / config->NumCellsTotal;
      int cellIndex = i % config->NumCellsTotal;
      localMapCells[blockIndex].distance[cellIndex] = NAN;
      localMapCells[blockIndex].weight[cellIndex] = 0;
      localMapCells[blockIndex].r[cellIndex] = 0;
      localMapCells[blockIndex].g[cellIndex] = 0;
      localMapCells[blockIndex].b[cellIndex] = 0;
      localMapCells[blockIndex].occupied[cellIndex] = 0;
      localMapCells[blockIndex].pI[cellIndex] = -1;
   }

   if (index == 0) {
      common->numBlocks = 0;
      common->numActiveBlocks = 0;
      common->numPoints = 0;
   }
}

/*
 * Transforms a point given the rotation matrix and origin
 */
float3 transformPoint(float3 point, const float3 origin, const float3 rotation0,
   const float3 rotation1, const float3 rotation2) {

   float3 transP;
   float3 temp = point * rotation0;
   transP.x = temp.x + temp.y + temp.z + origin.x;
   temp = point * rotation1;
   transP.y = temp.x + temp.y + temp.z + origin.y;
   temp = point * rotation2;
   transP.z = temp.x + temp.y + temp.z + origin.z;
   return transP;
}

/*
 * Finds the block of a local map that the point is found in
 * Returns the index
 */
int getBlockIndex(constant oclGraphSlam3DConfig *config, float3 point) {
   float off = (config->BlockSize * config->NumBlocksWidth) / 2.0f;
   float offZ = (config->BlockSize * config->NumBlocksHeight) / 2.0f;

   int x = (point.x + off) / config->BlockSize;
   int y = (point.y + off) / config->BlockSize;
   int z = (point.z + offZ) / config->BlockSize;

   if (x >= 0 && x < config->NumBlocksWidth && y >= 0 && y < config->NumBlocksWidth
         && z >= 0 && z < config->NumBlocksHeight) {
      return z * config->NumBlocksWidth * config->NumBlocksWidth +
         y * config->NumBlocksWidth + x;
   } else {
      return -1;
   }
}

/*
 * Gets the coordinates of the lowest corner of the block
 */
float3 getBlockPosition(constant oclGraphSlam3DConfig *config, int index) {
   int x = index % config->NumBlocksWidth;
   int y = (index / config->NumBlocksWidth) % config->NumBlocksWidth;
   int z = index / (config->NumBlocksWidth * config->NumBlocksWidth);
   float off = (config->BlockSize * config->NumBlocksWidth) / 2.0f;
   float offZ = (config->BlockSize * config->NumBlocksHeight) / 2.0f;
   
   float3 p;
   p.x = (x * config->BlockSize) - off;
   p.y = (y * config->BlockSize) - off;
   p.z = (z * config->BlockSize) - offZ;
   return p;   
}

/*
 * Gets the index of the adjacent block in z.
 */ 
int getBlockAdjZ(constant oclGraphSlam3DConfig *config, int index, int dir) {
   if (index < 0) return -1;
   int zVal = index / (config->NumBlocksWidth * config->NumBlocksWidth);
   if (dir < 0 && zVal > 0) {
      return index - (config->NumBlocksWidth * config->NumBlocksWidth);
   } else if (dir > 0 && zVal < config->NumBlocksHeight - 1) {
      return index + (config->NumBlocksWidth * config->NumBlocksWidth);
   } else {
      return -1;
   }
}
/*
 * Gets the index of the adjacent block in x.
 */ 
int getBlockAdjX(constant oclGraphSlam3DConfig *config, int index, int dir) {
   if (index < 0) return -1;
   int xVal = index % config->NumBlocksWidth;
   if (dir < 0 && xVal > 0) {
      return index - 1;
   } else if (dir > 0 && xVal < config->NumBlocksWidth - 1) {
      return index + 1;
   } else {
      return -1;
   }
}
/*
 * Gets the index of the adjacent block in y.
 */ 
int getBlockAdjY(constant oclGraphSlam3DConfig *config, int index, int dir) {
   if (index < 0) return -1;
   int yVal = (index / config->NumBlocksWidth) % config->NumBlocksWidth;
   if (dir < 0 && yVal > 0) {
      return index - config->NumBlocksWidth;
   } else if (dir > 0 && yVal < config->NumBlocksWidth - 1) {
      return index + config->NumBlocksWidth;
   } else {
      return -1;
   }
}

/*
 * Returns the point at the centre of the cell, given the index of the cell
 * in the block (cIndex), and the index of the block (bIndex)
 */
float3 getCellCentre(constant oclGraphSlam3DConfig *config, int cIndex, int bIndex) {
   float3 p = getBlockPosition(config, bIndex);

   int x = cIndex % config->NumCellsWidth;
   int y = (cIndex / config->NumCellsWidth) % config->NumCellsWidth;
   int z = cIndex / (config->NumCellsWidth * config->NumCellsWidth);

   p.x += x * config->CellSize + (config->CellSize / 2.0f);
   p.y += y * config->CellSize + (config->CellSize / 2.0f);
   p.z += z * config->CellSize + (config->CellSize / 2.0f);
   return p;
}

int getCellIndex(constant oclGraphSlam3DConfig *config, float3 point, int bIndex) {
   float3 b = getBlockPosition(config, bIndex);
   float3 diff = point - b;
   int x = diff.x / config->CellSize;
   int y = diff.y / config->CellSize;
   int z = diff.z / config->CellSize;
   return z * config->NumCellsWidth * config->NumCellsWidth + y * config->NumCellsWidth + x;
}

void markBlockActive(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalMapCommon *common, int bIndex, int isOrig, int cIndex,
      global oclLocalBlock *localMapCells) {
   if (bIndex >= 0) {
      //Deal with the block
      if (blocks[bIndex] == -1) {
         //The block doesn't exist
         int oldVal = atomic_dec(&(blocks[bIndex]));
         if (oldVal == -1) {
            //First thread this time so set as active block
            int aIndex = atomic_inc(&(common->numActiveBlocks));
            if (aIndex < MAX_NUM_ACTIVE_BLOCKS) {
               common->activeBlocks[aIndex] = bIndex;
            } else {
               atomic_xchg(&(blocks[bIndex]), -1);
            } 
         }
      } else if (blocks[bIndex] >= 0 && blocks[bIndex] < config->NumBlocksAllocated) {
         //The block already exists
         int oldVal = atomic_add(&(blocks[bIndex]), config->NumBlocksAllocated);
         if (oldVal < config->NumBlocksAllocated) {
            //First thread this time so set as active block
            int aIndex = atomic_inc(&(common->numActiveBlocks));
            if (aIndex < MAX_NUM_ACTIVE_BLOCKS) {
               common->activeBlocks[aIndex] = bIndex;
            } else {
               atomic_xchg(&(blocks[bIndex]), oldVal % config->NumBlocksAllocated);
            } 
         }
      } 
      if (blocks[bIndex] >= 0 && isOrig && config->UseOccupancyForSurface) {
         int blockI = blocks[bIndex] % config->NumBlocksAllocated;
         if (localMapCells[blockI].occupied[cIndex] == 0) {
            //Note: concurrency doesn't really matter here - as long as one
            //add works
            localMapCells[blockI].occupied[cIndex]++;
         }
      }
   }
}

/*
 * Marks blocks near points in the scan as active. transform coords given
 * transform the point from the sensor frame into the local map frame
 */
__kernel void checkBlocksExist(constant oclGraphSlam3DConfig *config,
      global int *blocks, global oclLocalMapCommon *common,
      global oclDepthPoints *points, global oclLocalBlock *localMapCells,
      const int numPoints, const float3 origin,
      const float3 rotation0, const float3 rotation1, const float3 rotation2) {

   int index = get_global_id(0);

   if (index < numPoints && !isnan(points->pointX[index])) {
      float3 point = (float3)(points->pointX[index], points->pointY[index], points->pointZ[index]);
      float3 transP = transformPoint(point, origin, rotation0, rotation1, rotation2);

      /*points->pointX[index] = transP.x;
      points->pointY[index] = transP.y;
      points->pointZ[index] = transP.z;*/

      int bIndex = getBlockIndex(config, transP);
      if (bIndex >= 0) {
         int cIndex = getCellIndex(config, transP, bIndex);
         markBlockActive(config, blocks, common, bIndex, 1, cIndex, localMapCells);
         
         //todo: could find needed adjacent points by looking at the intersection of the ray with the block

         float truncEst = config->TruncPos;
         float3 blockPos = getBlockPosition(config, bIndex);
         /*if (transP.x - blockPos.x < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjX(config, bIndex, -1));
         } 
         if (blockPos.x + config->BlockSize - transP.x < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjX(config, bIndex, 1));
         }
         if (transP.y - blockPos.y < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjY(config, bIndex, -1));
         } 
         if (blockPos.y + config->BlockSize - transP.y < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjY(config, bIndex, 1));
         }
         if (transP.z - blockPos.z < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjZ(config, bIndex, -1));
         } 
         if (blockPos.z + config->BlockSize - transP.z < truncEst) {
            markBlockActive(config, blocks, common, getBlockAdjZ(config, bIndex, 1));
         }*/

         int adjXP = getBlockAdjX(config, bIndex, 1);
         markBlockActive(config, blocks, common, adjXP, 0, cIndex, localMapCells);
         int adjXN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjXN, 0, cIndex, localMapCells);
         int adjYP = getBlockAdjX(config, bIndex, 1);
         markBlockActive(config, blocks, common, adjYP, 0, cIndex, localMapCells);
         int adjYN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjYN, 0, cIndex, localMapCells);
         int adjZP = getBlockAdjX(config, bIndex, 1);
         markBlockActive(config, blocks, common, adjZP, 0, cIndex, localMapCells);
         int adjZN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjZN, 0, cIndex, localMapCells);
         adjXP = getBlockAdjX(config, adjXP, 1);
         markBlockActive(config, blocks, common, adjXP, 0, cIndex, localMapCells);
         adjXN = getBlockAdjX(config, adjXN, -1);
         markBlockActive(config, blocks, common, adjXN, 0, cIndex, localMapCells);
         adjYP = getBlockAdjX(config, adjYP, 1);
         markBlockActive(config, blocks, common, adjYP, 0, cIndex, localMapCells);
         adjYN = getBlockAdjX(config, adjYN, -1);
         markBlockActive(config, blocks, common, adjYN, 0, cIndex, localMapCells);
         adjZP = getBlockAdjX(config, adjZP, 1);
         markBlockActive(config, blocks, common, adjZP, 0, cIndex, localMapCells);
         adjZN = getBlockAdjX(config, adjZN, -1);
         markBlockActive(config, blocks, common, adjZN, 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjXP, 1), 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjXN, -1), 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYP, 1), 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYN, -1), 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZP, 1), 0, cIndex, localMapCells);
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZN, -1), 0, cIndex, localMapCells);
         /*markBlockActive(config, blocks, common, getBlockAdjX(config, adjZP, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZP, -1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZN, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZN, -1));
         markBlockActive(config, blocks, common, getBlockAdjY(config, adjZP, 1));
         markBlockActive(config, blocks, common, getBlockAdjY(config, adjZP, -1));
         markBlockActive(config, blocks, common, getBlockAdjY(config, adjZN, 1));
         markBlockActive(config, blocks, common, getBlockAdjY(config, adjZN, -1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYP, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYP, -1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYN, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYN, -1));*/

      }
   }
}

__kernel void addRequiredBlocks(constant oclGraphSlam3DConfig *config,
      global int *blocks, global oclLocalBlock *localMapCells,
      global oclLocalMapCommon *common) {

   int index = get_global_id(0);

   //if (index == 0) {
   //common->numPoints = 0;
   //}

   if (index < common->numActiveBlocks && index < MAX_NUM_ACTIVE_BLOCKS) {
      int bIndex = common->activeBlocks[index];
      int cellIndex;
      if (blocks[bIndex] < 0) {
         //Create a new block
         cellIndex = atomic_inc(&(common->numBlocks));
         if (cellIndex < config->NumBlocksAllocated) {
            blocks[bIndex] = cellIndex;
            localMapCells[cellIndex].blockIndex = bIndex;
         } else {
            atomic_dec(&(common->numBlocks));
            blocks[bIndex] = -1;
            cellIndex = -1;
         }
      } else {
         cellIndex = blocks[bIndex] % config->NumBlocksAllocated;
         blocks[bIndex] = cellIndex;
      }
      common->activeBlocks[index] = cellIndex;
   }
}

/*
 * Adds a frame to the graph. The transform given by origin and rotations is the
 * transform from the local map frame to the sensor frame. Local map pose is the
 * pose of the sensor in local map coords
 */
__kernel void addFrame(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common, 
      global oclDepthPoints *points, const int numActiveBlocks,
      const float3 origin, const float3 rotation0, 
      const float3 rotation1, const float3 rotation2,
      const float3 localMapPose) {

   int gIndex = get_global_id(0);
   int lIndex = get_local_id(0);
   int groupId = get_group_id(0);
   if (gIndex == 0) {
      common->numActiveBlocks = 0;
   }

   int aIndex, startIndex;
   if (config->NumCellsTotal >= LOCAL_SIZE) {
      aIndex = groupId;
      startIndex = lIndex;
   } else {
      int numBlocksPerGroup = LOCAL_SIZE / config->NumCellsTotal;
      int blockNumInGroup = lIndex / config->NumCellsTotal;
      aIndex = groupId * numBlocksPerGroup + blockNumInGroup;
      if (blockNumInGroup < numBlocksPerGroup) {
         //startIndex = lIndex - (blockNumInGroup * config->NumCellsTotal);
         startIndex = lIndex % config->NumCellsTotal;
      } else {
         //cells don't divide evenly into group, and this thread is extra
         startIndex = config->NumCellsTotal;
      }
   }
   if (aIndex >= numActiveBlocks || common->activeBlocks[aIndex] < 0) {
      return;
   }

   for (int cIndex = startIndex; cIndex < config->NumCellsTotal; cIndex += LOCAL_SIZE) {
      int blockI = common->activeBlocks[aIndex];

      float3 cellCentre = getCellCentre(config, cIndex, localMapCells[blockI].blockIndex);
      //Convert point into camera frame
      float3 cameraFramePoint = transformPoint(cellCentre, origin, rotation0, rotation1, rotation2);
      //Find pixel coordinates of point
      int u = (config->fx * cameraFramePoint.x + config->tx) / cameraFramePoint.z + config->cx;
      int v = (config->fy * cameraFramePoint.y + config->ty) / cameraFramePoint.z + config->cy;
      if (u >= 0 && u < config->ScanWidth && v >= 0 && v < config->ScanHeight) {
         int pointI = config->ScanWidth * v + u;
         float3 point = (float3)(points->pointX[pointI], points->pointY[pointI], points->pointZ[pointI]);
         if (isnan(point.x)) {
            continue;
         }
         float distCell = fast_distance(cellCentre, localMapPose);
         if (distCell > config->MaxDistance) {
            continue;
         }
         float distPoint = fast_length(point);

         float tsdfVal = distPoint - distCell;
         float weightVal = 1.0f/* / distPoint*/;

         if ((tsdfVal >= 0 /*&& tsdfVal < config->TruncPos*/) ||
               (tsdfVal < 0 && tsdfVal * -1.0f < config->TruncNeg)) {
            if (tsdfVal >= config->TruncPos) {
               tsdfVal = config->TruncPos;
            }
         //atomic_inc(&(common->numPoints));
            if (isnan(localMapCells[blockI].distance[cIndex])) {
               localMapCells[blockI].distance[cIndex] = tsdfVal;
               localMapCells[blockI].weight[cIndex] = weightVal;
               localMapCells[blockI].r[cIndex] = points->r[pointI];
               localMapCells[blockI].g[cIndex] = points->g[pointI];
               localMapCells[blockI].b[cIndex] = points->b[pointI];
            } else {
               float weightPrev = localMapCells[blockI].weight[cIndex];
               localMapCells[blockI].distance[cIndex] = (localMapCells[blockI].distance[cIndex] * weightPrev +
                     tsdfVal * weightVal) / (weightPrev + weightVal);
               /*localMapCells[blockI].r[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].r[cIndex] +
                     weightVal * (float)points->r[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].g[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].g[cIndex] +
                     weightVal * (float)points->g[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].b[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].b[cIndex] +
                     weightVal * (float)points->b[pointI]) / (weightVal + weightPrev));*/
               localMapCells[blockI].r[cIndex] = points->r[pointI];
               localMapCells[blockI].g[cIndex] = points->g[pointI];
               localMapCells[blockI].b[cIndex] = points->b[pointI];
               localMapCells[blockI].weight[cIndex] += weightVal;
            } 
         }
      }
   }
}

/*float getOneCellValue(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, float3 pos, int3 blockI, int *count, float change) {
   if (pos.x < 0) {
      pos.x = config->BlockSize - change;
      blockI.x--;
   } else if (pos.x >= config->BlockSize) {
      pos.x = change;
      blockI.x++;
   }
   if (pos.y < 0) {
      pos.y = config->BlockSize - change;
      blockI.y--;
   } else if (pos.y >= config->BlockSize) {
      pos.y = change;
      blockI.y++;
   }
   if (pos.z < 0) {
      pos.z = config->BlockSize - change;
      blockI.z--;
   } else if (pos.z >= config->BlockSize) {
      pos.z = change;
      blockI.z++;
   }
   if (blockI.x >= 0 && blockI.x < config->NumBlocksWidth && blockI.y >= 0 &&
         blockI.y < config->NumBlocksWidth && blockI.z >= 0 && blockI.z < config->NumBlocksHeight) {

      int bIndex = blockI.z * config->NumBlocksWidth * config->NumBlocksWidth +
         blockI.y * config->NumBlocksWidth + blockI.x;
      int bI = blocks[bIndex];
      //2nd part test
      //if (bI >= 0 && bI < 1000) {
      if (bI >= 0) {
         int cZ = pos.z / config->CellSize;
         int cY = pos.y / config->CellSize;
         int cX = pos.x / config->CellSize;
         int cellI = cZ * config->NumCellsWidth * config->NumCellsWidth + cY * config->NumCellsWidth
            + cX;

         if (!isnan(localMapCells[bI].distance[cellI])) {
            *count += 1;
            return localMapCells[bI].distance[cellI];
         }
      }
   }
   return 0;
}

//blockCur are the x,y,z indexes of the main block, curPos in the pos in the block
float getCellValue(constant oclGraphSlam3DConfig *config, global int *blocks, global oclLocalBlock *localMapCells,
      float3 curPos, int3 blockCur) {
   float change = config->CellSize / 2.0f;
   int count = 0;
   float value = 0.0f;
   
   float3 pos = curPos + (float3)(-change, -change, -change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);
   
   pos = curPos + (float3)(-change, -change, change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(-change, change, -change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(-change, change, change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(change, -change, -change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(change, -change, change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(change, change, -change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   pos = curPos + (float3)(change, change, change);
   value += getOneCellValue(config, blocks, localMapCells, pos, blockCur, &count, change);

   if (count == 0) {
      return NAN;
   } else {
      return value / (float)count;
   }
}

//bI is index into blocks array
void checkDirection(constant oclGraphSlam3DConfig *config, global oclLocalBlock *localMapCells,
      global int *blocks, local float *x, local float *y, local float *z, local unsigned char *r, 
      local unsigned char *g, local unsigned char *b, local short *fullIndex,
      local int *blockCount, int bIndex, int bLocalIndex, int blockOffset,
      float3 start, float incXV, float incYV, float incZV) {

   int bI = localMapCells[bIndex].blockIndex;
   int3 blockCur;
   blockCur.z = bI / (config->NumBlocksWidth * config->NumBlocksWidth);
   blockCur.y = (bI / config->NumBlocksWidth) % config->NumBlocksWidth;
   blockCur.x = bI % config->NumBlocksWidth; 
   float3 inc = (float3)(incXV, incYV, incZV);

   float cellVal = getCellValue(config, blocks, localMapCells, start, blockCur);
   float nextCellVal;
   for (int i = 1; i < config->NumCellsWidth; i++) {
      nextCellVal = getCellValue(config, blocks, localMapCells, start + inc, blockCur);
      if (!isnan(cellVal) && !isnan(nextCellVal) && sign(cellVal) != sign(nextCellVal)) {
         //There is a crossing!
         float3 crossing = start + fabs(cellVal / (nextCellVal - cellVal)) * inc;
         float3 p = getBlockPosition(config, bI);
         int mid = config->CellSize / 2.0f;
         int xC = (start.x + mid) / config->CellSize;
         int yC = (start.y + mid) / config->CellSize;
         int zC = (start.z + mid) / config->CellSize;
         int cellI = zC * config->NumCellsWidth * config->NumCellsWidth + yC * config->NumCellsWidth
            + xC;


         if (!isnan(localMapCells[bIndex].distance[cellI]) && localMapCells[bIndex].pI[cellI] >= 0) {
            //Cell has been marked before, so average
            int retI = localMapCells[bIndex].pI[cellI];

            x[blockOffset + retI] = (x[blockOffset + retI] + crossing.x + p.x) / 2.0f;
            y[blockOffset + retI] = (y[blockOffset + retI] + crossing.y + p.y) / 2.0f;
            z[blockOffset + retI] = (z[blockOffset + retI] + crossing.z + p.z) / 2.0f;

            r[blockOffset + retI] = (unsigned char)(((int)r[blockOffset + retI] + 
                  (int)localMapCells[bIndex].r[cellI]) / 2);
            g[blockOffset + retI] = (unsigned char)(((int)g[blockOffset + retI] + 
                  (int)localMapCells[bIndex].g[cellI]) / 2);
            b[blockOffset + retI] = (unsigned char)(((int)b[blockOffset + retI] + 
                  (int)localMapCells[bIndex].b[cellI]) / 2);
         } else if (!isnan(localMapCells[bIndex].distance[cellI])) {
            int retI = atomic_inc(&(blockCount[bLocalIndex]));
            if (retI < MAX_POINTS_GROUP / BLOCKS_PER_GROUP) {
               x[blockOffset + retI] = crossing.x + p.x;
               y[blockOffset + retI] = crossing.y + p.y;
               z[blockOffset + retI] = crossing.z + p.z;
               r[blockOffset + retI] = localMapCells[bIndex].r[cellI];
               g[blockOffset + retI] = localMapCells[bIndex].g[cellI];
               b[blockOffset + retI] = localMapCells[bIndex].b[cellI];
               localMapCells[bIndex].pI[cellI] = retI;
               fullIndex[blockOffset + retI] = cellI;
            }
         }
      }
      cellVal = nextCellVal;
      start += inc;
   }
}*/


void checkDirection(constant oclGraphSlam3DConfig *config, global oclLocalBlock *localMapCells,
      local float *x, local float *y, local float *z, local unsigned char *r, 
      local unsigned char *g, local unsigned char *b, local short *fullIndex,
      local int *blockCount, int bIndex, int bLocalIndex, int blockOffset,
      int startI, int increment, int bNextIndex, float incXV, float incYV, float incZV) {

   float cellVal, cellNextVal;
   int nextI;
   int count = 0;
   for (int i = startI; count < config->NumCellsWidth; i += increment, count++) {
      nextI = i + increment;
      cellVal = localMapCells[bIndex].distance[i];
      int bni = 0;
      int ni = 0;
      if (count + 1 < config->NumCellsWidth) {
         cellNextVal = localMapCells[bIndex].distance[nextI];
         bni = bIndex;
         ni = nextI;
      } else if (bNextIndex < 0 || bNextIndex > config->NumBlocksAllocated) {
         continue;
      } else {
      //if (incYV > 0.5) { continue; }
      //continue;
         cellNextVal = localMapCells[bNextIndex].distance[startI];
         bni = bNextIndex;
         ni = startI;
      }
      if (isnan(cellVal) || isnan(cellNextVal)) {
         continue;
      }
      if (fabs(cellVal) > config->CellSize * 10 || fabs(cellNextVal) > config->CellSize * 10) {
         continue;
      }
      if ((sign(cellVal) != sign(cellNextVal) || cellVal == 0) && 
            localMapCells[bIndex].weight[i] > 5.0f && (config->UseOccupancyForSurface == 0 || 
            localMapCells[bIndex].occupied[i] > 0 || localMapCells[bni].occupied[ni] > 0)) {
         //There is a crossing!
         float3 p = getCellCentre(config, i, localMapCells[bIndex].blockIndex);
         float inc = fabs(cellVal / (cellNextVal - cellVal)) * config->CellSize;
         
         if (inc >= config->CellSize) {
            inc = config->CellSize;
         }
         float incX = inc * incXV;
         float incY = inc * incYV;
         float incZ = inc * incZV;
         if ((inc < config->CellSize/2.0f && localMapCells[bIndex].pI[i] >= 0) ||
               (inc >= config->CellSize/2.0f && count + 1 < config->NumCellsWidth && 
                localMapCells[bIndex].pI[nextI] >= 0)) {
            //Merge with existing
            int retI, colI;
            if (inc < config->CellSize/2.0f) {
               retI = localMapCells[bIndex].pI[i];
               colI = i;
            } else {
               retI = localMapCells[bIndex].pI[nextI];
               colI = nextI;
            }
            x[blockOffset + retI] = (x[blockOffset + retI] + p.x + incX) / 2.0f;
            y[blockOffset + retI] = (y[blockOffset + retI] + p.y + incY) / 2.0f;
            z[blockOffset + retI] = (z[blockOffset + retI] + p.z + incZ) / 2.0f;

            r[blockOffset + retI] = (unsigned char)(((int)r[blockOffset + retI] + 
                  (int)localMapCells[bIndex].r[colI]) / 2);
            g[blockOffset + retI] = (unsigned char)(((int)g[blockOffset + retI] + 
                  (int)localMapCells[bIndex].g[colI]) / 2);
            b[blockOffset + retI] = (unsigned char)(((int)b[blockOffset + retI] + 
                  (int)localMapCells[bIndex].b[colI]) / 2);
         } else {
            int retI = atomic_inc(&(blockCount[bLocalIndex]));
            if (retI >= MAX_POINTS_GROUP / BLOCKS_PER_GROUP) {
               continue;
            }
            x[blockOffset + retI] = p.x + incX;
            y[blockOffset + retI] = p.y + incY;
            z[blockOffset + retI] = p.z + incZ;
            if (inc < config->CellSize / 2.0f) {
               r[blockOffset + retI] = localMapCells[bIndex].r[i];
               g[blockOffset + retI] = localMapCells[bIndex].g[i];
               b[blockOffset + retI] = localMapCells[bIndex].b[i];
               localMapCells[bIndex].pI[i] = retI;
               fullIndex[blockOffset + retI] = i;
            } else if (count + 1 < config->NumCellsWidth) {
               r[blockOffset + retI] = localMapCells[bIndex].r[nextI];
               g[blockOffset + retI] = localMapCells[bIndex].g[nextI];
               b[blockOffset + retI] = localMapCells[bIndex].b[nextI];
               localMapCells[bIndex].pI[nextI] = retI;
               fullIndex[blockOffset + retI] = nextI;
            } else {
               r[blockOffset + retI] = localMapCells[bNextIndex].r[startI];
               g[blockOffset + retI] = localMapCells[bNextIndex].g[startI];
               b[blockOffset + retI] = localMapCells[bNextIndex].b[startI];
               
               //localMapCells[bIndex].pI[i] = retI;
               //fullIndex[blockOffset + retI] = i;
            }
         }
      }
   }
}

float getPoint(constant oclGraphSlam3DConfig *config, global int *blocks, 
      global oclLocalBlock *localMapCells, float3 p) {
   int blockI = getBlockIndex(config, p);
   int cellI = getCellIndex(config, p, blockI);

   if (blockI >= 0 && cellI >= 0 && blocks[blockI] >= 0) {
      return localMapCells[blocks[blockI]].distance[cellI];
   } else {
      return NAN;
   }

}

float3 getNormal(constant oclGraphSlam3DConfig *config, global int *blocks, 
      global oclLocalBlock *localMapCells, float x, float y, float z) {

   float change = config->CellSize / 2.0f + 0.001f;
   float3 p = (float3)(x,y,z);

   float3 pTemp = p;
   pTemp.x += change;
   float xH = getPoint(config, blocks, localMapCells, pTemp);
   pTemp = p;
   pTemp.x -= change;
   float xL = getPoint(config, blocks, localMapCells, pTemp);
   pTemp = p;
   pTemp.y += change;
   float yH = getPoint(config, blocks, localMapCells, pTemp);
   pTemp = p;
   pTemp.y -= change;
   float yL = getPoint(config, blocks, localMapCells, pTemp);
   pTemp = p;
   pTemp.z += change;
   float zH = getPoint(config, blocks, localMapCells, pTemp);
   pTemp = p;
   pTemp.z -= change;
   float zL = getPoint(config, blocks, localMapCells, pTemp);
   if (isnan(xH) || isnan(xL) || isnan(yH) || isnan(yL) ||
         isnan(zH) || isnan(zL)) {
      return (float3)(NAN,NAN,NAN);
   } else {
      float3 normal = (float3)(xH - xL, yH - yL, zH - zL);
      return normalize(normal);
   }
}

__kernel void extractPoints(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common,
      global float *ps, global unsigned char *cols, global float *normals) {
   
   int gIndex = get_global_id(0);
   int lIndex = get_local_id(0);
   int groupId = get_group_id(0);

   local float x[MAX_POINTS_GROUP];
   local float y[MAX_POINTS_GROUP];
   local float z[MAX_POINTS_GROUP];
   local unsigned char r[MAX_POINTS_GROUP];
   local unsigned char g[MAX_POINTS_GROUP];
   local unsigned char b[MAX_POINTS_GROUP];
   local short fullIndex[MAX_POINTS_GROUP];
   local int blockCount[BLOCKS_PER_GROUP];
   local int blockOffsetComp[BLOCKS_PER_GROUP];
   local int startGIndex;

   int sliceSize = config->NumCellsWidth * config->NumCellsWidth;
   int bLocalIndex = lIndex / sliceSize;
   int bIndex = BLOCKS_PER_GROUP * groupId + bLocalIndex;
   int cIndex = lIndex % sliceSize;
   int blockOffset = (MAX_POINTS_GROUP / BLOCKS_PER_GROUP) * bLocalIndex;

   if (cIndex == 0 && bLocalIndex < BLOCKS_PER_GROUP) {
      blockCount[bLocalIndex] = 1;
      x[blockOffset] = NAN;
      y[blockOffset] = NAN;
      z[blockOffset] = NAN;
      r[blockOffset] = 0;
      g[blockOffset] = 0;
      b[blockOffset] = 0;
      if (bIndex >= common->numBlocks) {
         blockCount[bLocalIndex] = 0;
      }
   }
   for (int i = lIndex; i < MAX_POINTS_GROUP; i+= LOCAL_SIZE) {
      fullIndex[i] = -1;
   }
   barrier(CLK_LOCAL_MEM_FENCE);

   /*float3 start;
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      int xx = cIndex % config->NumCellsWidth;
      int yy = cIndex / config->NumCellsWidth;
      start.x = (float)xx * config->CellSize;
      start.y = (float)yy * config->CellSize;
      start.z = 0.0f;
      checkDirection(config, localMapCells, blocks, x, y, z, r, g, b, fullIndex, blockCount,
            bIndex, bLocalIndex, blockOffset, start, 0.0f, 0.0f, config->CellSize);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      int yy = cIndex % config->NumCellsWidth;
      int zz = cIndex / config->NumCellsWidth;
      start.y = (float)yy * config->CellSize;
      start.z = (float)zz * config->CellSize;
      start.x = 0.0f;
      checkDirection(config, localMapCells, blocks, x, y, z, r, g, b, fullIndex, blockCount,
            bIndex, bLocalIndex, blockOffset, start, config->CellSize, 0.0f, 0.0f);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      int xx = cIndex % config->NumCellsWidth;
      int zz = cIndex / config->NumCellsWidth;
      start.x = (float)xx * config->CellSize;
      start.z = (float)zz * config->CellSize;
      start.y = 0.0f;
      checkDirection(config, localMapCells, blocks, x, y, z, r, g, b, fullIndex, blockCount,
            bIndex, bLocalIndex, blockOffset, start, 0.0f, config->CellSize, 0.0f);
   }*/
   

   int startI, bNextIndex, increment;
   //int nextI;
   //float cellVal, cellNextVal;
   
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      startI = cIndex;
      bNextIndex = getBlockAdjZ(config, localMapCells[bIndex].blockIndex, 1);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = config->NumCellsWidth * config->NumCellsWidth;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, 0, 0, 1);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      int zz = cIndex / config->NumCellsWidth;
      int yy = cIndex % config->NumCellsWidth;
      startI = zz * config->NumCellsWidth * config->NumCellsWidth + yy * config->NumCellsWidth;
      bNextIndex = getBlockAdjX(config, localMapCells[bIndex].blockIndex, 1);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = 1;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, 1, 0, 0);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndex < common->numBlocks) {
      int xx = cIndex % config->NumCellsWidth;
      int zz = cIndex / config->NumCellsWidth;
      startI = zz * config->NumCellsWidth * config->NumCellsWidth + xx;
      bNextIndex = getBlockAdjY(config, localMapCells[bIndex].blockIndex, 1);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = config->NumCellsWidth;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, 0, 1, 0);
   }

   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex == 0) {
      int totalPoints = 0;
      for (int i = 0; i < BLOCKS_PER_GROUP; i++) {
         if (blockCount[i] > MAX_POINTS_GROUP / BLOCKS_PER_GROUP) {
            blockCount[i] = MAX_POINTS_GROUP / BLOCKS_PER_GROUP;
         }
         blockOffsetComp[i] = totalPoints;
         totalPoints += blockCount[i];
      }
      startGIndex = atomic_add(&(common->numPoints), totalPoints);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   for (int i = cIndex; bLocalIndex < BLOCKS_PER_GROUP && i < blockCount[bLocalIndex]; i+= sliceSize) {
       ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = x[blockOffset + i];
       ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = y[blockOffset + i];
       ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = z[blockOffset + i];
       cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = r[blockOffset + i];
       cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = g[blockOffset + i];
       cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = b[blockOffset + i];
       
       float3 normal = getNormal(config, blocks, localMapCells, 
             x[blockOffset + i], y[blockOffset + i], z[blockOffset + i]);
       normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = normal.x;
       normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = normal.y;
       normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = normal.z;
       
       if (fullIndex[blockOffset + i] >= 0) {
          localMapCells[bIndex].pI[fullIndex[blockOffset + i]] = (startGIndex +
             blockOffsetComp[bLocalIndex] + i) * 3;
       }

   }

}

__kernel void transform3D(global float *points, const int numPoints, float3 origin,
      float3 rotation0, float3 rotation1, float3 rotation2) {
   int index = get_global_id(0);

   if (index < numPoints) {
      float3 point = (float3)(points[index * 3], points[index * 3 + 1], points[index * 3 + 2]);
      float3 temp = point * rotation0;
      points[index * 3] = temp.x + temp.y + temp.z + origin.x;
      temp = point * rotation1;
      points[index * 3 + 1] = temp.x + temp.y + temp.z + origin.y;
      temp = point * rotation2;
      points[index * 3 + 2] = temp.x + temp.y + temp.z + origin.z;

   }
}

__kernel void alignZ(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common,
      global float *ptsCurMap, global float *ptsPrevMap, global float *normsCurMap,
      global float *normsPrevMap, const int numPrevPts, const float zInc) {
   
   int index = get_global_id(0);
   int lIndex = get_local_id(0);

   local int count[LOCAL_SIZE];
   local float distance[LOCAL_SIZE];

   count[lIndex] = 0;
   distance[lIndex] = 0;

   if (index < numPrevPts) {
      float3 p;
      p.x = ptsPrevMap[index * 3];
      p.y = ptsPrevMap[index * 3 + 1];
      p.z = ptsPrevMap[index * 3 + 2] + zInc;
      float3 pNorm;
      pNorm.x = normsPrevMap[index * 3];
      pNorm.y = normsPrevMap[index * 3 + 1];
      pNorm.z = normsPrevMap[index * 3 + 2];
      

      int increment = config->NumCellsWidth * config->NumCellsWidth;
      int blockI = getBlockIndex(config, p);
      int cellI = getCellIndex(config, p, blockI);
      int cellXY = cellI % increment;


      int cont = 1;

      //if (p.z > -0.2) {
         //cont = 0;
      //}
      float normThresh = config->NormThresh;
      if (fabs(pNorm.z) < normThresh) {
         cont = 0;
      }

      if (blockI >= 0 && cont) {
         float minDist = INFINITY;
         int bInd = blocks[blockI];
         int maxTravel = config->MaxSearchCells;
         if (bInd >= 0 && localMapCells[bInd].pI[cellI] >= 0 
               && localMapCells[bInd].pI[cellI] / 3 < common->numPoints &&
               fabs(normsCurMap[localMapCells[bInd].pI[cellI] + 2]) > normThresh) {
            minDist = ptsCurMap[localMapCells[bInd].pI[cellI] + 2] - p.z;
            maxTravel = 1;
         }
         int travel;
         int bCur = bInd;
         int cellCur = cellI;
         int blocksCur = blockI;
         for (travel = 0; travel < maxTravel; travel++) {
            cellCur += increment;
            if (cellCur >= config->NumCellsTotal) {
               cellCur = cellXY;
               blocksCur = getBlockAdjZ(config, blocksCur, 1);
               if (blocksCur < 0) {
                  break;
               }
               bCur = blocks[blocksCur];
            }
            if (bCur >= 0 && localMapCells[bCur].pI[cellCur] >= 0
                  && localMapCells[bCur].pI[cellCur] / 3 < common->numPoints &&
                  fabs(normsCurMap[localMapCells[bCur].pI[cellCur] + 2]) > normThresh) {
               if (ptsCurMap[localMapCells[bCur].pI[cellCur] + 2] - p.z < fabs(minDist)) {
                  minDist = ptsCurMap[localMapCells[bCur].pI[cellCur] + 2] - p.z;
               }
               break;
            }
         }
         bCur = bInd;
         cellCur = cellI;
         blocksCur = blockI;
         for (travel = 0; travel < maxTravel; travel++) {
            cellCur -= increment;
            if (cellCur < 0) {
               cellCur += config->NumCellsTotal;
               blocksCur = getBlockAdjZ(config, blocksCur, -1);
               if (blocksCur < 0) {
                  break;
               }
               bCur = blocks[blocksCur];
            }
            if (bCur >= 0 && localMapCells[bCur].pI[cellCur] >= 0
                  && localMapCells[bCur].pI[cellCur] / 3 < common->numPoints &&
                  fabs(normsCurMap[localMapCells[bCur].pI[cellCur] + 2]) > normThresh) {
               if (ptsCurMap[localMapCells[bCur].pI[cellCur] + 2] - p.z < fabs(minDist)) {
                  minDist = ptsCurMap[localMapCells[bCur].pI[cellCur] + 2] - p.z;
               }
               break;
            }
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

