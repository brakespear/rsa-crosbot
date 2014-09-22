
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
      localMapCells[blockIndex].pI[cellIndex] = 255;
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

void markBlockActive(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalMapCommon *common, int bIndex) {
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
            }
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
      global oclDepthPoints *points,
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
         markBlockActive(config, blocks, common, bIndex);
         
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
         markBlockActive(config, blocks, common, adjXP);
         int adjXN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjXN);
         int adjYP = getBlockAdjX(config, bIndex, 1);
         markBlockActive(config, blocks, common, adjYP);
         int adjYN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjYN);
         int adjZP = getBlockAdjX(config, bIndex, 1);
         markBlockActive(config, blocks, common, adjZP);
         int adjZN = getBlockAdjX(config, bIndex, -1);
         markBlockActive(config, blocks, common, adjZN);
         /*adjXP = getBlockAdjX(config, adjXP, 1);
         markBlockActive(config, blocks, common, adjXP);
         adjXN = getBlockAdjX(config, adjXN, -1);
         markBlockActive(config, blocks, common, adjXN);
         adjYP = getBlockAdjX(config, adjYP, 1);
         markBlockActive(config, blocks, common, adjYP);
         adjYN = getBlockAdjX(config, adjYN, -1);
         markBlockActive(config, blocks, common, adjYN);
         adjZP = getBlockAdjX(config, adjZP, 1);
         markBlockActive(config, blocks, common, adjZP);
         adjZN = getBlockAdjX(config, adjZN, -1);
         markBlockActive(config, blocks, common, adjZN);*/
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjXP, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjXN, -1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYP, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjYN, -1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZP, 1));
         markBlockActive(config, blocks, common, getBlockAdjX(config, adjZN, -1));
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
               localMapCells[blockI].r[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].r[cIndex] +
                     weightVal * (float)points->r[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].g[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].g[cIndex] +
                     weightVal * (float)points->g[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].b[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].b[cIndex] +
                     weightVal * (float)points->b[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].weight[cIndex] += weightVal;
            } 
         }
      }
   }
}

void checkDirection(constant oclGraphSlam3DConfig *config, global oclLocalBlock *localMapCells,
      local float *x, local float *y, local float *z, local unsigned char *r, 
      local unsigned char *g, local unsigned char *b,
      local int *blockCount, int bIndex, int bLocalIndex, int blockOffset,
      int startI, int increment, int bNextIndex, float incXV, float incYV, float incZV) {

   float cellVal, cellNextVal;
   int nextI;
   int count = 0;
   for (int i = startI; count < config->NumCellsWidth; i += increment, count++) {
      nextI = i + increment;
      cellVal = localMapCells[bIndex].distance[i];
      if (count + 1 < config->NumCellsWidth) {
         cellNextVal = localMapCells[bIndex].distance[nextI];
      } else if (bNextIndex < 0 || bNextIndex > config->NumBlocksAllocated) {
         continue;
      } else {
      //if (incYV > 0.5) { continue; }
      //continue;
         cellNextVal = localMapCells[bNextIndex].distance[startI];
      }
      if (isnan(cellVal) || isnan(cellNextVal)) {
         continue;
      }
      if ((sign(cellVal) != sign(cellNextVal) || cellVal == 0) && localMapCells[bIndex].weight[i] > 18.9f) {
         //There is a crossing!
         float3 p = getCellCentre(config, i, localMapCells[bIndex].blockIndex);
         float inc = fabs(cellVal / (cellNextVal - cellVal)) * config->CellSize;
         if (inc >= config->CellSize) {
            inc = config->CellSize;
         }
         float incX = inc * incXV;
         float incY = inc * incYV;
         float incZ = inc * incZV;
         if ((inc < config->CellSize/2.0f && localMapCells[bIndex].pI[i] < 255) ||
               (inc >= config->CellSize/2.0f && count + 1 < config->NumCellsWidth && 
                localMapCells[bIndex].pI[nextI] < 255)) {
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
               localMapCells[bIndex].pI[i] = (unsigned char)retI;
            } else if (count + 1 < config->NumCellsWidth) {
               r[blockOffset + retI] = localMapCells[bIndex].r[nextI];
               g[blockOffset + retI] = localMapCells[bIndex].g[nextI];
               b[blockOffset + retI] = localMapCells[bIndex].b[nextI];
               localMapCells[bIndex].pI[nextI] = (unsigned char)retI;
            } else {
               r[blockOffset + retI] = localMapCells[bNextIndex].r[startI];
               g[blockOffset + retI] = localMapCells[bNextIndex].g[startI];
               b[blockOffset + retI] = localMapCells[bNextIndex].b[startI];
            }
         }
      }
   }
}


__kernel void extractPoints(constant oclGraphSlam3DConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common,
      global float *ps, global unsigned char *cols) {
   
   int gIndex = get_global_id(0);
   int lIndex = get_local_id(0);
   int groupId = get_group_id(0);

   local float x[MAX_POINTS_GROUP];
   local float y[MAX_POINTS_GROUP];
   local float z[MAX_POINTS_GROUP];
   local unsigned char r[MAX_POINTS_GROUP];
   local unsigned char g[MAX_POINTS_GROUP];
   local unsigned char b[MAX_POINTS_GROUP];
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
   barrier(CLK_LOCAL_MEM_FENCE);

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
      checkDirection(config, localMapCells, x, y, z, r, g, b, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, 0, 0, 1);
      
      /*for (int i = startI; i < config->NumCellsTotal; i += increment) {
         nextI = i + increment;
         cellVal = localMapCells[bIndex].distance[i];
         if (nextI < config->NumCellsTotal) {
            cellNextVal = localMapCells[bIndex].distance[nextI];
         } else if (bNextIndex < 0) {
            continue;
         } else {
            cellNextVal = localMapCells[bNextIndex].distance[startI];
         }
         if (sign(cellVal) != sign(cellNextVal)) {
            //There is a crossing!
            float3 p = getCellCentre(config, i, bIndex);
            float inc = fabs(cellVal / (cellNextVal - cellVal)) * config->CellSize;
            int retI = atomic_inc(blockCount[bLocalIndex]);
            if (retI >= MAX_POINTS_GOUP / BLOCKS_PER_GROUP) {
               continue;
            }
            x[blockOffset + retI] = p.x;
            y[blockOffset + retI] = p.y;
            z[blockOffset + retI] = p.z + inc;
            if (inc < config->CellSize / 2.0f) {
               r[blockOffset + retI] = localMapCells[bIndex].r[i];
               g[blockOffset + retI] = localMapCells[bIndex].g[i];
               b[blockOffset + retI] = localMapCells[bIndex].b[i];
               localMapCells[bIndex].pI[i] = (unsigned char)retI;
            } else if (nextI < config->numCellsTotal) {
               r[blockOffset + retI] = localMapCells[bIndex].r[nextI];
               g[blockOffset + retI] = localMapCells[bIndex].g[nextI];
               b[blockOffset + retI] = localMapCells[bIndex].b[nextI];
               localMapCells[bIndex].pI[nextI] = (unsigned char)retI;
            } else {
               r[blockOffset + retI] = localMapCells[bNextIndex].r[startI];
               g[blockOffset + retI] = localMapCells[bNextIndex].g[startI];
               b[blockOffset + retI] = localMapCells[bNextIndex].b[startI];
            }
         }
      }*/
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
      checkDirection(config, localMapCells, x, y, z, r, g, b, blockCount, bIndex,
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
      checkDirection(config, localMapCells, x, y, z, r, g, b, blockCount, bIndex,
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
   }

}

//Each float3 of rotation is a row of the rotation matrix
/*__kernel void transform3D(global oclDepthPoints *points, const int numPoints, const float3 origin, const float3 rotation0, const float3 rotation1, const float3 rotation2) {
   int index = get_global_id(0);

   if (index < numPoints) {
      float3 point = (float3)(points->pointX[index], points->pointY[index], points->pointZ[index]);
      float3 transP = transformPoint(point, origin, rotation0, rotation1, rotation2);
      points->pointX[index] = transP.x;
      points->pointY[index] = transP.y;
      points->pointZ[index] = transP.z;

   }
}*/
