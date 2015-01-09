/*
 * positionTrackFull3D.cl
 *
 * Created on: 12/12/2014
 *     Author: adrianr
 */

#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

__kernel void clearLocalMap(constant oclPositionTrackConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common) {
   int index = get_global_id(0);
   int globalSize = get_global_size(0);

   for (int i = index; i < config->NumBlocksTotal; i+= globalSize) {
      blocks[i] = -1;
   }
   for (int i = index; i < config->NumBlocksAllocated * config->NumCellsTotal; i+= globalSize) {
      int blockIndex = i / config->NumCellsTotal;
      int cellIndex = i % config->NumCellsTotal;
      localMapCells[blockIndex].distance[cellIndex] = NAN;
      localMapCells[blockIndex].weight[cellIndex] = 0;
      localMapCells[blockIndex].r[cellIndex] = 0;
      localMapCells[blockIndex].g[cellIndex] = 0;
      localMapCells[blockIndex].b[cellIndex] = 0;
      localMapCells[blockIndex].occupied[cellIndex] = 0;
   }

   for (int i = index; i < config->NumBlocksAllocated; i+= globalSize) {
      localMapCells[i].blockIndex = -1;
      common->emptyBlocks[i] = i;
   }

   if (index == 0) {
      common->highestBlockNum = 0;
      common->nextEmptyBlock = 0;
      common->numActiveBlocks = 0;
   }
}

/*
 * Converts a depth measurement (depth) at pixel(u,v) to a point
 */
float3 convertPixelToPoint(constant oclPositionTrackConfig *config,
      int u, int v, float depth) {
   float3 p;
   p.x = (((float)u - config->cx) * depth - config->tx) / config->fx;
   p.y = (((float)v - config->cy) * depth - config->ty) / config->fy;
   p.z = depth;
   return p;
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
int getBlockIndex(constant oclGraphSlam3DConfig *config, float3 point, const int3 cent,
      const int3 centMod) {

   int3 pointInd;
   pointInd.x = point.x / config->BlockSize;
   pointInd.y = point.y / config->Blocksize;
   pointInd.z = point.z / config->BlockSize;
   pointInd -= cent;

   int offXY = config->NumBlocksWidth / 2;
   int offZ = config->NumBlocksHeight / 2;

   if (pointInd.x >= -offXY && pointInd.x < offXY &&
         pointInd.y >= -offXY && pointInd.y < offXY &&
         pointInd.z >= -offZ && pointInd.z < offZ) {

   /*if (abs(pointInd.x) < config->NumBlocksWidth / 2 - 1 && 
         abs(pointInd.y) < config->NumBlockswidth / 2 - 1 &&
         abs(pointInd.z) < config->NumBlocksHeight / 2 - 1) {*/

      pointInd += centMod;

      if (pointInd.x < 0) {
         pointInd.x += config->NumBlocksWidth;
      } else if (pointInd.x >= config->NumBlocksWidth) {
         pointInd.x -= config->NumBlocksWidth;
      }
      if (pointInd.y < 0) {
         pointInd.y += config->NumBlocksWidth;
      } else if (pointInd.y >= config->NumBlocksWidth) {
         pointInd.y -= config->NumBlocksWidth;
      }
      if (pointInd.z < 0) {
         pointInd.z += config->NumBlocksHeight;
      } else if (pointInd.z >= config->NumBlocksHeight) {
         pointInd.z -= config->NumBlocksHeight;
      }
      return z * config->NumBlocksWidth * config->NumBlocksWidth +
         y * config->NumBlocksWidth + x;
   } else {
      return -1;
   }
}

/*
 * Gets the coordinates of the lowest corner of the block
 */
float3 getBlockPosition(constant oclGraphSlam3DConfig *config, int index,
      const int3 cent, const int3 centMod) {
   int x = index % config->NumBlocksWidth;
   int y = (index / config->NumBlocksWidth) % config->NumBlocksWidth;
   int z = index / (config->NumBlocksWidth * config->NumBlocksWidth);
   int offXY = config->NumBlocksWidth / 2;
   int offZ = config->NumBlocksHeight / 2;

   //Adjust x,y,z indexes of block for wrapping
   if (centMod.x > offXY && x < centMod.x - offXY) {
      x += config->NumBlocksWidth;
   } else if (centMod.x < offXY && x >= centMod.x + offXY) {
      x -= config->NumBlocksWidth;
   }
   if (centMod.y > offXY && y < centMod.y - offXY) {
      y += config->NumBlocksWidth;
   } else if (centMod.y < offXY && y >= centMod.y + offXY) {
      y -= config->NumBlocksWidth;
   }
   if (centMod.z > offZ && z < centMod.z - offZ) {
      z += config->NumBlocksHeight;
   } else if (centMod.z < offZ && z >= centMod.z + offZ) {
      z -= config->NumBlocksHeight;
   }
   float3 p;
   p.x = (cent.x + (x - centMod.x)) * config->BlockSize;
   p.y = (cent.y + (y - centMod.y)) * config->BlockSize;
   p.z = (cent.z + (z - centMod.z)) * config->BlockSize;
   return p;
}

/*
 * Gets the coordinates of the lowest corner of the block point p resides in
 */
float3 getBlockPositionFromPoint(constant oclGraphSlam3DConfig *config, float3 p) {
   return floor(p / config->BlockSize) * config->BlockSize;
}

int getCellIndex(constant oclGraphSlam3DConfig *config, float3 point) {
   float3 b = getBlockPositionFromPoint(config, point);
   float3 diff = point - b;
   int x = diff.x / config->CellSize;
   int y = diff.y / config->CellSize;
   int z = diff.z / config->CellSize;
   return z * config->NumCellsWidth * config->NumCellsWidth + y * config->NumCellsWidth + x;
}

/*
 * Returns the point at the centre of the cell, given the index of the cell
 * in the block (cIndex), and the index of the block (bIndex)
 */
float3 getCellCentre(constant oclGraphSlam3DConfig *config, int cIndex, int bIndex,
      const int3 cent, const int3 centMod) {
   float3 p = getBlockPosition(config, bIndex, cent, centMod);

   int x = cIndex % config->NumCellsWidth;
   int y = (cIndex / config->NumCellsWidth) % config->NumCellsWidth;
   int z = cIndex / (config->NumCellsWidth * config->NumCellsWidth);

   p.x += x * config->CellSize + (config->CellSize / 2.0f);
   p.y += y * config->CellSize + (config->CellSize / 2.0f);
   p.z += z * config->CellSize + (config->CellSize / 2.0f);
   return p;
}

/*
 * Gets the index of the adjacent block in z.
 */ 
int getBlockAdjZ(constant oclGraphSlam3DConfig *config, int index, int dir, int cent) {
   if (index < 0) return -1;
   int zVal = index / (config->NumBlocksWidth * config->NumBlocksWidth);
   int indexRem = index - zVal;
   int off = config->NumBlocksHeight / 2;
   if (dir < 0) {
      if ((cent >= off && zVal == cent - off) ||
            (cent < off && zVal == cent + off)) {
         return -1;
      } else if (zVal == 0) {
         return config->NumBlocksWidth * config->numBlocksWidth * (config->NumBlocksHeight - 1)
            + indexRem;
      } else {
         return index - (config->NumBlocksWidth * config->NumBlocksWidth);
      }
   } else {
      if ((cent <= off && zVal == cent + off - 1) ||
            (cent > off && zVal == cent - off - 1)) {
         return -1;
      } else if (zVal == config->NumBlocksHeight - 1) {
         return indexRem;
      } else {
         return index + (config->NumBlocksWidth * config->NumBlocksWidth);
      }
   }
}
/*
 * Gets the index of the adjacent block in x.
 */ 
int getBlockAdjX(constant oclGraphSlam3DConfig *config, int index, int dir, int cent) {
   if (index < 0) return -1;
   int xVal = index % config->NumBlocksWidth;
   int indexRem = index - xVal;
   int off = config->NumBlocksWidth / 2;
   if (dir < 0) {
      if ((cent >= off && xVal == cent - off) ||
            (cent < off && xVal == cent + off)) {
         return -1;
      } else if (xVal == 0) {
         return config->NumBlocksWidth - 1 + indexRem;
      } else {
         return index - 1;
      }
   } else {
      if ((cent <= off && xVal == cent + off - 1) ||
            (cent > off && xVal == cent - off - 1)) {
         return -1;
      } else if (xVal == config->NumBlocksWidth - 1) {
         return indexRem;
      } else {
         return index + 1;
      }
   }
}
/*
 * Gets the index of the adjacent block in y.
 */ 
int getBlockAdjY(constant oclGraphSlam3DConfig *config, int index, int dir, int cent) {
   if (index < 0) return -1;
   int yVal = (index / config->NumBlocksWidth) % config->NumBlocksWidth;
   int indexRem = index - yVal;
   int off = config->NumBlocksWidth / 2;
   if (dir < 0) {
      if ((cent >= off && yVal == cent - off) ||
            (cent < off && yVal == cent + off)) {
         return -1;
      } else if (yVal == 0) {
         return config->NumBlocksWidth * (config->NumBlocksWidth - 1) + indexRem;
      } else {
         return index - config->NumBlocksWidth;
      }
   } else {
      if ((cent <= off && yVal == cent + off - 1) ||
            (cent > off && yVal == cent - off - 1)) {
         return -1;
      } else if (yVal == config->NumBlocksWidth - 1) {
         return indexRem;
      } else {
         return index + config->NumBlocksWidth;
      }
   }
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
 * transform the point from the sensor frame into the icp frame
 */
__kernel void checkBlocksExist(constant oclPositionTrackConfig *config,
      global int *blocks, global oclLocalMapCommon *common,
      global oclLocalBlock *localMapCells, global float *depthP,
      const int numPoints, const int3 cent, const float3 origin,
      const float3 rotation0, const float3 rotation1, const float3 rotation2) {

   int index = get_global_id(0);

   if (index < numPoints && !isnan(depthP[index])) {
      //Convert depth to xyz
      int u = index % config->ImageWidth;
      int v = index / config->ImageWidth;
      float3 point = convertPixelToPoint(config, u, v, depthP[index]);
      
      //Transform point from camera frame to icp frame
      float3 transP = transformPoint(point, origin, rotation0, rotation1, rotation2);
      int3 centMod;
      centMod.x = cent.x % config->NumBlocksWidth;
      centMod.y = cent.y % config->NumBlocksWidth;
      centMod.z = cent.z % config->NumBlocksHeight;
      int bIndex = getBlockIndex(config, transP, cent, centMod);

      if (bIndex >= 0) {
         int cIndex = getCellIndex(config, transP);
         markBlockActive(config, blocks, common, bIndex, 1, cIndex, localMapCells);
         
         int adjXP = getBlockAdjX(config, bIndex, 1, centMod.x);
         markBlockActive(config, blocks, common, adjXP, 0, cIndex, localMapCells);
         int adjXN = getBlockAdjX(config, bIndex, -1, centMod.x);
         markBlockActive(config, blocks, common, adjXN, 0, cIndex, localMapCells);
         int adjYP = getBlockAdjY(config, bIndex, 1, centMod.y);
         markBlockActive(config, blocks, common, adjYP, 0, cIndex, localMapCells);
         int adjYN = getBlockAdjY(config, bIndex, -1, centMod.y);
         markBlockActive(config, blocks, common, adjYN, 0, cIndex, localMapCells);
         int adjZP = getBlockAdjZ(config, bIndex, 1, centMod.z);
         markBlockActive(config, blocks, common, adjZP, 0, cIndex, localMapCells);
         int adjZN = getBlockAdjZ(config, bIndex, -1, centMod.z);
         markBlockActive(config, blocks, common, adjZN, 0, cIndex, localMapCells);

      }
   }
}

__kernel void addRequiredBlocks(constant oclPositionTrackConfig *config,
      global int *blocks, global oclLocalBlock *localMapCells,
      global oclLocalMapCommon *common) {

   int index = get_global_id(0);

   if (index < common->numActiveBlocks && index < MAX_NUM_ACTIVE_BLOCKS) {
      int bIndex = common->activeBlocks[index];
      int cellIndex;
      if (blocks[bIndex] < 0) {
         //Create a new block
         cellIndex = atomic_inc(&(common->nextEmptyBlock));
         if (cellIndex < config->NumBlocksAllocated) {
            cellIndex = common->emptyBlocks[cellIndex];
            atomicFloatMax(&(common->highestBlockNum), cellIndex);
            blocks[bIndex] = cellIndex;
            localMapCells[cellIndex].blockIndex = bIndex;
         } else {
            atomic_dec(&(common->nextEmptyBlock));
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
 * transform from the icp frame to the sensor frame. 
 */
__kernel void addFrame(constant oclPositionTrackConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common, 
      global float *depthP, global oclColourPoints *colourP, 
      const int numActiveBlocks, const int3 cent,
      const float3 origin, const float3 rotation0, 
      const float3 rotation1, const float3 rotation2) {

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
   int3 centMod;
   centMod.x = cent.x % config->NumBlocksWidth;
   centMod.y = cent.y % config->NumBlocksWidth;
   centMod.z = cent.z % config->NumBlocksHeight;

   for (int cIndex = startIndex; cIndex < config->NumCellsTotal; cIndex += LOCAL_SIZE) {
      int blockI = common->activeBlocks[aIndex];

      float3 cellCentre = getCellCentre(config, cIndex, localMapCells[blockI].blockIndex, 
         cent, centMod);
      //Convert point into camera frame
      float3 cameraFramePoint = transformPoint(cellCentre, origin, rotation0, rotation1, rotation2);
      //Find pixel coordinates of point
      int u = (config->fx * cameraFramePoint.x + config->tx) / cameraFramePoint.z + config->cx;
      int v = (config->fy * cameraFramePoint.y + config->ty) / cameraFramePoint.z + config->cy;
      if (u >= 0 && u < config->ImageWidth && v >= 0 && v < config->ImageHeight) {
         int pointI = config->ImageWidth * v + u;
         float depthVal = depthP[pointI];
         
         if (isnan(depthVal)) {
            continue;
         }
         if (cameraFramePoint.z > config->MaxDistance) {
            continue;
         }
         float tsdfVal = depthVal - cameraFramePoint.z;
         float weightVal = 1.0f/* / distPoint*/;

         if ((tsdfVal >= 0 /*&& tsdfVal < config->TruncPos*/) ||
               (tsdfVal < 0 && tsdfVal * -1.0f < config->TruncNeg)) {
            if (tsdfVal >= config->TruncPos) {
               tsdfVal = config->TruncPos;
            }
            if (isnan(localMapCells[blockI].distance[cIndex])) {
               localMapCells[blockI].distance[cIndex] = tsdfVal;
               localMapCells[blockI].weight[cIndex] = weightVal;
               localMapCells[blockI].r[cIndex] = colourP->r[pointI];
               localMapCells[blockI].g[cIndex] = colourP->g[pointI];
               localMapCells[blockI].b[cIndex] = colourP->b[pointI];
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
               localMapCells[blockI].r[cIndex] = colourP->r[pointI];
               localMapCells[blockI].g[cIndex] = colourP->g[pointI];
               localMapCells[blockI].b[cIndex] = colourP->b[pointI];
               localMapCells[blockI].weight[cIndex] += weightVal;
            } 
         }
      }
   }
}

