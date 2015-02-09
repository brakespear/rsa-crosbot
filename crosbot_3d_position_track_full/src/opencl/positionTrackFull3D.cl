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
      localMapCells[blockIndex].pI[cellIndex] = -1;
   }

   for (int i = index; i < config->NumBlocksAllocated; i+= globalSize) {
      localMapCells[i].blockIndex = -1;
      localMapCells[i].haveExtracted = 0;
      common->emptyBlocks[i] = i;
   }

   if (index == 0) {
      common->highestBlockNum = 0;
      common->nextEmptyBlock = 0;
      common->numActiveBlocks = 0;
      common->numBlocksToExtract = 0;
      common->numPoints = 0;
      common->numBlocksToDelete = 0;
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
int getBlockIndex(constant oclPositionTrackConfig *config, float3 point, const int3 cent,
      const int3 centMod) {

   int3 pointInd;
   pointInd.x = floor(point.x / config->BlockSize);
   pointInd.y = floor(point.y / config->BlockSize);
   pointInd.z = floor(point.z / config->BlockSize);
   /*pointInd.x = point.x / config->BlockSize;
   pointInd.y = point.y / config->BlockSize;
   pointInd.z = point.z / config->BlockSize;*/
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
      return pointInd.z * config->NumBlocksWidth * config->NumBlocksWidth +
         pointInd.y * config->NumBlocksWidth + pointInd.x;
   } else {
      return -1;
   }
}

/*
 * Gets the coordinates of the lowest corner of the block
 */
float3 getBlockPosition(constant oclPositionTrackConfig *config, int index,
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
float3 getBlockPositionFromPoint(constant oclPositionTrackConfig *config, float3 p) {
   return floor(p / config->BlockSize) * config->BlockSize;
}

int getCellIndex(constant oclPositionTrackConfig *config, float3 point) {
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
float3 getCellCentre(constant oclPositionTrackConfig *config, int cIndex, int bIndex,
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
int getBlockAdjZ(constant oclPositionTrackConfig *config, int index, int dir, int cent) {
   if (index < 0) return -1;
   int zVal = index / (config->NumBlocksWidth * config->NumBlocksWidth);
   int indexRem = index - (zVal * config->NumBlocksWidth * config->NumBlocksWidth);
   int off = config->NumBlocksHeight / 2;
   if (dir < 0) {
      if ((cent >= off && zVal == cent - off) ||
            (cent < off && zVal == cent + off)) {
         return -1;
      } else if (zVal == 0) {
         return config->NumBlocksWidth * config->NumBlocksWidth * (config->NumBlocksHeight - 1)
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
int getBlockAdjX(constant oclPositionTrackConfig *config, int index, int dir, int cent) {
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
int getBlockAdjY(constant oclPositionTrackConfig *config, int index, int dir, int cent) {
   if (index < 0) return -1;
   int yVal = (index / config->NumBlocksWidth) % config->NumBlocksWidth;
   int indexRem = index - (yVal * config->NumBlocksWidth);
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

void markBlockActive(constant oclPositionTrackConfig *config, global int *blocks,
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
      int u = index % config->ImageWidth;
      int v = index / config->ImageWidth;
         
      float3 point = convertPixelToPoint(config, u, v, depthP[index]);
      //float3 point = (float3)(points->x[index], points->y[index], points->z[index]);

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
         
         int adjXXP = getBlockAdjX(config, adjXP, 1, centMod.x);
         markBlockActive(config, blocks, common, adjXXP, 0, cIndex, localMapCells);
         int adjXXN = getBlockAdjX(config, adjXN, -1, centMod.x);
         markBlockActive(config, blocks, common, adjXXN, 0, cIndex, localMapCells);
         int adjYYP = getBlockAdjY(config, adjYP, 1, centMod.y);
         markBlockActive(config, blocks, common, adjYYP, 0, cIndex, localMapCells);
         int adjYYN = getBlockAdjY(config, adjYN, -1, centMod.y);
         markBlockActive(config, blocks, common, adjYYN, 0, cIndex, localMapCells);
         int adjZZP = getBlockAdjZ(config, adjZP, 1, centMod.z);
         markBlockActive(config, blocks, common, adjZZP, 0, cIndex, localMapCells);
         int adjZZN = getBlockAdjZ(config, adjZN, -1, centMod.z);
         markBlockActive(config, blocks, common, adjZZN, 0, cIndex, localMapCells);

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
            atomic_max(&(common->highestBlockNum), cellIndex);
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
      if (config->ReExtractBlocks && cellIndex >= 0) {
         localMapCells[cellIndex].haveExtracted = 0;
      }
   }
}

/*
 * Adds a frame to the graph. The transform given by origin and rotations is the
 * transform from the icp frame to the sensor frame. 
 */
__kernel void addFrame(constant oclPositionTrackConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common, 
      global float *depthP, global oclColourPoints *colourP, global oclDepthPoints *normals,
      const int numActiveBlocks, const int3 cent,
      const float3 origin, const float3 rotation0, 
      const float3 rotation1, const float3 rotation2) {

   int gIndex = get_global_id(0);
   int lIndex = get_local_id(0);
   int groupId = get_group_id(0);
   if (gIndex == 0) {
      common->numActiveBlocks = 0;
      common->numBlocksToExtract = 0;
      common->numPoints = 0;
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
         
         if (isnan(depthVal) || isnan(normals->x[pointI])) {
            continue;
         }
         if (cameraFramePoint.z > config->MaxDistance) {
            continue;
         }
         float tsdfVal = depthVal - cameraFramePoint.z;
         //tsdfVal *= -1;


         float weightVal = 1.0f  / cameraFramePoint.z;
         //float3 normalP = (float3)(normals->x[pointI], normals->y[pointI], normals->z[pointI]);
         //float3 ray = (float3)((u - config->cx - config->tx) / config->fx,
         //      (v - config->cy - config->ty) / config->fy, 1.0f);
         //float weightVal = fabs(dot(normalP, fast_normalize(ray))) / cameraFramePoint.z;

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
               
               localMapCells[blockI].r[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].r[cIndex] +
                     weightVal * (float)colourP->r[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].g[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].g[cIndex] +
                     weightVal * (float)colourP->g[pointI]) / (weightVal + weightPrev));
               localMapCells[blockI].b[cIndex] = (unsigned char)((weightPrev * 
                        (float)localMapCells[blockI].b[cIndex] +
                     weightVal * (float)colourP->b[pointI]) / (weightVal + weightPrev));
               
               //localMapCells[blockI].r[cIndex] = colourP->r[pointI];
               //localMapCells[blockI].g[cIndex] = colourP->g[pointI];
               //localMapCells[blockI].b[cIndex] = colourP->b[pointI];
               localMapCells[blockI].weight[cIndex] += weightVal;
            } 
         }
      }
   }
}

__kernel void markForExtraction(constant oclPositionTrackConfig *config, 
      global int *blocks, global oclLocalBlock *localMapCells, 
      global oclLocalMapCommon *common, const int3 oldCent, const int3 newCent,
      int isFullExtract, const float3 position) {

   int index = get_global_id(0);
   int needExtract = 0;
   int needDelete = 0;

   int offXY = config->NumBlocksWidth / 2;
   int offZ = config->NumBlocksHeight / 2;
   int3 oldCentMod;
   oldCentMod.x = oldCent.x % config->NumBlocksWidth;
   oldCentMod.y = oldCent.y % config->NumBlocksWidth;
   oldCentMod.z = oldCent.z % config->NumBlocksHeight;

   if (index < common->highestBlockNum && localMapCells[index].blockIndex >= 0) {
      int bIndex = localMapCells[index].blockIndex;
      int x = bIndex % config->NumBlocksWidth;
      int y = (bIndex / config->NumBlocksWidth) % config->NumBlocksWidth;
      int z = bIndex / (config->NumBlocksWidth * config->NumBlocksWidth);

      int3 diff = newCent - oldCent;

      //Unwarp indexes
      if (oldCentMod.x > offXY && x < oldCentMod.x - offXY) {
         x += config->NumBlocksWidth;
      } else if (oldCentMod.x < offXY && x >= oldCentMod.x + offXY) {
         x -= config->NumBlocksWidth;
      }
      if (oldCentMod.y > offXY && y < oldCentMod.y - offXY) {
         y += config->NumBlocksWidth;
      } else if (oldCentMod.y < offXY && y >= oldCentMod.y + offXY) {
         y -= config->NumBlocksWidth;
      }
      if (oldCentMod.z > offZ && z < oldCentMod.z - offZ) {
         z += config->NumBlocksHeight;
      } else if (oldCentMod.z < offZ && z >= oldCentMod.z + offZ) {
         z -= config->NumBlocksHeight;
      }

      //Work out if indexes are border cases
      if ((diff.x > 0 && x < oldCentMod.x + diff.x - offXY) ||
          (diff.x < 0 && x >= oldCentMod.x + diff.x + offXY) ||
          (diff.y > 0 && y < oldCentMod.y + diff.y - offXY) ||
          (diff.y < 0 && y >= oldCentMod.y + diff.y + offXY) ||
          (diff.z > 0 && z < oldCentMod.z + diff.z - offZ) ||
          (diff.z < 0 && z >= oldCentMod.z + diff.z + offZ)) {
         needDelete = 1;
         if (localMapCells[index].haveExtracted == 0) {
            needExtract = 1;
         }
         //needExtract = 1;
      }
      
      if (isFullExtract && localMapCells[index].haveExtracted == 0) {
         //work out if block is behind robot
         float3 blockCent = getBlockPosition(config, bIndex, oldCent, oldCentMod) +
            (float3)(config->BlockSize/2.0f);
         float ang = atan2(blockCent.y - position.y, blockCent.x - position.x);
         float diffAng = position.z - ang;
         ANGNORM(diffAng);
         if (fabs(diffAng) > M_PI/2.0f) {
            needExtract = 1;
         }
      }
      //if (isFullExtract) {
      //   needExtract = 1;
      //}
   }

   if (needExtract) {
      int ret = atomic_inc(&(common->numBlocksToExtract));
      common->blocksToExtract[ret] = index;
      localMapCells[index].haveExtracted = 1;
   }
   if (needDelete) {
      int ret = atomic_inc(&(common->numBlocksToDelete));
      common->blocksToDelete[ret] = index;
   }
}

__kernel void markAllForExtraction(constant oclPositionTrackConfig *config,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common) {

   int index = get_global_id(0);

   if (index < common->highestBlockNum && localMapCells[index].blockIndex >= 0) {
      int ret = atomic_inc(&(common->numBlocksToExtract));
      common->blocksToExtract[ret] = index;
   }
}

void checkDirection(constant oclPositionTrackConfig *config, global oclLocalBlock *localMapCells,
      local float *x, local float *y, local float *z, local unsigned char *r, 
      local unsigned char *g, local unsigned char *b, local short *fullIndex,
      local int *blockCount, int bIndex, int bLocalIndex, int blockOffset,
      int startI, int increment, int bNextIndex, const int3 cent, const int3 centMod,
      float incXV, float incYV, float incZV) {

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
      } else if (bNextIndex < 0 /*|| bNextIndex > config->NumBlocksAllocated*/) {
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
      /*if (fabs(cellVal) > config->CellSize * 10 || fabs(cellNextVal) > config->CellSize * 10) {
         continue;
      }*/
      if ((sign(cellVal) != sign(cellNextVal) || cellVal == 0) /*&& 
            localMapCells[bIndex].weight[i] > 5.0f */&& (config->UseOccupancyForSurface == 0 || 
            localMapCells[bIndex].occupied[i] > 0 || localMapCells[bni].occupied[ni] > 0)) {
         //There is a crossing!

         float3 p = getCellCentre(config, i, localMapCells[bIndex].blockIndex, cent, centMod);
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
               
            }
         }
      }
   }
}

float3 getNormal(constant oclPositionTrackConfig *config, global int *blocks, 
      global oclLocalBlock *localMapCells, const int3 cent, int blockI, int cellI) {

   int xI = cellI % config->NumCellsWidth;
   int yI = (cellI / config->NumCellsWidth) % config->NumCellsWidth;
   int zI = cellI / (config->NumCellsWidth * config->NumCellsWidth);
      
   if (blockI < 0 || blocks[blockI] < 0) {
      //This shouldn't happen
      return (float3)(NAN,NAN,NAN);
   }

   float xH = NAN, xL = NAN, yH = NAN, yL = NAN, zL = NAN, zH = NAN;

   if (xI + 1 >= config->NumCellsWidth) {
      int bI = getBlockAdjX(config, blockI, 1, cent.x);
      if (bI >= 0 && blocks[bI] >= 0) {
         xH = localMapCells[blocks[bI]].distance[cellI - xI];
      }
   } else {
      xH = localMapCells[blocks[blockI]].distance[cellI + 1];
   }
   if (xI - 1 < 0) {
      int bI = getBlockAdjX(config, blockI, -1, cent.x);
      if (bI >= 0 && blocks[bI] >= 0) {
         xL = localMapCells[blocks[bI]].distance[cellI + config->NumCellsWidth - 1];
      }
   } else {
      xL = localMapCells[blocks[blockI]].distance[cellI - 1];
   }
   if (yI + 1 >= config->NumCellsWidth) {
      int bI = getBlockAdjY(config, blockI, 1, cent.y);
      if (bI >= 0 && blocks[bI] >= 0) {
         yH = localMapCells[blocks[bI]].distance[cellI - yI * config->NumCellsWidth];
      }
   } else {
      yH = localMapCells[blocks[blockI]].distance[cellI + config->NumCellsWidth];
   }
   if (yI - 1 < 0) {
      int bI = getBlockAdjY(config, blockI, -1, cent.y);
      if (bI >= 0 && blocks[bI] >= 0) {
         yL = localMapCells[blocks[bI]].distance[cellI + config->NumCellsWidth * 
            (config->NumCellsWidth - 1)];
      }
   } else {
      yL = localMapCells[blocks[blockI]].distance[cellI - config->NumCellsWidth];
   }
   if (zI + 1 >= config->NumCellsWidth) {
      int bI = getBlockAdjZ(config, blockI, 1, cent.z);
      if (bI >= 0 && blocks[bI] >= 0) {
         zH = localMapCells[blocks[bI]].distance[xI + yI * config->NumCellsWidth];
      }
   } else {
      zH = localMapCells[blocks[blockI]].distance[cellI + config->NumCellsWidth * 
         config->NumCellsWidth];
   }
   if (zI - 1 < 0) {
      int bI = getBlockAdjZ(config, blockI, -1, cent.z);
      if (bI >= 0 && blocks[bI] >= 0) {
         zL = localMapCells[blocks[bI]].distance[cellI + config->NumCellsWidth * 
            config->NumCellsWidth * (config->NumCellsWidth - 1)];
      }
   } else {
      zL = localMapCells[blocks[blockI]].distance[cellI - config->NumCellsWidth *
         config->NumCellsWidth];
   }
   if (isnan(xH) || isnan(xL) || isnan(yH) || isnan(yL) ||
         isnan(zH) || isnan(zL)) {
      return (float3)(NAN,NAN,NAN);
   } else {
      float3 normal = (float3)(xH - xL, yH - yL, zH - zL);
      
      //return normal;
      return fast_normalize(normal);
   }
}

__kernel void extractPoints(constant oclPositionTrackConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common,
      global float *ps, global unsigned char *cols, global float *normals,
      const int3 cent, const int extractNorms) {
   
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
   
   int3 centMod;
   centMod.x = cent.x % config->NumBlocksWidth;
   centMod.y = cent.y % config->NumBlocksWidth;
   centMod.z = cent.z % config->NumBlocksHeight;

   int sliceSize = config->NumCellsWidth * config->NumCellsWidth;
   int bLocalIndex = lIndex / sliceSize;
   int bIndexExtract = BLOCKS_PER_GROUP * groupId + bLocalIndex;
   int cIndex = lIndex % sliceSize;
   int blockOffset = (MAX_POINTS_GROUP / BLOCKS_PER_GROUP) * bLocalIndex;
   int bIndex = -1;
   if (bIndexExtract < common->numBlocksToExtract) {
      bIndex = common->blocksToExtract[bIndexExtract];
   }

   if (cIndex == 0 && bLocalIndex < BLOCKS_PER_GROUP) {
      /*blockCount[bLocalIndex] = 1;
      x[blockOffset] = NAN;
      y[blockOffset] = NAN;
      z[blockOffset] = NAN;
      r[blockOffset] = 0;
      g[blockOffset] = 0;
      b[blockOffset] = 0;
      if (bIndexExtract >= common->numBlocksToExtract) {
         blockCount[bLocalIndex] = 0;
      }*/
      blockCount[bLocalIndex] = 0;
   }
   for (int i = lIndex; i < MAX_POINTS_GROUP; i+= LOCAL_SIZE) {
      fullIndex[i] = -1;
   }

   barrier(CLK_LOCAL_MEM_FENCE);

   int startI, bNextIndex, increment;
   //int nextI;
   //float cellVal, cellNextVal;
   
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndexExtract < common->numBlocksToExtract) {
      startI = cIndex;
      bNextIndex = getBlockAdjZ(config, localMapCells[bIndex].blockIndex, 1, cent.z);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = config->NumCellsWidth * config->NumCellsWidth;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, cent, centMod, 0, 0, 1);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndexExtract < common->numBlocksToExtract) {
      int zz = cIndex / config->NumCellsWidth;
      int yy = cIndex % config->NumCellsWidth;
      startI = zz * config->NumCellsWidth * config->NumCellsWidth + yy * config->NumCellsWidth;
      bNextIndex = getBlockAdjX(config, localMapCells[bIndex].blockIndex, 1, cent.x);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = 1;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, cent, centMod, 1, 0, 0);
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (bLocalIndex < BLOCKS_PER_GROUP && bIndexExtract < common->numBlocksToExtract) {
      int xx = cIndex % config->NumCellsWidth;
      int zz = cIndex / config->NumCellsWidth;
      startI = zz * config->NumCellsWidth * config->NumCellsWidth + xx;
      bNextIndex = getBlockAdjY(config, localMapCells[bIndex].blockIndex, 1, cent.y);
      if (bNextIndex >= 0) {
         bNextIndex = blocks[bNextIndex];
      }
      increment = config->NumCellsWidth;
      checkDirection(config, localMapCells, x, y, z, r, g, b, fullIndex, blockCount, bIndex,
            bLocalIndex, blockOffset, startI, increment, bNextIndex, cent, centMod, 0, 1, 0);
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
      if (startGIndex + blockOffsetComp[bLocalIndex] + i < config->MaxPoints) {
         ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = x[blockOffset + i];
         ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = y[blockOffset + i];
         ps[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = z[blockOffset + i];
         cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = r[blockOffset + i];
         cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = g[blockOffset + i];
         cols[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = b[blockOffset + i];
      
         if (extractNorms) {
            float3 p = (float3)(x[blockOffset + i],y[blockOffset + i],z[blockOffset + i]);
            int blockI = getBlockIndex(config, p, cent, centMod);
            int cellI = getCellIndex(config, p);

            float3 normal = getNormal(config, blocks, localMapCells, 
                  cent, blockI, cellI);
            normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3] = normal.x;
            normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 1] = normal.y;
            normals[(startGIndex + blockOffsetComp[bLocalIndex] + i) * 3 + 2] = normal.z;
         }
      }
      if (fullIndex[blockOffset + i] >= 0 && bIndex>= 0) {
         localMapCells[bIndex].pI[fullIndex[blockOffset + i]] = -1;
      }
   }
}

__kernel void transformPoints(global oclLocalMapCommon *common,
      global float *ps, global float *normals, const int transformNormals,
      const int numPoints, const float3 origin,
      const float3 rotation0, const float3 rotation1, const float3 rotation2) {

   int index = get_global_id(0);
   
   if (index == 0) {
      common->numPoints = 0;
   }

   if (index < numPoints && !isnan(ps[index * 3])) {
      float3 p = (float3)(ps[index * 3], ps[index * 3 + 1], ps[index * 3 + 2]);
      p = transformPoint(p, origin, rotation0, rotation1, rotation2);
      ps[index * 3] = p.x;
      ps[index * 3 + 1] = p.y;
      ps[index * 3 + 2] = p.z;
      
      if (transformNormals) {
         p = (float3)(normals[index * 3], normals[index * 3 + 1], normals[index * 3 + 2]);
         float3 zero = (float3)(0.0f, 0.0f, 0.0f);
         p = transformPoint(p, zero, rotation0, rotation1, rotation2);
         normals[index * 3] = p.x;
         normals[index * 3 + 1] = p.y;
         normals[index * 3 + 2] = p.z;
      }
   }
}

__kernel void clearBlocks(constant oclPositionTrackConfig *config, 
      global int *blocks, global oclLocalBlock *localMapCells, 
      global oclLocalMapCommon *common, const int numToDelete) {

   int index = get_global_id(0);

   if (index == 0) {
      common->numBlocksToDelete = 0;
   }

   if (index < numToDelete) {
      int bIndex = common->blocksToDelete[index];
      int blockIndex = localMapCells[bIndex].blockIndex;
      blocks[blockIndex] = -1;
      int ret = atomic_dec(&(common->nextEmptyBlock));
      common->emptyBlocks[ret - 1] = bIndex;

      //reset everything in localMapCells[bIndex];
      for (int i = 0; i < config->NumCellsTotal; i++) {
         localMapCells[bIndex].distance[i] = NAN;
         localMapCells[bIndex].weight[i] = 0;
         localMapCells[bIndex].r[i] = 0;
         localMapCells[bIndex].g[i] = 0;
         localMapCells[bIndex].b[i] = 0;
         localMapCells[bIndex].occupied[i] = 0;
         localMapCells[bIndex].pI[i] = -1;
      }
      localMapCells[bIndex].blockIndex = -1;
      localMapCells[bIndex].haveExtracted = 0;
   }
}

__kernel void bilateralFilter(constant oclPositionTrackConfig *config,
      global float *depthP, global float *depthOut, const int numPoints) {

   int index = get_global_id(0);

   int windowSize = 4;
   float scale = 5.0f; //it is really scale^2

   if (index < numPoints) {
      int u = index % config->ImageWidth;
      int v = index / config->ImageWidth;
      if (isnan(depthP[index]) || u - windowSize < 0 || u + windowSize >= config->ImageWidth ||
            v - windowSize < 0 || v + windowSize >= config->ImageHeight) {
         depthOut[index] = NAN;
      } else {
         int i,j;
         float sumWeight = 0.0f;
         float depth = 0.0f;
         int valid = 1;
         for (j = v - windowSize; j <= v + windowSize && valid == 1; j++) {
            for (i = u - windowSize; i <= u + windowSize && valid == 1; i++) {
               int iC = j * config->ImageWidth + i;
               if (isnan(depthP[iC])) {
                  valid = 0;
               } else {
                  float diff = depthP[index] - depthP[iC];
                  float weight = exp(-(((u-i)*(u-i)+(v-j)*(v-j))/scale)-((diff * diff)/scale));
                  sumWeight += weight;
                  depth += depthP[iC] * weight;
               }
            }
         }
         if (valid == 1) {
            depthOut[index] = depth /= sumWeight;
         } else {
            depthOut[index] = NAN;
         }
      }
   }
}

__kernel void calculateNormals(constant oclPositionTrackConfig *config, 
   global oclDepthPoints *points, global oclDepthPoints *normals,
   global float *depthP, const int numPoints) {

   int index = get_global_id(0);

   if (index < numPoints && !isnan(depthP[index])) {
      int u = index % config->ImageWidth;
      int v = index / config->ImageWidth;
      int xPlusInd = index + 1;
      int yPlusInd = index + config->ImageWidth;
      if (u < config->ImageWidth - 1 && v < config->ImageHeight - 1 &&
            !isnan(depthP[xPlusInd]) && !isnan(depthP[yPlusInd])) {
         
         float3 point = convertPixelToPoint(config, u, v, depthP[index]);
         float3 pointXPlus = convertPixelToPoint(config, u + 1, v, depthP[xPlusInd]);
         float3 pointYPlus = convertPixelToPoint(config, u, v + 1, depthP[yPlusInd]);
         float3 crosA;
         crosA.x = pointXPlus.x - point.x;
         crosA.y = pointXPlus.y - point.y;
         crosA.z = pointXPlus.z - point.z;
         float3 crosB;
         crosB.x = pointYPlus.x - point.x;
         crosB.y = pointYPlus.y - point.y;
         crosB.z = pointYPlus.z - point.z;
         float3 res = cross(crosB, crosA);
         crosA = fast_normalize(res);

         points->x[index] = point.x;
         points->y[index] = point.y;
         points->z[index] = point.z;
         normals->x[index] = crosA.x;
         normals->y[index] = crosA.y;
         normals->z[index] = crosA.z;

      } else {
         points->x[index] = NAN;
      }
   } else if (index < numPoints) {
      points->x[index] = NAN;
   }
}

/*
 * Perform fast ICP method on map points
 */
__kernel void fastICP(constant oclPositionTrackConfig *config, global int *blocks,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common,
      global oclDepthPoints *points, global oclDepthPoints *normals, 
      global float *tempStore, const int numPoints, const int numGroups,
      const int3 cent, const float3 origin, const float3 rotation0, 
      const float3 rotation1, const float3 rotation2) {

   int gIndex = get_global_id(0);
   int lIndex = get_local_id(0);
   int wIndex = lIndex % WARP_SIZE;

   int groupNum = get_group_id(0);

   local float results[WARP_SIZE][NUM_RESULTS];
   local int goodCount[WARP_SIZE];
   int i;

   if (lIndex < WARP_SIZE) {
      for (i = 0; i < NUM_RESULTS; i++) {
         results[lIndex][i] = 0.0f;
      }
      goodCount[lIndex] = 0;
   }
   barrier(CLK_LOCAL_MEM_FENCE);

   /*int r = gIndex / config->ImageWidth;
   int c = gIndex % config->ImageWidth;
   r *= 2;
   c *= 2;
   if (c >= config->ImageWidth) {
      gIndex = numPoints + 1;
   } else {
      gIndex = r * config->ImageWidth + c;
   }*/

   if (gIndex < numPoints && !isnan(points->x[gIndex])) {
      int3 centMod;
      centMod.x = cent.x % config->NumBlocksWidth;
      centMod.y = cent.y % config->NumBlocksWidth;
      centMod.z = cent.z % config->NumBlocksHeight;

      float3 point = (float3)(points->x[gIndex], points->y[gIndex], points->z[gIndex]);
      float3 transP = transformPoint(point, origin, rotation0, rotation1, rotation2);
      float3 zero = (float3) (0.0f, 0.0f, 0.0f);
      float3 frameNormal = (float3)(normals->x[gIndex], normals->y[gIndex], normals->z[gIndex]);
      frameNormal = transformPoint(frameNormal, zero, rotation0, rotation1, rotation2);
      frameNormal = fast_normalize(frameNormal);

      int bIndex = getBlockIndex(config, transP, cent, centMod);

      if (bIndex >= 0 && blocks[bIndex] >= 0) {
         int bI = blocks[bIndex];
         int cIndex = getCellIndex(config, transP);

         float3 normal = getNormal(config, blocks, localMapCells, 
                  cent, bIndex, cIndex);
         /*float length = normal.x * normal.x + normal.y * normal.y + normal.z * normal.z;
         length = sqrt((float)9);
         normal /= length;
         //normal = fast_normalize(normal);*/
         if (!isnan(normal.x) && !isnan(localMapCells[bI].distance[cIndex])) {

            float3 vm = transP - (normal * localMapCells[bI].distance[cIndex]);

            int bIndexNew = getBlockIndex(config, vm, cent, centMod);
            if (bIndexNew >= 0 && blocks[bIndexNew] >= 0) {
               int cIndexNew = getCellIndex(config, vm);
               int bINew = blocks[bIndex];

               float3 newNormal = getNormal(config, blocks, localMapCells, cent, bIndexNew, cIndexNew);

               if (fabs(localMapCells[bINew].distance[cIndexNew]) <
                     fabs(localMapCells[bI].distance[cIndex]) &&
                     fabs(localMapCells[bINew].distance[cIndexNew]) < 0.025f &&
                     //fabs(localMapCells[bI].distance[cIndex]) < 0.3f &&
                     dot(frameNormal, newNormal) > 0.8f) {


            //   normal = getNormal(config, blocks, localMapCells, cent, bIndexNew, cIndexNew);
            //   if (!isnan(normal.x) /*&& fabs(localMapCells[bI].distance[cIndex]) > 
            //         fabs(localMapCells[bINew].distance[cIndexNew]) &&
            //         localMapCells[bINew].occupied[cIndexNew] > 0*/) {
                  //vm = vm - (normal * localMapCells[bINew].distance[cIndexNew]);
            

            //test code
            /*float3 vector = normalize(transP - origin);
            float3 startPos = transP - vector * 0.2f;
            int sign = 0;
            int found = 0;
            for (i = 0; i < 32 && found == 0; i++, startPos += 0.0125f * vector) {
               int bb = getBlockIndex(config, startPos, cent, centMod);
               if (bb >= 0 && blocks[bb] >= 0) {
                  int cc = getCellIndex(config, startPos);
                  float dist = localMapCells[blocks[bb]].distance[cc];
                  if (isnan(dist)) {
                     continue;
                  }
                  if (sign == 0 && dist > 0) {
                     sign = 1;
                  } else if (sign == 0) {
                     sign = -1;
                  } else if (sign > 0 && dist < 0) {
                     found = 1;
                     break;
                  } else if (sign < 0 && dist > 0) {
                     found = 1;
                     break;
                  }

               } 
            }
            if (found == 1) {
               int bb = getBlockIndex(config, startPos, cent, centMod);
               int cc = getCellIndex(config, startPos);
               normal = getNormal(config, blocks, localMapCells, cent, bb, cc);
               vm = startPos;*/


            //if (1/*fabs(localMapCells[bI].distance[cIndex]) < 0.2f &&
            //      frameNormal.x * normal.x + frameNormal.y * normal.y +
            //      frameNormal.z * normal.z > 0.75f*/) {


            /*float a = normal.y * transP.z - normal.z * transP.y;
            float b = -normal.x * transP.z + normal.z * transP.x;
            float c = normal.x * transP.y - normal.y * transP.x;*/
            float a = normal.z * transP.y - normal.y * transP.z;
            float b = normal.x * transP.z - normal.z * transP.x;
            float c = normal.y * transP.x - normal.x * transP.y;
            float d = normal.x;
            float e = normal.y;
            float f = normal.z;
            float normScale = normal.x * (vm.x - transP.x) + 
                              normal.y * (vm.y - transP.y) +
                              normal.z * (vm.z - transP.z);

            atomicFloatAddLocal(&(results[wIndex][0]), a*a);
            atomicFloatAddLocal(&(results[wIndex][1]), a*b);
            atomicFloatAddLocal(&(results[wIndex][2]), b*b);
            atomicFloatAddLocal(&(results[wIndex][3]), a*c);
            atomicFloatAddLocal(&(results[wIndex][4]), c*b);
            atomicFloatAddLocal(&(results[wIndex][5]), c*c);
            atomicFloatAddLocal(&(results[wIndex][6]), a*d);
            atomicFloatAddLocal(&(results[wIndex][7]), d*b);
            atomicFloatAddLocal(&(results[wIndex][8]), d*c);
            atomicFloatAddLocal(&(results[wIndex][9]), d*d);
            atomicFloatAddLocal(&(results[wIndex][10]), a*e);
            atomicFloatAddLocal(&(results[wIndex][11]), b*e);
            atomicFloatAddLocal(&(results[wIndex][12]), c*e);
            atomicFloatAddLocal(&(results[wIndex][13]), d*e);
            atomicFloatAddLocal(&(results[wIndex][14]), e*e);
            atomicFloatAddLocal(&(results[wIndex][15]), a*f);
            atomicFloatAddLocal(&(results[wIndex][16]), b*f);
            atomicFloatAddLocal(&(results[wIndex][17]), c*f);
            atomicFloatAddLocal(&(results[wIndex][18]), d*f);
            atomicFloatAddLocal(&(results[wIndex][19]), e*f);
            atomicFloatAddLocal(&(results[wIndex][20]), f*f);
            atomicFloatAddLocal(&(results[wIndex][21]), normScale * a);
            atomicFloatAddLocal(&(results[wIndex][22]), normScale * b);
            atomicFloatAddLocal(&(results[wIndex][23]), normScale * c);
            atomicFloatAddLocal(&(results[wIndex][24]), normScale * d);
            atomicFloatAddLocal(&(results[wIndex][25]), normScale * e);
            atomicFloatAddLocal(&(results[wIndex][26]), normScale * f);

            atomic_inc(&(goodCount[wIndex]));
            atomicFloatAddLocal(&(results[wIndex][27]), localMapCells[bI].distance[cIndex]);
            //}
            }
            }

         }
      }
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < 16 /*&& WARP_SIZE==32*/) {
      for(i = 0; i < NUM_RESULTS; i++) {
         results[lIndex][i] += results[lIndex + 16][i];
      }
      goodCount[lIndex] += goodCount[lIndex + 16];
   }
   if (lIndex < 8) {
      for(i = 0; i < NUM_RESULTS; i++) {
         results[lIndex][i] += results[lIndex + 8][i];
      }
      goodCount[lIndex] += goodCount[lIndex + 8];
   }
   if (lIndex < 4) {
      for(i = 0; i < NUM_RESULTS; i++) {
         results[lIndex][i] += results[lIndex + 4][i];
      }
      goodCount[lIndex] += goodCount[lIndex + 4];
   }
   if (lIndex < 2) {
      for(i = 0; i < NUM_RESULTS; i++) {
         results[lIndex][i] += results[lIndex + 2][i];
      }
      goodCount[lIndex] += goodCount[lIndex + 2];
   }
   barrier(CLK_LOCAL_MEM_FENCE);
   if (lIndex < NUM_RESULTS) {
      tempStore[groupNum + lIndex * numGroups] = results[0][lIndex] + results[1][lIndex];
   }
   if (lIndex == 0) {
      tempStore[groupNum + 28 * numGroups] = goodCount[0] + goodCount[1];
   }

   /*if (lIndex < NUM_RESULTS) {
      atomicFloatAdd(&(common->icpResults[lIndex]), results[0][lIndex] + results[1][lIndex]);
   }
   if (lIndex == 0) {
   //   atomic_add(&(common->goodCount), goodCount[0] + goodCount[1]);
      atomicFloatAdd(&(common->icpResults[28]), goodCount[0] + goodCount[1]);
   }*/
}

kernel void combineICPResults(global oclLocalMapCommon *common, global float *tempStore,
      const int numGroups) {
   int index = get_global_id(0);

   //int numGroups = LOCAL_SIZE / NUM_RESULTS;

   if (index < NUM_RESULTS) {
      float res = 0;
      int start = numGroups * index;
      int i;
      for (i = 0; i < numGroups; i++, start++) {
         res += tempStore[start];
      }
      common->icpResults[index] = res;
   }   
}
      
