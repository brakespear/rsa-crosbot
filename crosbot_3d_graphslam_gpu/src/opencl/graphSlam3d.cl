
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
      localMapCells[blockIndex].distance[cellIndex] = -1;
      localMapCells[blockIndex].weight[cellIndex] = 0;
      localMapCells[blockIndex].r[cellIndex] = 0;
      localMapCells[blockIndex].g[cellIndex] = 0;
      localMapCells[blockIndex].b[cellIndex] = 0;
   }

   if (index == 0) {
      common->numBlocks = 0;
   }
}

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

__kernel void checkBlocksExist(constant oclGraphSlam3DConfig *config,
      global int *blocks, global oclLocalMapCommon *common,
      global oclDepthPoints *points,
      const int numPoints, const float3 origin,
      const float3 rotation0, const float3 rotation1, const float3 rotation2) {

   int index = get_global_id(0);

   //atomic_add numblockallocated block if >= 0
   //otherwise atomic_dec.
   //if get ret val in range -1 - numblockalloced (ie this thread was the first one,
   //atomic inc numactiveblocks and add the active block (store index in blocks array)

}

__kernel void addRequiredBlocks(constant oclGraphSlam3dConfig *config,
      global int *blocks, global oclLocalMap *common) {
   //Go through all active blocks
   //Look up block in blocks array
   //if < 0, create new block, set index, increment numblocks
   //otherwise mod by numblockallocated to get back in proper range

   //change value in active blocks array to index of localblock

}

__kernel void addFrame(constant oclGraphSlam3DConfig *config,
      global oclLocalBlock *localMapCells, global oclLocalMapCommon *common, 
      global oclDepthPoints *points, const int numActiveBlocks,
      const float3 origin, const float3 rotation0, 
      const float3 rotation1, const float3 rotation2) {

   int index = get_global_id(0);
   if (index == 0) {
      common->numActiveBlocks = 0;
   }

   //Go through all cells in all active blocks, setting the tsdf

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
