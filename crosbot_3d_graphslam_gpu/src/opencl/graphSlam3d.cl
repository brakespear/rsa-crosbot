
#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

//Each float3 of rotation is a row of the rotation matrix
__kernel void transform3D(global oclDepthPoints *points, const int numPoints, const float3 origin, const float3 rotation0, const float3 rotation1, const float3 rotation2) {
   int index = get_global_id(0);

   if (index < numPoints) {
      float3 point = (float3)(points->pointX[index], points->pointY[index], points->pointZ[index]);
      float3 temp = point * rotation0;
      points->pointX[index] = temp.x + temp.y + temp.z + origin.x;
      temp = point * rotation1;
      points->pointY[index] = temp.x + temp.y + temp.z + origin.y;
      temp = point * rotation2;
      points->pointZ[index] = temp.x + temp.y + temp.z + origin.z;

   }
}
