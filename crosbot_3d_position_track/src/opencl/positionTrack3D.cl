
#define WARP_SIZE 32
#ifndef M_PI
#define M_PI 3.14159f
#endif
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

//Each float3 of rotation is a row of the rotation matrix
__kernel void transform3D(global oclDepthPoints *points, const int numPoints, float3 origin,
      float3 rotation[3]) {
   int index = get_global_id(0);

   //http://docs.ros.org/hydro/api/tf/html/c++/classtf_1_1Transform.html#a9eecdf07a46b63e55b58bbd19492299a
   //tf stores position as a vector (m_origin)
   //and stores orientation as a 3x3 matrix (m_basis)

   //multiplying a 3d point by the transformation
   //in my code do: trans * point.toTF()
   //point.toTF() is a vector.

   if (index < numPoints) {
      float3 point = (float3)(points->pointX[index], points->pointY[index], points->pointZ[index]);
      float3 temp = point * rotation[0];
      points->pointX[index] = temp.x + temp.y + temp.z + origin.x;
      temp = point * rotation[1];
      points->pointY[index] = temp.x + temp.y + temp.z + origin.y;
      temp = point * rotation[2];
      points->pointz[index] = temp.x + temp.y + temp.z + origin.z;

   }
}
