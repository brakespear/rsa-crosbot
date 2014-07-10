
//#include "casrobot/include/tasks/positiontrack/openclCommon.h"
//#include "/home/adrianrobolab/groovy_workspace/ratter_graphslam/include/ratter_graphslam/openclCommon.h"
//#include "/home/rescue/groovy_workspace/graphSlam/include/graphSlam/openclCommon.h"
//#include "/home/timothyw/workspace/ros/graphSlam/include/graphSlam/openclCommon.h"

#ifndef M_PI
#define M_PI 3.14159f
#endif
#define WARP_SIZE 32
#define ANGNORM(X) while (X < -M_PI) X += 2.0f*M_PI;while (X > M_PI) X -= 2.0f*M_PI

//This does an atomic add of floats as opencl can only do atomic adds of ints
void atomicFloatAdd(volatile global float *addr, float data) {
   while (data) {
      data = atomic_xchg(addr, data + atomic_xchg(addr, 0.0f));
   }
}

void atomicFloatAddLocal(volatile local float *addr, float data) {
   while (data) {
      data = atomic_xchg(addr, data + atomic_xchg(addr, 0.0f));
   }
}

void atomicFloatMin(volatile global float *addr, float data) {
   float j = atomic_xchg(addr, NAN);
   while (j == NAN) {
      j = atomic_xchg(addr, NAN);
   }
   /*float j = atomic_xchg(addr, -1.0f);
   while (j < 0) {
      j = atomic_xchg(addr, -1.0f);
   }*/
   j = fmin(j, data);
   atomic_xchg(addr, j);
}

void atomicFloatMax(volatile global float *addr, float data) {
   float j = atomic_xchg(addr, NAN);
   while (j == NAN) {
      j = atomic_xchg(addr, NAN);
   }
   j = fmax(j, data);
   atomic_xchg(addr, j);
}

float calcDeterminant(global float m[3][3]) {

   return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
          m[0][1] * (m[2][2] * m[1][0] - m[1][2] * m[2][0]) +
          m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

float invert3x3Matrix(global float m[3][3], local float res[3][3], int index) {
   if (index < 9) {
      int y = index / 3;
      int x = index % 3;

      int minx = min((x + 1) % 3, (x + 2) % 3);
      int maxx = max((x + 1) % 3, (x + 2) % 3);
      int miny = min((y + 1) % 3, (y + 2) % 3);
      int maxy = max((y + 1) % 3, (y + 2) % 3);

      float temp = m[miny][minx] * m[maxy][maxx] - m[miny][maxx] * m[maxy][minx];
      if ((x == 1 || y == 1) && !(x == 1 && y == 1)) {
         temp *= -1;
      }
      float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                  m[0][1] * (m[2][2] * m[1][0] - m[1][2] * m[2][0]) +
                  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      if (det != 0) {
         res[x][y] = temp * 1/det;
      } else {
         res[x][y] = 0.0001f;
      }
      return det;
   }
   return 0;
}

float invert3x3MatrixLocal(local float m[3][3], local float res[3][3], int index) {
   if (index < 9) {
      int y = index / 3;
      int x = index % 3;

      int minx = min((x + 1) % 3, (x + 2) % 3);
      int maxx = max((x + 1) % 3, (x + 2) % 3);
      int miny = min((y + 1) % 3, (y + 2) % 3);
      int maxy = max((y + 1) % 3, (y + 2) % 3);

      float temp = m[miny][minx] * m[maxy][maxx] - m[miny][maxx] * m[maxy][minx];
      if ((x == 1 || y == 1) && !(x == 1 && y == 1)) {
         temp *= -1;
      }
      float det = m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) -
                  m[0][1] * (m[2][2] * m[1][0] - m[1][2] * m[2][0]) +
                  m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
      if (det != 0) {
         res[x][y] = temp * 1/det;
      } else {
         res[x][y] = 0.0001f;
      }
      return det;
   }
   return 0;
}

void mult3x3Matrix(global float a[3][3], local float b[3][3], global float res[3][3], int index) {
   if (index < 9) {
      int y = index / 3;
      int x = index % 3;

      res[y][x] = a[y][0] * b[0][x] + a[y][1] * b[1][x]  + a[y][2] * b[2][x];
   }
}

void mult3x3MatrixLocal(local float a[3][3], local float b[3][3], local float res[3][3], int index) {
   if (index < 9) {
      int y = index / 3;
      int x = index % 3;

      res[y][x] = a[y][0] * b[0][x] + a[y][1] * b[1][x]  + a[y][2] * b[2][x];
   }
}

void parallelReduce3x3(local float arr[WARP_SIZE][3][3], int localIndex, 
      global float dest[3][3]) {
   if (localIndex < 16) {
      arr[localIndex][0][0] += arr[localIndex + 16][0][0];
      arr[localIndex][0][1] += arr[localIndex + 16][0][1];
      arr[localIndex][0][2] += arr[localIndex + 16][0][2];
      arr[localIndex][1][0] += arr[localIndex + 16][1][0];
      arr[localIndex][1][1] += arr[localIndex + 16][1][1];
      arr[localIndex][1][2] += arr[localIndex + 16][1][2];
      arr[localIndex][2][0] += arr[localIndex + 16][2][0];
      arr[localIndex][2][1] += arr[localIndex + 16][2][1];
      arr[localIndex][2][2] += arr[localIndex + 16][2][2];
   }
   if (localIndex < 8) {
      arr[localIndex][0][0] += arr[localIndex + 8][0][0];
      arr[localIndex][0][1] += arr[localIndex + 8][0][1];
      arr[localIndex][0][2] += arr[localIndex + 8][0][2];
      arr[localIndex][1][0] += arr[localIndex + 8][1][0];
      arr[localIndex][1][1] += arr[localIndex + 8][1][1];
      arr[localIndex][1][2] += arr[localIndex + 8][1][2];
      arr[localIndex][2][0] += arr[localIndex + 8][2][0];
      arr[localIndex][2][1] += arr[localIndex + 8][2][1];
      arr[localIndex][2][2] += arr[localIndex + 8][2][2];
   }
   if (localIndex < 4) {
      arr[localIndex][0][0] += arr[localIndex + 4][0][0];
      arr[localIndex][0][1] += arr[localIndex + 4][0][1];
      arr[localIndex][0][2] += arr[localIndex + 4][0][2];
      arr[localIndex][1][0] += arr[localIndex + 4][1][0];
      arr[localIndex][1][1] += arr[localIndex + 4][1][1];
      arr[localIndex][1][2] += arr[localIndex + 4][1][2];
      arr[localIndex][2][0] += arr[localIndex + 4][2][0];
      arr[localIndex][2][1] += arr[localIndex + 4][2][1];
      arr[localIndex][2][2] += arr[localIndex + 4][2][2];
   }
   if (localIndex < 2) {
      arr[localIndex][0][0] += arr[localIndex + 2][0][0];
      arr[localIndex][0][1] += arr[localIndex + 2][0][1];
      arr[localIndex][0][2] += arr[localIndex + 2][0][2];
      arr[localIndex][1][0] += arr[localIndex + 2][1][0];
      arr[localIndex][1][1] += arr[localIndex + 2][1][1];
      arr[localIndex][1][2] += arr[localIndex + 2][1][2];
      arr[localIndex][2][0] += arr[localIndex + 2][2][0];
      arr[localIndex][2][1] += arr[localIndex + 2][2][1];
      arr[localIndex][2][2] += arr[localIndex + 2][2][2];
   }
   if (localIndex == 0) {
      arr[localIndex][0][0] += arr[localIndex + 1][0][0];
      arr[localIndex][0][1] += arr[localIndex + 1][0][1];
      arr[localIndex][0][2] += arr[localIndex + 1][0][2];
      arr[localIndex][1][0] += arr[localIndex + 1][1][0];
      arr[localIndex][1][1] += arr[localIndex + 1][1][1];
      arr[localIndex][1][2] += arr[localIndex + 1][1][2];
      arr[localIndex][2][0] += arr[localIndex + 1][2][0];
      arr[localIndex][2][1] += arr[localIndex + 1][2][1];
      arr[localIndex][2][2] += arr[localIndex + 1][2][2];
   }
   if (localIndex < 3) {
      atomicFloatAdd(&(dest[localIndex][0]), arr[0][localIndex][0]);
      atomicFloatAdd(&(dest[localIndex][1]), arr[0][localIndex][1]);
      atomicFloatAdd(&(dest[localIndex][2]), arr[0][localIndex][2]);
   }
}

void parallelReduceInt(local int arr[WARP_SIZE], int localIndex, global int *dest) {
   if (localIndex < 16) {
      arr[localIndex] += arr[localIndex + 16];
   }
   if (localIndex < 8) {
      arr[localIndex] += arr[localIndex + 8];
   }
   if (localIndex < 4) {
      arr[localIndex] += arr[localIndex + 4];
   }
   if (localIndex < 2) {
      arr[localIndex] += arr[localIndex + 2];
   }
   if (localIndex == 0) {
      arr[localIndex] += arr[localIndex + 1];
      atomic_add(dest, arr[localIndex]);
   }
}

void parallelReduceFloat(local float arr[WARP_SIZE], int localIndex, global float *dest) {
   if (localIndex < 16) {
      arr[localIndex] += arr[localIndex + 16];
   }
   if (localIndex < 8) {
      arr[localIndex] += arr[localIndex + 8];
   }
   if (localIndex < 4) {
      arr[localIndex] += arr[localIndex + 4];
   }
   if (localIndex < 2) {
      arr[localIndex] += arr[localIndex + 2];
   }
   if (localIndex == 0) {
      arr[localIndex] += arr[localIndex + 1];
      atomicFloatAdd(dest, arr[localIndex]);
   }
}


