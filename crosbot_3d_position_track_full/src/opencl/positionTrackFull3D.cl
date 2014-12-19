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



