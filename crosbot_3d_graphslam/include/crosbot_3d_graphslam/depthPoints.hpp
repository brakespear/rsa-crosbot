/*
 * depthPoints.hpp
 *
 * Created on: 08/08/2014
 *     Author: adrianr
 */

#ifndef DEPTHPOINTS_HPP_
#define DEPTHPOINTS_HPP_

class DepthPoints {
public:
   vector<Point> points;
   vector<Colour> colour;
   //vector normals?
   
   DepthPoints(const vector<uint8_t>&points, int width, int height, double minDist, double maxDist, int skip);

   void transform(Pose correction);
   void transform(tf::Transform trans);

   //calculate normals, filter, etc
};

#endif
