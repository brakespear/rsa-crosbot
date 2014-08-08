/*
 * depthPoints.cpp
 *
 * Created on: 08/08/2014
 *     Author: adrianr
 */

DepthPoints::DepthPoints(const vector<uint8_t>&points, int width, int height, double minDist, double maxDist, int skip) {
   int numPoints = (width / skip) * (height / skip);
   points.resize(numPoints);
   colour.resize(numPoints);

   int numAdded = 0;

   int skipSize = 32 * width;
   int row, col;
   for (row = 0; row < height; row += skip) {
      int off = skipSize * row;
      for (col = 0; col < width; col++) {
         //Do filtering here before lose pixel coordinate
         //Then test if point is valid
         //Then filter out if < minDist or > maxDist
         //The save in points and colour arrays
      }
   }
   //Resize points and colour arrays

}

void DepthPoints::transform(tf::Transform trans) {
  for(int i = 0; i < points.size(); i++) {
     points[i] = trans * points[i].toTF();
  }
}

void DepthPoints::transform(Pose correction) {
  transform(correction.toTF());
}

