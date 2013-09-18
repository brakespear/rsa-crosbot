
#include <crosbot_ogmbicp/PointMap3D.hpp>

using namespace crosbot;
using namespace std;

PointMap3D::PointMap3D(double mapSize, double cellSize, double cellHeight): 
   MapSize(mapSize), CellSize(cellSize), CellHeight(cellHeight)
{  
   numWidth = mapSize / cellSize;

   grid.resize(numWidth);
   int i, j;
   for(i = 0; i < numWidth; i++) {
      grid[i] = new deque<Cell3DColumn *>();
      grid[i]->resize(numWidth);
      for (j = 0; j < numWidth; j++) {
         (*grid[i])[j] = new Cell3DColumn();
      }
   }
}




