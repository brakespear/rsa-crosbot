/*
 * Ogmbicp.cpp
 *
 * Created on: 16/9/2013
 *     Author: adrianr
 */

#include <crosbot_ogmbicp/Ogmbicp.hpp>

void Ogmbicp::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("MapSize", MapSize, 10);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("CellHeight", CellHeight, 0.05);
   paramNH.param<double>("MagSegLen", MaxSegLen, 0.1);
   MaxSegLen *= MaxSegLen;
   paramNH.param<int>("MaxIterations", MaxIterations, 200);
   paramNH.param<double>("MaxErrorXY", MaxErrorXY, 0.0001);
   paramNH.param<double>("MaxErrorTh", MaxErrorTh, 0.01);
   paramNH.param<double>("MaxErrorZ", MaxErrorZ, 0.001);
   paramNH.param<double>("MinAddHeight", MinAddHeight, 1.2);
   paramNH.param<double>("MaxAddHeight", MaxAddHeight, 3);
   paramNH.param<double>("FloorHeight", FloorHeight, 1);
   paramNH.param<double>("LaserMinDist", LaserMinDist, 0.4);
   paramNH.param<bool>("IgnoreZValues", IgnoreZValues, false);
   paramNH.param<double>("LaserMaxAlign", LaserMaxAlign, -1);
   paramNH.param<bool>("UseVariableL", UseVariableL, true);
   paramNH.param<double>("AlignmentDFix", AlignmentDFix, 7.0);
   paramNH.param<double>("LValue", LValue, 2.0);
   paramNH.param<int>("MinCellCount", MinCellCount, 3);
   paramNH.param<bool>("UseFactor", UseFactor, true);
   paramNH.param<bool>("UseSimpleH", UseSimpleH, false);
   paramNH.param<int>("MaxObservations", MaxObservations, 1000);
   paramNH.param<double>("MinFactor", MinFactor, 0.2);
   paramNH.param<int>("MinGoodCount", MinGoodCount, 5);

}

