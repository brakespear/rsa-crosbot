/*
 * Ogmbicp.cpp
 *
 * Created on: 16/9/2013
 *     Author: adrianr
 *
 * Common code for all ogmbicp methods    
 */

#include <newmat/newmat.h>
#include <crosbot_ogmbicp/Ogmbicp.hpp>

using namespace NEWMAT;

void Ogmbicp::initialise(ros::NodeHandle &nh) {
   ros::NodeHandle paramNH("~");
   paramNH.param<double>("MapSize", MapSize, 14);
   paramNH.param<double>("CellSize", CellSize, 0.05);
   paramNH.param<double>("CellHeight", CellHeight, 0.05);
   paramNH.param<double>("MaxSegLen", MaxSegLen, 0.1);
   MaxSegLen *= MaxSegLen;
   paramNH.param<int>("MaxIterations", MaxIterations, 50);
   paramNH.param<double>("MaxErrorXY", MaxErrorXY, 0.0001);
   paramNH.param<double>("MaxErrorTh", MaxErrorTh, 0.01);
   paramNH.param<double>("MaxErrorZ", MaxErrorZ, 0.001);
   paramNH.param<double>("MinAddHeight", MinAddHeight, -1);
   paramNH.param<double>("MaxAddHeight", MaxAddHeight, 1);
   paramNH.param<double>("FloorHeight", FloorHeight, -INFINITY);
   paramNH.param<double>("LaserMinDist", LaserMinDist, 0.4);
   paramNH.param<double>("LaserMaxDistance", LaserMaxDistance, 10.0);
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
   paramNH.param<int>("MinGoodCount", MinGoodCount, 10);
   paramNH.param<int>("FullSearchSize", FullSearchSize, 4);
   paramNH.param<int>("NearestAlgorithm", NearestAlgorithm, 1);
   paramNH.param<double>("MaxAlignDistance", MaxAlignDistance, 0.3);
   MaxAlignDistance *= MaxAlignDistance;
   paramNH.param<int>("InitialScans", InitialScans, 20);
   //paramNH.param<double>("LifeRatio", LifeRatio, 2.0);
   paramNH.param<double>("LifeRatio", LifeRatio, 3.0);
   paramNH.param<double>("InitHeight", InitHeight, 0.0);

   //paramNH.param<double>("MaxMoveXYZ", MaxMoveXYZ, 0.1);
   paramNH.param<double>("MaxMoveXYZ", MaxMoveXYZ, 0.2);
   paramNH.param<double>("MaxMoveTh", MaxMoveTh, 0.5);
   paramNH.param<int>("MaxScanSkip", MaxScanSkip, 1);
   paramNH.param<int>("AddSkipCount", AddSkipCount, 50);
   paramNH.param<int>("MaxFail", MaxFail, 5);
   paramNH.param<bool>("UsePriorMove", UsePriorMove, true);
   paramNH.param<int>("ImgTransmitTime", ImgTransmitTime, 50000);
   paramNH.param<double>("ScanListTime", ScanListTime, 20);
   //paramNH.param<bool>("UseIMUOrientation", UseIMUOrientation, true);
   paramNH.param<bool>("DiscardScansOrientation", DiscardScansOrientation, true);
   paramNH.param<double>("DiscardThreshold", DiscardThreshold, 0.6);

}

typedef enum RotationOrder {
	ROTATE_RPY,
	ROTATE_YRP,
	ROTATE_YPR
} RotationOrder;
#define DEFAULT_ROTATION_ORDER		ROTATE_YPR

// set the top 3x3 cells of a matrix from rpy
void getMatFromRot(double r, double p, double y, Matrix &mat, RotationOrder order) {
	double cr, cp, cy;
	double sr, sp, sy;

	cr = cos(r);
	cp = cos(p);
	cy = cos(y);

	sr = sin(r);
	sp = sin(p);
	sy = sin(y);
	switch (order) {
	case ROTATE_RPY:
		// Ry Rx Rz

		mat(1,1) = cr*cy + sr*sp*sy;
		mat(2,1) = cp*sy;
		mat(3,1) = -sr*cy + cr*sp*sy;

		mat(1,2) = -cr*sy + sr*sp*sy;
		mat(2,2) = cp*cy;
		mat(3,2) = sr*sy + cr*sp*sy;

		mat(1,3) = sr*cp;
		mat(2,3) = -sp;
		mat(3,3) = cr*cp;
		break;
	case ROTATE_YRP:
		// Rz Ry Rx, y r p
		mat(1,1) = cy*cr;
		mat(2,1) = sy*cr;
		mat(3,1) = -sr;

		mat(1,2) = cy*sr*sp - sy*cp;
		mat(2,2) = sy*sr*sp + cy*cp;
		mat(3,2) = cr*sp;

		mat(1,3) = cy*sr*cp + sy*sp;
		mat(2,3) = sy*sr*cp - cy*sp;
		mat(3,3) = cr*cp;
		break;
	case ROTATE_YPR:
		// RzRxRy y p r
		mat(1,1) = cr*cy - sr*sp*sy;
		mat(2,1) = cr*sy + sr*sp*cy;
		mat(3,1) = -cp*sr;

		mat(1,2) = -cp*sy;
		mat(2,2) = cp*cy;
		mat(3,2) = sp;

		mat(1,3) = cy*sr + cr*sp*sy;
		mat(2,3) = sr*sy - cr*sp*cy;
		mat(3,3) = cr*cp;
		break;
	}
}

inline void getMatFromTrans(double x, double y, double z, double r, double p, double ya, Matrix &mat, RotationOrder order = DEFAULT_ROTATION_ORDER) {
    getMatFromRot(r, p, ya, mat, order);
    mat(1,4) = x;
    mat(2,4) = y;
    mat(3,4) = z;
    mat(4,4) = 1.0;
    mat(4,1) = 0;
    mat(4,2) = 0;
    mat(4,3) = 0;
}

// invert a transform, uses the rules for transforms, not general matrix inversion
void transInverse(Matrix &trans) {
	int i, j;
	Matrix rot(3,3);
	ColumnVector pos(3);

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			rot(i+1, j+1) = trans(i+1, j+1);
		}
		pos(i+1) = trans(i+1, 4);
	}
	rot = rot.t();
	pos = -rot * pos;

	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			trans(i+1, j+1) = rot(i+1, j+1);
		}
		trans(i+1, 4) = pos(i+1);
	}
}

/**
 * Gets the RPY values from a transformation matrix.
 */
inline void getRotFromMat(double &r, double &p, double &y, Matrix &mat, RotationOrder order = DEFAULT_ROTATION_ORDER) {
	double cp;
	cp = sqrt(mat(3,1)*mat(3,1) + mat(3,3)*mat(3,3));
	r = atan2(-mat(3,1) / cp, mat(3,3) / cp);
	p = atan2(mat(3,2), cp);
	y = atan2(-mat(1,2) / cp, mat(2,2) / cp);
}

inline void getTransFromMat(double &x, double &y, double &z, double &r, double &p, double &ya, Matrix &mat, RotationOrder order = DEFAULT_ROTATION_ORDER) {
	getRotFromMat(r, p, ya, mat, order);
	
	x = mat(1, 4);
	y = mat(2, 4);
	z = mat(3, 4);
}

void Ogmbicp::transformToRobot(double &dx, double &dy, double &dz, double &dth) {
   Matrix lmat(4,4);
	Matrix limat(4,4);
	Matrix tmat(4,4);
	Matrix rmat(4,4);
	// transformation of laser
	getMatFromTrans(dx, dy, dz, 0.0, 0.0, dth, tmat);
	// transformation from robot to laser
	getMatFromTrans(laserPose.position.x, laserPose.position.y, laserPose.position.z, 0.0, 0.0, 0.0, lmat);
	limat = lmat;
	transInverse(limat);
	rmat = lmat * tmat * limat;
   double roll, pitch;
	getTransFromMat(dx, dy, dz, roll, pitch, dth, rmat);
}

crosbot::ImagePtr Ogmbicp::drawMap(LocalMapPtr localMap) {
   Time currentTime = Time::now();
   if (ImgTransmitTime > 0 &&
       (currentTime - lastImgTime).toSec() > ImgTransmitTime / 1000000.0) {
      lastImgTime = currentTime;
      getLocalMap(localMap);
      ImagePtr localMapImage = localMap->getImage();
      return localMapImage;
   } else {
      return NULL;
   }
}

void Ogmbicp::getRecentScans(deque<PointCloudPtr> &recent) {
   recent = recentScans;
}

void Ogmbicp::processImuOrientation(const geometry_msgs::Quaternion& quat) {
   tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
   tf::Matrix3x3 m(q);
   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   /*if (UseIMUOrientation) {
      if (!isOrientationValid) {
         double r, p, y;
         curPose.getYPR(y, p, r);
         yawOffset = yaw - y;
      }
      isOrientationValid = true;
      imuYaw = yaw - yawOffset;
   }*/
   if (DiscardScansOrientation) {
      if (fabs(roll) > DiscardThreshold || fabs(pitch) > DiscardThreshold) {
         discardScan = true;
      } else {
         discardScan = false;
      }
   }
}

