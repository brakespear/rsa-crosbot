/*
 * mbicp.cpp
 *
 *  Created on: 19/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include "../mbicpStd.h"
#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>

using namespace crosbot;

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#define DEFAULT_ICPFRAME			"/icp"
#define DEFAULT_BASEFRAME			"/base_link"
#define DEFAULT_ODOMFRAME			""
#define DEFAULT_MAXWAIT4TRANSFORM	2.0			// [s]

class MBICPNode {
	std::string icpFrame, baseFrame, odomFrame;

	ros::Subscriber scanSub;
	tf::TransformListener tfListener;

	tf::TransformBroadcaster tfPub;

	Pose previousPose, previousMove, previousOdom;
	bool useOdometry;

	mbicp::MBICPStd mbicpStd;


	// MBICP Parameters

    // Configurable parameters
    double maxAlignTheta, maxAlignDist;  // MBICP: can't align cells further than this apart
    double l;                           // MBICP: L
    int laserSkip;                      // MBICP: iteration value for laser readings for fine alignment
    int maxIterations;                  // MBICP: Maximum number of iterations
    bool useProjectionFilter;           // MBICP: Activate projection filter
    int iterSmoothCount;                //

    double z;                           // Height of the robot off the ground

    bool mbicpInitialized;
public:
	MBICPNode() :
		icpFrame(DEFAULT_ICPFRAME), baseFrame(DEFAULT_BASEFRAME),
		odomFrame(DEFAULT_ODOMFRAME), tfListener(ros::Duration(100000000,0)), useOdometry(false),
		mbicpInitialized(false)
	{

        maxAlignTheta = 0.4;        // MB: can't align cells further than this apart
        maxAlignDist = 0.3;         // MB: can't align cells further than this apart
        l = 2.0;
        laserSkip = 4;              // MB: iteration value for laser readings for fine alignment
        maxIterations = 200;
        useProjectionFilter = false;
        iterSmoothCount = 2;

        z = 0;
	}

	void populateTscan(mbicp::Tscan& tscan, const sensor_msgs::LaserScanConstPtr& latestScan, const PointCloud& cloud) {
	    double laserMin = latestScan->range_min,
	    		laserMax = latestScan->range_max;

	    tscan.numPuntos = 0;

	    double bearing = latestScan->angle_min;
	    for (size_t i = 0; i < latestScan->ranges.size(); ++i) {
	        const float& r = latestScan->ranges[i];
	        if (r > laserMax || r < laserMin) {
	            bearing += latestScan->angle_increment;
	            continue;
	        }

	        tscan.laserP[tscan.numPuntos].r = r;
	        tscan.laserP[tscan.numPuntos].t = bearing;

	        tscan.laserC[tscan.numPuntos].x = cloud.cloud[i].x;
	        tscan.laserC[tscan.numPuntos].y = cloud.cloud[i].y;

	        tscan.numPuntos++;
	        bearing += latestScan->angle_increment;
	    }
	}

	void initializeICP(const sensor_msgs::LaserScanConstPtr& latestScan, const PointCloud& cloud) {
	    LOG("Initialising Standard MBICP.\n");

	    // Initialise MBICPStd
	    mbicpStd.Init_MbICP_ScanMatching(// maximum laser reading
	        latestScan->range_max, // float max_laser_range,
	        /* Bw: maximum angle diference between points of different scans */
	        /* Points with greater Bw cannot be correspondent (eliminate spurius asoc.) */
	        /* This is a speed up parameter */
	        (float)maxAlignTheta, // float Bw,
	//              config->getAttributeAsDouble("std_angular_window", 1.0), // float Bw,
	        /* Br: maximum distance difference between points of different scans */
	        /* Points with greater Br cannot be correspondent (eliminate spurius asoc.) */
	        (float)maxAlignDist, // float Br,
	        /* L: value of the metric */
	        /* When L tends to infinity you are using the standard ICP */
	        /* When L tends to 0 you use the metric (more importance to rotation
	         * This is the most important parameter */
	        l, // float L,
	        /* laserStep: selects points of each scan with an step laserStep  */
	        /* When laserStep=1 uses all the points of the scans */
	        /* When laserStep=2 uses one each two ... */
	        /* This is an speed up parameter, although not as much as previously when it was using laserStep^2*/
	        laserSkip, // int   laserStep,
	        /* MaxDistInter: maximum distance to interpolate between points in the ref scan */
	        /* Consecutive points with less Euclidean distance than MaxDistInter are considered to be a segment */
	        0.5,// float MaxDistInter,
	        /* filtrado: in [0,1] sets the % of asociations NOT considered spurious
	         * probably don't change this */
	        0.95,// float filter,
	        /* ProjectionFilter: */
	        /* Eliminate the points that cannot be seen given the two scans (see Lu&Millios 97) */
	        /* It works well for angles < 45 \circ*/
	        /* 1 : activates the filter */
	        /* 0 : deactivates the filter
	         * probably don't turn this off */
	        (useProjectionFilter?1:0),// int   ProjectionFilter,
	        /* AsocError: in [0,1] */
	        /* One way to check if the algorithm diverges if to supervise if the number of associations goes below a threshold */
	        /* When the number of associations is below AsocError, the main function will return error in associations step */
	        0.1,// float AsocError,
	        /* MaxIter: sets the maximum number of iterations for the algorithm to exit */
	        /* More iterations more chance you give the algorithm to be more accurate   */
	        maxIterations,// int   MaxIter,
	        /* error_th: in [0,1] sets the maximum error ratio between iterations to exit */
	        /* In each iteration, the error is the residual of the minimization */
	        /* When error_th tends to 1 more precise is the solution of the scan matching */
	        0.0001,// float errorRatio,
	        /* errx_out,erry_out, errt_out: minimum error of the asociations to exit */
	        /* In each iteration, the error is the residual of the minimization in each component */
	        /* The condition is (lower than errx_out && lower than erry_out && lower than errt_out */
	        /* When error_XXX tend to 0 more precise is the solution of the scan matching */
	        0.0001,// float errx_out,
	        0.0001,// float erry_out,
	        0.0001,// float errt_out,
	        /* IterSmoothConv: number of consecutive iterations that satisfity the error criteria */
	        /* (error_th) OR (errorx_out && errory_out && errt_out) */
	        /* With this parameter >1 avoids random solutions */
	        iterSmoothCount,// int IterSmoothConv,
	        // not using these since we cull the points before sending
	        0.0, 0.0, 0.0, 0.0
	    );

	    if (latestScan->ranges.size() > MAXLASERPOINTS) {
	        ROS_ERROR("Number of points in laser scan is larger than can be handled by MBICP.\n");
	        return;
	    }
	    mbicpInitialized = true;
	    populateTscan(mbicpStd.ptosRef, latestScan, cloud);
	}

    void callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
    	Pose odomPose, sensorPose;

    	tf::StampedTransform transform;
    	bool haveOdometry = odomFrame != "";
    	try {
    		tfListener.waitForTransform(baseFrame, latestScan->header.frame_id, latestScan->header.stamp, ros::Duration(1, 0));
    		tfListener.lookupTransform(baseFrame,
    				latestScan->header.frame_id, latestScan->header.stamp, transform);
    		sensorPose = transform;

    		if (haveOdometry) {
        		tfListener.waitForTransform(odomFrame, baseFrame, latestScan->header.stamp, ros::Duration(1, 0));
        		tfListener.lookupTransform(odomFrame, baseFrame,
        				latestScan->header.stamp, transform);
        		odomPose = transform;
    		}
    	} catch (tf::TransformException& ex) {
    		ERROR("mbicp: Error getting transform. (%s) (%d.%d)\n", ex.what(),
    				latestScan->header.stamp.sec, latestScan->header.stamp.nsec);
    		return;
    	}

    	PointCloud cloud(baseFrame, PointCloud(latestScan, true), sensorPose);
    	if (!mbicpInitialized) {
    		initializeICP(latestScan, cloud);
    		if (useOdometry) {
    			previousOdom = odomPose;
    		}
    		return;
    	}

    	// Apply estimate to mbicp
    	double roll, pitch, yaw;
    	if (useOdometry) {
    		// odometry change estimate
    		btTransform prevOdomTInv, odomT;
    		prevOdomTInv = previousOdom.getTransform().inverse();
    		odomT = odomPose.getTransform();

    		odomT = prevOdomTInv * odomT;

    		Pose odomMotion = odomT;
    		odomMotion.getYPR(yaw, pitch, roll);
    		mbicpStd.motion2.x = odomMotion.position.x;
    		mbicpStd.motion2.y = odomMotion.position.y;
    		mbicpStd.motion2.tita = yaw;

    		previousOdom = odomPose;
    	} else {
    		// Use previous mostion for motion estimate
    		previousMove.getYPR(yaw, pitch, roll);
    		mbicpStd.motion2.x = previousMove.position.x;
    		mbicpStd.motion2.y = previousMove.position.y;
    		mbicpStd.motion2.tita = yaw;
    	}

        populateTscan(mbicpStd.ptosNew, latestScan, cloud);

        /**
         *
         */
        double x = 0, y = 0, theta = 0;
        int err = mbicpStd.MbICPMatchScans(x, y, theta);
        if (err != 1) {
            if (err == 2) {
                ERROR("MBICP: MBICP failed(too many iterations (>%d)) ignoring current scan.\n", maxIterations);
            } else {
                ERROR("MBICP: MBICP failed(%d) ignoring current scan.\n", err);
            }
            memcpy(&mbicpStd.ptosRef, &mbicpStd.ptosNew, sizeof(mbicp::Tscan));
        }

        previousMove.position.x = x; previousMove.position.y = y; previousMove.position.z = 0;
        previousMove.orientation.setYPR(theta, 0, 0);

        btTransform poseTransform, moveTransform;
        previousMove.getTransform(moveTransform);
        previousPose.getTransform(poseTransform);
        poseTransform *= moveTransform;
        previousPose = poseTransform;


        if (haveOdometry) {
            LOG("Publishing correction to odometry.\n");
        	Pose p = previousPose.getTransform() * odomPose.getTransform().inverse();
        	publishPose(p, odomFrame, latestScan->header.stamp);
        } else {
            LOG("Publishing pose.\n");
        	publishPose(previousPose, baseFrame, latestScan->header.stamp);
        }

        memcpy(&mbicpStd.ptosRef, &mbicpStd.ptosNew, sizeof(mbicp::Tscan));
    }

    void publishPose(const Pose& pose, std::string childFrame, ros::Time stamp = ros::Time::now()) {
    	geometry_msgs::TransformStamped ts;
    	ts.header.frame_id = icpFrame;
    	ts.header.stamp = stamp;
    	ts.child_frame_id = childFrame;
    	ts.transform.translation.x = pose.position.x;
    	ts.transform.translation.y = pose.position.y;
    	ts.transform.translation.z = pose.position.z;
    	ts.transform.rotation = pose.orientation.toROS();

    	tfPub.sendTransform(ts);
    }

	void initalise(ros::NodeHandle& nh) {
		// read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("icp_frame", icpFrame, DEFAULT_ICPFRAME);
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);
		paramNH.param<std::string>("odom_frame", odomFrame, DEFAULT_ODOMFRAME);

		// TODO: read MBICP parameters

		scanSub = nh.subscribe("scan", 1, &MBICPNode::callbackScan, this);
	}

	void shutdown() {
		scanSub.shutdown();
	}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mbicp");

    ros::NodeHandle nh;

    MBICPNode node;
    node.initalise(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}

