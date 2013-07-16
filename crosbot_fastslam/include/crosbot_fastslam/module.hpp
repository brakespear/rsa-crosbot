/*
 * module.hpp
 *
 *  Created on: 20/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_FASTSLAM_MODULE_HPP_
#define CROSBOT_FASTSLAM_MODULE_HPP_

#include <ros/ros.h>
#include "fastslam.hpp"

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>

namespace crosbot {

namespace fastslam {

#define DEFAULT_ODOMFRAME			"/icp"
#define DEFAULT_MAPFRAME			"/map"
#define DEFAULT_BASEFRAME			"/base_link"
#define DEFAULT_MAXWAIT4TRANSFORM	2.0			// [s]

class FastSLAMModule;
typedef Handle<FastSLAMModule> FastSLAMModulePtr;
class FastSLAMModule : public FastSLAMMap, virtual public MapListener {
protected:
	std::string mapFrame, odomFrame, baseFrame;

	ros::Subscriber scanSub, snapSub;
	tf::TransformListener tfListener;
	tf::TransformBroadcaster tfBroadcast;

	ros::Publisher gridPub, historyPub;
public:

    void callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
    	Pose robotPose, sensorPose;

    	tf::StampedTransform transform;
    	try {
    		tfListener.waitForTransform(odomFrame,
    				latestScan->header.frame_id, latestScan->header.stamp,
    				ros::Duration(DEFAULT_MAXWAIT4TRANSFORM));
    		tfListener.lookupTransform(baseFrame,
    				latestScan->header.frame_id, latestScan->header.stamp, transform);
    		sensorPose = transform;
    		tfListener.lookupTransform(odomFrame, baseFrame,
    				latestScan->header.stamp, transform);
    		robotPose = transform;
    	} catch (tf::TransformException& ex) {
    		ERROR("FastSLAMModule: Error getting transform. (%s)\n", ex.what());
    		return;
    	}

    	PointCloud pc(latestScan);
    	newPointCloud(new PointCloud(baseFrame, pc, sensorPose.getTransform()), robotPose, sensorPose);
	}

	void callbackSnap(const crosbot_map::SnapMsgConstPtr& snap) {
		// TODO: convert all point clouds to base frame
		addTag(new Snap(snap));
	}

	FastSLAMModule(std::string name = "") : FastSLAMMap(),
			mapFrame(DEFAULT_MAPFRAME), odomFrame(DEFAULT_ODOMFRAME),
			baseFrame(DEFAULT_BASEFRAME)
	{
		addListener(this);
	}

	~FastSLAMModule() {
		removeListener(this);
	}

	void publishTransform(const Pose& robot, const Pose& odometry, const Time& time) {
		geometry_msgs::TransformStamped ts;
		ts.header.stamp = time.toROS();
		ts.header.frame_id = mapFrame;
		ts.child_frame_id = odomFrame;

		Pose pose = robot.getTransform() * odometry.getTransform().inverse();

		ts.transform.translation.x = pose.position.x;
		ts.transform.translation.y = pose.position.y;
		ts.transform.translation.z = pose.position.z;
		ts.transform.rotation = pose.orientation.toROS();
		tfBroadcast.sendTransform(ts);
	}

	void motionTracked(MapPtr map) {
		ParticlePtr newMean = new Particle(mean, true);
		MapCloudPtr update = newMean->getLatestUpdate();
		if (update != NULL) {
			Pose pose = newMean->getPose();
			publishTransform(newMean->getPose(), update->odometry, update->timestamp);
		}
	}

	void mapUpdated(MapPtr map) {
		ParticlePtr newMean = new Particle(mean);

		// publish transform
		MapCloudPtr update = newMean->getLatestUpdate();
		if (update != NULL) {
			Pose pose = newMean->getPose();
			publishTransform(newMean->getPose(), update->odometry, update->timestamp);
		}

		if (gridPub.getNumSubscribers() > 0) {
			nav_msgs::OccupancyGridPtr grid = asOccupancyGrid(newMean);
			if (grid != NULL) {
				grid->header.frame_id = mapFrame;
				grid->header.stamp = ros::Time::now();
				gridPub.publish(grid);
			}
		}

		// TODO: publish history
	}

	void tagAdded(MapPtr map, TagPtr tag) {
		// TODO: FastSLAMModule::tagAdded()
	}

	void tagChanged(MapPtr map, TagPtr tag) {
		// TODO: FastSLAMModule::tagChanged()
	}

	void initialize(ros::NodeHandle& nh) {
		// TODO: read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("map_frame", mapFrame, DEFAULT_MAPFRAME);
		paramNH.param<std::string>("odom_frame", odomFrame, DEFAULT_ODOMFRAME);
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);

		scanSub = nh.subscribe("scan", 1, &FastSLAMModule::callbackScan, this);
		snapSub = nh.subscribe("snap", 1000, &FastSLAMModule::callbackSnap, this);
		gridPub = nh.advertise<nav_msgs::OccupancyGrid>("map", 1);
		historyPub = nh.advertise<nav_msgs::Path>("history", 1);

		start();
	}

	void shutdown() {
		stop();

		scanSub.shutdown();
		snapSub.shutdown();
		gridPub.shutdown();
		historyPub.shutdown();
	}
};

} // namespace fastslam

} // namespace crosbot

#endif /* CROSBOT_FASTSLAM_MODULE_HPP_ */
