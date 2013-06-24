/*
 * localmap.cpp
 *
 *  Created on: 15/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_map/localmap.hpp>
#include <crosbot/utils.hpp>

#include <sensor_msgs/LaserScan.h>
#include <tf/transform_listener.h>

using namespace crosbot;

#define DEFAULT_MAPFRAME			"/odom"
#define DEFAULT_BASEFRAME			"/base_link"

#define DEFAULT_MAPWIDTH			10.0
#define DEFAULT_MAPHEIGHT			10.0
#define DEFAULT_MAPRESOLUTION		0.05

class LocalMapNode {
	std::string mapFrame, baseFrame;


	LocalMapPtr map;

	double resolution, width, height;

	ros::Subscriber scanSub;
	tf::TransformListener tfListener;

	ros::Publisher imagePub, gridPub;

	Pose previousRobotPose;
public:
	LocalMapNode() :
		mapFrame(DEFAULT_MAPFRAME), baseFrame(DEFAULT_BASEFRAME),
		resolution(DEFAULT_MAPRESOLUTION), width(DEFAULT_MAPWIDTH), height(DEFAULT_MAPHEIGHT)
	{}

    void callbackScan(const sensor_msgs::LaserScanConstPtr& latestScan) {
    	Pose robotPose, sensorPose;

    	tf::StampedTransform transform;
    	try {
    		tfListener.waitForTransform(mapFrame,
    				latestScan->header.frame_id, latestScan->header.stamp,
    				ros::Duration(DEFAULT_MAXWAIT4TRANSFORM));
    		tfListener.lookupTransform(mapFrame,
    				latestScan->header.frame_id, latestScan->header.stamp, transform);
    		sensorPose = transform;
    		tfListener.lookupTransform(mapFrame, baseFrame,
    				latestScan->header.stamp, transform);
    		robotPose = transform;
    	} catch (tf::TransformException& ex) {
    		ERROR("localmap: Error getting transform. (%s)\n", ex.what());
    		return;
    	}

    	// Calculate shift to center map on base frame
        int sX = round(previousRobotPose.position.x / map->resolution),
                sY = round(previousRobotPose.position.y / map->resolution);
        sX -= round(robotPose.position.x / map->resolution);
        sY -= round(robotPose.position.y / map->resolution);
    	map->shift(sX, sY);
    	previousRobotPose = robotPose;
    	// Transform cloud to map frame
    	PointCloud cloud(mapFrame, PointCloud(latestScan), robotPose);

    	map->update(cloud);

    	if (imagePub.getNumSubscribers() > 0) {
    		ImagePtr image = map->getImage(robotPose);
    		imagePub.publish(image->toROS());
    	}
    	if (gridPub.getNumSubscribers() > 0) {
    		// get occupancy grid and publish it
    		gridPub.publish(map->getGrid());
    	}
    }

	void initalise(ros::NodeHandle& nh) {
		// read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("map_frame", mapFrame, DEFAULT_MAPFRAME);
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);

		// read map parameters
		paramNH.param<double>("width", width, DEFAULT_MAPWIDTH);
		paramNH.param<double>("height", height, DEFAULT_MAPHEIGHT);
		paramNH.param<double>("resolution", resolution, DEFAULT_MAPRESOLUTION);

		int cWidth = round(width / resolution), cHeight = round(height / resolution);
		if (cWidth < 1)
			cWidth = 1;
		if (cHeight < 1)
			cHeight = 1;
		map = new LocalMap(cWidth, cHeight, resolution, mapFrame);

		paramNH.param<int>("hit_increment", map->hitIncrement, 1);
		paramNH.param<int>("hit_max", map->maxHits, 100);
		paramNH.param<int>("miss_decrement", map->missDecrement, 1);

		scanSub = nh.subscribe("scan", 1, &LocalMapNode::callbackScan, this);
		imagePub = nh.advertise<sensor_msgs::Image>("image", 1);
		gridPub = nh.advertise<nav_msgs::OccupancyGrid>("localmap", 1);
	}

	void shutdown() {
		scanSub.shutdown();
		imagePub.shutdown();
		gridPub.shutdown();
	}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "localmap");

    ros::NodeHandle nh;

    LocalMapNode node;
    node.initalise(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}
