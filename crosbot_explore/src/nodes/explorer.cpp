/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_explore/explorer.hpp>
#include <crosbot/utils.hpp>

#include <tf/transform_listener.h>

#include <crosbot_explore/GetPath.h>
#include <geometry_msgs/Twist.h>

using namespace crosbot;
using namespace crosbot_explore;

#define DEFAULT_BASEFRAME			"/base_link"

class ExplorerNode : public Explorer {
protected:
	VoronoiGrid::Constraints voronoiConstraints;
	std::string baseFrame;

	ros::Subscriber gridSub, historySub;
	ros::Publisher imagePub, velPub;
	tf::TransformListener tfListener;

	ReadWriteLock rosLock;
	nav_msgs::OccupancyGridConstPtr latestMap;
	nav_msgs::PathConstPtr latestHistory;

	ReadWriteLock voronoiLock;
	VoronoiGridPtr latestVoronoi;
public:
	ExplorerNode() : baseFrame(DEFAULT_BASEFRAME) {
		planThread_.start();
		driveThread_.start();
	}

    void callbackOccGrid(const nav_msgs::OccupancyGridConstPtr& latestMap) {
    	Lock lock(rosLock, true);
    	this->latestMap = latestMap;
    }

    void callbackHistory(const nav_msgs::PathConstPtr& latestHistory) {
    	Lock lock(rosLock, true);
    	this->latestHistory = latestHistory;
    }

	void initalise(ros::NodeHandle& nh) {
		// read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);

		// TODO: read voronoi constraints

		gridSub = nh.subscribe("map", 1, &ExplorerNode::callbackOccGrid, this);
		historySub = nh.subscribe("history", 1, &ExplorerNode::callbackHistory, this);
		imagePub = nh.advertise<sensor_msgs::Image>("image", 1);
		velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	}

	void shutdown() {
		Explorer::shutdown();

		gridSub.shutdown();
		historySub.shutdown();
		imagePub.shutdown();
		velPub.shutdown();
	}

	VoronoiGridPtr getLatestVoronoi() {
		nav_msgs::OccupancyGridConstPtr latestMap = this->latestMap;
		if (latestMap == NULL)
			return NULL;
		if (latestVoronoi != NULL && latestVoronoi->timestamp == Time(latestMap->header.stamp)) {
			return latestVoronoi;
		}

		Pose robot = getLatestPose();
		latestVoronoi = new VoronoiGrid(*latestMap, voronoiConstraints, robot); // TODO: include history
		return latestVoronoi;
	}

	Pose getLatestPose() {
		if (latestMap == NULL) {
			return Pose(INFINITY, INFINITY, INFINITY);
		}
		std::string mapFrame = latestMap->header.frame_id;
		tf::StampedTransform transform;
		try {
			tfListener.waitForTransform(mapFrame, baseFrame, ros::Time(0), ros::Duration(2));
			tfListener.lookupTransform(mapFrame, baseFrame, ros::Time(0), transform);
		} catch (tf::TransformException& e) {
			LOG("ExplorerNode::getLatestPose(): Exception caught(%s).\n", e.what());
			return Pose(INFINITY, INFINITY, INFINITY);
		}
		return Pose(transform);
	}

	Pose findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot) {
		Pose rval = Explorer::findDriveTarget(voronoi, robot);

		if (imagePub.getNumSubscribers() > 0) {
			ImagePtr image = getPlanImage(voronoi, robot, rval);
			imagePub.publish(image->toROS());
		}

		return rval;
	}

	bool stopMotors() {
		if (((void*)(velPub)) == NULL)
			return false;

		LOG("ExplorerNode::stopMotors()\n");
		geometry_msgs::Twist twist;

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		velPub.publish(twist);

		return true;
	}

	void driveTo(const Pose& relativePosition) {
		if (((void*)(velPub)) == NULL)
			return;

		double d = relativePosition.position.distanceTo(Point()),
				a = atan2(relativePosition.position.y, relativePosition.position.x);


		geometry_msgs::Twist twist;

		twist.linear.x = twist.linear.y = twist.linear.z = 0;
		twist.angular.x = twist.angular.y = twist.angular.z = 0;

		if (fabs(a) >= drive.turnOnly) {
			twist.angular.z = (a<0)?-drive.maxTurn:drive.maxTurn;
		} else {
			twist.angular.z = a / drive.turnOnly * drive.maxTurn;

			twist.linear.x = (d / search.searchDistance) * drive.maxVel * (1 - (fabs(a) / drive.turnOnly));
			if (twist.linear.x > drive.maxVel) {
				twist.linear.x = drive.maxVel;
			}
		}
		velPub.publish(twist);
	}
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "explorer");

    ros::NodeHandle nh;

    ExplorerNode node;
    node.initalise(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}



