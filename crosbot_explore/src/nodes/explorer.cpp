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
#include <crosbot_explore/FollowPath.h>
#include <crosbot_explore/SetMode.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>

#include <crosbot/config.hpp>

using namespace crosbot;
using namespace crosbot_explore;

#define DEFAULT_BASEFRAME			"/base_link"
#define LOG_START                   "crosbot_explore ::"

class ExplorerNode : public Explorer {
protected:
//	VoronoiGrid::Constraints voronoiConstraints;
	std::string baseFrame;

	ros::Subscriber gridSub, historySub;
	ros::Publisher imagePub, velPub, statusPub;
	tf::TransformListener tfListener;
	ros::ServiceServer waypointSrv, setModeSrv;

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

    bool callbackFollowPath(crosbot_explore::FollowPath::Request& req, crosbot_explore::FollowPath::Response& res) {
    	VoronoiGridPtr voronoi = latestVoronoi;
    	if (voronoi == NULL) {
    		ERROR("explorer: No map to plan on.\n");
    		return false;
    	}

    	tf::StampedTransform transform;
    	try {
		ros::Time now = ros::Time::now();
    		tfListener.waitForTransform(voronoi->frame, req.path.header.frame_id, now, ros::Duration(DEFAULT_MAXWAIT4TRANSFORM));
			tfListener.lookupTransform(voronoi->frame, req.path.header.frame_id, now, transform);
		} catch (tf::TransformException& ex) {
			ERROR("explorer: Error getting transform. (%s)\n", ex.what());
			return false;
		}

		std::vector< Pose > waypoints;
		waypoints.resize(req.path.poses.size());

		for (size_t i = 0; i < waypoints.size(); ++i) {
			waypoints[i] = transform * Pose(req.path.poses[i]).toTF();
		}

		search.setWaypoints(waypoints, getLatestPose());
		search.strategy = SearchParameters::Waypoint;

    	return true;
    }

    bool callbackSetMode(crosbot_explore::SetMode::Request& req, crosbot_explore::SetMode::Response& res) {
    	switch (req.mode) {
    	case 1:
    		resume(); break;
    	case 2:
    		search.side = SearchParameters::Left;
    		search.strategy = SearchParameters::WallFollow; break;
    	case 3:
    		search.side = SearchParameters::Right;
    		search.strategy = SearchParameters::WallFollow; break;
    	case 4:
    		search.strategy = SearchParameters::Waypoint; break;
    	case 0: default:
    		pause(); break;
    	}

    	return true;
    }

	void initalise(ros::NodeHandle& nh) {
		// read configuration/parameters
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("base_frame", baseFrame, DEFAULT_BASEFRAME);

		// TODO: read voronoi constraints
		ConfigElementPtr cfg = new ROSConfigElement(paramNH);
		ConfigElementPtr voronoiCfg = cfg->getChild("voronoi");
		if (voronoiCfg != NULL) {
			voronoiConstraints.restricted = voronoiCfg->getParamAsDouble("restrict", voronoiConstraints.restricted);
			voronoiConstraints.partial = voronoiCfg->getParamAsDouble("partial", voronoiConstraints.partial);
			voronoiConstraints.expand = voronoiCfg->getParamAsDouble("expand", voronoiConstraints.expand);
			voronoiConstraints.orphanThreshold = voronoiCfg->getParamAsInt("orphan", voronoiConstraints.orphanThreshold);
			ROS_INFO("%s restrict: %.2lf - partial: %.2lf - expand: %.2lf - orphan: %d", LOG_START, voronoiConstraints.restricted, voronoiConstraints.partial, voronoiConstraints.expand, voronoiConstraints.orphanThreshold);
		}
		ConfigElementPtr searchCfg = cfg->getChild("search");
		if (searchCfg != NULL) {
			search.searchDistance = searchCfg->getParamAsDouble("search", search.searchDistance);
			search.searchDistance = searchCfg->getParamAsDouble("distance", search.searchDistance);
			search.searchAngle = searchCfg->getParamAsDouble("angle", search.searchAngle);
		}
		if (cfg != NULL) {
		    drive.maxVel = cfg->getParamAsDouble("maxVel", drive.maxVel);
		    drive.maxTurn = cfg->getParamAsDouble("maxTurn", drive.maxTurn);
		}

		gridSub = nh.subscribe("map", 1, &ExplorerNode::callbackOccGrid, this);
		historySub = nh.subscribe("history", 1, &ExplorerNode::callbackHistory, this);
		imagePub = nh.advertise<sensor_msgs::Image>("explore/image", 1);
		velPub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
		statusPub = nh.advertise<std_msgs::String>("status", 1);
		waypointSrv = nh.advertiseService("/explore/follow_path", &ExplorerNode::callbackFollowPath, this);
		setModeSrv = nh.advertiseService("/explore/set_mode", &ExplorerNode::callbackSetMode, this);
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
//      LOG("getLatestPose() - %s, %s\n", mapFrame.c_str(), baseFrame.c_str());

      tf::StampedTransform transform;
		try {
//			tfListener.waitForTransform(mapFrame, baseFrame, ros::Time(0), ros::Duration(5.0));
			tfListener.lookupTransform(mapFrame, baseFrame, ros::Time(0), transform);
		} catch (tf::TransformException& e) {
			LOG("%s getLatestPose(): Exception caught(%s).\n", LOG_START, e.what());
			return Pose(INFINITY, INFINITY, INFINITY);
		}
		return Pose(transform);
	}

	Pose findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot) {
		Pose rval = Explorer::findDriveTarget(voronoi, robot);

		// TODO: to view in camera relative frame need camera_info topic (see http://wiki.ros.org/rviz/DisplayTypes/Camera)
		if (imagePub.getNumSubscribers() > 0) {
			ImagePtr image = getPlanImage(voronoi, robot, rval);
			imagePub.publish(image->toROS());
		}

		return rval;
	}

	bool stopMotors() {
		if (((void*)(velPub)) == NULL)
			return false;

		LOG("%s stopMotors()\n", LOG_START);
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

	virtual void statusChanged(const std::string& status) {
		std_msgs::String msg;
		msg.data = status;
		statusPub.publish(msg);
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



