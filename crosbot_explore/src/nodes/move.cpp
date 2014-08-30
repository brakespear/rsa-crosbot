/*
 * move.cpp
 *
 *  Created on: 27/08/2014
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_explore/astar.hpp>
#include <crosbot_map/voronoi.hpp>
#include <crosbot/thread.hpp>
#include <crosbot/config.hpp>


#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <crosbot_explore/SetMode.h>
#include <crosbot_explore/FollowPath.h>


namespace crosbot {

#define DEFAULT_BASEFRAME			"base_link"

class MoveNode {
protected:
	ros::Subscriber goalSub, mapSub;

	class PlanningThread : public Thread {
	public:
		Pose goal;
		nav_msgs::OccupancyGridConstPtr latestMap;
		tf::TransformListener tfListener;
		ReadWriteLock dataLock;

		crosbot::AStarPlanner planner;
		bool operating, explorerPaused;
		ros::Publisher debugPub, pathPub;
		ros::ServiceClient followPathSrv, setModeSrv;

		VoronoiGrid::Constraints voronoiConstraints;
		std::string baseFrame;

		uint32_t updateInterval; // Microseconds
		double distanceThreshold, angleThreshold;
		PlanningThread() :
			goal(INFINITY, INFINITY, INFINITY), operating(true), explorerPaused(false),
			updateInterval(150000),
			distanceThreshold(0.5), angleThreshold(DEG2RAD(10))
		{}

		void setGoal(const Pose& goal) {
			Lock lock(dataLock, true);
			pauseExplorer();
			this->goal = goal;
		}

		void setMap(const nav_msgs::OccupancyGridConstPtr& map) {
			Lock lock(dataLock, true);
			latestMap = map;
		}

		void pauseExplorer() {
			if (!explorerPaused && setMode(0))
				explorerPaused = true;
		}

		bool setMode(int mode) {
			if (!((void *)setModeSrv)) {
				ros::NodeHandle nh;
				setModeSrv = nh.serviceClient< crosbot_explore::SetMode >("/explore/set_mode", true);
			}

			if ((void *)setModeSrv) {
				crosbot_explore::SetMode::Request req;
				crosbot_explore::SetMode::Response res;

				req.mode = mode;
				return setModeSrv.call(req, res);
			}
			return false;
		}

		void sendPath(const Plan& path, const std::string& frameId) {
			if (!((void *)followPathSrv)) {
				ros::NodeHandle nh;
				followPathSrv = nh.serviceClient< crosbot_explore::FollowPath >("/explore/follow_path", true);
			}

			if ((void *)followPathSrv) {
				crosbot_explore::FollowPath::Request req;
				crosbot_explore::FollowPath::Response res;
				req.path.header.frame_id = frameId;
				req.path.poses.resize(path.size());
				for (size_t i = 0; i < path.size(); ++i) {
					req.path.poses[i].pose = path[i].toROS();
				}

				if (followPathSrv.call(req, res)) {
					setMode(1);
				}
			}
		}

#define SLEEP_INTERVAL		50000
		void run() {
			VoronoiGridPtr voronoi;

			ros::Time nextUpdate = ros::Time::now();
			Pose currentPose(INFINITY, INFINITY, INFINITY);
			while (operating) {
				ros::Time now = ros::Time::now();
				if (now < nextUpdate) {
					usleep(SLEEP_INTERVAL);
					continue;
				}

				Pose goal;
				nav_msgs::OccupancyGridConstPtr currentMap;
				{{
					Lock lock(dataLock);
					goal = this->goal;
					if (currentMap != latestMap) {
						voronoi = NULL;
						currentMap = latestMap;
					}
				}}

				if (latestMap == NULL || !goal.isFinite()) {
					pauseExplorer();
					usleep(SLEEP_INTERVAL);
					continue;
				}

				tf::StampedTransform transform;
				try {
					tfListener.lookupTransform(currentMap->header.frame_id, baseFrame, ros::Time(0), transform);
					currentPose = transform;
				} catch (tf::TransformException& te) {
					ROS_WARN("Unable to determine current pose: %s", te.what());
					continue;
				}

				ImagePtr debugImage;
				if (voronoi == NULL) {
					voronoi = new VoronoiGrid(*currentMap, voronoiConstraints, currentPose);
				}

				if ((void *)debugPub) {
					debugImage = voronoi->getImage();
				}

				if (currentPose.position.distanceTo(goal.position) < distanceThreshold) {
					// At the position so just rotate.
					ROS_INFO("Arrived at %.3lf, %.3lf, %.3lf.", goal.position.x, goal.position.y, goal.position.z);

					// TODO: Rotate into position
					{{
						Lock lock(dataLock, true);
						if (this->goal == goal)
							this->goal = Pose(INFINITY, INFINITY, INFINITY);
					}}
					continue;
				}

				// plan to goal
				Plan plan = planner.getPath(voronoi, currentPose, goal, debugImage);
				if (plan.size() > 0) {
					// send path to explorer
					sendPath(plan, voronoi->frame);
				} else {
					ROS_WARN("Unable to plan a path to %.3lf, %.3lf, %.3lf.", goal.position.x, goal.position.y, goal.position.z);
				}

				if (((void *)debugPub) && debugImage != NULL) {
					debugPub.publish(debugImage->toROS());
				}

				nextUpdate += ros::Duration(0, updateInterval*1000);
				if (nextUpdate < now) {
					nextUpdate = now;
				}
			}

			pauseExplorer();
		}


	};
	PlanningThread thread;

public:
	void callbackGoal(const geometry_msgs::PoseStamped& goal) {
		// set goal in planning thread
		ROS_WARN("New goal received.");
		thread.setGoal(goal);
	}

	void callbackMap(const nav_msgs::OccupancyGridConstPtr& map) {
		thread.setMap(map);
	}

	void initalise(ros::NodeHandle& nh) {
		// read configuration/parameters
		ROS_INFO("Initialising");
		ros::NodeHandle paramNH("~");	// Because ROS's search order is buggered
		paramNH.param<std::string>("base_frame", thread.baseFrame, DEFAULT_BASEFRAME);
		ConfigElementPtr cfg = new ROSConfigElement(paramNH);

		// read voronoi constraints
		ConfigElementPtr voronoiCfg = cfg->getChild("voronoi");
		VoronoiGrid::Constraints& voronoiConstraints = thread.voronoiConstraints;
		if (voronoiCfg != NULL) {
				voronoiConstraints.restricted = voronoiCfg->getParamAsDouble("restrict", voronoiConstraints.restricted);
				voronoiConstraints.partial = voronoiCfg->getParamAsDouble("partial", voronoiConstraints.partial);
				voronoiConstraints.expand = voronoiCfg->getParamAsDouble("expand", voronoiConstraints.expand);
				voronoiConstraints.orphanThreshold = voronoiCfg->getParamAsInt("orphan", voronoiConstraints.orphanThreshold);
		}

		mapSub = nh.subscribe("map", 1, &MoveNode::callbackMap, this);
		goalSub = nh.subscribe("goal", 1, &MoveNode::callbackGoal, this);
		thread.debugPub = nh.advertise< sensor_msgs::Image >("debug", 1);

		thread.start();
	}

	void shutdown() {
		thread.operating = false;
		for (uint32_t i = 0; i < DEFAULT_WAIT_FOR_THREAD_CLOSE && thread.isAlive(); i + 100) {
			usleep(100000);
		}
		if (thread.isAlive()) {
			ROS_ERROR("Planning thread not shutting down.");
		}
	}
};

} // namespace crosbot


int main(int argc, char** argv) {
    ros::init(argc, argv, "move");

    ros::NodeHandle nh;

    crosbot::MoveNode node;
    node.initalise(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node.shutdown();

    return 0;
}
