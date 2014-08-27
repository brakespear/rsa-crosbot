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


namespace crosbot {

#define DEFAULT_BASEFRAME			"base_link"

class MoveNode {
protected:
	ros::Subscriber goalSub, mapSub;
	ros::ServiceClient explorerSrv;

	class PlanningThread : public Thread {
	public:
		Pose goal;
		nav_msgs::OccupancyGridConstPtr latestMap;

		ReadWriteLock dataLock;
		tf::TransformListener tf_listener;

		crosbot::AStarPlanner planner;
		bool operating, paused;

		VoronoiGrid::Constraints voronoiConstraints;
		std::string baseFrame;
		PlanningThread() :
			goal(INFINITY, INFINITY, INFINITY), operating(true), paused(false)
		{}

		void setGoal(const Pose& goal) {
			Lock lock(dataLock, true);
			// TODO: pause explorer
			this->goal = goal;
		}

		void setMap(const nav_msgs::OccupancyGridConstPtr& map) {
			Lock lock(dataLock, true);
			latestMap = map;
		}

		void run() {
			VoronoiGridPtr voronoi;

			while (operating) {
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


				if (!goal.isFinite() || latestMap == NULL) {
					// TODO: pause explorer
				}

				if (voronoi == NULL) {
					// TODO: create voronoi grid
				}

				// TODO: plan to goal

				// TODO: send path to explorer
			}

			// TODO: pause explorer
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

//		thread.start();
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
