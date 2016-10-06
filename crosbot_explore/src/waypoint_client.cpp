/*
 * waypoint_client.cpp
 * client for sending waypoints to crosbot_explore
 * calculates A* path between current/default location and waypoint and orders that path to be traversed
 * Author: Elliott Smith for COMP3431 Assignment 2
*/

#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/Path.h>
#include <crosbot_explore/GetPath.h>
#include <crosbot_explore/FollowPath.h>
#include <crosbot_explore/SetMode.h>
#include <crosbot_explore/explorer.hpp>
#include <crosbot/utils.hpp>
#include <crosbot/config.hpp>
#include <crosbot/geometry/poses.hpp>

using namespace crosbot;
using namespace crosbot_explore;
using namespace std;

//function definitions - might move to a header file later
nav_msgs::Path getAstarPath(double x, double y, double z);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "crosbot_explore_waypoint_client");
	if (argc != 4)
	{
	  ROS_INFO("usage: crosbot_explore_waypoint_client x y z");
	  return 1;
	}
	double x = atof(argv[1]);
	double y = atof(argv[2]);
	double z = atof(argv[3]);
	ros::NodeHandle n;
	ros::ServiceClient client = n.serviceClient<crosbot_explore::FollowPath>("/explore/follow_path");
	crosbot_explore::FollowPath srv;
	nav_msgs::Path pathToFollow = getAstarPath(x,y,z);
	srv.request.path = pathToFollow;
	if (client.call(srv))
	{
		ROS_INFO("Waypoint (x,y,z) = (%.2lf,%.2lf,%.2lf) sent to crosbot explore\n",x,y,z);
	} else {
    	ROS_ERROR("Failed to send waypoint (x,y,z) = (%.2lf,%.2lf,%.2lf) to crosbot explore\n",x,y,z);
    	return 1;
	}

	return 0;
}



nav_msgs::Path getAstarPath(double x, double y, double z) {
	ros::NodeHandle nh;
	ros::ServiceClient astarClient = nh.serviceClient<crosbot_explore::GetPath>("/get_path");
	crosbot_explore::GetPath getPathService;
	//default start position is (2,2,0)
	geometry_msgs::Pose startPoint = Pose3D(2.0,2.0,0.0).toROS();
	geometry_msgs::Pose endPoint = Pose3D(x,y,z).toROS();
	ROS_INFO("Start point is position (x,y,z) = (%.2lf,%.2lf,%.2lf) orientation (x,y,z,w) = (%.2lf,%.2lf,%.2lf,%.2lf)\n",startPoint.position.x,startPoint.position.y,startPoint.position.z,startPoint.orientation.x,startPoint.orientation.y,startPoint.orientation.z,startPoint.orientation.w);
	ROS_INFO("End point is position (x,y,z) = (%.2lf,%.2lf,%.2lf) orientation (x,y,z,w) = (%.2lf,%.2lf,%.2lf,%.2lf)\n",endPoint.position.x,endPoint.position.y,endPoint.position.z,endPoint.orientation.x,endPoint.orientation.y,endPoint.orientation.z,endPoint.orientation.w);
	
	getPathService.request.start = startPoint;
	getPathService.request.end = endPoint;
	if (astarClient.call(getPathService)) {
		return getPathService.response.path;
	} else {
		ROS_ERROR("Failed to call service get_path to set waypoint at %.2lf %.2lf %.2lf\n",x,y,z);
	}

}
