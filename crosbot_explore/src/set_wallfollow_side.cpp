/*
 * set_wallfollow_side.cpp
 * Calculate whether there is an object closer to the left or right side of the robot 
 * And set the wallfollowing side of crosbot_explore to the closer side
 * Written by Elliott Smith for COMP3431 Assignment 2
 * Date: 28/09/2016
*/


#include <ros/ros.h>
#include <crosbot_explore/set_wallfollow_side.hpp>

#define START_RIGHT_ANGLE -1.658063 //rads
#define STOP_RIGHT_ANGLE -1.4835299 //rads

#define STOP_LEFT_ANGLE 1.658063 //rads
#define START_LEFT_ANGLE 1.4835299 //rads

#define LEFT_WALLFOLLOW 2
#define RIGHT_WALLFOLLOW 3

using namespace std;

SetSide::SetSide(){
	ros::NodeHandle n;
	scanSub = n.subscribe<sensor_msgs::LaserScan>("scan", 1, &SetSide::callbackScan, this);
	setModeClient = n.serviceClient<crosbot_explore::SetMode>("/explore/set_mode");
}

void SetSide::callbackScan(const sensor_msgs::LaserScanConstPtr& scan){
	//Debugging
	if (!done) {
		cout << "Min angle is: " << scan->angle_min << " max angle is: " << scan->angle_max << " increment angle is: " << scan->angle_increment << endl;
		cout << "Min range is: " << scan->range_min << " max range is: " << scan->range_max << endl;
		float angle = scan->angle_min;
		for (int n = 0; n<scan->ranges.size(); n++, angle += scan->angle_increment) {
			cout << "Beam number " << n << " at angle " << angle << " has range " << scan->ranges[n] << endl;
		}
		done = true;
	}
	
	//Find average distance to the right of the robot between -95 and -85 degrees
	int n = (START_RIGHT_ANGLE-scan->angle_min)/scan->angle_increment;
	float angle = START_RIGHT_ANGLE;
	float totalRanges = 0.0;
	float numRanges = 0.0;
	while (angle<STOP_RIGHT_ANGLE) {
		float distance = scan->ranges[n];
		if (distance>scan->range_min && distance<scan->range_max) {
			totalRanges+=scan->ranges[n];
			numRanges+=1.0;
		}
		n++;
		angle+=scan->angle_increment;
	}
	float avgDistanceRight = totalRanges/numRanges;
	//ROS_INFO("Average distance to the right of the robot is %.5f",avgDistanceLeft);
	
	//Find average distance to the left of the robot between 85 and 95 degrees
	n = (START_LEFT_ANGLE-scan->angle_min)/scan->angle_increment;
	angle = START_LEFT_ANGLE;
	totalRanges = 0.0;
	numRanges = 0.0;
	while (angle<STOP_LEFT_ANGLE){
		float distance = scan->ranges[n];
		if (distance>scan->range_min && distance<scan->range_max) {
			totalRanges+=scan->ranges[n];
			numRanges+=1.0;
		}
		n++;
		angle+=scan->angle_increment;
	}
	float avgDistanceLeft = totalRanges/numRanges;
	//ROS_INFO("Average distance to the left of the robot is %.5f",avgDistanceRight);
	string side;
	if (avgDistanceLeft>avgDistanceRight) {
		side = "right";
	} else {
		side = "left";
	}
	updateSidesVector(side);

	
}


void SetSide::updateSidesVector(string side) {
	if (sides.size()==0) {
		sides.push_back(side);
		setWallfollowMode(side);
	} else {
		string prevSide = sides.back();
		if (prevSide.compare(side) != 0) {
			sides.clear();
		}
		sides.push_back(side);
		string newSideToFollow = sides.back();
		if (sides.size()>=15 && currentSide.compare(newSideToFollow) != 0) {
			setWallfollowMode(newSideToFollow);
		}
	}
}
			

void SetSide::setWallfollowMode(string sideToFollow) {
	currentSide = sideToFollow;
	crosbot_explore::SetMode srv;
	if (strcmp(sideToFollow.c_str(),"left")==0) {
		srv.request.mode = LEFT_WALLFOLLOW;
	} else {
		srv.request.mode = RIGHT_WALLFOLLOW;
	}
	if (setModeClient.call(srv)) {
		ROS_INFO("Wallfollowing side changed to %s",sideToFollow.c_str());
	} else {
		ROS_ERROR("Failed to set wallfollow side mode");
	}
}
		

int main(int argc, char** argv) {
	ros::init(argc, argv, "crosbot_set_wallfollow_side");
	SetSide sideSetter;
	while (ros::ok()){
        ROS_INFO("set_wallfollow_side spinning");

        ros::spin();
    }
    return 0;
}
