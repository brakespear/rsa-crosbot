/*
 * set_wallfollow_side.hpp
 * Calculate whether there is an object closer to the left or right side of the robot 
 * And set the wallfollowing side of crosbot_explore to the closer side
 * Written by Elliott Smith for COMP3431 Assignment 2
 * Date: 28/09/2016
*/

#ifndef SET_WALLFOLLOW_SIDE_H
#define SET_WALLFOLLOW_SIDE_H


#include <crosbot/handle.hpp>
#include <crosbot_explore/SetMode.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class SetSide {
private:
	ros::Subscriber scanSub;
    ros::ServiceClient setModeClient;
    bool done = false;
	vector<std::string> sides;
	string currentSide = "left";


public:
    SetSide();
    virtual ~SetSide() {};
    void callbackScan(const sensor_msgs::LaserScanConstPtr& scan);
	void updateSidesVector(string side);
	void setWallfollowMode(string sideToFollow);
};



#endif
