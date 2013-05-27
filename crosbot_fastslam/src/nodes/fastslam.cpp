/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_fastslam/module.hpp>
#include <crosbot/utils.hpp>

using namespace crosbot;
using namespace crosbot::fastslam;

class FastSLAMNode : public FastSLAMModule {
public:
	FastSLAMNode() : FastSLAMModule() {}
};
typedef Handle<FastSLAMNode> FastSLAMNodePtr;

int main(int argc, char** argv) {
    ros::init(argc, argv, "fastslam");

    ros::NodeHandle nh;

    FastSLAMNodePtr node = new FastSLAMNode();
    node->initialize(nh);

    while (ros::ok()) {
        ros::spin();
    }
    node->shutdown();

    return 0;
}



