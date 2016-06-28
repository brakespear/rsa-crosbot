/*
 * rosconfig.hpp
 *
 *  Created on: 28/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_ROSCONFIG_HPP_
#define CROSBOT_ROSCONFIG_HPP_

#include <crosbot/config.hpp>

#ifdef ROS_VERSION

namespace crosbot {

class ROSConfigElement;
typedef Handle<ROSConfigElement> ROSConfigElementPtr;

/**
 * Wrapper around ros::NodeHandle for loading configuration using the ROS
 *    parameter server system, providing support for loading
 *    Crosbot geometric data types.
 */
class ROSConfigElement : public ConfigElement {
    ros::NodeHandle nh;
    ROSConfigElementPtr parent;

public:

    /**
     * Load configuration via ROS, given the unresolved ROS namespace
     * @param name Unresolved namespace, typically <tt>"~"</tt>
     * @param parent Parent node for nested ROS configuration namespaces
     */
    ROSConfigElement(std::string name = "~", ROSConfigElementPtr parent = NULL);

    /**
     * Load configuration via ROS, given an existing ros::NodeHandle
     * @param nh Existing ros::NodeHandle from which to load configuration
     * @param parent Parent node for nested ROS configuration namespaces
     */
    ROSConfigElement(ros::NodeHandle nh, ROSConfigElementPtr parent = NULL);

    virtual ~ROSConfigElement() {};

    bool hasParam(std::string paramName) {
        return nh.hasParam(paramName);
    }

    std::string getParam(std::string paramName, std::string defaultValue="") {
        std::string rval;
        if (nh.getParam(paramName, rval)) {
            return rval;
        }
        return defaultValue;
    }

    bool getParamAsBool(std::string paramName, bool defaultValue=false) {
        bool rval;
        nh.param<bool>(paramName, rval, defaultValue);
        return rval;
    }

    int getParamAsInt(std::string paramName, int defaultValue=0) {
        int rval;
        nh.param<int>(paramName, rval, defaultValue);
        return rval;
    }

    long getParamAsLong(std::string paramName, long defaultValue=0) {
        int rval;
        nh.param<int>(paramName, rval, defaultValue);
        return rval;
    }

    float getParamAsFloat(std::string paramName, float defaultValue=0) {
        double rval;
        nh.param<double>(paramName, rval, defaultValue);
        return rval;
    }

    double getParamAsDouble(std::string paramName, double defaultValue=0) {
        double rval;
        nh.param<double>(paramName, rval, defaultValue);
        return rval;
    }

    /**
     * @warning ROS currently does not numerical indexing or counting of 'child' namespaces
     */
    unsigned int getChildCount();

    /**
     * @warning ROS currently does not numerical indexing or counting of 'child' namespaces
     */
    ConfigElementPtr getChild(int idx);

    ConfigElementPtr getChild(std::string name);

    ConfigElementPtr getParent() {
        return parent;
    }
};

} // namespace crosbot

#endif // ROS_VERSION


#endif /* CROSBOT_ROSCONFIG_HPP_ */
