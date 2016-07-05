/*
 * config.hpp
 *
 *  Created on: 06/12/2011
 *      Author: mmcgill
 */

#ifndef CROSBOT_CONFIG_HPP_
#define CROSBOT_CONFIG_HPP_

#include "handle.hpp"
#include "geometry.hpp"
#include <string>

namespace crosbot {

class ConfigElement;
typedef Handle<ConfigElement> ConfigElementPtr;

class ConfigElement : public HandledObject {
public:
	std::string name;

	virtual bool hasParam(std::string paramName)=0;
	virtual std::string getParam(std::string paramName, std::string defaultValue="")=0;

	virtual bool getParamAsBool(std::string paramName, bool defaultValue=false) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);


			if (strcasecmp("true", strVal.c_str()) == 0 ||
					strcasecmp("yes", strVal.c_str()) == 0 ||
					strcasecmp("on", strVal.c_str()) == 0 ||
					strcasecmp("t", strVal.c_str()) == 0 ||
					strcasecmp("y", strVal.c_str()) == 0) {
				return true;
			}
			return false;
		}
		return defaultValue;
	}

	virtual int getParamAsInt(std::string paramName, int defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atoi(strVal.c_str());
		}
		return defaultValue;
	}

	virtual long getParamAsLong(std::string paramName, long defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atol(strVal.c_str());
		}
		return defaultValue;
	}
	virtual float getParamAsFloat(std::string paramName, float defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atof(strVal.c_str());
		}
		return defaultValue;
	}

	virtual double getParamAsDouble(std::string paramName, double defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atof(strVal.c_str());
		}
		return defaultValue;
	}

	Point getParamAsPoint(std::string paramName, Point defaultValue=Point()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Point::parse(valueStr);
	}

	Quaternion getParamAsQuaternion(std::string paramName, Quaternion defaultValue=Quaternion()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Quaternion::parse(valueStr);
	}

	Pose getParamAsPose(std::string paramName, Pose defaultValue=Pose()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Pose::parse(valueStr);
	}

	virtual unsigned int getChildCount()=0;
	virtual ConfigElementPtr getChild(int idx)=0;
	virtual ConfigElementPtr getChild(std::string name)=0;
	virtual ConfigElementPtr getParent()=0;

	static ConfigElementPtr parseFile(std::string filename);
};

#ifdef ROS_VERSION
class ROSConfigElement : public ConfigElement {
	ros::NodeHandle nh;
	ConfigElementPtr parent;
public:
	ROSConfigElement(ros::NodeHandle nh, ConfigElementPtr parent=NULL) :
		nh(nh), parent(parent)
	{
		name = nh.getUnresolvedNamespace();
	}

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

	unsigned int getChildCount() {
		// TODO: ROSConfigElement::getChildCount()
		return 0;
	}

	ConfigElementPtr getChild(int idx) {
		// TODO: ROSConfigElement::getChild(int idx)
		return NULL;
	}

	ConfigElementPtr getChild(std::string chName) {
		try {
			ros::NodeHandle ch(nh, chName);

			return new ROSConfigElement(ch, this);
		} catch (ros::InvalidNameException& ine) {
		} catch (...) {
		}
		return NULL;
	}

	ConfigElementPtr getParent() {
		return parent;
	}
};
typedef Handle<ROSConfigElement> ROSConfigElementPtr;

#endif // ROS_VERSION


/**
 * Common elements and parameters
 */

#define ELEMENT_GUI				"gui"
#define ELEMENT_MAP				"map"

#define PARAM_NAME				"name"
#define PARAM_FILE				"file"
#define PARAM_POSE				"pose"
#define PARAM_TOPIC				"topic"
#define PARAM_TYPE				"type"

#define PARAM_WIDTH				"width"
#define PARAM_HEIGHT			"height"

#define PARAM_GRID				"grid"
#define PARAM_MAP				"map"

} // namespace crosbot

#endif /* CROSBOT_CONFIG_HPP_ */
