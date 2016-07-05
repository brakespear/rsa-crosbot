/*
 * config.hpp
 *
 *  Created on: 06/12/2011
 *      Author: Matt McGill

 */

#ifndef CROSBOT_CONFIG_HPP_
#define CROSBOT_CONFIG_HPP_

#include <crosbot/handle.hpp>
#include <crosbot/geometry.hpp>

#include <string>

namespace crosbot {

class ConfigElement;
typedef Handle<ConfigElement> ConfigElementPtr;

/**
 * Load launch configuration parameters from a file
 *    using the legacy CASrobot XML configuration format.
 * Provides function to load configuration options as different types
 *    including common data structures used in crosbot such as points and quaternions.
 */
class ConfigElement : public HandledObject {
protected:

    /**
     * Fully resolved namespace of the config element.
     * For ROS, this is equivalent to ros::NodeHandle::getNamespace().
     * For legacy CASrobot XML, this is equivalent to the XML tree-path.
     */
    std::string resolvedNamespace;

    /**
     * Unresolved name of the sole child element.
     * For both ROS and legacy CASrobot XML this is the last term of the config element's fully resolved namespace
     */
    std::string childName;

public:

    /**
     * Get the fully resolved namespace for this configuration element
     * @return Fully resolved namespace
     */
    virtual std::string getResolvedNamespace() {
        return resolvedNamespace;
    }

	/**
	 * Get the child name (or the "unresolved" name) of this configuration element
	 * @return Child name
	 */
	virtual std::string getChildName() {
	    return childName;
	}

	/**
	 * Check if the given parameter exists in the loaded configuration
	 * @param paramName Name of the parameter to check for
	 * @return True is parameter exists, false otherwise
	 */
	virtual bool hasParam(std::string paramName)=0;

    /**
     * Get the raw string value of the given parameter as found in the loaded configuration
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return String value of the given parameter
     */
	virtual std::string getParam(std::string paramName, std::string defaultValue="")=0;

    /**
     * Get the boolean of the given parameter as found in the loaded configuration.
     * All string variations of
     *  <tt>true</tt>,
     *  <tt>yes</tt>,
     *  <tt>on</tt>,
     *  <tt>t</tt>, and
     *  <tt>y</tt>
     *  generate a True result.
     * Otherwise false is returned.
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter,
     *  or if a valid string for True is not found
     * @return Boolean value of the given parameter
     */
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

	/**
     * Get the integer value of the given parameter as found in the loaded configuration
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return Integer value of the given parameter
     */
	virtual int getParamAsInt(std::string paramName, int defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atoi(strVal.c_str());
		}
		return defaultValue;
	}

    /**
     * Get the long integer value (<tt>long int</tt>) of the given parameter as found in the loaded configuration
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return <tt>long int</tt> value of the given parameter
     */
	virtual long getParamAsLong(std::string paramName, long defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atol(strVal.c_str());
		}
		return defaultValue;
	}

    /**
     * Get the floating value of the given parameter to <tt>float</tt> precision as found in the loaded configuration
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return <tt>float</tt> value of the given parameter
     */
	virtual float getParamAsFloat(std::string paramName, float defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atof(strVal.c_str());
		}
		return defaultValue;
	}

    /**
     * Get the floating value of the given parameter to <tt>double</tt> precision as found in the loaded configuration
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return <tt>double</tt> value of the given parameter
     */
	virtual double getParamAsDouble(std::string paramName, double defaultValue=0) {
		if (hasParam(paramName)) {
			std::string strVal = getParam(paramName);
			return atof(strVal.c_str());
		}
		return defaultValue;
	}

    /**
     * Get the value of the given parameter as found in the loaded configuration
     *  converted to a 3D Point.
     * The string conversion is performed by Point::parse(std::string)
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return 3D point representation of the value of the given parameter
     */
	Point getParamAsPoint(std::string paramName, Point defaultValue=Point()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Point::parse(valueStr);
	}

    /**
     * Get the value of the given parameter as found in the loaded configuration
     *  converted to a Quaternion.
     * The string conversion is performed by Quaternion::parse(std::string)
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return 3D point representation of the value of the given parameter
     */
	Quaternion getParamAsQuaternion(std::string paramName, Quaternion defaultValue=Quaternion()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Quaternion::parse(valueStr);
	}

    /**
     * Get the value of the given parameter as found in the loaded configuration
     *  converted to 6-DOF Pose.
     * The string conversion is performed by Pose::parse(std::string)
     * @param paramName Name of the parameter
     * @param defaultValue Default value to return if hasParam(std::string) returns false for the given parameter
     * @return 3D point representation of the value of the given parameter
     */
	Pose getParamAsPose(std::string paramName, Pose defaultValue=Pose()) {
		if (!hasParam(paramName))
			return defaultValue;
		std::string valueStr = getParam(paramName);
		return Pose::parse(valueStr);
	}

	/**
	 * For nested XML configurations, get the number of nested children XML as sub-configuration elements.
	 * @return Number of nested XML children configuration elements
	 */
	virtual unsigned int getChildCount()=0;

	/**
	 * Get the sub-configuration element by index
	 * @param idx Index of sub-configuration element
	 * @return Sub-configuration element for the given index
	 */
	virtual ConfigElementPtr getChild(int idx)=0;

    /**
     * Get the sub-configuration element by the given name or identifier
     * @param name Name or identifier of sub-configuration element
     * @return Sub-configuration element for the given name or identifier
     */
	virtual ConfigElementPtr getChild(std::string name)=0;

	/**
	 * Get the parent-configuration element, if this is a sub-configuration element,
	 *   or <tt>NULL</tt> if there is no parent
	 * @return Parent-configuration element
	 */
	virtual ConfigElementPtr getParent()=0;

	/**
	 * Parse a given configuration file, assumed to be in the legacy XML CASrobot format,
	 *  returning a pointer to the root configuration node.
	 * @param filename Path to the configuration file on disk
	 * @return Root configuration node of the legacy XML configuration file
	 */
	static ConfigElementPtr parseFile(std::string filename);
};

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

// FOR LEGACY SAKE TO AVOID CHANGING NUMEROUS HEADER FILES
#include <crosbot/config/rosconfig.hpp>


