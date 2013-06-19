/*
 * crosshair.h
 *
 *  Created on: 21/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_SIMPLE_HPP_
#define CROSBOT_RENDER_SIMPLE_HPP_

#include <crosbot_ui/panels/robot.hpp>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

namespace crosbot {

namespace gui {

#define RENDER_CROSSHAIR	"crosshair"
#define RENDER_SPEED		"speed"
#define RENDER_ATTITUDE		"attitude"
#define RENDER_POWER		"power"
#define RENDER_MESSAGE		"message"

class CrosshairRender : public RobotRender {
public:
	CrosshairRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start() {}
	virtual void stop() {}
	virtual void render();

protected:
	Colour4f colour;
};

class SpeedRender : public RobotRender {
public:
	SpeedRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start() {}
	virtual void stop() {}
	virtual void render();

	virtual bool wheelEvent(QWheelEvent *);
protected:
	float changeStep;
	Colour4f lineColour, fillColour;
};

class AttitudeRender : public RobotRender {
public:
	AttitudeRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();

	void callback(const geometry_msgs::Quaternion);
protected:
	Quaternion attitude;
	std::string topic;
	ros::Subscriber subscriber;

	Colour4f lineColour, fillColour;
};

class PowerRender : public RobotRender {
public:
	enum Type { Voltage, Amps, Watts, Percent };

	PowerRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();

	void callback(const std_msgs::Float32);
protected:
	Type type;
	float value;
	std::string topic;
	ros::Subscriber subscriber;

	Colour4f colour;
};

class MessageRender : public RobotRender {
public:
	MessageRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();

	void callback(const std_msgs::String);
protected:
	std::string message;
	Time messageArrived;
	float pixelsPerSecond;
	std::string topic;
	ros::Subscriber subscriber;

	Colour4f colour;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_RENDER_SIMPLE_HPP_ */
