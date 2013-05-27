/*
 * robotpanel.h
 *
 *  Created on: 13/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_ROBOT_HPP_
#define CROSBOT_PANEL_ROBOT_HPP_

#include <crosbot_ui/panels/panel.hpp>
#include <crosbot_ui/renders/robot/robotwidget.hpp>
#include <crosbot_ui/opengl.hpp>

#include <ros/ros.h>

namespace crosbot {

namespace gui {

#define PANEL_ROBOT				"robot"
#define ELEMENT_PARAM			"param"

#define ELEMENT_DRIVE			"drive"

#define PARAM_LEFT				"left"
#define PARAM_RIGHT				"right"
#define PARAM_STEP				"step"
#define PARAM_SPEED				"speed"
#define PARAM_TURNRATE			"turnrate"

#define PARAM_KEY				"key"
#define PARAM_INTERRUPT			"interrupt"
#define PARAM_ARGS				"args"
#define PARAM_VALUE				"value"
#define PARAM_QUERY				"query"

#define MOTOR_COMMAND_UPDATE_INTERVAL		100

class RobotRender; class RobotPanel;
typedef RobotRender* (*RobotRenderFactoryFunc)(ConfigElementPtr config, RobotPanel&);
/**
 * \image html panel-robot.png
 */
class RobotPanel : public Panel {
Q_OBJECT
public:
	RobotPanel(ConfigElementPtr config);
	~RobotPanel();

	QWidget *getWidget() {
		return &widget;
	}

	void start();
	void stop();

	static void addRenderFactory(RobotRenderFactoryFunc);
	static RobotRender *getRender(ConfigElementPtr config, RobotPanel&);

	void getThrottle(float& throttle, float& turnThrottle);
	void setThrottle(float throttle, float turnThrottle);
	void getCurrentSpeeds(float& speed, float& turnRate);
	void setCurrentSpeeds(const float speed, const float turnRate);

	void getPanTiltZoom(float& pan, float& tilt, float& zoom);
	void getDesiredPanTiltZoom(float& pan, float& tilt, float& zoom);
	void setDesiredPanTiltZoom(float pan, float tilt, float zoom);
protected:
	class MotorControlThread : public Thread {
	protected:
		RobotPanel& panel;
	public:
		MotorControlThread(RobotPanel& panel) :
			Thread("RobotPanel::MotorControlThread"),
			panel(panel) {}
		~MotorControlThread() {}
		void run() { panel.motorThreadBody(); }
	};

	RobotWidget widget;
	MotorControlThread motorThread;
	bool motorsOperating;
	void motorThreadBody();

	float speed, turnRate, throttle, throttleTurn, maxSpeed, maxTurn;
	std::string driveTopic;
	ros::Publisher* drivePub;

	float pan, tilt, zoom, desiredPan, desiredTilt, desiredZoom;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_ROBOT_HPP_ */
