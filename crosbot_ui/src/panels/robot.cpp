/*
 * robotpanel.cpp
 *
 *  Created on: 19/08/2009
 *      Author: rescue
 */
#include <crosbot_ui/panels/robot.hpp>
#include <string.h>

#include <QEvent>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QInputDialog>

#include <math.h>
#include <ctype.h>

#include <crosbot_ui/renders/robot/simple.hpp>
#include <crosbot_ui/renders/robot/image.hpp>
#include <crosbot_ui/renders/robot/joystick.hpp>
#include <crosbot/utils.hpp>

#include <geometry_msgs/Twist.h>

namespace crosbot {

namespace gui {
using namespace std;

#define DRIVE_FORWARD	Qt::Key_W
#define DRIVE_BACKWARD	Qt::Key_S
#define DRIVE_LEFT		Qt::Key_A
#define DRIVE_RIGHT		Qt::Key_D

#define MOUSE_DELTA		120
#define PTZ_SCALE		1

// Special keys.
#define KEY_MOUSE		(0x88000001)

#define REQUESTERID		"RobotGUI"

std::vector<RobotRenderFactoryFunc> robotRenderFactories;
void RobotPanel::addRenderFactory(RobotRenderFactoryFunc factory) {
	robotRenderFactories.push_back(factory);
}

void RobotPanel::getThrottle(float& speed, float& turnRate) {
	speed = this->throttle;
	turnRate = this->throttleTurn;
}

void RobotPanel::setThrottle(const float speed, const float turnRate) {
	this->throttle = speed;
	this->throttleTurn = turnRate;
}

void RobotPanel::getCurrentSpeeds(float& speed, float& turnRate) {
	speed = this->speed;
	turnRate = this->turnRate;
}

void RobotPanel::setCurrentSpeeds(const float speed, const float turnRate) {
	this->speed = speed;
	this->turnRate = turnRate;
}

void RobotPanel::getPanTiltZoom(float& pan, float& tilt, float& zoom) {
	pan = this->pan; tilt = this->tilt; zoom = this->zoom;
}

void RobotPanel::getDesiredPanTiltZoom(float& pan, float& tilt, float& zoom) {
	pan = this->desiredPan; tilt = this->desiredTilt; zoom = this->desiredZoom;
}

void RobotPanel::setDesiredPanTiltZoom(float pan, float tilt, float zoom) {
	desiredPan = pan; desiredTilt = tilt; desiredZoom = zoom;

	// TODO: Set desired pan tilt zooms.
}

RobotRender *RobotPanel::getRender(ConfigElementPtr config, RobotPanel& panel) {
	RobotRender* rval = NULL;

	for (int i = ((int)robotRenderFactories.size()) - 1; rval == NULL && i >= 0; --i) {
		rval = robotRenderFactories[i](config, panel);
	}
	if (rval == NULL) {
		if (strcasecmp(config->name.c_str(), RENDER_CROSSHAIR) == 0) {
			rval = new CrosshairRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_SPEED) == 0) {
			rval = new SpeedRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_POWER) == 0) {
			rval = new PowerRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_ATTITUDE) == 0) {
			rval = new AttitudeRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_MESSAGE) == 0) {
			rval = new MessageRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_IMAGE) == 0) {
			rval = new ImageRender(panel, config);
		} else if (strcasecmp(config->name.c_str(), RENDER_JOYSTICK) == 0) {
			rval = new JoystickRender(panel, config);
#ifdef JUNK_RENDER
		} else if (strcasecmp(config->name.c_str(), "junk") == 0) {
			rval = new JointRender(panel, config);
#endif
		}
	}

	return rval;
}

RobotPanel::RobotPanel(ConfigElementPtr config) :
		Panel(config), widget(*this),
		motorThread(*this), motorsOperating(true),
		speed(0), turnRate(0), throttle(0.5), throttleTurn(0.5),
		maxSpeed(0.5), maxTurn(0.5), drivePub(NULL)
{
	pan = tilt = zoom = desiredPan = desiredTilt = desiredZoom = 0;

	for (uint32_t i = 0; i < config->getChildCount(); ++i) {
		ConfigElementPtr child = config->getChild(i);

		if (strcasecmp(child->name.c_str(), ELEMENT_DRIVE) == 0) {
			driveTopic = child->getParam(PARAM_NAME);
			driveTopic = child->getParam(PARAM_TOPIC, driveTopic);

			maxSpeed = child->getParamAsFloat("speed", maxSpeed);
			maxTurn = child->getParamAsFloat("turn", maxTurn);
		} else if (strcasecmp(child->name.c_str(), ELEMENT_KEY) == 0 ||
				strcasecmp(child->name.c_str(), ELEMENT_JOINT) == 0) {
			widget.addInputListener(child);
		} else {
			RobotRender* render = getRender(child, *this);
			if (render != NULL) {
				widget.addRender(render);
			} else {
				ERROR("RobotPanel: Unknown child %s.\n", child->name.c_str());
			}
		}
	}
}

RobotPanel::~RobotPanel() {
	if (drivePub != NULL) {
		drivePub->shutdown();
		delete drivePub;
		drivePub = NULL;
	}
}

void RobotPanel::start() {
	if (driveTopic != "") {
		ros::NodeHandle nh("~");

		drivePub = new ros::Publisher();
		*drivePub = nh.advertise<geometry_msgs::Twist>(driveTopic, 1);
	}

	motorThread.start();
	widget.startRenders();
}

void RobotPanel::stop() {
	widget.stopRenders();
	motorsOperating = false;
}

void RobotPanel::motorThreadBody() {
	float speed = this->speed, turn = this->turnRate;
	bool stopSent = false;
	geometry_msgs::Twist twist;
	twist.linear.x = twist.linear.y = twist.linear.z = 0;
	twist.angular.x = twist.angular.y = twist.angular.z = 0;

	while (motorsOperating) {
		speed = this->speed;
		turn = this->turnRate;

		if ((fabs(speed) + fabs(turn)) < 0.05) {
			if (!stopSent) {
				// send stop signal
				LOG("RobotPanel: STOPPING motors.\n");
				if (drivePub != NULL) {
					twist.linear.x = twist.angular.z = 0;
					drivePub->publish(twist);
				}
				stopSent = true;
			}
		} else {
			// send motion signal
//			LOG("RobotPanel: Driving at (%.1f, %.1f)%% of MAX.\n", speed*100, turn*100);
			if (drivePub != NULL) {
				twist.linear.x = speed * maxSpeed;
				twist.angular.z = turn * maxTurn;
				drivePub->publish(twist);
			}
			stopSent = false;
		}

		usleep(20000);
	}

	// Send stop signal
	LOG("RobotPanel: STOPPING motors.\n");
	if (drivePub != NULL) {
		twist.linear.x = twist.angular.z = 0;
		drivePub->publish(twist);
	}
	stopSent = true;
}

} // namespace gui

} //namespace casros
