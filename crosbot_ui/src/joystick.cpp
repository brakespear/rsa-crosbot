/*
 * joystick.cpp
 *
 *  Created on: 23/03/2012
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot_ui/joystick.hpp>
#include <crosbot/data.hpp>

#include <linux/joystick.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <stdio.h>

namespace crosbot {

#ifndef __INT32_MAX__
#define __INT32_MAX__		2147483647
#endif

const int Joystick::Undefined = __INT32_MAX__;
const float Joystick::UndefinedF = INFINITY;

Joystick::Settings::Settings() {
	axisX = axisY = axisZ = axisThrottle = axisThrottle2 =
			axisPan = axisTilt = axisZoom = actionButton = Undefined;

	for (int i = 0; i < MAX_JOYSTICK_AXES; i++) {
		minAxes[i] = maxAxes[i] = zeroAxes[i] = Undefined;
		invertAxes[i] = Undefined;
	}
}

void Joystick::Settings::setDefaults(const Joystick::Settings& defaults) {
	if (axisX == Undefined)
		axisX = defaults.axisX;
	if (axisY == Undefined)
		axisY = defaults.axisY;
	if (axisZ == Undefined)
		axisZ = defaults.axisZ;
	if (axisThrottle == Undefined)
		axisThrottle = defaults.axisThrottle;
	if (axisThrottle2 == Undefined)
		axisThrottle2 = defaults.axisThrottle2;
	if (axisPan == Undefined)
		axisPan = defaults.axisPan;
	if (axisTilt == Undefined)
		axisTilt = defaults.axisTilt;
	if (axisZoom == Undefined)
		axisZoom = defaults.axisZoom;

	if (actionButton == Undefined)
		actionButton = defaults.actionButton;

	for (uint32_t i = 0; i < MAX_JOYSTICK_AXES; ++i) {

		if(minAxes[i] == Undefined)
			minAxes[i] = defaults.minAxes[i];
		if(maxAxes[i] == Undefined)
			maxAxes[i] = defaults.maxAxes[i];
		if(zeroAxes[i] == Undefined)
			zeroAxes[i] = defaults.zeroAxes[i];
		if(invertAxes[i] == Undefined)
			invertAxes[i] = defaults.invertAxes[i];
	}
}

void Joystick::Settings::setDefaults(std::string name) {
	axisX = 1;
	axisY = 0;
	axisThrottle = 3;
	axisThrottle2 = 2;
	axisPan = 4;
	axisTilt = 5;
	axisZoom = 6;
	actionButton = 0;

	for (int i = 0; i < MAX_JOYSTICK_AXES; i++) {
		minAxes[i] = -__SHRT_MAX__;
		maxAxes[i] = __SHRT_MAX__;
		zeroAxes[i] = 0;
		invertAxes[i] = 0;
	}

	if ((name == "Logitech Logitech Dual Action") || (name == "Logitech Logitech Cordless RumblePad 2")) {
		// Wire dual controller or wireless in D-Mode
		actionButton = 6;
		invertAxes[axisThrottle] = 1;
	} else if (name == "Generic X-Box pad") {
		// Wireless dual controller in X-Mode
//		actionButton = 4;
//		axisTurn = 3;
//		invertAxes[axisTurn] = 1;
//		axisThrottle = 4;
		actionButton = Undefined;
		axisThrottle = 2;
		invertAxes[axisThrottle] = 0;
		axisThrottle2 = 5;
		invertAxes[axisThrottle2] = 0;
		axisPan = 6;
		axisTilt = 7;
// A=0, B=1, X=2, Y=3, LB=4, RB=5, Back=6, Start=7, Logitech=8, LT=Axis 2, RT=Axis 5, LStick=9, RStick=10
	} else if (name == "Logitech WingMan Attack 2") {
		axisThrottle = 2;
		axisThrottle2 = 3;
	} else if (name == "Logitech Inc. WingMan Force 3D") {
		axisThrottle = 3;
		axisThrottle2 = Undefined;

		minAxes[axisX] = -14189; maxAxes[axisX] = 14188;
		minAxes[axisY] = -14189; maxAxes[axisY] = 19255;
		minAxes[axisThrottle] = -26012; maxAxes[axisThrottle] = 14526;
		invertAxes[axisThrottle] = true;
	} else if (name == "GREAT PLANES I-Controller") {
		axisThrottle2 = Undefined;
		axisTilt = 2;

		invertAxes[axisThrottle] = 1;
	}
}

Joystick::Joystick() : device("/dev/input/js0"), thread(*this), operating(true) {
}

Joystick::~Joystick() {}

void Joystick::configure(ConfigElementPtr config) {
	device = config->getParam("device", device);
	device = config->getParam("joystick", device);
}

void Joystick::start() {
	thread.start();
}

void Joystick::stop() {
	operating = false;
}

int Joystick::getPosition(Axis axis) {
	int axisIdx = Undefined;
	switch (axis) {
	case X:
		axisIdx = settings.axisX; break;
	case Y:
		axisIdx = settings.axisY; break;
	case Z:
		axisIdx = settings.axisZ; break;
	case Throttle:
		axisIdx = settings.axisThrottle; break;
	case Throttle2:
		axisIdx = settings.axisThrottle2; break;
	case Pan:
		axisIdx = settings.axisPan; break;
	case Tilt:
		axisIdx = settings.axisTilt; break;
	case Zoom:
		axisIdx = settings.axisZoom; break;
	}
	if (axisIdx == Undefined)
		return Undefined;
	return axesPosition[axisIdx];
}

float Joystick::getRelativePosition(Axis axis) {
	int axisIdx = Undefined;
	switch (axis) {
	case X:
		axisIdx = settings.axisX; break;
	case Y:
		axisIdx = settings.axisY; break;
	case Z:
		axisIdx = settings.axisZ; break;
	case Throttle:
		axisIdx = settings.axisThrottle; break;
	case Throttle2:
		axisIdx = settings.axisThrottle2; break;
	case Pan:
		axisIdx = settings.axisPan; break;
	case Tilt:
		axisIdx = settings.axisTilt; break;
	case Zoom:
		axisIdx = settings.axisZoom; break;
	}
	if (axisIdx == Undefined)
		return UndefinedF;

	int v = axesPosition[axisIdx];
	float rval;
	if (v > settings.zeroAxes[axisIdx]) {
		rval = 0.5 + (v - settings.zeroAxes[axisIdx]) /
				(float)((settings.maxAxes[axisIdx] - settings.zeroAxes[axisIdx])*2);
	} else {
		rval = 0.5 - (settings.zeroAxes[axisIdx] - v) /
				(float)((settings.zeroAxes[axisIdx] - settings.minAxes[axisIdx])*2);
	}

	if (settings.invertAxes[axisIdx])
		rval = 1 - rval;

	if (rval < 0)
		rval = 0;
	else if (rval > 1.0)
		rval = 1.0;

	return rval;
}

void Joystick::axisChanged(Axis, int) {}
void Joystick::buttonPressed(int i) {}
void Joystick::buttonReleased(int) {}

void Joystick::run() {
	int device = -1;
	Time lastOpenAttempt, lastRepeat = Time::now();

	char deviceName[MAX_JOYSTICK_NAME];
	int deviceAxesCount = 0, deviceButtonCount = 0;
	unsigned int dVersion;

	while (operating) {
		if (device == -1) {
			if ((Time::now() - lastOpenAttempt).toNSec() > 10000000000LL) {
				LOG("Joytick: Openning port %s.\n", this->device.c_str());
				device = open(this->device.c_str() , O_RDONLY);
				lastOpenAttempt = Time::now();
				if (device == -1)
					continue;

				ioctl(device, JSIOCGAXES, &deviceAxesCount);
				ioctl(device, JSIOCGBUTTONS, &deviceButtonCount);
				ioctl(device, JSIOCGVERSION, &dVersion);
				ioctl(device, JSIOCGNAME(MAX_JOYSTICK_NAME), deviceName);

				char mapping[ABS_CNT];
				ioctl(device, JSIOCGAXMAP, mapping);

				fcntl(device, F_SETFL, O_NONBLOCK);

				LOG("Joystick: Joystick %s found with %d axes and %d buttons.\n", deviceName, deviceAxesCount, deviceButtonCount);
				// load config for joystick
				settings = userSettings;

				Settings defaults;
				defaults.setDefaults(std::string(deviceName));
				settings.setDefaults(defaults);

				if (deviceAxesCount > MAX_JOYSTICK_AXES || deviceButtonCount > MAX_JOYSTICK_BUTTONS) {
					ERROR("Joystick: Joystick %s has too many axes(%d)/buttons(%d) to work fully with this render.\n", deviceName, deviceAxesCount, deviceButtonCount);
				}

				// TODO: setup force feedback
			} else {
				usleep(10000);
			}
			continue;
		}

		struct js_event js;

		/* read the joystick state */
		size_t nRead = read(device, &js, sizeof(struct js_event));
		if (nRead < sizeof(struct js_event)) {
			ERROR("Problem reading joystick state on %s.\n", this->device.c_str());
			close(device);
			device = -1;
			continue;
		}

		int n = js.number;
		int v = js.value;

		/* see what to do with the event */
		switch (js.type & ~JS_EVENT_INIT) {
			case JS_EVENT_AXIS:
				if (n >= 0 && n < MAX_JOYSTICK_AXES) {
//					LOG("Joystick: Axis %d value %d.\n", n, v);
					axesPosition[n] = v;

//					if (settings.minPosAxes[n] > v)
//						settings.minPosAxes[n] = v;
//					if (settings.maxPosAxes[n] < v)
//						settings.maxPosAxes[n] = v;
//					settings.sumAxes[n] += v;
//					settings.countAxes[n]++;

					bool usedAxis = true;
					Axis axis;
					if (n == settings.axisX) {
						axis = X;
					} else if (n == settings.axisY) {
						axis = Y;
					} else if (n == settings.axisZ) {
						axis = Z;
					} else if (n == settings.axisThrottle) {
						axis = Throttle;
					} else if (n == settings.axisThrottle2) {
						axis = Throttle2;
					} else if (n == settings.axisPan) {
						axis = Pan;
					} else if (n == settings.axisTilt) {
						axis = Tilt;
					} else if (n == settings.axisZoom) {
						axis = Zoom;
					} else {
						usedAxis = false;
					}
					if (usedAxis) {
						axisChanged(axis, v);
					}
				}
			break;
			case JS_EVENT_BUTTON:
				if (n >= 0 && n < MAX_JOYSTICK_BUTTONS && v != buttonState[n]) {
					buttonState[n] = v;
//					LOG("Joystick: Button %d value %d.\n", n, v);

					if (n == settings.actionButton) {
						// TODO: Force feedback
					}
					if (buttonState[n]) {
						buttonPressed(n);
					} else {
						buttonReleased(n);
					}
				}
			break;
		}

		if (Time::now() > lastRepeat + Duration(0, 100000000LL)) {
			bool repeat = false;
			for (int n = 0; n < MAX_JOYSTICK_BUTTONS; ++n) {
				if (buttonState[n]) {
					repeat = true;
					buttonPressed(n);
				}
			}
			if (repeat)
				lastRepeat = Time::now();
		}

		usleep(10000);
	}

	if (device != -1) {
		close(device);
		device = -1;
	}
}

} // namespace crosbot
