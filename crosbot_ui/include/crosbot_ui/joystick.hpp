/*
 * joystick.h
 *
 *  Created on: 23/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_JOYSTICK_HPP_
#define CROSBOT_JOYSTICK_HPP_

#include <crosbot/utils.hpp>
#include <crosbot/config.hpp>
#include <crosbot/thread.hpp>
#include <string>

namespace crosbot {

#define MAX_JOYSTICK_NAME		512
#define MAX_JOYSTICK_AXES		256
#define MAX_JOYSTICK_BUTTONS	256

class Joystick {
public:
	enum Axis {
		X=1, Y=2, Z=3,
		Throttle=4, Throttle2=5,
		Pan=6, Tilt=7, Zoom=8
	};

	struct Settings {
	public:
		std::string name;
		int axisX, axisY, axisZ, axisThrottle, axisThrottle2, axisPan, axisTilt, axisZoom,
			actionButton;

		int minAxes[MAX_JOYSTICK_AXES];
		int maxAxes[MAX_JOYSTICK_AXES];
		int zeroAxes[MAX_JOYSTICK_AXES];
		int invertAxes[MAX_JOYSTICK_AXES];
//		int buttonKey[MAX_JOYSTICK_BUTTONS];

		Settings();
		void setDefaults(std::string name);
		void setDefaults(const Settings& config);
	};

	class ProcessThread : public Thread {
	protected:
		Joystick& joystick;
	public:
		ProcessThread(Joystick& joystick) : joystick(joystick) {}
		void run() {
			joystick.run();
		}
	};

	static const int Undefined;
	static const float UndefinedF;

	Joystick();
	virtual ~Joystick();
	virtual void configure(ConfigElementPtr);

	void start();
	void stop();

	int getPosition(Axis);
	float getRelativePosition(Axis);

	inline int getActionButton() { return settings.actionButton; }
	inline bool isPressed(int button) {
		if (button >= 0 && button < MAX_JOYSTICK_BUTTONS) {
			return buttonState[button] != 0;
		}
		return false;
	}

protected:
	virtual void axisChanged(Axis, int);
	virtual void buttonPressed(int);
	virtual void buttonReleased(int);

	void run();

	std::string device;
	Settings settings, userSettings;

	int axesPosition[MAX_JOYSTICK_AXES];
	int buttonState[MAX_JOYSTICK_BUTTONS];

	ProcessThread thread;
	bool operating;
	friend class ProcessThread;
};

} // namespace crosbot

#endif /* CROSBOT_JOYSTICK_HPP_ */
