/*
 * image.h
 *
 *  Created on: 22/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_JOYSTICK_HPP_
#define CROSBOT_RENDER_JOYSTICK_HPP_

#include <crosbot_ui/panels/robot.hpp>
#include <crosbot_ui/joystick.hpp>

namespace crosbot {

namespace gui {

#define RENDER_JOYSTICK		"joystick"

class JoystickRender : public RobotRender {
public:
	class RenderJoystick : public Joystick {
	public:
		JoystickRender& render;

		RenderJoystick(JoystickRender& render) : render(render) {}
		void axisChanged(Axis, int);
		void buttonPressed(int);
		void buttonReleased(int);
	};

	JoystickRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();
protected:
	RenderJoystick joystick;
	float stickSize, hatSize;

	Colour4f colour, hatColour, activeColour;
	friend class RenderJoystick;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_RENDER_JOYSTICK_HPP_ */
