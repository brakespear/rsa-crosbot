/*
 * joystick.cpp
 *
 *  Created on: 23/03/2012
 *      Author: rescue
 */

#include <crosbot_ui/renders/robot/joystick.hpp>

#include <QtGui/qevent.h>

namespace crosbot {

namespace gui {

JoystickRender::JoystickRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0.25,0.25,0.5,0.5)),
		joystick(*this),
		stickSize(0.05), hatSize(0.020),
		colour(1, 1, 0, 0.5), hatColour(0.5, 1.0, 0.5),
		activeColour(1.0, 0, 0)
{
	joystick.configure(config);
}

void JoystickRender::start() {
	joystick.start();
}

void JoystickRender::stop() {
	joystick.stop();
}

void JoystickRender::RenderJoystick::axisChanged(Axis axis, int) {
	float speed, turn;
	if (axis == Joystick::Throttle || axis == Joystick::Throttle2) {
		render.panel.getThrottle(speed, turn);
		if (axis == Joystick::Throttle) {
			speed = getRelativePosition(Joystick::Throttle);
		} else {
			turn = getRelativePosition(Joystick::Throttle2);
		}
		render.panel.setThrottle(speed, turn);
		if (settings.actionButton == Undefined || isPressed(settings.actionButton)) {
			speed *= (0.5 - getRelativePosition(Joystick::X)) * 2;
			turn  *= (0.5 - getRelativePosition(Joystick::Y)) * 2;
			render.panel.setCurrentSpeeds(speed, turn);
		}
	} else if ((axis == Joystick::X || axis == Joystick::Y) &&
			(settings.actionButton == Undefined || isPressed(settings.actionButton))) {
		render.panel.getThrottle(speed, turn);
		speed *= (0.5 - getRelativePosition(Joystick::X)) * 2;
		turn  *= (0.5 - getRelativePosition(Joystick::Y)) * 2;
		render.panel.setCurrentSpeeds(speed, turn);
	}
}

void JoystickRender::RenderJoystick::buttonPressed(int b) {
	float speed, turn;
	if (b == settings.actionButton) {
		render.panel.getThrottle(speed, turn);
		speed *= (0.5 - getRelativePosition(Joystick::X)) * 2;
		turn  *= (0.5 - getRelativePosition(Joystick::Y)) * 2;
		render.panel.setCurrentSpeeds(speed, turn);
	} else if (buttonKeys[b] != Joystick::Undefined) {
		QKeyEvent keyEvent(QEvent::KeyPress, buttonKeys[b], 0);
		RobotWidget* widget = dynamic_cast < RobotWidget* > (render.panel.getWidget());
		if (widget != NULL) {
			widget->keyPressEvent(&keyEvent);
		}
	}

	// TODO: Add options to turn button presses into KeyEvents
}

void JoystickRender::RenderJoystick::buttonReleased(int b) {
	if (b == settings.actionButton) {
		render.panel.setCurrentSpeeds(0.0, 0.0);
	} else if (buttonKeys[b] != Joystick::Undefined) {
		QKeyEvent keyEvent(QEvent::KeyRelease, buttonKeys[b], 0);
		RobotWidget* widget = dynamic_cast < RobotWidget* > (render.panel.getWidget());
		if (widget != NULL) {
			widget->keyPressEvent(&keyEvent);
		}
	}
}

void JoystickRender::render() {
	preRender();

	// draw crosshair
	glColor4f(colour.r, colour.g, colour.b, colour.a);

 	glBegin(GL_LINE_STRIP);
		glVertex2f(0.0, 0.5);
		glVertex2f(1.0, 0.5);
	glEnd();
 	glBegin(GL_LINE_STRIP);
		glVertex2f(0.5, 0.0);
		glVertex2f(0.5, 1.0);
	glEnd();

	// draw stick position
	if (joystick.isPressed(joystick.getActionButton())) {
		glColor4f(activeColour.r, activeColour.g, activeColour.b, activeColour.a);
	}

	float x = joystick.getRelativePosition(Joystick::Y), y = 1 - joystick.getRelativePosition(Joystick::X);
	float s2 = stickSize/2;
 	glBegin(GL_QUADS);
		glVertex2f(x - s2, y - s2);
		glVertex2f(x + s2, y - s2);
		glVertex2f(x + s2, y + s2);
		glVertex2f(x - s2, y + s2);
	glEnd();

	// draw hat position
	float hatX = joystick.getRelativePosition(Joystick::Pan),
			hatY = joystick.getRelativePosition(Joystick::Tilt);
	if (hatX != Joystick::UndefinedF && hatY != Joystick::UndefinedF) {
		glColor4f(hatColour.r, hatColour.g, hatColour.b, hatColour.a);

		x -= stickSize * (0.5 - hatX);
		y += stickSize * (0.5 - hatY);
		s2 = hatSize/2;
		glBegin(GL_QUADS);
			glVertex2f(x - s2, y - s2);
			glVertex2f(x + s2, y - s2);
			glVertex2f(x + s2, y + s2);
			glVertex2f(x - s2, y + s2);
		glEnd();
	}

	postRender();
}

} // namespace gui

} // namespace crosbot
