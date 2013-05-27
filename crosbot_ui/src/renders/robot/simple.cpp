/*
 * simple.cpp
 *
 *  Created on: 21/03/2012
 *      Author: rescue
 */

#include <ros/ros.h>
#include <crosbot_ui/renders/robot/simple.hpp>
#include <crosbot/utils.hpp>

#include <QWheelEvent>

namespace crosbot {

namespace gui {

CrosshairRender::CrosshairRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0.25,0.25,0.5,0.5)),
		colour(1, 1, 0, 0.5)
{}

void CrosshairRender::render() {
	preRender();

	glColor4f(colour.r, colour.g, colour.b, colour.a);

 	glBegin(GL_LINE_STRIP);
		glVertex2f(0.0, 0.5);
		glVertex2f(1.0, 0.5);
	glEnd();
 	glBegin(GL_LINE_STRIP);
		glVertex2f(0.5, 0.0);
		glVertex2f(0.5, 1.0);
	glEnd();

	postRender();
}

//QRectF CrosshairRender::getDefaultBounds() {
//	LOG("CrosshairRender::getDefaultBounds()\n");
//	return QRectF(0.25,0.25,0.5,0.5);
//}

// These should add up to no more than 0.65
#define SPEED_HEIGHT		0.55
#define TURNRATE_HEIGHT		0.05
SpeedRender::SpeedRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0.25,0.9,0.5,0.1)),
		changeStep(0.1),
		lineColour(0,1,0,0.5), fillColour(1,0,0,0.5)
{}

//QRectF SpeedRender::getDefaultBounds() {
//	LOG("SpeedRender::getDefaultBounds()\n");
//	return QRectF(0.25,0.9,0.5,0.1);
//}

void SpeedRender::render() {
	preRender();

	float currentSpeed, currentTurnRate;
	panel.getThrottle(currentSpeed, currentTurnRate);

	// Fill in speed
	glColor4f(fillColour.r, fillColour.g, fillColour.b, fillColour.a);
	glBegin(GL_QUADS);
		glVertex2f(0.02,0.1);
		glVertex2f(0.02+currentSpeed*0.96, 0.1);
		glVertex2f(0.02+currentSpeed*0.96, 0.1 + SPEED_HEIGHT);
		glVertex2f(0.02,0.1 + SPEED_HEIGHT);
	glEnd();

	// Fill in turn rate
	float trd = currentTurnRate * 0.48;
	glBegin(GL_QUADS);
		glVertex2f(0.5 - trd,0.9 - TURNRATE_HEIGHT);
		glVertex2f(0.5 + trd, 0.9 - TURNRATE_HEIGHT);
		glVertex2f(0.5 + trd, 0.9);
		glVertex2f(0.5 - trd, 0.9);
	glEnd();

	glColor4f(lineColour.r, lineColour.g, lineColour.b, lineColour.a);

	// Draw speed box
	glBegin(GL_LINE_LOOP);
		glVertex2f(0.01,0.05);
		glVertex2f(0.99,0.05);
		glVertex2f(0.99,0.15 + SPEED_HEIGHT);
		glVertex2f(0.01, 0.15 + SPEED_HEIGHT);
	glEnd();

	postRender();
}

#define MOUSE_DELTA		120
bool SpeedRender::wheelEvent(QWheelEvent *we) {
	float currentSpeed, currentTurnRate;
	panel.getCurrentSpeeds(currentSpeed, currentTurnRate);
	if(we->modifiers() != 0) {
			float newTurnRate = currentTurnRate;

			newTurnRate += we->delta() / MOUSE_DELTA * changeStep;

			if (newTurnRate < 0)
					newTurnRate = 0;
			else if (newTurnRate > 1.0) {
					newTurnRate = 1.0;
			}
			currentTurnRate = newTurnRate;
	} else {
		float newSpeed = currentSpeed;

		newSpeed += we->delta() / MOUSE_DELTA * changeStep;

		if (newSpeed < 0)
			newSpeed = 0;
		else if (newSpeed > 1.0) {
			newSpeed = 1.0;
		}
		currentSpeed = newSpeed;
	}
	panel.setCurrentSpeeds(currentSpeed, currentTurnRate);
	return true;
}

#define ANGULAR_ERROR		0.001
#define PITCH_ANGLE_MAX		(M_PI / 3.0)
#define NUM_MARKS			13
#define MARK_BIG			0.2
#define MARK_SMALL			0.1

AttitudeRender::AttitudeRender(RobotPanel& panel, ConfigElementPtr config) :
	RobotRender(panel, config, QRectF(0, 0.8, 0.2, 0.2)),
	lineColour(0, 1, 0, 0.5), fillColour(1, 0, 0, 0.5)
{
	topic = config->getParam(PARAM_TOPIC);
}

void AttitudeRender::start() {
	if (topic != "") {
		ros::NodeHandle nh("~");
		subscriber = nh.subscribe(topic, 1, &AttitudeRender::callback, this);
	}
}

void AttitudeRender::stop() {
	if (topic != "") {
		subscriber.shutdown();
	}
}

void AttitudeRender::callback(const geometry_msgs::Quaternion att) {
	attitude = att;
}

void AttitudeRender::render() {
    preRender();

	double rollAngle = 0, pitchAngle = 0, yawAngle = 0;

	attitude.getYPR(yawAngle, pitchAngle, rollAngle);

//    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glEnable(GL_BLEND);
//    glShadeModel(GL_FLAT);

    glPushMatrix();

		// To simplify maths, use a square that begins at (-1,-1) to (1,1)
		glTranslatef(0.5,0.5,0);
		glScalef(0.4,0.4,0.5);

		// This is a little bit hacked for the Scorpion/Emu setup
		//Draw the main polygon. Assume range is +/- 60 degrees.

		float cosRoll = cos(rollAngle);
		if (cosRoll < ANGULAR_ERROR && cosRoll > -ANGULAR_ERROR) {
			if (rollAngle > 0) {
				glBegin(GL_POLYGON);
				glColor4f(fillColour.r, fillColour.g, fillColour.b, fillColour.a);
				glVertex2f(0, 1);
				glVertex2f(0, -1);
				glVertex2f(1, -1);
				glVertex2f(1, 1);
				glEnd();
			} else {
				glBegin(GL_POLYGON);
				glColor4f(fillColour.r, fillColour.g, fillColour.b, fillColour.a);
				glVertex2f(0, 1);
				glVertex2f(0, -1);
				glVertex2f(-1, -1);
				glVertex2f(-1, 1);
				glEnd();
			}
		} else {
			float centreOffset = -pitchAngle/PITCH_ANGLE_MAX;
			float leftIntersectY = -tan(rollAngle) + centreOffset;
			float rightIntersectY = tan(rollAngle) + centreOffset;
			float leftIntersectX = -1;
			float rightIntersectX = 1;

			if (leftIntersectY < -1) {
				leftIntersectX = 1.0 / (leftIntersectY - centreOffset) * (1 + centreOffset);
				leftIntersectY = -1;
			} else if (leftIntersectY > 1) {
				leftIntersectX = -1.0 / (leftIntersectY - centreOffset) * (1 - centreOffset);
				leftIntersectY = 1;
			}

			if (rightIntersectY > 1) {
				rightIntersectX = 1.0 / (rightIntersectY - centreOffset) * (1 - centreOffset);
				rightIntersectY = 1;
			} else if (rightIntersectY < -1) {
				rightIntersectX = -1.0 / (rightIntersectY - centreOffset) * (1 + centreOffset);
				rightIntersectY = -1;
			}

			if (cosRoll > 0) {
				glBegin(GL_POLYGON);
				glColor4f(fillColour.r, fillColour.g, fillColour.b, fillColour.a);
				glVertex2f(leftIntersectX, leftIntersectY);
				if (leftIntersectX > -1) {
					if (leftIntersectY > 0) {
						glVertex2f(-1, 1);
						glVertex2f(-1, -1);
					}
				} else {
					glVertex2f(-1, -1);
				}
				if (rightIntersectX < 1) {
					if (rightIntersectY > 0) {
						glVertex2f(1, -1);
						glVertex2f(1, 1);
					}
				} else {
					glVertex2f(1, -1);
				}
				glVertex2f(rightIntersectX, rightIntersectY);
				glEnd();
			} else {
				// Robot is upside down
				glBegin(GL_POLYGON);
				glColor4f(fillColour.r, fillColour.g, fillColour.b, fillColour.a);
				glVertex2f(leftIntersectX, leftIntersectY);
				if (leftIntersectX > -1) {
					if (leftIntersectY < 0) {
						glVertex2f(-1, -1);
						glVertex2f(-1, 1);
					}
				} else {
					glVertex2f(-1, 1);
				}
				if (rightIntersectX < 1) {
					if (rightIntersectY < 0) {
						glVertex2f(1, 1);
						glVertex2f(1, -1);
					}
				} else {
					glVertex2f(1, 1);
				}
				glVertex2f(rightIntersectX, rightIntersectY);
				glEnd();
			}
		}

		// Draw outside line.
		glColor4f(lineColour.r, lineColour.g, lineColour.b, lineColour.a);
		glBegin(GL_LINE_STRIP);
			glVertex2f(-1,-1);
			glVertex2f(1,-1);
			glVertex2f(1,1);
			glVertex2f(-1,1);
			glVertex2f(-1,-1);
		glEnd();
		// Draw angle markers, from +60 degrees to -60 degrees, with
		glColor4f(0,1,0,1);

		for(int i=-NUM_MARKS/2; i <= NUM_MARKS/2; i++){

			if(i==0){
				glBegin(GL_LINES);
					glVertex2f(-1,0);
					glVertex2f(1,0);
				glEnd();
			}
			else if(i%2 == 0){
				glBegin(GL_LINES);
					glVertex2f(-MARK_BIG, i*2/(float) (NUM_MARKS-1));
					glVertex2f(MARK_BIG, i*2/(float) (NUM_MARKS-1));
				glEnd();
			// qDebug("Drawing big line %f", i*2/(float) (numMarks-1));
			}
			else {
				glBegin(GL_LINES);
					glVertex2f(-MARK_SMALL, i*2/(float) (NUM_MARKS-1));
					glVertex2f(MARK_SMALL, i*2/(float) (NUM_MARKS-1));
				glEnd();
			// qDebug("Drawing small line %f", i*2/(float) (numMarks-1));
			}
		}
    glPopMatrix();
//    glDisable(GL_BLEND);

	postRender();
}

#define TEXTSIZE_MIN			8
#define TEXTSIZE_MAX			36
#define TEXT_REDUCTION_RATIO	0.8

#define TYPE_CHARGE		"charge"

#define TYPE_PERCENT	"percent"
#define TYPE_PERCENTAGE	"percentage"
#define TYPE_PERCENT2	"%"

#define TYPE_VOLT		"volt"
#define TYPE_VOLTS		"volts"
#define TYPE_VOLTAGE	"voltage"
#define TYPE_V			"v"

#define TYPE_WATT		"watt"
#define TYPE_WATTS		"watts"
#define TYPE_WATTAGE	"wattage"
#define TYPE_W			"w"

PowerRender::PowerRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0.8, 0.9, 0.2, 0.1)),
		type(Voltage), value(0), colour(1.0, 0, 0, 0.5)
{}

void PowerRender::start() {
	if (topic != "") {
		ros::NodeHandle nh("~");
		subscriber = nh.subscribe(topic, 1, &PowerRender::callback, this);
	}
}

void PowerRender::stop() {
	if (topic != "") {
		subscriber.shutdown();
	}
}

void PowerRender::callback(const std_msgs::Float32 msg) {
	value = msg.data;
}

#define MIN(X, Y)		(((X) < (Y)) ? (X):(Y))
#define MAX(X, Y)		(((X) > (Y)) ? (X):(Y))
void PowerRender::render() {
	preRender();

    char ascii[64];
    switch (type) {
    case Voltage:
		sprintf(ascii, "%.1fV", value);
		break;
    case Watts:
		sprintf(ascii, "%.1fW", value);
		break;
    case Amps:
		sprintf(ascii, "%.2fA", value);
		break;
    default:
		sprintf(ascii, "%.1f%%", value);
		break;
    }
    QString label("");
    label.append(ascii);

    RobotWidget* widget = dynamic_cast<RobotWidget*>(panel.getWidget());
    if (widget != NULL) {
		int textSize = MIN(widget->size().width(),widget->size().height()) * rect.height() * TEXT_REDUCTION_RATIO;
		if (textSize < TEXTSIZE_MIN) {
			textSize = TEXTSIZE_MIN;
		} else if (textSize > TEXTSIZE_MAX) {
			textSize = TEXTSIZE_MAX;
		}
		glColor4f(colour.r, colour.g, colour.b, colour.a);
		widget->renderText(0.0f,0.0f,0.0f,label,QFont("Helvetica", textSize));
    }
	postRender();
}

MessageRender::MessageRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0, 0, 1, 0.1)),
		message(""), pixelsPerSecond(20),
		colour(1.0, 0, 0, 0.5)
{}

void MessageRender::start() {
	if (topic != "") {
		ros::NodeHandle nh("~");
		subscriber = nh.subscribe(topic, 1, &MessageRender::callback, this);
	}
}

void MessageRender::stop() {
	if (topic != "") {
		subscriber.shutdown();
	}
}

void MessageRender::callback(const std_msgs::String msg) {
	message = msg.data;
}

void MessageRender::render() {
	std::string message = this->message;
	if (message == "")
		return;
	preRender();

    QString label("");
    label.append(message.c_str());

    RobotWidget* widget = dynamic_cast<RobotWidget*>(panel.getWidget());
    if (widget != NULL) {
		int textSize = MIN(widget->size().width(),widget->size().height()) * rect.height() * TEXT_REDUCTION_RATIO;
		if (textSize < TEXTSIZE_MIN) {
			textSize = TEXTSIZE_MIN;
		} else if (textSize > TEXTSIZE_MAX) {
			textSize = TEXTSIZE_MAX;
		}
		glColor4f(colour.r, colour.g, colour.b, colour.a);
		QFont font("Helvetica", textSize);
		QFontMetrics metrics(font);

		QSize messageSize = metrics.size(Qt::TextSingleLine, label),
				widgetSize = widget->size();

		float renderHeight = rect.height() * MIN(widgetSize.width(), widgetSize.height());
		float renderWidth = rect.width() * MIN(widgetSize.width(), widgetSize.height());

		Duration d = Time::now() - messageArrived;

		float seconds = (d.sec * 1000 + d.nsec / 1000000) / 1000.0f;
		float pixels = fmodf(seconds*20, messageSize.width() + renderWidth);

//		int pixels = ;
//		pixels = pixels % (messageSize.width() + (int)renderWidth);

		float x = (renderWidth - pixels) / renderWidth;
		float y = metrics.descent() / renderHeight;
		widget->renderText(x, y, 0.0f, label, font);
    }
	postRender();
}

} // namespace gui

} // namespace crosbot
