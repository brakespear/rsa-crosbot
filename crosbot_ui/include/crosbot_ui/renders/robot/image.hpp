/*
 * image.h
 *
 *  Created on: 22/03/2012
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_IMAGE_HPP_
#define CROSBOT_RENDER_IMAGE_HPP_

#include <crosbot_ui/panels/robot.hpp>

#include <sensor_msgs/Image.h>
#include <ros/ros.h>

namespace crosbot {

namespace gui {

#define RENDER_IMAGE	"image"

class ImageRender : public RobotRender {
public:
	ImageRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();

	virtual bool isHidden();
	virtual bool keyPressEvent(QKeyEvent *ke);

	void callback(sensor_msgs::ImageConstPtr);
protected:
	ImagePtr image, textureImage;
	bool hidden;
	int key;

	Colour4f colour;
	GLuint textureId;
	std::string topic;
	ros::Subscriber subscriber;
};

} // namespace gui

} // namespace casros

#endif /* CROSBOT_RENDER_IMAGE_HPP_ */
