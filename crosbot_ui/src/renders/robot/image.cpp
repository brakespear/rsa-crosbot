/*
 * image.cpp
 *
 *  Created on: 22/03/2012
 *      Author: rescue
 */

#include <ros/ros.h>
#include <crosbot_ui/renders/robot/image.hpp>

#include <crosbot/utils.hpp>

#include <QKeyEvent>

namespace crosbot {

namespace gui {

ImageRender::ImageRender(RobotPanel& panel, ConfigElementPtr config) :
	RobotRender(panel, config),
	hidden(false), key(0), colour(0,1,0,1)
{
	if (config->hasParam(PARAM_KEY)) {
		std::string key = config->getParam(PARAM_KEY);
		if (key.size() > 0) {
			hidden = true;
			this->key = panel.getKeyForChar(key[0]);
		}
	}

	topic = config->getParam(PARAM_TOPIC);
	topic = config->getParam(RENDER_IMAGE, topic);
	topic = config->getParam(PARAM_NAME, topic);
	transport = config->getParam("transport", "raw");
}

void ImageRender::start() {
	if (topic != "") {
		ros::NodeHandle nh("~");
		image_transport::ImageTransport it(nh);
		image_transport::TransportHints hint(transport);
		subscriber = it.subscribe(topic, 1, &ImageRender::callback, this, hint);
	}
}

void ImageRender::stop() {
	if (topic != "") {
		subscriber.shutdown();
	}
}

bool ImageRender::isHidden() {
	return hidden;
}

bool ImageRender::keyPressEvent(QKeyEvent *ke) {
	if (key == ke->key()) {
		hidden = !hidden;
		return true;
	}
	return false;
}

void ImageRender::callback(const sensor_msgs::ImageConstPtr& rosImage) {
	ImagePtr image = new Image(rosImage);
	if (image->encoding == Image::Unknown) {
		ERROR("crosbot::Image: Unknown encoding %s\n", rosImage->encoding.c_str());
	}

	this->image = image;
}

void ImageRender::render() {
	preRender();

	ImagePtr newImage = image, currentImage;
	if (newImage != NULL) {
		if (newImage != textureImage) {
			if (textureId == 0) {
				glGenTextures(1, &textureId);
			}

			currentImage = newImage->inEncoding(Image::RGBA8);

			if (usePOTTextures) {
				currentImage = getPOTImage(currentImage);
			}
			glPushMatrix();
			glBindTexture(GL_TEXTURE_2D, textureId);
			glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, currentImage->width, currentImage->height, 0, GL_RGBA, GL_UNSIGNED_BYTE, currentImage->data);
			glPopMatrix();
		}

		if (newImage->width < newImage->height) {
			glScalef(newImage->width / (float)newImage->height, 1.0, 1.0);
		} else if (newImage->width > newImage->height) {
			glScalef(1.0, newImage->height / (float)newImage->width, 1.0);
		}

		glBindTexture(GL_TEXTURE_2D, textureId);

		// Now the core rendering code.
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
					GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
					GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
		glEnable(GL_TEXTURE_2D);
//		glShadeModel(GL_FLAT);
		glColor4f(colour.r, colour.g, colour.b, colour.a);
		glBegin(GL_QUADS);
			if (!usePOTTextures || newImage == NULL) {
				glTexCoord2f(0.0,1.0);
				glVertex2f(0.0,0.0);
				glTexCoord2f(1.0,1.0);
				glVertex2f(1.0,0.0);
				glTexCoord2f(1.0,0.0);
				glVertex2f(1.0,1.0);
				glTexCoord2f(0.0,0.0);
				glVertex2f(0.0,1.0);
			} else {
				glTexCoord2f(0.0,newImage->height/(float)currentImage->height);
				glVertex2f(0.0,0.0);
				glTexCoord2f(newImage->width/(float)currentImage->width,
							newImage->height/(float)currentImage->height);
				glVertex2f(1.0,0.0);
				glTexCoord2f(newImage->width/(float)currentImage->width,0.0);
				glVertex2f(1.0,1.0);
				glTexCoord2f(0.0,0.0);
				glVertex2f(0.0,1.0);
			}
		glEnd();
		glDisable(GL_TEXTURE_2D);
	}

	postRender();
}

} // namespace gui

} // namespace crosbot
