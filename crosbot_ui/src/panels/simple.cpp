/*
 * simple.cpp
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */

#include <ros/ros.h>
#include <crosbot_ui/panels/simple.hpp>
#include <crosbot_ui/panels/panelfactory.hpp>

#include <crosbot/utils.hpp>

#include <QResizeEvent>
#include <QPainter>
namespace crosbot {

namespace gui {

ImageLabel::ImageLabel(QString str) : QFrame() {
	connect(this, SIGNAL(imageUpdated()), this, SLOT(update()));
	connect(this, SIGNAL(resized()), this, SLOT(update()));
}

void ImageLabel::paintEvent(QPaintEvent *event) {
	QFrame::paintEvent(event);

	QPainter painter(this);
	QImage scaled;
	QSize size = this->size();
	if (size.width() == 0 || size.height() == 0 ||
			qImage.width() == 0 || qImage.height() == 0)
		return;
	if (qImage.width() / (float)size.width() < qImage.height() / (float)size.height()) {
		scaled = qImage.scaledToHeight(size.height(), Qt::SmoothTransformation);
	} else {
		scaled = qImage.scaledToWidth(size.width(), Qt::SmoothTransformation);
	}

	QPoint p((size.width() - scaled.width())/2, (size.height() - scaled.height())/2);

	painter.drawImage(p, scaled);
}

void ImageLabel::resizeEvent(QResizeEvent * event) {
	Q_EMIT resized();
}

void ImageLabel::setImage(ImagePtr image) {
	this->image = image;
	qImage = QImage((unsigned char *)image->data, image->width, image->height,
						image->width*3, QImage::Format_RGB888);

	Q_EMIT imageUpdated();
}

void ImageLabel::setImage(QImage image) {
	qImage = image;

	Q_EMIT imageUpdated();
}

void ImageLabel::update() {
	repaint();
}

ImagePanel::ImagePanel(ConfigElementPtr config) :
		Panel(config), label(QString("Image")) {
//	label.setAlignment(Qt::AlignCenter | Qt::AlignVCenter);

	std::string src = config->getParam(PARAM_SRC);
	if (src == "")
		src = config->getParam(PARAM_FILE);
	if (src == "")
		src = config->getParam(PARAM_IMAGE);
	if (src != "") {
		label.setImage(QImage(QString(src.c_str())));
	}

	topic = config->getParam(PARAM_TOPIC);
	if (topic == "" && config->hasParam(PARAM_IMAGE)) {
		topic = config->getParam(PARAM_IMAGE);
	}
	if (config->hasParam(PARAM_MAP)) {
		grid = config->getParam(PARAM_MAP);
	} else {
	}
}

ImagePanel::~ImagePanel() {}

QWidget *ImagePanel::getWidget() {
	return &label;
}

void ImagePanel::start() {
	ros::NodeHandle nh("~");
	if (topic != "") {
//		LOG("Subscribing to image: %s\n", topic.c_str());
		subscribe = nh.subscribe(topic, 1, &crosbot::gui::ImagePanel::callbackImage, this);
	} else if (grid != "") {
//		LOG("Subscribing to occupancy: %s\n", grid.c_str());
		subscribe = nh.subscribe(grid, 1, &crosbot::gui::ImagePanel::callbackOccupancyGrid, this);
	}
}

void ImagePanel::stop() {
	if (topic != "" || grid != "") {
		subscribe.shutdown();
	}
}

void ImagePanel::callbackImage(const sensor_msgs::ImageConstPtr ros) {
	crosbot::ImagePtr image = new crosbot::Image(ros);
	image = image->inEncoding(Image::RGB8);

	label.setImage(image);
}


void ImagePanel::callbackOccupancyGrid(const nav_msgs::OccupancyGridConstPtr map) {
	crosbot::ImagePtr image = new crosbot::Image(Image::RGB8, map->info.height, map->info.width);
	uint8_t* pixel = (uint8_t*)image->data;

	for (uint32_t y = 0; y < image->height; ++y) {
		uint32_t rowStart = image->width * (image->height - y - 1);
		for (uint32_t x = 0; x < image->width; ++x, pixel += 3) {
			int8_t v = map->data[rowStart+x];
			if (v < 0) {
				pixel[0] = 0; pixel[1] = 0; pixel[2] = 255;
			} else if (v > 100) {
				pixel[0] = 255; pixel[1] = 255; pixel[2] = 255;
			} else {
				pixel[0] = pixel[1] = pixel[2] = round(255 * (v / 100.0));
			}
		}
	}

	label.setImage(image);
}

ScrollPanel::ScrollPanel(ConfigElementPtr config) : Panel(config), panel(NULL) {
	scroll.setAlignment(Qt::AlignCenter);

	for (unsigned int i = 0; panel == NULL && i < config->getChildCount(); i++) {
		panel = PanelFactory::createPanel(config->getChild(i));
	}
	if (panel != NULL) {
		scroll.setWidget(panel->getWidget());
	}
}

ScrollPanel::~ScrollPanel() {
	if (panel != NULL)
		delete panel;
	panel = NULL;
}

QWidget *ScrollPanel::getWidget() {
	return &scroll;
}

void ScrollPanel::start() {
	if (panel != NULL)
		panel->start();
}

void ScrollPanel::stop() {
	if (panel != NULL)
		panel->stop();
}

#define ORIENTATION_HORIZONTAL	"horizontal"
#define ORIENTATION_H			"h"
#define ORIENTATION_VERTICAL	"vertical"
#define ORIENTATION_V			"v"

SplitPanel::SplitPanel(ConfigElementPtr config) : Panel(config) {
	double maxsize = 0;
	double tmpsize;
	std::vector<int> szlist;
	std::string orientation = config->getParam(PARAM_ORIENTATION, ORIENTATION_HORIZONTAL);

	if (strcasecmp(orientation.c_str(), ORIENTATION_V) == 0 ||
			strcasecmp(orientation.c_str(), ORIENTATION_VERTICAL) == 0) {
		split.setOrientation(Qt::Vertical);
	} else {
		split.setOrientation(Qt::Horizontal);
	}

	QList<int> sizes;
	int subPanels = 0;
	for (unsigned int i = 0; i < config->getChildCount(); i++) {
		ConfigElementPtr childConfig = config->getChild(i);

		Panel *panel = PanelFactory::createPanel(childConfig);
		if (panel != NULL) {
			split.addWidget(panel->getWidget());
			split.setStretchFactor(subPanels, 1);
			subPanels++;
			tmpsize = childConfig->getParamAsInt("SplitSubSize", 100);
			szlist.push_back(tmpsize);
			maxsize += tmpsize;

			panels.push_back(panel);
		}
	}
	for (int i = 0; i < subPanels; i++) {
//		sizes.append(1000/subPanels);
		sizes.append((1000) * (szlist[i] / maxsize));
	}
	split.setSizes(sizes);
}

SplitPanel::~SplitPanel() {
	for (uint32_t p = 0; p < panels.size(); ++p) {
		if (panels[p] != NULL) {
			delete panels[p];
		}
	}
	panels.clear();
}

QWidget *SplitPanel::getWidget() {
	return &split;
}

void SplitPanel::start() {
	for (size_t i = 0; i < panels.size(); i++) {
		panels[i]->start();
	}
}

void SplitPanel::stop() {
	for (size_t i = 0; i < panels.size(); i++) {
		panels[i]->stop();
	}
}

TabPanel::TabPanel(ConfigElementPtr config) : Panel(config) {
	for (unsigned int i = 0; i < config->getChildCount(); i++) {
		ConfigElementPtr childConfig = config->getChild(i);

		if (childConfig->getChildCount() != 1 || strcasecmp(childConfig->getChildName().c_str(), ELEMENT_TAB)) {
//			logger->log(LOG_IMPORTANT, "TabPanel: Unable to configure tab for element %s.\n", childConfig->name.c_str());
			continue;
		}
		std::string name = childConfig->getParam(PARAM_NAME);
		std::string icon = childConfig->getParam(PARAM_ICON);
		Panel * panel = PanelFactory::createPanel(childConfig->getChild(0));
		if (panel == NULL) {
//			logger->log(LOG_ERROR, "TabPanel: Unable to create panel for tab %s.\n", name.c_str());
		} else {
			if (icon != "") {
				QIcon qicon(QString(icon.c_str()));

				tabs.addTab(panel->getWidget(), qicon, QString(name.c_str()));
			} else {
				tabs.addTab(panel->getWidget(), QString(name.c_str()));
			}

			panels.push_back(panel);
		}
	}
}

TabPanel::~TabPanel() {
	for (uint32_t p = 0; p < panels.size(); ++p) {
		delete panels[p];
	}
	panels.clear();
}

QWidget *TabPanel::getWidget() {
	return &tabs;
}

void TabPanel::start() {
	for (size_t i = 0; i < panels.size(); i++) {
		panels[i]->start();
	}
}

void TabPanel::stop() {
	for (size_t i = 0; i < panels.size(); i++) {
		panels[i]->stop();
	}
}

} // namespace gui

} // namespace casros
