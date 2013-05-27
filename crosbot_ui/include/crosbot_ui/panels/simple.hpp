/*
 * simple.h
 *
 * A number of very simple panels that can be used to build a user interface.
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_SIMPLE_HPP_
#define CROSBOT_PANEL_SIMPLE_HPP_

#include <crosbot_ui/panels/panel.hpp>
#include <crosbot/data.hpp>

#include <QtGui/QLabel>
#include <QtGui/QScrollArea>
#include <QtGui/QSplitter>
#include <QtGui/QTabWidget>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/OccupancyGrid.h>

namespace crosbot {

namespace gui {

#define PANEL_IMAGE			"image"
#define PANEL_SCROLL		"scroll"
#define PANEL_SPLIT			"split"
#define PANEL_TAB			"tab"
#define PANEL_TABS			"tabs"

#define ELEMENT_TAB			"tab"

#define PARAM_ICON			"icon"
#define PARAM_IMAGE			"image"
#define PARAM_ORIENTATION	"orientation"
#define PARAM_POSITION		"position"
#define PARAM_SRC			"src"

class ImageLabel : public QFrame {
Q_OBJECT
protected:
	QImage qImage;
	ImagePtr image;
	void resizeEvent(QResizeEvent *);
	void paintEvent(QPaintEvent *);
public:
	ImageLabel(QString);

	void setImage(ImagePtr image);
	void setImage(QImage image);
public Q_SLOTS:
	void update();

Q_SIGNALS:
	void resized();
	void imageUpdated();
};

/**
 * \image html panel-image.png
 */
class ImagePanel : public Panel {
Q_OBJECT
public:
	ImagePanel(ConfigElementPtr config);
	~ImagePanel();

	QWidget *getWidget();
	void start();
	void stop();

	void callbackImage(const sensor_msgs::ImageConstPtr);
	void callbackOccupancyGrid(const nav_msgs::OccupancyGridConstPtr);

protected:
	ImageLabel label;

	std::string topic, grid;
	ros::Subscriber subscribe;
};

/**
 * \image html panel-scroll.png
 */
class ScrollPanel : public Panel {
Q_OBJECT
public:
	ScrollPanel(ConfigElementPtr config);
	~ScrollPanel();

	QWidget *getWidget();
	void start();
	void stop();
protected:
	QScrollArea scroll;
	Panel *panel;
};

/**
 * \image html panel-split.png
 */
class SplitPanel : public Panel {
Q_OBJECT
public:
	SplitPanel(ConfigElementPtr config);
	~SplitPanel();

	QWidget *getWidget();
	void start();
	void stop();
protected:
	QSplitter split;
	std::vector<Panel *>panels;
};

/**
 * \image html panel-tabs.png
 */
class TabPanel : public Panel {
Q_OBJECT
public:
	TabPanel(ConfigElementPtr config);
	~TabPanel();

	QWidget *getWidget();
	void start();
	void stop();
protected:
	QTabWidget tabs;
	std::vector<Panel *>panels;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_SIMPLE_HPP_ */
