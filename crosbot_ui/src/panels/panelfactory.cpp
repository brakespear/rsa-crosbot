/*
 * panelfactory.cpp
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */
#include <ros/ros.h>
#include <crosbot_ui/panels/panelfactory.hpp>
#include <crosbot_ui/panels/simple.hpp>
#include <crosbot_ui/panels/pointcloud.hpp>
#include <crosbot_ui/panels/map.hpp>
#include <crosbot_ui/panels/snap.hpp>
#include <crosbot_ui/panels/snap2.hpp>
#include <crosbot_ui/panels/robot.hpp>

namespace crosbot {

namespace gui {

Panel::Panel(ConfigElementPtr config) {}
Panel::~Panel() {}
void Panel::start() {}
void Panel::stop() {}

int Panel::getKeyForChar(char c) {
	// If c is a printable character then its value is the ascii value
		if (c >= 0x20 && c <= 0x7e) {
			if (isalpha(c))
				return toupper(c);
			return c;
		}
	return 0;
}

std::vector<PanelFactoryFunc> panelFactories;

Panel *PanelFactory::createPanel(ConfigElementPtr config) {
	Panel *rval = NULL;

	for (size_t i = 0; rval == NULL && i < panelFactories.size(); i++) {
		rval = panelFactories[i](config);
	}

	if (rval == NULL) {
		if (strcasecmp(config->getChildName().c_str(), PANEL_IMAGE) == 0) {
			rval = new ImagePanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_SCROLL) == 0) {
			rval = new ScrollPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_SPLIT) == 0) {
			rval = new SplitPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_TABS) == 0 ||
				strcasecmp(config->getChildName().c_str(), PANEL_TAB) == 0) {
			rval = new TabPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_POINTCLOUD) == 0) {
			rval = new PointCloudPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_MAP) == 0) {
			rval = new MapPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_SNAP) == 0 ||
				strcasecmp(config->getChildName().c_str(), ELEMENT_SNAPS) == 0) {
			rval = new SnapPanel(config);
		} else if (strcasecmp(config->getChildName().c_str(), "snap2") == 0 ||
				strcasecmp(config->getChildName().c_str(), "snaps2") == 0) {
			rval = new SnapPanel2(config);
		} else if (strcasecmp(config->getChildName().c_str(), PANEL_ROBOT) == 0) {
			rval = new RobotPanel(config);
		}
	}
	return rval;
}

void PanelFactory::addFactory(PanelFactoryFunc factory) {
	panelFactories.push_back(factory);
}

} // namespace gui

} // namespace casros
