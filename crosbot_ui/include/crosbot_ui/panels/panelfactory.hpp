/*
 * panelfactory.h
 *
 *  Created on: 13/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_PANELFACTORY_HPP_
#define CROSBOT_PANELFACTORY_HPP_

#include <crosbot/config.hpp>
#include <crosbot_ui/panels/panel.hpp>

namespace crosbot {

namespace gui {

typedef Panel* (*PanelFactoryFunc)(ConfigElementPtr config);

class PanelFactory {
public:
	static Panel *createPanel(ConfigElementPtr config);
	static void addFactory(PanelFactoryFunc factory);
};

} //namespace gui

} //namespace crosbot

#endif /* CROSBOT_PANELFACTORY_HPP_ */
