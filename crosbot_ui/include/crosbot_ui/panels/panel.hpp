/*
 * rgpanel.h
 *
 *  Created on: 18/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_HPP_
#define CROSBOT_PANEL_HPP_

#include <QtCore/QObject>
#include <crosbot/config.hpp>

namespace crosbot {

namespace gui {

class Panel : public QObject {
Q_OBJECT
public:
	Panel(ConfigElementPtr config);
	virtual ~Panel();
	virtual QWidget *getWidget()=0;
	virtual void start();
	virtual void stop();

	static int getKeyForChar(char c);
protected:
	std::string name;
};

} // namespace gui

} //namespace crosbot

#endif /* CROSBOT_PANEL_HPP_ */
