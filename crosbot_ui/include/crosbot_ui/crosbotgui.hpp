/*
 * robotgui.h
 *
 *  Created on: 31/07/2009
 *      Author: rescue
 */

#ifndef CROSBOTGUI_HPP_
#define CROSBOTGUI_HPP_

#include <crosbot/utils.hpp>
#include <crosbot_ui/panels/panel.hpp>
#include <crosbot_map/map.hpp>

#include <QtGui/QMainWindow>

namespace crosbot {

namespace gui {

#define ELEMENT_MENU	"menu"
#define ELEMENT_LIBRARY	"library"

class Gui;
typedef Handle<Gui> GuiPtr;

class GuiWindow : public QMainWindow {
Q_OBJECT
public:
	GuiWindow();
	~GuiWindow();
	QSize sizeHint();
protected:
};

class Gui : public HandledObject {
protected:
	Gui();
	~Gui();
	GuiWindow window;
	
	std::vector<Panel *> panels;

	bool showMaxed_;
	int showSizex_;
	int showSizey_;
	int showPosx_;
	int showPosy_;

	class ROSThread : public Thread {
	public:
		bool operating;

		ROSThread();
		void run();
		void stop();
	};

	ROSThread rosThread;
public:
	void configure(ConfigElementPtr config);
	
	void startGUI();
	void stopGUI();
	
	static GuiPtr createGUI(ConfigElementPtr config);
	
	void addPanel(Panel *panel);

	GuiWindow *getWindow() {
		return &window;
	}

	void loadLibrary(ConfigElementPtr config);

	static MapManager maps;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOTGUI_HPP_ */
