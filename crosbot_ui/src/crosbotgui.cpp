/*
 * robotgui.cpp
 *
 *  Created on: 07/08/2009
 *      Author: rescue
 */

#include <ros/ros.h>
#include <crosbot_ui/crosbotgui.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_ui/opengl.hpp>

#include <crosbot_ui/panels/panelfactory.hpp>

#include <ros/console.h>
#include <string.h>
#include <QtGui/QIcon>
#include <GL/gl.h>

#include <dlfcn.h>

namespace crosbot {
using namespace ros::console;

namespace gui {

#define ATTRIB_OPENGL	"opengl"

#define ROS_NAME	"Gui"

MapManager Gui::maps;

Gui::Gui() : HandledObject() {
	window.setWindowTitle("Crosbot Gui");
	showSizex_ = 0;
	showSizey_ = 0;
	showPosx_ = -1;
	showPosy_ = -1;
	showMaxed_ = false;

	usePOTTextures = true;
#ifdef	GL_ARB_texture_non_power_of_two
#if GL_ARB_texture_non_power_of_two
	usePOTTextures = false;
#endif
#endif
}

Gui::~Gui() {
	stopGUI();

	for (uint32_t p = 0; p < panels.size(); ++p) {
		if (panels[p] != NULL)
			delete panels[p];
	}
	panels.clear();
}

void Gui::configure(ConfigElementPtr config) {
	bool guiConfigured = false;

	if (config->hasParam(ATTRIB_OPENGL)) {
		std::string opengl = config->getParam(ATTRIB_OPENGL, "");
		if (strcasecmp(opengl.c_str(), "pot") == 0 || strcasecmp(opengl.c_str(), "po2") == 0) {
			usePOTTextures = true;
		}
	}

	for (unsigned int i = 0; i < config->getChildCount(); i++) {
		ConfigElementPtr childConfig = config->getChild(i);
		// TODO: Add library inclusion.
		if (strcasecmp(childConfig->getChildName().c_str(), ELEMENT_LIBRARY) == 0) {
			loadLibrary(childConfig);
//		} else if (strcasecmp(childConfig->name.c_str(), ELEMENT_BENCHMARK) == 0) {
//            loadBenchmark(childConfig);
		} else if (strcasecmp(childConfig->getChildName().c_str(), ELEMENT_MAP) == 0) {
			MapPtr map = MapFactory::createMap(childConfig);
			if (map != NULL) {
				std::string name = childConfig->getParam(PARAM_NAME, "Map");
				maps.addMap(name, map);
			}
		} else if (strcasecmp(childConfig->getChildName().c_str(), "viewport") == 0) {
			showMaxed_ = childConfig->getParamAsBool("Maximize", false);
			showPosx_ = childConfig->getParamAsInt("x", -1);
			showPosy_ = childConfig->getParamAsInt("y", -1);
			showSizex_ = childConfig->getParamAsInt("w", 0);
			showSizey_ = childConfig->getParamAsInt("h", 0);
		} else if (strcasecmp(childConfig->getChildName().c_str(), ELEMENT_GUI) == 0) {
			if (guiConfigured) {
				ROS_LOG(levels::Error, ROS_NAME, "The GUI has been configured by a previous element.\n");
				continue;
			}
			for (unsigned int j = 0; j < childConfig->getChildCount(); j++) {
				ConfigElementPtr grandchildConfig = childConfig->getChild(j);
				if (strcasecmp(grandchildConfig->getChildName().c_str(), ELEMENT_MENU) == 0) {
					// TODO: Gui: setup the menu
				} else if (!guiConfigured) {
					Panel *panel = PanelFactory::createPanel(grandchildConfig);
					if (panel != NULL) {
						QWidget *w = panel->getWidget();
						window.setCentralWidget(w);
						w->setParent(&window);
						guiConfigured = true;
						panels.push_back(panel);
					} else {
						ROS_LOG(levels::Error, ROS_NAME, "Unable to create panel for element %s.\n", grandchildConfig->getChildName().c_str());
					}
				}
			}
		}
	}
}

void Gui::startGUI() {
	if (showMaxed_) {
		if ((showPosx_ >= 0) && (showPosy_ >= 0)) {
			window.move(showPosx_, showPosy_);
		}
		window.showMaximized();
	} else {
		if ((showSizex_ != 0) && (showSizey_ != 0)) {
			window.resize(showSizex_, showSizey_);
		}
		if ((showPosx_ >= 0) && (showPosy_ >= 0)) {
			window.move(showPosx_, showPosy_);
		}
		window.show();
	}
	
//	LOG("Starting Maps.\n");
	maps.startMaps();

//	LOG("Starting Panels.\n");
	std::vector<Panel *>::iterator it = panels.begin();
	while (it != panels.end()) {
		(*it)->start();
		it++;
	}

	rosThread.start();
}

void Gui::stopGUI() {
//	window.show();
	
	std::vector<Panel *>::iterator it = panels.begin();
	while (it != panels.end()) {
		(*it)->stop();
		it++;
	}
	
	maps.stopMaps();
	maps.clear();

	window.close();
}

GuiPtr Gui::createGUI(ConfigElementPtr config) {
	GuiPtr rval = new Gui();
	rval->configure(config);
	return rval;
}

void Gui::addPanel(Panel *panel) {
	panels.push_back(panel);
}

//Panel * Gui::getPanel(QString name) {
//	unsigned int i;
//	for (i = 0; i < panels.size(); i++) {
//		if (panels[i]->getName() == name) {
//			return panels[i];
//		}
//	}
//	return NULL;
//}


GuiWindow::GuiWindow() : QMainWindow() {
	setWindowIcon(QIcon(":/cas-icon.png"));
}

GuiWindow::~GuiWindow() {}

QSize GuiWindow::sizeHint() {
	return QSize(1300, 600);
}

Gui::ROSThread::ROSThread() : Thread("ROS_Thread"), operating(true) {}

void Gui::ROSThread::run() {
	while(operating && ros::ok()) {
		ros::spin();
	}
}

void Gui::ROSThread::stop() {
	operating = false;

	for (int i = 0; isAlive() && i < DEFAULT_WAIT_FOR_THREAD_CLOSE; i+= 10) {
		usleep(10000);
	}

	if (isAlive()) {
		ERROR("ROSThread is refusing to close.\n");
	}
}

#define MAX_PACKAGE_LOC		4096
void Gui::loadLibrary(ConfigElementPtr config) {
	std::string package = config->getParam("pkg", "");
	std::string find = config->getParam("find", "");
	std::string libraryFile = config->getParam(PARAM_FILE, "");

	std::string packageLoc;
	if (package != "") {
		std::string consoleCmd = "rospack find ";
		consoleCmd.append(package);

		FILE *console = popen(consoleCmd.c_str(), "r");
		char buffer[MAX_PACKAGE_LOC+1], buffer2[256];
		sprintf(buffer2, "%%%us", MAX_PACKAGE_LOC);
		fscanf(console, buffer2, buffer);
		pclose(console);

		packageLoc = buffer;
//		LOG("GUI::loadLibrary() - package location: %s\n", packageLoc.c_str());
	}

	if (find != "") {
		std::string consoleCmd = "catkin_find ";
		consoleCmd.append(find);

		FILE *console = popen(consoleCmd.c_str(), "r");
		char buffer[MAX_PACKAGE_LOC+1], buffer2[256];
		sprintf(buffer2, "%%%us", MAX_PACKAGE_LOC);
		fscanf(console, buffer2, buffer);
		pclose(console);

		if (strlen(buffer) == 0 || buffer[0] != '/') {
			ERROR("Gui: Unable to find library %s to load.\n", find.c_str());
			return;
		}

		packageLoc = buffer;
//		LOG("GUI::loadLibrary() - library location: %s\n", packageLoc.c_str());

		libraryFile = packageLoc;
		packageLoc = "";
	}

	if (libraryFile == "") {
		ERROR("Gui: No filename given for dynamic library.\n");
		return;
	}

	libraryFile = packageLoc + libraryFile;

	LOG("Gui: Loading library: %s\n", libraryFile.c_str());

	void *handle = dlopen(libraryFile.c_str(), RTLD_NOW | RTLD_GLOBAL);
//	void *handle = dlopen(libraryFile.c_str(), RTLD_NOW);

	if (handle == NULL) {
		ERROR("Gui: Unable to load library %s. (error: %s)\n", libraryFile.c_str(), dlerror());
	}
}


} // namespace gui

} // namespace casros
