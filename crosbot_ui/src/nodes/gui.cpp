/*
 * main.cpp
 *
 *  Created on: 30/07/2009
 *      Author: rescue
 */

#include <string>
#include <string.h>
#include <stdio.h>

#include <ros/ros.h>
#define MODULE_NAME 	"gui"
#include <crosbot_ui/crosbotgui.hpp>

#include <signal.h>
#include <QtGui/QApplication>
#include <QtCore/QSettings>
#include <QtGui/QFileDialog>
#include <GL/glut.h>

//#define LOG(...)	ROS_LOG(ros::console::levels::Info, "casgui", __VA_ARGS__)
//#define DEBUG(...)	ROS_LOG(ros::console::levels::Debug, "casgui", __VA_ARGS__)
//#define ERROR(...)	ROS_LOG(ros::console::levels::Error, "casgui", __VA_ARGS__)

using namespace crosbot;
using namespace crosbot::gui;
using namespace std;

GuiPtr myGui;

bool interrupted = false;
void catch_int(int sig_num)
{
	signal(SIGINT, catch_int);
	interrupted = true;

	if (myGui != NULL) {
		myGui->getWindow()->close();
	}
}

void printConfig(ConfigElementPtr config, string indent) {
	printf("%s%s", indent.c_str(), config->name.c_str());

//	for (unsigned int i = 0; i < config-> attributes.size(); i++)
//		printf(" %s=\"%s\"", config->attributes[i].c_str(), config->values[i].c_str());
	printf("\n");
	string childIndent = indent;
	childIndent.append("\t");
	for (unsigned int i = 0; i < config->getChildCount(); i++)
		printConfig(config->getChild(i), childIndent);
}

void printConfig(ConfigElementPtr config) {
	printConfig(config, "");
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "gui");
    ros::NodeHandle n("~");

    ConfigElementPtr config;
    if (argc > 1) {
    	LOG("Loading configuration from file %s\n", argv[1]);
    	config = ConfigElement::parseFile(argv[1]);

    	if (config == NULL && strlen(argv[1]) > 0 && argv[1][0] != '/') {
    		std::string cmd = argv[0];

    		size_t f = cmd.find_last_of('/');
    		if (f != cmd.npos) {
    			std::string file = cmd.substr(0, f) + "/../" + argv[1];
    			config = ConfigElement::parseFile(file.c_str());
    		}
    	}
    }
    if (config == NULL)
    	config = new ROSConfigElement(n);

	signal(SIGINT, catch_int);
	
//	printConfig(config);
	
	// Seed random number generator.
	srand(Time::now().sec);
	
	QApplication app( argc, argv );
	app.setApplicationName(QString("gui"));
	glutInit(&argc, argv);

	myGui = Gui::createGUI(config);
	int rval = 0;
	
	if (myGui == NULL) {
		ERROR("Unable to create GUI.\n");
		return -1;
	}

	LOG("Starting GUI.\n");
	
	try {
		myGui->startGUI();
	} catch (std::exception& e) {
		ERROR("Exception(%s) thrown while starting gui.\n", e.what());
		rval = -1;
	} catch (...) {
		ERROR("Error thrown while starting gui.\n");
		rval = -1;
	}
	
	if (rval == 0) {
		LOG("GUI started.\n");
		app.connect( &app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
		rval = app.exec();
	}

	LOG("Closing GUI.\n");

	myGui->stopGUI();
	myGui = NULL;

//	LOG("Saving Qt settings.\n");
	QFileDialog fd;
	QSettings settings("Trolltech");
	settings.beginGroup("Qt");
	settings.setValue("filedialog", fd.saveState());
	settings.endGroup();
	settings.sync();
//	LOG("Saved Qt settings.\n");

	// Output benchmarking statistics
//    benchmark->displayBenchmarks();

	exit(rval);
	return rval;
}

