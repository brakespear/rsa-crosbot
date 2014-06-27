/*
 * gui_node.cpp
 *
 * Wrapper around the gui.cpp class which actually runs the node
 *
 *  Created on: 27/06/2014
 *      Author: timothyw
 */


#include <crosbot_ui/nodes/gui.hpp>

int main(int argc, char **argv) {
    return crosbot::gui::run_crosbot_ui_main(argc, argv);
}
