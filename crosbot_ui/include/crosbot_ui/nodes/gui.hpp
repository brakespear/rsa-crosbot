/*
 * gui.hpp
 *
 * Allows the crosbot_ui gui node to be instantied from another package
 * to allow those packages to inlucde their own libraries
 *
 *  Created on: 27/06/2014
 *      Author: timothyw
 */

#ifndef GUI_HPP_
#define GUI_HPP_

namespace crosbot {
namespace gui {

int run_crosbot_ui_main(int argc, char **argv);

} // namespace gui
} // namespace crosbot


#endif /* GUI_HPP_ */
