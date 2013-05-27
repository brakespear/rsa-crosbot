/*
 * preset.cpp
 *
 *  Created on: 26/03/2012
 *      Author: rescue
 */

#include <crosbot_ui/renders/robot/preset.hpp>

#include <QKeyEvent>

namespace crosbot {

namespace gui {

PresetRender::PresetRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config),
		renderPresets(true), layout(1)
{
	presets.resize(10);
}

void PresetRender::render() {
	preRender();

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//	glEnable(GL_BLEND);
	if(!renderPresets){
		renderRobot("", std::vector<float>(), true);
	} else {
		// Now we have to chop up the screen.
		// if presetsAcross = 1 then it's 1/13th.
		// if presetsAcross = 2 then it's 1/7th
		// Proportion for the main picture is equal to (1/((12/presetsAcross)_+1);
		// glTranslatef(0, 1-(1.0/((12.0/presetsAcross_)+1.0));
		// glScalef(1, (1.0/((12.0/presetsAcross_)+1.0));
		// For now, just hack it. Make it three across.
		// Total is 7 units high. Top is 3 units, then each preset is 1 unit high.
		glColor4f(1.0,1.0,1.0,0.4); // Background colour of the render
		glBegin(GL_QUADS);
		glVertex2f(0,0);
		glVertex2f(1,0);
		glVertex2f(1,1);
		glVertex2f(0,1);
		glEnd();
		glPushMatrix();
			if(layout == 1){
				glTranslatef(0, 4.0/7.0,0);
				glScalef(1, 3.0/7.0, 1);
			}
			else if(layout == 2){
				glScalef(0.25,1,1);
			}
			renderRobot("", std::vector<float>(), true);
		glPopMatrix();
		std::string label;
		for(int i=0; i < 10; i++){
			if(i==9){
				label = "0";
			} else {
				char buffer[3];
				buffer[0] = (char)('1' + i);
				buffer[1] = '\0';
				label = buffer;
			}
			glPushMatrix();
				// They go across.
				if(layout == 1){
					int row = i/3;
					int column = i%3;
					glTranslatef(column/3.0,(3.0-row)/7.0,0);
					glScalef(1/3.0,1/7.0,0.2);
					renderRobot(label, presets[i], false);
				} else if(layout == 2) {
					int row = i%5;
					int column = i/5;
					glTranslatef(0.25+row*0.125,(1-column)*0.5,0);
					glScalef(0.125,0.5,0.2);
					renderRobot(label, presets[i], false);
				}
			glPopMatrix();
		 }
	}
//	glDisable(GL_BLEND);

	postRender();
}

bool PresetRender::keyPressEvent(QKeyEvent *ke) {
	if (renderPresets) {
		int preset = ke->key() - Qt::Key_1;
		if (ke->key() == Qt::Key_0)
			preset = 9;
		if (preset < 0 || preset > 9)
			return false;

		if (ke->modifiers() && Qt::AltModifier) {
			presets[preset] = getConfiguration();
		} else {
			setConfiguration(presets[preset]);
		}

		return true;
	}
	return false;
}

} // namespace gui

} // namespace crosbot
