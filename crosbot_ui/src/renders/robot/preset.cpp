/*
 * preset.cpp
 *
 *  Created on: 26/03/2012
 *      Author: rescue
 */

#include <crosbot_ui/renders/robot/preset.hpp>
#include <crosbot_ui/renders/robot/jointcontrol.hpp>

#include <QtGui/QKeyEvent>

namespace crosbot {

namespace gui {

std::vector<std::string> getParamAsList(std::string param) {
	std::vector<std::string> rval;

	std::string elem;
	char c[2], cOld = '\0';
	c[1] = '\0';
	for (unsigned int i = 0; i < param.size(); i++) {
		c[0] = param.at(i);
		if (isspace(c[0])) {
			if (cOld != '\0') {
				rval.push_back(elem);
				elem = "";
				cOld = '\0';
			}
		} else {
			elem.append(c);
			cOld = c[0];
		}
	}
	if (cOld != '\0') {
		rval.push_back(elem);
		elem = "";
		cOld = '\0';
	}

	return rval;
}

PresetRender::PresetRender(RobotPanel& panel, ConfigElementPtr config) :
		RobotRender(panel, config, QRectF(0, 0, 1, 0.2)),
		renderPresets(true), layout(2)
{
	presets.resize(10);
	int setPresets = 0;
	for (size_t i = 0; i < config->getChildCount() && setPresets < 10; ++i) {
		ConfigElementPtr child = config->getChild(i);

		if (strcasecmp(child->name.c_str(), "preset") == 0) {
			std::string set = child->getParam("value");
			std::vector< std::string > angs = getParamAsList(set);

			if (angs.size() > 0) {
				std::vector<float> preset;
				for (size_t j = 0; j < angs.size(); ++j) {
					preset.push_back(atof(angs[j].c_str()));
				}
				presets[setPresets++] = preset;
			}
		}
	}
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

EmuRender::EmuRender(RobotPanel& panel, ConfigElementPtr config) : PresetRender(panel, config) {}

std::vector<float> EmuRender::getConfiguration() {
	std::vector<float> rval;
	rval.push_back(RAD2DEG(JointController::getPos("arm_base")));
	rval.push_back(RAD2DEG(JointController::getPos("arm_shoulder")));
	rval.push_back(RAD2DEG(JointController::getPos("arm_elbow")));
	rval.push_back(RAD2DEG(JointController::getPos("neck_tilt")));
	rval.push_back(RAD2DEG(JointController::getPos("neck_pan")));
	return rval;
}

void EmuRender::setConfiguration(std::vector<float> preset) {
	if (preset.size() > 0 && VALID_PRESET(preset[0])) {
		float armAngle = DEG2RAD(preset[0]);
		JointController::setPos("arm_base", preset[0]);
	}
	if (preset.size() > 1 && VALID_PRESET(preset[1])) {
		float armAngle = DEG2RAD(preset[1]);
		JointController::setPos("arm_shoulder", preset[1]);
	}
	if (preset.size() > 2 && VALID_PRESET(preset[2])) {
		float armAngle = DEG2RAD(preset[0]);
		JointController::setPos("arm_elbow", preset[2]);
	}
	if (preset.size() > 3 && VALID_PRESET(preset[3])) {
		float armAngle = DEG2RAD(preset[3]);
		JointController::setPos("neck_tilt", preset[3]);
	}
	if (preset.size() > 4 && VALID_PRESET(preset[4])) {
		float armAngle = DEG2RAD(preset[4]);
		JointController::setPos("neck_pan", preset[4]);
	}
}

void EmuRender::renderRobot(std::string label, std::vector<float> configuration, bool renderGoals) {
	// Set the colours used in the render
	float lightpos[4] = {0,0,0,1};
	float white[4] = {1.0, 1.0f, 1.0f,1.0f};
	float baseMat[4] = {0,1,1.0f, 0.6f};
	float wheelMat[4] = {0.9,0,1.0f, 0.6f};
	float armMat[4] = {0,0,1.0f, 0.6f};
	float goalMat[4] = {1.0,0,0,0.6f};

	// The robot's measurements
	float baseWidth = 0.32, baseLength = 0.36, baseHeight = 0.14;
	float wheelHeight = 0.24, wheelWidth = .08, wheelClearence = 0.09, wheelSpacing = 0.02;
	float cubeSide = 0.07;//, cubeFromRightSide = 0.08;  -- not used
	float armWidth = 0.04, armDepth = 0.02, armLength = 0.60;
	float headWidth = 0.14, headLength = 0.17, headHeight = 0.09;
	float headFromArm = 0.03;

	float baseA = JointController::getPos("arm_base"),
			shoulderA = JointController::getPos("arm_shoulder"),
			elbowA = JointController::getPos("arm_elbow"),
			tiltA = JointController::getPos("neck_tilt"),
			panA = JointController::getPos("neck_pan");
//	PanTiltZoom ptz = panel->getCurrentPanTilt();


	if (configuration.size() > 0 && VALID_PRESET(configuration[0])) {
		baseA = DEG2RAD(configuration[0]);
	}
	if (configuration.size() > 1 && VALID_PRESET(configuration[1])) {
		shoulderA = DEG2RAD(configuration[1]);
	}
	if (configuration.size() > 2 && VALID_PRESET(configuration[2])) {
		elbowA = DEG2RAD(configuration[2]);
	}
	if (configuration.size() > 3 && VALID_PRESET(configuration[3])) {
		tiltA = DEG2RAD(configuration[3]);
	}
	if (configuration.size() > 4 && VALID_PRESET(configuration[4])) {
		panA = DEG2RAD(configuration[4]);
	}

	// Origin is where the arm joins the base

	if(!displayListsSet){
		displayLists = glGenLists(6);
		//The Base of the robot
		glNewList(displayLists, GL_COMPILE);
			glPushMatrix();
				glTranslatef(0,-baseHeight/2-cubeSide/2, -(baseLength-cubeSide)/2);
				glScalef(baseWidth, baseHeight, baseLength);
				glutSolidCube(1);
			glPopMatrix();
//        	glEnd();
		glEndList();


		// The power cubes
		glNewList(displayLists+1, GL_COMPILE);
			glPushMatrix();
				glScalef(cubeSide, cubeSide, cubeSide);
				glutSolidCube(1);
			glPopMatrix();
		glEndList();


		// The robots arm
		glNewList(displayLists+2, GL_COMPILE);
			glPushMatrix();
				glTranslatef(0, armLength/2 - cubeSide/2, (armDepth+cubeSide)/2);
				glScalef(armWidth, armLength, armDepth);
				glutSolidCube(1);
			glPopMatrix();
		glEndList();

		//The sensor head
		glNewList(displayLists+3, GL_COMPILE);
			glPushMatrix();
				glTranslatef(0, headHeight/2 + headFromArm, 0);
				//Rays show fov roughly
				glDisable(GL_LIGHTING);
				glColor3f(1,1,0);
				glBegin(GL_LINES);
					glVertex3f(0,0,0);
					glVertex3f(0,0,0.6);
				glEnd();
				glEnable(GL_LIGHTING);
				glScalef(headWidth, headHeight, headLength);
	//        	glScalef(0.11, 0.13, 0.09);
				glutSolidCube(1);
			glPopMatrix();
		glEndList();

		// The Wheels
		glNewList(displayLists+4, GL_COMPILE);
			glPushMatrix();
				GLUquadric *quad = gluNewQuadric();
				gluQuadricNormals(quad, GLU_SMOOTH);
				gluQuadricTexture(quad, GLU_TRUE);
				gluQuadricDrawStyle(quad, GLU_FILL);

				glScalef(wheelWidth, wheelHeight, wheelHeight);
				glRotatef(90,0,1,0);

				gluCylinder(quad, 0.5, 0.5, 1, 12, 1);
				gluDisk(quad,0,0.5,12,1);
				glTranslatef(0, 0, 1);
				gluDisk(quad,0,0.5,12,1);

				gluDeleteQuadric(quad);
			glPopMatrix();
		glEndList();

		displayListsSet = true;
	}

	glEnable(GL_NORMALIZE);
	glEnable(GL_RESCALE_NORMAL);
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	glPushMatrix();
		glScalef(0.6,0.6,0.6);
		glTranslatef(1,1,0);
		glScalef(-1.5,1.5,1.5);
		glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
		glTranslatef(0.25,-0.25,1);
		glRotatef(45,1,0,0);
		glRotatef(-135,0,1,0);
		glColor4f(0,1,0,0.5);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, white);

		glTranslatef(0,-0.2,0.4);
		// Draw Robot
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, baseMat);
		glEnable(GL_LIGHT0);
		glEnable(GL_LIGHTING);
		glCallList(displayLists);

		// draw fixed power cube
		glPushMatrix();
			glTranslatef(cubeSide/2, 0, 0);

			glCallList(displayLists + 1);
		glPopMatrix();

		// Draw wheels
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, wheelMat);
		glPushMatrix();
			float ymove = -cubeSide/2 - baseHeight + (wheelHeight /2 - wheelClearence);
			float zmove1 = (-wheelHeight - wheelSpacing + baseLength - cubeSide)/2;
			float zmove2 = zmove1 - (wheelHeight + wheelSpacing);
			glPushMatrix();
				glTranslatef(baseWidth/2, ymove, zmove1);
				glCallList(displayLists + 4);
			glPopMatrix();
			glPushMatrix();
				glTranslatef(-baseWidth/2-wheelWidth, ymove, zmove1);
				glCallList(displayLists + 4);
			glPopMatrix();
			glPushMatrix();
				glTranslatef(baseWidth/2, ymove, zmove2);
				glCallList(displayLists + 4);
			glPopMatrix();
			glPushMatrix();
				glTranslatef(-baseWidth/2-wheelWidth, ymove, zmove2);
				glCallList(displayLists + 4);
			glPopMatrix();
		glPopMatrix();

		//  draw power cube
		glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, armMat);
		glPushMatrix();
			glRotatef(RAD2DEG(shoulderA), 1, 0, 0);
			glTranslatef(-cubeSide/2, 0, 0);

			glCallList(displayLists + 1);
		glPopMatrix();

		// draw arm
		glPushMatrix();
			glRotatef(RAD2DEG(shoulderA),1,0,0);
			glCallList(displayLists + 2);
		glPopMatrix();

		// draw head
		glPushMatrix();
			float distFromOrigin = armLength-cubeSide/2;
			glTranslatef(0, cos(shoulderA)*distFromOrigin, sin(shoulderA)*distFromOrigin);
			glRotatef(-RAD2DEG(tiltA),1,0,0);
			glRotatef(-RAD2DEG(panA),0,1,0);
			glCallList(displayLists + 3);
		glPopMatrix();

		//  draw head
		// If wer're drawing goal positions
//		if(renderGoals){
//			float goalArm = panel->getDesiredPos(armName);
//			PanTiltZoom goalPTZ = panel->getDesiredPanTilt();
//
//			glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, goalMat);
//
//			//  draw power cube
//			glPushMatrix();
//				glRotatef(RAD2DEG(-goalArm), 1, 0, 0);
//				glTranslatef(-cubeSide/2, 0, 0);
//
//				glCallList(displayLists + 1);
//			glPopMatrix();
//
//			// draw arm
//			glPushMatrix();
//				glRotatef(RAD2DEG(-goalArm),1,0,0);
//				glCallList(displayLists + 2);
//			glPopMatrix();
//
//			// draw head
//			glPushMatrix();
//				float distFromOrigin = armLength-cubeSide/2;
//				glTranslatef(0, cos(goalArm)*distFromOrigin, sin(-goalArm)*distFromOrigin);
//				glRotatef(-RAD2DEG(goalPTZ.tilt),1,0,0);
//				glRotatef(-RAD2DEG(goalPTZ.pan),0,1,0);
//				glCallList(displayLists + 3);
//			glPopMatrix();
//		}
	glPopMatrix();

	glDisable(GL_LIGHTING);
	glDisable(GL_LIGHT0);
	glColor3f(1,1,1);
	panel.getRobotWidget().renderText(0.8f,0.0f,0.0f,QString(label.c_str()),QFont("Helvetica", 8));
	glDisable(GL_RESCALE_NORMAL);
	glDisable(GL_NORMALIZE);
	glDisable(GL_DEPTH_TEST);
}

} // namespace gui

} // namespace crosbot
