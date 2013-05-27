/*
 * rgglwidget.cpp
 *
 *  Created on: 19/08/2009
 *      Author: rescue
 */
#include <crosbot_ui/renders/robot/robotwidget.hpp>
#include <crosbot_ui/panels/robot.hpp>

#include <QTimer>
#include <QKeyEvent>

#include <crosbot/utils.hpp>

namespace crosbot {

namespace gui {
using namespace std;

RobotWidget::RobotWidget(RobotPanel& panel) : panel(panel) {
	aspectRatio = 1.0;
	updateInterval = 50;

	installEventFilter(this);
	setFocusPolicy(Qt::WheelFocus);

	upPressed = downPressed = leftPressed = rightPressed = 0;
}

RobotWidget::~RobotWidget() {
}

void RobotWidget::setAspectRatio(float aspectRatio) {
	this->aspectRatio = aspectRatio;
}

float RobotWidget::getAspectRatio() {
	return aspectRatio;
}

void RobotWidget::initializeGL() {
    glClearColor(0.3f, 0.3f, 0.3f, 1.0f);
    glFlush();
}

bool RobotWidget::initialized() {
	return isInitialized;
}

void RobotWidget::resizeGL(int w, int h) {
    width = w;
    height = h;
}

void RobotWidget::paintGL() {
	isInitialized = true;
	
	int w = width, h = height;
    int xOffset = 0, yOffset =0;
    if((float) w/h > aspectRatio){
		w = (int) (height*aspectRatio);
        xOffset = (width-w)/2;
    } else if((float) w/h < aspectRatio) {
    	h = (int) (width/aspectRatio);
        yOffset = (height-h)/2;
    }
	
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, aspectRatio, 0, 1, -200, 200);
    glMatrixMode(GL_MODELVIEW);
    glViewport( xOffset, yOffset, w, h );

	glShadeModel(GL_FLAT);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glDisable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    for(unsigned int i = 0; i < renders.size(); i++){
    	RobotRender* render = renders[i];
    	if (!render->isHidden()) {
    		render->render();
    	}
    }
    glDisable(GL_BLEND);
    glEnable(GL_DEPTH_TEST);
}

void RobotWidget::setVisible(bool vis) {
	QGLWidget::setVisible(vis);
	QTimer *timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(updateGL()));
	timer->start(updateInterval);
}


void RobotWidget::addRender(RobotRender *render) {
	renders.push_back(render);
}

void RobotWidget::startRenders() {
	vector<RobotRender *>::iterator it = renders.begin();
	while (it != renders.end()) {
		(*it)->start();
		
		it++;
	}
}

void RobotWidget::stopRenders() {
	vector<RobotRender *>::iterator it = renders.begin();
	while (it != renders.end()) {
		(*it)->stop();
		
		it++;
	}
}

void RobotWidget::setUpdateInterval(int updateInterval) {
	this->updateInterval = updateInterval;
}

void RobotWidget::keyPressEvent(QKeyEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->keyPressEvent(e)) {
			return;
		}
	}

	if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up ||
			e->key() == Qt::Key_S || e->key() == Qt::Key_Down ||
			e->key() == Qt::Key_A || e->key() == Qt::Key_Left ||
			e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
		if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
			upPressed++;
		} else if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
			downPressed++;
		}if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
			leftPressed++;
		} else if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
			rightPressed++;
		}

		float throttle, turnThrottle, speed = 0, turn = 0;
		panel.getThrottle(throttle, turnThrottle);
		if (upPressed)
			speed += throttle;
		if (downPressed)
			speed -= throttle;
		if (leftPressed)
			turn += turnThrottle;
		if (rightPressed)
			turn -= turnThrottle;
		panel.setCurrentSpeeds(speed, turn);
	}
}

#define MAX(X,Y)	((X)>(Y)?(X):(Y))
void RobotWidget::keyReleaseEvent(QKeyEvent *e) {
	if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up ||
			e->key() == Qt::Key_S || e->key() == Qt::Key_Down ||
			e->key() == Qt::Key_A || e->key() == Qt::Key_Left ||
			e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
		if (e->key() == Qt::Key_W || e->key() == Qt::Key_Up) {
			upPressed = MAX(upPressed-1, 0);
		} else if (e->key() == Qt::Key_S || e->key() == Qt::Key_Down) {
			downPressed = MAX(downPressed-1, 0);
		}if (e->key() == Qt::Key_A || e->key() == Qt::Key_Left) {
			leftPressed = MAX(leftPressed-1, 0);
		} else if (e->key() == Qt::Key_D || e->key() == Qt::Key_Right) {
			rightPressed = MAX(rightPressed-1, 0);
		}

		float throttle, turnThrottle, speed = 0, turn = 0;
		panel.getThrottle(throttle, turnThrottle);
		if (upPressed)
			speed += throttle;
		if (downPressed)
			speed -= throttle;
		if (leftPressed)
			turn += turnThrottle;
		if (rightPressed)
			turn -= turnThrottle;
		panel.setCurrentSpeeds(speed, turn);
	}
}

void  RobotWidget::focusOutEvent(QFocusEvent *) {
	upPressed = downPressed = leftPressed = rightPressed = 0;
	panel.setCurrentSpeeds(0.0, 0.0);
}

void RobotWidget::mousePressEvent(QMouseEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->mousePressEvent(e)) {
			return;
		}
	}
}

void RobotWidget::mouseReleaseEvent(QMouseEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->mouseReleaseEvent(e)) {
			return;
		}
	}
}

void RobotWidget::mouseMoveEvent(QMouseEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->mouseMoveEvent(e)) {
			return;
		}
	}
}

void RobotWidget::mouseDoubleClickEvent(QMouseEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->mouseDoubleClickEvent(e)) {
			return;
		}
	}
}

void RobotWidget::wheelEvent(QWheelEvent *e) {
	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->wheelEvent(e)) {
			return;
		}
	}
}

RobotRender::RobotRender(RobotPanel& panel, ConfigElementPtr config) :
		panel(panel), rect(0,0,1,1),
		invertX(false), invertY(false), rotate(0)
{
	if (config->hasParam(PARAM_ROTATE)) {
		rotate = config->getParamAsDouble(PARAM_ROTATE, 0);
	}
	if (config->hasParam(PARAM_BOUNDS)) {
		std::string bounds = config->getParam(PARAM_BOUNDS);

		float x, y, w, h;
		int n = sscanf(bounds.c_str(), "%f %f %f %f", &x, &y, &w, &h);
		if (n == 4) {
			rect = QRectF(x,y,w,h);
		}
	}
}

RobotRender::RobotRender(RobotPanel& panel, ConfigElementPtr config, QRectF rect) :
		panel(panel), rect(rect),
		invertX(false), invertY(false), rotate(0)
{
	if (config->hasParam(PARAM_ROTATE)) {
		rotate = DEG2RAD(config->getParamAsDouble(PARAM_ROTATE, 0));
	}
	if (config->hasParam(PARAM_BOUNDS)) {
		std::string bounds = config->getParam(PARAM_BOUNDS);

		float x, y, w, h;
		int n = sscanf(bounds.c_str(), "%f %f %f %f", &x, &y, &w, &h);
		if (n == 4) {
			rect = QRectF(x,y,w,h);
		}
	}
}

RobotRender::~RobotRender() {}

//QRectF RobotRender::getDefaultBounds() {
//	LOG("RobotRender::getDefaultBounds()\n");
//	return QRectF(0,0,1,1);
//}

void RobotRender::preRender() {
//	LOG("RobotRender::preRender - rect(%.3lf, %.3lf, %.3lf, %.3lf)\n",
//			rect.x(), rect.y(), rect.width(), rect.height());

	// We assume that we have the current context.
	glPushMatrix();

	// Remember, in openGL the first transform is the last applied.
	// Finally position it
	glTranslatef(rect.x(), rect.y(), 0);

	// Secondly scale it
	glScalef(rect.width(), rect.height(), 1);

	// Firstly invert it. We assume it's about the origin and that the
	// the glRender is drawn from (0,0) to (1,1).

	if(invertX || invertY || rotate != 0){
		glTranslatef(0.5,0.5,0);
		if(invertX){
			glScalef(-1, 1, 1);
		}
		if(invertY){
			glScalef(1,-1,1);
		}
		if(rotate != 0){
			glRotatef(rotate, 0, 0, 1);
		}
		glTranslatef(-0.5,-0.5,0);
	}
}

void RobotRender::postRender() {
	// Matches the pushMatrix in preRender
	glPopMatrix();
}

bool RobotRender::keyPressEvent(QKeyEvent *e) { return false; }
bool RobotRender::mousePressEvent(QMouseEvent *) { return false; }
bool RobotRender::mouseReleaseEvent(QMouseEvent *) { return false; }
bool RobotRender::mouseMoveEvent(QMouseEvent *) { return false; }
bool RobotRender::mouseDoubleClickEvent(QMouseEvent *) { return false; }
bool RobotRender::wheelEvent(QWheelEvent *) { return false; }

} // namespace gui

} // namespace robotgui
