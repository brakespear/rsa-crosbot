/*
 * rgglwidget.cpp
 *
 *  Created on: 19/08/2009
 *      Author: rescue
 */

//#include <ros/ros.h>
#include <crosbot_ui/renders/robot/robotwidget.hpp>
#include <crosbot_ui/panels/robot.hpp>

#include <QTimer>
#include <QKeyEvent>
#include <std_msgs/String.h>
#include <sensor_msgs/JointState.h>

#include <crosbot/utils.hpp>

namespace crosbot {

namespace gui {
using namespace std;

class JointController {
protected:
	ros::Subscriber jointSub;
	ros::Publisher jointPub;
	bool connected;

	class Joint {
	public:
		std::string name;
		double pos, desiredPos;
		Joint(const std::string& name) : name(name), pos(NAN), desiredPos(NAN) {}
	};


	Joint& getJoint();
	ReadWriteLock jLock;
	std::vector< Joint > joints;
public:
	JointController() : connected(false) {}

	void shutdown() {
		jointPub.shutdown();
		jointSub.shutdown();
		connected = false;
	}

	Joint *findJoint(const std::string& joint) {
		Lock lock(jLock);
		for (size_t i = 0; i < joints.size(); ++i) {
			if (joints[i].name == joint) {
				return &joints[i];
			}
		}
		return NULL;
	}

	void callbackJoints(const sensor_msgs::JointStateConstPtr& state) {
		for (size_t i = 0; i < state->name.size(); ++i) {
			Joint *joint = findJoint(state->name[i]);

			{{
				Lock lock(jLock, true);
				if (joint == NULL) {
					joints.push_back(Joint(state->name[i]));
					joint = &joints[joints.size() - 1];
				}
				joint->pos = state->position[i];
			}}
		}
	}

	void connect() {
		if (connected)
			return;

		ros::NodeHandle nh;
		jointSub = nh.subscribe("/joint_states", 2, &JointController::callbackJoints, this);
		jointPub = nh.advertise< sensor_msgs::JointState >("/joint_control", 1);
	}

	double getPos(const std::string& joint) {
		Lock lock(jLock);
		for (size_t i = 0; i < joints.size(); ++i) {
			Joint& j = joints[i];
			if (j.name == joint)
				return j.pos;
		}

		return NAN;
	}

	void setPos(const std::string& joint, double pos) {
		Joint *j = findJoint(joint);
		if (jointPub.getNumSubscribers() == 0)
			return;
		sensor_msgs::JointState state;
		state.header.stamp = ros::Time::now();
		state.name.push_back(joint);
		state.position.push_back(pos);

		if (j != NULL) {
			j->desiredPos = pos;
		}

		jointPub.publish(state);
	}
};
JointController joints;

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
		return;
	}

	for (uint32_t i = 0; i < renders.size(); ++i) {
		if (renders[i]->keyPressEvent(e)) {
			return;
		}
	}
	for (size_t i = 0; i < listeners.size(); ++i) {
		KeyListener* l = listeners[i];
		if (l != NULL && l->keyPressEvent(e))
			return;
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

class TopicMessageKey : public RobotWidget::KeyListener {
public:
	int key;
	ros::Publisher pub;
	std::string message;
	TopicMessageKey(ConfigElementPtr cfg) : key(-1) {
		string str = cfg->getParam("key", "");
		if (str != "") {
			key = Panel::getKeyForChar(str[0]);
		}

		str = cfg->getParam("topic", "");
		if (str != "") {
			ros::NodeHandle nh;
			pub = nh.advertise< std_msgs::String >(str, 1, false);
		}

		message = cfg->getParam("message", message);
		message = cfg->getParam("string", message);
		message = cfg->getParam("msg", message);
	}

	bool keyPressEvent(QKeyEvent *e) {
		if (key == e->key()) {
			if (pub.getNumSubscribers() > 0) {
				std_msgs::String msg;
				msg.data = message;
				pub.publish(msg);
			}
			return true;
		}
		return false;
	}

	bool keyReleaseEvent(QKeyEvent *e) { return false; }
};

class JointKey : public RobotWidget::KeyListener {
public:
	std::string joint;
	int up, down;
	double step;

	JointKey(ConfigElementPtr cfg) : up(-1), down(-1), step(0.05) {
		joint = cfg->getParam("joint");
		joint = cfg->getParam("name", joint);

		string str = cfg->getParam("up", "");
		if (str != "") {
			up = Panel::getKeyForChar(str[0]);
		}
		str = cfg->getParam("down", "");
		if (str != "") {
			down = Panel::getKeyForChar(str[0]);
		}

		step = cfg->getParamAsDouble("step", step);
	}

	bool keyPressEvent(QKeyEvent *e) {
		if (up == e->key()) {
			joints.setPos(joint, joints.getPos(joint) + step);
			return true;
		} else if (down == e->key()) {
			joints.setPos(joint, joints.getPos(joint) - step);
			return true;
		}
		return false;
	}

	bool keyReleaseEvent(QKeyEvent *e) { return false; }
};

void RobotWidget::addInputListener(ConfigElementPtr cfg) {
	if (strcasecmp(cfg->name.c_str(), ELEMENT_KEY) == 0) {
		listeners.push_back(new TopicMessageKey(cfg));
	} else if (strcasecmp(cfg->name.c_str(), ELEMENT_JOINT) == 0) {
		joints.connect();
		listeners.push_back(new JointKey(cfg));
	}
}

} // namespace gui

} // namespace robotgui
