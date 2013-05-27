/*
 * WASDMousePanel.cpp
 *
 *  Created on: 28/09/2010
 *      Author: amilstein
 */
#include <crosbot_ui/panels/wasdmouse.hpp>
#include <crosbot/utils.hpp>

#include <QtGui/QMouseEvent>

namespace crosbot {

namespace gui {
using namespace std;

#define RESET_ZOFFSET		-10
#define GRID_SIZE			1
#define	GRID_EXTENT			20

WASDMouseView::WASDMouseView() {
	init();
}

void WASDMouseView::init() {
	zoomFactor = 10;

	showGrids = true;

	setFocusPolicy(Qt::StrongFocus);
	installEventFilter(this);


	// Can't use actual values as they're at a badly handled singularity.
	defaultView = Pose(Point(0, 0, 10), Quaternion(DEG2RAD(89.99999), DEG2RAD(89.99999), 0));

	viewer = defaultView;
	draggingView = false;

	rotateCenter = false;
	useDepthTest = true;

	connect(this, SIGNAL(updateNeeded()),this,SLOT(update()));
}

WASDMouseView::~WASDMouseView() {
	disconnect(this, SIGNAL(updateNeeded()),this,SLOT(update()));
}

void WASDMouseView::initializeGL() {
	glClearColor(0.70f, 0.7f, 0.7f, 1.0f);
	displayList = glGenLists(1);
}

void WASDMouseView::resizeGL(int w, int h) {
	screenWidth = w;
	screenHeight = h;
	aspectRatio = ((float) w)/((float) h);
	updateGL();
}

void WASDMouseView::paintGL() {
//	double x;
	glPushMatrix();
		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();

		gluPerspective(60, aspectRatio, 0.01,1000);

		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glTranslatef(0,0,0);
		glViewport(0,0,screenWidth,screenHeight);

		glClearColor(0.7,0.7,0.7,0.0);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		glEnable(GL_BLEND);
//		glDisable(GL_DEPTH_TEST);

		if (useDepthTest) {
			glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);
		} else {
			glDisable(GL_BLEND);
			glEnable(GL_DEPTH_TEST);
			glDepthFunc(GL_LEQUAL);
		}
		//
		glEnable(GL_POINT_SMOOTH);
		glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
		glEnable(GL_LINE_SMOOTH);
		glMatrixMode(GL_MODELVIEW);

		glColor4f(1,0,0,1);

		glScalef(1/zoomFactor, 1/zoomFactor, 1/zoomFactor);


		glPushMatrix();
			if (rotateCenter) {
				glTranslatef(-viewer.position.x, -viewer.position.y, -viewer.position.z);
			}
			double vroll, vpitch, vyaw;
			viewer.orientation.getYPR(vyaw, vpitch, vroll);
			glRotatef(-90, 1, 0, 0);
			glRotatef(90, 0, 0, 1);
			glRotatef(-RAD2DEG(vroll), 1, 0, 0);
			glRotatef(-RAD2DEG(vpitch), 0, 1, 0);
			glRotatef(-RAD2DEG(vyaw), 0, 0, 1);
			if (!rotateCenter) {
				glTranslatef(-viewer.position.x, -viewer.position.y, -viewer.position.z);
			}

			if(showGrids) {
				float gridScaledExtent = GRID_EXTENT*GRID_SIZE;
				float gridHeight = -0.001;
				glBegin(GL_LINES);

				// Draw axes.
				glColor4f(1,0,0,0.5);
				glVertex3f(gridScaledExtent, 0, gridHeight);
				glVertex3f(0, 0, gridHeight);

				glColor4f(0,1,0,0.5);
				glVertex3f(0, gridScaledExtent, gridHeight);
				glVertex3f(0, 0, gridHeight);

				glColor4f(0,0,0,0.5);
				glVertex3f(0, 0, gridHeight);
				glVertex3f(-gridScaledExtent, 0, gridHeight);
				glVertex3f(0, 0, gridHeight);
				glVertex3f(0, -gridScaledExtent, gridHeight);

				for (int i = 1; i < GRID_EXTENT;i++) {
					float scaledI = i * GRID_SIZE;
						if (i % 5 == 0) {
							glColor4f(0,0,0,0.5);
						} else {
							glColor4f(0.5,0.5,0.5,0.5);
						}

						glVertex3f(-gridScaledExtent, scaledI, gridHeight);
						glVertex3f( gridScaledExtent, scaledI, gridHeight);
						glVertex3f(scaledI, -gridScaledExtent, gridHeight);
						glVertex3f(scaledI,  gridScaledExtent, gridHeight);

						glVertex3f(-gridScaledExtent, -scaledI, gridHeight);
						glVertex3f( gridScaledExtent, -scaledI, gridHeight);
						glVertex3f(-scaledI, -gridScaledExtent, gridHeight);
						glVertex3f(-scaledI,  gridScaledExtent, gridHeight);
				}
				glEnd();
			}

			glColor4f(0,0,0,1.0);

			glPushMatrix();
				glCallList(displayList);
			glPopMatrix();

			if (textQueue.size() > 0) {
				btTransform invView, labelMat;
//				Matrix invView(4,4), labelMat(4,4);
//				getMatFromTrans(viewer, invView);
//				transInverse(invView);
				invView = viewer.getTransform().inverse();

				for (unsigned int i = 0; i < textQueue.size(); i++) {
					GLLabel label = textQueue[i];
					Pose labelPos(Point(label.x, label.y, label.z),
							Quaternion(0 , 0, 0)), relLabel;
//					getMatFromTrans(labelPos, labelMat);
					labelMat = labelPos.getTransform();
					labelMat = invView*labelMat;
//					getTransFromMat(relLabel, labelMat);
					relLabel = labelMat;

					if (relLabel.position.x >= 0) {
						glColor4f(label.colour.r, label.colour.g, label.colour.b, label.colour.a);
						renderText(label.x, label.y, label.z, QString(label.label.c_str()));
					}
				}
			}
		glPopMatrix();

		glColor4f(0,0,0,1.0);

		glDisable(GL_BLEND);
		glEnable(GL_DEPTH_TEST);
		glDisable(GL_POINT_SMOOTH);
		glDisable(GL_LINE_SMOOTH);
		glDisable(GL_POLYGON_SMOOTH);
	glPopMatrix();
}

void WASDMouseView::performUpdate() {
	Q_EMIT updateNeeded();
}

void WASDMouseView::update() {
	int foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("WASDMouseView: OpenGL error number BEFORE update %d\n", foo);
	}

	// This is supposed to fix the opengl text problem with display lists, but doesn't
	textQueue.clear();

	makeCurrent();
		compileGLLists();
		glNewList(displayList, GL_COMPILE);
			glPushMatrix();
				paintWorldGL(textQueue);
			glPopMatrix();
		glEndList();
	doneCurrent();

	foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("WASDMouseView: OpenGL error number AFTER update %d\n", foo);
	}

	updateGL();
}

void WASDMouseView::mouseDoubleClickEvent(QMouseEvent *e) {
	double yaw, pitch, roll;
	viewer.getYPR(yaw, pitch, roll);
	if (e->buttons() & Qt::RightButton) {
		viewer.setYPR(yaw, 0, 0);

		Q_EMIT updateNeeded();
	}
	if (e->buttons() & Qt::MiddleButton) {
		viewer.setYPR(yaw, pitch, 0);

		Q_EMIT updateNeeded();
	}
}

void WASDMouseView::mousePressEvent(QMouseEvent *e) {
	if (draggingView)
		return;

	dragOriginalView = viewer;
	dragOriginalX = e->x(); dragOriginalY = e->y();
	dragOriginalViewMat = dragOriginalView.getTransform();
//	getMatFromTrans(dragOriginalView, dragOriginalViewMat);
	draggingView = true;

}
void WASDMouseView::mouseReleaseEvent(QMouseEvent *e) {
	if (e->buttons() == 0) {
		draggingView = false;
	}
}

void WASDMouseView::mouseMoveEvent(QMouseEvent *e) {
	if (e->buttons() == 0) {
		draggingView = false;
		return;
	}

	Pose motion;
	float anglePerPix = (M_PI / 3) / screenHeight;

	if (((e->buttons() & Qt::LeftButton) != 0 && (e->buttons() & Qt::RightButton) != 0) ||
			(e->buttons() & Qt::MidButton) != 0) {
		float centreX = screenWidth / 2.0, centreY = screenHeight / 2.0;
		double originalAngle = atan2(centreY - dragOriginalY, dragOriginalX - centreX),
				currentAngle = atan2(centreY - e->y(), e->x() - centreX);
//		motion.o.roll = currentAngle - originalAngle;
		motion.setYPR(0,0, currentAngle - originalAngle);
	} else if ((e->buttons() & Qt::RightButton) != 0) {
		motion.position.z = dragOriginalView.position.z * sin((e->y() - dragOriginalY) * anglePerPix);
		motion.position.y = dragOriginalView.position.z * sin((e->x() - dragOriginalX) * anglePerPix);
	} else {
//		motion.o.pitch = (e->y() - dragOriginalY) * anglePerPix;
//		motion.o.yaw = (e->x() - dragOriginalX) * anglePerPix;
		double yaw = (e->x() - dragOriginalX) * anglePerPix;
		double pitch = (dragOriginalY - e->y()) * anglePerPix;
//		yaw = 0;

		motion.setYPR(yaw, pitch, 0);
	}

//	Matrix motionMat(4,4);
//	getMatFromTrans(motion, motionMat);
	btTransform motionMat = motion.getTransform();

	motionMat = dragOriginalViewMat*motionMat;

//	getTransFromMat(viewer, motionMat);
	viewer = motionMat;

	Q_EMIT updateNeeded();
}

void WASDMouseView::wheelEvent(QWheelEvent *e) {
	draggingView = false;
	Pose motion;

	float step = 0.05;
	if (e->modifiers() & Qt::ShiftModifier) {
		if (e->modifiers() & Qt::ControlModifier) {
			step = 2;
		} else {
			step = 0.2;
		}
	}

	motion.position.x = (step * zoomFactor * e->delta()) / 120;

//	Matrix viewerMat(4,4), motionMat(4,4);
//	getMatFromTrans(viewer, viewerMat);
//	getMatFromTrans(motion, motionMat);
	btTransform viewerMat = viewer.getTransform(), motionMat = motion.getTransform();

	viewerMat = viewerMat*motionMat;
//	getTransFromMat(viewer, viewerMat);
	viewer = viewerMat;

	Q_EMIT updateNeeded();
}

void WASDMouseView::keyPressEvent(QKeyEvent *e) {
	float step = 0;
	bool needUpdate = true;
	Pose3D motion;

	switch (e->key()) {
	case Qt::Key_W: case Qt::Key_A: case Qt::Key_S: case Qt::Key_D:
	case Qt::Key_Q: case Qt::Key_E:
		step = 0.05;
		if ((e->modifiers() & Qt::ShiftModifier) || (e->modifiers() & Qt::ControlModifier)) {
			if ((e->modifiers() & Qt::ShiftModifier) && (e->modifiers() & Qt::ControlModifier)) {
				step = 2;
			} else {
				step = 0.2;
			}
		}

		switch(e->key()) {
		case Qt::Key_W:
			motion.position.x = step*zoomFactor; break;
		case Qt::Key_S:
			motion.position.x = -step*zoomFactor; break;
		case Qt::Key_A:
			motion.position.y = step*zoomFactor; break;
		case Qt::Key_D:
			motion.position.y = -step*zoomFactor; break;
		case Qt::Key_Q:
			motion.position.z = step*zoomFactor; break;
		case Qt::Key_E:
			motion.position.z = -step*zoomFactor; break;
		}

		{
//			Matrix viewerMat(4,4), motionMat(4,4);
//			getMatFromTrans(viewer, viewerMat);
//			getMatFromTrans(motion, motionMat);
			btTransform viewerMat = viewer.getTransform(),
					motionMat = motion.getTransform();

			viewerMat = viewerMat*motionMat;
//			getTransFromMat(viewer, viewerMat);
			viewer = viewerMat;
		}
		break;
	case Qt::Key_BracketLeft:
		zoomFactor *= 1.1; break;
	case Qt::Key_BracketRight:
		zoomFactor /= 1.1; break;
	case Qt::Key_Left: case Qt::Key_Right: case Qt::Key_Up: case Qt::Key_Down:
		step = DEG2RAD(2);
		if(e->modifiers() & Qt::ShiftModifier) {
			if(e->modifiers() & Qt::ControlModifier){
				step = DEG2RAD(30);
			} else {
				step = DEG2RAD(5);
			}
		}

		if (e->key() == Qt::Key_Left || e->key() == Qt::Key_Right) {
			double vyaw, vpitch, vroll;
			viewer.getYPR(vyaw, vpitch, vroll);
			switch(e->key()) {
			case Qt::Key_Left:
				vyaw += step; break;
			case Qt::Key_Right:
				vyaw -= step; break;
			}
			viewer.setYPR(vyaw, vpitch, vroll);
		} else {
			double myaw, mpitch, mroll;
			motion.getYPR(myaw, mpitch, mroll);
			switch(e->key()) {
			case Qt::Key_Up:
				mpitch += step; break;
			case Qt::Key_Down:
				mpitch -= step; break;
			}
			motion.setYPR(myaw, mpitch, mroll);
			{
//				Matrix viewerMat(4,4), motionMat(4,4);
//				getMatFromTrans(viewer, viewerMat);
//				getMatFromTrans(motion, motionMat);
				btTransform viewerMat = viewer.getTransform(),
						motionMat = motion.getTransform();

				viewerMat = viewerMat*motionMat;
//				getTransFromMat(viewer, viewerMat);
				viewer = viewerMat;
			}
		}
		break;
	case Qt::Key_R:
		zoomFactor=10;
		viewer = defaultView;
		break;
	case Qt::Key_P:
		rotateCenter = !rotateCenter;
		break;
	default:
		needUpdate = false; break;
	}

	if (needUpdate) {
		Q_EMIT updateNeeded();
	}
}

void WASDMouseView::drawPose(Pose pose, double size) {
	glTranslatef(pose.position.x, pose.position.y, pose.position.z);
	double yaw, pitch, roll;
	pose.getYPR(yaw, pitch, roll);
	// apply rotations
	glRotatef(RAD2DEG(yaw), 0, 0, 1);
	glRotatef(RAD2DEG(pitch), 0, 1, 0);
	glRotatef(RAD2DEG(roll), 1, 0, 0);

	glBegin(GL_LINES);
		glColor4f(1, 0, 0, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(size, 0, 0);
		glColor4f(0, 1, 0, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(0, size, 0);
		glColor4f(0, 0, 1, 0.5);
		glVertex3f(0, 0, 0);
		glVertex3f(0, 0, size);
	glEnd();
}

} // namespace gui

} // namespace casros
