/*
 * WASDMousePanel.h
 *
 *  Created on: 28/09/2010
 *      Author: amilstein
 */

#ifndef CROSBOT_PANEL_WASDMOUSE_HPP_
#define CROSBOT_PANEL_WASDMOUSE_HPP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <crosbot_ui/panels/panel.hpp>

#include <QtOpenGL/QGLWidget>
#include <crosbot_ui/opengl.hpp>

namespace crosbot {

namespace gui {

class WASDMouseView : public QGLWidget {
	Q_OBJECT
	public:
		WASDMouseView();
		~WASDMouseView();

		QSize sizeHint();
		QSize minimumSizeHint();

		virtual void initializeGL();
		virtual void resizeGL(int w, int h);
		virtual void paintGL();

		virtual void keyPressEvent(QKeyEvent *e);
		virtual void mousePressEvent(QMouseEvent *);
		virtual void mouseReleaseEvent(QMouseEvent *);
		virtual void mouseMoveEvent(QMouseEvent *);
	    virtual void mouseDoubleClickEvent(QMouseEvent *);
		virtual void wheelEvent(QWheelEvent *);
	Q_SIGNALS:
		void updateNeeded();
	public Q_SLOTS:
		void update();
	public:
		void performUpdate();


		/**
		 * Sub classes need to implement this to draw what is needed.
		 */
		virtual void paintWorldGL(GLTextQueue& text)=0;
		virtual void compileGLLists() {}

		Pose defaultView;
		Pose viewer;
		bool showGrids;
		bool rotateCenter;

	protected:
		virtual void init();

		bool draggingView;
		Pose dragOriginalView;
		tf::Transform dragOriginalViewMat;
		int dragOriginalX, dragOriginalY;

		// OpenGL stuff
		int displayList;
		int screenWidth, screenHeight;
		float aspectRatio, zoomFactor;

		GLTextQueue textQueue;

		void drawPose(Pose pose, double size);
		bool useDepthTest;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_WASDMOUSE_HPP_ */
