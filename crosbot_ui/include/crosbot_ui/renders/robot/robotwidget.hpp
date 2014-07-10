/*
 * rgglwidget.h
 *
 *  Created on: 19/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_ROBOTWIDGET_HPP_
#define CROSBOT_ROBOTWIDGET_HPP_

#include <QtOpenGL/QGLWidget>
#include <crosbot/config.hpp>
#include <crosbot_ui/opengl.hpp>

#include <string>
#include <vector>
#include <set>

namespace crosbot {

namespace gui {

#define PARAM_KEY		"key"
#define PARAM_ROTATE	"rotate"
#define PARAM_BOUNDS	"bounds"

class RobotPanel;
class RobotRender {
public:
	RobotRender(RobotPanel& panel, ConfigElementPtr);
	RobotRender(RobotPanel& panel, ConfigElementPtr, QRectF);
	virtual ~RobotRender();

	virtual bool isHidden() { return false; }
	virtual void start()=0;
	virtual void stop()=0;
	virtual void render()=0;

	void preRender();
	void postRender();
//	virtual QRectF getDefaultBounds();

	virtual bool keyPressEvent(QKeyEvent *e);
	virtual bool keyReleaseEvent(QKeyEvent *e);
	virtual bool mousePressEvent(QMouseEvent *);
	virtual bool mouseReleaseEvent(QMouseEvent *);
	virtual bool mouseMoveEvent(QMouseEvent *);
    virtual bool mouseDoubleClickEvent(QMouseEvent *);
	virtual bool wheelEvent(QWheelEvent *);

protected:
	RobotPanel& panel;
    QRectF rect;

    bool invertX, invertY;
    float rotate;
};

#define JUNK_RENDER
#ifdef JUNK_RENDER

class JointRender : public RobotRender {
public:
	Colour4f colour;

	JointRender(RobotPanel& panel, ConfigElementPtr config);
	virtual void start();
	virtual void stop();
	virtual void render();
};

#endif
class RobotWidget: public QGLWidget
{
Q_OBJECT;
public:
	class KeyListener {
	public:
		virtual ~KeyListener() {}
		virtual bool keyPressEvent(QKeyEvent *e)=0;
		virtual bool keyReleaseEvent(QKeyEvent *e)=0;
	};

	RobotWidget(RobotPanel&);
	virtual ~RobotWidget();
	
	// The screen is assumed to go from (0,0) to (AR,1)
	virtual void setAspectRatio(float aspectRatio);
	virtual float getAspectRatio();
	
	virtual void initializeGL();
	virtual bool initialized();
	
	virtual void resizeGL(int w, int h);
	virtual void paintGL();
	virtual void setVisible(bool vis);
	
	virtual void addRender(RobotRender *render);
	
	virtual void startRenders();
	virtual void stopRenders();
	
	virtual void setUpdateInterval(int updateInterval);
	
	inline int getWidth() {
		return width;
	}
	
	inline int getHeight() {
		return height;
	}

	virtual void keyPressEvent(QKeyEvent *e);
	virtual void keyReleaseEvent(QKeyEvent *e);
	virtual void mousePressEvent(QMouseEvent *);
	virtual void mouseReleaseEvent(QMouseEvent *);
	virtual void mouseMoveEvent(QMouseEvent *);
    virtual void mouseDoubleClickEvent(QMouseEvent *);
	virtual void wheelEvent(QWheelEvent *);
	virtual void focusOutEvent(QFocusEvent *);
//	virtual void clickToCameraCoord(int inX, int inY, float &outX, float &outY);
	void addInputListener(ConfigElementPtr cfg);
protected:
	RobotPanel& panel;

	float aspectRatio;
	std::vector<RobotRender *> renders;
	std::vector<KeyListener *> listeners;
	
	int width;
	int height;
	
	bool isInitialized;
	
	int updateInterval;

	friend class RobotPanel;

	int upPressed, downPressed, leftPressed, rightPressed;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_ROBOTWIDGET_HPP_ */
