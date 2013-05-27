/*
 * mappanel.h
 *
 *  Created on: 15/09/2010
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_MAP_HPP_
#define CROSBOT_PANEL_MAP_HPP_

#include <crosbot_map/map.hpp>
#include <crosbot_ui/renders/map/map.hpp>
#include <crosbot_ui/panels/panel.hpp>

#include <crosbot_ui/panels/wasdmouse.hpp>

#include <QtOpenGL/QGLWidget>
#include <QGroupBox>
#include <QCheckBox>
#include <QPushButton>
#include <QTreeWidget>
#include <QSpinBox>
#include <QComboBox>
#include <QFileDialog>

namespace crosbot {

namespace gui {

#define PANEL_MAP	"map"

class MapView : public WASDMouseView,
		virtual public MapListener {
Q_OBJECT
public:
	MapView();
	~MapView();
	void setMap(MapPtr map);

	void initializeGL();
	void paintGL();

	void motionTracked(MapPtr map) {}
	void mapUpdated(MapPtr);
	void tagAdded(MapPtr, TagPtr);
	void tagChanged(MapPtr, TagPtr);

	void keyPressEvent(QKeyEvent *e);

	void setCursorEnabled(bool enable);

	void paintWorldGL(GLTextQueue& textQueue);
	void compileGLLists();

public Q_SLOTS:
	void setShowGrids(int show);
	void setShowRobots(int show);
	void setShowTags(int show);
	void setShowPointClouds(int show);
	void setShowGeoTiff(int show);

protected:
	MapPtr map;
	MapRender* render;

	ReadWriteLock rwLock, tagLock;

	bool showRobots,
		showTags;
	int	showPointclouds;
	int showGeoTiff;

	// OpenGL stuff
	GLTextQueue mapText;
	int mapGLList, pointcloudGLList, geotiffGLList, robotGLList;
	bool mapDirty, pointcloudDirty, tiffDirty, robotsDirty;

	bool useCursor;
	Pose2D cursor;

	Map::TagListPtr tags;

	friend class MapPanelControl;
	friend class MapPanel;
};

class MapPanelControl: public QWidget {
Q_OBJECT
public:
	MapPanelControl(MapView *mapView);
	~MapPanelControl();
	void setMap(MapPtr map);
	void clearMaps();
public Q_SLOTS:
	void handleSelectionChanged();
	void activateMap();
	void newMap();
//	void fuseMap();

	void load();
	void saveAs();
	void exportGeoTiff();
	void stairs();
	// reset the robot's position
	// usually used on loading a map
	void resetPose();

	void mapSelectionChanged(int index);
	void mapIndexChanged(int i);

protected:
	MapView *mapView;
	MapPtr activeMap;

	QGroupBox mapsGB;
	QTreeWidget mapTree;
	QTreeWidgetItem activeMapTWI;
	QTreeWidgetItem allMapsTWI;
	QPushButton activateMapBtn;
	QPushButton newMapBtn;

	QSpinBox mapIndex;
	QComboBox mapSelect;

	QGroupBox showHideGB;
	QCheckBox showGrids;
	QCheckBox showRobots;
	QCheckBox showTags;
	QCheckBox showPointclouds;
	QCheckBox showGeoTiff;

	QGroupBox actionGB;
	QPushButton loadBtn;
	QPushButton saveAsBtn;
	QPushButton exportGeoTIffBtn;
	QPushButton pauseBtn;
	QPushButton resetPoseBtn;

	QFileDialog fileDialog;

	void checkMapList();
	void activateMap(std::string name);
	friend class MapPanel;
};

/**
 * \image html panel-map.png
 */
class MapPanel : public Panel {
Q_OBJECT
public:
	MapPanel(ConfigElementPtr config);
	~MapPanel();

	void start();
	void stop();

	QWidget *getWidget();
public Q_SLOTS:
	void checkMaps();
protected:
	QFrame widget;
	MapView mapView;
	MapPanelControl controlPanel;

	std::string initialMap;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_MAP_HPP_ */
