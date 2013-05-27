/*
 * mappanel.cpp
 *
 *  Created on: 16/09/2010
 *      Author: rescue
 */

#include <crosbot_ui/panels/map.hpp>
#include <crosbot_ui/opengl.hpp>
#include <crosbot_ui/crosbotgui.hpp>

#include <typeinfo>

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QMessageBox>
#include <QKeyEvent>
#include <QTimer>
#include <QProcess>
#include <QCommonStyle>

namespace crosbot {

namespace gui {

MapPanel::MapPanel(ConfigElementPtr config) :
				Panel(config), controlPanel(&mapView) {
    QVBoxLayout *vlayout = new QVBoxLayout();
//    vlayout->addWidget(&mapView, Qt::AlignCenter);
//    vlayout->addWidget(&controlPanel, Qt::AlignCenter);
    vlayout->addWidget(&mapView, 3, 0);
    vlayout->addWidget(&controlPanel, 1, 0);
    widget.setLayout(vlayout);

    initialMap = config->getParam(PARAM_NAME, initialMap);
    initialMap = config->getParam(ELEMENT_MAP, initialMap);
}

MapPanel::~MapPanel() {
	mapView.setMap(NULL);
}

void MapPanel::start() {
	checkMaps();

	QTimer* timer = new QTimer(this);
	connect(timer, SIGNAL(timeout()), this, SLOT(checkMaps()));
	timer->start(250);

	if (initialMap != "") {
		controlPanel.activateMap(initialMap);
	}
}

void MapPanel::stop() {
	mapView.setMap(NULL);
}

QWidget *MapPanel::getWidget() {
	return &widget;
}

void MapPanel::checkMaps() {
	controlPanel.checkMapList();
	if (mapView.map == NULL || mapView.map->isPaused()) {
		Q_EMIT this->mapView.updateNeeded();
	}
}

MapPanelControl::MapPanelControl(MapView *mapView) :
		mapsGB("Open Maps"),
		mapTree(this),
		activeMapTWI(&mapTree),
		allMapsTWI(&mapTree),
		activateMapBtn("Make Active"),
		newMapBtn("New Map"),
		showHideGB("Show/Hide"),
//		showGrids("Grids"),
//		showRobots("Robots"),
//		showSnaps("Snaps"),
//		showPointclouds("Pointclouds"),
//		showGeoTiff("Show GeoTiff"),
		actionGB("Actions"),
//		loadBtn("Load"),
//		saveAsBtn("Save As"),
//		exportGeoTIffBtn("Export GeoTiff"),
//		pauseBtn("Pause/Resume"),
//		resetPoseBtn("Reset Position"),
		fileDialog(this)
{
	this->mapView = mapView;

	QHBoxLayout *hlayout = new QHBoxLayout();
	hlayout->addWidget(&mapsGB);
	hlayout->addWidget(&showHideGB);
	hlayout->addWidget(&actionGB);
	setLayout(hlayout);

	mapTree.setHeaderLabels(QStringList("Maps"));
	mapTree.setHeaderHidden(true);
	mapTree.setColumnCount(1);
	mapTree.setSelectionMode(QAbstractItemView::SingleSelection);

	activeMapTWI.setText(0, "Active");
	activeMapTWI.setExpanded(true);
	allMapsTWI.setText(0, "All Maps");
	allMapsTWI.setExpanded(true);

	activateMapBtn.setEnabled(false);
	newMapBtn.setEnabled(false);

	QVBoxLayout *vlayout = new QVBoxLayout();
	vlayout->addWidget(&mapTree);
	vlayout->addSpacing(10);
	hlayout = new QHBoxLayout();

	vlayout->addLayout(hlayout);

	mapsGB.setLayout(vlayout);
	vlayout = new QVBoxLayout();

	vlayout->addWidget(&activateMapBtn);
	vlayout->addWidget(&newMapBtn);

	hlayout->addLayout(vlayout);

	vlayout = new QVBoxLayout();

	mapSelect.addItem("Max");
	mapSelect.addItem("Motion");
	mapSelect.addItem("Cursor");
	mapSelect.addItem("Index");
	mapSelect.setCurrentIndex(0);
	mapSelect.setEditable(false);
	mapIndex.setMinimum(0);
//	mapIndex.setM
	mapIndex.setEnabled(false);
	mapIndex.setMaximum(50000);
	connect(&mapSelect, SIGNAL(currentIndexChanged(int)),this,SLOT(mapSelectionChanged(int)));
	connect(&mapIndex, SIGNAL(valueChanged(int)),this,SLOT(mapIndexChanged(int)));

	vlayout->addWidget(&mapSelect);
	vlayout->addWidget(&mapIndex);

	hlayout->addLayout(vlayout);

	connect(&mapTree, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelectionChanged()));
	connect(&activateMapBtn, SIGNAL(clicked()),this,SLOT(activateMap()));
	connect(&newMapBtn, SIGNAL(clicked()),this,SLOT(newMap()));

	showGeoTiff.setTristate(true);
	showPointclouds.setTristate(true);

	vlayout = new QVBoxLayout();
	vlayout->addWidget(&showGrids);
	vlayout->addWidget(&showRobots);
	vlayout->addWidget(&showTags);
	vlayout->addWidget(&showPointclouds);
	vlayout->addWidget(&showGeoTiff);
	showHideGB.setLayout(vlayout);

	connect(&showGrids, SIGNAL(stateChanged(int)),mapView,SLOT(setShowGrids( int )));
	connect(&showRobots, SIGNAL(stateChanged(int)),mapView,SLOT(setShowRobots( int )));
	connect(&showTags, SIGNAL(stateChanged(int)),mapView,SLOT(setShowTags( int )));
	connect(&showPointclouds, SIGNAL(stateChanged(int)),mapView,SLOT(setShowPointClouds( int )));
	connect(&showGeoTiff, SIGNAL(stateChanged(int)),mapView,SLOT(setShowGeoTiff( int )));

	showGrids.setIcon(QIcon(":/grid-icon.png"));
	showGrids.setToolTip("Grids");
	showRobots.setIcon(QIcon(":/android-icon.png"));
	showRobots.setToolTip("Robots");
	showTags.setIcon(QIcon(":/snaps-icon.png"));
	showTags.setToolTip("Snaps");
	showPointclouds.setIcon(QIcon(":/pointcloud2-icon.png"));
	showPointclouds.setToolTip("Pointclouds");
	showGeoTiff.setIcon(QIcon(":/geotiff-icon.png"));
	showGeoTiff.setToolTip("GeoTiff");

	showGrids.setChecked(true);
	showRobots.setChecked(true);
	showTags.setChecked(true);
	showPointclouds.setChecked(false);
	showGeoTiff.setChecked(false);

	vlayout = new QVBoxLayout();
	vlayout->addWidget(&loadBtn);
	vlayout->addWidget(&saveAsBtn);
	vlayout->addWidget(&exportGeoTIffBtn);
	vlayout->addWidget(&pauseBtn);
	vlayout->addWidget(&resetPoseBtn);
	actionGB.setLayout(vlayout);

	QCommonStyle qcs;
	QSize icosize(22,22);
	loadBtn.setIcon(qcs.standardIcon(QStyle::SP_DialogOpenButton));
	loadBtn.setIconSize(icosize);
	saveAsBtn.setIcon(qcs.standardIcon(QStyle::SP_DialogSaveButton));
	saveAsBtn.setIconSize(icosize);
	exportGeoTIffBtn.setIcon(QIcon(":/geotiff-icon.png"));
	exportGeoTIffBtn.setIconSize(icosize);
	pauseBtn.setIcon(qcs.standardIcon(QStyle::SP_MediaPause));
	pauseBtn.setIconSize(icosize);
	resetPoseBtn.setIcon(QIcon(":/resetpos-icon.png"));
	resetPoseBtn.setIconSize(icosize);

	loadBtn.setToolTip("Load");
	saveAsBtn.setToolTip("Save As");
	exportGeoTIffBtn.setToolTip("Export GeoTiff");
	pauseBtn.setToolTip("Pause/Resume");
	resetPoseBtn.setToolTip("Reset Position");

	connect(&loadBtn, SIGNAL(clicked()),this,SLOT(load()));
	connect(&saveAsBtn, SIGNAL(clicked()),this,SLOT(saveAs()));
	connect(&exportGeoTIffBtn, SIGNAL(clicked()),this,SLOT(exportGeoTiff()));
	connect(&pauseBtn, SIGNAL(clicked()),this,SLOT(stairs()));
	connect(&resetPoseBtn, SIGNAL(clicked()),this,SLOT(resetPose()));

	fileDialog.setViewMode(QFileDialog::Detail);
}

MapPanelControl::~MapPanelControl() {
	disconnect(&mapSelect, SIGNAL(currentIndexChanged(int)),this,SLOT(mapSelectionChanged(int)));
	disconnect(&mapIndex, SIGNAL(valueChanged(int)),this,SLOT(mapIndexChanged(int)));
	disconnect(&mapTree, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelectionChanged()));
	disconnect(&activateMapBtn, SIGNAL(clicked()),this,SLOT(activateMap()));
	disconnect(&newMapBtn, SIGNAL(clicked()),this,SLOT(newMap()));
	disconnect(&showGrids, SIGNAL(stateChanged(int)),mapView,SLOT(setShowGrids( int )));
	disconnect(&showRobots, SIGNAL(stateChanged(int)),mapView,SLOT(setShowRobots( int )));
	disconnect(&showTags, SIGNAL(stateChanged(int)),mapView,SLOT(setShowTags( int )));
	disconnect(&showPointclouds, SIGNAL(stateChanged(int)),mapView,SLOT(setShowPointClouds( int )));
	disconnect(&showGeoTiff, SIGNAL(stateChanged(int)),mapView,SLOT(setShowGeoTiff( int )));
	disconnect(&loadBtn, SIGNAL(clicked()),this,SLOT(load()));
	disconnect(&saveAsBtn, SIGNAL(clicked()),this,SLOT(saveAs()));
	disconnect(&exportGeoTIffBtn, SIGNAL(clicked()),this,SLOT(exportGeoTiff()));
	disconnect(&pauseBtn, SIGNAL(clicked()),this,SLOT(stairs()));
	disconnect(&resetPoseBtn, SIGNAL(clicked()),this,SLOT(resetPose()));
}

class MapTreeItem : public QTreeWidgetItem {
public:
	MapPtr map;
	MapTreeItem(MapPtr map, std::string name) : QTreeWidgetItem(QStringList(QString(name.c_str()))) {
		this->map = map;
	}
};

void MapPanelControl::mapSelectionChanged(int idx) {
	mapView->setCursorEnabled(false);
	if (mapView->render == NULL) {
		if (idx == 2) {
			mapView->setCursorEnabled(true);
		}
		return;
	}

	mapIndex.setEnabled(false);
	if (idx == 0) {
		mapView->render->setSelectedMap(-1);
	} else if (idx == 1) {
		mapView->render->setSelectedMap(-2);
	} else if (idx == 2) {
		mapView->setCursorEnabled(true);
		mapView->render->setSelectedMap(Point(mapView->cursor.position.x, mapView->cursor.position.x, 0));
	} else {
		mapView->render->setSelectedMap(mapIndex.value());
		mapIndex.setEnabled(true);
	}

	mapView->mapDirty = true;
	mapView->robotsDirty = true;
	mapView->pointcloudDirty = true;
	mapView->tiffDirty = true;

	Q_EMIT mapView->updateNeeded();
}

void MapPanelControl::mapIndexChanged(int idx) {
	if (mapView->render == NULL)
		return;
	mapView->render->setSelectedMap(mapIndex.value());
	mapIndex.setEnabled(true);

	mapView->mapDirty = true;
	mapView->robotsDirty = true;
	mapView->pointcloudDirty = true;
	mapView->tiffDirty = true;

	Q_EMIT mapView->updateNeeded();
}

void MapPanelControl::setMap(MapPtr map) {
	if (activeMap != NULL) {
		if (activeMapTWI.childCount() > 0) {
			QTreeWidgetItem *twi = activeMapTWI.child(0);
			activeMapTWI.removeChild(twi);
			allMapsTWI.addChild(twi);
		}
	}

	this->activeMap = map;
	if (map != NULL) {
		activeMapTWI.addChild(new MapTreeItem(map, Gui::maps.getName(map)));
	}
}

void MapPanelControl::clearMaps() {
	mapTree.clearSelection();
	setMap(NULL);

	while (activeMapTWI.childCount() > 0) {
		QTreeWidgetItem *twi = activeMapTWI.child(0);
		activeMapTWI.removeChild(twi);
		delete(twi);
	}

	while (allMapsTWI.childCount() > 0) {
		QTreeWidgetItem *twi = allMapsTWI.child(0);
		allMapsTWI.removeChild(twi);
		delete(twi);
	}
	mapTree.clearSelection();
}

void MapPanelControl::handleSelectionChanged() {
	QList<QTreeWidgetItem *> selection = mapTree.selectedItems();
	QTreeWidgetItem *item = NULL;
	if (selection.size() > 0)
		item = selection[0];
	if (item == NULL || typeid(*item) != typeid(MapTreeItem)) {
		activateMapBtn.setEnabled(false);
		return;
	}
	MapPtr map = ((MapTreeItem *)item)->map;
	if (map == activeMap) {
		activateMapBtn.setEnabled(false);
		return;
	}
	activateMapBtn.setEnabled(true);
}

void MapPanelControl::activateMap(std::string name) {
	MapPtr map = Gui::maps.getMap(name);

	if (map == NULL)
		return;

	mapTree.clearSelection();

	QTreeWidgetItem *twi = activeMapTWI.child(0);
	activeMapTWI.removeChild(twi);
	delete twi;

	activeMap = map;
	twi = new MapTreeItem(map, name);
	activeMapTWI.addChild(twi);
	twi->setSelected(true);
	mapView->setMap(map);
	newMapBtn.setEnabled(true);
}

void MapPanelControl::activateMap() {
	QList<QTreeWidgetItem *> selection = mapTree.selectedItems();
	QTreeWidgetItem *item = NULL;
	if (selection.size() > 0)
		item = selection[0];
	if (item == NULL || typeid(*item) != typeid(MapTreeItem)) {
		return;
	}
	MapPtr map = ((MapTreeItem *)item)->map;
	if (map == activeMap) {
		return;
	}

	if (activeMap != NULL) {
		if (activeMapTWI.childCount() > 0) {
			QTreeWidgetItem *twi = activeMapTWI.child(0);
			if (twi == item)
				mapTree.clearSelection();
			activeMapTWI.removeChild(twi);
			delete twi;
		}

		activeMap = NULL;
	}

	item = new MapTreeItem(map, item->text(0).toStdString());
	activeMapTWI.addChild(item);
	activeMap = map;

	if (activeMap == NULL)
		newMapBtn.setEnabled(false);
	else
		newMapBtn.setEnabled(true);

	mapView->setMap(map);

	mapTree.clearSelection();
	item->setSelected(true);

	if (!activeMap->isPaused()) {
		pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPause));
	} else {
		pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPlay));
	}

	Q_EMIT mapView->updateNeeded();
}

void MapPanelControl::newMap() {
	bool ok;
	if (activeMap == NULL) {
		QMessageBox::warning(this, "No Map", "No map has been activated in order to be cloned.");
		return;
	}

	QString mapName = QInputDialog::getText(this, "RobotGUI: New Map", "Enter a name for the new map:", QLineEdit::Normal,
			QString::null, &ok);
	if (!ok) {
		return;
	}
	MapPtr map = activeMap->clone();
	if (map == NULL) {
		QMessageBox::warning(this, "No Clone", "An empty clone could not be generated.\n");
		return;
	}
//	map->setPaused(true);

	Gui::maps.addMap(mapName.toStdString(), map);
}

void MapPanelControl::load() {
	if (activeMap == NULL) {
		QMessageBox::warning(this, "No Map", "A map needs to be active before the data can be loaded.\n");
		return;
	}

	fileDialog.setFilter(QString("*.map"));
	fileDialog.setWindowTitle("Load Map");
	fileDialog.setAcceptMode(QFileDialog::AcceptOpen);
	fileDialog.setFileMode(QFileDialog::ExistingFile);
	int i = fileDialog.exec();
	if (i == 0)
		return;
	QStringList fileList = fileDialog.selectedFiles();
	if (fileList.size() == 0)
		return;
	QString file = fileList.at(0);
    if (file == QString::null)
    	return;
    if (!activeMap->load(file.toStdString())) {
    	QMessageBox::critical(this, "Load Error", "An error occurred when loading the map.\n");
		ERROR("MapPanel: Map couldn't be loaded from file %s.\n", file.toStdString().c_str());
    }

    mapView->mapDirty = true;
	mapView->robotsDirty = true;
    mapView->pointcloudDirty = true;
    mapView->tiffDirty = true;

	if (!activeMap->isPaused()) {
		pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPause));
	} else {
		pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPlay));
	}
    Q_EMIT mapView->updateNeeded();
}

void MapPanelControl::saveAs() {
	if (activeMap == NULL) {
		ERROR("MapPanel: Map isn't available for saving.\n");
		QMessageBox::warning(this, "No Map", "There isn't an active map that can be saved.\n");
		return;
	}

	fileDialog.setFilter(QString("*.map"));
	fileDialog.setWindowTitle("Save Map");
	fileDialog.setAcceptMode(QFileDialog::AcceptSave);
	fileDialog.setFileMode(QFileDialog::AnyFile);
	int i = fileDialog.exec();
	if (i == 0)
		return;
	QStringList fileList = fileDialog.selectedFiles();
	if (fileList.size() == 0)
		return;
	QString file = fileList.at(0);
    if (file == QString::null)
    	return;

//    QString file = QFileDialog::getSaveFileName(this, "Save Map", ".", "*.map");

    std::string filename = file.toStdString();
    if (filename.size() < 4 ||
    		strcasecmp(filename.substr(filename.size()-4).c_str(), ".map") != 0) {
    	filename = filename + ".map";
    }

    bool saved = false;
    if (mapView->render != NULL) {
    	saved = mapView->render->saveMap(filename);
    } else {
    	saved = activeMap->save(filename);
    }

    if (!saved) {
		ERROR("MapPanel: Map couldn't be saved to %s.\n", filename.c_str());
    	QMessageBox::critical(this, "Save Error", "An error occurred when saving the map.\n");
    }
}

void MapPanelControl::exportGeoTiff() {
	if (activeMap == NULL) {
		ERROR("MapPanel: Map isn't available for exporting.\n");
		QMessageBox::warning(this, "No Map", "There isn't an active map that can be exported.\n");
		return;
	} else if (mapView->render == NULL) {
		QMessageBox::warning(this, "No Render", "There isn't a render available for producing the GeoTIFF image.\n");
    	return;
    }

	fileDialog.setFilter(QString("*.tif"));
	fileDialog.setWindowTitle("Export GeoTIFF");
	fileDialog.setAcceptMode(QFileDialog::AcceptSave);
	fileDialog.setFileMode(QFileDialog::AnyFile);
	int i = fileDialog.exec();
	if (i == 0)
		return;
	QStringList fileList = fileDialog.selectedFiles();
	if (fileList.size() == 0)
		return;
	QString filename = fileList.at(0);
    if (filename == QString::null)
    	return;
    if (!(filename.endsWith(QString(".tif")) || filename.endsWith(".tiff"))) {
    	filename += ".tif";
    }
    std::string title = filename.toStdString();
    size_t n = title.find_last_of("/");
    if (n != title.npos) {
    	title = title.substr(n+1, title.length()-(n+1));
    }
	QString geoData; Point2D minXY;
	QImage* geotiff = mapView->render->getGeoTIFF(QString(title.c_str()), geoData, minXY);
	if (geotiff == NULL) {
		LOG("MapPanel: Problem Rendering GeoTIFF. (map %s)\n", Gui::maps.getName(activeMap).c_str());
		QMessageBox::critical(this, "Problem Saving GeoTIFF", "Unable to render GeoTIFF from map.");
		return;
	}

	// save file// write image to file
	std::string rawFileName = filename.toStdString() + "f";
	if (!geotiff->save(QString(rawFileName.c_str()), "TIFF")) {
		char eMsg[rawFileName.size() + 128];

		sprintf(eMsg, "Could not save raw TIFF to file %s.", rawFileName.c_str());
		LOG("MapPanel: Problem saving map to TIFF. (file %s)\n", rawFileName.c_str());
		QMessageBox::critical(this, "Problem Saving GeoTIFF", QString(eMsg));
		delete geotiff;
		return;
	}
	// GeoTiff saved so we delete it.
	delete geotiff;

	// write the world file
	std::string worldFileName = filename.toStdString();
	if (worldFileName.size() > 5 && strcasecmp(worldFileName.substr(worldFileName.size()-4).c_str(), ".tif") == 0) {
		worldFileName = worldFileName.substr(0, worldFileName.size()-4);
	} else if (worldFileName.size() > 6 && strcasecmp(worldFileName.substr(worldFileName.size()-5).c_str(), ".tiff") == 0) {
		worldFileName = worldFileName.substr(0, worldFileName.size()-5);
	}
	worldFileName += ".tfw";

	FILE *worldFile = fopen(worldFileName.c_str(), "w");
	if (worldFile == NULL) {
		LOG("MapPanel: Problem opening GeoTIFF world file. (file %s)\n", worldFileName.c_str());
		QMessageBox::critical(this, "Problem Saving GeoTIFF", "Problem opening GeoTIFF world file.");
		return;
	}

	fprintf(worldFile, "%s\n", geoData.toStdString().c_str());

	fflush(worldFile);
	fclose(worldFile);

	// Combine tif and worldfile into a geotiff file

	// need to use an external process to create geotiff from tiff and esri file
	// if the image isn't a tif nothing happens
	QProcess qproc;
	char cmdtxt[filename.size() + rawFileName.size() + worldFileName.size() + 128];
	sprintf(cmdtxt, "geotifcp -e %s %s %s", worldFileName.c_str(), rawFileName.c_str(), filename.toStdString().c_str());
	qproc.start(cmdtxt);
	// if geotifcp ever stops returning disable this
	qproc.waitForFinished();

}

void MapPanelControl::stairs() {
	if (activeMap != NULL) {
		activeMap->setPaused(!activeMap->isPaused());

		if (!activeMap->isPaused()) {
			pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPause));
		} else {
			pauseBtn.setIcon(QCommonStyle().standardIcon(QStyle::SP_MediaPlay));
		}

		Q_EMIT mapView->updateNeeded();
	}
}

void MapPanelControl::resetPose() {
	if (activeMap == NULL) {
		return;
	}

	Pose resetPose;
	if (mapView->useCursor) {
		resetPose = Pose3D(mapView->cursor.position.x, mapView->cursor.position.y, 0, 0, 0, mapView->cursor.orientation);
	}
	activeMap->resetPose(resetPose);
}

void MapPanelControl::checkMapList() {
	std::vector<MapTreeItem*> currentMaps;

	for (int j = 0; j < allMapsTWI.childCount(); j++) {
		MapTreeItem* mti = dynamic_cast<MapTreeItem*>(allMapsTWI.child(j));
		if (mti != NULL) {
			currentMaps.push_back(mti);
		}
	}

	for (unsigned int i = 0; i < Gui::maps.getCount(); i++) {
		MapPtr m = Gui::maps.getMap(i);
		if (m == NULL || m == activeMap)
			continue;
		bool found = false;
		for (unsigned int j = 0; j < currentMaps.size(); j++) {
			if (currentMaps[j] != NULL && m == currentMaps[j]->map) {
				found = true;
				currentMaps[j] = NULL;
			}
		}

		if (!found) {
			allMapsTWI.addChild(new MapTreeItem(m, Gui::maps.getName(m)));
		}
	}
}

MapView::MapView() :
		WASDMouseView(),
		showRobots(true),
		showTags(true),
		showPointclouds(0),
		showGeoTiff(0),
		useCursor(false)
{
	render = NULL;
	mapDirty = pointcloudDirty = tiffDirty = robotsDirty = true;
}

MapView::~MapView() {
	if (render != NULL) {
		delete render;
	}
}

void MapView::setMap(MapPtr map) {
	Lock lock(rwLock, true);
	if (this->map == map) {
		return;
	} else if (this->map != NULL) {
		this->map->removeListener(this);
	}

	this->map = map;
	if (render != NULL) {
		delete render;
	}
	if (map == NULL) {
		render = NULL;
		tags = NULL;
	} else {{
		map->addListener(this);
		render = MapRender::getRender(map);
		Map::TagListPtr tags = map->getTags();
		Lock lock2(tagLock, true);
		this->tags = tags;
	}}

	mapDirty = pointcloudDirty = tiffDirty = robotsDirty = true;
}

void MapView::initializeGL() {
	WASDMouseView::initializeGL();

	mapGLList = glGenLists(1);
	pointcloudGLList = glGenLists(1);
	geotiffGLList = glGenLists(1);
	robotGLList = glGenLists(1);
}

void MapView::paintGL() {
	useDepthTest = showPointclouds != 0;
	WASDMouseView::paintGL();

	if (map != NULL) {
		bool paused = map->isPaused();

		if (paused) {
			Time t = Time::now();
			if ((t.nsec / 500000000) % 2 == 0) {
				glColor4f(1,0,0,1);
			} else {
				glColor4f(0,0,0,1);
			}

			QFont fnt;
			fnt.setPointSize(50);
			int x = QFontMetrics(fnt).width("PAUSED");

			x = x / screenWidth;

			renderText(-zoomFactor*aspectRatio*x - .5, 0, 0, "PAUSED", fnt);
		}
	} else {
		glColor4f(1,0,0,1);
		QFont fnt;
		fnt.setPointSize(50);
		int x = QFontMetrics(fnt).width("No Map!");

		x = x / screenWidth;

		renderText(-zoomFactor*aspectRatio*x - .5, 0, 0, "No Map!", fnt);
	}
}

void MapView::compileGLLists() {
	if (render == NULL)
		return;
    int foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE compiling MapView GLLists. (%s)\n", glutGetGLError(foo).c_str());
    }

	render->compileGLLists();

	if (render != NULL && mapDirty && (showGeoTiff < 2 && showPointclouds < 2)) {
		Lock lock(rwLock);
		mapText.clear();
//		glPushMatrix();
			glNewList(mapGLList, GL_COMPILE);
				render->renderMap(mapText);
			glEndList();
//		glPopMatrix();
		mapDirty = false;
	}

	if (render != NULL && robotsDirty && showRobots) {
		Lock lock(rwLock);
//		glPushMatrix();
			glNewList(robotGLList, GL_COMPILE);
				render->renderRobots(textQueue);
			glEndList();
//		glPopMatrix();
		robotsDirty = false;
	}

	if (render != NULL && pointcloudDirty && showPointclouds != 0) {
		Lock lock(rwLock);
//		glPushMatrix();
			glNewList(pointcloudGLList, GL_COMPILE);
				render->renderPointClouds(textQueue);
			glEndList();
//		glPopMatrix();
		pointcloudDirty = false;
	}

	if (render != NULL && tiffDirty && showGeoTiff != 0) {
		Lock lock(rwLock);
//		glPushMatrix();
			glNewList(geotiffGLList, GL_COMPILE);
				render->renderGeoTIFF(showRobots);
			glEndList();
//		glPopMatrix();
		tiffDirty = false;
	}
    foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER compiling MapView GLLists. (%s)\n", glutGetGLError(foo).c_str());
    }
}

void MapView::paintWorldGL(GLTextQueue& text) {
//	compileGLLists();

	if (useCursor) {
		glPushMatrix();
			glTranslatef(cursor.position.x, cursor.position.y, 0);
			glRotatef(RAD2DEG(cursor.orientation),0,0,1);

			glLineWidth(2.0);
			glBegin(GL_LINES);
				glColor4f(1.0, 0, 0, 1.0);
				glVertex3f(0, 0, 0);
				glVertex3f(1.0, 0, 0);

				glColor4f(0, 1.0, 0, 1.0);
				glVertex3f(0, 0, 0);
				glVertex3f(0, 1.0, 0);

				glColor4f(1.0, 1.0, 0, 1.0);
				glVertex3f(0, 0, 0);
				glVertex3f(0, -1.0, 0);
				glVertex3f(0, 0, 0);
				glVertex3f(-1.0, 0, 0);
			glEnd();
			glLineWidth(1.0);
		glPopMatrix();
	}

	glColor4f(0,0,0,1.0);

	if (showGeoTiff != 0) {
		glPushMatrix();
			glCallList(geotiffGLList);
		glPopMatrix();
	}

	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	if (showPointclouds != 0) {
		glPushMatrix();
			glCallList(pointcloudGLList);
		glPopMatrix();
	}

	if (showRobots && showGeoTiff < 2) {
		glPushMatrix();
			glCallList(robotGLList);
		glPopMatrix();
	}

	if (showGeoTiff < 2 && showPointclouds < 2) {
		glPushMatrix();
			glCallList(mapGLList);
		glPopMatrix();
	}

	if (showGeoTiff < 2 && render != NULL) {
		Lock lock(rwLock);

		if (showTags) {
			render->renderTags(textQueue);
		}
	}

	textQueue.insert(textQueue.end(), mapText.begin(), mapText.end());
}

void MapView::mapUpdated(MapPtr map) {
	Lock lock(rwLock);
	if (map == this->map) {
		mapDirty = tiffDirty = robotsDirty = true;
		Map::TagListPtr tags = map->getTags();

		if (tags != this->tags) {{
			Map::TagListPtr tags = map->getTags();
			Lock lock2(tagLock, true);
			pointcloudDirty = true;
			this->tags = tags;
		}}
	}
	Q_EMIT updateNeeded();
}

void MapView::tagAdded(MapPtr map, TagPtr tag) {
	Lock lock(rwLock);
	if (this->map == map) {{
		Map::TagListPtr tags = map->getTags();
		Lock lock2(tagLock, true);
		pointcloudDirty = true;
		this->tags = tags;
	}}
	Q_EMIT updateNeeded();
}

void MapView::tagChanged(MapPtr map, TagPtr tag) {
	Lock lock(rwLock);
	if (this->map == map) {{
		Map::TagListPtr tags = map->getTags();
		Lock lock2(tagLock, true);
		pointcloudDirty = true;
		this->tags = tags;
	}}
	Q_EMIT updateNeeded();
}

void MapView::keyPressEvent(QKeyEvent *e) {
	bool eventConsumed = false;
	double step = 1;
	if (useCursor) {
		switch (e->key()) {
		case Qt::Key_Y:
			eventConsumed = true;
			cursor = Pose2D();

			if (render != NULL) {
				render->setSelectedMap(Point(cursor.position.x, cursor.position.y, 0));

				mapDirty = pointcloudDirty = tiffDirty = robotsDirty = true;
			}
			break;
		case Qt::Key_I:case Qt::Key_J:case Qt::Key_K:case Qt::Key_L:
			eventConsumed = true;
			step = 1;
			if((e->modifiers() & Qt::ShiftModifier) || (e->modifiers() & Qt::ControlModifier)) {
				if((e->modifiers() & Qt::ShiftModifier) && (e->modifiers() & Qt::ControlModifier)){
					step = 0.01;
				} else {
					step = 0.1;
				}
			}

			switch(e->key()) {
			case Qt::Key_I:
				cursor.position.y += step; break;
			case Qt::Key_J:
				cursor.position.x -= step; break;
			case Qt::Key_K:
				cursor.position.y -= step; break;
			case Qt::Key_L:
				cursor.position.x += step; break;
			}

			if (render != NULL) {
				render->setSelectedMap(Point3D(cursor.position.x, cursor.position.y, 0));

				mapDirty = pointcloudDirty = tiffDirty = robotsDirty = true;
			}
			break;
		case Qt::Key_U:case Qt::Key_O:
			eventConsumed = true;
			step = DEG2RAD(15);
			if(e->modifiers() & Qt::ShiftModifier) {
				if(e->modifiers() & Qt::ControlModifier){
					step = DEG2RAD(0.5);
				} else {
					step = DEG2RAD(3);
				}
			}

			switch (e->key()) {
			case Qt::Key_U:
				cursor.orientation += step; break;
			case Qt::Key_O:
				cursor.orientation -= step; break;
			}
			break;
		}
	}

	if (eventConsumed) {
		Q_EMIT this->updateNeeded();
	} else {
		WASDMouseView::keyPressEvent(e);
	}
}

void MapView::setShowGrids(int show) {
	showGrids = (show != 0);

	Q_EMIT updateNeeded();
}

void MapView::setShowRobots(int show) {
	showRobots = show != 0;
	tiffDirty = true;

	Q_EMIT updateNeeded();
}

void MapView::setShowTags(int show) {
	showTags = show != 0;

	Q_EMIT updateNeeded();
}

void MapView::setShowPointClouds(int show) {
	if (show != showPointclouds) {
		mapDirty = robotsDirty = true;
	}
	showPointclouds = show;

	Q_EMIT updateNeeded();
}

void MapView::setShowGeoTiff(int show) {
	showGeoTiff = show;

	Q_EMIT updateNeeded();
}


void MapView::setCursorEnabled(bool enable) {
	useCursor = enable;

	Q_EMIT updateNeeded();
}

} // namespace gui

} // namespace casros
