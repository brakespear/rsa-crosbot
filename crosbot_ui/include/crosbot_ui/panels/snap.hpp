/*
 * snapviewpanel.h
 *
 *  Created on: 13/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_SNAP_H_
#define CROSBOT_PANEL_SNAP_H_

#include <crosbot_ui/panels/panel.hpp>
#include <crosbot/data.hpp>
#include <crosbot_map/map.hpp>

#include <QtGui/QTextEdit>
#include <QtGui/QTreeWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QRadioButton>
//#include <QDialog>
//#include <QTimer>
//#include <QPushButton>
//#include <queue>

namespace crosbot {

namespace gui {

#define PANEL_SNAP	"snap"
#define ELEMENT_SNAPS	"snaps"

class SnapView : public QTextEdit {
Q_OBJECT
public:
	void setSnap(SnapPtr snap);
	SnapPtr getSnap();
protected:
	QVariant loadResource ( int type, const QUrl & name );
	SnapPtr snap;
	std::vector<ImagePtr> rgbImages;
};

class SnapTreeItem;
class SnapViewWidget : public QWidget, virtual public MapListener {
Q_OBJECT
public:
	SnapViewWidget(ConfigElementPtr config);
	~SnapViewWidget();

	void start();
	void stop();

	void motionTracked(MapPtr map) {}
	void mapUpdated(MapPtr map);
	void tagAdded(MapPtr map, TagPtr tag);
	void tagChanged(MapPtr map, TagPtr tag);

	void listSnaps(Map::TagListPtr tags);
public Q_SLOTS:
	void handleSelectionChanged();
	void handleChangeInStatus();
	void handleSnapConfirmation();
Q_SIGNALS:
	void confirmationNeeded();
protected:
	SnapView snapView;

	std::string mapName;
	
	MapPtr map;
	Map::TagListPtr tags;
	
	QTreeWidget snapTree;
	QTreeWidgetItem victims;
	QTreeWidgetItem landmarks;
	QTreeWidgetItem scans;
	QTreeWidgetItem other;
	
	QGroupBox statusGB;
	QRadioButton confirmedRB;
	QRadioButton unconfirmedRB;
	QRadioButton duplicateRB;
	QRadioButton rejectedRB;
	
	std::vector<SnapTreeItem *> currentSnaps();
};

/**
 * \image html panel-snap.png
 */
class SnapPanel : public Panel {
Q_OBJECT
protected:
	SnapViewWidget widget;
public:
	SnapPanel(ConfigElementPtr config);
	~SnapPanel();

	QWidget *getWidget();
	
	void start();
	void stop();
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_SNAP_H_ */
