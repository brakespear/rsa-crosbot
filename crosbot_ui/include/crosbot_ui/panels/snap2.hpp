/*
 * snapviewpanel.h
 *
 *  Created on: 13/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_SNAP2_H_
#define CROSBOT_PANEL_SNAP2_H_

#include <ros/ros.h>
#include <crosbot_ui/panels/panel.hpp>
#include <crosbot/data.hpp>
#include <crosbot_map/map.hpp>

#include <crosbot_map/SnapMsg.h>

#include <QtGui/QTextEdit>
#include <QtGui/QTreeWidget>
#include <QtGui/QGroupBox>
#include <QtGui/QRadioButton>
#include <QtGui/QDialog>
#include <QtGui/QPushButton>

//#include <QDialog>
//#include <QTimer>
//#include <QPushButton>
//#include <queue>

namespace crosbot {

namespace gui {

#define PANEL_SNAP	"snap"
#define ELEMENT_SNAPS	"snaps"

class SnapTreeItem2 : public QTreeWidgetItem {
public:
	SnapPtr snap;
	SnapTreeItem2(SnapPtr snap) :
			QTreeWidgetItem(QStringList(QString(snap->description.c_str()))),
			snap(snap)
	{
		updateView();
	}

	~SnapTreeItem2() {}

	void updateView() {
		QFont font = this->font(0);
		font.setBold(false);
		font.setItalic(false);
		font.setStrikeOut(false);

		switch (snap->status) {
		case Snap::CONFIRMED:
			font.setBold(true); break;
		case Snap::DUPLICATE:
			font.setItalic(true); break;
		case Snap::REJECTED:
			font.setStrikeOut(true); break;
		case Snap::UNCONFIRMED: default:
			break;
		}

		setFont(0, font);
	}
};

class SnapView2 : public QTextEdit {
Q_OBJECT
public:
	void setSnap(SnapPtr snap);
	SnapPtr getSnap();
protected:
	QVariant loadResource ( int type, const QUrl & name );
	SnapPtr snap;
	std::vector<ImagePtr> rgbImages;
};

class SnapTreeItem2;
class SnapViewWidget2 : public QWidget {
Q_OBJECT
public:
	SnapViewWidget2(ConfigElementPtr config);
	~SnapViewWidget2();

	void start();
	void stop();

//	void motionTracked(MapPtr map) {}
//	void mapUpdated(MapPtr map);
//	void tagAdded(MapPtr map, TagPtr tag);
//	void tagChanged(MapPtr map, TagPtr tag);

	void listSnaps(Map::TagListPtr tags);
	void callbackSnap(crosbot_map::SnapMsgPtr snap);
public Q_SLOTS:
	void handleSelectionChanged();
	void handleChangeInStatus();
	void handleSnapConfirmation(SnapPtr snap);
Q_SIGNALS:
	void confirmationNeeded(SnapPtr snap);
protected:
	SnapView2 snapView;

//	std::string mapName;
	
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
	
	std::vector<SnapTreeItem2 *> currentSnaps();

	SnapTreeItem2* findSnap(int32_t id, Snap::Type type);
	void updateList();

	friend class SnapUpdateThread;
	class SnapUpdateThread: public Thread {
	public:
		SnapViewWidget2& panel;
		bool operating;
		SnapUpdateThread(SnapViewWidget2& panel) : panel(panel), operating(true) {}

		void run() {
			panel.updateList();
		}
	};
	SnapUpdateThread updateThread;
	ros::Subscriber snapSub;
	ros::ServiceClient listSrv, getSrv, modifySrv;

	void setStatus(SnapPtr snap, Snap::Status status);
};

/**
 * \image html panel-snap.png
 */
class SnapPanel2 : public Panel {
Q_OBJECT
protected:
	SnapViewWidget2 widget;

public:
	SnapPanel2(ConfigElementPtr config);
	~SnapPanel2();

	QWidget *getWidget();
	
	void start();
	void stop();
};


/*
 * confirmation dialog for autodetected victims
 * closes after result selected and 30 seconds passed
 */
class SnapConfirmDialog: public QDialog {
Q_OBJECT
public:
	SnapConfirmDialog(SnapPtr snap, QWidget *parent = 0);

public Q_SLOTS:
	void confirmFn();
	void rejectFn();
	void deferFn();
	void timeup();
	void counter();
private:
	SnapPtr snap;
	SnapView2 snapView;
//		QTimer waittime;
//	QTimer counttime;
	int result;
	QPushButton countlbl;
	int tcount;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_SNAP2_H_ */
