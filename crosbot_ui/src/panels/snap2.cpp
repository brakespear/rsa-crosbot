/*
 * snapviewpanel.cpp
 *
 *  Created on: 04/09/2009
 *      Author: rescue
 */
#include <crosbot_ui/panels/snap2.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_ui/crosbotgui.hpp>

#include <crosbot_map/ListSnaps.h>
#include <crosbot_map/GetSnap.h>
#include <crosbot_map/ModifySnap.h>

//#include <typeinfo>
#include <QtCore/QUrl>
#include <QtGui/QHBoxLayout>
#include <QtGui/QLabel>
#include <QtGui/QSound>

#ifdef USE_PHONON

#include <phonon/audiooutput.h>
#include <phonon/mediaobject.h>
#include <phonon/mediasource.h>
#include <phonon/videowidget.h>

#endif

//#include <QVBoxLayout>
//#include <QLabel>
//#include <casrobot/tasks/autonomy/autonomy.h>

namespace crosbot {

namespace gui {

SnapPanel2::SnapPanel2(ConfigElementPtr config) : Panel(config), widget(config)
{}

SnapPanel2::~SnapPanel2() {
}

void SnapPanel2::start() {
	widget.start();
}

void SnapPanel2::stop() {
	widget.stop();
}

QWidget *SnapPanel2::getWidget() {
	return &widget;
}

void SnapView2::setSnap(SnapPtr snap) {
	this->snap = snap;
	rgbImages.clear();
	if (snap == NULL) {
		setHtml(QString(""));
		return;
	}
	char line[4096];
	sprintf(line, "<h2>%d - %s</h2>", snap->id, snap->description.c_str());
	std::string contents = line;

	std::string type = "Unknown";
	switch (snap->type) {
	case Snap::VICTIM:
		type = "Victim"; break;
	case Snap::LANDMARK:
		type = "Landmark"; break;
	case Snap::SCAN:
		type = "Scan"; break;
	}
	std::string status = "Unknown";
	switch (snap->status) {
	case Snap::CONFIRMED:
		status = "Confirmed"; break;
	case Snap::UNCONFIRMED:
		status = "Unconfirmed"; break;
	case Snap::DUPLICATE:
		status = "Duplicate"; break;
	case Snap::REJECTED:
		status = "Rejected"; break;
	}
	sprintf(line, "<h3>%s - %s</h3>", type.c_str(), status.c_str());
	contents += line;

	sprintf(line, "<p>There are a total of %Zu images.<p>", snap->images.size());
	contents += line;
	sprintf(line, "<p>There are a total of %Zu point clouds.<p>", snap->clouds.size());
	contents += line;
	for(size_t i = 0; i < snap->images.size(); i++){
		double rotation=0;
		sprintf(line, "<p><img src=\"snapimage/%Zu/%lf\"></p>", i, rotation);
		contents += line;
		rgbImages.push_back(snap->images[i]->inEncoding(Image::RGB8));
	}

//	QString contents = QString("<h2>%1</h2>").arg(snap->description.c_str());
//	contents.append(QString("<p>There are a total of %1 images.<p>").arg(snap->images.size()));
//	for(unsigned int i = 0; i < snap->images.size(); i++){
//    	double rotation=0;
//	    contents.append(QString("<p><img src=\"snapimage/%1/%2\"></p>").arg(i).arg(rotation));
//	    rgbImages.push_back(snap->images[i]->inEncoding(Image::RGB8));
//	}
	setHtml(QString(contents.c_str()));
}

SnapPtr SnapView2::getSnap() {
	return this->snap;
}

QVariant SnapView2::loadResource ( int type, const QUrl & name ) {
	QString resourceName = name.toString();
	QStringList parts = resourceName.split("/");
	if(snap != NULL && parts[0] == "snapimage" && parts.size() > 2) {
		int imageId = parts[1].toInt();
		int rotation = parts[2].toInt();
		
		if (imageId < 0 || imageId >= (int)rgbImages.size()) {
			ERROR("Snap %s has no image %d.\n", snap->description.c_str(), imageId);
			return QVariant(QVariant::String);
		}
		
		ImagePtr image = rgbImages[imageId];
		
		QImage returnImg((unsigned char *)image->data, image->width, image->height,
				image->width*3, QImage::Format_RGB888);
		returnImg = returnImg.transformed(QMatrix().rotate(rotation));
		return QVariant::fromValue(returnImg);
	} else {
		LOG("SnapView::loadResource: incorrect URL passed in.\n");
		return QTextEdit::loadResource(type, name);
	}
}

SnapViewWidget2::SnapViewWidget2(ConfigElementPtr config) :
		QWidget(),
		snapTree(this),
		victims(&snapTree),
		landmarks(&snapTree),
		scans(&snapTree),
		other(&snapTree),
		statusGB("Status"),
		confirmedRB("Confirmed", &statusGB),
		unconfirmedRB("Unconfirmed", &statusGB),
		duplicateRB("Duplicate", &statusGB),
		rejectedRB("Rejected", &statusGB),
		updateThread(*this)
{
    snapTree.setHeaderLabels(QStringList("Snaps"));
    snapTree.setColumnCount(1);
    snapTree.setSelectionMode(QAbstractItemView::SingleSelection);
    victims.setText(0,"Victims");
    victims.setExpanded(true);
    landmarks.setText(0,"Landmarks");
    landmarks.setExpanded(true);
    scans.setText(0,"Scans");
    scans.setExpanded(true);
    other.setText(0,"Other");
    other.setExpanded(true);
    QFont font = victims.font(0);
    font.setBold(true);
    victims.setFont(0, font);
    landmarks.setFont(0, font);
    scans.setFont(0, font);
    other.setFont(0, font);
    
    QHBoxLayout *statusLayout = new QHBoxLayout();
    statusLayout->addWidget(&confirmedRB);
    statusLayout->addWidget(&unconfirmedRB);
    statusLayout->addWidget(&duplicateRB);
    statusLayout->addWidget(&rejectedRB);
    statusGB.setLayout(statusLayout);
    
    QVBoxLayout *vlayout = new QVBoxLayout();
    vlayout->addWidget(&snapTree,3);
    vlayout->addWidget(&statusGB,1);
    vlayout->addWidget(&snapView,10);
    setLayout(vlayout);
    
    connect(&snapTree, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelectionChanged()));
    connect(&unconfirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    connect(&confirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    connect(&duplicateRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    connect(&rejectedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    
    confirmedRB.setEnabled(false);
    unconfirmedRB.setEnabled(false);
    duplicateRB.setEnabled(false);
    rejectedRB.setEnabled(false);
    
    connect(this, SIGNAL(confirmationNeeded()), this, SLOT(handleSnapConfirmation()));
    connect(this, SIGNAL(updateSnapList()), this, SLOT(handleListUpdate()));

//	mapName = config->getParam(PARAM_NAME);
//	mapName = config->getParam(ELEMENT_MAP, mapName);
}

SnapViewWidget2::~SnapViewWidget2() {
    disconnect(this, SIGNAL(confirmationNeeded()), this, SLOT(handleSnapConfirmation()));
    disconnect(&snapTree, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelectionChanged()));
    disconnect(&unconfirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&confirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&duplicateRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&rejectedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));

//	tags = NULL;
//	map = NULL;
}

std::vector<SnapTreeItem2 *> SnapViewWidget2::currentSnaps() {
    std::vector<SnapTreeItem2 *> knownSnaps;

    for (int i = 0; i < victims.childCount(); ++i) {
    	SnapTreeItem2 *known = dynamic_cast<SnapTreeItem2*>(victims.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < landmarks.childCount(); ++i) {
    	SnapTreeItem2 *known = dynamic_cast<SnapTreeItem2*>(landmarks.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < scans.childCount(); ++i) {
    	SnapTreeItem2 *known = dynamic_cast<SnapTreeItem2*>(scans.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < other.childCount(); ++i) {
    	SnapTreeItem2 *known = dynamic_cast<SnapTreeItem2*>(other.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }

    return knownSnaps;
}

SnapTreeItem2* SnapViewWidget2::findSnap(int32_t id, Snap::Type type) {
	switch(type) {
	case Snap::VICTIM:
		for (int i = 0; i < victims.childCount(); ++i) {
			SnapTreeItem2 *leaf = dynamic_cast<SnapTreeItem2 *>(victims.child(i));
			if (leaf != NULL && leaf->snap->id == id) {
				return leaf;
			}
		} break;
	case Snap::LANDMARK:
		for (int i = 0; i < landmarks.childCount(); ++i) {
			SnapTreeItem2 *leaf = dynamic_cast<SnapTreeItem2 *>(landmarks.child(i));
			if (leaf != NULL && leaf->snap->id == id) {
				return leaf;
			}
		} break;
	case Snap::SCAN:
		for (int i = 0; i < scans.childCount(); ++i) {
			SnapTreeItem2 *leaf = dynamic_cast<SnapTreeItem2 *>(scans.child(i));
			if (leaf != NULL && leaf->snap->id == id) {
				return leaf;
			}
		} break;

	default:
		for (int i = 0; i < other.childCount(); ++i) {
			SnapTreeItem2 *leaf = dynamic_cast<SnapTreeItem2 *>(other.child(i));
			if (leaf != NULL && leaf->snap->id == id && leaf->snap->type == type) {
				return leaf;
			}
		} break;
	}

	return NULL;
}


void SnapViewWidget2::setStatus(SnapPtr snap, Snap::Status status) {
	snap->status = status;
	crosbot_map::ModifySnap::Request req;
	crosbot_map::ModifySnap::Response res;
	req.id.data =  snap->id;
	req.type.data = snap->type;
	req.status.data = snap->status;
	req.description.data = snap->description;
	modifySrv.call(req, res);
}

void SnapViewWidget2::callbackSnap(crosbot_map::SnapMsgPtr snap) {
	if (snap->type == Snap::VICTIM &&snap->status == Snap::UNCONFIRMED) {
		Pose p = snap->pose;
		if (p.position.length() <= 1.5) {
			SnapPtr s = new Snap(snap);
			toConfirm.push_back(s);

			Q_EMIT confirmationNeeded();
		}
	}
}

void SnapViewWidget2::handleListUpdate() {
	crosbot_map::ListSnaps::Request request;
	crosbot_map::ListSnaps::Response response;

	if (listSrv.call(request, response)) {
		for (size_t i = 0; i < response.snaps.size(); ++i) {
			SnapTreeItem2* snapItem = findSnap(response.snaps[i].id, (Snap::Type)response.snaps[i].type);

			if (snapItem == NULL) {
				SnapPtr snap =  new Snap(response.snaps[i]);
				QTreeWidgetItem *item = new SnapTreeItem2(snap);

				if (snap->type == Snap::VICTIM) {
					victims.addChild(item);
				} else if (snap->type == Snap::LANDMARK) {
					landmarks.addChild(item);
				} else if (snap->type == Snap::SCAN) {
					scans.addChild(item);
				} else {
					other.addChild(item);
				}
			} else {
				crosbot_map::SnapMsg& msg = response.snaps[i];
				bool repaint = false;
				if (snapItem->snap->status != msg.status) {
					snapItem->snap->status = (Snap::Status)msg.status;
					repaint = true;
				}
				if (snapItem->snap->description != msg.description) {
					snapItem->snap->description = msg.description;
					repaint = true;
				}

				if (snapItem->snap->type !=  msg.type) {
					repaint = true;

					snapItem->parent()->removeChild(snapItem);
					snapItem->snap->type =  (Snap::Type)msg.type;
					switch(snapItem->snap->type) {
					case Snap::VICTIM:
						victims.addChild(snapItem); break;
					case Snap::LANDMARK:
						landmarks.addChild(snapItem); break;
					case Snap::SCAN:
						scans.addChild(snapItem); break;
					defautl:
						other.addChild(snapItem); break;
					}
				}

				if (repaint) {
					snapItem->updateView();
				}

				if (snapItem->snap == snapView.getSnap())
					snapView.setSnap(snapItem->snap);
			}
		}
	}
}

void SnapViewWidget2::updateList() {
	while (updateThread.operating) {
		Q_EMIT updateSnapList();

		usleep(2000000);
	}
}

void SnapViewWidget2::listSnaps(Map::TagListPtr tags) {
//	if (this->tags == tags) {
//		return;
//	}
//
//	this->tags = tags;

    std::vector<SnapTreeItem2*> knownSnaps = currentSnaps();

//    for (size_t i = 0; i < tags->tags.size(); ++i) {
	for (size_t i = 0; i < knownSnaps.size(); ++i) {
		SnapPtr snap;
//    	SnapPtr snap = dynamic_cast<Snap*>(tags->tags[i].tag.get());
    	if (snap == NULL)
    		continue;

    	bool found = false;
    	for (size_t j = 0; !found && j < knownSnaps.size(); ++j) {
    		if (knownSnaps[j]->snap == snap) {
    			found = true;

    			knownSnaps.erase(knownSnaps.begin() + j);
    		}
    	}

    	if (found)
    		continue;

    	SnapTreeItem2 * twi = new SnapTreeItem2(snap);
    	switch (snap->type) {
    	case Snap::VICTIM:
    		victims.addChild(twi); break;
    	case Snap::LANDMARK:
    		landmarks.addChild(twi); break;
    	case Snap::SCAN:
    		scans.addChild(twi); break;
    	default:
    		other.addChild(twi); break;
    	}
    }

    for (size_t i = 0; i < knownSnaps.size(); ++i) {
    	SnapTreeItem2 *sti = knownSnaps[i];
    	if (sti->snap == snapView.getSnap()) {
    		snapTree.clearSelection();
    	}

    	victims.removeChild(sti);
    	landmarks.removeChild(sti);
    	scans.removeChild(sti);
    	other.removeChild(sti);

    	delete sti;
    }
}

#ifdef USE_PHONON

Phonon::AudioOutput *audio = NULL;
Phonon::MediaObject obj;

#endif

void SnapViewWidget2::start() {
	snapTree.clearSelection();
	updateThread.start();

	ros::NodeHandle nh;
	snapSub = nh.subscribe("/snaps/publisher", 100, &SnapViewWidget2::callbackSnap, this);
	listSrv = nh.serviceClient<crosbot_map::ListSnaps>("/snaps/list", false);
	getSrv = nh.serviceClient<crosbot_map::GetSnap>("/snaps/get", false);
	modifySrv = nh.serviceClient<crosbot_map::ModifySnap>("/snaps/update", false);

#ifdef USE_PHONON

	audio = new Phonon::AudioOutput( this );
	if (audio != NULL)
		Phonon::createPath( &obj, audio );

#endif

//	if (mapName != "") {
//		map = Gui::maps.getMap(mapName);
//		if (map != NULL) {
//			map->addListener(this);
//			tags = map->getTags();
//
//			mapUpdated(map);
//		} else {
//			ERROR("SnapPanel: Unable to access map %s.\n", mapName.c_str());
//		}
//	}
}

void SnapViewWidget2::stop() {
	updateThread.operating = false;
	snapView.setSnap(NULL);
	snapTree.clearSelection();
//	tags = NULL;
//	map = NULL;

	while (victims.childCount() > 0) {
		QTreeWidgetItem *twi = victims.child(0);
		victims.removeChild(twi);
		delete twi;
	}
	while (landmarks.childCount() > 0) {
		QTreeWidgetItem *twi = landmarks.child(0);
		landmarks.removeChild(twi);
		delete twi;
	}
	while (scans.childCount() > 0) {
		QTreeWidgetItem *twi = scans.child(0);
		scans.removeChild(twi);
		delete twi;
	}
	while (other.childCount() > 0) {
		QTreeWidgetItem *twi = other.child(0);
		other.removeChild(twi);
		delete twi;
	}
}

//void SnapViewWidget2::mapUpdated(MapPtr map) {
//	listSnaps(map->getTags());
//}
//
//void SnapViewWidget2::tagAdded(MapPtr map, TagPtr tag) {
//	listSnaps(map->getTags());
//
//	SnapPtr snap = dynamic_cast<Snap *>(tag.get());
//	if (snap == NULL)
//		return;
//
//	//logger->log("SnapViewWidget :: Got snap. type: %d status: %d\n", snap->type, snap->status);
//	if (snap->type == Snap::VICTIM && snap->status == Snap::UNCONFIRMED) {
//		Q_EMIT confirmationNeeded();
//	}
//}
//
//void SnapViewWidget2::tagChanged(MapPtr map, TagPtr tag) {
//	SnapPtr snap = dynamic_cast<Snap*>(tag.get());
//	if (snapView.getSnap() != snap || snap == NULL)
//		return;
//
//
//	std::vector<SnapTreeItem*> knowns = currentSnaps();
//	for (size_t i = 0; i < knowns.size(); ++i) {
//		if (knowns[i]->snap == snap)
//			knowns[i]->updateView();
//	}
////	snapView.setSnap(snap);
//}

void SnapViewWidget2::playSound(std::string file) {
#ifdef USE_PHONON
	if (audio == NULL)
		return;
	const QUrl url = QUrl( QLatin1String( file.c_str() ) );
	Phonon::MediaSource src( url );
	obj.setCurrentSource( src );
//		VideoWidget video;
//		video.show();
//		Phonon::createPath( &obj, &video );
	obj.play();
//	obj.state();
#endif
}

void SnapViewWidget2::handleSnapConfirmation() {
	while (toConfirm.size() > 0) {
		SnapPtr snap = toConfirm[0];
		toConfirm.erase(toConfirm.begin());

		SnapConfirmDialog dialog(snap, this);
//		playSound("file:///usr/lib/libreoffice/share/gallery/sounds/train.wav");
		playSound("file:///usr/lib/libreoffice/share/gallery/sounds/beam.wav");
		int result = dialog.exec();bool statusChanged = false;
		if (result == 1) {
			setStatus(snap, Snap::CONFIRMED);
			playSound("file:///usr/lib/libreoffice/share/gallery/sounds/applause.wav");
		} else if (result == -1) {
			setStatus(snap, Snap::REJECTED);
		}
	}
}

void SnapViewWidget2::handleChangeInStatus() {
	SnapPtr snap = snapView.getSnap();
	if (snap == NULL)
		return;

	if (confirmedRB.isChecked()) {
		if (snap->status == Snap::CONFIRMED)
			return;
		setStatus(snap, Snap::CONFIRMED);
	} else if (duplicateRB.isChecked()) {
		if (snap->status == Snap::DUPLICATE)
			return;
		setStatus(snap, Snap::DUPLICATE);
	} else if (rejectedRB.isChecked()) {
		if (snap->status == Snap::REJECTED)
			return;
		setStatus(snap, Snap::REJECTED);
	} else {
		if (snap->status == Snap::UNCONFIRMED)
			return;
		setStatus(snap, Snap::UNCONFIRMED);
	}

	SnapTreeItem2* item = findSnap(snap->id, snap->type);
	if (item != NULL)
		item->updateView();

//	map->tagChanged(snap);
}

void SnapViewWidget2::handleSelectionChanged() {
	QList<QTreeWidgetItem *> selection = snapTree.selectedItems();
	QTreeWidgetItem *item = NULL;
	if (selection.size() > 0)
		item = selection[0];
	if (item == NULL || typeid(*item) != typeid(SnapTreeItem2)) {
		snapView.setSnap(NULL);
		confirmedRB.setEnabled(false);
		unconfirmedRB.setEnabled(false);
		duplicateRB.setEnabled(false);
		rejectedRB.setEnabled(false);
		return;
	}

	SnapPtr snap = ((SnapTreeItem2 *)item)->snap;

	if (snap->images.size() == 0 && snap->clouds.size() == 0) {
			crosbot_map::GetSnap::Request req;
			req.id.data = snap->id;
			req.type.data = snap->type;
			crosbot_map::GetSnap::Response res;

			if (getSrv.call(req, res)) {
				SnapPtr snap2 = new Snap(res.snap);
				snap->images = snap2->images;
				snap->clouds = snap2->clouds;
			}
	}

	snapView.setSnap(snap);
	switch (snap->status) {
	case Snap::CONFIRMED:
		confirmedRB.setChecked(true); break;
	case Snap::DUPLICATE:
		duplicateRB.setChecked(true); break;
	case Snap::REJECTED:
		rejectedRB.setChecked(true); break;
	case Snap::UNCONFIRMED: default:
		unconfirmedRB.setChecked(true); break;
	}

	confirmedRB.setEnabled(true);
	unconfirmedRB.setEnabled(true);
	duplicateRB.setEnabled(true);
	rejectedRB.setEnabled(true);
}

SnapConfirmDialog::SnapConfirmDialog(SnapPtr snap, QWidget *parent) : QDialog(parent),
// waittime(this),
snap(snap),
//counttime(this),
countlbl("30", this)

{
	setModal(true);

	QPushButton *confirmBtn = new QPushButton(tr("Confirm"));
	QPushButton *deferBtn = new QPushButton(tr("Defer"));
	QPushButton *rejectBtn = new QPushButton(tr("Reject"));

	QGroupBox *gb = new QGroupBox("");

	QHBoxLayout *hbox = new QHBoxLayout();
	hbox->addWidget(confirmBtn);
	hbox->addWidget(deferBtn);
	hbox->addWidget(rejectBtn);
	hbox->addWidget(&countlbl);

	QVBoxLayout *vbox = new QVBoxLayout();
	QLabel *instlbl = new QLabel("Is this really a victim?\n"
			"Select Defer to leave victim as unconfirmed.\n"
			"Warning, If you defer another notification may be sent if a closer observation is made\n\n");

	vbox->addWidget(instlbl, 1);
	vbox->addWidget(&snapView, 20);
	gb->setLayout(hbox);
	vbox->addWidget(gb, 1);

	setLayout(vbox);

	connect(confirmBtn, SIGNAL(clicked()), this, SLOT(confirmFn()));
	connect(deferBtn, SIGNAL(clicked()), this, SLOT(deferFn()));
	connect(rejectBtn, SIGNAL(clicked()), this, SLOT(rejectFn()));
//	connect(&waittime, SIGNAL(timeout()), this, SLOT(timeup()));
//	connect(&counttime, SIGNAL(timeout()), this, SLOT(counter()));
	connect(&countlbl, SIGNAL(clicked()), this, SLOT(timeup()));

	QPalette ptmp;
	ptmp.setColor(QPalette::Button, QColor(255, 0, 0));
	countlbl.setPalette(ptmp);

	setWindowTitle(tr("New Victim"));
	resize(500, 1000);
	snapView.setSnap(snap);
	result = -2;
//	waittime.start(30000);
	// TODO SnapConfirmDialog: load this from config
	tcount = 31;
	// wait 1 second
//	counttime.start(1000);
}

// if time up then return, else set return value
void SnapConfirmDialog::confirmFn() {
	Q_EMIT done(1);
}
// if time up then return, else set return value
void SnapConfirmDialog::rejectFn() {
	Q_EMIT done(-1);
}

// if time up then return, else set return value
void SnapConfirmDialog::deferFn() {
	Q_EMIT done(0);
}

// override timeout
void SnapConfirmDialog::timeup() {
	tcount = 1;
}

// every second
void SnapConfirmDialog::counter() {
	if (--tcount > 0) { // keep counting
	} else { // time up
//		counttime.stop();
		QPalette ptmp;
		// change display
		ptmp.setColor(QPalette::Button, QColor(0, 255, 0));
		countlbl.setPalette(ptmp);
		if (result == -2) {
			// no decision made yet, return immediately on click
			result = 2;
		} else if (result != 2){
			// return if button clicked
			Q_EMIT done(result);
		}
	}
	countlbl.setText(QString("%1").arg(tcount));
	countlbl.update();
}

} // namespace gui

} //namespace casros
