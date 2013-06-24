/*
 * snapviewpanel.cpp
 *
 *  Created on: 04/09/2009
 *      Author: rescue
 */
#include <crosbot_ui/panels/snap.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_ui/crosbotgui.hpp>

//#include <typeinfo>
#include <QtCore/QUrl>
#include <QtGui/QHBoxLayout>
//#include <QVBoxLayout>
//#include <QLabel>
//#include <casrobot/tasks/autonomy/autonomy.h>

namespace crosbot {

namespace gui {

SnapPanel::SnapPanel(ConfigElementPtr config) : Panel(config), widget(config)
{}

SnapPanel::~SnapPanel() {
}

void SnapPanel::start() {
	widget.start();
}

void SnapPanel::stop() {
	widget.stop();
}

QWidget *SnapPanel::getWidget() {
	return &widget;
}

void SnapView::setSnap(SnapPtr snap) {
	this->snap = snap;
	rgbImages.clear();
	if (snap == NULL) {
		setHtml(QString(""));
		return;
	}
	char line[4096];
	sprintf(line, "<h2>%s</h2>", snap->description.c_str());
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
		type = "Confirmed"; break;
	case Snap::UNCONFIRMED:
		type = "Unconfirmed"; break;
	case Snap::DUPLICATE:
		type = "Duplicate"; break;
	case Snap::REJECTED:
		type = "Rejected"; break;
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

SnapPtr SnapView::getSnap() {
	return this->snap;
}

QVariant SnapView::loadResource ( int type, const QUrl & name ) {
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

SnapViewWidget::SnapViewWidget(ConfigElementPtr config) :
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
		rejectedRB("Rejected", &statusGB)
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

	mapName = config->getParam(PARAM_NAME);
	mapName = config->getParam(ELEMENT_MAP, mapName);
}

SnapViewWidget::~SnapViewWidget() {
    disconnect(this, SIGNAL(confirmationNeeded()), this, SLOT(handleSnapConfirmation()));
    disconnect(&snapTree, SIGNAL(itemSelectionChanged()), this, SLOT(handleSelectionChanged()));
    disconnect(&unconfirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&confirmedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&duplicateRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));
    disconnect(&rejectedRB, SIGNAL(clicked()), this, SLOT(handleChangeInStatus()));

	tags = NULL;
	map = NULL;
}

class SnapTreeItem : public QTreeWidgetItem {
public:
	SnapPtr snap;
	MapPtr map;
	SnapTreeItem(SnapPtr snap, MapPtr map) :
			QTreeWidgetItem(QStringList(QString(snap->description.c_str()))),
			snap(snap), map(map)
	{
		updateView();
	}

	~SnapTreeItem() {}

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


std::vector<SnapTreeItem *> SnapViewWidget::currentSnaps() {
    std::vector<SnapTreeItem*> knownSnaps;

    for (int i = 0; i < victims.childCount(); ++i) {
    	SnapTreeItem *known = dynamic_cast<SnapTreeItem*>(victims.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < landmarks.childCount(); ++i) {
    	SnapTreeItem *known = dynamic_cast<SnapTreeItem*>(landmarks.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < scans.childCount(); ++i) {
    	SnapTreeItem *known = dynamic_cast<SnapTreeItem*>(scans.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }
    for (int i = 0; i < other.childCount(); ++i) {
    	SnapTreeItem *known = dynamic_cast<SnapTreeItem*>(other.child(i));
    	if (known != NULL)
    		knownSnaps.push_back(known);
    }

    return knownSnaps;
}

void SnapViewWidget::listSnaps(Map::TagListPtr tags) {
	if (this->tags == tags) {
		return;
	}

	this->tags = tags;

    std::vector<SnapTreeItem*> knownSnaps = currentSnaps();

    for (size_t i = 0; i < tags->tags.size(); ++i) {
    	SnapPtr snap = dynamic_cast<Snap*>(tags->tags[i].tag.get());
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

    	SnapTreeItem * twi = new SnapTreeItem(snap, map);
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
    	SnapTreeItem* sti = knownSnaps[i];
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

void SnapViewWidget::start() {
	snapTree.clearSelection();
	if (mapName != "") {
		map = Gui::maps.getMap(mapName);
		if (map != NULL) {
			map->addListener(this);
			tags = map->getTags();

			mapUpdated(map);
		} else {
			ERROR("SnapPanel: Unable to access map %s.\n", mapName.c_str());
		}
	}
}

void SnapViewWidget::stop() {
	snapView.setSnap(NULL);
	snapTree.clearSelection();
	tags = NULL;
	map = NULL;

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

void SnapViewWidget::mapUpdated(MapPtr map) {
	listSnaps(map->getTags());
}

void SnapViewWidget::tagAdded(MapPtr map, TagPtr tag) {
	listSnaps(map->getTags());

	SnapPtr snap = dynamic_cast<Snap *>(tag.get());
	if (snap == NULL)
		return;

	//logger->log("SnapViewWidget :: Got snap. type: %d status: %d\n", snap->type, snap->status);
	if (snap->type == Snap::VICTIM && snap->status == Snap::UNCONFIRMED) {
		Q_EMIT confirmationNeeded();
	}
}

void SnapViewWidget::tagChanged(MapPtr map, TagPtr tag) {
	SnapPtr snap = dynamic_cast<Snap*>(tag.get());
	if (snapView.getSnap() != snap || snap == NULL)
		return;


	std::vector<SnapTreeItem*> knowns = currentSnaps();
	for (size_t i = 0; i < knowns.size(); ++i) {
		if (knowns[i]->snap == snap)
			knowns[i]->updateView();
	}
//	snapView.setSnap(snap);
}

void SnapViewWidget::handleSnapConfirmation() {
    // TODO: SnapViewWidget::handleSnapConfirmation()
}

void SnapViewWidget::handleChangeInStatus() {
	SnapPtr snap = snapView.getSnap();
	if (snap == NULL)
		return;

	if (confirmedRB.isChecked()) {
		if (snap->status == Snap::CONFIRMED)
			return;
		snap->status = Snap::CONFIRMED;
	} else if (duplicateRB.isChecked()) {
		if (snap->status == Snap::DUPLICATE)
			return;
		snap->status = Snap::DUPLICATE;
	} else if (rejectedRB.isChecked()) {
		if (snap->status == Snap::REJECTED)
			return;
		snap->status = Snap::REJECTED;
	} else {
		if (snap->status == Snap::UNCONFIRMED)
			return;
		snap->status = Snap::UNCONFIRMED;
	}

	map->tagChanged(snap);
}

void SnapViewWidget::handleSelectionChanged() {
	QList<QTreeWidgetItem *> selection = snapTree.selectedItems();
	QTreeWidgetItem *item = NULL;
	if (selection.size() > 0)
		item = selection[0];
	if (item == NULL || typeid(*item) != typeid(SnapTreeItem)) {
		snapView.setSnap(NULL);
		confirmedRB.setEnabled(false);
		unconfirmedRB.setEnabled(false);
		duplicateRB.setEnabled(false);
		rejectedRB.setEnabled(false);
		return;
	}

	SnapPtr snap = ((SnapTreeItem *)item)->snap;
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

} // namespace gui

} //namespace casros
