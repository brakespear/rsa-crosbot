/*
 * geo_logger.cpp
 *
 *  Created on: 25/06/2013
 *      Author: rescue
 */

#include <ros/ros.h>
#include <crosbot_ui/renders/map/geotiff.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_map/tag.hpp>

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <crosbot_map/ListSnaps.h>

#include <QtCore/QProcess>
#include <QtGui/QImage>
#include <QtGui/QColor>
#include <QtGui/QPen>
#include <QtGui/QPainter>
#include <QtGui/QFontMetrics>
#include <QtGui/QApplication>

using namespace crosbot;

class GeoLogger : public Thread {
public:
	bool operating;
	Mutex mapLock;
	nav_msgs::OccupancyGridConstPtr latestMap;
	nav_msgs::PathConstPtr latestHistory;

	ros::Subscriber gridSub, histSub;
	ros::ServiceClient snapSrv;

	std::string fileLoc;

	void callbackOccupancy(nav_msgs::OccupancyGridConstPtr grid) {
		Lock lock(mapLock);
		latestMap = grid;
	}

	void callbackHistory(nav_msgs::PathConstPtr history) {
		Lock lock(mapLock);
		latestHistory = history;
	}

	GeoLogger() : operating(true), fileLoc("/home/rescue/workspace/log/") {
		ros::NodeHandle nh;
		gridSub = nh.subscribe("map", 1, &GeoLogger::callbackOccupancy, this);
		histSub = nh.subscribe("history", 1, &GeoLogger::callbackHistory, this);
		snapSrv = nh.serviceClient< crosbot_map::ListSnaps >("/snaps/list", true);

		// Config
		ros::NodeHandle nhPriv("~");
		nhPriv.param("dir", fileLoc, fileLoc);
		if (*(fileLoc.rbegin()) != '/') fileLoc.append("/");
	}

	QImage *getGeoTiffImage(nav_msgs::OccupancyGridConstPtr& map, const std::string& title, nav_msgs::PathConstPtr& history, const std::vector< crosbot_map::SnapMsg >& snaps, std::string& geoData, std::string& snapDetails) {
		if (map == NULL || map->info.width == 0 || map->info.height == 0) {
			ERROR("GeoTiff: Cannot draw an empty map.\n");
			return NULL;
		}

	//	Point2D originAHM;
	//	originAHM.x = -(hmm->offx / map->info.resolution) - minHMX * hmm->subwidth;
	//	originAHM.y = -(hmm->offy / map->info.resolution) - minHMY * hmm->subheight;
	//	logger->log("FastSLAMRender:: o(%.4lf, %.4lf) o2(%.4lf, %.4lf)\n", minXY.x, minXY.y, originAHM.x, originAHM.y);

		// create image
		unsigned int imageWidth = map->info.width,
				imageHeight = map->info.height;

		QImage geotiffImage(imageWidth, imageHeight, QImage::Format_RGB32);
		QPainter painter(&geotiffImage);

		// paint unexplored checker board
		int checkerBoardPixelWidth = (int)round(GeoTIFFConstants::unexploredCheckerBoardSize / map->info.resolution),
				checkerBoardPixelHeight = (int)round(GeoTIFFConstants::unexploredCheckerBoardSize / map->info.resolution);

		for (unsigned int x = 0; x < imageWidth; x += checkerBoardPixelWidth) {
			for (unsigned int y = 0; y < imageHeight; y += checkerBoardPixelHeight) {
				Colour4f c;
				if (((x / checkerBoardPixelWidth + y / checkerBoardPixelHeight) & 0x01) == 0)
					c = GeoTIFFConstants::unexploredLight;
				else
					c = GeoTIFFConstants::unexploredDark;
				painter.fillRect(QRect(x, y, checkerBoardPixelWidth, checkerBoardPixelHeight),
						QColor((int)c.r, (int)c.g, (int)c.b, 255));
			}
		}

		// paint mapped areas
		int gridWidth = (int)round(GeoTIFFConstants::exploredGridSize / map->info.resolution),
				gridHeight = (int)round(GeoTIFFConstants::exploredGridSize / map->info.resolution);

		QColor occupied((int)GeoTIFFConstants::wallAndObstacleColour.r,
				(int)GeoTIFFConstants::wallAndObstacleColour.g,
				(int)GeoTIFFConstants::wallAndObstacleColour.b);
		QColor grid((int)GeoTIFFConstants::exploredGrid.r,
				(int)GeoTIFFConstants::exploredGrid.g,
				(int)GeoTIFFConstants::exploredGrid.b);

		QColor clearedMin((int)GeoTIFFConstants::clearedAreaLowConfidence.r,
				(int)GeoTIFFConstants::clearedAreaLowConfidence.g,
				(int)GeoTIFFConstants::clearedAreaLowConfidence.b);
		QColor clearedMax((int)GeoTIFFConstants::clearedAreaHighConfidence.r,
				(int)GeoTIFFConstants::clearedAreaHighConfidence.g,
				(int)GeoTIFFConstants::clearedAreaHighConfidence.b);
		Colour4f clearedDiff(GeoTIFFConstants::clearedAreaHighConfidence.r - GeoTIFFConstants::clearedAreaLowConfidence.r,
				GeoTIFFConstants::clearedAreaHighConfidence.g - GeoTIFFConstants::clearedAreaLowConfidence.g,
				GeoTIFFConstants::clearedAreaHighConfidence.b - GeoTIFFConstants::clearedAreaLowConfidence.b);

		QColor freeMin((int)GeoTIFFConstants::searchedAreaLowConfidence.r,
				(int)GeoTIFFConstants::searchedAreaLowConfidence.g,
				(int)GeoTIFFConstants::searchedAreaLowConfidence.b);
		QColor freeMax((int)GeoTIFFConstants::searchedAreaHighConfidence.r,
				(int)GeoTIFFConstants::searchedAreaHighConfidence.g,
				(int)GeoTIFFConstants::searchedAreaHighConfidence.b);
		Colour4f freeDiff(GeoTIFFConstants::searchedAreaHighConfidence.r - GeoTIFFConstants::searchedAreaLowConfidence.r,
				GeoTIFFConstants::searchedAreaHighConfidence.g - GeoTIFFConstants::searchedAreaLowConfidence.g,
				GeoTIFFConstants::searchedAreaHighConfidence.b - GeoTIFFConstants::searchedAreaLowConfidence.b);

		for (uint32_t y = 0; y < imageHeight; ++y) {
			for (uint32_t x = 0; x < imageWidth; ++x) {
				QRect cell(x, y, 1, 1);

				const int8_t& v = map->data[y * imageWidth + x];
				if (v > 50) {
					painter.fillRect(cell, occupied);
				} else if (v != -1) {
					float conf = v / (float)100.0;
					int r = freeMin.red() + (int)(conf * freeDiff.r),
							g = freeMin.green() + (int)(conf * freeDiff.g),
							b = freeMin.blue() + (int)(conf * freeDiff.b);
					QColor c(r, g, b);
					painter.fillRect(cell, c);
				}
			}
		}

		float minVisX = map->info.origin.position.x;
		float minVisY = map->info.origin.position.y;
//	//	float maxVisX = hmm->offsetX + (maxHMX+1) * (hmm->patchColumns * map->info.resolution);
//		float maxVisY = hmm->offsetY + (maxHMY+1) * (hmm->parameters.patchRows * map->info.resolution);

		// paint robot path
		if (history != NULL && history->poses.size() > 0) {
			float penSize = GeoTIFFConstants::robotPathThickness / map->info.resolution;
			if (penSize < 1)
				penSize = 1;
			QPen penPath(QColor((int)GeoTIFFConstants::robotPathColour.r,
					(int)GeoTIFFConstants::robotPathColour.g,
							(int)GeoTIFFConstants::robotPathColour.b));
			penPath.setWidthF(penSize);
			painter.setPen(penPath);
			Pose startPose = history->poses[0];
			// get pixel for xy coord.
			Pose p = startPose;

			QPoint start((p.position.x - minVisX) / map->info.resolution, (p.position.y - minVisY) / map->info.resolution);

			for (unsigned int i = 1; i < history->poses.size(); i++) {
				p = history->poses[i];
				QPoint end((p.position.x - minVisX) / map->info.resolution, (p.position.y - minVisY) / map->info.resolution);
				painter.drawLine(start, end);
				start = end;
			}

			QPen penRobot(QColor((int)GeoTIFFConstants::initialPositionColour.r,
					(int)GeoTIFFConstants::initialPositionColour.g,
					(int)GeoTIFFConstants::initialPositionColour.b));
			penRobot.brush().setStyle(Qt::SolidPattern);
			painter.setPen(penRobot);
			painter.setBrush(penRobot.brush());
			float robotSizePixels = GeoTIFFConstants::robotPositionSize/map->info.resolution;

			{{
					// paint initial robot positions
					Pose p = startPose;
					double yaw, pitch, roll;
					p.getYPR(yaw, pitch, roll);

					float cX = (p.position.x - minVisX) / map->info.resolution,
							cY = (p.position.y - minVisY) / map->info.resolution;
					float cosYawRobotSize = cos(yaw)*robotSizePixels,
							sinYawRobotSize = sin(yaw)*robotSizePixels;

					QPointF polygon[4];
					// front
					polygon[0] = QPointF(cX + cosYawRobotSize, cY + sinYawRobotSize);
					// right
					polygon[1] = QPointF(cX - (sinYawRobotSize + cosYawRobotSize),
							cY - (sinYawRobotSize + cosYawRobotSize));
					// back
					polygon[2] = QPointF(cX - cosYawRobotSize/2, cY - sinYawRobotSize/2);
					// left
					polygon[3] = QPointF(cX - (sinYawRobotSize + cosYawRobotSize),
							cY + (sinYawRobotSize + cosYawRobotSize));

					painter.drawPolygon(polygon, 4);
			}}
		}

		// The painted image is upside down so mirror it
		QImage* rval = new QImage();
	//	QImage correctGeotiffImage = geotiffImage.mirrored(false, true);
	//	QPainter textPainter(&correctGeotiffImage);
		*rval = geotiffImage.mirrored(false, true);

		QPainter textPainter(rval);

		// paint victim/hazard labels
		QFontMetrics fm = textPainter.fontMetrics();
		int minLabelSize = fm.height() - fm.descent();

		QPen penHazard(QColor((int)GeoTIFFConstants::hazardColour.r,
				(int)GeoTIFFConstants::hazardColour.g,
				(int)GeoTIFFConstants::hazardColour.b));
		penHazard.brush().setStyle(Qt::SolidPattern);
		QPen penVictim(QColor((int)GeoTIFFConstants::victimColour.r,
				(int)GeoTIFFConstants::victimColour.g,
				(int)GeoTIFFConstants::victimColour.b));
		penVictim.brush().setStyle(Qt::SolidPattern);
		QPen penVictimUnconfirmed(QColor((int)GeoTIFFConstants::victimUnconfirmedColour.r,
				(int)GeoTIFFConstants::victimUnconfirmedColour.g,
				(int)GeoTIFFConstants::victimUnconfirmedColour.b));
		penVictimUnconfirmed.brush().setStyle(Qt::SolidPattern);

		float victimRadius = GeoTIFFConstants::victimMarkerDiameter / map->info.resolution /2;
		if (victimRadius*2 < minLabelSize) {
			victimRadius = (minLabelSize + 1) / 2;
		}
		float hazardXYSize = sqrt(GeoTIFFConstants::hazardMarkerSide*GeoTIFFConstants::hazardMarkerSide/2)/map->info.resolution;
		if (hazardXYSize*2 < minLabelSize) {
			hazardXYSize = (minLabelSize + 1) / 2;
		}

		int victimCount = 0, hazardCount = 0;
		std::string victimDetails, hazardDetails;
		for (unsigned int i = 0; i < snaps.size(); i++) {
			SnapPtr snap = new Snap(snaps[i]);
			if (snap == NULL)
				continue;
			if (snap->status == Snap::CONFIRMED || snap->status == Snap::UNCONFIRMED) {
				Pose targetPose = snap->robot.toTF() * snap->pose.toTF();

				float cX = (targetPose.position.x - minVisX) / map->info.resolution,
						cY = (targetPose.position.y - minVisY) / map->info.resolution;
				cY = geotiffImage.height() - cY;

				char iStr[64];
				if (snap->type == Snap::VICTIM) {
					sprintf(iStr, "%d", ++victimCount);
					if (snap->status == Snap::CONFIRMED) {
						textPainter.setPen(penVictim);
						textPainter.setBrush(penVictim.brush());
					} else {
						textPainter.setPen(penVictimUnconfirmed);
						textPainter.setBrush(penVictimUnconfirmed.brush());
					}

					victimDetails += std::string(iStr) + ", " + targetPose.position.toString() + ", " + snap->description + "\n";

					textPainter.drawEllipse(QPointF(cX, cY), victimRadius,victimRadius);
				} else if (snap->type == Snap::LANDMARK){
					sprintf(iStr, "%d", ++hazardCount);

					textPainter.setPen(penHazard);
					textPainter.setBrush(penHazard.brush());
					QPointF diamond[4];
					diamond[0] = QPointF(cX, cY + hazardXYSize);
					diamond[1] = QPointF(cX + hazardXYSize, cY);
					diamond[2] = QPointF(cX, cY - hazardXYSize);
					diamond[3] = QPointF(cX - hazardXYSize, cY);

					hazardDetails += std::string(iStr) + ", " + targetPose.position.toString() + ", " + snap->description + "\n";

					textPainter.drawPolygon(diamond, 4);
				}

				textPainter.setPen(QColor(255,255,255));
				float dX = fm.width(QString(iStr)) / 2.0, dY = fm.height() / 2.0 - fm.descent();

				textPainter.drawText(QPointF(cX - dX, cY + dY), QString(iStr));
			}
		}
		if (victimCount > 0)
			snapDetails += "Victims:\n" + victimDetails;
		if (hazardCount > 0)
			snapDetails += "QR Codes:\n" + hazardDetails;

		// Scale
		textPainter.setPen(QColor((int)GeoTIFFConstants::scaleColour.r,
				(int)GeoTIFFConstants::scaleColour.g,
				(int)GeoTIFFConstants::scaleColour.b));
		int endCapLen = 6;
		QPoint scaleTop(checkerBoardPixelWidth/2, checkerBoardPixelHeight);
		QPoint scaleBottom(scaleTop.x(), scaleTop.y()+checkerBoardPixelHeight);
		textPainter.drawLine(scaleTop, scaleBottom);
		textPainter.drawLine(QPoint(scaleTop.x() - endCapLen / 2,  scaleTop.y()),
				QPoint(scaleTop.x() + endCapLen / 2,  scaleTop.y()));
		textPainter.drawLine(QPoint(scaleBottom.x() - endCapLen / 2,  scaleBottom.y()),
				QPoint(scaleBottom.x() + endCapLen / 2,  scaleBottom.y()));
		QPoint scaleText(scaleTop.x() + 5,
				(scaleTop.y() + scaleBottom.y()) / 2 + (fm.height() / 2 - fm.descent()));
		textPainter.drawText(scaleText, "1m");

		// Orientation
		// MM: The robocup rescue instructions call for the x axis to go up and y axis to go left.
		//     We accomplish this by swapping the x and y values and then negating y.
		textPainter.setPen(QColor((int)GeoTIFFConstants::orientationColour.r,
				(int)GeoTIFFConstants::orientationColour.g,
				(int)GeoTIFFConstants::orientationColour.b));
		QPoint orientOrigin(checkerBoardPixelWidth*3, checkerBoardPixelHeight*2);
		QPoint xEnd(orientOrigin.x(), orientOrigin.y()-checkerBoardPixelHeight);
		QPoint yEnd(orientOrigin.x()+checkerBoardPixelWidth, orientOrigin.y());
		textPainter.drawLine(orientOrigin, xEnd);
		textPainter.drawLine(orientOrigin, yEnd);
		textPainter.drawLine(xEnd, QPoint(xEnd.x() - 2, xEnd.y() + 2));
		textPainter.drawLine(xEnd, QPoint(xEnd.x() + 2, xEnd.y() + 2));
		textPainter.drawLine(yEnd, QPoint(yEnd.x() - 2, yEnd.y() - 2));
		textPainter.drawLine(yEnd, QPoint(yEnd.x() - 2, yEnd.y() + 2));
		int dY = (fm.height() - fm.descent() -fm.ascent()) / 2;
		int dX = fm.width("y") / 2;
//		QPoint yText(orientOrigin.x() + 5, (orientOrigin.y() + xEnd.y()) / 2 + dY);
		QPoint yText((orientOrigin.x() + yEnd.x()) / 2 - dX - 2, orientOrigin.y() + fm.height() - fm.descent());
//		QPoint xText((orientOrigin.x() + yEnd.x()) / 2 - dX, orientOrigin.y() + fm.height() - fm.descent());
		QPoint xText(orientOrigin.x() - 12, (orientOrigin.y() + xEnd.y()) / 2 + dY + 2);
		textPainter.drawText(xText, "y");
		textPainter.drawText(yText, "x");


		if (title == "") {
			textPainter.setPen(QColor((int)GeoTIFFConstants::textColour.r,
					(int)GeoTIFFConstants::textColour.g,
					(int)GeoTIFFConstants::textColour.b));
			QFontMetrics fm = textPainter.fontMetrics();
			textPainter.drawText(checkerBoardPixelWidth, checkerBoardPixelHeight - fm.descent(), QString(title.c_str()));
		}

		char data[64*6];
		geoData = "";
		sprintf(data, "%f\n", map->info.resolution);	// x resolution (meters/pixel)
		geoData += data;
		sprintf(data, "%f\n", 0.0);			// translation
		geoData += data;
		sprintf(data, "%f\n", 0.0);			// rotation
		geoData += data;
		sprintf(data, "%f\n", map->info.resolution);	// -y resolution (meters/pixel)
		geoData += data;
		sprintf(data, "%f\n", minVisX);	// upper left cell
		geoData += data;
		sprintf(data, "%f\n", minVisY + (imageHeight * map->info.resolution));	// upper left cell
		geoData += data;

		return rval;
	}

	void saveMap() {
		nav_msgs::OccupancyGridConstPtr currentMap;
		nav_msgs::PathConstPtr currentHistory;

		{{
			Lock lock(mapLock);
			currentMap = latestMap;
			currentHistory = latestHistory;
		}}

		if (currentMap == NULL)
			return;

		Time current = crosbot::Time::now();
		std::string filename = fileLoc + current.formatDateAndTime();

		LOG("Saving to %s.\n", filename.c_str());

		// TODO: Get snaps
		crosbot_map::ListSnaps::Request snapsReq;
		crosbot_map::ListSnaps::Response snapsRes;
		try {
			if (!snapSrv.call(snapsReq, snapsRes)) {
				snapsRes.snaps.clear();
			}
		} catch (...) {
			ERROR("geo_logger: Unable to get snap list.\n");
			snapsRes.snaps.clear();
		}

		LOG("Have snaps list.\n", filename.c_str());
		// Paint GeoTiff
		std::string geoData, snapData; Point2D minXY;
		QImage *geotiff = getGeoTiffImage(currentMap, "", currentHistory, snapsRes.snaps, geoData, snapData);

		// Write geotiff
//		QString geoData; Point2D minXY;
//			QImage* geotiff = mapView->render->getGeoTIFF(QString(title.c_str()), geoData, minXY);
			if (geotiff == NULL) {
				ERROR("geo_logger: Problem Rendering GeoTIFF.\n");
//				QMessageBox::critical(this, "Problem Saving GeoTIFF", "Unable to render GeoTIFF from map.");
				return;
			}

			LOG("Have geotiff image list.\n", filename.c_str());
			// save file// write image to file
			std::string rawFileName = filename + ".tiff";
			if (!geotiff->save(QString(rawFileName.c_str()), "TIFF")) {
				char eMsg[rawFileName.size() + 128];

				sprintf(eMsg, "Could not save raw TIFF to file %s.", rawFileName.c_str());
				ERROR("geo_logger: Problem saving map to TIFF. (file %s)\n", rawFileName.c_str());
//				QMessageBox::critical(this, "Problem Saving GeoTIFF", QString(eMsg));
				delete geotiff;
				return;
			}
			// GeoTiff saved so we delete it.
			delete geotiff;

			// write the world file
			std::string worldFileName = filename + ".tfw";
//			if (worldFileName.size() > 5 && strcasecmp(worldFileName.substr(worldFileName.size()-4).c_str(), ".tif") == 0) {
//				worldFileName = worldFileName.substr(0, worldFileName.size()-4);
//			} else if (worldFileName.size() > 6 && strcasecmp(worldFileName.substr(worldFileName.size()-5).c_str(), ".tiff") == 0) {
//				worldFileName = worldFileName.substr(0, worldFileName.size()-5);
//			}
//			worldFileName += ".tfw";

			FILE *worldFile = fopen(worldFileName.c_str(), "w");
			if (worldFile == NULL) {
				ERROR("geo_logger: Problem opening GeoTIFF world file. (file %s)\n", worldFileName.c_str());
//				QMessageBox::critical(this, "Problem Saving GeoTIFF", "Problem opening GeoTIFF world file.");
				return;
			}

			fprintf(worldFile, "%s\n", geoData.c_str());

			fflush(worldFile);
			fclose(worldFile);

			// Combine tif and worldfile into a geotiff file

			// need to use an external process to create geotiff from tiff and esri file
			// if the image isn't a tif nothing happens
			QProcess qproc;
			char cmdtxt[filename.size() + rawFileName.size() + worldFileName.size() + 128];
			sprintf(cmdtxt, "geotifcp -e %s %s %s", worldFileName.c_str(), rawFileName.c_str(), (filename + ".tif").c_str());
			LOG("%s\n", cmdtxt);

			qproc.start(cmdtxt);
			// if geotifcp ever stops returning disable this
			qproc.waitForFinished();
			if (snapData != "") {
				FILE* file = fopen((filename + ".csv").c_str(), "w");
				if (file != NULL) {
					fprintf(file, "%s", snapData.c_str());
				}
			}
	}

	void run() {
		while (operating) {
			if (usleep(10000000) == -1) {
//				LOG("Hello\n");
				continue;
			}

			saveMap();
		}

		saveMap();
	}

};

//QApplication app;
int main(int argc, char**argv) {
	ros::init(argc, argv, "geo_logger");

	GeoLogger geo;
	geo.start();

	QApplication app( argc, argv );
	app.setApplicationName(QString("gui"));
//	app.exec();
	while (ros::ok()) {
		ros::spin();
	}

	geo.operating = false;

	return 0;
}
