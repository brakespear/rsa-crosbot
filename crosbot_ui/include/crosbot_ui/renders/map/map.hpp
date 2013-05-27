/*
 * maprender.h
 *
 *  Created on: 23/08/2010
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_MAP_HPP_
#define CROSBOT_RENDER_MAP_HPP_

#include <crosbot_map/map.hpp>
#include <crosbot_ui/opengl.hpp>

#include <QImage>

namespace crosbot {

namespace gui {

#define TEXT_COLOUR_DISTANCE	0.75

class MapRender;
typedef MapRender* (*MapRenderFactoryFunc)(MapPtr map);
class MapRender {
public:
	MapRender(MapPtr map) : map(map) {
		isSelected = false;
		robotColour = Colour4f(1,.5,.5,1);
		robotEdgeColour = Colour4f(0.9,.45,.45,1);
		confirmedVictimColour = Colour4f(1,0,0,1);
		unconfirmedVictimColour = Colour4f(1,1,0,1);
		landmarkColour = Colour4f(0,0,1,1);
		selectedEdgeColour = Colour4f(1,0,0,1);
	}

	virtual ~MapRender() {}

	/**
	 * If a map has multiple maps this is used to select them.
	 *
	 * -1 is the default map, -2 for motion model, 0+ for specific map
	 */
	virtual void setSelectedMap(int idx)=0;
	virtual void setSelectedMap(Point3D p)=0;
	virtual int getMapCount()=0;

	virtual void compileGLLists() {};
	virtual void renderMap(GLTextQueue& textQueue)=0;

	virtual void renderRobots(GLTextQueue& textQueue) {
		PathPtr path = map->getPath();
		size_t n = path->path.size();
		for (size_t i = 0; i < n; i++) {
			renderRobot(path->path[i]);
		}
	}

	virtual void renderTags(GLTextQueue& textQueue) {
		Map::TagListPtr tags = map->getTags();
		size_t n = tags->tags.size();
		for (size_t i = 0; i < n; i++) {
			renderTag(tags->tags[i].tag, tags->tags[i].mapPose, textQueue);
		}
	}

	virtual void renderPointClouds(GLTextQueue& textQueue) {}

	virtual void renderGeoTIFF(bool showPath = true)=0;
	virtual QImage* getGeoTIFF(QString title, QString& fileData, Point2D& minXY, bool showPath = true, float* cellSize=NULL, Pose3D* currentPose=NULL, Pose3D* trackerPose=NULL)=0;

	virtual bool saveMap(std::string filename)=0;

	virtual void renderRobot(const Pose& robotPose, float size = 1);
	virtual void renderTag(TagPtr tag, const Pose& tagPose, GLTextQueue& textQueue, float size = 1);
	virtual void renderCloud(PointCloudPtr cloud, const Pose& robotPose, GLTextQueue& textQueue);

	virtual void setSelected(bool selected) {
		this->isSelected = selected;
	}

	static inline Colour4f getTextColour(const Colour4f& c) {
		float r = (c.r < 0.4)?1 - TEXT_COLOUR_DISTANCE*(1-c.r):c.r*TEXT_COLOUR_DISTANCE;
		float g = (c.g < 0.4)?1 - TEXT_COLOUR_DISTANCE*(1-c.g):c.g*TEXT_COLOUR_DISTANCE;
		float b = (c.b < 0.4)?1 - TEXT_COLOUR_DISTANCE*(1-c.b):c.b*TEXT_COLOUR_DISTANCE;

		return Colour4f(r,g,b,1);
	}

	static inline Colour4f getNearColour(const Colour4f& c) {
		float r = (c.r < 0.05) ? c.r + 0.05: c.r - 0.05;
		float g = (c.g < 0.05) ? c.g + 0.05: c.g - 0.05;
		float b = (c.b < 0.05) ? c.b + 0.05: c.r - 0.05;

		return Colour4f(r,g,b,c.a);
	}

protected:
	bool isSelected;
	MapPtr map;

	Colour4f robotColour, robotEdgeColour;
	Colour4f confirmedVictimColour;
	Colour4f unconfirmedVictimColour;
	Colour4f landmarkColour;
	Colour4f selectedEdgeColour;

public:
	static void addRenderFactory(MapRenderFactoryFunc);
	static MapRender *getRender(MapPtr map);
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_RENDER_MAP_HPP_ */
