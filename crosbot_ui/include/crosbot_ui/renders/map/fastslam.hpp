/*
 * fastslam.h
 *
 *  This should be part of the casros_fastslam package but would create a circular dependency.
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_RENDER_FASTSLAM_HPP_
#define CROSBOT_RENDER_FASTSLAM_HPP_

#include <crosbot_ui/renders/map/map.hpp>
#include <crosbot_fastslam/fastslam.hpp>

namespace crosbot {

namespace fastslam {
using namespace crosbot::gui;

class FastSLAMRender : public gui::MapRender {
protected:
	FastSLAMMapPtr map;
	int selectedMap;
	Point selectedPoint;

	unsigned int geoTIFFTexture;

	ParticlePtr getSelectedParticle();
public:
	FastSLAMRender(FastSLAMMapPtr map) : MapRender(map), map(map), selectedMap(-1),
		selectedPoint(NAN, NAN, NAN), geoTIFFTexture(0) {}
	~FastSLAMRender() {}

	void setSelectedMap(int idx) { selectedMap = idx; selectedPoint = Point(NAN, NAN, NAN); }
	void setSelectedMap(Point p) { selectedPoint = p; }
	int getMapCount();

//	void mapUpdated(MapPtr map);
//	void tagAdded(MapPtr map, TagPtr tag);

	void compileGLLists() {}
	void renderMap(GLTextQueue& textQueue);
	void renderRobots(GLTextQueue& textQueue);
	void renderTags(GLTextQueue& textQueue);
	void renderPointClouds(GLTextQueue& textQueue);

	void renderGeoTIFF(bool showPath = true);
	QImage* getGeoTIFF(QString title, QString& fileData, Point2D& minXY, bool showPath = true, float* cellSize=NULL, Pose3D* currentPose=NULL, Pose3D* trackerPose=NULL);

	bool saveMap(std::string filename);

	/*
	 * Used to create a version of the map which can be drawn by opengl
	 * Stores the occupancy state of the cell's neighbours (all 8 of them)
	 * This lets the drawing function draw only the outer sides and edges.
	 * */
	struct Point3Dnbr {
		double x, y, z; // position and height
		bool xp, xm, yp, ym; // x and y positive and negative states (xp = x+1, xm = x-1, ...)
		bool xpyp, xmyp, xpym, xmym; // corner neighbors (xpyp = x+1, y+1, ...)
		bool featurePoint; // used for feature detection to mark the cell as containing a feature.
		float xpz, xmz, ypz, ymz; // x and y positive and negative states (xp = x+1, xm = x-1, ...)
		float xpypz, xmypz, xpymz, xmymz; // corner neighbors (xpyp = x+1, y+1, ...)

		Point3Dnbr() {
			featurePoint = false;
		}
		Point3Dnbr(double x_, double y_, double z_, bool xp_, bool xm_, bool yp_, bool ym_) {
			x = x_; y = y_; z = z_;
			xp = xp_;
			xm = xm_;
			yp = yp_;
			ym = ym_;
			xpyp = false;
			xpym = false;
			xmyp = false;
			xmym = false;
			featurePoint = false;

			xpz = xmz = ypz = ymz = 0.0;
			xpypz = xmypz = xpymz = xmymz = 0.0;
		}
		Point3Dnbr(double x_, double y_, double z_) {
			x = x_; y = y_; z = z_;
			xp = false;
			xm = false;
			yp = false;
			ym = false;
			xpyp = false;
			xpym = false;
			xmyp = false;
			xmym = false;
			featurePoint = false;

			xpz = xmz = ypz = ymz = 0.0;
			xpypz = xmypz = xpymz = xmymz = 0.0;
		}
	};

	/*
	 * A drawable cloud of points.  Used internally by the map to save its graphical state and draw itself
	 */
	class DisplayCloud : public HandledObject {
	public:
		Pose3D origin;
		std::vector<Point3Dnbr> mapPoints;

		DisplayCloud() {}
		DisplayCloud(Pose3D pose) {
			origin = pose;
		}
	};
	typedef Handle<DisplayCloud> DisplayCloudPtr;

	DisplayCloudPtr getDisplayCloud(ParticlePtr part);
};

} // namespace fastslam

} // namespace crosbot

#endif /* CROSBOT_RENDER_FASTSLAM_HPP_ */
