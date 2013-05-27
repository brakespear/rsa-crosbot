/*
 * fastslam.cpp
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#include <crosbot_ui/renders/map/fastslam.hpp>
#include <crosbot_ui/renders/map/geotiff.hpp>
#include <crosbot_fastslam/serialization.hpp>

#include <QtOpenGL/QGLWidget>

namespace crosbot {

namespace fastslam {

#define PARTICLE_SPECIAL	5
#define PARTICLE_NORMAL		2

// FIXME: This should be configurable
#define OCCUPIED_SLICE		0.5

FastSLAMRender::DisplayCloudPtr FastSLAMRender::getDisplayCloud(ParticlePtr part) {
	DisplayCloudPtr rval = new DisplayCloud();

	HeightMultiMapPtr hmm = part->map;

	if (hmm != NULL) {
		for (unsigned int i = 0; i < hmm->rows; i++) {
			for (unsigned int j = 0; j < hmm->columns; j++) {
				HeightMapPtr hm = hmm->patches[i][j];
				if (hm == NULL)
					continue;

				for (unsigned int y = 0; y < hm->rows; y++) {
					for (unsigned int x = 0; x < hm->columns; x++) {
						HeightVal *hv = hm->getIndex(y * hm->columns + x);

						if (hv && hv->occupied(OCCUPIED_SLICE)) {
							// Cell is occupied so draw it.
							Point3Dnbr pn = Point3Dnbr(hm->xOrig + x * CELL_WIDTH, hm->yOrig + y * CELL_HEIGHT, hv->height);

							// set the neighbour values
							// Note, can't look to adjacent HeightMaps
							if (x == 0) {
								pn.xm = false;
								pn.xmym = false;
								pn.xmyp = false;
							} else {
								hv = hm->getIndex(y * hm->columns + (x - 1));
								if (hv && hv->occupied(OCCUPIED_SLICE)) {
									pn.xm = true;
									pn.xmz = hv->height;
								}
								if (y > 0) {
									hv = hm->getIndex((y - 1) * hm->columns + (x - 1));
									if (hv && hv->occupied(OCCUPIED_SLICE)) {
										pn.xmym = true;
										pn.xmymz = hv->height;
									}
								}
								if (y < (hm->rows - 1)) {
									hv = hm->getIndex((y + 1) * hm->columns + (x - 1));
									if (hv && hv->occupied(OCCUPIED_SLICE)) {
										pn.xmyp = true;
										pn.xmypz = hv->height;
									}
								}
							}
							if (x == (hm->columns - 1)) {
								pn.xp = false;
								pn.xpym = false;
								pn.xpyp = false;
							} else {
								hv = hm->getIndex(y * hm->columns + (x + 1));
								if (hv && hv->occupied(OCCUPIED_SLICE)) {
									pn.xp = true;
									pn.xpz = hv->height;
								}
								if (y > 0) {
									hv = hm->getIndex((y - 1) * hm->columns + (x + 1));
									if (hv && hv->occupied(OCCUPIED_SLICE)) {
										pn.xpym = true;
										pn.xpymz = hv->height;
									}
								}
								if (y < (hm->rows - 1)) {
									hv = hm->getIndex((y + 1) * hm->columns + (x + 1));
									if (hv && hv->occupied(OCCUPIED_SLICE)) {
										pn.xpyp = true;
										pn.xpypz = hv->height;
									}
								}
							}
							if (y == 0) {
								pn.ym = false;
								pn.xpym = false;
								pn.xmym = false;
							} else {
								hv = hm->getIndex((y - 1) * hm->columns + x);
								if (hv && hv->occupied(OCCUPIED_SLICE)) {
									pn.ym = true;
									pn.ymz = hv->height;
								}
							}
							if (y == (hm->rows - 1)) {
								pn.yp = false;
								pn.xpyp = false;
								pn.xmyp = false;
							} else {
								hv = hm->getIndex((y + 1) * hm->columns + x);
								if (hv && hv->occupied(OCCUPIED_SLICE)) {
									pn.yp = true;
									pn.ypz = hv->height;
								}
							}
							rval->mapPoints.push_back(pn);
//						} else if (hv && hv->feature >= 0) {
//							// Always display features
//							Point3Dnbr pn = Point3Dnbr(hm->xOrig + x * CELL_WIDTH, hm->yOrig + y * CELL_HEIGHT, 0.1);
//							pn.featurePoint = true;
//							rval->mapPoints.push_back(pn);
						}
					}
				}
			}
		}
	}
	return rval;
}

ParticlePtr FastSLAMRender::getSelectedParticle() {
	if (map == NULL)
		return NULL;
	ParticlePtr rval;
	if (selectedPoint.isFinite()) {{
		Lock lock(map->particlesLock);
		if (map->particles.size() == 0)
			return NULL;
		ParticlePtr rval = map->particles[0];
		double nearestDistance = rval->pose.position.distanceTo(selectedPoint);
		for (size_t i = 1; i < map->particles.size(); i++) {
			ParticlePtr p = map->particles[i];

			double d = p->pose.position.distanceTo(selectedPoint);
			if (d < nearestDistance) {
				rval = p; nearestDistance = d;
			}
		}
		return rval;
	}} else if (selectedMap == -2) {
		rval = map->motion;
	} else if (selectedMap < 0) {
		rval = map->mean;
	} else {{
		Lock lock(map->particlesLock);
		if (selectedMap < (int)map->particles.size()) {
			rval = map->particles[selectedMap];
		} else {
			rval = map->mean;
		}
	}}

	if (rval == NULL)
		return NULL;
	// We return a copy of the selected particle
	return new Particle(rval);
}

int FastSLAMRender::getMapCount() {
	Lock lock(map->particlesLock);
	return map->particles.size();
}

#define MAX(X, Y)	((X)>(Y)?(X):(Y))
#define MIN(X, Y)	((X)<(Y)?(X):(Y))

Colour4f wallBlendColour = Colour4f(0, 0, 0, 0.2);
Colour4f wallColour = Colour4f(0.5, 0.5, 0.5, 1);
Colour4f edgeColour = Colour4f(0, 0, 0, 0.9);
Colour4f meanParticleColour = Colour4f(1, 0, 0, 1);
Colour4f motionParticleColour = Colour4f(0, 0, 1, 1);
Colour4f displayParticleColour = Colour4f(1, 1, 0, 1);
Colour4f otherParticleColour = Colour4f(0, 1, 0, .6);
void FastSLAMRender::renderMap(GLTextQueue& textQueue) {
	std::vector<Point> particleLocations;
	Point meanLocation, motionLocation;

    int foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE FastSLAM render. (%s)\n", glutGetGLError(foo).c_str());
    }
	ParticlePtr particle = getSelectedParticle();
	if (particle == NULL) {
		return;
	}
	{	Lock lock(map->particlesLock);
		for (size_t i = 0; i < map->particlePoses.size(); i++) {
			particleLocations.push_back(map->particlePoses[i].position);
		}
		if (map->mean == NULL) {
			meanLocation = Point(NAN, NAN, NAN);
		} else {{
			meanLocation = map->meanPose.position;
//			Lock lock(map->mean->rwLock);
//			meanLocation = map->mean->pose.position;
		}}
		if (map->motion == NULL) {
			motionLocation = Point(NAN, NAN, NAN);
		} else {{
			Lock lock(map->motion->rwLock);
			motionLocation = map->motion->pose.position;
		}}
	}

	double htmp;

	// the radius of a cell
	double wtmp = CELL_WIDTH / 2.0;

	// show the map

	// this is the old code
	/*glPushMatrix();
		PointCloud displayCloud = FastSLAMRender_getPointCloud(part);
		glPointSize(3.0);
		glBegin(GL_LINES);
			glColor4f(0.0, 0.0, 0.0, 0.1);

			for (unsigned int i = 0; i < displayCloud->laserPoints.size(); i++) {
				Point3D& p = displayCloud->laserPoints[i];
				glVertex3f(p.x, p.y, p.z);
				glVertex3f(p.x, p.y, 0);
			}
		glEnd();
	glPopMatrix();
	return;*/
	Colour4f wallColour = crosbot::fastslam::wallColour;
	if (glIsEnabled(GL_BLEND)) {
		wallColour = wallBlendColour;
	}
	glPushMatrix();
		DisplayCloudPtr displayCloud = getDisplayCloud(particle);
		glPointSize(3.0);
//		glBlendFunc(GL_SRC_ALPHA,GL_ONE);
//		glBlendFunc(GL_SRC_COLOR,GL_DST_COLOR);
		// XXX: AHM: someone who understands this function might choose better parameters
//		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		glEnable(GL_BLEND);
//		glDisable(GL_DEPTH_TEST);

		// Code for rendering of DisplayClouds
//		glColor4f(0.0, 0.0, 0.0, 0.1);
		glBegin(GL_QUADS);
//			for (unsigned int i = 0; i < displayCloud->laserPoints.size(); i++) {
//				Point3D& p = displayCloud->laserPoints[i];
			// for each occupied cell, draw the visible faces (where the neighbour is unoccupied)
			for (unsigned int i = 0; i < displayCloud->mapPoints.size(); i++) {
				Point3Dnbr p = displayCloud->mapPoints[i];
				// features are blue
				glColor4f(wallColour.r, wallColour.g, wallColour.b, wallColour.a);

				glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
				glVertex3f(p.x - wtmp, p.y + wtmp, p.z);

				glVertex3f(p.x + wtmp, p.y + wtmp, 0);
				glVertex3f(p.x + wtmp, p.y - wtmp, 0);
				glVertex3f(p.x - wtmp, p.y - wtmp, 0);
				glVertex3f(p.x - wtmp, p.y + wtmp, 0);

				if ((!p.ym) || (p.ymz < p.z)) {
					glVertex3f(p.x - wtmp, p.y - wtmp, p.ymz);
					glVertex3f(p.x + wtmp, p.y - wtmp, p.ymz);
					glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
					glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
				}

				if ((!p.xp) || (p.xpz < p.z)) {
					glVertex3f(p.x + wtmp, p.y - wtmp, p.xpz);
					glVertex3f(p.x + wtmp, p.y + wtmp, p.xpz);
					glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
					glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
				}

				if ((!p.yp) || (p.ypz < p.z)) {
					glVertex3f(p.x + wtmp, p.y + wtmp, p.ypz);
					glVertex3f(p.x - wtmp, p.y + wtmp, p.ypz);
					glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
					glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
				}

				if ((!p.xm) || (p.ymz < p.z)) {
					glVertex3f(p.x - wtmp, p.y + wtmp, p.xmz);
					glVertex3f(p.x - wtmp, p.y - wtmp, p.xmz);
					glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
					glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
				}
			}
		glEnd();

		glBegin(GL_LINES);
		// for each occupied cell, draw the wall's convex edges
		for (unsigned int i = 0; i < displayCloud->mapPoints.size(); i++) {
			Point3Dnbr& p = displayCloud->mapPoints[i];
			if (isSelected) {
				glColor4f(selectedEdgeColour.r,selectedEdgeColour.g,selectedEdgeColour.b,selectedEdgeColour.a);
			} else {
				glColor4f(edgeColour.r, edgeColour.g, edgeColour.b, edgeColour.a);
			}
/*
			glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
			glVertex3f(p.x + wtmp, p.y - wtmp, p.z);

			glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
			glVertex3f(p.x + wtmp, p.y + wtmp, p.z);

			glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
			glVertex3f(p.x - wtmp, p.y + wtmp, p.z);

			glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
			glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
*/
			// the four outside corners
			// oxxo
			// xxxx
			// oxxo
			htmp = MAX(MAX(p.xpypz, p.xpz), p.ypz);
//			if ((!p.xpyp && p.xp && p.yp) || (!p.xp && !p.yp)) {
			if (htmp < p.z) {
				glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y + wtmp, htmp);
			}
			htmp = MAX(MAX(p.xpymz, p.xpz), p.ymz);
//			if ((!p.xpym && p.xp && p.ym) || (!p.xp && !p.ym)) {
			if (htmp < p.z) {
				glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y - wtmp, htmp);
			}
			htmp = MAX(MAX(p.xmypz, p.xmz), p.ypz);
//			if ((!p.xmyp && p.xm && p.yp) || (!p.xm && !p.yp)) {
			if (htmp < p.z) {
				glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
				glVertex3f(p.x - wtmp, p.y + wtmp, htmp);
			}
			htmp = MAX(MAX(p.xmymz, p.xmz), p.ymz);
//			if ((!p.xmym && p.xm && p.ym) || (!p.xm && !p.ym)) {
			if (htmp < p.z) {
				glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x - wtmp, p.y - wtmp, htmp);
			}

			// outside edges along the top
			//       _____
			// oooo  |   |
			// oxxo  |   |
			// oooo  '---'
			if (!p.ym) {
				glVertex3f(p.x + wtmp, p.y - wtmp, 0);
				glVertex3f(p.x - wtmp, p.y - wtmp, 0);
			}
			if ((p.ymz < p.z - 0.02)) {
				glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
			}
			if (!p.xp) {
				glVertex3f(p.x + wtmp, p.y + wtmp, 0);
				glVertex3f(p.x + wtmp, p.y - wtmp, 0);
			}
			if ((p.xpz < p.z - 0.02)) {
				glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y - wtmp, p.z);
			}
			if (!p.yp) {
				glVertex3f(p.x - wtmp, p.y + wtmp, 0);
				glVertex3f(p.x + wtmp, p.y + wtmp, 0);
			}
			if ((p.ypz < p.z - 0.02)) {
				glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
				glVertex3f(p.x + wtmp, p.y + wtmp, p.z);
			}
			if (!p.xm) {
				glVertex3f(p.x - wtmp, p.y - wtmp, 0);
				glVertex3f(p.x - wtmp, p.y + wtmp, 0);
			}
			if ((p.xmz < p.z - 0.02)) {
				glVertex3f(p.x - wtmp, p.y - wtmp, p.z);
				glVertex3f(p.x - wtmp, p.y + wtmp, p.z);
			}
		}
		glEnd();

		glPointSize(1.0);

//		glDisable(GL_BLEND);
//		glEnable(GL_DEPTH_TEST);

	glPopMatrix();

	// Draw the positions of all particles
	glPushMatrix();
		// Display particles
		glPointSize(3.0);
		glBegin(GL_LINES);
			// The others are green
			glColor4f(otherParticleColour.r, otherParticleColour.g, otherParticleColour.b, otherParticleColour.a);
			for (unsigned int i = 0; i < particleLocations.size(); i++) {
				Point pLocation = particleLocations[i];
				if (pLocation != meanLocation && pLocation != motionLocation) {
					glVertex3f(pLocation.x, pLocation.y, PARTICLE_NORMAL + pLocation.z);
					glVertex3f(pLocation.x, pLocation.y, pLocation.z);
				}
			}

			if (meanLocation.isFinite()) {
				glColor4f(meanParticleColour.r, meanParticleColour.g, meanParticleColour.b, meanParticleColour.a);
				glVertex3f(meanLocation.x, meanLocation.y, PARTICLE_SPECIAL + meanLocation.z);
				glVertex3f(meanLocation.x, meanLocation.y, meanLocation.z);
			}
			if (motionLocation.isFinite()) {
				glColor4f(motionParticleColour.r, motionParticleColour.g, motionParticleColour.b, motionParticleColour.a);
				glVertex3f(motionLocation.x, motionLocation.y, PARTICLE_SPECIAL + motionLocation.z);
				glVertex3f(motionLocation.x, motionLocation.y, motionLocation.z);
			}

			if (selectedMap >= 0) {
				glColor4f(displayParticleColour.r, displayParticleColour.g, displayParticleColour.b, displayParticleColour.a);
				glVertex3f(particle->pose.position.x, particle->pose.position.y, PARTICLE_SPECIAL + particle->pose.position.z);
				glVertex3f(particle->pose.position.x, particle->pose.position.y, particle->pose.position.z);
			}
		glEnd();
		glPointSize(1.0);
	glPopMatrix();

    foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER FastSLAM render. (%s)\n", glutGetGLError(foo).c_str());
    }
}

void FastSLAMRender::renderRobots(GLTextQueue& textQueue) {
	ParticlePtr particle = getSelectedParticle();
	if (particle == NULL)
		return;
	{ Lock lock(particle->rwLock);
		for (size_t r = 0; r < particle->history.size(); r++) {
			renderRobot(particle->history[r].pose);
		}
	}
}

void FastSLAMRender::renderTags(GLTextQueue& textQueue) {
	ParticlePtr particle = getSelectedParticle();
	if (particle == NULL)
		return;
	Map::TagListPtr tags = particle->tags;
	for (size_t t = 0; t < tags->tags.size(); t++) {
		Map::TagLocation& tag = tags->tags[t];
		renderTag(tag.tag, tag.mapPose, textQueue);
	}
}

void FastSLAMRender::renderPointClouds(GLTextQueue& textQueue) {
	ParticlePtr particle = getSelectedParticle();
	if (particle == NULL)
		return;

//	for (size_t h = 0; h < particle->history.size(); h++) {
//		Particle::History& hist = particle->history[h];
//		if (hist.cloud != NULL)
//			renderCloud(hist.cloud, hist.pose, textQueue);
//	}

	Map::TagListPtr tags = particle->tags;
	for (size_t t = 0; t < tags->tags.size(); t++) {
		Map::TagLocation& tag = tags->tags[t];

		SnapPtr snap = dynamic_cast<Snap *>(tag.tag.get());
		if (snap == NULL)
			continue;

		btTransform correction = tag.robot.getTransform() *
				snap->robot.getTransform().inverse();

		Pose robotPose = correction * snap->robot.getTransform();
		if (snap->status != Snap::REJECTED && snap->status != Snap::DUPLICATE) {
			for (size_t c = 0; c < snap->clouds.size(); c++) {
				renderCloud(snap->clouds[c], robotPose, textQueue);
			}
		}
	}
}

QImage* FastSLAMRender::getGeoTIFF(QString title, QString& geoData, Point2D& minXY, bool showPath, float* cellSize, Pose* currentPose, Pose* trackerPose) {
	ParticlePtr part = getSelectedParticle();
	if (part == NULL) {
//		ERROR("FastSLAMRender: No map to create GeoTiff from.\n");
		minXY = Point2D();
		if (currentPose != NULL)
			*currentPose = Pose();
		if (trackerPose != NULL)
			*trackerPose = Pose();
		if (cellSize != NULL)
			*cellSize = 0;
		return NULL;
	}

	Lock lock(part->rwLock);

	if (currentPose != NULL)
		*currentPose = part->pose;
	if (trackerPose != NULL)
		*trackerPose = part->trackerPose;
	if (cellSize != NULL)
		*cellSize = CELL_WIDTH;



	HeightMultiMapPtr hmm = part->map;

	unsigned int minHMX = hmm->columns, maxHMX = 0, minHMY = hmm->rows, maxHMY = 0;

	minXY.x = INFINITY; minXY.y = INFINITY;

	// find which HeightMaps have data and need to be drawn
	for (unsigned int i = 0; i < hmm->rows; i++) {
		for (unsigned int j = 0; j < hmm->columns; j++) {
			HeightMapPtr hm = hmm->patches[i][j];
			if (hm != NULL) {
				if (i < minHMY)
					minHMY = i;
				if (i > maxHMY)
					maxHMY = i;
				if (j < minHMX)
					minHMX = j;
				if (j > maxHMX)
					maxHMX = j;

				if (hm->xOrig < minXY.x) {
					minXY.x = hm->xOrig;
				}
				if (hm->yOrig < minXY.y) {
					minXY.y = hm->yOrig;
				}
			}
		}
	}

	if (minHMX > maxHMX || minHMY > maxHMY) {
		ERROR("FastSLAMRender: No data in map to export to GeoTIFF.\n");
		minXY = Point2D();
		return NULL;
	}

//	Point2D originAHM;
//	originAHM.x = -(hmm->offx / CELL_WIDTH) - minHMX * hmm->subwidth;
//	originAHM.y = -(hmm->offy / CELL_WIDTH) - minHMY * hmm->subheight;
//	logger->log("FastSLAMRender:: o(%.4lf, %.4lf) o2(%.4lf, %.4lf)\n", minXY.x, minXY.y, originAHM.x, originAHM.y);

	// create image
	unsigned int imageWidth = (maxHMX + 1 - minHMX) * hmm->parameters.patchColumns,
			imageHeight = (maxHMY + 1 - minHMY) * hmm->parameters.patchRows;

	QImage geotiffImage(imageWidth, imageHeight, QImage::Format_RGB32);
	QPainter painter(&geotiffImage);

	// paint unexplored checker board
	int checkerBoardPixelWidth = (int)round(GeoTIFFConstants::unexploredCheckerBoardSize / CELL_WIDTH),
			checkerBoardPixelHeight = (int)round(GeoTIFFConstants::unexploredCheckerBoardSize / CELL_HEIGHT);

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
	int gridWidth = (int)round(GeoTIFFConstants::exploredGridSize / CELL_WIDTH),
			gridHeight = (int)round(GeoTIFFConstants::exploredGridSize / CELL_HEIGHT);

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
	for (unsigned int i = minHMY; i <= maxHMY; i++) {
		for (unsigned int j = minHMX; j <= maxHMX; j++) {
			HeightMapPtr hm = hmm->patches[i][j];
			if (hm == NULL)
				continue;
			int offX = (j - minHMX) * hmm->parameters.patchColumns,
					offY = (i - minHMY) * hmm->parameters.patchRows;
			for (unsigned int il = 0; il < hmm->parameters.patchRows; il++) {
				for (unsigned int jl = 0; jl < hmm->parameters.patchColumns; jl++) {
					HeightVal hv = *(hm->getIndex(il * hmm->parameters.patchColumns + jl));
					QRect cell(offX + jl, offY + il, 1, 1);
					if (hv.occupied(OCCUPIED_SLICE)) {
						// Cell is occupied.
						painter.fillRect(cell, occupied);
					} else if (hv.observations > 0) {
						if ((offX + jl) % gridWidth == 0 || (offY + il) % gridHeight == 0) {
							painter.fillRect(cell, grid);
						} else if (hv.searches >= GeoTIFFConstants::maxClearViews) {
							painter.fillRect(cell, clearedMax);
						} else if (hv.searches > 0) {
							float conf = hv.searches / (float)GeoTIFFConstants::maxClearViews;
							int r = clearedMin.red() + (int)(conf * clearedDiff.r),
									g = clearedMin.green() + (int)(conf * clearedDiff.g),
									b = clearedMin.blue() + (int)(conf * clearedDiff.b);
//							logger->log("FastSLAMMap: Cleared colour(%d, %d, %d)\n", r, g, b);
							r = MIN(255, MAX(0, r));
							g = MIN(255, MAX(0, g));
							b = MIN(255, MAX(0, b));
							QColor c(r, g, b);
							painter.fillRect(cell, c);
						} else {
							float conf = 1 - ProbabilityTable.invodds(hv.prob);
							int r = freeMin.red() + (int)(conf * freeDiff.r),
									g = freeMin.green() + (int)(conf * freeDiff.g),
									b = freeMin.blue() + (int)(conf * freeDiff.b);
							QColor c(r, g, b);
							painter.fillRect(cell, c);
						}
					}
				}
			}
		}
	}

	float minVisX = hmm->offsetX + minHMX * (hmm->parameters.patchColumns * CELL_WIDTH);
	float minVisY = hmm->offsetY + minHMY * (hmm->parameters.patchRows * CELL_HEIGHT);
//	float maxVisX = hmm->offsetX + (maxHMX+1) * (hmm->patchColumns * CELL_WIDTH);
	float maxVisY = hmm->offsetY + (maxHMY+1) * (hmm->parameters.patchRows * CELL_HEIGHT);

	// paint robot path
	if (showPath) {
		if (part->history.size() > 0) {
			float penSize = GeoTIFFConstants::robotPathThickness / CELL_WIDTH;
			if (penSize < 1)
				penSize = 1;
			QPen penPath(QColor((int)GeoTIFFConstants::robotPathColour.r,
					(int)GeoTIFFConstants::robotPathColour.g,
							(int)GeoTIFFConstants::robotPathColour.b));
			penPath.setWidthF(penSize);
			painter.setPen(penPath);
			Pose p = part->history[0].pose;
			// get pixel for xy coord.

			QPoint start((p.position.x - minVisX) / CELL_WIDTH, (p.position.y - minVisY) / CELL_HEIGHT);

			for (unsigned int i = 1; i < part->history.size(); i++) {
				p = part->history[i].pose;
				QPoint end((p.position.x - minVisX) / CELL_WIDTH, (p.position.y - minVisY) / CELL_HEIGHT);

				if (!part->history[i].restart) {
					painter.drawLine(start, end);
				}
				start = end;
			}

			QPen penRobot(QColor((int)GeoTIFFConstants::initialPositionColour.r,
					(int)GeoTIFFConstants::initialPositionColour.g,
					(int)GeoTIFFConstants::initialPositionColour.b));
			penRobot.brush().setStyle(Qt::SolidPattern);
			painter.setPen(penRobot);
			painter.setBrush(penRobot.brush());
			float robotSizePixels = GeoTIFFConstants::robotPositionSize/CELL_WIDTH;
			for (unsigned int i = 0; i < part->history.size(); i++) {
				if (part->history[i].restart) {
					// paint initial robot positions
					Pose p = part->history[i].pose;
					double yaw, pitch, roll;
					p.getYPR(yaw, pitch, roll);

					float cX = (p.position.x - minVisX) / CELL_WIDTH,
							cY = (p.position.y - minVisY) / CELL_HEIGHT;
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
				}
			}
		}
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

	float victimRadius = GeoTIFFConstants::victimMarkerDiameter / CELL_WIDTH /2;
	if (victimRadius*2 < minLabelSize) {
		victimRadius = (minLabelSize + 1) / 2;
	}
	float hazardXYSize = sqrt(GeoTIFFConstants::hazardMarkerSide*GeoTIFFConstants::hazardMarkerSide/2)/CELL_WIDTH;
	if (hazardXYSize*2 < minLabelSize) {
		hazardXYSize = (minLabelSize + 1) / 2;
	}

	int victimCount = 0, hazardCount = 0;
	for (unsigned int i = 0; i < part->tags->tags.size(); i++) {
		Map::TagLocation& tl = part->tags->tags[i];
		SnapPtr snap = dynamic_cast<Snap*>(tl.tag.get());
		if (snap == NULL)
			continue;
		if (snap->status == Snap::CONFIRMED || snap->status == Snap::UNCONFIRMED) {
			float cX = (tl.mapPose.position.x - minVisX) / CELL_WIDTH,
					cY = (tl.mapPose.position.y - minVisY) / CELL_HEIGHT;
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

				textPainter.drawEllipse(QPointF(cX, cY), victimRadius,victimRadius);
			} else {
				sprintf(iStr, "%d", ++hazardCount);

				textPainter.setPen(penHazard);
				textPainter.setBrush(penHazard.brush());
				QPointF diamond[4];
				diamond[0] = QPointF(cX, cY + hazardXYSize);
				diamond[1] = QPointF(cX + hazardXYSize, cY);
				diamond[2] = QPointF(cX, cY - hazardXYSize);
				diamond[3] = QPointF(cX - hazardXYSize, cY);

				textPainter.drawPolygon(diamond, 4);
			}

			textPainter.setPen(QColor(255,255,255));
			float dX = fm.width(QString(iStr)) / 2.0, dY = fm.height() / 2.0 - fm.descent();

			textPainter.drawText(QPointF(cX - dX, cY + dY), QString(iStr));
		}
	}

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
	QPoint yEnd(orientOrigin.x()-checkerBoardPixelWidth, orientOrigin.y());
	textPainter.drawLine(orientOrigin, xEnd);
	textPainter.drawLine(orientOrigin, yEnd);
	textPainter.drawLine(xEnd, QPoint(xEnd.x() - 2, xEnd.y() + 2));
	textPainter.drawLine(xEnd, QPoint(xEnd.x() + 2, xEnd.y() + 2));
	textPainter.drawLine(yEnd, QPoint(yEnd.x() + 2, yEnd.y() - 2));
	textPainter.drawLine(yEnd, QPoint(yEnd.x() + 2, yEnd.y() + 2));
	int dY = (fm.height() - fm.descent() -fm.ascent()) / 2;
	QPoint xText(orientOrigin.x() + 5, (orientOrigin.y() + xEnd.y()) / 2 + dY);
	int dX = fm.width("y") / 2;
	QPoint yText((orientOrigin.x() + yEnd.x()) / 2 - dX, orientOrigin.y() + fm.height() - fm.descent());
	textPainter.drawText(xText, "x");
	textPainter.drawText(yText, "y");

	if (title.compare(QString("")) != 0) {
		textPainter.setPen(QColor((int)GeoTIFFConstants::textColour.r,
				(int)GeoTIFFConstants::textColour.g,
				(int)GeoTIFFConstants::textColour.b));
		QFontMetrics fm = textPainter.fontMetrics();
		textPainter.drawText(checkerBoardPixelWidth, checkerBoardPixelHeight - fm.descent(), title);
	}

	char data[64*6];
	geoData = "";
	sprintf(data, "%f\n", CELL_HEIGHT);	// x resolution (meters/pixel)
	geoData += data;
	sprintf(data, "%f\n", 0.0);			// translation
	geoData += data;
	sprintf(data, "%f\n", 0.0);			// rotation
	geoData += data;
	sprintf(data, "%f\n", -CELL_WIDTH);	// -y resolution (meters/pixel)
	geoData += data;
	sprintf(data, "%f\n", maxVisY);	// upper left cell
	geoData += data;
	sprintf(data, "%f\n", -minVisX);	// upper left cell
	geoData += data;

	return rval;
}

void FastSLAMRender::renderGeoTIFF(bool showPath) {
	double scalevarx, scalevary;

	QString data;
	Point2D minXY;
	QImage* gtiff = getGeoTIFF(QString(""), data, minXY, showPath);
	if (gtiff == NULL) {
		return;
	}

	// OpenGL nonsense, convert the image
	QImage texImage = QGLWidget::convertToGLFormat(*gtiff);

//		glDrawPixels( t.width(), t.height(), GL_RGBA, GL_UNSIGNED_BYTE, t.bits());
	// need to scale the image properly so it matches the grid
	scalevarx = texImage.width() * CELL_WIDTH;
	scalevary = texImage.height() * CELL_HEIGHT;

	// turn the tiff into a texture
	if (geoTIFFTexture == 0) {
		glGenTextures(1, &geoTIFFTexture);
	}

	// The actual rendering of the GeoTIFF image.
	glPushMatrix();
//		glBlendFunc(GL_ONE_MINUS_SRC_ALPHA, GL_SRC_COLOR);
//		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//		glEnable(GL_BLEND);

		glBindTexture(GL_TEXTURE_2D, geoTIFFTexture);

		// set tiff as a texture
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB8, texImage.width(), texImage.height(), 0, GL_RGBA, GL_UNSIGNED_BYTE, texImage.bits());

		// Now the core rendering code.
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
						GL_LINEAR);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
						GL_LINEAR);
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
		glEnable(GL_TEXTURE_2D);
				glShadeModel(GL_FLAT);
				glColor4f(1, 1, 1, 1);
				glBegin(GL_QUADS);
					glTexCoord2f(0.0,0.0);
					glVertex3f(minXY.x,minXY.y, 0);
					glTexCoord2f(1.0,0.0);
					glVertex3f(minXY.x+scalevarx,minXY.y, 0);
					glTexCoord2f(1.0,1.0);
					glVertex3f(minXY.x+scalevarx,minXY.y+scalevary, 0);
					glTexCoord2f(0.0,1.0);
					glVertex3f(minXY.x,minXY.y+scalevary, 0);
				glEnd();
		glDisable(GL_TEXTURE_2D);

//		glDisable(GL_BLEND);
	glPopMatrix();

	delete gtiff;
}

bool FastSLAMRender::saveMap(std::string filename) {
	ParticlePtr particle = getSelectedParticle();
	if (particle == NULL)
		return false;
	LOG("FastSLAMRender::saveMap - Have selected particle.\n");
	try {
		crosbot::serialization::Serializer<Particle> serializer;
		crosbot::serialization::FileOutputStream fos(filename);

		LOG("FastSLAMRender::saveMap - Have opened output stream.\n");
		serializer.write(*particle, fos);
		LOG("FastSLAMRender::saveMap - Particle saved.\n");
	} catch (IOException& exc) {
		ERROR("FastSLAMRender::saveMap - %s\n", exc.what());
		return false;
	}

	return true;
}

} // namespace fastslam

} // namespace crosbot
