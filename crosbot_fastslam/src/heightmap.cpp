/*
 * heightmultimap.cpp
 *
 *  Created on: 09/09/2009
 *      Author: rescue
 */
#include <crosbot_fastslam/heightmap.hpp>

namespace crosbot {

namespace fastslam {

double HeightMultiMap::rayTrace(const Point& orig, const Point& dest) {
	double x = orig.x, y = orig.y, z = orig.z,
			dx, dy, dz, maxRay, ray;

	dx = dest.x - orig.x;
	dy = dest.y - orig.y;
	dz = dest.z - orig.z;

	maxRay = sqrt(dx*dx + dy*dy + dz+dz);

	ray = maxRay / RAY_STEP_SIZE;
	dx = dx / ray;
	dy = dy / ray;
	dz = dz / ray;

	maxRay += RAY_TRACE_EXTEND; // Look up to 5 meters beyond end of ray for the wall
//	maxRay = 6; // FIXME : this is AHMs max

	ray = 0;

	HeightMapPtr currentMap;
	while (ray < maxRay) {
		// Get the patch the ray ends in
		currentMap = getMapByXY(x, y);
		if (currentMap != NULL) {
			double xOrig = currentMap->xOrig,
					yOrig = currentMap->yOrig,
					maxX = xOrig + currentMap->width,
					maxY = yOrig + currentMap->height;
			do {
				// While still within the patch check if we have hit an obstruction
				HeightVal* hVal = currentMap->getByXY(x, y);
				if (hVal != NULL && hVal->occupied(z)) {
					return ray;
				}

				x += dx; y += dy; z += dz; ray += RAY_STEP_SIZE;
			} while (x >= xOrig && y >= yOrig && x < maxX && y < maxY);
		} else {
//			LOG("Unable to locate map for point (%.3lf, %.3lf) in raytrace.\n",
//					x, y);
			x += dx; y += dy; z += dz; ray += RAY_STEP_SIZE;
		}
	}

	// XXX: Is there a better value for this?
	return maxRay + 100;
}

void HeightMultiMap::updateTrace(const Point& orig, const Point& dest, double maxSensorRange, const SearchConstraints& search) {
	double x = orig.x, y = orig.y, z = orig.z,
			dx, dy, dz, maxRay, ray;

	dx = dest.x - orig.x;
	dy = dest.y - orig.y;
	dz = dest.z - orig.z;

	maxRay = sqrt(dx*dx + dy*dy + dz+dz);

	ray = maxRay / RAY_STEP_SIZE;
	dx = dx / ray;
	dy = dy / ray;
	dz = dz / ray;

	ray = 0;

//	LOG("HeightMultiMap::updateTrace(): maxSensorRange=%lf\n", maxSensorRange);

	HeightMapPtr endPatch = getMapByXY(dest.x, dest.y, true, true);
	HeightVal* endCell = NULL;
	if (maxRay <= maxSensorRange && endPatch != NULL) {
//		LOG("HMM: marking sensor hit.\n");
		endCell = endPatch->getByXY(dest.x, dest.y);
		if (endCell != NULL && dest.z >= parameters.minHeight &&
				dest.z <= parameters.maxHeight) {
			endCell->hit(dest.z);
		}
//	} else if (endPatch != NULL) {
//		LOG("Max ray out of sensor range.\n");
//	} else {
//		LOG("HMM: Unable to get map for sensor hit at (%.3lf, %.3lf, %.3lf)\n",
//				dest.x, dest.y, dest.z);
	}

	HeightMapPtr currentMap;
	while (ray < maxRay && ray < maxSensorRange && z >= parameters.minHeight && z <= parameters.maxHeight) {
		// Get the patch the ray ends in
		currentMap = getMapByXY(x, y, true, true);
		if (currentMap != NULL) {{
//			Lock lock(currentMap->rwLock);

			double xOrig = currentMap->xOrig,
					yOrig = currentMap->yOrig,
					maxX = xOrig + currentMap->width,
					maxY = yOrig + currentMap->height;
			HeightVal* prevCell = NULL;
			do {
				HeightVal* cell = currentMap->getByXY(x, y);

				if (cell != NULL && cell != prevCell && cell != endCell) {
					// Update cell occupancy probabilities
					cell->miss(z);

					// Check if cell in search area
					if (search.maxDist > 0) {
						double sdx = x - search.sensor.position.x,
								sdy = y - search.sensor.position.y,
								d = sqrt(sdx*sdx + sdy*sdy);
						if (d <= search.maxDist) {
							double angle = atan2(sdy, sdx) - search.sensor.orientation;
							NORMALISE_ANGLE(angle);
							if (fabs(angle) <= search.fov) {
								cell->searches++;
							}
						}
					}
//				} else if (cell == NULL) {
//					ERROR("HMM: Point (%lf, %lf) not in HM but within (%lf<>%lf,%lf<>%lf)\n",
//							x, y, xOrig, maxX, yOrig, maxY);
				}
				prevCell = cell;

				x += dx; y += dy; z += dz; ray += RAY_STEP_SIZE;
			} while (x >= xOrig && y >= yOrig && x < maxX && y < maxY && ray < maxRay &&
					z >= parameters.minHeight && z <= parameters.maxHeight);
		}} else {
			LOG("HMM: Unable to get map for point (%.3lf, %.3lf, %.3lf) in updatetrace.\n", x, y, z);
			x += dx; y += dy; z += dz; ray += RAY_STEP_SIZE;
		}
	}
}

#define HMM_FACTOR_RES(RES, D)		(10.0 * ProbabilityTable.getProbOf(D * 1000.0, RES * 1000.0, gain))
double HeightMultiMap::getProb(MapCloudPtr cloud, double gain, double maxSensorRange) {
	unsigned int i;
	double res;
	double prob = 1.0;
	double tmp;
	double dx, dy, dz, d;

	Pose sensorPose = cloud->getAbsoluteSensorPose();
	if (!sensorPose.isFinite()) {
		ERROR("HeightMultiMap::getProb() - Invalid sensor pose.\n");
		return 0;
	}

//	float x, y;
	// laser points
	for (i = 0; i < cloud->cloud.size(); i++) {
		const Point& point = cloud->cloud[i];
		if (!point.isFinite() || point.withinError(sensorPose.position))
			continue;

		dx = point.x - sensorPose.position.x;
		dy = point.y - sensorPose.position.y;
		dz = point.z - sensorPose.position.z;
		d = sqrt(dx*dx + dy*dy + dz*dz); // distance of sensor ray
		if (d > maxSensorRange)
			continue;

		res = rayTrace(sensorPose.position, point);

		// need a factor to prevent underflow
		tmp = HMM_FACTOR_RES(res, d);
		prob = prob * tmp;
	}

	return prob;
}

void HeightMultiMap::update(MapCloudPtr cloud, double maxSensorRange, const SearchConstraints& search) {
	unsigned int i;

	Pose sensorPose = cloud->getAbsoluteSensorPose();
	if (!sensorPose.isFinite()) {
		ERROR("HeightMultiMap::update() - Invalid sensor pose.\n");
		return;
	}

	for (i = 0; i < cloud->cloud.size(); i++) {
		const Point& point = cloud->cloud[i];
		if (!point.isFinite() || point.withinError(sensorPose.position))
			continue;
		updateTrace(sensorPose.position, point, maxSensorRange, search);
	}
}

} // namespace fastslam

} // namespace crosbot
