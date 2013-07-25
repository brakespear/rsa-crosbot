/*
 * explorer.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <crosbot_explore/explorer.hpp>
#include <crosbot/utils.hpp>

namespace crosbot {

#define DEFAULT_SLEEP_TIME		50000
#define DEFAULT_SLEEP_TIME_MAX	10000000

#define ACCEPTABLE_DRIFT_IN_WALL_ANGLE		DEG2RAD(10)

Explorer::Explorer() :
	planThread_(*this), driveThread_(*this),
	operating(true), paused(true), stopSent(false), searchWhilePaused(true)
{}

Explorer::~Explorer() { shutdown(); }

void Explorer::planThread() {
	while (operating) {
		if (!paused || searchWhilePaused) {
			VoronoiGridPtr voronoi = getLatestVoronoi();
			Pose robot = getLatestPose();
			if (voronoi != NULL && robot.isFinite()) {
				// search voronoi grid
				Pose foundTarget = findDriveTarget(*voronoi, robot);

				if (!paused) {
					drive.setDriveTarget(foundTarget);
				} else {
					drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
				}
			} else {
				drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
			}
		} else {
			drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY));
		}

		usleep(DEFAULT_SLEEP_TIME);
	}
}

void Explorer::driveThread() {
	while (operating) {
		Pose target = drive.getDriveTarget();
		Pose pose = getLatestPose();

		if (!paused && target.position.isFinite()) {
			// send motion
			Pose relative = pose.getTransform().inverse() * target.getTransform();
			driveTo(relative);
		} else if (!stopSent) {
			// send stop
			stopSent = stopMotors();
		}
		usleep(DEFAULT_SLEEP_TIME);
	}

	if (!stopSent) {
		// send stop
		stopSent = stopMotors();
	}
}

void Explorer::shutdown() {
	operating = false;

	uint64_t slept = 0;
	while (slept < DEFAULT_SLEEP_TIME_MAX && planThread_.isAlive() && driveThread_.isAlive()) {
		usleep(DEFAULT_SLEEP_TIME);
		slept += DEFAULT_SLEEP_TIME;
	}
}

Pose Explorer::findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot) {
	if (search.strategy == SearchParameters::WallFollow) {
		return findWallFollowTarget(voronoi, robot);
	} else if (search.strategy == SearchParameters::Waypoint) {
		return findWaypointTarget(voronoi, robot);
	}

	return Pose(INFINITY, INFINITY, INFINITY);
}

Pose Explorer::findWallFollowTarget(const VoronoiGrid& voronoi, const Pose& robot) {
	search.previousWallTarget = search.wallTarget;

	double Y, P, R;
	robot.orientation.getYPR(Y,P,R);

	double startAngle = (search.side==SearchParameters::Left)?search.searchAngle:-search.searchAngle;
	if (search.previousWallTarget.isFinite()) {
		double prevStartAngle = atan2(search.previousWallTarget.y - robot.position.y,
				search.previousWallTarget.x - robot.position.x) - Y;
		if (fabs(prevStartAngle - startAngle) <= ACCEPTABLE_DRIFT_IN_WALL_ANGLE) {
			startAngle = prevStartAngle;
		}
	}

	search.wallTarget = findWall(voronoi, robot, startAngle);

	if (!search.wallTarget.isFinite()) {
		WARN("Explorer::findWallFollowTarget(): No wall found.\n");
		return Pose(INFINITY, INFINITY, INFINITY);
	}
	double wallAngle = atan2(search.wallTarget.y - robot.position.y, search.wallTarget.x - robot.position.x);

	Index2D currentCell = findFirstSkeletonCell(voronoi, robot, wallAngle),
			previousCell(-1,-1);

	if (!CELL_IS_VALID(currentCell, voronoi)) {
		WARN("Explorer::findWallFollowTarget(): Unable to find a skeleton cell in range.\n");
		return Pose(INFINITY, INFINITY, INFINITY);
	}

	Index2D robotCell = voronoi.findCell(robot.position);
	uint32_t n = 0;
	for (; n < search.maxIterations; ++n) {
		Index2D nextCell = findNextSkeletonCell(voronoi, currentCell, previousCell);

		if (!CELL_IS_VALID(nextCell, voronoi))
			break;

		if (((nextCell.distanceTo(robotCell) * voronoi.resolution) > search.searchDistance) ||
				!cellIsDirectlyTraversible(voronoi, nextCell, robotCell))
			break;

		previousCell = currentCell;
		currentCell = nextCell;
	}

	if (n >= search.maxIterations) {
		WARN("Explorer::findWallFollowTarget(): Skeleton found is a probably a small closed loop.\n");
		return Pose(INFINITY, INFINITY, INFINITY);
	}

	return Pose(voronoi.getPosition(currentCell), Quaternion());
}

Point Explorer::findWall(const VoronoiGrid& voronoi, const Pose& robot, double startAngle) {
	double Y,P,R, cosRay, sinRay;
	robot.orientation.getYPR(Y,P,R);
	Index2D robotCell = voronoi.findCell(robot.position), cell;
	Index2D wallCell(-1,-1);

	for (double rayAngle = 0; rayAngle < 2*M_PIl && !CELL_IS_VALID(wallCell, voronoi); rayAngle += search.angleStep) {
		if (search.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (search.searchDistance / voronoi.resolution) &&
								!CELL_IS_VALID(wallCell, voronoi); ray += search.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Restricted) {
				wallCell = vCell.nearestWallCell;
			}
		}
	}

	for (double rayAngle = 0; rayAngle < 2*M_PIl && !CELL_IS_VALID(wallCell, voronoi); rayAngle += search.angleStep) {
		if (search.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (search.searchDistance / voronoi.resolution) &&
								!CELL_IS_VALID(wallCell, voronoi); ray += search.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
			} else if ((vCell.status & VoronoiGrid::VoronoiCell::Restricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Expansion)) {
				wallCell = vCell.nearestWallCell;
			}
		}
	}

	if (CELL_IS_VALID(wallCell, voronoi)) {
		return voronoi.getPosition(wallCell);
	}
	return Point(INFINITY, INFINITY, INFINITY);
}

Index2D Explorer::findFirstSkeletonCell(const VoronoiGrid& voronoi, const Pose& robot, double startAngle) {
	double cosRay, sinRay;
	search.skeleton1st = search.skeleton2nd = Index2D(-1,-1);
	search.skeletonSearch.clear();

	Index2D robotCell = voronoi.findCell(robot.position), cell;

	for (double rayAngle = 0; rayAngle < 2*M_PIl; rayAngle += search.angleStep) {
		if (search.side == SearchParameters::Left) {
			cosRay = cos(startAngle - rayAngle);
			sinRay = sin(startAngle - rayAngle);
		} else {
			cosRay = cos(startAngle + rayAngle);
			sinRay = sin(startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (search.searchDistance / voronoi.resolution); ray += search.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if ((vCell.status & VoronoiGrid::VoronoiCell::Wall) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Restricted))
				break;
			else if (vCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
				search.skeleton1st = cell;
				search.skeletonSearch.push_back(cell);
				return cell;
			}
		}
	}
	return Index2D(-1,-1);
}

Index2D Explorer::findNextSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D previous) {
	Index2D prev = previous;
	bool lookingFor2ndCell = false;
	if (!CELL_IS_VALID(previous, voronoi)) {
		// Use wall point to find a virtual previous
		lookingFor2ndCell = true;
		Index2D wallCell = voronoi.findCell(search.wallTarget);

		int dx = wallCell.x - currentCell.x,
				dy = wallCell.y - currentCell.y;
		if (dx == 0 && dy == 0) {
			return Index2D(-1, -1);
		}

		if (search.side == SearchParameters::Left) {
			if (dx > 0 && dy >= 0)
				prev = Index2D(currentCell.x, currentCell.y+1);
			else if (dy > 0) {
				prev = Index2D(currentCell.x-1, currentCell.y);
			} else if (dx < 0) {
				prev = Index2D(currentCell.x, currentCell.y-1);
			} else {
				prev = Index2D(currentCell.x+1, currentCell.y);
			}
		} else {
			if (dx >= 0 && dy > 0)
				prev = Index2D(currentCell.x+1, currentCell.y);
			else if (dx > 0) {
				prev = Index2D(currentCell.x, currentCell.y-1);
			} else if (dy < 0) {
				prev = Index2D(currentCell.x-1, currentCell.y);
			} else {
				prev = Index2D(currentCell.x, currentCell.y+1);
			}
		}
	}

	int dx = currentCell.x - prev.x,
			dy = currentCell.y - prev.y;

	Index2D searchOrder[4];
	if (search.side == SearchParameters::Left) {
		searchOrder[0] = Index2D(-dy, dx);
		searchOrder[1] = Index2D(dx, dy);
		searchOrder[2] = Index2D(dy, -dx);
		searchOrder[3] = Index2D(-dx, -dy);
	} else {
		searchOrder[0] = Index2D(dy, -dx);
		searchOrder[1] = Index2D(dx, dy);
		searchOrder[2] = Index2D(-dy, dx);
		searchOrder[3] = Index2D(-dx, -dy);
	}

	for (unsigned int i = 0; i < 4; i++) {
		Index2D next = currentCell + searchOrder[i];

		if (!CELL_IS_VALID(next, voronoi))
			continue;

		if (voronoi.cells[next.y * voronoi.width + next.x].status & VoronoiGrid::VoronoiCell::Skeleton) {
			if (lookingFor2ndCell)
				search.skeleton2nd = next;
			search.skeletonSearch.push_back(next);

			return next;
		}
	}

	return Index2D(-1,-1);
}

Pose Explorer::findWaypointTarget(const VoronoiGrid& voronoi, const Pose& robot) {
	if (search.waypoints.size() == 0)
		search.setWaypoints(SearchParameters::createStair(), robot);


	{{
		Lock lock(search.waypointLock, true);
		while (search.currentWaypoint < search.waypoints.size() &&
				search.waypoints[search.currentWaypoint].position.distanceTo(robot.position) < search.proximityDistance)
			++search.currentWaypoint;

		if (search.currentWaypoint >= search.waypoints.size()) {
			if (!search.atDestination && search.waypoints.size() > 0) {
				Point destination = search.waypoints[search.waypoints.size()-1].position;
				search.atDestination = true;
				LOG("Explorer: Arrived at destination (%.2lf, %.2lf, %.2lf)\n",
						destination.x, destination.y, destination.z);
			}
			search.waypoint = Point(INFINITY, INFINITY, INFINITY);
			return Pose(INFINITY, INFINITY, INFINITY);
		}
		search.waypoint = search.waypoints[search.currentWaypoint].position;
	}}

	// Head towards search.waypoint
	double waypointHeading = atan2(search.waypoint.y - robot.position.y, search.waypoint.x - robot.position.x),
			waypointDistance = search.waypoint.distanceTo(robot.position);

	double minAcceptableDistance = search.searchDistance - (search.rayStep * voronoi.resolution);
	if ((waypointDistance - search.proximityDistance) < minAcceptableDistance)
		minAcceptableDistance = waypointDistance - search.proximityDistance;
	double d = traversibleDistance(voronoi, robot.position, waypointHeading, search.searchDistance);
	if (d >= minAcceptableDistance) {
		// Head directly towards waypoint
//		LOG("Explorer::findWaypointTarget(): Heading directly towards waypoint\n");
		search.pathBlocked = false;
		Point target = robot.position + Point(d*cos(waypointHeading), d*sin(waypointHeading), 0);
		return Pose(target, Quaternion());
	}

	// Search for a path in the direction of the waypoint
	for (double a = search.angleStep; a <= search.obstructionAngle; a += search.angleStep) {
		d = traversibleDistance(voronoi, robot.position, waypointHeading + a, search.searchDistance);
		if (d >= minAcceptableDistance) {
//			LOG("Explorer::findWaypointTarget(): Taking left branch (%.1lf).\n", RAD2DEG((a)));
			search.pathBlocked = false;
			Point target = robot.position + Point(d*cos(waypointHeading+a), d*sin(waypointHeading+a), 0);
			return Pose(target, Quaternion());
		}

		d = traversibleDistance(voronoi, robot.position, waypointHeading - a, search.searchDistance);
		if (d >= minAcceptableDistance) {
//			LOG("Explorer::findWaypointTarget(): Taking right branch (%.1lf).\n", RAD2DEG((-a)));
			search.pathBlocked = false;
			Point target = robot.position + Point(d*cos(waypointHeading-a), d*sin(waypointHeading-a), 0);
			return Pose(target, Quaternion());
		}
	}

	if (!search.pathBlocked) {
		WARN("Explorer::findWaypointTarget(): Path to waypoint (%.2lf, %.2lf, %.2lf) is blocked.\n",
				search.waypoint.x, search.waypoint.y, search.waypoint.z);
	}
	search.pathBlocked = true;
	return Pose(INFINITY, INFINITY, INFINITY);
}

double Explorer::traversibleDistance(const VoronoiGrid& voronoi, const Point position, const double angle, const double max) {
	double sinA = sin(angle), cosA = cos(angle), rStep = search.rayStep * voronoi.resolution;

	double ray = 0;
	for (; ray <= max; ray += rStep) {
		Index2D cIdx = voronoi.findCell(position + Point(ray*cosA, ray*sinA, 0));

		if (!CELL_IS_VALID(cIdx, voronoi)) {
			break;
		}

		const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cIdx);
		if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
				vCell.status == VoronoiGrid::VoronoiCell::Restricted) {
			break;
		}
	}
	ray -= rStep;


	return ray;
}

#define CELL_PXL(C)		(((uint8_t *)rval->data) + (voronoi.height - (C).y - 1) * rval->step + 3 * (C).x)
ImagePtr Explorer::getPlanImage(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target) {
	ImagePtr rval = voronoi.getImage();

//	LOG("Explorer::getPlanImage(): - robot(%.2lf, %.2lf, %.2lf) (%.2lf, %.2lf, %.2lf, %.2lf).\n",
//			robot.position.x, robot.position.y, robot.position.z,
//			robot.orientation.x, robot.orientation.y, robot.orientation.z, robot.orientation.w);
	// Draw robot pose
	Index2D robotCell = voronoi.findCell(robot.position);

	if (CELL_IS_VALID(robotCell, voronoi)) {
		double roll, pitch, yaw, cosYaw, sinYaw;
		robot.orientation.getYPR(yaw, pitch, roll);
		cosYaw = cos(yaw);
		sinYaw = sin(yaw);

		/**
		 * Draw robot heading.
		 */
		for (float r = 0; r <= 20; r += 0.95) {
			Index2D rayCell = robotCell + Index2D(r * cosYaw, r * sinYaw);

			if (!CELL_IS_VALID(rayCell, voronoi))
				continue;

			uint8_t* pxl = CELL_PXL(rayCell);

			pxl[0] = 127;
			pxl[1] = 127;
			pxl[2] = 255;
		}

		/**
		 * Mark robot position
		 */
		uint8_t* pxl = CELL_PXL(robotCell);
		pxl[0] = 0;
		pxl[1] = 0;
		pxl[2] = 255;
	}

	if (target.isFinite()) {
		Index2D targetCell = voronoi.findCell(target.position);

		if (CELL_IS_VALID(targetCell, voronoi)) {
			for (uint32_t i = 0; i < 3; ++i) {

				Index2D neighbors[4];
				neighbors[0] = Index2D(0, i);
				neighbors[1] = Index2D(0, -i);
				neighbors[2] = Index2D(i, 0);
				neighbors[3] = Index2D(-i, 0);

				for (uint32_t n = 0; n < 4; ++n) {
					Index2D mark = targetCell + neighbors[n];
					if (CELL_IS_VALID(mark, voronoi)) {
						uint8_t* pxl = CELL_PXL(mark);
						pxl[0] = 255;
						pxl[1] = 191;
						pxl[2] = 0;
					}
				}
			}
		}
	}


	// Draw search results
	if (search.strategy == SearchParameters::WallFollow) {
		Index2D wallC(-1,-1), prevWallC(-1,-1);
		uint8_t* pxl = NULL;
		if (search.wallTarget.isFinite()) {
			wallC = voronoi.findCell(search.wallTarget);
		}
		if (search.previousWallTarget.isFinite()) {
			prevWallC = voronoi.findCell(search.previousWallTarget);
		}

		if (CELL_IS_VALID(wallC, voronoi) || wallC == prevWallC) {
			pxl = CELL_PXL(wallC);

			pxl[0] = 127;
			pxl[1] =   0;
			pxl[2] = 255;
		} else {
			if (CELL_IS_VALID(wallC, voronoi)) {
				pxl = CELL_PXL(wallC);

				pxl[0] = 255;
				pxl[1] =   0;
				pxl[2] = 255;
			}
			if (CELL_IS_VALID(prevWallC, voronoi)) {
				pxl = CELL_PXL(prevWallC);

				pxl[0] = 255;
				pxl[1] = 127;
				pxl[2] = 255;
			}
		}

		// The section of skeleton searched
		for (size_t i = 0; i < search.skeletonSearch.size(); ++i) {
			Index2D idx = search.skeletonSearch[i];
			if (CELL_IS_VALID(idx, voronoi)) {
				pxl = CELL_PXL(idx);

				pxl[0] = 127;
				pxl[1] = 127;
				pxl[2] = 127;
			}
		}

		if (CELL_IS_VALID(search.skeleton2nd, voronoi)) {
			pxl = CELL_PXL(search.skeleton2nd);

			pxl[0] = 191;
			pxl[1] = 127;
			pxl[2] =   0;
		}

		if (CELL_IS_VALID(search.skeleton1st, voronoi)) {
			pxl = CELL_PXL(search.skeleton1st);

			pxl[0] = 255;
			pxl[1] = 127;
			pxl[2] =   0;
		}
	} else if (search.strategy == SearchParameters::Waypoint) {
		uint8_t* pxl = NULL;
		Lock lock(search.waypointLock);

		if (search.waypoint.isFinite()) {
			Index2D wC = voronoi.findCell(search.waypoint);
			if (CELL_IS_VALID(wC, voronoi)) {
				Colour c(191, 127, 0);
				if (search.pathBlocked)
					c = Colour(191,0,0);
				for (uint32_t i = 0; i < 3; ++i) {

					Index2D neighbors[4];
					neighbors[0] = Index2D(i, i);
					neighbors[1] = Index2D(i, -i);
					neighbors[2] = Index2D(-i, i);
					neighbors[3] = Index2D(-i, -i);

					for (uint32_t n = 0; n < 4; ++n) {
						Index2D mark = wC + neighbors[n];
						if (CELL_IS_VALID(mark, voronoi)) {
							uint8_t* pxl = CELL_PXL(mark);
							pxl[0] = c.r;
							pxl[1] = c.g;
							pxl[2] = c.b;
						}
					}
				}
			}
		}

		for (size_t w = search.currentWaypoint; w < search.waypoints.size(); ++w) {
			Index2D wC = voronoi.findCell(search.waypoints[w].position);

			if (!CELL_IS_VALID(wC, voronoi))
				continue;

			pxl = CELL_PXL(wC);

			pxl[0] = 255;
			pxl[1] = 127;
			pxl[2] =   0;
		}
	}

	return rval;
}

} // namespace crosbot


