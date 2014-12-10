/*
 * explorer.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_explore/explorer.hpp>
#include <crosbot/utils.hpp>

#include <unistd.h>

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
			Pose relative = pose.toTF().inverse() * target.toTF();
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

	Index2D saferCell = findSaferSkeletonCell(voronoi, currentCell, robotCell);

	return Pose(voronoi.getPosition(saferCell), Quaternion());
}

Point Explorer::findWall(const VoronoiGrid& voronoi, const Pose& robot, double startAngle) {
	double Y,P,R, cosRay, sinRay;
	robot.orientation.getYPR(Y,P,R);
	Index2D robotCell = voronoi.findCell(robot.position), cell;
	Index2D wallCell(-1,-1);

	double rayAngle = 0;
	for (; rayAngle < 2*M_PIl && !CELL_IS_VALID(wallCell, voronoi); rayAngle += search.angleStep) {
		if (search.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (search.searchDistance / voronoi.resolution); ray += search.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
				return voronoi.getPosition(wallCell);
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Restricted) {
				wallCell = vCell.nearestWallCell;
				return voronoi.getPosition(wallCell);
			} else if ((vCell.status & VoronoiGrid::VoronoiCell::Restricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Expansion)) {
				wallCell = vCell.nearestWallCell;
			}
		}
	}

	for (; rayAngle < 2*M_PIl; rayAngle += search.angleStep) {
		if (search.side == SearchParameters::Left) {
			cosRay = cos(Y + startAngle - rayAngle);
			sinRay = sin(Y + startAngle - rayAngle);
		} else {
			cosRay = cos(Y + startAngle + rayAngle);
			sinRay = sin(Y + startAngle + rayAngle);
		}

		for (double ray = 0; ray <= (search.searchDistance / voronoi.resolution); ray += search.rayStep) {
			cell = Index2D(robotCell.x + ray * cosRay, robotCell.y + ray * sinRay);

			if (!CELL_IS_VALID(cell, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell);
			if (vCell.status & VoronoiGrid::VoronoiCell::Wall) {
				wallCell = cell;
				return voronoi.getPosition(wallCell);
			} else if (vCell.status & VoronoiGrid::VoronoiCell::Restricted) {
				wallCell = vCell.nearestWallCell;
				return voronoi.getPosition(wallCell);
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

Index2D Explorer::findSaferSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot) {
	uint32_t n = 0;
	Index2D safeCell = currentCell;
	if (currentCell == robot)
		return currentCell;

//	double currentRisk = cellTraversibleRisk(voronoi, currentCell, robot);
//	Index2D searchOrder[4];
//	searchOrder[0] = Index2D(0,1);
//	searchOrder[1] = Index2D(0,-1);
//	searchOrder[2] = Index2D(1,0);
//	searchOrder[3] = Index2D(-1,0);
//
//	bool riskDecreasing = true;
//	for (; n < search.maxIterations && riskDecreasing; ++n) {
//		riskDecreasing = false;
//		for (int ne = 0; ne < 4; ++ne) {
//			Index2D neighbour = safeCell + searchOrder[ne];
//
//			if (!CELL_IS_VALID(neighbour, voronoi))
//				continue;
//
//			const VoronoiGrid::VoronoiCell& cell = voronoi.cells[neighbour.y * voronoi.width + neighbour.x];
//			if (!(cell.status & VoronoiGrid::VoronoiCell::Skeleton))
//				continue;
//
//			double neighbourRisk = cellTraversibleRisk(voronoi, neighbour, robot);
//			neighbourRisk *= 1 + neighbour.distanceTo(currentCell) / currentCell.distanceTo(robot);
//			if (neighbourRisk < currentRisk) {
//				safeCell = neighbour;
//				currentRisk = neighbourRisk;
//				riskDecreasing = true;
//			}
//		}
//	}

	int32_t windowSize = ceil(search.searchDistance / voronoi.resolution) + 1;
	double restrictedC = (voronoiConstraints.restricted / voronoi.resolution),
			maxExpandC = ((voronoiConstraints.partial + voronoiConstraints.expand) / voronoi.resolution);

	double maxScore = 0, currentScore = -INFINITY, currentD = currentCell.distanceTo(robot);
	for (int y = -windowSize; y <= windowSize; ++y) {
		for (int x = -windowSize; x <= windowSize; ++x) {
			Index2D cell2Check = currentCell + Index2D(x,y);

			if (!CELL_IS_VALID(cell2Check, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell2Check);
			if (!(vCell.status & VoronoiGrid::VoronoiCell::Skeleton))
				continue;
			double d = cell2Check.distanceTo(robot), score = 0;
			double dx = (cell2Check.x - robot.x) / d,
					dy = (cell2Check.y - robot.y) / d;

			int count = 0;
			for (double r = 0; r <= d; r += search.rayStep) {
				Index2D rayCell(robot.x + dx*r, robot.y + dy*r);

				if (!CELL_IS_VALID(rayCell, voronoi))
					continue;

				const VoronoiGrid::VoronoiCell& rCell = voronoi.getVoronoiCell(rayCell);
				if ((rCell.status & VoronoiGrid::VoronoiCell::Wall) ||
						(rCell.status & VoronoiGrid::VoronoiCell::Restricted)) {
					score = 0;
					break;
				}

				// The score for a raytraced cell is as 0..1 value representing distance from wall
				double d = (rCell.distanceFromWall - restrictedC) / maxExpandC;
				if (d > 1)
					d = 1;
				double cellScore;
//				if (rCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
//					cellScore = 1.2 * d;
//				} else
				if (rCell.status & VoronoiGrid::VoronoiCell::Vacant) {
					cellScore = 1;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
					cellScore = 0.5 * d;
				} else {
					cellScore = d;
				}

				score += cellScore;
				++count;
//				ROS_INFO("Score Sum (%d, %d)(%.3lf) - %.2lf, %.2lf - %.2lf, %.2lf", cell2Check.x, cell2Check.y, score,
//						rCell.distanceFromWall, restrictedC);
			}

			if (count == 0)
				continue;

			// Score for a cell is the mean of the intermediate ray traced cells
			score = score / count;

			// Adjust score based on distance from the goal
			double adjustment = 1 - pow((currentCell.distanceTo(cell2Check) / currentD), 2);
			score *= adjustment;

//			ROS_INFO("Score (%d, %d)(%.3lf)", cell2Check.x, cell2Check.y, score);
			if (currentCell == cell2Check) {
				currentScore = score;
			}

			if (score > maxScore) {
				safeCell = cell2Check;
				maxScore = score;
			}
		}
	}

//	ROS_WARN("Preferring (%d, %d)(%.3lf) over (%d, %d)(%.3lf)",
//			safeCell.x, safeCell.y, maxScore,
//			currentCell.x, currentCell.y, currentScore);
	return safeCell;
}

Index2D Explorer::findSaferCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot) {
	double voronoiDistance = currentCell.distanceTo(robot) * voronoi.resolution;
	double score, maxScore = 0, meanScore, sint, cost, xf, yf, cosA;
	int c, count, xi, yi;

	double vdSq = voronoiDistance*voronoiDistance, vd2cosA;
	Index2D vectorCurrent = currentCell - robot;

	Index2D saferCell = currentCell;
	int32_t windowSize = ceil(search.searchDistance / voronoi.resolution) + 1;

	double restrictedC = (voronoiConstraints.restricted / voronoi.resolution),
			maxExpandC = ((voronoiConstraints.partial + voronoiConstraints.expand) / voronoi.resolution);

	for (int y = -windowSize; y <= windowSize; ++y) {
		for (int x = -windowSize; x <= windowSize; ++x) {
			Index2D cell2Check = currentCell + Index2D(x,y);

			if (!CELL_IS_VALID(cell2Check, voronoi))
				continue;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(cell2Check);
			if ((vCell.status & VoronoiGrid::VoronoiCell::Wall) ||
					(vCell.status & VoronoiGrid::VoronoiCell::Restricted))
				continue;

			double d = cell2Check.distanceTo(robot);
			double dx = (cell2Check.x - robot.x) / d,
					dy = (cell2Check.y - robot.y) / d;

			score = count = 0;
			for (double r = 0; r <= d; r += search.rayStep) {
				Index2D rayCell(robot.x + dx*r, robot.y + dy*r);

				if (!CELL_IS_VALID(rayCell, voronoi))
					continue;

				const VoronoiGrid::VoronoiCell& rCell = voronoi.getVoronoiCell(rayCell);
				if ((rCell.status & VoronoiGrid::VoronoiCell::Wall) ||
									(rCell.status & VoronoiGrid::VoronoiCell::Restricted)) {
					score = 0; break;
				}
				double d = (rCell.distanceFromWall - restrictedC) / maxExpandC;
				if (d > 1)
					d = 1;
				double cellScore;
				if (rCell.status & VoronoiGrid::VoronoiCell::Skeleton) {
					cellScore = 1.2 * d;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::Vacant) {
					cellScore = 1;
				} else if (rCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
					cellScore = 0.5 * d;
				} else {
					cellScore = d;
				}

				score += cellScore;
				++count;
			}

			if (count == 0)
				continue;

			meanScore = score / count;

			// adjust meanScore based on distance to goal

			Index2D vectorCheck = cell2Check - robot;
			double theta = atan2(currentCell.y - robot.y, currentCell.x - robot.x) -
					atan2(cell2Check.y - robot.y, cell2Check.x - robot.x);
			cosA = cos(theta);
			vd2cosA = voronoiDistance*cosA*2;
			double r = currentCell.distanceTo(cell2Check);

			d = sqrt(vdSq + r*r - r*vd2cosA);
			meanScore = (meanScore / exp2(2*d*search.searchDistance/voronoiDistance));

			if (meanScore > maxScore) {
				saferCell = cell2Check;
				maxScore = meanScore;
			}
		}
	}

	return saferCell;
}

//void Explorer::findSaferTarget(double& voronoiAngle, double& voronoiDistance, const Pose& robot) {
//	double preferredAngle = voronoiAngle, preferredDist = voronoiDistance;
//	double score, maxScore = 0, meanScore, sint, cost, xf, yf, cosA;
//
//	double vdSq = voronoiDistance*voronoiDistance, vd2cosA;
//
//	int c, count, xi, yi;
//	VoronoiCell *cell = NULL;
//
//	for (double theta = -angleAcceptDiff; theta <= angleAcceptDiff; theta += searchRes) {
//		sint = sin(theta + voronoiAngle + localMap->robotPose.o.yaw),
//		cost = cos(theta + voronoiAngle + localMap->robotPose.o.yaw),
//		cosA = cos(theta);
//
//		vd2cosA = voronoiDistance*cosA*2;
//
//		score = count = 0;
//
//		for (double ray = searchStep; ray <= searchDistance; ray += searchStep) {
//			GET_CELL_FOR_RAY(ray, c);
//			if (c == -1 || cell->status == VoronoiCell::Occupied || cell->status == VoronoiCell::Obstacle) {
//				break;
//			}
//			double d = (cell->voronoiDistance - localMap->restrictedC) / localMap->maxExpansionC;
//			if (d > 1)
//				d = 1;
//			double cellScore;
//			if (cell->status == VoronoiCell::Horizon) {
//				cellScore = 1.2 * d;
//			} else if (cell->status == VoronoiCell::Vacant) {
//				cellScore = 1;
//			} else {
//				cellScore = d;
//			}
//
//			score += cellScore;
//			++count;
//
//			meanScore = score / count;
//
//			// adjust meanScore based on distance to goal
//			d = sqrt(vdSq + ray*ray - ray*vd2cosA);
//			meanScore = (meanScore / exp2(2*d*searchDistance/voronoiDistance));
//
//			if (meanScore > maxScore) {
//				preferredAngle = theta + voronoiAngle; preferredDist = ray;
//				maxScore = meanScore;
//			}
//		}
//	}
//
//	NORMALISE_ANGLE(preferredAngle);
//	voronoiAngle = preferredAngle; voronoiDistance = preferredDist;
//}

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
				statusChanged("Arrived");
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
	statusChanged("Blocked");
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

			pxl[0] = 196;
			pxl[1] = 196;
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


