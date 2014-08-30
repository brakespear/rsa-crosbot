/*
 * explorer.hpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_EXPLORER_HPP_
#define CROSBOT_EXPLORER_HPP_

#include <crosbot/thread.hpp>
#include <crosbot_map/voronoi.hpp>

namespace crosbot {

#define CELL_IS_VALID(C,V)		((C).x >= 0 && (C).y >= 0 && (C).x < (V).width && (C).y < (V).height)

class Explorer {
protected:
	void planThread();
	void driveThread();

	class PlanThread : public Thread {
	public:
		Explorer& explorer;
		PlanThread(Explorer& explorer) : explorer(explorer) {}

		void run() {
			explorer.planThread();
		}
	};

	class DriveThread : public Thread {
	public:
		Explorer& explorer;
		DriveThread(Explorer& explorer) : explorer(explorer) {}

		void run() {
			explorer.driveThread();
		}
	};

	PlanThread planThread_;
	DriveThread driveThread_;


	bool operating, paused, stopSent, searchWhilePaused;
	VoronoiGrid::Constraints voronoiConstraints;

	/**
	 * Search parameters.
	 */

	struct SearchParameters {
	public:
		enum Strategy {
			WallFollow, Waypoint
		};
		Strategy strategy;

		enum Side {
			Left, Right
		};
		Side side;

		double searchDistance, searchAngle;
		double rayStep, angleStep;
		uint32_t maxIterations;

		ReadWriteLock waypointLock;
		std::vector<Pose> waypoints;
		size_t currentWaypoint;
		double proximityDistance, obstructionAngle;
		bool atDestination, pathBlocked;

		static std::vector<Pose> createSpiral() {
			std::vector<Pose> rval;
			double a, r;
			for (int32_t i = 0; i < 100; ++i) {
				r = i * 0.02; a = i * .10;
				rval.push_back(Pose(r * cos(a), r * sin(a), 0));
			}

			return rval;
		}

		static std::vector<Pose> createStair() {
			std::vector<Pose> rval;
			double x, y;
			for (int32_t i = 0; i < 1000; ++i) {
				if ((i / 100) % 2 == 0) {
					y = i / 200;
					x = (i % 100) * 0.01 + i / 200;
				} else {
					y = (i % 100) * 0.01 + (i-100) / 200;
					x = (i + 100) / 200;
				}

				rval.push_back(Pose(x + 2, y, 0));
			}

			return rval;
		}

		// Search results
		Point wallTarget, previousWallTarget;
		Index2D skeleton1st, skeleton2nd;
		std::vector<Index2D> skeletonSearch;
		Point waypoint;
		SearchParameters() :
			strategy(WallFollow), side(Left),
			searchDistance(1.0), searchAngle(DEG2RAD(90)),
			rayStep(0.9), angleStep(DEG2RAD(1)), maxIterations(1000),
			currentWaypoint(0), proximityDistance(0.5), obstructionAngle(DEG2RAD(45)),
			atDestination(false), pathBlocked(false),
			wallTarget(INFINITY, INFINITY, INFINITY),
			previousWallTarget(INFINITY, INFINITY, INFINITY)
		{}

		void setWaypoints(const std::vector<Pose>& waypoints, const Pose& pose) {
			Lock lock(waypointLock, true);
			atDestination = false; pathBlocked = false;
			this->waypoints = waypoints;
			for (int64_t w = waypoints.size() - 1; w >= 0; --w) {
				if (waypoints[w].position.distanceTo(pose.position) <= proximityDistance) {
					currentWaypoint = w;
					return;
				}
			}
			currentWaypoint = 0;
		}
	};
	SearchParameters search;
	virtual Pose findDriveTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual Pose findWallFollowTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual Pose findWaypointTarget(const VoronoiGrid& voronoi, const Pose& robot);
	virtual ImagePtr getPlanImage(const VoronoiGrid& voronoi, const Pose& robot, const Pose& target);
	inline bool cellIsDirectlyTraversible(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const {
		double d = cell.distanceTo(pov);
		double dx = (cell.x - pov.x) / d,
				dy = (cell.y - pov.y) / d;

		for (double r = 0; r <= d; r += search.rayStep) {
			Index2D rayCell(pov.x + dx*r, pov.y + dy*r);

			if (!CELL_IS_VALID(rayCell, voronoi))
				return false;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(rayCell);

			if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
					vCell.status == VoronoiGrid::VoronoiCell::Restricted ||
					vCell.status == VoronoiGrid::VoronoiCell::NotVisible)
				return false;
		}
		return true;
	}

	inline double cellTraversibleRisk(const VoronoiGrid& voronoi, const Index2D& cell, const Index2D& pov) const {
		double d = cell.distanceTo(pov);
		double dx = (cell.x - pov.x) / d,
				dy = (cell.y - pov.y) / d;

		double risk = 0; uint32_t stepCount = 0;
		double restrictScale = voronoiConstraints.restricted / voronoi.resolution,
				partialScale = (voronoiConstraints.partial - voronoiConstraints.restricted) / voronoi.resolution,
				partialMax = voronoiConstraints.partial / voronoi.resolution,
				expansionScale = voronoiConstraints.expand / voronoi.resolution,
				expansionMax = (voronoiConstraints.expand + voronoiConstraints.partial) / voronoi.resolution;
		for (double r = 0; r <= d; r += search.rayStep) {
			Index2D rayCell(pov.x + dx*r, pov.y + dy*r);

			if (!CELL_IS_VALID(rayCell, voronoi))
				return INFINITY;

			const VoronoiGrid::VoronoiCell& vCell = voronoi.getVoronoiCell(rayCell);

			if (vCell.status == VoronoiGrid::VoronoiCell::Wall ||
					vCell.status == VoronoiGrid::VoronoiCell::Restricted ||
					vCell.status == VoronoiGrid::VoronoiCell::NotVisible)
				return INFINITY;

			++stepCount;
			if (vCell.status & VoronoiGrid::VoronoiCell::Vacant) {
				risk += 0.0;
			} else if (vCell.status & VoronoiGrid::VoronoiCell::PatiallyRestricted) {
				if (vCell.distanceFromWall > partialMax)
					risk += 1;
				else
					risk += 1 + (partialMax - vCell.distanceFromWall) / partialScale;
			} else {
				if (vCell.distanceFromWall > expansionMax)
					risk += 0.0;
				else {
					risk += (expansionMax - vCell.distanceFromWall) / expansionScale;
				}
			}
		}
		if (stepCount == 0) {
			return INFINITY;
		}
		return risk / stepCount;
	}

	// functions for the wall follow implementation
	virtual Point findWall(const VoronoiGrid& voronoi, const Pose& robot, double startAngle);
	virtual Index2D findFirstSkeletonCell(const VoronoiGrid& voronoi, const Pose& robot, double startAngle);
	virtual Index2D findNextSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D previous);
	virtual Index2D findSaferSkeletonCell(const VoronoiGrid& voronoi, const Index2D currentCell, const Index2D robot);

	// functions for the waypoint follow implementation
	virtual double traversibleDistance(const VoronoiGrid& voronoi, const Point position, const double angle, const double max = INFINITY);

	/**
	 * Drive parameters.
	 */

	struct DriveParameters {
	public:
		double maxVel, maxTurn;
		double turnOnly;

		ReadWriteLock driveLock;
		Pose driveTarget;

		DriveParameters() :
			maxVel(0.5), maxTurn(DEG2RAD(15)),
			turnOnly(DEG2RAD(30)), driveTarget(INFINITY, INFINITY, INFINITY)
		{}

		inline Pose getDriveTarget() {
			Lock lock(driveLock);
			return driveTarget;
		}
		inline void setDriveTarget(Pose pose) {
			Lock lock(driveLock, true);
			driveTarget = pose;
		}
	};
	DriveParameters drive;

	virtual Pose getLatestPose()=0;
	virtual bool stopMotors()=0;
	virtual void driveTo(const Pose& relativePosition)=0;
public:
	Explorer();
	virtual ~Explorer();

	virtual void shutdown();
	virtual void pause() { paused = true; statusChanged("Paused"); }
	virtual void resume() { drive.setDriveTarget(Pose(INFINITY, INFINITY, INFINITY)); paused = false; statusChanged("Running"); }
	virtual void statusChanged(const std::string& status) {}

	virtual VoronoiGridPtr getLatestVoronoi()=0;
protected:
};

} // namespace crosbot

#endif /* CROSBOT_EXPLORER_HPP_ */
