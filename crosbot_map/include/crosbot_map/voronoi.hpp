/*
 * voronoi.hpp
 *
 *  Created on: 15/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_VORONOI_HPP_
#define CROSBOT_VORONOI_HPP_

#include <crosbot/geometry.hpp>
#include <crosbot/data.hpp>

#ifdef ROS_VERSION

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>	// TODO: create own path class

#endif

namespace crosbot {

#define ERASE_SKELETON  ~VoronoiCell::Skeleton

class VoronoiGrid : public Data {
public:
	struct VoronoiCell {
	public:
	    enum CellStatus {
	        Vacant = 0,
	        Skeleton = 1,
	        Wall = 2,
	        Restricted = 4,
	        PatiallyRestricted = 8,
	        Expansion = 16,
	        NotVisible = 255
	    };

	    uint8_t status;
	    int nearestWall, nextNearestWall;
	    Index2D nearestWallCell;
	    double distanceFromWall, nextNearestWallDistance;	// In cells

	    double distanceToRobot, historyDistance;			// In cells

	    VoronoiCell() : status(Vacant), nearestWall(-1), nextNearestWall(-1),
	            distanceFromWall(INFINITY), nextNearestWallDistance(INFINITY),
	            nearestWallCell(-1, -1),
	            distanceToRobot(INFINITY), historyDistance(INFINITY)
	    {
	    }
	};

	struct Wall {
	public:
	    std::vector<Index2D> cells;
	    Index2D minBounds, maxBounds;

	    Wall() {}
	    Wall(const Index2D& idx) : minBounds(idx), maxBounds(idx) { cells.push_back(idx); }

	    inline void addAll(const Wall& wall) {
	        for (unsigned int i = 0; i < wall.cells.size(); i++) {
	        	const Index2D& idx = wall.cells[i];
	            cells.push_back(idx);

	            if (idx.x < minBounds.x)
	            	minBounds.x = idx.x;
	            if (idx.y < minBounds.y)
	            	minBounds.y = idx.y;
	            if (idx.x > maxBounds.x)
	            	maxBounds.x = idx.x;
	            if (idx.y > maxBounds.y)
	            	maxBounds.y = idx.y;
	        }
	    }

	    inline bool connectsTo(const Index2D& cell, const double maxConnectionDist) const {
	        double distance;

	        for (unsigned int i = 0; i < cells.size(); i++) {
	            distance = cell.distanceTo(cells[i]);
	            if (distance <= maxConnectionDist) {
	                return true;
	            }
	        }
	        return false;
	    }

	    inline double getVoronoiDistance(const Index2D& cell, Index2D& nearestPoint) const {
	        double d = INFINITY;
	        for (unsigned int i = 0; i < cells.size(); i++) {
	            double d2 = cell.distanceTo(cells[i]);
	            if (d2 < d) {
	                d = d2;
	                nearestPoint = cells[i];
	            }
	        }
	        return d;
	    }

	    inline size_t getIndex(const Index2D& cell) const {
	        for (size_t i = 0; i < cells.size(); i++) {
	            const Index2D& hCell = cells[i];
	            if (hCell.x == cell.x && hCell.y == cell.y)
	                return i;
	        }
	        return -1;
	    }
	};

	struct Constraints {
	public:
		enum HistoryEffect {
		    EraseHorizon, KeepMidLine
		};

		double restricted, partial, expand;
		double connection;
		int orphanThreshold, minOccupiedProb;
		bool make4connected;
		HistoryEffect historyEffect;

		Constraints() : restricted(0.5), partial(0.5), expand(0.5),
				connection(0.0), orphanThreshold(20), minOccupiedProb(50),
				make4connected(true), historyEffect(EraseHorizon)
		{}
	};

    unsigned int width, height;
    float resolution;
    Pose origin;
    std::string frame;

    std::vector<VoronoiCell> cells;

    Wall skeleton, history;
    std::vector<Wall> walls;
    double minRobotDistance;	// In cells.

    VoronoiGrid() : width(0), height(0), resolution(0) {}

    ImagePtr getImage() const;
    inline Index2D findCell(const Point p) const {
    	// TODO: Use origin orientation
    	return Index2D((p.x - origin.position.x) / resolution, (p.y - origin.position.y) / resolution);
    }
    inline const VoronoiCell& getVoronoiCell(const Index2D& idx) const {
    	return cells[idx.y * width + idx.x];
    }
    inline Point getPosition(const Index2D idx) const {
    	// TODO: Use origin orientation
    	return Point(origin.position.x + (idx.x + 0.5) * resolution,
    			origin.position.y + (idx.y + 0.5) * resolution, 0);
    }

#ifdef ROS_VERSION

    VoronoiGrid(const nav_msgs::OccupancyGrid&, const Constraints&,
    		Pose = Pose(INFINITY, INFINITY, INFINITY),
    		const nav_msgs::Path& = nav_msgs::Path());
protected:
    void findWalls(const nav_msgs::OccupancyGrid&, const Constraints&, Pose, const nav_msgs::Path&);
public:
#endif

protected:
    void calculateCellDistances(const Constraints&, Pose);
    void calculateCellStatus(const Constraints&);
    void makeSkeleton4Connected(const Constraints&);

    inline bool isHorizonEnd(int x, int y) {
        if (x < 0 || x >= (int)width || y < 0 || y >= (int)height)
            return false;
        int c = y * width + x;

        int horizonNeighbours = 0;
        if (x > 0 && (cells[c-1].status & VoronoiCell::Skeleton))
            horizonNeighbours += 1;
        if (x < (int)width - 1 && (cells[c+1].status & VoronoiCell::Skeleton))
            horizonNeighbours += 1;
        if (y > 0 && (cells[c-width].status & VoronoiCell::Skeleton))
            horizonNeighbours += 1;
        if (y < (int)height - 1 && (cells[c+width].status & VoronoiCell::Skeleton))
            horizonNeighbours += 1;

        return horizonNeighbours < 2;
    }


    inline int deleteHorizonEnd(int x, int y, int cellsAlreadyDeleted, const Constraints& constraints) {
        if (x < 0 || x >= (int)width || y < 0 || y >= (int)height)
            return 0;

        int c = y * width + x;
        if (!(cells[c].status & VoronoiCell::Skeleton) || !isHorizonEnd(x, y) ||
                cellsAlreadyDeleted >= constraints.orphanThreshold)
            return 0;

        cells[c].status &= ERASE_SKELETON;
        size_t hc = skeleton.getIndex(Index2D(x,y));
        if (hc != ((size_t)-1)) {
            skeleton.cells.erase(skeleton.cells.begin()+hc);
        }
        ++cellsAlreadyDeleted;
        return 1 +
                deleteHorizonEnd(x-1, y, cellsAlreadyDeleted, constraints) +
                deleteHorizonEnd(x+1, y, cellsAlreadyDeleted, constraints) +
                deleteHorizonEnd(x, y-1, cellsAlreadyDeleted, constraints) +
                deleteHorizonEnd(x, y+1, cellsAlreadyDeleted, constraints);
    }


    inline int getConnectedHorizonSize(std::vector<char>& marks, int x, int y) {
        if (y < 0 || y >= (int)height || x < 0 || x >= (int)width) return 0;
        int c = y * width + x;
        if (marks[c] != 0) return 0;
        marks[c] = 1;
        if (!(cells[c].status & VoronoiCell::Skeleton)) return 0;
        return 1 + getConnectedHorizonSize(marks, x+1, y) +
                + getConnectedHorizonSize(marks, x - 1, y)
                + getConnectedHorizonSize(marks, x, y + 1)
                + getConnectedHorizonSize(marks, x, y - 1);
    }

    inline int deleteConnectedHorizon(int x, int y) {
        if (y < 0 || y >= (int)height || x < 0 || x >= (int)width) return 0;
        int c = y * width + x;
        if (!(cells[c].status & VoronoiCell::Skeleton)) return 0;

        cells[c].status &= ERASE_SKELETON;
    //    grid->cells[c].status = VORONOICELL_EXPAND;
        size_t hc = skeleton.getIndex(Index2D(x,y));
        if (hc != ((size_t)-1)) {
            skeleton.cells.erase(skeleton.cells.begin()+hc);
        }

        return 1 +
                deleteConnectedHorizon(x+1, y) +
                deleteConnectedHorizon(x-1, y) +
                deleteConnectedHorizon(x, y+1) +
                deleteConnectedHorizon(x, y-1);
    }
};
typedef Handle<VoronoiGrid> VoronoiGridPtr;

}  // namespace crosbot


#endif /* CROSBOT_VORONOI_HPP_ */
