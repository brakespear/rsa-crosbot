/*
 * astar.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_explore/astar.hpp>
#include <crosbot/utils.hpp>

namespace crosbot {

Plan PathPlanner::getPath(VoronoiGridPtr voronoi, Pose start, Pose goal, ImagePtr image) {
	Plan rval;
	Index2D startCell, endCell;


	tf::Transform trans = voronoi->origin.toTF().inverse();
	start = trans * start.toTF();
	goal = trans * goal.toTF();

//	LOG("Start location (%.3f, %.3f) and goal (%.3f, %.3f)\n",
//			start.x, start.y, goal.x, goal.y);

	startCell = Index2D(start.position.x / voronoi->resolution, start.position.y / voronoi->resolution);
	endCell = Index2D(goal.position.x / voronoi->resolution, goal.position.y / voronoi->resolution);

	// If start/goal off map change to nearest cell
	if (startCell.x < 0)
		startCell.x = 0;
	else if ((unsigned int)startCell.x >= voronoi->width)
		startCell.x = voronoi->width - 1;

	if (startCell.y < 0)
		startCell.y = 0;
	else if ((unsigned int)startCell.y >= voronoi->height)
		startCell.y = voronoi->height - 1;

	if (startCell.x < 0)
		startCell.x = 0;
	else if ((unsigned int)startCell.x >= voronoi->width)
		startCell.x = voronoi->width - 1;

	if (startCell.y < 0)
		startCell.y = 0;
	else if ((unsigned int)startCell.y >= voronoi->height)
		startCell.y = voronoi->height - 1;

	CellPath cells = getCellPath(voronoi, startCell, endCell, image);

	// turn cells into points
	trans = voronoi->origin.toTF();
	for (uint32_t i = 0; i < cells.size(); ++i) {
		Index2D cell = cells[i];
		Pose p = Pose((cell.x+0.5)*voronoi->resolution,(cell.y+0.5)*voronoi->resolution,0);
		p = trans * p.toTF();
		rval.push_back(p);
	}

	return rval;
}

Colour AStarPlanner::searchedSkeletonColour(0, 160, 255),
		AStarPlanner::searchedExpansionColour(0, 200, 0),
		AStarPlanner::searchedRestrictedColour(200, 0, 0),
		AStarPlanner::searchedPartialRestrictedColour(200, 200, 0),
		AStarPlanner::searchedVacantColour(64, 64, 64),
		AStarPlanner::pathColour(255, 0, 255),
		black(0,0,0), white(255,255,255),
		skeletonColour(0, 255, 255),
		restrictColour(255, 0, 0), partiallyRestrictedColour(255, 255, 0),
		expandColour(0, 255, 0), notVisibleColour(0, 0, 255);

#define PAINT_PIXEL(P, COLOUR)			pxl = 3 * (P);						\
										bytes[pxl] = (COLOUR).r;			\
										bytes[pxl+1] = (COLOUR).g;			\
										bytes[pxl+2] = (COLOUR).b;

#define IS_COLOUR(C, COLOUR)			\
		(bytes[3*(C)] == (COLOUR).r && bytes[3*(C)+1] == (COLOUR).g && bytes[3*(C)+2] == (COLOUR).b)

class _AStarNode : public HandledObject {
public:
	Index2D currentCell, parentCell;
	double goalEstimate;
	double pathCost;

	_AStarNode() :
		HandledObject(), currentCell(-1, -1), parentCell(-1,-1), goalEstimate(INFINITY), pathCost(0)
	{}

	_AStarNode(Index2D startCell) :
		HandledObject(), currentCell(startCell), parentCell(-1,-1), goalEstimate(INFINITY), pathCost(0)
	{}

	_AStarNode(const Index2D& currentCell, double cellCost, double goalEstimate, Index2D parentCell, double parentCost) :
		HandledObject(), currentCell(currentCell), parentCell(parentCell), goalEstimate(goalEstimate), pathCost(cellCost + parentCost)
	{

	}

	inline double costEstimate() const {
		return goalEstimate + pathCost;
	}
};
typedef Handle<_AStarNode> AStarNode;

/**  Higher priority nodes have lower costs.  **/
inline bool operator<(const AStarNode& lhs, const AStarNode& rhs) {
	return lhs->costEstimate() > rhs->costEstimate();
}
inline bool operator<=(const AStarNode& lhs, const AStarNode& rhs) {
	return lhs->costEstimate() >= rhs->costEstimate();
}
inline bool operator>(const AStarNode& lhs, const AStarNode& rhs) {
	return lhs->costEstimate() < rhs->costEstimate();
}
inline bool operator>=(const AStarNode& lhs, const AStarNode& rhs) {
	return lhs->costEstimate() <= rhs->costEstimate();
}

struct AStarCellSearchStatus {
	bool closed;
	double goalEstimate;
	double cellRisk;
	Index2D parent;

	AStarCellSearchStatus() : closed(false), goalEstimate(-1), cellRisk(0) {}
};

#define GOAL_ESTIMATE(CIDX)	(lookForUnseenSkeleton?					\
		voronoi->skeleton.getVoronoiDistance(CIDX, nearCell):		\
		goalCell.distanceTo(CIDX))

CellPath AStarPlanner::getCellPath(VoronoiGridPtr voronoi, Index2D startCell, Index2D goalCell, ImagePtr image) {
	CellPath path;

	// The actual A* search

	// TODO: avoid robot history
	// TODO: get History
	bool lookForUnseenSkeleton = false;
	if (goalCell.x < 0 || goalCell.x >= voronoi->width || goalCell.y < 0 || goalCell.y >= voronoi->height)

//	if (debug) {
//		log("Starting in cell: (%d, %d)\n", startCell.x, startCell.y);
//		log("Goal in cell: (%d, %d)\n", goalCell.x, goalCell.y);
//	}

	if (!lookForUnseenSkeleton) {
		int gc = goalCell.y*voronoi->width+goalCell.x;
		double goalRisk = calculateCellRisk(voronoi->cells[gc], maxExpandC, expandScale);
		if (goalRisk == INFINITY || goalRisk < 0 || goalRisk == NAN) {
			ERROR("AStarPlanner::getPath(): Goal is not traversible.\n");
			return path;
		}
	}

	// Now perform the A* search.
	std::priority_queue<AStarNode> openNodes;
	std::vector<AStarCellSearchStatus> cellStats(voronoi->cells.size());
	AStarNode goalNode;

	std::vector<Index2D> neighbours;
	neighbours.push_back(Index2D(-1,-1));
	neighbours.push_back(Index2D(0,-1));
	neighbours.push_back(Index2D(1,-1));
	neighbours.push_back(Index2D(-1,0));
	neighbours.push_back(Index2D(1,0));
	neighbours.push_back(Index2D(-1,1));
	neighbours.push_back(Index2D(0,1));
	neighbours.push_back(Index2D(1,1));

//	Index2D* idx = &startCell;
	int c = startCell.y*voronoi->width + startCell.x;

	Index2D nearCell;
	cellStats[c].cellRisk = 0;
	cellStats[c].goalEstimate = GOAL_ESTIMATE(startCell);

	openNodes.push(new _AStarNode(startCell));

	// while not at goal and have open nodes
	while (goalNode == NULL && openNodes.size() > 0) {
		AStarNode node = openNodes.top();
		openNodes.pop();

		c = node->currentCell.y*voronoi->width + node->currentCell.x;
		if (cellStats[c].closed)
			continue;
		cellStats[c].closed = true;
		cellStats[c].parent = node->parentCell;

		if (node->pathCost == INFINITY || node->pathCost < 0 || node->pathCost == NAN) {
			LOG("AStarPlanner::getPath(): Have an invalid path cost of %.2lf for cell (%d, %d)\n", node->pathCost,
					node->currentCell.x, node->currentCell.y);
		}

//		log(LOG_PROGRESS, "Parent of (%d, %d) is (%d, %d). Risk %.2lf.\n",
//				node->currentCell.x, node->currentCell.y,
//				node->parentCell.x, node->parentCell.y,
//				cellStats[c].cellRisk);

		if (node->goalEstimate == 0) {
			goalNode = node;
			break;
		}

		for (unsigned int n = 0; n < neighbours.size(); n++) {
			Index2D neighbour = node->currentCell + neighbours[n];
			if (neighbour.x < 0 || neighbour.x >= (int)voronoi->width ||
					neighbour.y < 0 || neighbour.y >= (int)voronoi->height)
				continue;
//			LOG("Neighbour of (%d, %d) is (%d, %d)\n", node->currentCell.x, node->currentCell.y,
//					neighbour.x, neighbour.y);

			int nc = neighbour.y*voronoi->width + neighbour.x;
			if (cellStats[nc].closed)
				continue;

			if (cellStats[nc].goalEstimate < 0) {
				cellStats[nc].goalEstimate = GOAL_ESTIMATE(neighbour);
				cellStats[nc].cellRisk = calculateCellRisk(voronoi->cells[nc], maxExpandC, expandScale);

				if (cellStats[nc].cellRisk < 0 || cellStats[nc].cellRisk == NAN) {
					const VoronoiGrid::VoronoiCell& ncell = voronoi->cells[nc];
					ERROR("AStarPlanner::getPath(): Cell (%d, %d) with status %d has risk %.2lf. vd: %.2lf maxEC: %d\n",
							neighbour.x, neighbour.y,
							ncell.status, cellStats[nc].cellRisk,
							ncell.distanceFromWall, maxExpandC);
				}
			}

			// Skip non-traversible cells.
			if (cellStats[nc].cellRisk == INFINITY) {
//				log("Skipping cell with infinite risk.\n");
				continue;
			}

			int dx = neighbours[n].x, dy = neighbours[n].y;
			double cost = sqrt((double)(dx*dx + dy*dy));
			cost *= 1 + cellStats[nc].cellRisk;

			// create open node for neighbour
			openNodes.push(new _AStarNode(neighbour, cost, cellStats[nc].goalEstimate , node->currentCell, node->pathCost));
		}
	}

	if (image != NULL) {
		assert(image->width == voronoi->width && image->height == voronoi->height &&
				image->encoding == Image::RGB8);

		// Paint nodes in image.
		uint8_t* bytes = (uint8_t*)image->data;
		int pxl;

		for (unsigned int y = 0; y < voronoi->height; ++y) {
			for (unsigned int x = 0; x < voronoi->width; ++x) {
				unsigned int p = (voronoi->height - 1 - y) * voronoi->width + x,
						c = y * voronoi->width + x;
				if (voronoi->cells[c].status & VoronoiGrid::VoronoiCell::Skeleton) {
					if (cellStats[c].closed) {
						PAINT_PIXEL(p, searchedSkeletonColour);
					} else {
						PAINT_PIXEL(p, skeletonColour);
					}
				} else {
					switch(voronoi->cells[c].status) {
						case VoronoiGrid::VoronoiCell::Wall:
							PAINT_PIXEL(p, white); break;
						case VoronoiGrid::VoronoiCell::NotVisible:
							PAINT_PIXEL(p, notVisibleColour); break;
						case VoronoiGrid::VoronoiCell::Restricted:
							if (cellStats[c].closed) {
								PAINT_PIXEL(p, searchedRestrictedColour);
							} else {
								PAINT_PIXEL(p, restrictColour);
							}
							break;
						case VoronoiGrid::VoronoiCell::PatiallyRestricted:
							if (cellStats[c].closed) {
								PAINT_PIXEL(p, searchedPartialRestrictedColour);
							} else {
								PAINT_PIXEL(p, partiallyRestrictedColour);
							}
							break;
						case VoronoiGrid::VoronoiCell::Expansion:
							if (cellStats[c].closed) {
								PAINT_PIXEL(p, searchedExpansionColour);
							} else {
								PAINT_PIXEL(p, expandColour);
							}
							break;
						case VoronoiGrid::VoronoiCell::Vacant:
							if (cellStats[c].closed) {
								PAINT_PIXEL(p, searchedVacantColour);
							} else {
								PAINT_PIXEL(p, black);
							}
							break;
					}
				}
			}
		}
	}

	CellPath pathRev;
	if (goalNode == NULL) {
		LOG("AStarPlanner::getPath(): Unable to find path.\n");
		return path;
	}

//	log("Have arrived at goal node (%d, %d)) with estimate %.1lf.\n",
//			goalNode->currentCell.x, goalNode->currentCell.y, goalNode->goalEstimate);
	Index2D pCell = goalNode->currentCell;

	do {
		pathRev.push_back(pCell);
		c = pCell.y*voronoi->width+pCell.x;

		pCell = cellStats[c].parent;
	} while (pCell.x != -1 && pCell.y != -1);

//	if (debug) {
//		if (pathRev.size() < 2) {
//			LOG("Goal path is short.\n");
//		} else {
//			LOG("Found path from (%d, %d) to (%d, %d) in %u steps. Cost %.1lf\n",
//					pathRev[pathRev.size()-1].x, pathRev[pathRev.size()-1].y,
//					pathRev[0].x, pathRev[0].y, pathRev.size(),
//					goalNode->pathCost);
//		}
//	}

//	double d = (pathLength / (double)pathRev.size()) - 1;
//	if (fabs(d) > 0.1) {
//		LOG("Significant change in path length from %d to % u. Cost %.1lf\n", pathLength, pathRev.size(), goalNode->pathCost);
//	}
//	pathLength = pathRev.size();

	if (image != NULL) {
		uint8_t* bytes = (uint8_t*)image->data;
		int pxl;
		for (size_t wp = 0; wp < pathRev.size(); ++wp) {
			unsigned int p = (voronoi->height - pathRev[wp].y - 1) * voronoi->width + pathRev[wp].x;
			PAINT_PIXEL(p, pathColour);
		}
	}

	unsigned int culled = 0;
	if (pathRev.size() > 1 && cull > 0) {
		Index2D dxy = pathRev[1] - pathRev[0];
		for (size_t i = 1; i < pathRev.size()-1; i++) {
			Index2D ndxy = pathRev[i+1] - pathRev[i];
			if (ndxy == dxy && culled < cull) {
				pathRev.erase(pathRev.begin()+i);
				++culled; --i;
			} else {
				culled = 0;
			}
			dxy = ndxy;
		}
	}

	path.push_back(startCell);

	for (int w = pathRev.size()-1; w >= 0; w--) {
		path.push_back(pathRev[w]);
	}
	if (!lookForUnseenSkeleton) {
		path.push_back(goalCell);
	}

	return path;
}

} // namespace crosbot
