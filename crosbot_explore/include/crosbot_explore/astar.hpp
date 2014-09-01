/*
 * astar.hpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_ASTAR_HPP_
#define CROSBOT_ASTAR_HPP_

#include <crosbot_map/voronoi.hpp>
#include <crosbot/data.hpp>

namespace crosbot {

#define DEFAULT_SKELETON_DISCOUNT		0.5
#define DEFAULT_RESTRICTED_RISK			INFINITY
#define DEFAULT_RISK_K					50 // 5 // 3

typedef std::vector<Pose> Plan;
typedef std::vector<Index2D> CellPath;

class PathPlanner {
public:
	PathPlanner() {}
	virtual ~PathPlanner() {}
	virtual Plan getPath(VoronoiGridPtr grid, Pose start, Pose goal, ImagePtr);
	virtual CellPath getCellPath(VoronoiGridPtr grid, Index2D start, Index2D goal, ImagePtr)=0;
};

class AStarPlanner: public PathPlanner {
protected:

	inline double calculateCellRisk(const VoronoiGrid::VoronoiCell& cell, int maxExpandC, double expansionScale) {
		double risk = k * (maxExpandC - cell.distanceFromWall) / expansionScale;
		switch (cell.status) {
		case VoronoiGrid::VoronoiCell::Wall: case VoronoiGrid::VoronoiCell::NotVisible:
			risk = INFINITY; break;
		case VoronoiGrid::VoronoiCell::Vacant:
			risk = 0; break;
		case VoronoiGrid::VoronoiCell::Skeleton:
			risk = risk * skeletonDiscount; break;
		case VoronoiGrid::VoronoiCell::Restricted: case VoronoiGrid::VoronoiCell::PatiallyRestricted:
			risk = risk * restrictedBonus; break;
		case VoronoiGrid::VoronoiCell::Expansion:
			break;
		}
		if (risk < 0)
			risk = 0;
		return risk;
	}

public:
	/*
	 * Cell risk options
	 * horizonDiscount: a reduction of risk for travelling along horizon
	 * k: risk multiplier, roughly equates to for every 1cm of travel touching wall
	 * 		it is preferred to travel kcm in an open area
	 * restrictedBonus: increased risk of travelling through restricted areas,
	 * 		roughly equates to for every 1cm of travel inside restricted its preffered
	 * 		to travel restrictedBonus cm touching wall or restrictedBonus*k cm in
	 * 		open area
	 */
	double skeletonDiscount, restrictedBonus, k;
	unsigned int cull;
	int maxExpandC; double expandScale;

	static Colour searchedSkeletonColour, searchedExpansionColour,
		searchedRestrictedColour, searchedPartialRestrictedColour,
		searchedVacantColour, pathColour;

	AStarPlanner() :
		skeletonDiscount(DEFAULT_SKELETON_DISCOUNT), restrictedBonus(DEFAULT_RESTRICTED_RISK),
		k(DEFAULT_RISK_K), cull(0), maxExpandC(20), expandScale(10)
	{}

	virtual CellPath getCellPath(VoronoiGridPtr grid, Index2D start, Index2D goal, ImagePtr);
};

} // namespace crosbot

#endif /* ASTAR_HPP_ */
