/*
 * localmap.hpp
 *
 *  Created on: 15/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_LOCALMAP_HPP_
#define CROSBOT_LOCALMAP_HPP_

#include <crosbot/data.hpp>


#ifdef ROS_VERSION

#include <nav_msgs/OccupancyGrid.h>

#endif

namespace crosbot {

class LocalMap : public Data {
public:
	class Cell {
	public:
		bool current;
		uint32_t hits;

		Cell() : current(false), hits(0) {}
	};

	/**
	 * The frame of reference for the local map.
	 */
	std::string frameID;

	/**
	 * Origin of the map. [ m, m, rad ]
	 */
	Pose origin;

	/**
	 * Cellular resolution of the map. [ m ]
	 */
	double resolution;

	/**
	 * Size of the map [ pxl , pxl ]
	 */
	uint32_t width, height;

	/**
	 * The cells of the map. ( [row][column] )
	 */
	std::vector < std::vector < Cell > > cells;

	/**
	 * Parameters for updating cells.
	 */
	int hitIncrement, missDecrement, maxHits;

	LocalMap(uint32_t width, uint32_t height, double resolution, std::string frameID="") :
		frameID(frameID),
		width(width), height(height), resolution(resolution),
		hitIncrement(1), missDecrement(1), maxHits(100)
	{
		if (height > 0) {
			cells.resize(height);
			if (width > 0) {
				for (uint32_t y = 0; y < height; ++y)
					cells[y].resize(width);
			}
		}

		origin.position.x = -(width * resolution) / 2;
		origin.position.y = -(height * resolution) / 2;
	}

	/**
	 * Shifts the local map by the given number of cells in each axis.
	 */
	void shift(int x, int y);

	/**
	 * Updates the local map occupancies with hits from a point cloud.
	 */
	void update(const PointCloud&);
	inline void update(PointCloudPtr ptr) { update(*ptr); }

	/**
	 * Gets a visual representation of the local map.
	 */
	ImagePtr getImage();
	ImagePtr getImage(Pose robotPose);

protected:
	inline void clearCurrent() {
		for (uint32_t y = 0; y < height; ++y) {
			for (uint32_t x = 0; x < width; ++x) {
				cells[y][x].current = false;
			}
		}
	}

public:

#ifdef ROS_VERSION
	nav_msgs::OccupancyGridPtr getGrid();
#endif

};
typedef Handle<LocalMap> LocalMapPtr;

}  // namespace crosbot


#endif /* CROSBOT_LOCALMAP_HPP_ */
