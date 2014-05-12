/*
 * localmap.cpp
 *
 *  Created on: 15/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_map/localmap.hpp>
#include <crosbot/utils.hpp>

namespace crosbot {

void LocalMap::shift(int x, int y) {
	// XXX: Consider optimising this operation.
	if (x == 0 && y == 0) {
		// Nothing required to change.
		return;
	} else if (abs(x) >= width || abs(y) >= height) {
		// Shift is so large the whole map should be cleared.
		for (uint32_t r = 0; r < height; ++r) {
			memset(&cells[r][0], 0, width * sizeof(Cell));
		}
	} else {
		// Shift vertically
		if (y < 0) {
			uint32_t r = 0;
			for (; r < height + y; ++r) {
				cells[r] = cells[r - y];
			}
			for (; r < height; ++r) {
				memset(&cells[r][0], 0, width * sizeof(Cell));
			}
		} else if (y > 0) {
			int32_t r = height - 1;
			for (; r >= y; --r) {
				cells[r] = cells[r - y];
			}
			for (; r >= 0; --r) {
				memset(&cells[r][0], 0, width * sizeof(Cell));
			}
		}

		// Shift horizontally
		if (x < 0) {
			for (size_t r = 0; r < height; ++r) {
				std::vector< Cell >& row = cells[r];
				uint32_t c = 0;
				for (; c < width + x; ++c) {
					row[c] = row[c - x];
				}
				memset(&row[c], 0, (-x) * sizeof(Cell));
			}
		} else if (x > 0) {
			for (size_t r = 0; r < height; ++r) {
				std::vector< Cell >& row = cells[r];
				int32_t c = width - 1;
				for (; c >= x; --c) {
					row[c] = row[c - x];
				}
				memset(&row[0], 0, x * sizeof(Cell));
			}
		}
	}

	/**
	 * Update origin of map
	 */
	origin.position.x -= x * resolution;
	origin.position.y -= y * resolution;
}

void LocalMap::update(const PointCloud& cloud) {
	if (cloud.frameID != frameID) {
		ERROR("crosbot::LocalMap: Frame of pointcloud(\"%s\") does not match map(\"%s\")\n",
				cloud.frameID.c_str(), frameID.c_str());
		return;
	}

	clearCurrent();
	int x,y;

	for (size_t i = 0; i < cloud.cloud.size(); ++i) {
		const Point& p = cloud.cloud[i];

		x = (p.x - origin.position.x) / resolution;
		y = (p.y - origin.position.y) / resolution;

		if (x >= 0 && x < width && y >= 0 && y < height) {
			Cell& c = cells[y][x];
			c.current = true;
			c.hits += hitIncrement;
		}
	}

	for (uint32_t y = 0; y < height; ++y) {
		std::vector< Cell >& row = cells[y];

		for (uint32_t x = 0; x < width; ++x) {
			Cell& c = row[x];

			if (!c.current) {
				if (c.hits > missDecrement)
					c.hits = c.hits - missDecrement;
				else
					c.hits = 0;
			} else if (c.hits > maxHits) {
				c.hits = maxHits;
			}
		}
	}
}

ImagePtr LocalMap::getImage() {
	ImagePtr rval = new Image(Image::RGB8, height, width);

	memset(rval->data, 0, rval->dataLength);

	/**
	 * Draw map.
	 */
	for (uint32_t y = 0; y < height; ++y) {
		std::vector< Cell >& row = cells[y];
		uint8_t* pxl = ((uint8_t *)rval->data) + ((height - y - 1) * rval->step);

		for (uint32_t x = 0; x < width; ++x, pxl += 3) {
			Cell& c = row[x];

			if (c.current) {
				pxl[0] = 255;
         } /*else if (c.hits && c.hits == maxHits - 15) {
            pxl[1] = 255;
			}*/ else if (c.hits) {
				uint32_t i = ((c.hits * 255) / maxHits);
				pxl[0] = i;
				pxl[1] = i;
				pxl[2] = i;
			}
		}
	}

	return rval;
}

ImagePtr LocalMap::getImage(Pose robotPose) {
	ImagePtr rval = getImage();

	int RX = (robotPose.position.x - origin.position.x) / resolution,
			RY = (robotPose.position.y - origin.position.y) / resolution;

	if (RX >= 0 && RX < width && RY >= 0 && RY < height) {
		double roll, pitch, yaw, cosYaw, sinYaw;
		robotPose.orientation.getYPR(yaw, pitch, roll);
		cosYaw = cos(yaw);
		sinYaw = sin(yaw);

		/**
		 * Draw robot heading.
		 */
		for (float r = 0; r <= 20; r += 0.95) {
			int pxlX = RX + r * cosYaw,
					pxlY = RY - r * sinYaw; // image is inverted compared to map

			if (pxlX < 0 || pxlX >= width || pxlY < 0 || pxlY >= height)
				continue;

			uint8_t* pxl = ((uint8_t *)rval->data) + pxlY * rval->step + 3 * pxlX;

			pxl[0] = 127;
			pxl[1] = 127;
			pxl[2] = 255;
		}

		/**
		 * Mark robot position
		 */
		uint8_t* pxl = ((uint8_t *)rval->data) + RY * rval->step + 3 * RX;
		pxl[0] = 0;
		pxl[1] = 0;
		pxl[2] = 255;
	}
	return rval;
}

nav_msgs::OccupancyGridPtr LocalMap::getGrid() {
	nav_msgs::OccupancyGridPtr rval(new nav_msgs::OccupancyGrid());

	rval->header.stamp = ros::Time::now();
	rval->header.frame_id = frameID;

	rval->info.width = width;
	rval->info.height = height;
	rval->info.resolution = resolution;

	rval->info.origin = origin.toROS();

	rval->data.resize(width * height);

	size_t cell = 0;
	for (uint32_t r = 0; r < height; ++r) {
		std::vector< Cell >& row = cells[r];

		for (uint32_t c = 0; c < width; ++c, ++cell) {
			rval->data[cell] = (row[c].hits * 100) / maxHits;
		}
	}

	return rval;
}

} // namespace crosbot
