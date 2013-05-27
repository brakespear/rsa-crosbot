/*
 * pointgrid.hpp
 *
 *  Created on: 28/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_POINTGRID_HPP_
#define CROSBOT_POINTGRID_HPP_

namespace crosbot {

namespace ogmbicp {

class PointGrid {
public:
	class Cell : public HandledObject {
	public:
		Cell() {}
	};
	typedef Handle<Cell> CellPtr;

	uint32_t width, height;
	double resolution;
	std::vector< std::vector < CellPtr > > cells;
	Pose origin;

	PointGrid(uint32_t width, uint32_t height, double resolution) :
		width(width), height(height), resolution(resolution),
		origin(-width * resolution / 2.0, -height * resolution / 2.0, 0)
	{
		if (width > 0 && height > 0) {
			cells.resize(height);
			for (uint32_t r = 0; r < height; ++r) {
				cells[r].resize(width);
				for (uint32_t c = 0; c < width; ++c) {
					cells[r][c] = new Cell();
				}
			}
		}
	}

	void centerOn(Point p);
protected:
	void shift(int x, int y);
};

}

} // namespace crosbot




#endif /* CROSBOT_POINTGRID_HPP_ */
