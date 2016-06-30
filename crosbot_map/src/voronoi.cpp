/*
 * voronoi.cpp
 *
 *  Created on: 22/02/2013
 *      Author: mmcgill
 */

#include <ros/ros.h>
#include <crosbot_map/voronoi.hpp>

#include <crosbot/utils.hpp>

namespace crosbot {

#define SKELETON_THICKNESS		1

ImagePtr VoronoiGrid::getImage() const {
	ImagePtr rval = new Image(Image::RGB8, height, width);
	uint8_t* imgData = (uint8_t*)rval->data;

	unsigned int b = 0, c = 0;
	for (uint32_t y = 0; y < rval->height; ++y) {
		b = 3 * (rval->height - y - 1) * rval->width;
		for (uint32_t x = 0; x < rval->width; ++x, ++c, b += 3) {
			uint8_t cStatus = cells[c].status;
			if (cStatus & VoronoiCell::Skeleton) {
				imgData[b]   = 0;
				imgData[b+1] = 255;
				imgData[b+2] = 255;
			} else if (cStatus == VoronoiCell::Wall) {
				imgData[b]   = 255;
				imgData[b+1] = 255;
				imgData[b+2] = 255;
			} else if (cStatus == VoronoiCell::Restricted) {
				imgData[b]   = 255;
				imgData[b+1] = 0;
				imgData[b+2] = 0;
			} else if (cStatus == VoronoiCell::PatiallyRestricted) {
				imgData[b]   = 255;
				imgData[b+1] = 255;
				imgData[b+2] = 0;
			} else if (cStatus == VoronoiCell::Expansion) {
				imgData[b]   = 0;
				imgData[b+1] = 255;
				imgData[b+2] = 0;
			} else if (cStatus == VoronoiCell::Vacant) {
				imgData[b]   = 0;
				imgData[b+1] = 0;
				imgData[b+2] = 0;
			} else {
				imgData[b]   = 128;
				imgData[b+1] = 128;
				imgData[b+2] = 128;
			}
		}
	}

	return rval;
}

VoronoiGrid::VoronoiGrid(const nav_msgs::OccupancyGrid& grid, const Constraints& c,
    		Pose robot, const nav_msgs::Path& history) :
    TimeStamptedData(Time(grid.header.stamp)),
    width(grid.info.width), height(grid.info.height), resolution(grid.info.resolution),
    origin(grid.info.origin), frame(grid.header.frame_id)
{
	cells.resize(width*height);

	findWalls(grid, c, robot, history);
	calculateCellDistances(c, robot);
	calculateCellStatus(c);
}

void VoronoiGrid::findWalls(const nav_msgs::OccupancyGrid& grid, const Constraints& constraints, Pose robot, const nav_msgs::Path& history) {
	walls.clear();
	this->history.cells.clear();
	skeleton.cells.clear();
	minRobotDistance = INFINITY;

	double connectionDistance = 2 * constraints.restricted / resolution + constraints.connection;

	bool haveRobotPosition = robot.position.isFinite();
	Index2D robotCell;
	if (haveRobotPosition) {
		robotCell = Index2D((robot.position.x - origin.position.x) / resolution,
				(robot.position.y - origin.position.y) / resolution);
	}

	uint32_t c = 0;
	for (uint32_t y = 0; y < height; ++y) {
		for (uint32_t x = 0; x < width; ++x, ++c) {
			int gd = grid.data[c];
			if (gd > constraints.minOccupiedProb) {
				Index2D idx(x, y);
				VoronoiCell& vc = cells[c];

				vc.status |= VoronoiCell::Wall;
				vc.distanceFromWall = 0;
				vc.nearestWallCell = idx;

				if (haveRobotPosition) {
					vc.distanceToRobot = idx.distanceTo(robotCell);
					if (vc.distanceToRobot < minRobotDistance) {
						minRobotDistance = vc.distanceToRobot;
					}
				}

				Wall *wFound = NULL;
				for (size_t w = 0; w < walls.size(); ++w) {
					Wall& wall = walls[w];

					if (wall.connectsTo(idx, connectionDistance)) {
						if (wFound == NULL) {
							wFound = &wall;
							wall.cells.push_back(idx);
						} else {
							wFound->addAll(wall);
							walls.erase(walls.begin() + w);
							--w;
						}
					}
				}
				if (wFound == NULL) {
					walls.push_back(Wall(idx));
				}
			}
		}
	}

	for (size_t w = 0; w < walls.size(); ++w) {
		Wall& wall = walls[w];
		for (size_t wc = 0; wc < wall.cells.size(); ++wc) {
			Index2D& cIdx = wall.cells[wc];
			uint32_t c = cIdx.y * width + cIdx.x;
			cells[c].nextNearestWall = w;
		}
	}

	// TODO: populate history wall
}

#define		CELL_CONSTRAINTS(C)		double restrictC = ceil((C).restricted / resolution),	\
										partialC = ceil((C).partial / resolution),	\
										maxExpandC = (C).expand / resolution;		\
										if (partialC < restrictC)							\
											partialC = restrictC;							\
											maxExpandC += partialC;

void VoronoiGrid::calculateCellDistances(const Constraints& constraints, Pose robot) {
	bool haveRobotPosition = robot.position.isFinite();
	Index2D robotCell, nearestCell;
	if (haveRobotPosition) {
		robotCell = Index2D((robot.position.x - origin.position.x) / resolution,
				(robot.position.y - origin.position.y) / resolution);
	}

	CELL_CONSTRAINTS(constraints);

#ifdef CROSBOT_VORONOI_COMPLETE
	uint32_t c = 0;
	for (uint32_t y = 0; y < height; ++y) {
		for (uint32_t x = 0; x < width; ++x, ++c) {
			VoronoiCell& vc = cells[c];
			Index2D idx(x ,y);

			if (haveRobotPosition) {
				vc.distanceToRobot = idx.distanceTo(robotCell);
			}
			vc.historyDistance = history.getVoronoiDistance(idx, nearestCell);

			// TODO: CROSBOT_VORONOI_COMPLETE
		}
	}
#else

	/*
	 * Speed up.  For every wall cell we search within a window around it to find
	 * any cells to which it is closer than previous walls cells.
	 */
	int window = ceil(maxExpandC + 1);
	for (size_t w = 0; w < walls.size(); ++w) {
		Wall& wall = walls[w];

		for (size_t wc = 0; wc < wall.cells.size(); wc++) {
			Index2D wallCellIdx = wall.cells[wc];
			int wcIdxC = wallCellIdx.y*width+wallCellIdx.x;

			VoronoiCell& wallCell = cells[wcIdxC];
			wallCell.distanceFromWall = 0;
			wallCell.nearestWall = w;
			wallCell.nearestWallCell = wallCellIdx;

			for (int dy = -window; dy <= window; dy++) {
				Index2D cellIdx(-1, wallCellIdx.y + dy);
				if (cellIdx.y < 0 || cellIdx.y >= (int)height)
					continue;

				for (int dx = -window; dx <= window; dx++) {
					cellIdx.x = wallCellIdx.x + dx;
					if (cellIdx.x < 0 || cellIdx.x >= (int)width)
						continue;
					int cellIdxC = cellIdx.y * width + cellIdx.x;
					VoronoiCell& cell = cells[cellIdxC];
					if (cell.status == VoronoiCell::Wall)
						continue;
					if (haveRobotPosition) {
						cell.distanceToRobot = cellIdx.distanceTo(robotCell);
					}
					double d = cellIdx.distanceTo(wallCellIdx);
					if (d >= cell.nextNearestWallDistance)
						continue;

					if (d < cell.distanceFromWall) {
						if (cell.nearestWall != (int)w) {
							cell.nextNearestWallDistance = cell.distanceFromWall;
							cell.nextNearestWall = cell.nearestWall;
						}

						cell.nearestWall = w;
						cell.distanceFromWall = d;
						cell.nearestWallCell = wallCellIdx;
					} else if (cell.nearestWall != (int)w) {
						cell.nextNearestWallDistance = d;
						cell.nextNearestWall = w;
					}
				}
			}
		}
	}

	// Locate all cells that may be affected by the history.
	for (size_t hc = 0; hc < history.cells.size(); ++hc) {
		Index2D histCellIdx = history.cells[hc];
		VoronoiCell* histCell = &cells[histCellIdx.y*width+histCellIdx.x];
		histCell->historyDistance = 0;

		for (int dy = -window; dy <= window; dy++) {
			Index2D cellIdx(-1, histCellIdx.y + dy);
			if (cellIdx.y < 0 || cellIdx.y >= (int)height)
				continue;

			for (int dx = -window; dx <= window; dx++) {
				cellIdx.x = histCellIdx.x + dx;
				if (cellIdx.x < 0 || cellIdx.x >= (int)width)
					continue;

				VoronoiCell* cell = &cells[cellIdx.y * width + cellIdx.x];
				double d = cellIdx.distanceTo(histCellIdx);

				if (d < cell->historyDistance)
					cell->historyDistance = d;
			}
		}
	}
#endif
}

void VoronoiGrid::calculateCellStatus(const Constraints& constraints) {
	CELL_CONSTRAINTS(constraints);

    if (minRobotDistance < restrictC) {
        WARN("VoronoiGrid::calculateCellStatus: Robot close to wall, reducing restricted distance.\n");
    }
//    skeleton.cells.clear();

    int c = 0;
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++, c++) {
            VoronoiCell& cell = cells[c];

            // Calculate cells status.
            if (cell.status == VoronoiCell::Wall) {
                // leave status as is
            } else if (cell.distanceFromWall <= restrictC) {
                if (minRobotDistance <= restrictC &&
                        cell.distanceFromWall >= minRobotDistance && cell.distanceToRobot <= cell.distanceFromWall) {
                    cell.status = VoronoiCell::PatiallyRestricted;
                } else {
                    cell.status = VoronoiCell::Restricted;
                }
            } else if (cell.distanceFromWall <= partialC) {
                cell.status = VoronoiCell::PatiallyRestricted;
            } else if (cell.distanceFromWall <= maxExpandC) {
                cell.status = VoronoiCell::Expansion;
            } else if (cell.historyDistance <= maxExpandC) {
                cell.status = VoronoiCell::Expansion;
            } else {
                cell.status = VoronoiCell::Vacant;
            }

            // Determine if cell is on skeleton
            if (cell.distanceFromWall <= maxExpandC) {
                if (cell.nextNearestWallDistance - cell.distanceFromWall <= SKELETON_THICKNESS) {
                    if (constraints.historyEffect != Constraints::EraseHorizon || cell.historyDistance > maxExpandC) {
                        cell.status |= VoronoiCell::Skeleton;
                        skeleton.cells.push_back(Index2D(x,y));
                    }
                }
            } else if (cell.distanceFromWall <= maxExpandC + SKELETON_THICKNESS) {
                if (cell.historyDistance > maxExpandC ||
                        (constraints.historyEffect == Constraints::KeepMidLine &&
                        		cell.nextNearestWallDistance - cell.distanceFromWall <= SKELETON_THICKNESS)) {
                    cell.status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(x,y));
                }
            } else if (cell.historyDistance <= maxExpandC + SKELETON_THICKNESS) {
                if (cell.historyDistance > maxExpandC || (constraints.historyEffect == Constraints::KeepMidLine &&
                            cell.nextNearestWallDistance - cell.distanceFromWall <= SKELETON_THICKNESS)) {
                    cell.status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(x,y));
                }
            }
        }
    }

    // Now generate the skeletons leading into/out of dead ends
    for (unsigned int y = 1; y < height - 1; y++) {
        for (unsigned int x = 1; x < width - 1; x++) {
            int c = y * width + x;
            VoronoiCell *cell = &cells[c], *up, *down, *left, *right,
                    *upleft, *upright, *downleft, *downright;

            if ((cell->status != VoronoiCell::Expansion && cell->status != VoronoiCell::PatiallyRestricted) ||
            		cell->distanceFromWall <= restrictC ||
                    cell->distanceFromWall == INFINITY ||
                    (cell->historyDistance <= maxExpandC ||
                            (cell->distanceFromWall > maxExpandC + SKELETON_THICKNESS &&
                                    cell->historyDistance <= maxExpandC))) {
                continue;
            }

            up = &cells[c-width]; down = &cells[c+width];
            left = &cells[c-1]; right = &cells[c+1];
            upleft = &cells[c-width-1]; upright = &cells[c-width+1];
            downleft = &cells[c+width-1]; downright = &cells[c+width+1];

            double maxForDrop = cell->distanceFromWall - CROSBOT_MATH_ERROR,
                    minForRise = cell->distanceFromWall + CROSBOT_MATH_ERROR;
            if ((up->distanceFromWall < maxForDrop && down->distanceFromWall < maxForDrop) ||
                (left->distanceFromWall < maxForDrop && right->distanceFromWall < maxForDrop) ||
                (upleft->distanceFromWall < maxForDrop && downright->distanceFromWall < maxForDrop) ||
                (downleft->distanceFromWall < maxForDrop && upright->distanceFromWall < maxForDrop)) {
                cell->status |= VoronoiCell::Skeleton;
                skeleton.cells.push_back(Index2D(x,y));
            } else if (!(up->distanceFromWall > minForRise || down->distanceFromWall > minForRise ||
                    left->distanceFromWall > minForRise || right->distanceFromWall > minForRise)) {
                cell->status |= VoronoiCell::Skeleton;
                skeleton.cells.push_back(Index2D(x,y));
            }
        }
    }

    if (constraints.make4connected)
        makeSkeleton4Connected(constraints);
}

// Macros for making skeleton 4 connected
#define VALID_SKELETON_CELL(C)  ((C)->status == VoronoiCell::Expansion || (C)->status == VoronoiCell::Vacant || (C)->status == VoronoiCell::PatiallyRestricted)

#define COUNT_LINES()                                                               \
        lineAbove = lineBelow = lineLeft = lineRight = 0;                           \
        if (upleft->status & VoronoiCell::Skeleton) { ++lineAbove; ++lineLeft; }    \
        if (up->status & VoronoiCell::Skeleton) { ++lineAbove; }                    \
        if (upright->status & VoronoiCell::Skeleton) { ++lineAbove; ++lineRight; }  \
        if (left->status & VoronoiCell::Skeleton) { ++lineLeft; }                   \
        if (right->status & VoronoiCell::Skeleton) { ++lineRight; }                 \
        if (downleft->status & VoronoiCell::Skeleton) { ++lineBelow; ++lineLeft; }  \
        if (down->status & VoronoiCell::Skeleton) { ++lineBelow; }                  \
        if (downright->status & VoronoiCell::Skeleton) { ++lineBelow; ++lineRight; }

#define CELL_DUPLICATE_HORIZON()                                                    \
        (lineAbove == 0 && lineBelow == 3) || (lineAbove == 3 && lineBelow == 0) || \
        (lineLeft == 0 && lineRight == 3) || (lineLeft == 3 && lineRight == 0) ||   \
        (lineLeft == 3 && lineAbove == 3) || (lineLeft == 3 && lineBelow == 3) ||   \
        (lineRight == 3 && lineAbove == 3) || (lineRight == 3 && lineBelow == 3) || \
        (lineAbove == 0 && lineRight == 0 && lineLeft == 2 && lineBelow == 2) ||    \
        (lineAbove == 0 && lineRight == 2 && lineLeft == 0 && lineBelow == 2) ||    \
        (lineAbove == 2 && lineRight == 0 && lineLeft == 2 && lineBelow == 0) ||    \
        (lineAbove == 2 && lineRight == 2 && lineLeft == 0 && lineBelow == 0) ||    \
        (lineAbove == 0 && lineRight == 0 && lineLeft == 0 && lineBelow == 0)

void VoronoiGrid::makeSkeleton4Connected(const Constraints& constraints) {
    if (height < 2 || width < 2)
        return;

    VoronoiCell hiddenCell;
    hiddenCell.status = VoronoiCell::NotVisible;
    VoronoiCell *upleft = NULL, *up = NULL, *upright = NULL,
                *left = NULL, *cell = NULL, *right = NULL,
                *downleft = NULL, *down = NULL, *downright = NULL;

    // Join up cells
    int c;
    for (unsigned int hc = 0; hc < skeleton.cells.size(); hc++) {
        Index2D xy = skeleton.cells[hc];
        if (xy.y == (int)height - 1)
            continue;
        c = xy.y * width + xy.x; cell = &cells[c];
        down = &cells[c+width];
        if (down->status & VoronoiCell::Skeleton)
            continue;

        if (xy.x == 0) {
            downright = &cells[c+width+1];
            if (!(downright->status & VoronoiCell::Skeleton))
                continue;

            right = &cells[c+1];

            if (right->status & VoronoiCell::Skeleton)
                continue;

            if (VALID_SKELETON_CELL(down)) {
                if (VALID_SKELETON_CELL(right) &&
                        (right->nextNearestWallDistance - right->distanceFromWall < down->nextNearestWallDistance - down->distanceFromWall)) {
                    right->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x+1, xy.y));
                } else {
                    down->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x, xy.y+1));
                    continue;
                }
            } else if (VALID_SKELETON_CELL(right)) {
                right->status |= VoronoiCell::Skeleton;
                skeleton.cells.push_back(Index2D(xy.x+1, xy.y));
            }
        } else if (xy.x == (int)width-1) {
            downleft = &cells[c+width-1];
            if (!(downleft->status & VoronoiCell::Skeleton))
                continue;

            left = &cells[c-1];

            if (left->status & VoronoiCell::Skeleton)
                continue;

            if (VALID_SKELETON_CELL(down)) {
                if (VALID_SKELETON_CELL(left) &&
                        (left->nextNearestWallDistance - left->distanceFromWall < down->nextNearestWallDistance - down->distanceFromWall)) {
                    left->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x-1, xy.y));
                } else {
                    down->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x, xy.y+1));
                    continue;
                }
            } else if (VALID_SKELETON_CELL(left)) {
                left->status |= VoronoiCell::Skeleton;
                skeleton.cells.push_back(Index2D(xy.x-1, xy.y));
            }
        } else {
            bool leftDeferred = false;
            left = &cells[c-1];
            downleft = &cells[c+width-1];
            if (!(left->status & VoronoiCell::Skeleton) && (downleft->status & VoronoiCell::Skeleton)) {
                if (VALID_SKELETON_CELL(down)) {
                    if (VALID_SKELETON_CELL(left) &&
                            (left->nextNearestWallDistance - left->distanceFromWall < down->nextNearestWallDistance - down->distanceFromWall)) {
                        leftDeferred = true;
                    } else {
                        down->status |= VoronoiCell::Skeleton;
                        skeleton.cells.push_back(Index2D(xy.x, xy.y+1));
                        continue;
                    }
                } else if (VALID_SKELETON_CELL(left)) {
                    left->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x-1, xy.y));
                }
            }

            right = &cells[c+1];
            downright = &cells[c+width+1];
            if (!(right->status & VoronoiCell::Skeleton) && (downright->status & VoronoiCell::Skeleton)) {
                if (VALID_SKELETON_CELL(down)) {
                    if (VALID_SKELETON_CELL(right) &&
                            (right->nextNearestWallDistance - right->distanceFromWall < down->nextNearestWallDistance - down->distanceFromWall)) {
                        right->status |= VoronoiCell::Skeleton;
                        skeleton.cells.push_back(Index2D(xy.x+1, xy.y));
                    } else {
                        down->status |= VoronoiCell::Skeleton;
                        skeleton.cells.push_back(Index2D(xy.x, xy.y+1));
                        continue;
                    }
                } else if (VALID_SKELETON_CELL(right)) {
                    right->status |= VoronoiCell::Skeleton;
                    skeleton.cells.push_back(Index2D(xy.x+1, xy.y));
                }
            }

            if (leftDeferred && !(down->status & VoronoiCell::Skeleton)) {
                left->status |= VoronoiCell::Skeleton;
                skeleton.cells.push_back(Index2D(xy.x-1, xy.y));
            }
        }
    }

    // Erase extra thick cells
    c = 0;
    int lineAbove, lineBelow, lineLeft, lineRight;
    for (unsigned int hc = 0; hc < skeleton.cells.size(); hc++) {
        Index2D xy = skeleton.cells[hc];
        c = xy.y * width + xy.x;

        cell = &cells[c];
        if (xy.y == 0) {
            upleft = up = upright = &hiddenCell;
            down = &cell[c+width];
            if (xy.x == 0) {
                left = downleft = &hiddenCell;
                right = &cells[c+1]; downright = &cells[c+width+1];
            } else if (xy.x == (int)width - 1) {
                left = &cells[c-1]; downleft = &cells[c+width-1];
                right = downright = &hiddenCell;
            } else {
                left = &cells[c-1]; downleft = &cells[c+width-1];
                right = &cells[c+1]; downright = &cells[c+width+1];
            }
        } else if (xy.y == (int)height - 1) {
            downleft = down = downright = &hiddenCell;
            up = &cells[c-width];
            if (xy.x == 0) {
                left = upleft = &hiddenCell;
                right = &cells[c+1]; upright = &cells[c-width+1];
            } else if (xy.x == (int)width - 1) {
                left = &cells[c-1]; upleft = &cells[c-width-1];
                right = upright = &hiddenCell;
            } else {
                left = &cells[c-1]; upleft = &cells[c-width-1];
                right = &cells[c+1]; upright = &cells[c-width+1];
            }
        } else {
            up = &cells[c-width]; down = &cells[c+width];
            if (xy.x == 0) {
                upleft = left = downleft = &hiddenCell;
                upright = &cells[c-width+1]; right = &cells[c+1]; downright = &cells[c+width+1];
            } else if (xy.x == (int)width - 1) {
                upright = right = downright = &hiddenCell;
                upleft = &cells[c-width-1]; left = &cells[c-1]; downleft = &cells[c+width-1];
            } else {
                upleft = &cells[c-width-1]; left = &cells[c-1]; downleft = &cells[c+width-1];
                upright = &cells[c-width+1];right = &cells[c+1]; downright = &cells[c+width+1];
            }
        }

        COUNT_LINES();
        if (CELL_DUPLICATE_HORIZON()) {
            cell->status &= ERASE_SKELETON;
            // remove from skeleton
            skeleton.cells.erase(skeleton.cells.begin() + hc);
            --hc;
        }
    }

    // Erase tails entering walls
    std::vector<int> endDists(cells.size());

    std::vector<Index2D> neighbours;
    {
        neighbours.push_back(Index2D(-1, -1));
        neighbours.push_back(Index2D( 0, -1));
        neighbours.push_back(Index2D( 1, -1));

        neighbours.push_back(Index2D(-1, 0));
        neighbours.push_back(Index2D( 1, 0));

        neighbours.push_back(Index2D(-1,  1));
        neighbours.push_back(Index2D( 0,  1));
        neighbours.push_back(Index2D( 1,  1));
    }

    memset(&endDists[0], 0, sizeof(int)*endDists.size());
    for (int hc = 0; hc < (int)skeleton.cells.size(); hc++) {
        Index2D& hcIdx = skeleton.cells[hc];

        if (isHorizonEnd(hcIdx.x, hcIdx.y)) {
            c = hcIdx.y * width + hcIdx.x;
            bool nextToWall = false;

            for (unsigned int n = 0; !nextToWall && n < neighbours.size(); n++) {
                Index2D neighbour = hcIdx + neighbours[n];

                if (neighbour.x < 0 || neighbour.x >= (int)width ||
                        neighbour.y < 0 || neighbour.y >= (int)height)
                    continue;

                int nc = neighbour.y * width + neighbour.x;
                unsigned int status = cells[nc].status;

                if (status == VoronoiCell::Wall || status == VoronoiCell::Restricted)
                    nextToWall = true;
            }

            if (nextToWall) {
                hc -= deleteHorizonEnd(hcIdx.x, hcIdx.y, 0, constraints);
                if (hc < 0)
                    hc = -1;
            }
        }
    }

    // Delete orphan sections
    std::vector<char> marks(cells.size());
    memset(&marks[0], 0, marks.size()*sizeof(char));

    for (int hc = 0; hc < (int)skeleton.cells.size(); hc++) {
        Index2D& hcIdx = skeleton.cells[hc];
        int c = hcIdx.y * width + hcIdx.y;

        if (marks[c] != 0) {
            continue;
        }

        int n = getConnectedHorizonSize(marks, hcIdx.x, hcIdx.y);

        if (n > 0 && n < constraints.orphanThreshold) {
//            printf("Deleting orphan(%d)\n", n);
            hc -= deleteConnectedHorizon(hcIdx.x, hcIdx.y);
            if (hc < 0)
                hc = -1;
        }
    }
}

} // namespace crosbot

