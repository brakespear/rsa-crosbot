/*
 * index.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_INDEX2D_HPP_
#define CROSBOT_GEOMETRY_INDEX2D_HPP_

#include <cmath>
#include <ostream>

#include <crosbot/geometry/defines.hpp>

namespace crosbot {

struct Index2D {
public:
    int x, y;

    Index2D() : x(0), y(0) {}
    Index2D(int x, int y) : x(x), y(y) {}
    Index2D(const Index2D& idx) : x(idx.x), y(idx.y) {}

    inline double distanceTo(const Index2D& next) const {
        int dx = x - next.x, dy = y - next.y;
        return sqrt((double)(dx*dx + dy*dy));
    }

    inline bool operator==(const Index2D& rhs) const {
        return x == rhs.x && y == rhs.y;
    }

    inline bool operator!=(const Index2D& rhs) const {
        return x != rhs.x || y != rhs.y;
    }

    inline Index2D operator+(const Index2D& rhs) const {
        return Index2D(x + rhs.x, y + rhs.y);
    }

    inline Index2D operator-(const Index2D& rhs) const {
        return Index2D(x - rhs.x, y - rhs.y);
    }
};
inline std::ostream& operator<<(std::ostream& os, const Index2D& i) {
    return os << "Index2D(" << i.x << ", " << i.y << ")";
}

} // namespace crosbot

#endif /* CROSBOT_GEOMETRY_INDEX2D_HPP_ */
