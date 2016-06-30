/*
 * colour.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_COLOUR_HPP_
#define CROSBOT_DATA_COLOUR_HPP_

#include <cstdint>
#include <ostream>
#include <sstream>

#include <crosbot/data/defines.hpp>

#ifdef ROS_VERSION
#include <crosbot/ColourMsg.h>
#endif

namespace crosbot {

/**
 * RGB-A colour in 0-255 format.
 */
struct Colour {
public:
    uint8_t r, g, b, a;

    Colour() : r(0), g(0), b(0), a(255) {}
    Colour(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b), a(255) {}
    Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a) {}
    Colour(const Colour& colour) : r(colour.r), g(colour.g), b(colour.b), a(colour.a) {}

    inline std::string toString() const;

#ifdef ROS_VERSION

    Colour(const ColourMsg& c) : r(c.r), g(c.g), b(c.b), a(c.a) {}

    inline Colour& operator=(const ColourMsg& c) {
        r = c.r; g = c.g; b = c.b; a = c.a;
        return *this;
    }

    inline ColourMsg toROS() const {
        ColourMsg rval;
        rval.r = r; rval.g = g; rval.b = b; rval.a = a;
        return rval;
    }

#endif
};

// Static cast is requires since uint_8 may be a char type
inline std::ostream& operator<<(std::ostream& os, const Colour& c) {
    return os << "Colour("
              << static_cast<uint>(c.r) << ", "
              << static_cast<uint>(c.g) << ", "
              << static_cast<uint>(c.b) << ", "
              << static_cast<uint>(c.a) << ")";
}

inline std::string Colour::toString() const {
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

} // namespace crosbot

#endif /* CROSBOT_DATA_COLOUR_HPP_ */
