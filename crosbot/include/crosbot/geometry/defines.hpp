/*
 * defines.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_DEFINES_HPP_
#define CROSBOT_GEOMETRY_DEFINES_HPP_

#ifndef BT_USE_DOUBLE_PRECISION
#define BT_USE_DOUBLE_PRECISION
#endif

namespace crosbot {

#define CROSBOT_MATH_ERROR   0.00001

/**
 * Macros for converting between degrees and radians
 */
#define DEG2RAD(A)      ((A)*M_PI/180)
#define RAD2DEG(A)      ((A)*180/M_PI)

/**
 * Macro to normalise an angle to between Pi and -Pi.
 */
#define NORMALISE_ANGLE(_A_)    if (_A_ != INFINITY && _A_ != -INFINITY && _A_ != NAN) {       \
                                    while (_A_ > M_PIl) { _A_ -= 2* M_PIl; }                    \
                                    while (_A_ < -M_PIl) { _A_ += 2*M_PIl; }                    \
                                }

/**
 * Common frames of reference used throughout casros.
 */
#define FRAME_SENSOR        0
#define FRAME_ROBOT         1
#define FRAME_WORLD         2

} // namespace crosbot


#endif /* CROSBOT_GEOMETRY_DEFINES_HPP_ */
