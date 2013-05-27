/*
 * consants.hpp
 *
 *  Created on: 09/09/2009
 *      Author: rescue
 */

#ifndef CROSBOT_FASTSLAM_CONSTANTS_HPP_
#define CROSBOT_FASTSLAM_CONSTANTS_HPP_

namespace crosbot {

namespace fastslam {

// The dimensions[m] of a cell in the occupancy grid.
#define CELL_WIDTH 0.05
#define CELL_HEIGHT 0.05

// max distance entry of the table
#define PROBTABLESIZE_MAX		1000
#define TABLERESOLUTION			20

// Values for laser
#define LASERLENGTH				5000
#define LASERSIGMA				30

#define FEATURESIGMA			300

#define DEFAULT_GAINVALUE		0.0003
#define DEFAULT_UNOBSERVEDLODDS	0.5
#define DEFAULT_OCCUPIEDLODDS	0.55
#define DEFAULT_FREELODDS		0.49

// Thresholds for HeightMaps
#define HEIGHT_MAX				2.5
#define HEIGHT_THRESHOLD		-10.0
#define SEARCHED_DISTANCE         1.5

#define DEFAULT_SNAP_THRESHOLD		1.0

// Values for the HeightMultiMap
#define MULTIMAP_COLUMNS		100
#define MULTIMAP_ROWS			100
#define HEIGHTMAP_COLUMNS		100
#define HEIGHTMAP_ROWS			100

#define RAY_TRACE_EXTEND		5
#define RAY_STEP_SIZE			0.040

// Values for the particles
#define KTH						(1.0 * (M_PI / 60.0) / (2.0 * M_PI)) // rad/rad
#define	KR 						(5.0 * 1.0 / 100.0)                  // m/m
#define	KRXY					(0.0)                                // m/m
#define KDR						((M_PI / 90.0) / 0.20)               // rad/m
// #define KSH						(1.0 / (2.0 * M_PI))
#define	KSH 					(5.0 * 1.0 / 100.0)                  // m/m
#define	KSHXY 					(0.0)                                // m/m
#define KSLP					(0.0)								 // m/rad

#define SMEAR_X					0.1
#define SMEAR_Y					0.1
#define SMEAR_Z					0.05
//#define SMEAR_YAW				(M_PI/30.0) //(6.0*M_PI/180.0)
#define SMEAR_YAW				DEG2RAD(6.0)

#define DEFAULT_NUMPARTICLES	200

#define DEFAULT_DISTANCETHRESHOLD	0.2
#define DEFAULT_ANGULARTHRESHOLD	15 // Degrees

} // namespace fastslam

} // namespace crosbot

#endif /* CROSBOT_FASTSLAM_CONSTANTS_HPP_ */
