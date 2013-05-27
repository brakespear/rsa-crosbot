/*
 * probtable.hpp
 *
 *  Created on: 08/09/2009
 *      Author: rescue
 */

#ifndef CROSBOT_FASTSLAM_PROBTABLE_H_
#define CROSBOT_FASTSLAM_PROBTABLE_H_

#include "constants.hpp"

#include <math.h>
#include <stdlib.h>

namespace crosbot {

namespace fastslam {

/*
 * psd table for raytracing.  Gives probability of sensor readings (s) given actual wall distance(d).
 * Uses a precalculated table because of the complexity of the sensor function
 * Also contains the inverse sensor model, calculated using log odds but not precached
 * It is a good idea to only create one of these
 * It's also a good idea not to mess with it.
 */
class _ProbabilityTable {
public:
	_ProbabilityTable();
	
//	static ProbabilityTable table;
	
	// the actual table.  Sensor readings can be beyond the max length, hence the extra position in the first parameter
	double psdtable[PROBTABLESIZE_MAX+1][PROBTABLESIZE_MAX];
	
	double pfeaturetable[PROBTABLESIZE_MAX];

	// the default gain value.  getProbOf uses this if no gain is passed
	// yes, I know there's 2 copies of the same value, don't ask
//	double gain;
//	double gainval;
	
	// the 3 log odds ratios
	// l_0 is unobserved.  l_occ is observed occupied, l_free is observed unoccupied
	double l_0, l_occ, l_free, l_test;

	// returns P(value | walldist) mixed with a uniform according to gain
	double getProbOf(double value, double walldist, double tgain);
	
	// return P(observing feature value from its map location) mixed with uniform according to gain
	double getFeatureProbOf(double value, double tgain);

	// same as getProbOf but default gain
//	inline double getProbOf(double value, double walldist) {
//		return getProbOf(value, walldist, gain);
//	}
	
	// gets the inverse sensor model, takes the width of the cells to figure out if values == wdist
	// most of the time l_occ and l_free are used directly.
	inline double getInverse(double value, double wdist, double width) {
		double range;
		
		if (wdist >= LASERLENGTH) {
			return l_free;
		}
		if (value > (wdist + width)) {
			return l_0;
		}
		range = fabs(wdist - value);
		
		if (range <= width) {
			return l_occ;
		}
		
		return l_free;
	}

	// log odds helper functions: gets log odds score for prob value
	static inline double lodds(double val) {
		return log(val / (1.0 - val));
	}
	
	// log odds helper functions: gets prob for log odds score
	// watch out you don't get 0
	static inline double invodds(double val) {
//		return 1.0 - (1.0 / (1.0 + exp(val)));
		return (1.0 / (1.0 + exp(-val)));
	}

	// probability helper functions
	// random double between x and y
	static inline double randxy(double x, double y) {
		return( x + (rand() / (double) RAND_MAX) * (y - x));
	}
	
	// random int
	static inline int randintxy(int x, int y) {
		int tmp;
		tmp = x - 1;
		while ((tmp < x) || (tmp > y)) {
			tmp = (int) randxy(x, y + 1);
		}
		return tmp;
	}
	
	// random number according to normal distribution
	static inline double NormRand48(double mean, double sigma) {
		// AHM: I don't know why this works or what it's doing
		// note: sigma is std deviation
		double v1, v2, r, fac;
		
		do
		{
			v1 = randxy(0, 2.0) - 1.0;
			v2 = randxy(0, 2.0) - 1.0;
			r = v1*v1 + v2*v2;
		} while(r >= 1.0);
		
		fac = sqrt(-2.0 * log(r) / r);
		v1 *= fac;
		v2 *= fac;
		
		return(mean + v1 * sigma);
	}
	
	// normal distribution
	static inline double norm(double x, double mean, double sig) {
		return exp(- (pow((x - mean),2) / (2.0 * sig * sig))) / (sig * sqrt(2.0 * M_PI));
	}
	
	// beta distribution
	static inline double beta(double x) {
		return (0.4 + (0.4 * (20000.0 - x) / 20000.0));
	}
};

// The one and hopefully only ProbabilityTable, causes a delay on startup while table is calculated
extern _ProbabilityTable ProbabilityTable;

} // namespace fastslam

} // namespace crosbot

#endif /* CROSBOT_FASTSLAM_PROBTABLE_H_ */
