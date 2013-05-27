/*
 * probtable.cpp
 *
 *  Created on: 08/09/2009
 *      Author: rescue
 */
#include <crosbot_fastslam/probtable.hpp>
#include <stdio.h>
#include <iostream>
#include <fstream>

namespace crosbot {

namespace fastslam {

_ProbabilityTable ProbabilityTable;

/*
 * the constructor creates the probability table
 */
_ProbabilityTable::_ProbabilityTable() {
//	gain = DEFAULT_GAINVALUE;
	
	l_0 = lodds(DEFAULT_UNOBSERVEDLODDS);
	l_occ = lodds(DEFAULT_OCCUPIEDLODDS);
	l_free = lodds(DEFAULT_FREELODDS);
	l_test = l_0;

	int d;
	int s;
	double sum;
	double lf = .02 / 1000.0; // false positive term
	
	double sig = LASERSIGMA; // sigma of the laser
	double dval;
	double sval;
	// MPI
	for (d = 0; d < PROBTABLESIZE_MAX; d++) {
		// get the value for the d index
		dval = (TABLERESOLUTION * d) + TABLERESOLUTION / 2.0;
		sum = 1.0; // need to store leftover for row
		for (s = 0; s < PROBTABLESIZE_MAX; s++) {
			// get the value for the s index
			sval = (TABLERESOLUTION * s) + (TABLERESOLUTION) / 2.0;
			psdtable[s][d] = 0; // initialize probability
			
			// if sensor < actual distance need false pos term
			if (s < d) {
				psdtable[s][d] = ((double) TABLERESOLUTION) // change to real probability
						* (lf * exp(- lf * sval)                // false positive
							+ beta(dval) * norm(sval, dval, sig) * exp(- lf * sval));// false negative and normal curve
			} else {

				// no false positive after peak
				psdtable[s][d] = ((double) TABLERESOLUTION)
						* (beta(dval) * norm(sval, dval, sig) * exp(- lf * dval));

			}
			// decrease sum
			sum -= psdtable[s][d];
			// mix in uniform with gain plus scaling factor of 10 to prevent underflow
			// AHM: gain is set dynamically so it can change now, no longer put into table
			// psdtable[s][d] = (psdtable[s][d]);
		}
		// don't make final prob < 0
		if (sum < 0.0) {
			sum = 0.0;
		}
		
		// prob of no reading
		psdtable[PROBTABLESIZE_MAX][d] = (sum);
//		fprintf(stdout, "\r# Generating Table ... (%.2f%%)", ((float) (d / (float) PROBTABLESIZE_MAX)) * 100);
	}
//	fprintf(stdout, "\r# Generating Table ... (%.2f%%)\n", 100.0);

/*
	// AHM: Matt, never delete this again!
	// use this to output the table for testing
	std::ofstream ofile;
	ofile.open("psdtable.txt");
	for (d = 0; d < PROBTABLESIZE_MAX; d += 10) {
	//		ofile << "-1, ";
		for (s = 0; s < PROBTABLESIZE_MAX; s+= 10) {
			ofile << psdtable[s][d] << ", ";
		}
		ofile << psdtable[PROBTABLESIZE_MAX][d] << std::endl;
	}

	ofile.close();
*/

	// create feature probability table, currently just a normal
	sig = FEATURESIGMA;
	for (s = 0; s < PROBTABLESIZE_MAX; s++) {
		sval = (TABLERESOLUTION * s) + (TABLERESOLUTION) / 2.0;
		pfeaturetable[s] = ((double) TABLERESOLUTION) * norm(sval, 0, sig);
	}

}

/*
 * returns P(value | walldist) mixed with a uniform according to gain
 * This doesn't return an actual probability, it's modified because of underflow
 */
double _ProbabilityTable::getProbOf(double value, double walldist, double tgain) {
	int sindx;
	int dindx;

	sindx = (int) (((double) value) / ((double) TABLERESOLUTION));
	dindx = (int) (((double) walldist) / ((double) TABLERESOLUTION));  //tableresolution is an int

	if (sindx < 0) sindx = 0;
	if (dindx < 0) dindx = 0;
	// check for outside of table
	if (sindx > PROBTABLESIZE_MAX) {
		sindx = PROBTABLESIZE_MAX;
	}
	if (dindx >= PROBTABLESIZE_MAX) {
		dindx = PROBTABLESIZE_MAX - 1;
	}

	return 100.0 * (tgain * psdtable[sindx][dindx] + (1.0 - tgain) * (20.0 / (double) (20000.0 + 20.0)));
//		return psdtable[sindx][dindx];

}

// return P(observing feature value from its map location) mixed with uniform according to gain
// Must use values compatible with getProbOf
double _ProbabilityTable::getFeatureProbOf(double value, double tgain) {
	int sindx;

	sindx = (int) (((double) value) / ((double) TABLERESOLUTION));

	if (sindx < 0) sindx = 0;
	if (sindx > PROBTABLESIZE_MAX) {
		sindx = PROBTABLESIZE_MAX;
	}

	return 100.0 * (tgain * pfeaturetable[sindx] + (1.0 - tgain) * (20.0 / (double) (20000.0)));
}

} // namespace fastslam

} // namespace crosbot
