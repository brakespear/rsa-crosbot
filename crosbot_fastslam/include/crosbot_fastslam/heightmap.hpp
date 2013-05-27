/*
 * heightmap.hpp
 *
 *  Created on: 08/09/2009
 *      Author: rescue
 */

#ifndef CROSBOT_FASTSLAM_HEIGHTMAP_HPP_
#define CROSBOT_FASTSLAM_HEIGHTMAP_HPP_

#include <crosbot_fastslam/probtable.hpp>
#include <crosbot/data.hpp>
#include <crosbot/utils.hpp>


namespace crosbot {

namespace fastslam {

#define ALIGN_SIZE 120

class MapCloud;
typedef Handle<MapCloud> MapCloudPtr;
struct MapCloud : public PointCloud {
	Pose robot;	// The pose of the robot in the map/particle
	Pose odometry;  // The pose of the robot as given by the odometry
	Pose sensor; // The pose of the sensor relative to the base frame for the robot

	MapCloud() : PointCloud() {}

	MapCloud(const PointCloud& cld, Pose odometry = Pose(), Pose sensor = Pose()) :
		PointCloud(cld), odometry(odometry), sensor(sensor)
	{}

	MapCloud(Pose robot, const MapCloud& cld) :
		PointCloud("/particle", cld, robot), robot(robot), odometry(cld.odometry), sensor(cld.sensor)
	{}

	inline Pose getAbsoluteSensorPose() {
		Pose rval = robot.getTransform() * sensor.getTransform();
		return rval;
	}
};

struct FastSLAMParameters {
public:
	uint32_t numberOfParticles;
	double gain;
	float initialOccupiedOdds;
	double initialZ;

	double updateThresholdXY, updateThresholdTheta;

	double minDistanceBetweenSnaps;

	Pose searchPose;
	double searchDistance, searchFOV;
	double maxSensorRange;

	unsigned int mapRows, mapColumns, patchRows, patchColumns;

	double maxHeight, minHeight;

	FastSLAMParameters() {
		numberOfParticles = DEFAULT_NUMPARTICLES;
		gain = DEFAULT_GAINVALUE;

		initialOccupiedOdds = ProbabilityTable.lodds(DEFAULT_UNOBSERVEDLODDS);
		initialZ = 0;

		updateThresholdXY = DEFAULT_DISTANCETHRESHOLD;
		updateThresholdTheta = DEG2RAD(DEFAULT_ANGULARTHRESHOLD);

		minDistanceBetweenSnaps = DEFAULT_SNAP_THRESHOLD;

		searchDistance = -1;
		searchFOV = DEG2RAD(60);

		mapRows = MULTIMAP_ROWS; mapColumns = MULTIMAP_COLUMNS;
		patchRows = HEIGHTMAP_ROWS; patchColumns = HEIGHTMAP_COLUMNS;

		minHeight = 0; maxHeight = INFINITY;
		maxSensorRange = INFINITY;
	}
};

/*
 * The actual map values
 * Keep this as small as possible
 */
struct HeightVal {
public:
	float height; // height of the cell
	float prob; // occupancy log odds score

	int observations; // # of observations
	int searches;	// # of searches in the cell

	inline bool occupied(double z) {
		return observations > 0 && z <= height && prob > ProbabilityTable.l_test;
	}

	inline void hit(double z) {
		observations++;
		if (height < z)
			height = z + 0.01;
		prob = prob + ProbabilityTable.l_occ - ProbabilityTable.l_0;
	}

	inline void miss(double z) {
		observations++;
//		if (height > z)
//			height = z - 0.01;
		prob = prob + ProbabilityTable.l_free - ProbabilityTable.l_0;
	}
};

class HeightMap; class HeightMultiMap;
typedef Handle<HeightMap> HeightMapPtr;
class HeightMap : public HandledObject {
public:
	HeightMultiMap *owner;

	unsigned int rows, columns;
	float xOrig, yOrig;
	float width, height;

	HeightVal* hlist;

	HeightMap(unsigned int rows, unsigned int cols, float initialOdds) :
		owner(NULL), rows(rows), columns(cols), hlist(NULL)
	{
		initMap(initialOdds);
	}

	HeightMap(HeightMapPtr cpy) :
		owner(NULL), rows(cpy->rows), columns(cpy->columns),
		xOrig(cpy->xOrig), yOrig(cpy->yOrig), hlist(NULL)
	{
		initMap();
		if (hlist == NULL || cpy->hlist == NULL) {
			ERROR("HeightMap: Attempted to copy empty height map.\n");
		} else {
			memcpy(hlist, cpy->hlist, (rows * columns) * sizeof(HeightVal));
		}
	}

	~HeightMap() {
		if (hlist != NULL) {
			free(hlist);
		}
	}

	inline void initMap() {
		width = columns * CELL_WIDTH;
		height = rows * CELL_HEIGHT;
		hlist = (HeightVal *) malloc(sizeof(HeightVal) * rows * columns);
	}

	inline void initMap(float initialOdds) {
		width = columns * CELL_WIDTH;
		height = rows * CELL_HEIGHT;
		hlist = (HeightVal *) malloc(sizeof(HeightVal) * rows * columns);
		zeroMap(initialOdds);
	}

	inline void zeroMap(float initialOdds) {
		unsigned int count = rows * columns;
		for (unsigned int i = 0; i < count; i++) {
			hlist[i].height = 0.0;
			hlist[i].prob = initialOdds;
			hlist[i].observations = 0;
			hlist[i].searches = 0;
		}
	}

	/**
	 * Returns a cell based upon its index
	 */
	inline HeightVal *getIndex(int idx) {
			return &(hlist[idx]);
	}

	// returns a particular cell by row and column
	inline HeightVal *getByIJ(unsigned int i, unsigned int j) {
		if ((i >= rows) || (j >= columns))
			return NULL;
		return getIndex(i * columns + j);
	}

	// returns a cell by x and y value
	inline HeightVal *getByXY(float x, float y) {
		unsigned int r = (y - yOrig) / CELL_HEIGHT;
		unsigned int c = (x - xOrig) / CELL_WIDTH;
		return getByIJ(r, c);
	};

	// returns the row and column of an x and y position
	inline void getRowColumn(float x, float y, unsigned int &row, unsigned int &column) {
		column = (x - xOrig) / CELL_WIDTH;
		row = (y - yOrig) / CELL_HEIGHT;
	};

	// checks if a location is in this patch
	// NOTE: this function isn't currently used since it's more efficient not to make invalid calls
	// This means it hasn't been tested in use
	inline bool checkWithinMap(float x, float y) {
		if ((x < xOrig) || (x >= (xOrig + width)) || (y < yOrig) || (y >= (yOrig + height)))
			return false;
		return true;
	}
};

struct HeightMapArray {
public:
	unsigned int rows, columns;
	std::vector< std::vector<HeightMapPtr> > array;

	HeightMapArray(unsigned int rows, unsigned int columns) :
		rows(rows), columns(columns)
	{
		array.resize(rows);
		for (unsigned int r = 0; r < rows; r++) {
			array[r].resize(columns);
		}
	}

	inline std::vector<HeightMapPtr>& operator[](int row) {
		return array[row];
	}
};

typedef Handle<HeightMultiMap> HeightMultiMapPtr;
class HeightMultiMap : public HandledObject {
public:
	struct SearchConstraints {
	public:
		double maxDist, fov;
		Pose2D sensor;

		SearchConstraints() : maxDist(-1), fov(0), sensor(NAN, NAN, NAN) {}
		SearchConstraints(double maxDist, double fov, const Pose2D& sensor) :
			maxDist(maxDist), fov(fov), sensor(sensor) {}
	};

	// x,y global position of the map
	float offsetX, offsetY;
//	unsigned int rows, columns, patchRows, patchColumns;
//	float initialOdds;
	FastSLAMParameters& parameters;
	unsigned int rows, columns;

	HeightMapArray patches;

	HeightMultiMap(FastSLAMParameters& parameters) :
		parameters(parameters),
		rows(parameters.mapRows), columns(parameters.mapColumns),
		patches(rows, columns)
	{

		zeroMap();
		offsetX = 0 - (columns * parameters.patchColumns * CELL_WIDTH / 2);
		offsetY = 0 - (rows * parameters.patchRows * CELL_HEIGHT / 2);
	}

	// performs shallow copy of all non-NULL HeightMaps
	HeightMultiMap(HeightMultiMapPtr cpy) :
		offsetX(cpy->offsetX), offsetY(cpy->offsetY),
		parameters(cpy->parameters),
		rows(parameters.mapRows), columns(parameters.mapColumns),
		patches(rows, columns)
	{
		unsigned int i, j;

		for (i = 0; i < rows; i++) {
			for (j = 0; j < columns; j++) {
				HeightMapPtr hm = cpy->patches[i][j];
				if (hm != NULL) {
					patches[i][j] = hm;
					hm->owner = NULL;
				}
			}
		}
	}

	~HeightMultiMap() {
		releaseSubMaps();
	}

	// set all submaps to NULL
	void zeroMap() {
		unsigned int i, j;
		for (i = 0; i < rows; i++) {
			for (j = 0; j < columns; j++) {
				HeightMapPtr hm = patches[i][j];
				if (hm!= NULL && hm->owner == this)
					hm->owner = NULL;
				patches[i][j] = NULL;
			}
		}
	}

	// release ownership of all submaps
	void releaseSubMaps() {
		unsigned int i, j;
		for (i = 0; i < rows; i++) {
			for (j = 0; j < columns; j++) {
				HeightMapPtr hm = patches[i][j];
				if (hm != NULL && hm->owner == this) {
					hm->owner = NULL;
				}
			}
		}
	}

	/**
	 * raytraces to each point from the proper origin and updates the cell probabilities
	 * pre: cloud must be in the world frame for the particle/map
	 */
	void update(MapCloudPtr cloud, double maxSensorRange, const SearchConstraints& search);

	/**
	 * raytraces to each point from the proper origin and returns the p(z | x) value
	 * pre: cloud must be in the world frame for the particle/map
	 */
	double getProb(MapCloudPtr cloud, double gain, double maxSensorRange);


	// All these similar functions are for efficiency in raytracing
	// normally it's probably best to just use getHeight rather than bother with the submaps

	// returns the heightmap containing x, y
	// if create is true the HeightMap is created if it's null, otherwise the null is returned
	// if unique is true the data will be deep copied unless it's owned by this particle.
	// You must set unique true if you're going to change any data in the local map.
	inline HeightMapPtr getMapByXY(float x, float y, bool create = false, bool unique = false) {
		unsigned int i, j;
		getIJ(x, y, i, j);
		return getMapByIJ(i, j, create, unique);
	}

	// returns the heightmap containing x, y
	// also returns the row (i) and column (j) of the heightmap
	// if create is true the HeightMap is created if it's null, otherwise the null is returned
	// if unique is true the data will be deep copied unless it's owned by this particle.
	// You must set unique true if you're going to change any data in the local map.
	inline HeightMapPtr getMapIJ(float x, float y, unsigned int &i, unsigned int &j, bool create = false, bool unique = false) {
		getIJ(x, y, i, j);
		return getMapByIJ(i, j, create, unique);
	}

	// returns patches[i][j]
	// if create is true the HeightMap is created if it's null, otherwise the null is returned
	// if unique is true the data will be deep copied unless it's owned by this particle.
	// You must set unique true if you're going to change any data in the local map.
	HeightMapPtr getMapByIJ(unsigned int i, unsigned int j, bool create = false, bool unique = false) {
		HeightMapPtr rval;
		HeightMapPtr prev;

		if (!valid(i, j)) {
			LOG("HMM: Invalid i,j (%u, %u)\n", i, j);
			return NULL;
		}

		rval = patches[i][j];

		if (rval == NULL) {
			if (!create) return rval;
			// just create an empty map
			rval = newMap(i, j);
			patches[i][j] = rval;
		} else if (unique && (rval->owner != this)) {
			// on a create call need to duplicate the map if we don't own it
			// only if we need a unique copy.  raytrace may ask for a new map but not a unique one
			rval = new HeightMap(rval);
			rval->owner = this;
			patches[i][j] = rval;
		}

		return rval;
	}

	// returns the row (i) and column (j) of the heightmap containing x, y
	inline void getIJ(float x, float y, unsigned int &i, unsigned int &j) {
		// note: assumes the submaps are using SUB_WIDTH and SUB_HEIGHT
		double ti, tj;

		tj = x - offsetX;
		ti = y - offsetY;

		tj = tj / CELL_WIDTH;
		ti = ti / CELL_HEIGHT;

		tj = tj / parameters.patchColumns;
		ti = ti / parameters.patchRows;

		j = (int) floor(tj);
		i = (int) floor(ti);

		if (j == __UINT32_MAX__)
			j = 0;
		if (i == __UINT32_MAX__)
			i = 0;
	}

	// checks if i,j is a valid heightmap
	inline bool valid(unsigned int i, unsigned int j) {
		// I don't use this much, it's more efficient not to use invalid values
		return (i < rows && j < columns);
//		if (j >= columns)) return false;
//		if (i >= rows)) return false;
//		return true;
	}

	// returns an entirely new HeightMap, belonging to this MultiMap
	// initialize the map at i,j to a new localmap
	// usually called from the retrieval function when create is true
	inline HeightMapPtr newMap(unsigned int i, unsigned int j) {
		HeightMapPtr rval = new HeightMap(parameters.patchRows,
				parameters.patchColumns, parameters.initialOccupiedOdds);
		rval->owner = this;
		rval->xOrig = offsetX + j * CELL_WIDTH * parameters.patchColumns;
		rval->yOrig = offsetY + i * CELL_HEIGHT * parameters.patchRows;
		return rval;
	}

	/**
	 * Ray traces from origin to destination.
	 * Returns distance traced before encountering an obstruction.
	 */
	double rayTrace(const Point& orig, const Point& dest);

	/**
	 * Ray traces from origin to destination updating cell probabilities.
	 */
	void updateTrace(const Point& orig, const Point& dest, double maxSensorRange, const SearchConstraints& = SearchConstraints());
};

} // namespace fastslam

} // namespace crosbot

#endif /* CROSBOT_FASTSLAM_HEIGHTMAP_HPP_ */
