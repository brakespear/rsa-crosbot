/*
 * serialization.hpp
 *
 *  Created on: 28/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_ASTSLAM_SERIALIZATION_HPP_
#define CROSBOT_ASTSLAM_SERIALIZATION_HPP_

#include <crosbot_fastslam/fastslam.hpp>
#include <crosbot/serialization.hpp>

namespace crosbot {
using namespace fastslam;

namespace serialization {

template<>
class Serializer<MapCloud> {
public:
	inline size_t write(const MapCloud& cloud, OutputStream& stream) throw (IOException) {
		Serializer<PointCloud> serCloud;
		Serializer<Pose> serPose;

		size_t rval = serCloud.write(cloud, stream);
		rval += serPose.write(cloud.odometry, stream);
		rval += serPose.write(cloud.sensor, stream);

		return rval;
	}

	inline size_t read(MapCloud& cloud, InputStream& stream) throw (IOException) {
		Serializer<PointCloud> serCloud;
		Serializer<Pose> serPose;

		size_t rval = serCloud.read(cloud, stream);
		rval += serPose.read(cloud.odometry, stream);
		rval += serPose.read(cloud.sensor, stream);

		return rval;
	}

	inline size_t serializedLength(const MapCloud& cloud) {
		Serializer<PointCloud> serCloud;
		return serCloud.serializedLength(cloud) + sizeof(Pose) + sizeof(Pose);
	}
};


CASROS_SIMPLE_SERIALIZER(HeightVal);

template <>
class Serializer<HeightMap> {
public:
	inline size_t write(const HeightMap& hm, OutputStream& stream) throw (IOException) {
		Serializer<uint32_t> serUInt32;
		Serializer<float> serFloat;
		Serializer<HeightVal> serHeightVals;

		size_t rval = serUInt32.write(hm.rows, stream);
		rval += serUInt32.write(hm.columns, stream);
		rval += serFloat.write(hm.xOrig, stream);
		rval += serFloat.write(hm.yOrig, stream);
		rval += serFloat.write(hm.width, stream);
		rval += serFloat.write(hm.height, stream);

		uint32_t numVals = hm.rows * hm.columns;
		for (uint32_t i = 0; i < numVals; i++) {
			rval += serHeightVals.write(hm.hlist[i], stream);
		}

		return rval;
	}

	inline size_t read(HeightMap& hm, InputStream& stream) throw (IOException) {
		Serializer<uint32_t> serUInt32;
		Serializer<float> serFloat;
		Serializer<HeightVal> serHeightVals;

		size_t rval = serUInt32.read(hm.rows, stream);
		rval += serUInt32.read(hm.columns, stream);
		rval += serFloat.read(hm.xOrig, stream);
		rval += serFloat.read(hm.yOrig, stream);
		rval += serFloat.read(hm.width, stream);
		rval += serFloat.read(hm.height, stream);

		uint32_t numVals = hm.rows * hm.columns;
		if (hm.hlist != NULL) {
			free(hm.hlist);
		}
		hm.hlist = (HeightVal *)malloc(numVals * sizeof(HeightVal));
		if (hm.hlist == NULL)
			throw IOException::OutOfMemory();

		for (uint32_t i = 0; i < numVals; i++) {
			rval += serHeightVals.read(hm.hlist[i], stream);
		}

		return rval;
	}

	inline size_t serializedLength(const HeightMap& hm) {
		return 2 * sizeof(uint32_t) + 4 * sizeof(float) +
				hm.rows * hm.columns * sizeof(HeightVal);
	}
};

template <>
class Serializer<HeightMultiMap> {
public:
	inline size_t write(const HeightMultiMap& hmm, OutputStream& stream) throw (IOException) {
		Serializer<float> serFloat;
		Serializer<uint32_t> serUInt32;
		Serializer<HeightMap> serHeightMap;

		size_t rval = serUInt32.write(hmm.rows, stream);
		rval += serUInt32.write(hmm.columns, stream);
		rval += serUInt32.write(hmm.parameters.patchRows, stream);
		rval += serUInt32.write(hmm.parameters.patchColumns, stream);

		rval += serFloat.write(hmm.offsetX, stream);
		rval += serFloat.write(hmm.offsetY, stream);
		rval += serFloat.write(hmm.parameters.initialOccupiedOdds, stream);

		uint32_t nonNulls = 0;
		for (uint32_t i = 0; i < hmm.rows; i++) {
			for (uint32_t j = 0; j < hmm.columns; j++) {
				if (hmm.patches.array[i][j] != NULL) {
					++nonNulls;
				}
			}
		}

		rval += serUInt32.write(nonNulls, stream);
		for (uint32_t i = 0; i < hmm.rows; i++) {
			for (uint32_t j = 0; j < hmm.columns; j++) {
				HeightMapPtr hm = hmm.patches.array[i][j];
				if (hm != NULL) {
					rval += serUInt32.write(i, stream);
					rval += serUInt32.write(j, stream);
					rval += serHeightMap.write(*hm, stream);
				}
			}
		}
		return rval;
	}

	inline size_t read(HeightMultiMap& hmm, InputStream& stream) throw (IOException) {
		hmm.zeroMap();

		Serializer<float> serFloat;
		Serializer<uint32_t> serUInt32;
		Serializer<HeightMap> serHeightMap;

		size_t rval = serUInt32.read(hmm.rows, stream);
		rval += serUInt32.read(hmm.columns, stream);
		rval += serUInt32.read(hmm.parameters.patchRows, stream);
		rval += serUInt32.read(hmm.parameters.patchColumns, stream);

		rval += serFloat.read(hmm.offsetX, stream);
		rval += serFloat.read(hmm.offsetY, stream);
		rval += serFloat.read(hmm.parameters.initialOccupiedOdds, stream);

		hmm.patches = HeightMapArray(hmm.rows, hmm.columns);

		uint32_t nonNulls = 0;
		rval += serUInt32.read(nonNulls, stream);

		for (uint32_t i = 0; i < nonNulls; i++) {
			uint32_t i, j;
			rval += serUInt32.read(i, stream);
			rval += serUInt32.read(j, stream);

			HeightMapPtr hm = hmm.patches[i][j] = new HeightMap(hmm.parameters.patchRows, hmm.parameters.patchColumns, hmm.parameters.initialOccupiedOdds);
			rval += serHeightMap.read(*hm, stream);
		}
		return rval;
	}

	inline size_t serializedLength(const HeightMultiMap& hmm) {
		Serializer<HeightMap> serHeightMap;

		size_t rval = 4 * sizeof(uint32_t) + 3 * sizeof(float);

		uint32_t nonNulls = 0;
		rval += sizeof(nonNulls);
		for (uint32_t i = 0; i < hmm.rows; i++) {
			for (uint32_t j = 0; j < hmm.columns; j++) {
				if (hmm.patches.array[i][j] != NULL) {
					rval += 2*sizeof(uint32_t) +
							serHeightMap.serializedLength(*(hmm.patches.array[i][j]));
				}
			}
		}
		return rval;
	}
};

CASROS_SIMPLE_SERIALIZER(FastSLAMParameters);

template<>
class Serializer<Particle::History> {
public:
	inline size_t write(const Particle::History& hist, OutputStream& stream) throw (IOException) {
		Serializer<Pose> serPose;
		Serializer<MapCloud> serPointCloud;
		size_t rval = serPose.write(hist.pose, stream);
		uint8_t byte = hist.restart;
		rval += stream.write(&byte, 1);
		if (hist.cloud == NULL) {
			byte = 0;
			rval += stream.write(&byte, 1);
		} else {
			byte = 1;
			rval += stream.write(&byte, 1);
			rval += serPointCloud.write(*(hist.cloud), stream);
		}
		return rval;
	}

	inline size_t read(Particle::History& hist, InputStream& stream) throw (IOException) {
		Serializer<Pose> serPose;
		Serializer<MapCloud> serPointCloud;
		size_t rval = serPose.read(hist.pose, stream);
		uint8_t byte;
		rval += stream.read(&byte, 1);
		hist.restart = byte;
		rval += stream.read(&byte, 1);
		if (byte == 0) {
			hist.cloud = NULL;
		} else {
			hist.cloud = new MapCloud();
			rval += serPointCloud.read(*(hist.cloud), stream);
		}
		return rval;
	}

	inline size_t serializedLength(const Particle::History& hist) {
		Serializer<Pose> serPose;
		Serializer<MapCloud> serPointCloud;

		size_t rval = serPose.serializedLength(hist.pose) + 2;
		if (hist.cloud != NULL) {
			rval += serPointCloud.serializedLength(*(hist.cloud));
		}
		return rval;
	}
};

template <>
class Serializer<Particle> {
	static const uint16_t version, subVersion;
public:
	inline size_t write(const Particle& p, OutputStream& stream) throw (IOException) {
		Serializer<uint8_t> serUInt8;
		Serializer<uint16_t> serUInt16;
		Serializer<float> serFloat;
		Serializer<Pose> serPose;
		uint16_t paramSize = sizeof(FastSLAMParameters),
				heightValSize = sizeof(HeightVal);
		uint8_t byte = 0;
		size_t rval = serUInt8.write(byte, stream);
		rval += serUInt16.write(version, stream);
		rval += serUInt16.write(subVersion, stream);
		rval += serUInt16.write(paramSize, stream);
		rval += serUInt16.write(heightValSize, stream);

		float f = CELL_HEIGHT;
		rval += serFloat.write(f, stream);
		f = CELL_WIDTH;
		rval += serFloat.write(f, stream);

		Serializer<FastSLAMParameters> serParams;
		Serializer<HeightMultiMap> serHMM;
		rval += serPose.write(p.pose, stream);
		rval += serPose.write(p.trackerPose, stream);

		rval += serParams.write(p.parameters, stream);
		rval += serHMM.write(p.map, stream);

		// write history
		Serializer<uint32_t> serUInt32;
		Serializer<Particle::History> serHistory;
		uint32_t count = p.history.size();
		rval += serUInt32.write(count, stream);
		for (uint32_t i = 0; i < count; i++) {
			rval += serHistory.write(p.history[i], stream);
		}

		// write tags
		Serializer<Tag> serTag;
		count = p.tags->tags.size();
		rval += serUInt32.write(count, stream);
		for (uint32_t i = 0; i < count; i++) {
			const Map::TagLocation& tl = p.tags->tags[i];
			rval += serPose.write(tl.robot, stream);
			rval += serPose.write(tl.mapPose, stream);

			if (tl.tag == NULL) {
				byte = 0;
				rval += serUInt8.write(byte, stream);
			} else {
				byte = 1;
				rval += serUInt8.write(byte, stream);
				rval += serTag.write(*(tl.tag), stream);
			}
		}
		return rval;
	}

	size_t readPreviousVersion(Particle&, InputStream&) throw (IOException);

	inline size_t read(Particle& p, InputStream& stream) throw (IOException) {
		bool currentVersion = true;

		Serializer<uint8_t> serUInt8;
		Serializer<uint16_t> serUInt16;
		Serializer<uint32_t> serUInt32;
		Serializer<float> serFloat;
		Serializer<Pose> serPose;

		uint8_t byte;
		size_t rval = serUInt8.read(byte, stream);
		if (byte == 0) {
			uint16_t v, sv, psz, hvsz;
			rval += serUInt16.read(v, stream);
			rval += serUInt16.read(sv, stream);
			rval += serUInt16.read(psz, stream);
			rval += serUInt16.read(hvsz, stream);

			if (v != version || sv != subVersion ||
					psz != sizeof(FastSLAMParameters) ||
					hvsz != sizeof(HeightVal)) {
				currentVersion = false;
			}
		} else {
			currentVersion = false;
		}

		if (!currentVersion) {
			stream.seek(0);
			return readPreviousVersion(p, stream);
		}

		float cH, cW;
		rval += serFloat.read(cH, stream);
		rval += serFloat.read(cW, stream);

		float diffH = fabs(cH - CELL_HEIGHT), diffW = fabs(cW - CELL_WIDTH);

		if (diffH > 0.01 || diffW > 0.01) {
			ERROR("Serialised map uses a different cell resolution. (%f, %f) not (%f, %f)",
					cW, cH, CELL_WIDTH, CELL_HEIGHT);
		}

		Serializer<FastSLAMParameters> serParams;
		Serializer<HeightMultiMap> serHMM;
		rval += serPose.read(p.pose, stream);
		rval += serPose.read(p.trackerPose, stream);
		p.pose.getTransform(p.poseTransform);

		rval += serParams.read(p.parameters, stream);
		if (p.map == NULL) {
			p.map = new HeightMultiMap(p.parameters);
		}
		rval += serHMM.read(*(p.map), stream);

		uint32_t count;
		// read history
		Serializer<Particle::History> serHistory;
		rval += serUInt32.read(count, stream);
		p.history.clear();
		Particle::History tmpHistory(Pose(), NULL);
		for (uint32_t i = 0; i < count; ++i) {
			rval += serHistory.read(tmpHistory, stream);
			p.history.push_back(tmpHistory);
		}

		// read tags
		Serializer<Tag> serTag;
		rval += serUInt32.read(count, stream);
		p.tags = new Map::TagList();
		Map::TagLocation tmpTagLoc(NULL, Pose(), Pose());
		for (uint32_t i = 0; i < count; ++i) {
			rval += serPose.read(tmpTagLoc.robot, stream);
			rval += serPose.read(tmpTagLoc.mapPose, stream);
			rval += serUInt8.read(byte, stream);

			if (byte == 0) {
				tmpTagLoc.tag = NULL;
			} else {
				tmpTagLoc.tag = serTag.read(stream);
			}

			p.tags->tags.push_back(tmpTagLoc);
		}

		return rval;
	}

	inline size_t serializedLength(const Particle& p) {
		Serializer<FastSLAMParameters> serParams;
		Serializer<HeightMultiMap> serHMM;

		size_t rval = sizeof(uint8_t) + 4*sizeof(uint16_t) +
				2*sizeof(float) + 2*sizeof(Pose);
		rval += serParams.serializedLength(p.parameters);
		rval += serHMM.serializedLength(*(p.map));

		// history size
		Serializer<Particle::History> serHistory;
		rval += sizeof(uint32_t);
		for (uint32_t i = 0; i < p.history.size(); ++i) {
			rval += serHistory.serializedLength(p.history[i]);
		}

		// tags size
		Serializer<Tag> serTag;
		rval += sizeof(uint32_t);
		for (uint32_t i = 0; i < p.tags->tags.size(); ++i) {
			rval += 2*sizeof(Pose) + 1;
			if (p.tags->tags[i].tag != NULL)
				rval += serTag.serializedLength(*(p.tags->tags[i].tag));
		}

		return rval;
	}
};

} // namespace serialization

} // namespace crosbot

#endif /* CROSBOT_ASTSLAM_SERIALIZATION_HPP_ */
