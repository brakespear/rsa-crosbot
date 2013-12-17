/*
 * particle.hpp
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_FASTSLAM_PARTICLE_HPP_
#define CROSBOT_FASTSLAM_PARTICLE_HPP_

#include <crosbot_fastslam/heightmap.hpp>
#include <crosbot_map/map.hpp>

namespace crosbot {

namespace fastslam {

class Particle;
typedef Handle<Particle> ParticlePtr;
class Particle : public HandledObject {
public:
	struct History {
	public:
		Pose pose;
		bool restart;
		MapCloudPtr cloud;

		History(Pose pose, MapCloudPtr cloud = NULL, bool restart = false) :
			pose(pose), restart(restart), cloud(cloud)
		{}
	};

protected:
	Pose pose, trackerPose;
	Map::TagListPtr tags;
	FastSLAMParameters& parameters;
	std::vector<History> history;

	HeightMultiMapPtr map;

	ReadWriteLock rwLock;

	tf::Transform poseTransform;
	double weight;
	MapCloudPtr motionCloud;

public:
	Particle(FastSLAMParameters& parameters);
	Particle(ParticlePtr parent, bool shallow = false);
	~Particle();

	void addTag(TagPtr tag, bool checkForDuplicate = false);

	void applyMotion(tf::Transform motion, MapCloudPtr cloud, bool calculateWeight = true, double gain = DEFAULT_GAINVALUE);
	void update(MapCloudPtr cloud);
	MapCloudPtr getLatestUpdate() { return motionCloud; }

	inline PathPtr getPath() {
		Lock lock(rwLock);
		PathPtr rval = new Path();

		for (size_t i = 0; i < history.size(); i++) {
			rval->path.push_back(history[i].pose);
		}

		if (rval->path.size() == 0 || rval->path[rval->path.size()-1] != pose)
			rval->path.push_back(pose);

		return rval;
	}

	void setPose(Pose pose) {
		Lock lock(rwLock, true);
		history.push_back(History(pose, NULL, true));
		this->pose = pose;
	}

	inline Pose getPose() {
		Lock lock(rwLock);
		return pose;
	}

	inline Pose getPose(Pose& tracker) {
		Lock lock(rwLock);
		tracker = trackerPose;
		return pose;
	}

	friend class FastSLAMMap;
	friend class FastSLAMRender;
	friend class crosbot::serialization::Serializer<Particle>;
};

} // namespace crosbot

} // namespace fastslam

#endif /* CROSBOT_FASTSLAM_PARTICLE_HPP_ */
