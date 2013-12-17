/*
 * particle.cpp
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#include <crosbot_fastslam/particle.hpp>

namespace crosbot {

namespace fastslam {

Particle::Particle(FastSLAMParameters& parameters) :
		pose(0, 0, parameters.initialZ), trackerPose(), tags(new Map::TagList()),
		parameters(parameters), weight(1.0)
{
	map = new HeightMultiMap(parameters);
}

// Copy parent
Particle::Particle(ParticlePtr parent, bool shallow) : parameters(parent->parameters)
{
	Lock lock(parent->rwLock);
	pose = parent->pose;
	trackerPose = parent->trackerPose;
	tags = parent->tags;
	weight = parent->weight;

	if (shallow) {
		map = parent->map;
	} else {
		map = new HeightMultiMap(parent->map);
	}
	history = parent->history;
	poseTransform = parent->poseTransform;
	motionCloud = parent->motionCloud;
}

Particle::~Particle()
{}

void Particle::addTag(TagPtr tag, bool checkForDuplicate) {
	Pose robotPose;
	Pose tagPose;

	// Calculate tag & robot poses
	tf::Transform robotTrans =
			pose.toTF() * trackerPose.toTF().inverse() *
			tag->robot.toTF();
	robotPose = robotTrans;
	tagPose = robotTrans * tag->pose.toTF();

	if (checkForDuplicate) {
		// check if the tag is a duplicate snap
		SnapPtr snap = dynamic_cast<Snap*>(tag.get());
		if (snap != NULL && snap->status == Snap::UNCONFIRMED) {
			for (uint32_t i = 0; i < tags->tags.size(); i++) {
				Map::TagLocation& tl = tags->tags[i];
				SnapPtr snap2 = dynamic_cast<Snap*>(tl.tag.get());
				if (snap2 == NULL || snap->type != snap2->type ||
						snap2->status == Snap::REJECTED || snap2->status == Snap::DUPLICATE)
					continue;

				if (tl.mapPose.position.distanceTo(tagPose.position) < parameters.minDistanceBetweenSnaps) {
					if (snap2->status == Snap::UNCONFIRMED &&
							(robotPose.position.distanceTo(tagPose.position) <
									tl.robot.position.distanceTo(tl.mapPose.position))) {
						snap2->status = Snap::DUPLICATE;
					} else {
						snap->status = Snap::DUPLICATE;
					}
				}
			}
		}
	}

	Lock lock(rwLock, true);
	tags = new Map::TagList(tags);
	tags->tags.push_back(Map::TagLocation(tag, robotPose, tagPose));
}

void Particle::applyMotion(tf::Transform motion, MapCloudPtr cloud, bool calculateWeight, double gain) {
	{
		Lock lock(rwLock, true);
		// update particle pose
		poseTransform *= motion;
		pose = poseTransform;
		trackerPose = cloud->odometry;
	}
	if (!calculateWeight) {
		motionCloud = cloud;
		weight = 1.0;
		return;
	}

	history.push_back(History(pose, cloud));
	motionCloud = new MapCloud(pose, *cloud);

	// calculate particle weight
	weight = map->getProb(motionCloud, gain, parameters.maxSensorRange);
}

void Particle::update(MapCloudPtr cloud) {
	Lock lock(rwLock, true);
	if (cloud != motionCloud && cloud->timestamp != motionCloud->timestamp) {
		ERROR("crosbot_fastslam::Particle: invalid update cloud.\n");
		return;
	}

	HeightMultiMap::SearchConstraints search;
	// Define parameters for marking cells as searched
	search.maxDist = parameters.searchDistance;
	search.fov = parameters.searchFOV;
	if (search.maxDist > 0) {
		Pose sPose = pose.toTF() * parameters.searchPose.toTF();
		double yaw, pitch, roll;
		sPose.getYPR(yaw, pitch, roll);
		search.sensor = Pose2D(sPose.position.x, sPose.position.y, yaw);
	}

	map->update(motionCloud, parameters.maxSensorRange, search);
}

} // namespace fastslam

} // namespace crosbot
