/*
 * fastslam.cpp
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#include <ros/ros.h>
//#include <casros/fastslam/module.h>

#include <crosbot_fastslam/fastslam.hpp>
#include <crosbot/utils.hpp>
#include <crosbot_fastslam/serialization.hpp>

//#include <ros/xmlrpc_manager.h>


namespace crosbot {

namespace fastslam {

FastSLAMMap::FastSLAMMap() :
	motionSumXY(0), motionSumTheta(0),
	slamOperating(true), slamPaused(false), slamUpdating(true),
	slamThread(this), workers("FastSLAMMap::WorkerThread")
{
}

FastSLAMMap::~FastSLAMMap() {
	stop();
}

void FastSLAMMap::configure(ConfigElementPtr config) {
	parameters.gain = config->getParamAsDouble("gain", parameters.gain);
	parameters.numberOfParticles = config->getParamAsInt("particles", parameters.numberOfParticles);
	workers.setThreads(config->getParamAsInt("threads", 1));
	if (config->hasParam("initialOdds")) {
		LOG("BEFORE: %lf\n", parameters.initialOccupiedOdds);
		parameters.initialOccupiedOdds =
				ProbabilityTable.lodds(config->getParamAsDouble("initialOdds", DEFAULT_UNOBSERVEDLODDS));
		LOG("AFTER: %lf\n", parameters.initialOccupiedOdds);
	}
	parameters.initialZ = config->getParamAsDouble("z", parameters.initialZ);
	parameters.updateThresholdXY = config->getParamAsDouble("update", parameters.updateThresholdXY);
	parameters.updateThresholdTheta = DEG2RAD(config->getParamAsDouble("updateTheta", DEFAULT_ANGULARTHRESHOLD));
	parameters.minDistanceBetweenSnaps = config->getParamAsDouble("snapThreshold", parameters.minDistanceBetweenSnaps);

	parameters.minHeight = config->getParamAsDouble("min", parameters.minHeight);
	parameters.maxHeight = config->getParamAsDouble("max", parameters.maxHeight);

	if (config->hasParam("initialOdds"))
		parameters.initialOccupiedOdds = ProbabilityTable.lodds(config->getParamAsDouble("initialOdds", DEFAULT_UNOBSERVEDLODDS));

	ConfigElementPtr motionConfig = config->getChild("motion");
	if (motionConfig == NULL)
		motionConfig = config;
	motionModel.range = motionConfig->getParamAsDouble("range", motionModel.range);
	motionModel.rangeXY = motionConfig->getParamAsDouble("rangeXY", motionModel.rangeXY);
	motionModel.drift = motionConfig->getParamAsDouble("drift", motionModel.drift);
	motionModel.shift = motionConfig->getParamAsDouble("shift", motionModel.shift);
	motionModel.shiftXY = motionConfig->getParamAsDouble("shiftXY", motionModel.shiftXY);
	motionModel.slip = motionConfig->getParamAsDouble("slip", motionModel.slip);

	ConfigElementPtr smearConfig = config->getChild("smear");
	if (smearConfig == NULL)
		smearConfig = motionConfig;
	motionModel.smearX = smearConfig->getParamAsDouble("x", motionModel.smearX);
	motionModel.smearY = smearConfig->getParamAsDouble("y", motionModel.smearY);
	motionModel.smearTheta = smearConfig->getParamAsDouble("theta", motionModel.smearTheta);

	ConfigElementPtr searchConfig = config->getChild("search");
	if (searchConfig == NULL)
		searchConfig = config;
	parameters.searchDistance = searchConfig->getParamAsDouble("search", parameters.searchDistance);
	parameters.searchDistance = searchConfig->getParamAsDouble("distance", parameters.searchDistance);
	parameters.searchFOV = DEG2RAD(searchConfig->getParamAsDouble("fov", RAD2DEG(parameters.searchFOV)));
	if (searchConfig->hasParam("pose")) {
		parameters.searchPose = searchConfig->getParamAsPose("pose", parameters.searchPose);
	}

	parameters.addMotionCopy = config->getParamAsBool("add_motion_copy", parameters.addMotionCopy);
}

void FastSLAMMap::start() {
	slamOperating = true;
	if (!slamThread.isAlive())
		slamThread.start();
}

void FastSLAMMap::stop() {
	slamOperating = false;
	slamSemaphore.notify();

	for (unsigned int i = 0; i < DEFAULT_WAIT_FOR_THREAD_CLOSE && slamThread.isAlive(); i+=10) {
		usleep(10000);
	}

	if (slamThread.isAlive()) {
		ERROR("FastSLAMMap: SlamThread chose not to complete in %f seconds.\n", DEFAULT_WAIT_FOR_THREAD_CLOSE/1000.0);
	}
}

MapPtr FastSLAMMap::clone() {
	FastSLAMMapPtr rval = new FastSLAMMap();
	rval->parameters = parameters;
	rval->motionModel = motionModel;
	rval->workers.setThreads(workers.threads());
	rval->start();

	return rval;
}

bool FastSLAMMap::load(std::string filename) throw (IOException) {
	setPaused(true);
	for (uint32_t i = 0; !isPaused() && i < 10000; i += 10) {
		usleep(10000);
	}

	if (!isPaused()) {
		ERROR("FastSLAM map is not pausing. Cannot load from file.\n");
		return false;
	}

	ParticlePtr newMean = new  Particle(parameters);
	serialization::Serializer<Particle> serializer;
	serialization::FileInputStream fis(filename);
	serializer.read(*(newMean), fis);

	{ Lock lock(particlesLock, true);
		mean = newMean;
		meanPose = mean->pose;
		motion = new Particle(newMean);

		particles.clear();
		particles.push_back(mean);
		particlePoses.clear();
		particlePoses.push_back(meanPose);
	}

	mapChanged();

	return true;
}

bool FastSLAMMap::save(std::string filename) throw (IOException) {
	if (mean == NULL)
		return false;

	ParticlePtr particle = new Particle(mean);

	serialization::Serializer<Particle> serializer;
	serialization::FileOutputStream fos(filename);
	serializer.write(*(particle), fos);

	return true;
}

Pose FastSLAMMap::getCurrentPose(Pose* trackerPose) {
	ParticlePtr particle = mean;
	if (particle == NULL) {
		if (trackerPose != NULL)
			*trackerPose = Pose(NAN, NAN, NAN);
		return Pose();
	}
	{ Lock lock(particle->rwLock);
		if (trackerPose != NULL)
			*trackerPose = particle->trackerPose;
		return particle->pose;
	}
}

Map::TagListPtr FastSLAMMap::getTags() {
	ParticlePtr particle = mean;
	if (particle == NULL)
		return NULL;
	return particle->tags;
}

PathPtr FastSLAMMap::getPath() {
	ParticlePtr particle = mean;
	if (particle == NULL)
		return NULL;
	return particle->getPath();
}

nav_msgs::OccupancyGridPtr FastSLAMMap::asOccupancyGrid() {
	// Get copy of mean
	ParticlePtr p;
	if (mean == NULL) {
		p = new Particle(parameters);
	} else {
		p = new Particle(mean);
	}

	return asOccupancyGrid(p);
}

nav_msgs::OccupancyGridPtr FastSLAMMap::asOccupancyGrid(ParticlePtr particle) {
	nav_msgs::OccupancyGridPtr rval(new nav_msgs::OccupancyGrid());
	rval->info.origin = Pose().toROS();

	// Get map dimensions and origin

	Lock lock(particle->rwLock);
	HeightMultiMapPtr hmm;
	if (particle != NULL)
		hmm = particle->map;

	if (particle == NULL || hmm == NULL) {
		rval->info.origin = Pose().toROS();
		rval->info.height = rval->info.width = 0;
		rval->info.resolution = 0;

		rval->data.clear();
		return rval;
	}

	unsigned int minHMX = hmm->columns, maxHMX = 0, minHMY = hmm->rows, maxHMY = 0;

	rval->info.origin.position.x = INFINITY; rval->info.origin.position.y = INFINITY;

	// find which HeightMaps have data and need to be drawn
	for (unsigned int i = 0; i < hmm->rows; i++) {
		for (unsigned int j = 0; j < hmm->columns; j++) {
			HeightMapPtr hm = hmm->patches[i][j];
			if (hm != NULL) {
				if (i < minHMY)
					minHMY = i;
				if (i > maxHMY)
					maxHMY = i;
				if (j < minHMX)
					minHMX = j;
				if (j > maxHMX)
					maxHMX = j;

				if (hm->xOrig < rval->info.origin.position.x) {
					rval->info.origin.position.x = hm->xOrig;
				}
				if (hm->yOrig < rval->info.origin.position.y) {
					rval->info.origin.position.y = hm->yOrig;
				}
			}
		}
	}

	if (minHMX > maxHMX || minHMY > maxHMY) {
		ERROR("FastSLAMRender: No data in map to export to GeoTIFF.\n");
		rval->info.origin = Pose().toROS();
		return rval;
	}

	// create image
	unsigned int imageWidth = (maxHMX + 1 - minHMX) * hmm->parameters.patchColumns,
			imageHeight = (maxHMY + 1 - minHMY) * hmm->parameters.patchRows;

	rval->info.height = imageHeight; rval->info.width = imageWidth;
	rval->info.resolution = CELL_WIDTH;
	rval->data.resize(imageHeight * imageWidth);

	for (unsigned int i = minHMY; i <= maxHMY; i++) {
		for (unsigned int j = minHMX; j <= maxHMX; j++) {
			HeightMapPtr hm = hmm->patches[i][j];
			int offX = (j - minHMX) * hmm->parameters.patchColumns,
					offY = (i - minHMY) * hmm->parameters.patchRows;
			if (hm == NULL) {
				for (unsigned int il = 0; il < hmm->parameters.patchRows; il++) {
					for (unsigned int jl = 0; jl < hmm->parameters.patchColumns; jl++) {
						int pixel = (offY + il) * imageWidth + offX + jl;
						rval->data[pixel] = -1;
					}
				}
			} else {
				for (unsigned int il = 0; il < hmm->parameters.patchRows; il++) {
					for (unsigned int jl = 0; jl < hmm->parameters.patchColumns; jl++) {
						HeightVal hv = *(hm->getIndex(il * hmm->parameters.patchColumns + jl));

						int pixel = (offY + il) * imageWidth + offX + jl;

						if (hv.observations <= 0) {
							rval->data[pixel] = -1;
						} else {
							double conf = ProbabilityTable.invodds(hv.prob);
							rval->data[pixel] = conf * 100;
						}
					}
				}
			}
		}
	}

	return rval;
}

void FastSLAMMap::slam() {
	while (slamOperating) {
		slamSemaphore.wait();

		MapCloudPtr update;
		{{
			Lock lock(incomingMutex);
			if (!slamPaused && receivedTags.size() > 0) {
				TagPtr tag = receivedTags.front();
				receivedTags.pop();
				lock.unlock();
				insertTag(tag);
				tagAdded(tag);
				slamSemaphore.notify();
				continue;
			} else if (queuedActions.size() > 0) {
				QueuedAction action = queuedActions.front();
				queuedActions.pop();
				lock.unlock();

				if (action.type == QueuedAction::Pause) {
					slamPaused = true;
				} else if (action.type == QueuedAction::Resume) {
					slamPaused = false;
				} else if (action.type == QueuedAction::ResetPose) {{
					// Reset pose in FastSLAM map.
					Lock lock2(particlesLock, true);

					motion->setPose(action.pose);
					for (uint32_t i = 0; i < particles.size(); i++)
						particles[i]->setPose(action.pose);
					mapChanged();
				}}

				slamSemaphore.notify();
				continue;
			}

			update = latestPointCloud;
			latestPointCloud = NULL;
		}}

		if (slamPaused || update == NULL) {
			continue;
		}

		moveAndUpdate(update);

		slamSemaphore.notify();
	}

	slamPaused = true;
}

void FastSLAMMap::insertTag(TagPtr tag) {
	if (mean == NULL) {
		// No mean particle so create one

		mean = new Particle(parameters);
		mean->addTag(tag, true);
		return;
	}

	mean->addTag(tag, true);
	motion->addTag(tag);
	Lock lock(particlesLock, true);
	for (size_t i = 0; i < particles.size(); i++) {
		if (particles[i] != mean)
			particles[i]->addTag(tag);
	}
}

void FastSLAMMap::moveAndUpdate(MapCloudPtr cloud) {
	if (motion == NULL) {
		// No motion particle means FastSLAM has not yet been started
		ParticlePtr newMean = mean, newMotion;
		if (newMean == NULL)
			newMean = new Particle(parameters);
		newMean->trackerPose = cloud->odometry;
		newMean->history.push_back(Particle::History(newMean->pose, cloud, true));
		newMean->poseTransform = newMean->pose.toTF();

		if (slamUpdating) {
//			double Y,P,R;
//			cloud->odometry.orientation.getYPR(Y,P,R);
			newMean->motionCloud = cloud;
			newMean->update(newMean->motionCloud);
		}

		newMotion = new Particle(newMean);


		{ Lock lock(particlesLock, true);
			mean = newMean;
			motion = newMotion;
			particles.clear();
			particles.push_back(mean);
			particlePoses.push_back(mean->pose);
			meanPose = mean->pose;
		}

		resample();
		mapChanged();
		return;
	}

//	LOG("FastSLAM - Moving\n");
	// Get motion
	double yaw, pitch, roll;
	motion->trackerPose.getYPR(yaw, pitch, roll);

	Pose motionPose;
	tf::Transform motionTransform,
		trackerTransform = cloud->odometry.toTF();

	motionTransform = motion->trackerPose.toTF().inverse() * trackerTransform;
	motionPose = motionTransform;

	motionPose.getYPR(yaw, pitch, roll);
	motionSumTheta += fabs(yaw);
	motionSumXY += sqrt(motionPose.position.x*motionPose.position.x +
			motionPose.position.y*motionPose.position.y);

	bool resampleNeeded = true;
	if (motionSumXY < parameters.updateThresholdXY && motionSumTheta < parameters.updateThresholdTheta)
		resampleNeeded = false;

	moveParticles(motionPose, cloud, resampleNeeded);

	if (!resampleNeeded) {
		motionTracked();
		return;
	}

	motionSumXY = motionSumTheta = 0.0;

	// Combine update and resample into one more efficient block
	if (slamUpdating) {
//		LOG("FastSLAM - Updating\n");
		resampleAndUpdate(cloud);
//		updateParticles(cloud);
//		resample();
	} else {
//		LOG("FastSLAM - Resampling\n");
		resample();
	}

	mapChanged();
}


void FastSLAMMap::moveParticles(Pose relativeMotion, MapCloudPtr cloud, bool calculateWeight) {
	tf::Transform motionTransform = relativeMotion.toTF();
	std::vector<ParticleMotionJob *> jobs;
	std::vector<Pose> newPoses(particles.size());
	for (size_t i = 0; i < particles.size(); i++) {
		ParticleMotionJob* job = new ParticleMotionJob(particles[i],
				relativeMotion, newPoses[i], motionModel, cloud, calculateWeight,
				parameters.gain);
		workers.addJob(job);
		jobs.push_back(job);
	}

	// Wait for all jobs to finish.
	while (workers.jobCount() > 0) {
		usleep(5000);
	}

	{ Lock lock(particlesLock);
		motion->applyMotion(motionTransform, cloud, calculateWeight);
		particlePoses = newPoses;
		meanPose = mean->pose;
	}

	for (size_t i = 0; i < jobs.size(); i++) {
		delete jobs[i];
	} jobs.clear();
}

void FastSLAMMap::updateParticles(MapCloudPtr cloud) {
	std::vector<Job *> jobs;
	Job* job = new ParticleUpdateJob(motion, cloud);
	jobs.push_back(job);
	workers.addJob(job);
	for (size_t i = 0; i < particles.size(); i++) {
		job = new ParticleUpdateJob(particles[i], cloud);
		workers.addJob(job);
		jobs.push_back(job);
	}
	// Wait for all jobs to finish.
	while (workers.jobCount() > 0) {
		usleep(5000);
	}
	for (size_t i = 0; i < jobs.size(); i++) {
		delete jobs[i];
	} jobs.clear();
}

void FastSLAMMap::resample() {
	if (particles.size() == 0)
		return;
	double weightMax, weightSum = 0;
	ParticlePtr newMean = particles[0];
	weightMax = newMean->weight;
	for (size_t i = 0; i < particles.size(); i++) {
		ParticlePtr p = particles[i];
		if (p->weight > weightMax) {
			newMean = p;
			weightMax = p->weight;
		}
		weightSum += p->weight;
	}

	std::vector<ParticlePtr> newParticles;
	std::vector<Pose> newPoses;
	std::vector<int> marks;
	marks.resize(particles.size());
	memset(&marks[0], 0, marks.size()*sizeof(int));

	newParticles.resize(parameters.numberOfParticles);
	newPoses.resize(newParticles.size());
	size_t i = 0;
	if (parameters.addMotionCopy) {
		newParticles[i] = new Particle(motion);
		++i;
	}

	for (; i < newParticles.size(); i++) {
		double rand = ProbabilityTable.randxy(0, weightSum);
		ParticlePtr p = particles[0];
		double wCurrent = p->weight;
		size_t j = 0;
		for (++j; j < particles.size() && wCurrent <= rand; j++) {
			p = particles[j];
			wCurrent += p->weight;
		}

		j--;
		if (marks[j]) {
			newParticles[i] = new Particle(p);
			newPoses[i] = p->pose;
		} else {
			newParticles[i] = p;
			newPoses[i] = p->pose;
			marks[j] = true;
		}
	}

	{ Lock lock(particlesLock, true);
		mean = newMean;
		particles.clear();
		particles = newParticles;
		newParticles.clear();
		particlePoses = newPoses;
		meanPose = mean->pose;
	}
}

void FastSLAMMap::resampleAndUpdate(MapCloudPtr cloud) {
	if (particles.size() == 0)
		return;

	double weightMax, weightSum = 0;
	ParticlePtr newMean = particles[0];
	weightMax = newMean->weight;
	for (size_t i = 0; i < particles.size(); i++) {
		ParticlePtr p = particles[i];
		if (p->weight > weightMax) {
			newMean = p;
			weightMax = p->weight;
		}
		weightSum += p->weight;
	}

	std::vector<int> marks;
	marks.resize(particles.size());
	memset(&marks[0], 0, marks.size()*sizeof(int));
	std::vector<ParticlePtr> newParticles;
	std::vector<Pose> newPoses;

	Mutex newParticleMutex;

	std::vector<ParticleUpdateJob *> updates;
	std::vector<ParticleCopyJob *> copies;

	// Update motion particle
	ParticleUpdateJob *motionJob = new ParticleUpdateJob(motion, cloud);
	updates.push_back(motionJob);
	workers.addJob(motionJob);

	size_t i = 0;
	if (parameters.addMotionCopy) {
		ParticleCopyJob *copy = new ParticleCopyJob(motion, newParticles, newPoses, newParticleMutex);
		copies.push_back(copy);
		++i;
	}

	for (; i < parameters.numberOfParticles; ++i) {
		double rand = ProbabilityTable.randxy(0, weightSum);
		ParticlePtr p = particles[0];
		double wCurrent = p->weight;
		size_t j = 0;
		for (++j; j < particles.size() && wCurrent <= rand; ++j) {
			p = particles[j];
			wCurrent += p->weight;
		}

		j--;
		if (!marks[j]) {
			ParticlePtr p = particles[j];
			ParticleUpdateJob *update = new ParticleUpdateJob(p, cloud);
			updates.push_back(update);
			workers.addJob(update);
			newParticles.push_back(p);
			newPoses.push_back(p->pose);
			marks[j] = true;
		} else {
			ParticleCopyJob *copy = new ParticleCopyJob(particles[j], newParticles, newPoses, newParticleMutex);
			copies.push_back(copy);
		}
	}

//	LOG("FastSLAM %Zu particle updates, %Zu particle copies.\n",
//			updates.size(), copies.size());

	// Wait for updates to finish
	while (workers.jobCount() > 0) {
		usleep(5000);
	}

	for (size_t c = 0; c < copies.size(); c++) {
		workers.addJob(copies[c]);
	}

	for (size_t u = 0; u < updates.size(); u++) {
		delete updates[u];
	} updates.clear();

	// Wait for copies to finish
	while (workers.jobCount() > 0) {
		usleep(5000);
	}

	for (size_t c = 0; c < copies.size(); c++) {
		delete copies[c];
	} copies.clear();

	// Swap in the new mean and new particles
	{ Lock lock(particlesLock, true);
		mean = newMean;
		particles = newParticles;
		particlePoses = newPoses;
		meanPose = mean->pose;
	}
}

} // namespace fastslam

} // namespace crosbot
