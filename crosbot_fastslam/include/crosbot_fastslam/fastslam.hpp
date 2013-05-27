/*
 * fastslam.hpp
 *
 *  Created on: 20/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_FASTSLAM_HPP_
#define CROSBOT_FASTSLAM_HPP_

#include <crosbot_map/map.hpp>
#include <crosbot_fastslam/particle.hpp>
#include <crosbot/thread.hpp>

namespace crosbot {

namespace fastslam {

class FastSLAMMap;
typedef Handle<FastSLAMMap> FastSLAMMapPtr;
class FastSLAMMap : public Map {
protected:
	/**
	 * The mean and motion particles
	 */
	ParticlePtr mean, motion;

	/**
	 * All of the particles
	 */
	std::vector<ParticlePtr> particles;
	ReadWriteLock particlesLock;

	// List of poses of particles, used in rendering
	std::vector<Pose> particlePoses;
	Pose meanPose;
public:
	FastSLAMMap();
	virtual ~FastSLAMMap();

	/**
	 * Methods for configuring and starting/stopping.
	 */
	void configure(ConfigElementPtr config);
	void start();
	void stop();

	/**
	 * Allow for map to be paused resumed.
	 */
	void setPaused(bool pause) {
		QueuedAction action;
		action.type = pause?QueuedAction::Pause:QueuedAction::Resume;

		Lock lock(incomingMutex);
		queuedActions.push(action);

		slamSemaphore.notify();
	}

	bool isPaused() { return slamPaused; }

	/**
	 * Allow map to be set in track only mode.
	 */
	void setUpdateMap(bool update) {
		slamUpdating = update;
	}

	/**
	 * Map can be cloned, saved and loaded.
	 */
	MapPtr clone();
	bool load(std::string filename) throw(IOException);
	bool save(std::string filename) throw(IOException);

	/**
	 * Gets the current pose and position tracker pose.
	 */
	Pose getCurrentPose(Pose* trackerPose = NULL);

	/**
	 * Gets the tags in the map.
	 */
	TagListPtr getTags();

	/**
	 * Gets the path the robot has followed if the map allows it.
	 */
	PathPtr getPath();

	/**
	 * Allows the robots pose to be reset.
	 */
	void resetPose(Pose3D pose = Pose()) {
		QueuedAction action;
		action.type = QueuedAction::ResetPose;
		action.pose = pose;

		Lock lock(incomingMutex);
		queuedActions.push(action);
		slamSemaphore.notify();
	}

	/**
	 * Data for maps. Cloud should be relative to the robot.
	 */
	void newPointCloud(PointCloudPtr cloud, Pose odometry, Pose sensor) {
		Lock lock(incomingMutex);
		latestPointCloud = new MapCloud(*cloud, odometry, sensor);
		slamSemaphore.notify();
	}

	void addTag(TagPtr tag) {
		Lock lock(incomingMutex);
		receivedTags.push(tag);
		slamSemaphore.notify();
	}

	/**
	 * Convert to a simple map type.
	 */
	nav_msgs::OccupancyGridPtr asOccupancyGrid();
	nav_msgs::OccupancyGridPtr asOccupancyGrid(ParticlePtr);

protected:
	struct MotionModel {
	public:
		double theta, drift, range, rangeXY, shift, shiftXY, slip;
		double smearX, smearY, smearTheta;

		MotionModel() :
			theta(KTH), drift(KDR), range(KR), rangeXY(KRXY),
			shift(KSH), shiftXY(KSHXY), slip(KSLP),
			smearX(SMEAR_X), smearY(SMEAR_Y), smearTheta(SMEAR_YAW)
		{}
	};

	FastSLAMParameters parameters;
	MotionModel motionModel;
	double motionSumXY, motionSumTheta;

	Mutex incomingMutex;
	MapCloudPtr latestPointCloud;
	std::queue<TagPtr> receivedTags;

	class QueuedAction {
	public:
		enum Type { Pause, Resume, ResetPose };
		Type type;
		Pose pose;
	};
	std::queue<QueuedAction> queuedActions;

	bool slamOperating, slamPaused, slamUpdating;
	Semaphore slamSemaphore;
	void slam();
	void insertTag(TagPtr tag);
	void moveAndUpdate(MapCloudPtr cloud);
	void moveParticles(Pose relativeMotion, MapCloudPtr cloud, bool calculateWeight = true);
	void updateParticles(MapCloudPtr cloud);
	void resample();
	void resampleAndUpdate(MapCloudPtr cloud);

	/**
	 * Thread in which to run slam algorithm
	 */
	class FastSLAMThread : public Thread {
	protected:
		FastSLAMMapPtr map;
	public:
		FastSLAMThread(FastSLAMMapPtr map) :
			Thread("FastSLAMMap::FastSLAMThread"), map(map)
		{ setGuarded(false); }
		void run() { map->slam(); }
	};
	FastSLAMThread slamThread;

	class ParticleMotionJob : public Job {
	public:
		ParticlePtr particle;
		Pose motion;
		Pose& pose;
		const MotionModel& model;
		MapCloudPtr cloud;
		bool calculateWeight;
		double gain;

		ParticleMotionJob(ParticlePtr particle, const Pose& motion, Pose& pose,
				const MotionModel& model, MapCloudPtr cloud, bool calculateWeight,
				double gain) :
			particle(particle), motion(motion), pose(pose), model(model), cloud(cloud),
			calculateWeight(calculateWeight), gain(gain)
		{}

		void run() {
			Pose randomMotion;
			double yaw, pitch, roll;
			motion.getYPR(yaw, pitch, roll);

			double& dx = motion.position.x;
			double& dy = motion.position.y;
			double dxy = sqrt(dx*dx + dy*dy);

			randomMotion.position.x = ProbabilityTable.NormRand48(dx,
					model.range * fabs(dx) + model.rangeXY * fabs(dxy) +
					model.slip * fabs(yaw));
			randomMotion.position.y = ProbabilityTable.NormRand48(dy,
					model.shift * fabs(dy) + model.shiftXY * fabs(dxy) +
					model.slip * fabs(yaw));

			yaw = ProbabilityTable.NormRand48(yaw,
					model.theta * fabs(yaw) + model.drift * fabs(dxy));

			// Apply smearing.
			if (calculateWeight) {
				randomMotion.position.x += ProbabilityTable.NormRand48(0, model.smearX);
				randomMotion.position.y += ProbabilityTable.NormRand48(0, model.smearY);
				yaw += ProbabilityTable.NormRand48(0, model.smearTheta);
			}

			randomMotion.setYPR(yaw, pitch, roll);
			particle->applyMotion(randomMotion.getTransform(), cloud, calculateWeight, gain);
			pose = particle->pose;
		}
	};

	class ParticleUpdateJob : public Job {
	public:
		ParticlePtr particle;
		MapCloudPtr cloud;

		ParticleUpdateJob(ParticlePtr particle, MapCloudPtr cloud) :
			particle(particle), cloud(cloud)
		{}

		void run() {
			particle->update(cloud);
		}
	};

	class ParticleCopyJob : public Job {
	public:
		ParticlePtr particle;
		std::vector<ParticlePtr>& particleList;
		std::vector<Pose>& poseList;
		Mutex& listMutex;

		ParticleCopyJob(ParticlePtr particle, std::vector<ParticlePtr>& particleList, std::vector<Pose>& poseList, Mutex& listMutex) :
			particle(particle), particleList(particleList), poseList(poseList), listMutex(listMutex)
		{}

		void run() {
			ParticlePtr newParticle = new Particle(particle);
			{ Lock lock(listMutex);
				particleList.push_back(newParticle);
				poseList.push_back(newParticle->pose);
			}
		}
	};

	/**
	 * A set of threads for multi-threading map updates
	 */
	JobDispatcher workers;
	friend class FastSLAMRender;
	friend class FastSLAMThread;

//	/**
//	 * ROS specific options;
//	 */
//	std::string cloudTopic, snapTopic;
//	ros::Subscriber cloudSubscriber, snapSubscriber;
//public:
//	void callbackPointCloud(casros_msgs::PointCloudConstPtr);
//	void callbackSnap(casros_msgs::SnapConstPtr);
};

} // namespace fastslam

} //namespace crosbot


#endif /* CROSBOT_FASTSLAM_HPP_ */
