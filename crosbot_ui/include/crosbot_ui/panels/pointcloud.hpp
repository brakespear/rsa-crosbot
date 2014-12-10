/*
 * pointcloud.h
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_PANEL_POINTCLOUD_HPP_
#define CROSBOT_PANEL_POINTCLOUD_HPP_

#include <ros/ros.h>
#include <crosbot_ui/panels/panel.hpp>
#include <crosbot_ui/panels/wasdmouse.hpp>
#include <ros/ros.h>

#include <crosbot/PointCloudMsg.h>
//#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Pose.h>
#include <crosbot/thread.hpp>

namespace crosbot {

namespace gui {

#define PANEL_POINTCLOUD	"pointcloud"

class PointCloudView : public WASDMouseView {
	ReadWriteLock cloudLock, poseLock;
	PointCloudPtr cloud;

	std::vector<Pose> poses;
public:
	void paintWorldGL(GLTextQueue& text);

	void setCloud(PointCloudPtr cloud) {
		Lock lock(cloudLock, true);
		this->cloud = cloud;
		lock.unlock();

		performUpdate();
	}

	void setPose(const Pose& pose, int idx) {
		Lock lock(poseLock, true);
		if (idx >= (int)poses.size()) {
			poses.resize(idx+1);
		}
		poses[idx] = pose;
		lock.unlock();

		performUpdate();
	}
};

/**
 * \image html panel-image.png
 */
class PointCloudPanel : public Panel {
Q_OBJECT
public:
	class PoseSubscription {
	public:
		int i;
		PointCloudView* view;
		ros::Subscriber subscriber;

		PoseSubscription(int i, PointCloudView*);
		~PoseSubscription();
		void callbackPose(const geometry_msgs::Pose&);

		PoseSubscription& operator=(const PoseSubscription& other) {
			i = other.i;
			view = other.view;
			subscriber = other.subscriber;

			return *this;
		}
	};

	PointCloudPanel(ConfigElementPtr config);
	~PointCloudPanel();

	QWidget *getWidget();
	void start();
	void stop();

	void callbackPointCloud(crosbot::PointCloudMsgConstPtr);
protected:
	PointCloudView view;

	std::string cloudTopic;
	std::vector<std::string> poseTopics;

	ros::Subscriber cloudSubscriber;
	std::vector<PoseSubscription> poseSubscribers;
};

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_PANEL_POINTCLOUD_HPP_ */
