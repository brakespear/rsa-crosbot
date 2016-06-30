/*
 * pointcloud.cpp
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */
#include <ros/ros.h>
#include <crosbot_ui/panels/pointcloud.hpp>
#include <crosbot/utils.hpp>

#include <rosbag/structures.h>

namespace crosbot {

namespace gui {

PointCloudPanel::PoseSubscription::PoseSubscription(int i, PointCloudView* view) :
		i(i), view(view) {}

PointCloudPanel::PoseSubscription::~PoseSubscription() {
	subscriber.shutdown();
}

void PointCloudPanel::PoseSubscription::callbackPose(const geometry_msgs::Pose& pose) {
	view->setPose(pose, i);
}

PointCloudPanel::PointCloudPanel(ConfigElementPtr config) : Panel(config) {
	cloudTopic = config->getParam(PARAM_TOPIC);
	cloudTopic = config->getParam(PARAM_NAME, cloudTopic);
	cloudTopic = config->getParam("cloud", cloudTopic);

	if (config->hasParam(PARAM_POSE)) {
		poseTopics.push_back(config->getParam(PARAM_POSE));
	}

	for (uint32_t i = 0; i < config->getChildCount(); ++i) {
		ConfigElementPtr child = config->getChild(i);
		if (strcasecmp(child->getChildName().c_str(), PARAM_POSE) == 0) {
			std::string pose = child->getParam(PARAM_NAME);
			pose = child->getParam(PARAM_POSE, pose);
			pose = child->getParam(PARAM_TOPIC, pose);

			if (pose != "")
				poseTopics.push_back(pose);
		}
	}
}

PointCloudPanel::~PointCloudPanel() {
	stop();
}

QWidget *PointCloudPanel::getWidget() {
	return &view;
}

void PointCloudPanel::start() {
	ros::NodeHandle nh("~");
	if (cloudTopic != "") {
		cloudSubscriber = nh.subscribe(cloudTopic, 1,
				&PointCloudPanel::callbackPointCloud, this);
	}

	for (uint32_t i = 0; i < poseTopics.size(); ++i) {
		poseSubscribers.push_back(PoseSubscription(i, &view));
		poseSubscribers[i].subscriber = nh.subscribe(poseTopics[i], 1,
				&PointCloudPanel::PoseSubscription::callbackPose,
				&poseSubscribers[i]);
	}
}

void PointCloudPanel::stop() {
	if (cloudTopic != "") {
		cloudSubscriber.shutdown();
	}
	poseSubscribers.clear();
}

void PointCloudPanel::callbackPointCloud(crosbot_msgs::PointCloudMsgConstPtr ros) {
	PointCloudPtr pc = new PointCloud(ros);
	view.setCloud(pc);
}

void PointCloudView::paintWorldGL(GLTextQueue& text) {
	PointCloudPtr cloud;

	{{
		Lock lock(cloudLock);
		cloud = this->cloud;
	}}

	int foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE point cloud render. (%s)\n", glutGetGLError(foo).c_str());
	}
	glPushMatrix();
		glPointSize(1.0);

		glBegin(GL_POINTS);
			if (cloud != NULL) {
				if (cloud->colours.size() >= cloud->cloud.size()) {
					size_t n = cloud->cloud.size();
					for(unsigned int i = 0; i < n; i++) {
						Point& p = cloud->cloud[i];
						Colour& c = cloud->colours[i];
						glColor4f(c.r/255.0, c.g/255.0,
								c.b/255.0, c.a/510.0);
						glVertex3f(p.x, p.y, p.z);
					}
				} else {
					glColor4f(0, 0, 0, 0.5);

					size_t n = cloud->cloud.size();
					for(unsigned int i = 0; i < n; i++) {
						Point& p = cloud->cloud[i];
						glVertex3f(p.x, p.y, p.z);
					}
				}
			}
		glEnd();
		{{ Lock lock(poseLock);
			for (size_t i = 0; i < poses.size(); i++) {
				drawPose(poses[i], 0.1);
			}
		}}
	glPopMatrix();
	foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER point cloud render. (%s)\n", glutGetGLError(foo).c_str());
	}

}

} // namespace gui

} // namespace casros
