#include <crosbot_ui/renders/map/map.hpp>
#include <crosbot_ui/renders/map/fastslam.hpp>

#include <crosbot/utils.hpp>

namespace crosbot {

namespace gui {

std::vector<MapRenderFactoryFunc> mapRenderFactories;

void MapRender::addRenderFactory(MapRenderFactoryFunc factory) {
	mapRenderFactories.push_back(factory);
}

MapRender *MapRender::getRender(MapPtr map) {
	MapRender *rval = NULL;

	for (size_t i = 0; rval == NULL && i < mapRenderFactories.size(); i++) {
		rval = mapRenderFactories[i](map);
	}

	if (rval == NULL) {
		fastslam::FastSLAMMapPtr fastSLAMMap = dynamic_cast<fastslam::FastSLAMMap *>(map.get());
		if (fastSLAMMap != NULL) {
			rval = new fastslam::FastSLAMRender(fastSLAMMap);
		}
	}

	return rval;
}

void MapRender::renderRobot(const Pose& robotPose, float size) {
    int foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE robot render. (%s)\n", glutGetGLError(foo).c_str());
    }

	float xDim = 0.15*size;
	float yDim = 0.1*size;
	float zDim = 0.1*size;
	glPushMatrix();

		glTranslatef(robotPose.position.x, robotPose.position.y, robotPose.position.z);
		double yaw, pitch, roll;
		robotPose.getYPR(yaw, pitch, roll);

		// apply rotations
		glRotatef(RAD2DEG(yaw), 0, 0, 1);
		glRotatef(RAD2DEG(pitch), 0, 1, 0);
		glRotatef(RAD2DEG(roll), 1, 0, 0);

		glColor4f(robotColour.r,robotColour.g,robotColour.b,robotColour.a);
		glBegin(GL_TRIANGLES);
			glVertex3f(xDim, 0, 0);
			glVertex3f(-xDim, -yDim, 0);
			glVertex3f(-xDim, yDim, 0);

			glVertex3f(-xDim, yDim, 0);
			glVertex3f(-xDim, -yDim, 0);
			glVertex3f(-xDim, 0, zDim);

			glVertex3f(-xDim, 0, zDim);
			glVertex3f(xDim, 0, 0);
			glVertex3f(-xDim, yDim, 0);

			glVertex3f(-xDim, -yDim, 0);
			glVertex3f(xDim, 0, 0);
			glVertex3f(-xDim, 0, zDim);
		glEnd();

		if (isSelected) {
			glColor4f(selectedEdgeColour.r,selectedEdgeColour.g,selectedEdgeColour.b,selectedEdgeColour.a);
		} else {
			glColor4f(robotEdgeColour.r,robotEdgeColour.g,robotEdgeColour.b,robotEdgeColour.a);
		}
		glPointSize(2.0);
		glBegin(GL_LINES);
			glVertex3f(-xDim, yDim, 0);
			glVertex3f(-xDim, -yDim, 0);

			glVertex3f(-xDim, -yDim, 0);
			glVertex3f(xDim, 0, 0);

			glVertex3f(xDim, 0, 0);
			glVertex3f(-xDim, yDim, 0);

			glVertex3f(-xDim, 0, zDim);
			glVertex3f(-xDim, yDim, 0);

			glVertex3f(-xDim, 0, zDim);
			glVertex3f(-xDim, -yDim, 0);

			glVertex3f(-xDim, 0, zDim);
			glVertex3f(xDim, 0, 0);
		glEnd();
		glPointSize(1.0);
	glPopMatrix();

    foo = glGetError();
    if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER robot render. (%s)\n", glutGetGLError(foo).c_str());
    }
}

void MapRender::renderTag(TagPtr tag, const Pose& tagPose, GLTextQueue& textQueue, float size) {
	SnapPtr snap = dynamic_cast<Snap *>(tag.get());
	if (snap == NULL)
		return;

	if (snap->status == Snap::REJECTED || snap->status == Snap::DUPLICATE) {
		return;
	}
	if (snap->type == Snap::SCAN)
		return;
	int foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE snap render. (%s)\n", glutGetGLError(foo).c_str());
	}
	glPushMatrix();
		glPointSize(20 * size);
		Colour4f colour = landmarkColour;

		if (snap->type == Snap::VICTIM) {
			colour = unconfirmedVictimColour;
			if (snap->status == Snap::CONFIRMED)
				colour = confirmedVictimColour;
		}

		glColor4f(colour.r,colour.g,colour.b,colour.a);

		glBegin(GL_POINTS);
		glVertex3f(tagPose.position.x, tagPose.position.y, tagPose.position.z);
		glEnd();

		textQueue.push_back(GLLabel(snap->description, tagPose.position.x,
				tagPose.position.y, tagPose.position.z + 1.0, colour));
		glFlush();
		glPointSize(1.0);
	glPopMatrix();
	foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER snap render. (%s)\n", glutGetGLError(foo).c_str());
	}
}

void MapRender::renderCloud(PointCloudPtr cloud, const Pose& robotPose, GLTextQueue& textQueue) {
	int foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number BEFORE point cloud render. (%s)\n", glutGetGLError(foo).c_str());
	}
	glPushMatrix();
		glPointSize(1.0);

		size_t N = cloud->cloud.size();

		// All clouds should be in robot relative frame.
		// translate to robot pose
		glTranslatef(robotPose.position.x, robotPose.position.y, robotPose.position.z);
		double yaw, pitch, roll;
		robotPose.getYPR(yaw, pitch, roll);

		// apply rotations
		glRotatef(RAD2DEG(yaw), 0, 0, 1);
		glRotatef(RAD2DEG(pitch), 0, 1, 0);
		glRotatef(RAD2DEG(roll), 1, 0, 0);


		glBegin(GL_POINTS);
			if (cloud->colours.size() >= cloud->cloud.size()) {
				for(unsigned int i = 0; i < N; i++) {
					Point& p = cloud->cloud[i];
					Colour& c = cloud->colours[i];
					if (p.isFinite()) {
						glColor4f(c.r / 255.0, c.g / 255.0, c.b / 255.0, 0.5);
						glVertex3f(p.x, p.y, p.z);
					}
				}
			} else {
				glColor4f(0, 0, 0, 0.5);
				for(unsigned int i = 0; i < N; i++) {
					Point& p = cloud->cloud[i];
					if (p.isFinite()) {
						glVertex3f(p.x, p.y, p.z);
					}
				}
			}
		glEnd();
	glPopMatrix();
	foo = glGetError();
	if(foo != GL_NO_ERROR){
		ERROR("OpenGL error number AFTER point cloud render. (%s)\n", glutGetGLError(foo).c_str());
	}
}

} // namespace gui

} // namespace crosbot
