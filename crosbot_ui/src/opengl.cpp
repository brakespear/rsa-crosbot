/*
 * opengl.cpp
 *
 *  Created on: 09/02/2010
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot_ui/opengl.hpp>
#include <string.h>
#include <crosbot/utils.hpp>

namespace crosbot {

namespace gui {

bool usePOTTextures = false;

unsigned long getNextPOT(unsigned long l) {
	unsigned long mask;
	for (mask = 1; mask; mask *= 2)
		if (mask >= l)
			return mask;
	return mask;
}

ImagePtr getPOTImage(ImagePtr image) {
	// a powerOfTwo larger image

	unsigned long newHeight = getNextPOT(image->height),
			newWidth = getNextPOT(image->width);

	if (newHeight == image->height && newWidth == image->width) {
		return image;
	}

	ImagePtr rval = new Image(image->encoding, newHeight, newWidth);
//
	unsigned int bytesPerPixel = 1, bytesPerLineI, bytesPerLineO;
//
	unsigned char *bytesI = (unsigned char *)image->data,
			*bytesO = (unsigned char *)rval->data;
//	unsigned short *shortsI = (unsigned short *)image->image->image,
//			*shortsO = (unsigned short *)rval->image->image;
//	unsigned int *intsI = (unsigned int *)image->image->image,
//			*intsO = (unsigned int *)rval->image->image;
	switch (image->encoding) {
	case Image::Mono8: case Image::Mono16: case Image::Mono32:
	case Image::RGB8: case Image::BGR8: case Image::RGB16: case Image::BGR16:
	case Image::RGBA8: case Image::BGRA8: case Image::RGBA16: case Image::BGRA16:
			switch (image->encoding) {
			case Image::RGBA16: case Image::BGRA16:
				bytesPerPixel = 8; break;
			case Image::RGB16: case Image::BGR16:
				bytesPerPixel = 6; break;
			case Image::Mono32: case Image::RGBA8: case Image::BGRA8:
				bytesPerPixel = 4; break;
			case Image::RGB8: case Image::BGR8:
				bytesPerPixel = 3; break;
			case Image::Mono16:
				bytesPerPixel = 2; break;
			default:
				break;
			}

			bytesPerLineI = image->width * bytesPerPixel;
			bytesPerLineO = newWidth * bytesPerPixel;

			for (unsigned int i = 0; i < image->height; i++) {
				// copy image line
				memcpy(bytesO, bytesI, bytesPerLineI);
#ifdef BLACK_PAD
				// blackout right pad
				if (bytesPerLineI < bytesPerLineO) {
					memset(bytesO + bytesPerLineI, 0, bytesPerLineO - bytesPerLineI);
				}
#endif
				bytesI += bytesPerLineI;
				bytesO += bytesPerLineO;
			}
#ifdef BLACK_PAD
			// blackout bottom pad
			unsigned int padBelow = (newHeight - image->height);
			if (padBelow + image->getHeight() < newHeight) {
				memset(bytesO, 0, (newHeight - (padBelow + image->getHeight())) * bytesPerLineO);
			}
#endif
		break;
	case Image::YUV422:
		// TODO OpenGL:getPOTImage Image::YUV422
	case Image::YUV420P:
		// TODO OpenGL:getPOTImage Image::YUV420P
	default:
		ERROR("getPOTImage: Unable to convert image format %d to POT image.\n", image->encoding);
		return image;
	}

	return rval;
}

void glutWireHemiSphere(double radius, int slices, int stacks) {
	if (slices < 1 || stacks < 1) {
		return;
	}
	double angPerSlice = M_PI_2l / slices, angPerStack = (2*M_PIl) / stacks;
	double sinSlice[slices], cosSlice[slices], sinStack[stacks], cosStack[stacks];
	sinSlice[0] = sinStack[0] = 0; cosSlice[0] = cosStack[0] = 1;
	for (int i = 1; i < slices; i++) {
		double a = i * angPerSlice;
		sinSlice[i] = sin(a);
		cosSlice[i] = cos(a);
	}
	for (int i = 1; i < stacks; i++) {
		double a = i * angPerStack;
		sinStack[i] = sin(a);
		cosStack[i] = cos(a);
	}

	// Calculate vertices
	Point3D vertices[slices][stacks];
	for (int j = 0; j < stacks; j++) {
		vertices[0][j] = Point3D(radius*cosStack[j], radius*sinStack[j],0);
	}
	for (int i = 1; i < slices; i++) {
		double sliceRadius = cosSlice[i]*radius;
		double sliceHeight = sinSlice[i]*radius;
		for (int j = 0; j < stacks; j++) {
			vertices[i][j] = Point3D(sliceRadius*cosStack[j], sliceRadius*sinStack[j], sliceHeight);
		}
	}

	glPushMatrix();
	glBegin(GL_LINES);

		// Draw latitude lines
		for (int i = 0; i < slices; i++) {
			glVertex3f(vertices[i][0].x, vertices[i][0].y, vertices[i][0].z);

			for (int j = 1; j < stacks; j++) {
				Point3D& p = vertices[i][j];
				glVertex3f(p.x, p.y, p.z);

				glVertex3f(p.x, p.y, p.z);
			}
			glVertex3f(vertices[i][0].x, vertices[i][0].y, vertices[i][0].z);
		}

		// Draw longitude lines
		for (int j = 0; j < stacks; j++) {
			glVertex3f(vertices[0][j].x, vertices[0][j].y, vertices[0][j].z);
			for (int i = 1; i < slices; i++) {
				Point3D& p = vertices[i][j];
				glVertex3f(p.x, p.y, p.z);

				glVertex3f(p.x, p.y, p.z);

			}
			glVertex3f(0, 0, radius);
		}

	glEnd();
	glPopMatrix();
}

void glutSolidHemiSphere(double radius, int slices, int stacks) {
	if (slices < 1 || stacks < 1) {
		return;
	}
	double angPerSlice = M_PI_2l / slices, angPerStack = (2*M_PIl) / stacks;
	double sinSlice[slices], cosSlice[slices], sinStack[stacks], cosStack[stacks];
	sinSlice[0] = sinStack[0] = 0; cosSlice[0] = cosStack[0] = 1;
	for (int i = 1; i < slices; i++) {
		double a = i * angPerSlice;
		sinSlice[i] = sin(a);
		cosSlice[i] = cos(a);
	}
	for (int i = 1; i < stacks; i++) {
		double a = i * angPerStack;
		sinStack[i] = sin(a);
		cosStack[i] = cos(a);
	}

	// Calculate vertices
	Point3D vertices[slices][stacks];
	for (int j = 0; j < stacks; j++) {
		vertices[0][j] = Point3D(radius*cosStack[j], radius*sinStack[j],0);
	}
	for (int i = 1; i < slices; i++) {
		double sliceRadius = cosSlice[i]*radius;
		double sliceHeight = sinSlice[i]*radius;
		for (int j = 0; j < stacks; j++) {
			vertices[i][j] = Point3D(sliceRadius*cosStack[j], sliceRadius*sinStack[j], sliceHeight);
		}
	}

	glPushMatrix();
	glBegin(GL_QUADS);
		for (int i = 0; i < slices - 1; i++) {
			for (int j = 0; j < stacks - 1; j++) {
				Point3D& p1 = vertices[i][j];
				Point3D& p2 = vertices[i][j+1];
				Point3D& p3 = vertices[i+1][j+1];
				Point3D& p4 = vertices[i+1][j];

				glVertex3f(p1.x, p1.y, p1.z);
				glVertex3f(p2.x, p2.y, p2.z);
				glVertex3f(p3.x, p3.y, p3.z);
				glVertex3f(p4.x, p4.y, p4.z);
			}

			Point3D& p1 = vertices[i][stacks-1];
			Point3D& p2 = vertices[i][0];
			Point3D& p3 = vertices[i+1][0];
			Point3D& p4 = vertices[i+1][stacks-1];

			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p2.x, p2.y, p2.z);
			glVertex3f(p3.x, p3.y, p3.z);
			glVertex3f(p4.x, p4.y, p4.z);
		}
	glEnd();

	glBegin(GL_TRIANGLES);
		for (int j = 0; j < stacks-1; j++) {
			Point3D& p1 = vertices[slices-1][j];
			Point3D& p2 = vertices[slices-1][j+1];

			glVertex3f(p1.x, p1.y, p1.z);
			glVertex3f(p2.x, p2.y, p2.z);
			glVertex3f(0, 0, radius);
		}

		Point3D& p1 = vertices[slices-1][stacks-1];
		Point3D& p2 = vertices[slices-1][0];

		glVertex3f(p1.x, p1.y, p1.z);
		glVertex3f(p2.x, p2.y, p2.z);
		glVertex3f(0, 0, radius);
	glEnd();
	glPopMatrix();
}

void glutWireCylinderSegment(double base, double top, double height, double angle, int slices, int stacks) {
	if (slices < 1 || stacks < 1)
		return;

	double angPerStack = angle / stacks;
	double sinStack[stacks+1], cosStack[stacks+1];
	sinStack[0] = 0; cosStack[0] = 1;

	for (int j = 1; j <= stacks; j++) {
		double ang = j * angPerStack;
		sinStack[j] = sin(ang);
		cosStack[j] = cos(ang);
	}

	Point3D vertices[slices+1][stacks+1];

	for (int i = 0; i <= slices; i++) {
		double sliceHeight = (height * i) / slices;
		double sliceRadius = base + ((top - base) * i) / slices;

		for (int j = 0; j <= stacks; j++) {
			vertices[i][j] = Point3D(cosStack[j]*sliceRadius, sinStack[j]*sliceRadius, sliceHeight);
		}
	}

	// Draw Lines

	glPushMatrix();
		glBegin(GL_LINES);

		for (int i = 0; i < slices; i++) {
			for (int j = 0; j < stacks; j++) {
				Point3D& p = vertices[i][j];
				Point3D& p1 = vertices[i][j+1];
				Point3D& p2 = vertices[i+1][j];

				glVertex3f(p.x, p.y, p.z);
				glVertex3f(p1.x, p1.y, p1.z);

				glVertex3f(p.x, p.y, p.z);
				glVertex3f(p2.x, p2.y, p2.z);
			}
			Point3D& p = vertices[i][stacks];
			Point3D& p1 = vertices[i+1][stacks];

			glVertex3f(p.x, p.y, p.z);
			glVertex3f(p1.x, p1.y, p1.z);
		}

		for (int j = 0; j < stacks; j++) {
			Point3D& p = vertices[slices][j];
			Point3D& p1 = vertices[slices][j+1];

			glVertex3f(p.x, p.y, p.z);
			glVertex3f(p1.x, p1.y, p1.z);
		}


		glEnd();
	glPopMatrix();
}

void glutSolidCylinderSegment(double base, double top, double height, double angle, int slices, int stacks) {
	if (slices < 1 || stacks < 1)
		return;

	double angPerStack = angle / stacks;
	double sinStack[stacks+1], cosStack[stacks+1];
	sinStack[0] = 0; cosStack[0] = 1;

	for (int j = 1; j <= stacks; j++) {
		double ang = j * angPerStack;
		sinStack[j] = sin(ang);
		cosStack[j] = cos(ang);
	}

	Point3D vertices[slices+1][stacks+1];

	for (int i = 0; i <= slices; i++) {
		double sliceHeight = (height * i) / slices;
		double sliceRadius = base + ((top - base) * i) / slices;

		for (int j = 0; j <= stacks; j++) {
			vertices[i][j] = Point3D(cosStack[j]*sliceRadius, sinStack[j]*sliceRadius, sliceHeight);
		}
	}

	// Draw Lines

	glPushMatrix();
		glBegin(GL_QUADS);

		for (int i = 0; i < slices; i++) {
			for (int j = 0; j < stacks; j++) {
				Point3D& p = vertices[i][j];
				Point3D& p1 = vertices[i][j+1];
				Point3D& p2 = vertices[i+1][j+1];
				Point3D& p3 = vertices[i+1][j];

				glVertex3f(p.x, p.y, p.z);
				glVertex3f(p1.x, p1.y, p1.z);
				glVertex3f(p2.x, p2.y, p2.z);
				glVertex3f(p3.x, p3.y, p3.z);
			}
		}


		glEnd();
	glPopMatrix();
}

} // namespace gui

} // namespace casros
