/*
 * opengl.h
 *
 *  Created on: 09/02/2010
 *      Author: rescue
 */

#ifndef CROSBOT_OPENGL_HPP_
#define CROSBOT_OPENGL_HPP_

#include <crosbot/data.hpp>
#include <crosbot/geometry.hpp>

#include <GL/glut.h>

namespace crosbot {

namespace gui {

extern bool usePOTTextures;
unsigned long getNextPOT(unsigned long l);
ImagePtr getPOTImage(ImagePtr image);

struct Colour4f {
public:
	float r,g,b,a;

	Colour4f() : r(0), g(0), b(0), a(1) {}
	Colour4f(float r, float g, float b, float a = 1.0) : r(r),g(g),b(b),a(a) {}
	Colour4f(const Colour&c) : r(c.r/255.0), g(c.g/255.0), b(c.b/255.0), a(c.a/255.0) {}

	Colour4f& operator=(const Colour& c) {
		r = (c.r/255.0);
		g = (c.g/255.0);
		b = (c.b/255.0);
		a = (c.a/255.0);

		return *this;
	}
};

class GLLabel {
public:
	float x, y, z;
	std::string label;
	Colour4f colour;

	GLLabel(std::string label = "", float x = 0, float y = 0, float z = 0,
	        Colour4f colour = Colour4f(0,0,0,1)) {
	    this->label = label;
		this->x = x;
		this->y = y;
		this->z = z;
		this->colour = colour;
	}

	GLLabel(const GLLabel& other) {
		this->label = other.label;
		this->x = other.x;
		this->y = other.y;
		this->z = other.z;
		this->colour = other.colour;
	}
};

typedef std::vector<GLLabel> GLTextQueue;

inline std::string glutGetGLError(int err) {
	std::string rval;
	switch (err) {
	case GL_NO_ERROR:
		rval = ""; break;
	case GL_INVALID_ENUM:
		rval = "GL_INVALID_ENUM"; break;
	case GL_INVALID_VALUE:
		rval = "GL_INVALID_VALUE"; break;
	case GL_INVALID_OPERATION:
		rval = "GL_INVALID_OPERATION"; break;
	case GL_STACK_OVERFLOW:
		rval = "GL_STACK_OVERFLOW"; break;
	case GL_STACK_UNDERFLOW:
		rval = "GL_STACK_UNDERFLOW"; break;
	case GL_OUT_OF_MEMORY:
		rval = "GL_OUT_OF_MEMORY"; break;
	case GL_TABLE_TOO_LARGE:
		rval = "GL_TABLE_TOO_LARGE"; break;
	default:
		{
			char strBuff[64];
			sprintf(strBuff, "0x%x", err);
			rval = strBuff;
		} break;
	}
	return rval;
}

void glutWireHemiSphere(double radius, int slices, int stacks);
void glutSolidHemiSphere(double radius, int slices, int stacks);
void glutWireCylinderSegment(double base, double top, double height, double angle, int slices, int stacks);
void glutSolidCylinderSegment(double base, double top, double height, double angle, int slices, int stacks);

} // namespace gui

} // namespace crosbot

#endif /* CROSBOT_OPENGL_HPP_ */
