/*
 * data.h
 *
 *  Have to define our own classes because ROS is %@&#ing stupid.
 *
 *  Created on: 09/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_H_
#define CROSBOT_DATA_H_


#include <crosbot/geometry.hpp>
#include <crosbot/handle.hpp>

#ifdef ROS_VERSION

#include <crosbot/PointCloudMsg.h>
#include <crosbot/ColouredCloudMsg.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>

#include <netinet/in.h>

#endif

namespace crosbot {

#define NSECS_PER_SEC	1000000000LL
#define DEFAULT_MAXWAIT4TRANSFORM	2.0			// [s]

struct UUID {
public:
	uint64_t value1;
	uint64_t value2;

	UUID() : value1(0), value2(0) {}
	UUID(uint64_t value1, uint64_t value2) : value1(value1), value2(value2) {}
	UUID(std::string uuid) : value1(0), value2(0) {
		uint64_t v1, v2, v3, v4, v5;
#if __WORDSIZE == 64
		int n = sscanf(uuid.c_str(), "%lx-%lx-%lx-%lx-%lx",
#else
		int n = sscanf(uuid.c_str(), "%llx-%llx-%llx-%llx-%llx",
#endif
		&v1, &v2, &v3, &v4, &v5);

		if (n == 5) {
			value1 = ((v1 & 0xFFFFFFFFLU) << 32) | ((v2 & 0xFFFFLU) << 16) | (v3 & 0xFFFFLU);
			value2 = ((v4 & 0xFFFFLU) << 48) | (v5 & 0xFFFFFFFFFFFF);
		} else if (n >= 2) {
			value1 = v1;
			value2 = v2;
		} else if (n == 1) {
			value1 = v1;
		}
	}

	std::string toString() const {
		char uuidStr[256];

#if __WORDSIZE == 64
		sprintf(uuidStr, "%08lx-%04lx-%04lx-%04lx-%012lx",
#else
		sprintf(uuidStr, "%08llx-%04llx-%04llx-%04llx-%012llx",
#endif
				(value1 & 0xFFFFFFFF00000000) >> 32,
				(value1 & 0x00000000FFFF0000LU) >> 16, value1 & 0x000000000000FFFFLU,
				(value2 & 0xFFFF000000000000LU) >> 48, value2 & 0x0000FFFFFFFFFFFFLU);
		return std::string(uuidStr);
	}

	inline bool operator==(const UUID& other) const {
		return value1 == other.value1 && value2 == other.value2;
	}
};

/**
 * Duration class based on ros::Duration
 */
struct Duration {
	int32_t sec;
	int32_t nsec;

	Duration() : sec(0), nsec(0) {}
	Duration(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}
	Duration(int64_t nsec) : sec(nsec/NSECS_PER_SEC), nsec(nsec%NSECS_PER_SEC) {}
	Duration(double sec) : sec(sec), nsec((sec-this->sec)*NSECS_PER_SEC) {}

	inline int64_t toNSec() const { return sec * NSECS_PER_SEC + nsec; }
	inline double toSec() const { return sec + ((double)nsec)/NSECS_PER_SEC; }

	inline bool isZero() const { return sec == 0 && nsec == 0; }

	inline bool operator==(const Duration& d) const { return sec == d.sec && nsec == d.nsec; }

	inline bool operator!=(const Duration& d) const { return sec != d.sec || nsec != d.nsec; }

	inline bool operator<(const Duration& d) const {
		if (sec == d.sec) {
			return nsec < d.nsec;
		} else {
			return sec < d.sec;
		}
	}

	inline bool operator<=(const Duration& d) const {
		if (sec == d.sec) {
			return nsec <= d.nsec;
		} else {
			return sec < d.sec;
		}
	}

	inline bool operator>(const Duration& d) const {
		if (sec == d.sec) {
			return nsec > d.nsec;
		} else {
			return sec > d.sec;
		}
	}

	inline bool operator>=(const Duration& d) const {
		if (sec == d.sec) {
			return nsec >= d.nsec;
		} else {
			return sec > d.sec;
		}
	}

	inline Duration operator+(const Duration& d) const {
		int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
		return Duration(nsecs);
	}

	inline Duration operator-(const Duration& d) const {
		int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
		return Duration(nsecs);
	}

	inline Duration operator*(const double scale) const {
		int64_t nsecs = sec * NSECS_PER_SEC + nsec;
		nsecs *= scale;
		return Duration(nsecs);
	}

	inline Duration operator/(const double scale) const {
		int64_t nsecs = sec * NSECS_PER_SEC + nsec;
		nsecs /= scale;
		return Duration(nsecs);
	}

	inline Duration& operator+=(const Duration& d) {
		int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

	inline Duration& operator-=(const Duration& d) {
		int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

	inline Duration& operator*=(const double scale) {
		int64_t nsecs = sec * NSECS_PER_SEC + nsec;
		nsecs *= scale;
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

	inline Duration& operator/=(const double scale) {
		int64_t nsecs = sec * NSECS_PER_SEC + nsec;
		nsecs /= scale;
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

#ifdef ROS_VERSION

	Duration(const ros::Duration& d) : sec(d.sec), nsec(d.nsec) {}

	inline Duration& operator=(const ros::Duration& d) {
		sec = d.sec; nsec = d.nsec;
		return *this;
	}

    inline ros::Duration toROS() const {
    	return ros::Duration(sec, nsec);
    }

#endif
};

/**
 * Time class based on ros::Time
 */
struct Time {
public:
	int32_t sec;
	int32_t nsec;

	Time() : sec(0), nsec(0) {}
	Time(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}
	Time(int64_t nsec) : sec(nsec/NSECS_PER_SEC), nsec(nsec%NSECS_PER_SEC) {}
	Time(double sec) : sec(sec), nsec((sec-this->sec)*NSECS_PER_SEC) {}

	static Time now();

	inline int64_t toNSec() { return sec * NSECS_PER_SEC + nsec; }
	inline double toSec() { return sec + ((double)nsec)/NSECS_PER_SEC; }

	inline bool isZero() const { return sec == 0 && nsec == 0; }

	inline bool operator==(const Time& t) const { return sec == t.sec && nsec == t.nsec; }

	inline bool operator!=(const Time& t) const { return sec != t.sec || nsec != t.nsec; }

	inline bool operator<(const Time& t) const {
		if (sec == t.sec) {
			return nsec < t.nsec;
		} else {
			return sec < t.sec;
		}
	}

	inline bool operator<=(const Time& t) const {
		if (sec == t.sec) {
			return nsec <= t.nsec;
		} else {
			return sec < t.sec;
		}
	}

	inline bool operator>(const Time& t) const {
		if (sec == t.sec) {
			return nsec > t.nsec;
		} else {
			return sec > t.sec;
		}
	}

	inline bool operator>=(const Time& t) const {
		if (sec == t.sec) {
			return nsec >= t.nsec;
		} else {
			return sec > t.sec;
		}
	}

	inline Time operator+(const Duration& d) const {
		int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
		return Time(nsecs);
	}

	inline Time operator-(const Duration& d) const {
		int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
		return Time(nsecs);
	}

	inline Duration operator-(const Time& t) const {
		int64_t nsecs = (sec - t.sec) * NSECS_PER_SEC + (nsec - t.nsec);
		return Duration(nsecs);
	}

	inline Time& operator+=(const Duration& d) {
		int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

	inline Time& operator-=(const Duration& d) {
		int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
		sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
		return *this;
	}

	std::string formatDate() const;
	std::string formatDateAndTime() const;

#ifdef ROS_VERSION

	Time(const ros::Time& t) : sec(t.sec), nsec(t.nsec) {}

	inline Time& operator=(const ros::Time& t) {
		sec = t.sec; nsec = t.nsec;
		return *this;
	}

    inline ros::Time toROS() const {
    	return ros::Time(sec, nsec);
    }

#endif
};

struct Colour {
public:
	uint8_t r, g, b, a;

	Colour() : r(0), g(0), b(0), a(255) {}
	Colour(uint8_t r, uint8_t g, uint8_t b) : r(r), g(g), b(b), a(255) {}
	Colour(uint8_t r, uint8_t g, uint8_t b, uint8_t a) : r(r), g(g), b(b), a(a) {}
	Colour(const Colour& colour) : r(colour.r), g(colour.g), b(colour.b), a(colour.a) {}

#ifdef ROS_VERSION

	Colour(const ColourMsg& c) : r(c.r), g(c.g), b(c.b), a(c.a) {}

	inline Colour& operator=(const ColourMsg& c) {
		r = c.r; g = c.g; b = c.b; a = c.a;
		return *this;
	}

    inline ColourMsg toROS() const {
    	ColourMsg rval;
    	rval.r = r; rval.g = g; rval.b = b; rval.a = a;
    	return rval;
    }

#endif
};
inline std::ostream& operator<<(std::ostream& os, const Colour& c) {
	return os << "Colour(" << c.r << ", " << c.g << ", " << c.b << ", " << c.a << ")";
}

struct ColouredPoint {
public:
	crosbot::Point3D point;
	Colour colour;

	ColouredPoint() {}
	ColouredPoint(Point p) : point(p) {}
	ColouredPoint(Point p, Colour c) : point(p), colour(c) {}
	ColouredPoint(double x, double y, double z) : point(x,y,z) {}
	ColouredPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b) : point(x,y,z), colour(r,g,b) {}
	ColouredPoint(double x, double y, double z, uint8_t r, uint8_t g, uint8_t b, uint8_t a) : point(x,y,z), colour(r,g,b,a) {}

	inline bool hasNAN() const { return point.hasNAN(); }

#ifdef ROS_VERSION

	ColouredPoint(const ColouredPointMsg& cp) : point(cp.p), colour(cp.c) {}

	inline ColouredPoint& operator=(const ColouredPointMsg& cp) {
		point = cp.p; colour = cp.c;
		return *this;
	}

    inline ColouredPointMsg toROS() const {
    	ColouredPointMsg rval;

    	rval.p.x = point.x;
    	rval.p.y = point.y;
    	rval.p.z = point.z;

    	rval.c.r = colour.r;
    	rval.c.g = colour.g;
    	rval.c.b = colour.b;
    	rval.c.a = colour.a;

    	return rval;
    }

#endif
};
inline std::ostream& operator<<(std::ostream& os, const ColouredPoint& cp) {
	return os << "ColouredPoint(" << cp.point << ", " << cp.colour << ")";
}


/**
 * A superclass for more complex data classes that keep a generated timestamp.
 */
class Data : public HandledObject {
public:
	Time timestamp;

	Data(Time stamp = Time::now()) : timestamp(stamp) {}
	virtual ~Data() {}
};

/**
 * create image class compatible with ros::Image class
 */
class Image;
typedef Handle<Image> ImagePtr;
class Image : public Data {
public:
	enum Encoding {
		// Unknown format
		Unknown = 0,

		// Grey Scale
		Mono8 = 1, Mono16 = 2, Mono32 = 4,

		// Colour
		RGB8 = 11, RGBA8 = 12, BGR8 = 13, BGRA8 = 14,
		RGB16 = 21, RGBA16 = 22, BGR16 = 23, BGRA16 = 24,
		YUV422 = 31, YUV420P = 35,

		// Bayer Encodings
	    BAYER_RGGB8 = 51, BAYER_BGGR8 = 52, BAYER_GBRG8 = 53, BAYER_GRBG8 = 54,
	    BAYER_RGGB16 = 61, BAYER_BGGR16 = 62, BAYER_GBRG16 = 63, BAYER_GRBG16 = 64,

	    // Compressed
	    JPEG = 101, PNG = 102,
	};

	static int bitDepth(Encoding encoding);
	static int numberOfChannels(Encoding encoding);
	static bool isBayer(Encoding encoding);
	static bool isCompressed(Encoding encoding);

	static inline const char* defaultSuffix(Encoding encoding) {
		switch (encoding) {
		case Mono8: case Mono16: case Mono32:
			return "pgm"; break;
		case RGB8: case RGB16:
			return "ppm"; break;
		case JPEG:
			return "jpg"; break;
		case PNG:
			return "png"; break;
		default:
			return "pam"; break;
		}
		return "pam";
	}

	unsigned int height, width;
	Encoding encoding;
	unsigned int step;
	uint64_t dataLength;
	void *data;

	Image() :
		height(0), width(0), encoding(Unknown),
		step(0), data(NULL)
	{}

	Image(Encoding encoding, unsigned int height, unsigned int width);
	Image(const Image& image, Encoding encoding);
	~Image();

	ImagePtr inEncoding(Encoding encoding) const {
		if (encoding == this->encoding)
			return this;
		return new Image(*this, encoding);
	}

	void writeToFile(std::string filename) throw (IOException);
	static ImagePtr readFromFile(std::string filename) throw (IOException);

#ifdef ROS_VERSION
	static std::string toROS(Image::Encoding e) {
		switch (e) {
		case JPEG: case PNG: case Mono32: case YUV420P: case Unknown: break;
		case RGB8:
			return sensor_msgs::image_encodings::RGB8;
			break;
		case RGB16:
			return sensor_msgs::image_encodings::RGB16;
			break;
		case RGBA8:
			return sensor_msgs::image_encodings::RGBA8;
			break;
		case RGBA16:
			return sensor_msgs::image_encodings::RGBA16;
			break;
		case BGR8:
			return sensor_msgs::image_encodings::BGR8;
			break;
		case BGR16:
			return sensor_msgs::image_encodings::BGR16;
			break;
		case BGRA8:
			return sensor_msgs::image_encodings::BGRA8;
			break;
		case BGRA16:
			return sensor_msgs::image_encodings::BGRA16;
			break;
		case Mono8:
			return sensor_msgs::image_encodings::MONO8;
			break;
		case Mono16:
			return sensor_msgs::image_encodings::MONO16;
			break;
		case YUV422:
			return sensor_msgs::image_encodings::YUV422;
			break;
		case BAYER_RGGB8:
			return sensor_msgs::image_encodings::BAYER_RGGB8;
			break;
		case BAYER_BGGR8:
			return sensor_msgs::image_encodings::BAYER_BGGR8;
			break;
		case BAYER_GRBG8:
			return sensor_msgs::image_encodings::BAYER_GRBG8;
			break;
		case BAYER_GBRG8:
			return sensor_msgs::image_encodings::BAYER_GBRG8;
			break;
		case BAYER_RGGB16:
			return sensor_msgs::image_encodings::BAYER_RGGB16;
			break;
		case BAYER_BGGR16:
			return sensor_msgs::image_encodings::BAYER_BGGR16;
			break;
		case BAYER_GRBG16:
			return sensor_msgs::image_encodings::BAYER_GRBG16;
			break;
		case BAYER_GBRG16:
			return sensor_msgs::image_encodings::BAYER_GBRG16;
			break;
		}

		return "";
	}

	static Image::Encoding fromROS(std::string e) {
		if (e == sensor_msgs::image_encodings::MONO8 || e == sensor_msgs::image_encodings::TYPE_8UC1) {
			return Mono8;
		} else if (e == sensor_msgs::image_encodings::MONO16 || e == sensor_msgs::image_encodings::TYPE_16UC1) {
			return Mono16;
		} else if (e == sensor_msgs::image_encodings::YUV422) {
			return YUV422;
		} else if (e == sensor_msgs::image_encodings::RGB8) {
			return RGB8;
		} else if (e == sensor_msgs::image_encodings::RGBA8) {
			return RGBA8;
		} else if (e == sensor_msgs::image_encodings::BGR8 || e == sensor_msgs::image_encodings::TYPE_8UC3) {
			return BGR8;
		} else if (e == sensor_msgs::image_encodings::BGRA8 || e == sensor_msgs::image_encodings::TYPE_8UC4) {
			return BGRA8;
		} else if (e == sensor_msgs::image_encodings::RGB16) {
			return RGB16;
		} else if (e == sensor_msgs::image_encodings::RGBA16) {
			return RGBA16;
		} else if (e == sensor_msgs::image_encodings::BGR16 || e == sensor_msgs::image_encodings::TYPE_16UC3) {
			return BGR16;
		} else if (e == sensor_msgs::image_encodings::BGRA16 || e == sensor_msgs::image_encodings::TYPE_16UC4) {
			return BGRA16;
		} else if (e == sensor_msgs::image_encodings::BAYER_RGGB8) {
			return BAYER_RGGB8;
		} else if (e == sensor_msgs::image_encodings::BAYER_BGGR8) {
			return BAYER_BGGR8;
		} else if (e == sensor_msgs::image_encodings::BAYER_GRBG8) {
			return BAYER_GRBG8;
		} else if (e == sensor_msgs::image_encodings::BAYER_GBRG8) {
			return BAYER_GBRG8;
		} else if (e == sensor_msgs::image_encodings::BAYER_RGGB16) {
			return BAYER_RGGB16;
		} else if (e == sensor_msgs::image_encodings::BAYER_BGGR16) {
			return BAYER_BGGR16;
		} else if (e == sensor_msgs::image_encodings::BAYER_GRBG16) {
			return BAYER_GRBG16;
		} else if (e == sensor_msgs::image_encodings::BAYER_GBRG16) {
			return BAYER_GBRG16;
		}
		return Unknown;
	}

	Image(const sensor_msgs::Image& image) : dataLength(0), data(NULL) {
		*this = image;
	}

	Image(const sensor_msgs::ImageConstPtr& image) : dataLength(0), data(NULL) {
		*this = image;
	}

	inline Image& operator=(const sensor_msgs::Image& image) {
		timestamp = image.header.stamp;
		width = image.width;
		height = image.height;
		step = image.step;
		encoding = fromROS(image.encoding);

		uint64_t dl = step*height;
		if (dataLength < dl && data != NULL) {
			free(data); data = NULL;
		}
		dataLength = dl;
		if (data == NULL) {
			data = malloc(dataLength);
		}

		if (bitDepth(encoding) == 16 && image.is_bigendian) {
			uint32_t nPL = width * numberOfChannels(encoding);
			for (uint32_t i = 0; i < height; ++i) {
				uint16_t* in = (uint16_t*)&(image.data[i*step]);
				uint16_t* out = (uint16_t*)(((uint8_t *)data)+(i*step));

				for (uint32_t j = 0; j < nPL; ++j) {
					out[j] = ntohs(in[j]);
				}
			}
		} else {
			memcpy(data, &(image.data[0]), dataLength);
		}

		return *this;
	}

	inline Image& operator=(const sensor_msgs::ImageConstPtr& image) {
		return operator=(*image);
	}

    inline sensor_msgs::ImagePtr toROS() const {
    	if (encoding == JPEG || encoding == PNG)
    		return inEncoding(RGB8)->toROS();
    	else if (encoding == Mono32)
    		return inEncoding(Mono16)->toROS();

    	sensor_msgs::ImagePtr rval(new sensor_msgs::Image());
    	rval->header.stamp = timestamp.toROS();
    	rval->width = width;
    	rval->height = height;
    	rval->step = step;

    	rval->is_bigendian = false;
    	rval->encoding = toROS(encoding);

		rval->data.resize(dataLength);
		memcpy(&(rval->data[0]), data, dataLength);

    	return rval;
    }

#endif
};

class PointCloud;
typedef Handle<PointCloud> PointCloudPtr;
class PointCloud : public Data {
public:
	/**
	 * The frame of reference the points in the cloud are in.
	 */
	std::string frameID;

	/**
	 * The set of points in the cloud.
	 */
	std::vector<Point> cloud;

	/**
	 * The set colours for each point in the cloud.  This can be empty if the points aren't coloured.
	 */
	std::vector<Colour> colours;

	PointCloud() : frameID("") {}
	PointCloud(std::string frameID) : frameID(frameID) {}
	PointCloud(const PointCloud& pc) : Data(pc.timestamp), frameID(pc.frameID) { cloud = pc.cloud; colours = pc.colours; }
	PointCloud(std::string frameID, const PointCloud&, Pose correction);

	inline bool hasNAN() {
		for (size_t i = 0; i < cloud.size(); i++) {
			if (cloud[i].hasNAN())
				return true;
		}

		return false;
	}

#ifdef ROS_VERSION

	// TODO: read colours from ROS messages
	PointCloud(const PointCloudMsg& c) {
		timestamp = c.header.stamp;
		frameID = c.header.frame_id;

		size_t n = c.points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c.points[i];
		}
	}

	PointCloud(const PointCloudMsgConstPtr& c) {
		timestamp = c->header.stamp;
		frameID = c->header.frame_id;

		size_t n = c->points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c->points[i];
		}
	}

	PointCloud(const sensor_msgs::PointCloud& c) {
		timestamp = c.header.stamp;
		frameID = c.header.frame_id;

		size_t n = c.points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c.points[i];
		}
	}

	PointCloud(const sensor_msgs::PointCloudConstPtr& c) {
		timestamp = c->header.stamp;
		frameID = c->header.frame_id;

		size_t n = c->points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c->points[i];
		}
	}

	PointCloud(const sensor_msgs::LaserScan& ls) {
		timestamp = ls.header.stamp;
		frameID = ls.header.frame_id;

		Point p, origin;
		float angle = ls.angle_min, angleInc = ls.angle_increment, range,
				rangeMin = ls.range_min, rangeMax = ls.range_max;

		size_t n = ls.ranges.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; ++i, angle += angleInc) {
			range = ls.ranges[i];
			if (range >= rangeMin && range <= rangeMax) {
				p.x = cos(angle) * range;
				p.y = sin(angle) * range;

				cloud[i] = p;
			} else {
				cloud[i] = origin;
			}
		}
	}

	PointCloud(const sensor_msgs::LaserScanConstPtr& ls, bool includeOutOfRange = false) {
		timestamp = ls->header.stamp;
		frameID = ls->header.frame_id;

		Point p;
		float angle = ls->angle_min, angleInc = ls->angle_increment, range,
				rangeMin = ls->range_min, rangeMax = ls->range_max;
		uint32_t discardedReadings = 0;

		size_t n = ls->ranges.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; ++i, angle += angleInc) {
			range = ls->ranges[i];
			if (includeOutOfRange || (range >= rangeMin && range <= rangeMax)) {
				p.x = cos(angle) * range;
				p.y = sin(angle) * range;

				cloud[i - discardedReadings] = p;
			} else {
				++discardedReadings;
			}
		}
		cloud.resize(n - discardedReadings);
	}

	inline PointCloud& operator=(const sensor_msgs::PointCloud& c) {
		timestamp = c.header.stamp;
		frameID = c.header.frame_id;

		size_t n = c.points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c.points[i];
		}
		return *this;
	}

	inline PointCloud& operator=(const sensor_msgs::PointCloudConstPtr& c) {
		timestamp = c->header.stamp;
		frameID = c->header.frame_id;

		size_t n = c->points.size();
		cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			cloud[i] = c->points[i];
		}
		return *this;
	}

    inline PointCloudMsgPtr toROS() const {
    	PointCloudMsgPtr rval(new PointCloudMsg());
    	rval->header.stamp = timestamp.toROS();
    	rval->header.frame_id = frameID;

		size_t n = cloud.size();
		rval->points.resize(n);
		for (size_t i = 0; i < n; i++) {
			rval->points[i] = cloud[i].toROS();
		}

    	return rval;
    }

    inline sensor_msgs::PointCloudPtr tosROS1() const {
    	sensor_msgs::PointCloudPtr rval(new sensor_msgs::PointCloud());
    	rval->header.stamp = timestamp.toROS();
    	rval->header.frame_id = frameID;

    	rval->points.resize(cloud.size());

    	for (size_t i = 0; i < cloud.size(); ++i) {
    		rval->points[i] = cloud[i].toROS32();
    	}

    	// Add colour channels
    	if (colours.size() > 0) {
    		rval->channels.resize(1);
    		sensor_msgs::ChannelFloat32& channel = rval->channels[0];
    		channel.name = "rgb";
    		channel.values.resize(cloud.size());

    		uint32_t c32;
        	for (size_t i = 0; i < cloud.size(); ++i) {
        		const Colour& colour = colours[i];
        		c32 = ((uint32_t)colour.r) << 16 | ((uint32_t)colour.g) << 8 | ((uint32_t)colour.b);
        		channel.values[i] = *(float*)(&c32);
        	}
    	}

    	return rval;
    }

    PointCloud(const sensor_msgs::PointCloud2ConstPtr& c);
    PointCloud(const sensor_msgs::PointCloud2& c);
    PointCloud& operator=(const sensor_msgs::PointCloud2ConstPtr& c);
    PointCloud& operator=(const sensor_msgs::PointCloud2& c);
#endif
};

#ifdef ROS_VERSION
// XXX: Why does ROS not generate an equality operator?

inline bool operator==(const ColourMsg& c1, const ColourMsg& c2) {
    return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b && c1.a == c2.a;
}

inline bool operator==(const ColouredPointMsg& p1, const ColouredPointMsg& p2) {
    return p1.p == p2.p && p1.c == p2.c;
}

/**
 * Tests that a value isn't corrupt.
 */
inline bool hasNAN(const ColouredPointMsg& cp) {
    return hasNAN(cp.p);
}

inline bool hasNAN(const PointCloudMsg& pc) {
	for (size_t i = 0; i < pc.points.size(); i++) {
		if (hasNAN(pc.points[i]))
			return true;
	}
	return false;
}

inline bool hasNAN(const ColouredCloudMsg& pc) {
	for (size_t i = 0; i < pc.points.size(); i++) {
		if (hasNAN(pc.points[i]))
			return true;
	}
	return false;
}

#endif // ROS_VERSION

namespace serialization {

CASROS_SIMPLE_SERIALIZER(UUID);
CASROS_SIMPLE_SERIALIZER(Time);
CASROS_SIMPLE_SERIALIZER(Duration);
CASROS_SIMPLE_SERIALIZER(Colour);
CASROS_SIMPLE_SERIALIZER(ColouredPoint);

template <>
class Serializer<PointCloud> {
public:
	size_t write(const PointCloud& cloud, OutputStream& stream) throw (IOException) {
		Serializer<Time> serTime;
		Serializer<uint64_t> serUint64;
		Serializer<Point> serPoint;
		Serializer<std::string> serString;
		Serializer<Colour> serColour;

		size_t rval = serTime.write(cloud.timestamp, stream);
		rval += serString.write(cloud.frameID, stream);

		uint64_t n = cloud.cloud.size();
		rval += serUint64.write(n , stream);
		for (size_t i = 0; i < n; i++) {
			rval += serPoint.write(cloud.cloud[i], stream);
		}

		n = cloud.colours.size();
		rval += serUint64.write(n , stream);
		for (size_t i = 0; i < n; i++) {
			rval += serColour.write(cloud.colours[i], stream);
		}
		return rval;
	}

	size_t read(PointCloud& cloud, InputStream& stream) throw (IOException) {
		Serializer<Time> serTime;
		Serializer<uint64_t> serUint64;
		Serializer<Point> serPoint;
		Serializer<std::string> serString;
		Serializer<Colour> serColour;

		size_t rval = serTime.read(cloud.timestamp, stream);
		rval += serString.read(cloud.frameID, stream);

		uint64_t n;
		rval += serUint64.read(n , stream);
		cloud.cloud.resize(n);
		for (size_t i = 0; i < n; i++) {
			rval += serPoint.read(cloud.cloud[i], stream);
		}
		rval += serUint64.read(n , stream);
		cloud.colours.resize(n);
		for (size_t i = 0; i < n; i++) {
			rval += serColour.read(cloud.colours[i], stream);
		}
		return rval;
	}

	size_t serializedLength(const PointCloud& cloud) {
		Serializer<std::string> serString;
		return sizeof(Time) + serString.serializedLength(cloud.frameID) +
				sizeof(uint64_t) + sizeof(Point) * cloud.cloud.size() +
				sizeof(uint64_t) + sizeof(Colour) * cloud.colours.size();
	}
};

template <>
class Serializer<Image> {
public:
	size_t write(const Image& image, OutputStream& stream) throw (IOException) {
		Serializer<Time> serTime;
		Serializer<uint32_t> serUInt32;
		Serializer<uint64_t> serUInt64;

		size_t rval = serTime.write(image.timestamp, stream);
		rval += serUInt32.write(image.height, stream);
		rval += serUInt32.write(image.width, stream);
		rval += serUInt32.write((uint32_t)image.encoding, stream);
		rval += serUInt32.write(image.step, stream);
		rval += serUInt64.write(image.dataLength, stream);

		if (image.dataLength > 0) {
			if (stream.write(image.data, image.dataLength) != image.dataLength) {
				throw IOException("Error writing image data to stream.");
			}
		}
		rval += image.dataLength;
		return rval;
	}

	size_t read(Image& image, InputStream& stream) throw (IOException) {
		Serializer<Time> serTime;
		Serializer<uint32_t> serUInt32;
		Serializer<uint64_t> serUInt64;

		size_t rval = serTime.read(image.timestamp, stream);
		rval += serUInt32.read(image.height, stream);
		rval += serUInt32.read(image.width, stream);
		uint32_t encoding;
		rval += serUInt32.read(encoding, stream);
		image.encoding = (Image::Encoding) encoding;
		rval += serUInt32.read(image.step, stream);
		uint64_t oldLength = image.dataLength;
		rval += serUInt64.read(image.dataLength, stream);

		if (image.dataLength == 0) {
			if (image.data != NULL) {
				free(image.data);
			}
		} else {
			if (image.dataLength > oldLength || image.data == NULL) {
				if (image.data != NULL)
					free(image.data);
				image.data = malloc(image.dataLength);
				if (image.data == NULL)
					throw IOException("Out of memory for storing image data.");
			}

			if (stream.read(image.data, image.dataLength) != image.dataLength) {
				throw IOException("Error reading image data.");
			}
			rval += image.dataLength;
		}
		return rval;
	}

	size_t serializedLength(const Image& image) {
		return sizeof(Time) + // timestamp
				sizeof(unsigned int) + sizeof(unsigned int) + // height & width
				sizeof(unsigned int) +	// encoding
				sizeof(unsigned int) +	// step
				sizeof(uint64_t) +		// dataLength
				image.dataLength;
	}
};

} // namespace serialization

} // namespace crosbot

#endif /* CROSBOT_DATA_H_ */
