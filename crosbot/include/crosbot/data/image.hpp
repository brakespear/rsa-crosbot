/*
 * image.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_IMAGE_HPP_
#define CROSBOT_DATA_IMAGE_HPP_

#include <cstdint>
#include <string>

#include <crosbot/data/defines.hpp>
#include <crosbot/data/timestampedData.hpp>
#include <crosbot/exceptions.hpp>

#ifdef ROS_VERSION
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <netinet/in.h>
#endif

namespace crosbot {

class Image;
typedef Handle<Image> ImagePtr;

/**
 * Image class compatible with ros::Image class
 */
class Image : public TimeStamptedData {
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
        step(0), dataLength(0), data(NULL)
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

} // namespace crosbot

#endif /* CROSBOT_DATA_IMAGE_HPP_ */
