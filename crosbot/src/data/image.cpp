/*
 * image.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */



#include <ros/ros.h>

#include <crosbot/data/image.hpp>
#include <crosbot/utils.hpp>

#include "../image.hpp"

namespace crosbot {

int Image::bitDepth(Encoding encoding) {
    switch (encoding) {
    case Mono8: case RGB8: case RGBA8: case BGR8: case BGRA8:
    case BAYER_RGGB8: case BAYER_BGGR8: case BAYER_GBRG8: case BAYER_GRBG8:
    case YUV422: case YUV420P:
        return 8; break;
    case Mono16: case RGB16: case RGBA16: case BGR16: case BGRA16:
    case BAYER_RGGB16: case BAYER_BGGR16: case BAYER_GBRG16: case BAYER_GRBG16:
        return 16; break;
    case Mono32:
        return 32; break;
    case JPEG: case PNG: case Unknown:
        return 0;
    }
    return 0;
}

int Image::numberOfChannels(Encoding encoding) {
    switch (encoding) {
    case Mono8: case Mono16: case Mono32:
    case BAYER_RGGB8: case BAYER_BGGR8: case BAYER_GBRG8: case BAYER_GRBG8:
    case BAYER_RGGB16: case BAYER_BGGR16: case BAYER_GBRG16: case BAYER_GRBG16:
        return 1; break;
    case RGB8: case BGR8: case RGB16: case BGR16:
        return 3; break;
    case RGBA8: case BGRA8: case RGBA16: case BGRA16:
        return 4; break;
    case YUV422: case YUV420P:
    case JPEG: case PNG: case Unknown:
        return 0;
    }
    return 0;
}

bool Image::isBayer(Encoding encoding) {
    return encoding == Image::BAYER_RGGB8 || encoding == Image::BAYER_BGGR8 ||
            encoding == Image::BAYER_GRBG8 || encoding == Image::BAYER_GBRG8 ||
            encoding == Image::BAYER_RGGB16 || encoding == Image::BAYER_BGGR16 ||
            encoding == Image::BAYER_GRBG16 || encoding == Image::BAYER_GBRG16;
}

bool Image::isCompressed(Encoding encoding) {
    return encoding == Image::JPEG || encoding == Image::PNG;
}

Image::Image(Encoding encoding, unsigned int height, unsigned int width) :
    height(height), width(width), encoding(encoding),
    dataLength(0), data(NULL)
{
    switch (encoding) {
    case Mono8: case Mono16: case Mono32:
    case BAYER_RGGB8: case BAYER_BGGR8: case BAYER_GBRG8: case BAYER_GRBG8:
    case BAYER_RGGB16: case BAYER_BGGR16: case BAYER_GBRG16: case BAYER_GRBG16:
    case RGB8: case BGR8: case RGB16: case BGR16:
    case RGBA8: case BGRA8: case RGBA16: case BGRA16:
        step = width * ((bitDepth(encoding) / 8)) * numberOfChannels(encoding);
        dataLength = height*step;
        break;
    case YUV422:
        step = (width + (width%2)) * 2;
        dataLength = step * height;
        break;
    case YUV420P:
        // There is 1 Y for every pixel + 1 U & 1V for every 2*2 quad
        step = width;
        dataLength = width * height +
                2 * ((width + 1) / 2) * ((height + 1) / 2);
        break;
    case JPEG:
        step = 0;
        dataLength = width * height * 3;
        break;
    case Unknown: case PNG:
        step = dataLength = 0;
        break;
    }

    if (dataLength != 0) {
        data = malloc(dataLength);
        if (data == NULL)
            dataLength = 0;
    }
}

Image::Image(const Image& image, Encoding encoding) :
    height(image.height), width(image.width), encoding(encoding),
    dataLength(0), data(NULL)
{
//  LOG("Converting image to encoding %d from %d\n", encoding, image.encoding);
    timestamp = image.getTimestamp();
    switch (encoding) {
    case Mono8: case Mono16: case Mono32:
    case BAYER_RGGB8: case BAYER_BGGR8: case BAYER_GBRG8: case BAYER_GRBG8:
    case BAYER_RGGB16: case BAYER_BGGR16: case BAYER_GBRG16: case BAYER_GRBG16:
    case RGB8: case BGR8: case RGB16: case BGR16:
    case RGBA8: case BGRA8: case RGBA16: case BGRA16:
        step = width * ((bitDepth(encoding) / 8)) * numberOfChannels(encoding);
        dataLength = height*step;
        break;
    case YUV422:
        step = (width + (width%2)) * 2;
        dataLength = step * height;
        break;
    case YUV420P:
        // There is 1 Y for every pixel + 1 U & 1V for every 2*2 quad
        step = width;
        dataLength = width * height +
                2 * ((width + 1) / 2) * ((height + 1) / 2);
        break;
    case JPEG: case PNG:
        step = 0;
        dataLength = width * height * 3;
        break;
    case Unknown:
        step = dataLength = 0;
        break;
    }

    if (dataLength != 0) {
        data = malloc(dataLength);
        if (data == NULL)
            dataLength = 0;
    }
    if (data == NULL)
        return;

    if (!image::convertImage(*this, image)) {
        ERROR("Image: Problem converting to %d encoding.\n", encoding);
        dataLength = 0;
        free(data);
        data = NULL;
    }
}

Image::~Image() {
    if (data != NULL) {
        free(data);
        data = NULL;
    }
}

void Image::writeToFile(std::string filename) throw (IOException) {
    if (data == NULL)
        throw IOException("Cannot write image with no data.\n");
    size_t suffixN = filename.find_last_of(".");
    if (suffixN == filename.npos) {
        image::writePAM(filename, *this);
        return;
    }
    std::string suffix = filename.substr(suffixN+1, filename.size()-suffixN);

    if (strcasecmp(suffix.c_str(), "jpeg") == 0 ||
            strcasecmp(suffix.c_str(), "jpg") == 0 ||
            strcasecmp(suffix.c_str(), "jpe") == 0) {
        image::writeJPEG(filename, *this);
    } else if (strcasecmp(suffix.c_str(), "png") == 0) {
        image::writePNG(filename, *this);
    } else if (strcasecmp(suffix.c_str(), "bmp") == 0) {
        image::writeBMP(filename, *this);
    } else if (strcasecmp(suffix.c_str(), "pgm") == 0) {
        image::writePGM(filename, *this);
    } else if (strcasecmp(suffix.c_str(), "ppm") == 0) {
        image::writePPM(filename, *this);
    } else {
        image::writePAM(filename, *this);
    }
}

ImagePtr Image::readFromFile(std::string filename) throw (IOException) {
    size_t suffixN = filename.find_last_of(".");
    if (suffixN == filename.npos) {
        return image::readPBM(filename);
    }
    std::string suffix = filename.substr(suffixN+1, filename.size()-suffixN);

    if (strcasecmp(suffix.c_str(), "jpeg") == 0 ||
            strcasecmp(suffix.c_str(), "jpg") == 0 ||
            strcasecmp(suffix.c_str(), "jpe") == 0) {
        return image::readJPEG(filename);
    } else if (strcasecmp(suffix.c_str(), "png") == 0) {
        return image::readPNG(filename);
    } else if (strcasecmp(suffix.c_str(), "bmp") == 0) {
        return image::readBMP(filename);
    } else if (strcasecmp(suffix.c_str(), "pgm") == 0 ||
            strcasecmp(suffix.c_str(), "ppm") == 0) {
        return image::readPBM(filename);
    } else {
        return image::readPBM(filename);
    }
    return NULL;
}

} // namespace crosbot
