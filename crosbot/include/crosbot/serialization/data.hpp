/*
 * data.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_SERIALIZATION_DATA_HPP_
#define CROSBOT_SERIALIZATION_DATA_HPP_

#include <crosbot/data.hpp>
#include <crosbot/serialization.hpp>

namespace crosbot {
namespace serialization {

CASROS_SIMPLE_SERIALIZER(UUID);
CASROS_SIMPLE_SERIALIZER(Time);
CASROS_SIMPLE_SERIALIZER(Duration);
CASROS_SIMPLE_SERIALIZER(Colour);
CASROS_SIMPLE_SERIALIZER(ColouredPoint);

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
                sizeof(unsigned int) +  // encoding
                sizeof(unsigned int) +  // step
                sizeof(uint64_t) +      // dataLength
                image.dataLength;
    }
};

} // namespace serialization
} // namespace crosbot


#endif /* CROSBOT_SERIALIZATION_DATA_HPP_ */
