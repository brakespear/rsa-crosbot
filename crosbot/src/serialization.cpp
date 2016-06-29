/*
 * serialization.cpp
 *
 *  Created on: 29/02/2012
 *      Author: rescue
 */

#include <crosbot/serialization.hpp>
#include <string.h>

namespace crosbot {

IOException::IOException() :
    description("IO Exception")
{}

IOException::IOException(const char *description) :
    description(description)
{}

IOException::IOException(const std::string& description) :
    description(description)
{}

IOException::~IOException() throw()
{}

const char* IOException::what() const throw() {
    return description.c_str();
}

IOException IOException::UnableToOpenFile(std::string filename) {
    std::string msg = "Unable to open file " + filename + ".";
    return IOException(msg);
}

IOException IOException::ErrorWritingFile(std::string filename) {
    std::string msg = "Error when writing to file " + filename + ".";
    return IOException(msg);
}

IOException IOException::ErrorReadingFile(std::string filename) {
    std::string msg = "Error when reading from file " + filename + ".";
    return IOException(msg);
}

IOException IOException::OutOfMemory() {
    return IOException("Out of memory.");
}

} // namespace crosbot

namespace crosbot {

namespace serialization {

size_t BufferInputStream::read(void *data, size_t bytes) throw (IOException) {
	size_t remaining = buffer.bufferSize - posn,
			cpy = (remaining<bytes?remaining:bytes);
	if (cpy > 0)
		memcpy(data, ((char*)buffer.data)+posn, cpy);

	return cpy;
}

void BufferInputStream::seek(size_t pt) throw (IOException) {
	posn = pt;
	if (posn > buffer.bufferSize)
		posn = buffer.bufferSize;
}

size_t BufferOutputStream::write(const void *data, size_t bytes) throw (IOException) {
	size_t remaining = buffer.bufferSize - posn,
			cpy = (remaining<bytes?remaining:bytes);
	if (cpy > 0)
		memcpy(((char*)buffer.data)+posn, data, cpy);

	return cpy;
}

void BufferOutputStream::seek(size_t pt) throw (IOException) {
	posn = pt;
	if (posn > buffer.bufferSize)
		posn = buffer.bufferSize;
}

} // namespace serialization

} // namespace crosbot
