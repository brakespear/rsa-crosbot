/*
 * exceptions.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot/exceptions.hpp>

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
