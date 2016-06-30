/*
 * exceptions.hpp
 *
 * Common exceptions for crosbot
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXCEPTIONS_HPP_
#define CROSBOT_EXCEPTIONS_HPP_

#include <exception>
#include <string>

namespace crosbot {

/**
 * Exception thrown by crosbot::Serializer if problem are encountered
 *    when writing crosbot data structures to file or
 *    loading crosbot data structures from plain-text files.
 */
class IOException : public std::exception {
private:
    std::string description;

public:
    IOException();
    IOException(const char *description);
    IOException(const std::string& description);
    ~IOException() throw();

    const char* what() const throw();

    static IOException UnableToOpenFile(std::string filename);
    static IOException ErrorWritingFile(std::string filename);
    static IOException ErrorReadingFile(std::string filename);
    static IOException OutOfMemory();
};

} // namespace crosbot


#endif /* CROSBOT_EXCEPTIONS_HPP_ */
