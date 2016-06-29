/*
 * handle.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */


#include <ros/ros.h>

#include <crosbot/handle.hpp>

#define DEFAULT_NULL_HANDLE_EXCEPTION_MESSAGE "Null Handle Exception."

namespace crosbot {

NullHandleException::NullHandleException() :
    exception(),
    msg(NULL)
{}

NullHandleException::NullHandleException(const char * what): exception() {
    if (what == NULL) {
        msg = NULL;
    }
    size_t n = strlen(what) + 1;
    msg = (char *)malloc(n);
    if (msg != NULL) {
        memcpy(msg, what, n);
    }
}

NullHandleException::NullHandleException(const std::string& what): exception() {
    size_t n = what.size() + 1;
    msg = (char *)malloc(n);
    if (msg != NULL) {
        memcpy(msg, &what[0], n);
    }
}

NullHandleException::~NullHandleException() throw () {
    if (msg != NULL) {
        free(msg);
    }
}

const char* NullHandleException::what() const throw() {
    if (msg == NULL) {
        return DEFAULT_NULL_HANDLE_EXCEPTION_MESSAGE;
    }
    return msg;
}

} // namespace crosbot
