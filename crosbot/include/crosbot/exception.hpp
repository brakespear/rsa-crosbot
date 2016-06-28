/*
 * exception.hpp
 *
 *  Custom exceptions has used in the Crosbot libraries and software packages
 *
 *  Created on: 28/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_EXCEPTION_HPP_
#define CROSBOT_EXCEPTION_HPP_

#include <cstdio>

class NullHandleException: public std::exception {
public :
    char* msg;

    NullHandleException(): exception(), msg(NULL) {}
    NullHandleException(const char * what): exception() {
        if (what == NULL) {
            msg = NULL;
        }
        size_t n = strlen(what) + 1;
        msg = (char *)malloc(n);
        if (msg != NULL) {
            memcpy(msg, what, n);
        }
    }
    NullHandleException(const std::string& what): exception() {
        size_t n = what.size() + 1;
        msg = (char *)malloc(n);
        if (msg != NULL) {
            memcpy(msg, &what[0], n);
        }
    }
    ~NullHandleException() throw () {
        if (msg != NULL) {
            free(msg);
        }
    }

    const char* what() const throw() {
        if (msg == NULL) {
            return "Null Handle Exception.";
        }
        return msg;
    }
};


#endif /* CROSBOT_EXCEPTION_HPP_ */
