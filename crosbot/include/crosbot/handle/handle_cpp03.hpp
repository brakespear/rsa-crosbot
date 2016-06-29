/*
 * handle_cpp03.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef HANDLE_CPP03_HPP_
#define HANDLE_CPP03_HPP_

#if __cplusplus < 201103L

#include <crosbot/handle.hpp>
#include <crosbot/thread.hpp>
#include <string.h>

namespace crosbot {

struct _HandledObjectData {
    REF_COUNT_TYPE _refCount;
    Mutex _refMutex;
};

inline HandledObject::HandledObject() {
    _handledData = new _HandledObjectData;
    _handledData->_refCount = 0;
}

inline HandledObject::HandledObject(const HandledObject&) {
    _handledData = new _HandledObjectData;
    _handledData->_refCount = 0;
}

inline HandledObject::~HandledObject() {
};

/**
 * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
 */
inline void HandledObject::__incRef() {
    Lock lock(_handledData->_refMutex);
    ++_handledData->_refCount;
}

/**
 * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
 */
inline void HandledObject::__incRef() const {
    Lock lock(*((Mutex *)&_handledData->_refMutex));
    ++*((int *)&_handledData->_refCount);
}

/**
 * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
 */
inline void HandledObject::__decRef() {
    Lock lock(_handledData->_refMutex);
    --_handledData->_refCount;
    if (_handledData->_refCount < 0) {
        fprintf(stderr, "How did the ref count go below zero?\n");
    }
    if (_handledData->_refCount == 0) {
        lock.unlock();
        delete this;
    }
}

/**
 * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
 */
inline void HandledObject::__decRef() const {
    Lock lock(*((Mutex *)&_handledData->_refMutex));
    --*((int *)&_handledData->_refCount);
    if (_handledData->_refCount < 0) {
        fprintf(stderr, "How did the ref count go below zero?\n");
    }
    if (_handledData->_refCount == 0) {
        lock.unlock();
        delete this;
    }
}

/**
 * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
 */
inline REF_COUNT_TYPE HandledObject::__getRef() const {
    return _handledData->_refCount;
}

} // namespace crosbot

#endif /* __cplusplus < 201103L */

#endif /* HANDLE_CPP03_HPP_ */
