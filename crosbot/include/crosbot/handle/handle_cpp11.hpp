/*
 * handle_cpp11.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_HANDLE_CPP11_HPP_
#define CROSBOT_HANDLE_CPP11_HPP_

#if __cplusplus >= 201103L

#include <crosbot/handle.hpp>

#include <atomic>
#include <string>

namespace crosbot {

struct _HandledObjectData {
    std::atomic< REF_COUNT_TYPE > _refCount;
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

inline void HandledObject::__incRef() {
    ++_handledData->_refCount;
}

inline void HandledObject::__incRef() const {
    ++*((std::atomic< int > *)&_handledData->_refCount);
}

inline void HandledObject::__decRef() {
    --_handledData->_refCount;
    if (_handledData->_refCount < 0) {
        fprintf(stderr, "How did the ref count go below zero?\n");
    }
    if (_handledData->_refCount == 0) {
        delete this;
    }
}

inline void HandledObject::__decRef() const {
    --*((std::atomic< int > *)&_handledData->_refCount);
    if (_handledData->_refCount < 0) {
        fprintf(stderr, "How did the ref count go below zero?\n");
    }
    if (_handledData->_refCount == 0) {
        delete this;
    }
}

inline REF_COUNT_TYPE HandledObject::__getRef() const {
    return _handledData->_refCount.load();
}

} // namespace crosbot

#endif /* __cplusplus >= 201103L */

#endif /* CROSBOT_HANDLE_CPP11_HPP_ */
