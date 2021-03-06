/*
 * handle.h
 *
 *  Because we need reference counted pointers that are thread safe.
 *
 *  Created on: 10/06/2009
 *      Author: rescue
 */

#ifndef CROSBOT_HANDLE_H_
#define CROSBOT_HANDLE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>

namespace crosbot {

/**
 * Exception thrown by crosbot::Handle<T> if attempting to perform invalid
 *    operations on a NULL pointer.
 */
class NullHandleException : public std::exception {
private:
    char* msg;

public:
    NullHandleException();
    NullHandleException(const char * what);
    NullHandleException(const std::string& what);
    ~NullHandleException() throw ();

    const char* what() const throw();
};

#define     REF_COUNT_TYPE  long int

/**
 * Internal data structure defining the data of the handled objects
 */
struct _HandledObjectData;

/**
 * Superclass for Crosbot implementation of thread-safe shared pointers.
 * Any class to be used as a thread-safe shared pointer must inherit this class.
 *
 * @warning All methods provided by this class should be be directly called, but are only
 *    to be used by crosbot::Handle
 */
class HandledObject {
private:
    _HandledObjectData *_handledData;
public:
	HandledObject();
	HandledObject(const HandledObject&);
	virtual ~HandledObject();

    /**
     * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
     */
	void __incRef();

    /**
     * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
     */
	void __incRef() const;

    /**
     * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
     */
	void __decRef();

    /**
     * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
     */
	void __decRef() const;

    /**
     * @warning Do not call directly. Used by crosbot::Handle in managing the shared objects
     */
	REF_COUNT_TYPE __getRef() const;
};

} // namespace crosbot

// Include correct handle version here
#if __cplusplus < 201103L
#include <crosbot/handle/handle_cpp03.hpp>
#else
#include <crosbot/handle/handle_cpp11.hpp>
#endif

namespace crosbot {

inline bool operator==(const HandledObject& lhs, const HandledObject& rhs)
{
    return &(lhs) == &(rhs);
}

inline bool operator==(const HandledObject& lhs, void *rhs)
{
    return &(lhs) == rhs;
}

inline bool operator==(void *lhs, const HandledObject& rhs)
{
    return lhs == &(rhs);
}

/**
 * A handle (pointer) to an object inheriting from crosbot::HandledObject.
 * Ensure thread-safe management of the shared pointer using the interface methods of crosbot::HandledObject.
 */
template<typename T>
class Handle {
private:
    T* _ptr;

    void throwNullHandleException() const {
        throw NullHandleException();
    }

public:
	Handle(const T* p = NULL)
	{
		this->_ptr = const_cast<T*>(p);

		if(this->_ptr)
		{
			this->_ptr->__incRef();
		}
	}

	Handle(const Handle& r)
    {
        this->_ptr = const_cast<T*>(r.get());

        if(this->_ptr)
        {
            this->_ptr->__incRef();
        }
    }

	template<typename Y>
    Handle(const Handle<Y>& r)
    {
        this->_ptr = const_cast<Y*>(r.get());

        if(this->_ptr)
        {
            this->_ptr->__incRef();
        }
    }

    ~Handle()
    {
        if(this->_ptr)
        {
            this->_ptr->__decRef();
        }
    }

    inline T* get()
    {
        return _ptr;
    }

    inline const T* get() const
    {
        return _ptr;
    }

    inline T* operator->()
    {
        if(!_ptr)
        {
            throwNullHandleException();
        }

        return _ptr;
    }

    inline const T* operator->() const
    {
        if(!_ptr)
        {
            throwNullHandleException();
        }

        return _ptr;
    }

    inline T& operator*()
    {
        if(!_ptr)
        {
            throwNullHandleException();
        }

        return *_ptr;
    }

    inline const T& operator*() const
    {
        if(!_ptr)
        {
            throwNullHandleException();
        }

        return *_ptr;
    }

    Handle& operator=(const T* p)
	{
    	if(this->_ptr != p)
		{
			if(p)
			{
			   const_cast<T*>(p)->__incRef();
			}

			T* ptr = this->_ptr;
			this->_ptr = const_cast<T*>(p);

			if(ptr)
			{
				ptr->__decRef();
			}
		}
	   return *this;
	}

	template<typename Y>
	Handle& operator=(const Handle<Y>& r)
	{
		if(this->_ptr != r.get())
		{
			if(r.get())
			{
			   const_cast<Y*>(r.get())->__incRef();
			}

			T* ptr = this->_ptr;
			this->_ptr = const_cast<Y*>(r.get());

			if(ptr)
			{
				ptr->__decRef();
			}
		}
		return *this;
	}

	Handle& operator=(const Handle& r)
	{
		if(this->_ptr != r._ptr)
		{
			if(r._ptr)
			{
			   const_cast<T*>(r._ptr)->__incRef();
			}

			T* ptr = this->_ptr;
			this->_ptr = const_cast<T*>(r._ptr);

			if(ptr)
			{
				ptr->__decRef();
			}
		}
		return *this;
	}

	template<class Y>
    static Handle dynamicCast(const Handle<Y>& r)
    {
        return Handle(dynamic_cast<const T*>(r.get()));
    }

    template<class Y>
    static Handle dynamicCast(const Y* p)
    {
        return Handle(dynamic_cast<const T*>(p));
    }

};

template<typename T, typename U>
inline bool operator==(const Handle<T>& lhs, const Handle<U>& rhs)
{
    const T* l = lhs.get();
    const U* r = rhs.get();
    if(l && r)
    {
        return *l == *r;
    }
    return !l && !r;
}

template<typename T, typename U>
inline bool operator!=(const Handle<T>& lhs, const Handle<U>& rhs)
{
    return !operator==(lhs, rhs);
}

template<typename T, typename U>
inline bool operator==(const Handle<T>& lhs, const U *rhs)
{
    const T* l = lhs.get();
    if(l && rhs)
    {
        return *l == *rhs;
    }
    return !l && !rhs;
}

template<typename T, typename U>
inline bool operator!=(const Handle<T>& lhs, const U *rhs)
{
    return !operator==(lhs, rhs);
}

template<typename T>
inline bool operator==(const Handle<T>& lhs, const void *rhs)
{
    const T* l = lhs.get();
    return l == rhs;
}

template<typename T>
inline bool operator!=(const Handle<T>& lhs, const void *rhs)
{
    return !operator==(lhs, rhs);
}

template<typename T, typename U>
inline bool operator<(const Handle<T>& lhs, const Handle<U>& rhs)
{
    const T* l = lhs.get();
    const U* r = rhs.get();
    if(l && r)
    {
        return *l < *r;
    }
    return !l && r;
}

template<typename T, typename U>
inline bool operator<=(const Handle<T>& lhs, const Handle<U>& rhs)
{
    return lhs < rhs || lhs == rhs;
}

template<typename T, typename U>
inline bool operator>(const Handle<T>& lhs, const Handle<U>& rhs)
{
    return !(lhs < rhs || lhs == rhs);
}

template<typename T, typename U>
inline bool operator>=(const Handle<T>& lhs, const Handle<U>& rhs)
{
    return !(lhs < rhs);
}

} // namespace crosbot

#endif /* CROSBOT_HANDLE_H_ */
