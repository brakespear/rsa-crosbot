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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <exception>
#include <crosbot/thread.hpp>

namespace crosbot {

/**
 * A reference counted self managed object.
 */
class HandledObject {
private:
	int _refCount;
	Mutex _refMutex;
public:
	HandledObject() {
		_refCount = 0;
	}
	
	virtual ~HandledObject() {
	};
	
	void __incRef() {
		Lock lock(_refMutex);
		++_refCount;
	}

	void __decRef() {
		Lock lock(_refMutex);
		--_refCount;
		if (_refCount < 0) {
			fprintf(stderr, "How did the ref count go below zero?\n");
		}
		if (_refCount == 0) {
			lock.unlock();
			delete this;
		}
	}

	int __getRef() {
		return _refCount;
	}
};

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

#define DEFAULT_NULLHANDLEMESSAGE   "Null Handle Exception."
class NullHandleException: public std::exception {
public :
    char* msg;

	NullHandleException(): exception(), msg(NULL) {}
	NullHandleException(const char * what): exception() {
	    if (what == NULL)
	        msg = NULL;
	    size_t n = strlen(what) + 1;
	    msg = (char *)malloc(n);
	    if (msg != NULL)
	        memcpy(msg, what, n);
	}
	NullHandleException(const std::string& what): exception() {
        size_t n = what.size() + 1;
        msg = (char *)malloc(n);
        if (msg != NULL)
            memcpy(msg, &what[0], n);
	}
	~NullHandleException() throw () {
	    if (msg != NULL)
	        free(msg);
	}

	const char* what() const throw() {
	    if (msg == NULL)
	        return DEFAULT_NULLHANDLEMESSAGE;
	    return msg;
	}
};

/**
 * A handle(pointer) to a ManagedObject.
 */
template<typename T>
class Handle
{
private:
    T* _ptr;
    void throwNullHandleException() const;
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

template<typename T> inline void
Handle<T>::throwNullHandleException() const
{
    throw NullHandleException();
}

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
