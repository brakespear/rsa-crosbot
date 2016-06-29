/*
 * thread.h
 *
 * Multi-threading functionality and Parallelism controls
 *
 *  Created on: 10/08/2009
 *      Author: rescue
 */

#ifndef CROSBOT_THREAD_H_
#define CROSBOT_THREAD_H_

#include <string>
#include <queue>
#include <vector>

namespace crosbot {

/**
 * Default thread closing in milliseconds
 */
#define DEFAULT_WAIT_FOR_THREAD_CLOSE		10000

/**
 * Exception thrown if problems occur with thread management
 */
class ThreadException: public std::exception {
private:
    char* msg;

public:
    ThreadException();
    ThreadException(const char * what);
    ThreadException(const std::string& what);
    ~ThreadException() throw ();
    const char* what() const throw();
};

/**
 * Internal thread data structures, not given in header to allow ease of change of implementation
 */
struct _ThreadData;

/**
 * Class based implementation of multi-threading.
 * Classes that wish to spawn a new process in a new thread should inherit from this class
 *   and implemented the crosbot::Thread::run() method.
 * A new thread is spawned by calling crosbot::Thread::start(), which detaches a new thread,
 *   calls the crosbot::Thread::run() method in the new thread, and
 *   returns control of the current thread to the entity that called crosbot::Thread::start().
 * The spawned thread will terminate only once the crosbot::Thread::run() returns.
 *
 * @warning It is the responsibility of the sub-class to ensure the thread can be terminated,
 *          and to provide interface methods to allow the main thread to terminate the spawned thread.
 *
 * Currently implemented via the PThread C++ Library.
 */
class Thread {
public:
	/**
	 * Define a class which can spawn a new thread as either
	 * joinable or detached.
	 * @param name Name of this thread used for log messages.
	 *             Default value: the empty string
	 * @param detached True if this thread should be created as detached,
	 * 	               False if the thread should be created as joinable.
	 *                 Default value: true
	 */
	Thread(std::string name = "", bool detached = true);
	virtual ~Thread();
	
	/**
	 * Start and spawn a new thread.
	 */
	void start();

	/**
	 * Called once a new thread has been spawned, passing control of the thread to the sub-class.
	 * If this method returns, the thread will be terminated.
	 */
	virtual void run()=0;
	
	/**
	 * Join the thread, if the thread cannot be joined.
	 * @throw crosbot::ThreadException if called for a thread created as detached.
	 */
	void join();

	/**
	 * Test if the thread is currently running.
	 */
	bool isAlive();

	/**
	 * Gets the name of the thread.
	 */
	std::string getName();
	
	/**
	 * Gets whether or not the thread is catching thrown exceptions.
	 */
	bool isGuarded();

	/**
	 * Sets whether or not the thread should guard against thrown exceptions.
	 */
	void setGuarded(bool guard);

	/**
	 * True if the thread is created as detached, false if the thread is created
	 * as joinable.
	 * @return True for detached threads, false otherwise.
	 */
	bool isDetached();

private:
	/**
	 * Whether or not the thread is guarded.
	 */
	bool guard;

	/**
	 * Is this thread created as detached or joinable.
	 */
	bool detached;

	struct _ThreadData *data;
};

/**
 * Internal mutex data structures, not given in header to allow ease of change of implementation
 */
struct _MutexData;

/**
 * Class-based mutex for protecting data from corruption.
 * Currently implemented as a wrapper around the PThread C++ Library mutex.
 */
class Mutex {
public:
	Mutex();
	~Mutex();
	
	/**
	 * Lock the mutex so only the locking thread can access the critical section.
	 * Blocks until the thread has the lock.
	 */
	void lock();

	/**
	 * Unlock the mutex so other threads can access the critical section.
	 */
	void unlock();

	/**
	 * Tries to get a lock on the mutex. Returns true if the lock is obtained, false if is isn't.
	 */
	bool trylock();
private:
	struct _MutexData *data;
};

/**
 * Internal semaphore data structures, not given in header to allow ease of change of implementation
 */
struct _SemaphoreData;

/**
 * Class-based Semaphore for efficient waiting in multi-threading applications.
 * Currently implemented as a wrapper around the PThread C++ Library semaphore.
 */
class Semaphore {
public:
	Semaphore(unsigned int initValue = 0);
	~Semaphore();

	void wait();
	bool tryWait();
	void notify(unsigned int incCount = 1);
private:
	struct _SemaphoreData *data;
};

/**
 * Internal read/write lock data structures, not given in header to allow ease of change of implementation
 */
struct RWLockData;

/**
 * A read/write lock.
 */
class ReadWriteLock {
public:
	ReadWriteLock();
	~ReadWriteLock();

	void lockForRead();
	void lockForWrite();
	bool tryReadLock();
	bool tryWriteLock();
	bool unlock();
private:
	struct RWLockData *data;
};

/**
 * Class-based Lock for scoped based automatic locking/unlocking of mutex'es or read/write locks.
 * The mutex is automatically locked when the Lock object is created, and automatically unlocked
 *     when the Lock object goes out of scope.
 */
class Lock {
private:
	Mutex *mutexPtr;
	ReadWriteLock *rwlockPtr;
	bool locked, write;
public:

	/**
	 * Create a Lock around a mutex and immediately attempt to lock the mutex.
	 * @param mutex Mutex to lock
	 */
	Lock(Mutex& mutex) {
		mutexPtr = &mutex;
		rwlockPtr = NULL;
		mutexPtr->lock();
		locked = true;
		write = false;
	}
	
	/**
     * Create a Lock around a read/write mutex and immediately attempt to lock the specified channel.
     * @param rwlock Read/Write mutex to lock
     * @param write If true, lock the write channel of the mutex, otherwise lock the read channel
     */
	Lock(ReadWriteLock& rwlock, bool write = false) {
		mutexPtr = NULL;
		rwlockPtr = &rwlock;
		this->write = write;
		if (write) {
			rwlockPtr->lockForWrite();
		} else {
			rwlockPtr->lockForRead();
		}
		locked = true;
	}

	bool isLocked() {
		return locked;
	}
	
	void lock() {
		if (mutexPtr != NULL) {
			mutexPtr->lock();
		} else if (rwlockPtr != NULL) {
			if (write) {
				rwlockPtr->lockForWrite();
			} else {
				rwlockPtr->lockForRead();
			}
		}
		locked = true;
	}
	
	void unlock() {
		if (mutexPtr != NULL)
			mutexPtr->unlock();
		if (rwlockPtr != NULL)
			rwlockPtr->unlock();
		locked = false;
	}

	~Lock() {
		if (locked) {
			unlock();
		}
	}
};

} // namespace crosbot

#endif /* CROSBOT_THREAD_H_ */
