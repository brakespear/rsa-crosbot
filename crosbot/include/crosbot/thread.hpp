/*
 * thread.h
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

#define DEFAULT_WAIT_FOR_THREAD_CLOSE		10000 // Milliseconds

struct _ThreadData;
/**
 * A thread for processing.
 */
class Thread {
public:
	/**
	 * Create a new thread with an optional name that can be created as either
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
	 * Starts the thread running.
	 */
	void start();

	/**
	 * The thread.
	 */
	virtual void run()=0;
	
	/**
	 * Join the thread, if the thread is joinable. Throws an exception if called
	 * for a thread created as detached.
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

struct _MutexData;
/**
 * A mutex for protecting data from corruption.
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

struct SemaphoreData;
/**
 * A semaphore.
 */
class Semaphore {
public:
	Semaphore(unsigned int initValue = 0);
	~Semaphore();

	void wait();
	bool tryWait();
	void notify(unsigned int incCount = 1);
private:
	struct SemaphoreData *data;
};

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
 * A class for locking a mutex/rwlock and automatically unlocking when a function returns.
 */
class Lock {
private:
	Mutex *mutexPtr;
	ReadWriteLock *rwlockPtr;
	bool locked, write;
public:
	Lock(Mutex& mutex) {
		mutexPtr = &mutex;
		rwlockPtr = NULL;
		mutexPtr->lock();
		locked = true;
	}
	
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
		if (locked)
			unlock();
	}
};

class Job {
public:
	virtual ~Job(){}
	virtual void run()=0;
	virtual void complete() {}
};

class JobDispatcher {
protected:
	class WorkerThread : public Thread {
	protected:
		JobDispatcher *dispatch;
		bool operating;
	public:
		WorkerThread(JobDispatcher *dispatch, std::string name = "") :
			Thread(name),dispatch(dispatch), operating(true)
		{
			setGuarded(false);
			this->start();
		}
		void run();

		friend class JobDispatcher;
	};

	std::string name;
	bool paused;
	Semaphore semaphore;
	std::vector<WorkerThread *> workers;
	std::vector<WorkerThread *> dyingWorkers;

	Mutex jobMutex;
	std::queue<Job *> jobs;
	std::vector<Job *> activeJobs;

public:

	/**
	 * Creates JobDispatcher
	 */
	JobDispatcher(int numThreads = 1, std::string name = "");

	/**
	 * Creates JobDispatcher
	 */
	JobDispatcher(std::string name);

	/**
	 * Destructor. Will try to stop all threads.
	 */
	~JobDispatcher();

	/**
	 * Changes the number of threads used to run the jobs.
	 */
	void setThreads(unsigned int numThreads);
	size_t threads() {
		return workers.size();
	}

	/**
	 * Pauses job processing. Any jobs being run when this is called will be allowed to finish.
	 */
	void pause() {
		paused = true;
	}

	/**
	 * Resumes job processing.
	 */
	void resume() {
		paused = false;
		Lock lock(jobMutex);
		if (jobs.size() > 0)
			semaphore.notify(jobs.size());
	}

	/**
	 * Adds a job to the processing queue.
	 */
	void addJob(Job *job) {
		Lock lock(jobMutex);
		jobs.push(job);
		semaphore.notify();
	}

	/**
	 * Gets the number of jobs currently in the queue.
	 */
	size_t jobCount() {
		Lock lock(jobMutex);
		return jobs.size() + activeJobs.size();
	}

	friend class WorkerThread;
};

} // namespace crosbot

#endif /* CROSBOT_THREAD_H_ */
