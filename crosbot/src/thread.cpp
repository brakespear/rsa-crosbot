/*
 * thread.cpp
 *
 *  Created on: 10/08/2009
 *      Author: rescue
 */

#include <crosbot/thread.hpp>

#define _MULTI_THREADED
#include <pthread.h>
#include <semaphore.h>
#include <stdio.h>
#include <errno.h>

#include <ros/ros.h>
#include <crosbot/utils.hpp>

namespace crosbot {
using namespace std;

struct _ThreadData {
	string name;
	
	pthread_t thread;
};

#define DEFAULT_THREADEXCEPTMESSAGE   "Thread Exception"
class ThreadException: public std::exception {
public :
    char* msg;

    ThreadException(): exception(), msg(NULL) {}
    ThreadException(const char * what): exception() {
        if (what == NULL)
            msg = NULL;
        size_t n = strlen(what) + 1;
        msg = (char *)malloc(n);
        if (msg != NULL)
            memcpy(msg, what, n);
    }
    ThreadException(const std::string& what): exception() {
        size_t n = what.size() + 1;
        msg = (char *)malloc(n);
        if (msg != NULL)
            memcpy(msg, &what[0], n);
    }
    ~ThreadException() throw () {
        if (msg != NULL)
            free(msg);
    }

    const char* what() const throw() {
        if (msg == NULL)
            return DEFAULT_THREADEXCEPTMESSAGE;
        return msg;
    }
};

Thread::Thread(std::string name, bool detached) {
	data = new _ThreadData;
	data->name = name;
	
	data->thread = 0;
	guard = true;
	this->detached = detached;
}

Thread::~Thread() {
	delete data;
}

string Thread_getError(int err) {
	switch (err) {
	case EINVAL:
		return "EINVAL"; break;
	case EBUSY:
		return "EBUSY"; break;
	case EAGAIN:
		return "EAGAIN"; break;
	case EDEADLK:
		return "EDEADLK"; break;
	case EPERM:
		return "EPERM"; break;
	default:
		return "";
	}
}

void *_Thread_startRoutine(void *threadPtr) {
	Thread *thread = (Thread *)threadPtr;
	string name = thread->getName();
	if (name == "") {
		pthread_t self = pthread_self();

		char uintStr[64];
		sprintf(uintStr, "0x%lx", self);
		name = "Thread_";
		name.append(uintStr);
	}
	ROS_DEBUG("Starting thread %s.\n", name.c_str());
	if (!thread->isGuarded()) {
		thread->run();
	} else {
		try {
			thread->run();
		} catch (std::exception& exception) {
            ROS_ERROR("Thread %s died after exception %s was thrown.\n", name.c_str(), exception.what());
		} catch (std::exception* exception) {
            ROS_ERROR("Thread %s died after exception %s was thrown.\n", name.c_str(), exception->what());
		} catch (long l) {
            ROS_ERROR("Thread %s died after long %ld was thrown.\n", name.c_str(), l);
		} catch (int i) {
            ROS_ERROR("Thread %s died after int %d was thrown.\n", name.c_str(), i);
		} catch (double d) {
            ROS_ERROR("Thread %s died after double %lf was thrown.\n", name.c_str(), d);
		} catch (float f) {
            ROS_ERROR("Thread %s died after float %f was thrown.\n", name.c_str(), f);
		} catch (...) {
            ROS_ERROR("Thread %s died after an unknown value was thrown.\n", name.c_str());
		}
	}
	return NULL;
}

void Thread::start(){
	
	pthread_attr_t attr;
	int err = pthread_attr_init(&attr);
	if (err != 0) {
			ROS_ERROR("Unable to initialise attributes for thread %s: %d.\n", data->name.c_str(), err);
	}
	
	// Set the detached/joinable state
	int detachState = detached ? PTHREAD_CREATE_DETACHED : PTHREAD_CREATE_JOINABLE;
	err = pthread_attr_setdetachstate(&attr, detachState);
	if (err != 0) {
		string detachStr = detached ? "detached" : "joinable";
        ROS_ERROR("Unable to set thread %s to %s: %d.\n", data->name.c_str(), detachStr.c_str(), err);
	}
	
	err = pthread_create(&(data->thread), NULL, _Thread_startRoutine, this);
	if (err != 0) {
        ROS_ERROR("Thread %s could not be started: %d.\n", data->name.c_str(), err);
	}
}

void Thread::join() {
	// Check the thread is joinable
	if (detached) {
		char msg[1000];
		sprintf(msg, "Attempted to join non-joinable thread: %s", getName().c_str());
        ROS_ERROR("%s\n", msg);
		throw ThreadException(string(msg));
	}

	// Join
	pthread_join(data->thread, NULL);
}

bool Thread::isAlive() {
	int policy;
	sched_param param;
	if (data->thread == 0)
		return false;
	int err = pthread_getschedparam(data->thread, &policy, &param);
	if (err == 0) {
		return true;
	}
	return false;
}

string Thread::getName() {
	return data->name;
}


bool Thread::isGuarded() {
	return guard;
}

void Thread::setGuarded(bool guard) {
	this->guard = guard;
}

bool Thread::isDetached() {
	return detached;
}

struct _MutexData {
	pthread_mutex_t mutex;
};

Mutex::Mutex() {
	data = new struct _MutexData;
	int err = pthread_mutex_init(&(data->mutex), NULL);
	if (err != 0) {
		string eStr = Thread_getError(err);
        if (eStr != "") {
            ROS_ERROR("Could not initialise mutex. (%s)\n", eStr.c_str());
        } else {
            ROS_ERROR("Could not initialise mutex. (%d)\n", err);
		}
	}
}

Mutex::~Mutex() {
	pthread_mutex_destroy(&(data->mutex));
	delete data;
}

void Mutex::lock() {
	int err = pthread_mutex_lock(&(data->mutex));
	if (err != 0) {
		string eStr = Thread_getError(err);
        if (eStr != "") {
            ROS_ERROR("Problem occurred locking mutex. (%s)\n", eStr.c_str());
        } else {
            ROS_ERROR("Problem occurred locking mutex. (%d)\n", err);
        }
	}
}

void Mutex::unlock() {
	int err = pthread_mutex_unlock(&(data->mutex));
	if (err != 0) {
		string eStr = Thread_getError(err);
        if (eStr != "") {
            ROS_ERROR("Problem occurred unlocking mutex. (%s)\n", eStr.c_str());
        } else {
            ROS_ERROR("Problem occurred unlocking mutex. (%d)\n", err);
        }
	}
}

bool Mutex::trylock() {
	int err = pthread_mutex_trylock(&(data->mutex));
	if (err == 0)
		return true;
	else if (err != EBUSY) {
		string eStr = Thread_getError(err);
        if (eStr != "") {
            ROS_ERROR("Problem occurred trying to lock mutex. (%s)\n", eStr.c_str());
        } else {
            ROS_ERROR("Problem occurred trying to lock mutex. (%d)\n", err);
        }
	}
	return false;
}

struct SemaphoreData {
	sem_t semaphore;
};

Semaphore::Semaphore(unsigned int initValue) {
	data = (struct SemaphoreData *)malloc(sizeof(struct SemaphoreData));
	sem_init(&(data->semaphore), 0, initValue);
}

Semaphore::~Semaphore() {
	sem_destroy(&(data->semaphore));
	free(data);
}

void Semaphore::wait() {
	sem_wait(&(data->semaphore));
}

bool Semaphore::tryWait() {
	if (sem_trywait(&(data->semaphore))) {
		return true;
	}
	return false;
}

void Semaphore::notify(unsigned int incCount) {
	while (incCount-- > 0) {
		sem_post(&(data->semaphore));
	}
}

struct RWLockData {
	pthread_rwlock_t rwlock;
};
ReadWriteLock::ReadWriteLock() {
	data = new RWLockData;
	pthread_rwlock_init(&data->rwlock, NULL);
}

ReadWriteLock::~ReadWriteLock() {
	pthread_rwlock_destroy(&data->rwlock);
	delete data;
}

void ReadWriteLock::lockForRead() {
	int err = pthread_rwlock_rdlock(&data->rwlock);
	if (err != 0) {
		string eStr = Thread_getError(err);
        ROS_ERROR("Problem occurred locking for read. (%s)\n", eStr.c_str());
	}
}

void ReadWriteLock::lockForWrite() {
	int err = pthread_rwlock_wrlock(&data->rwlock);
	if (err != 0) {
		string eStr = Thread_getError(err);
        ROS_ERROR("Problem occurred locking for write. (%s)\n", eStr.c_str());
	}
}

bool ReadWriteLock::tryReadLock() {
	int err = pthread_rwlock_tryrdlock(&data->rwlock);
	if (err != 0)
		return true;
	else if (err != EBUSY) {
		string eStr = Thread_getError(err);
        ROS_ERROR("Problem occurred trying to lock for read. (%s)\n", eStr.c_str());
	}
	return false;
}

bool ReadWriteLock::tryWriteLock() {
	int err = pthread_rwlock_trywrlock(&data->rwlock);
	if (err != 0)
		return true;
	else if (err != EBUSY) {
		string eStr = Thread_getError(err);
        ROS_ERROR("Problem occurred trying to lock for write. (%s)\n", eStr.c_str());
	}
	return false;
}

bool ReadWriteLock::unlock() {
	int err = pthread_rwlock_unlock(&data->rwlock);
	if (err == 0)
		return true;
	else if (err != EBUSY) {
		string eStr = Thread_getError(err);
        ROS_ERROR("Problem occurred unlocking. (%s)\n", eStr.c_str());
	}
	return false;
}

void JobDispatcher::WorkerThread::run() {
	while (operating) {
		dispatch->semaphore.wait();

		if (!operating)
			break;
		if (dispatch->paused)
			continue;

		Job *myJob = NULL;
		{{
			Lock lock(dispatch->jobMutex);
			if (dispatch->jobs.size() == 0) {
				lock.unlock();
				continue;
			}

			myJob = dispatch->jobs.front();
			dispatch->jobs.pop();

			if (myJob != NULL)
				dispatch->activeJobs.push_back(myJob);
		}}

		if (!operating)
			break;

		if (myJob != NULL) {
			try {
				myJob->run();
			} catch (std::exception& e) {
				ERROR("Problem occurred running job. (\"%s\")\n", e.what());
			} catch (int i) {
				ERROR("Problem occurred running job. (%d)\n", i);
			} catch (...) {
				ERROR("Problem occurred running job. Type unknown.\n");
			}

			{{
				Lock lock(dispatch->jobMutex);
				for (size_t i = 0; i < dispatch->activeJobs.size(); i++) {
					if (dispatch->activeJobs[i] == myJob) {
						dispatch->activeJobs.erase(dispatch->activeJobs.begin()+i);
						break;
					}
				}
			}}
		}

		dispatch->semaphore.notify();
	}

	// Remove thread from dispatcher.
	{{
		Lock lock(dispatch->jobMutex);

		for (size_t i = 0; i < dispatch->dyingWorkers.size(); i++) {
			if (dispatch->dyingWorkers[i] == this) {
				dispatch->dyingWorkers.erase(dispatch->dyingWorkers.begin()+i);
				break;
			}
		}
	}}

	delete this;
}

JobDispatcher::JobDispatcher(int numThreads, std::string name) :
	name(name), paused(false)
{
	setThreads(numThreads);
}

JobDispatcher::JobDispatcher(std::string name) :
	name(name), paused(false)
{
	setThreads(1);
}

JobDispatcher::~JobDispatcher() {
	setThreads(0);

	for (int i = 0; dyingWorkers.size() > 0 && i < DEFAULT_WAIT_FOR_THREAD_CLOSE; i += 10) {
		usleep(10000);
	}

	if (dyingWorkers.size() > 0) {
		ERROR("One or more worker threads is refusing to finish.\n");
	}
}

void JobDispatcher::setThreads(unsigned int numThreads) {
	Lock lock(jobMutex);
	unsigned int notifications = 0;

	if (workers.size() < numThreads) {
		notifications = numThreads;
		while (workers.size() < numThreads) {
			if (name == "") {
				workers.push_back(new WorkerThread(this));
			} else {
				char cStr[name.size() + 128];
				sprintf(cStr, "%s-%Zd", name.c_str(), workers.size());
				workers.push_back(new WorkerThread(this, std::string(cStr)));
			}
		}
	} else if (workers.size() > numThreads) {
		notifications = workers.size();
		while (workers.size() > numThreads) {
			size_t idx = workers.size()-1;
			WorkerThread * wt = workers[idx];
			workers.erase(workers.begin()+idx);
			wt->operating = false;
			dyingWorkers.push_back(wt);
		}
	}

	if (notifications > 0) {
		semaphore.notify(notifications);
	}
}

} // namespace crosbot
