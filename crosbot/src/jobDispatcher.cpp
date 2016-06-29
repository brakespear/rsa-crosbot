/*
 * jobDispatcher.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */


#include <ros/ros.h>

#include <crosbot/jobDispatcher.hpp>
#include <crosbot/utils.hpp>

namespace crosbot {

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
