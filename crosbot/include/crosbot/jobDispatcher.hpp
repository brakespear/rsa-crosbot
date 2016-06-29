/*
 * job_dispatcher.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_JOB_DISPATCHER_HPP_
#define CROSBOT_JOB_DISPATCHER_HPP_

#include <crosbot/thread.hpp>

namespace crosbot {

/**
 * Generic interface description of jobs to be processed in parallel by
 *    crosbot::JobDispatcher
 */
class Job {
public:
    virtual ~Job(){}
    virtual void run()=0;
    virtual void complete() {}
};

/**
 * Generic implementation of a job server to process multiple similar jobs in parallel.
 */
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

#endif /* CROSBOT_JOB_DISPATCHER_HPP_ */
