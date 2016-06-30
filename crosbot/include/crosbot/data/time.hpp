/*
 * time.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_TIME_HPP_
#define CROSBOT_DATA_TIME_HPP_

#include <cstdint>

#include <crosbot/data/defines.hpp>

// TODO: Make duration and time immutable

namespace crosbot {

/**
 * Duration class based on ros::Duration, with additional arithmetic operations
 */
struct Duration {
    int32_t sec;
    int32_t nsec;

    Duration() : sec(0), nsec(0) {}
    Duration(Duration& other) : sec(other.sec), nsec(other.nsec) {}
    Duration(const Duration& other) : sec(other.sec), nsec(other.nsec) {}
    Duration(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}
    Duration(int64_t nsec) : sec(nsec/NSECS_PER_SEC), nsec(nsec%NSECS_PER_SEC) {}
    Duration(double sec) : sec(sec), nsec((sec-this->sec)*NSECS_PER_SEC) {}

    inline int64_t toNSec() const { return sec * NSECS_PER_SEC + nsec; }
    inline double toSec() const { return sec + ((double)nsec)/NSECS_PER_SEC; }

    inline bool isZero() const { return sec == 0 && nsec == 0; }

    inline bool operator==(const Duration& d) const { return sec == d.sec && nsec == d.nsec; }

    inline bool operator!=(const Duration& d) const { return sec != d.sec || nsec != d.nsec; }

    inline bool operator<(const Duration& d) const {
        if (sec == d.sec) {
            return nsec < d.nsec;
        } else {
            return sec < d.sec;
        }
    }

    inline bool operator<=(const Duration& d) const {
        if (sec == d.sec) {
            return nsec <= d.nsec;
        } else {
            return sec < d.sec;
        }
    }

    inline bool operator>(const Duration& d) const {
        if (sec == d.sec) {
            return nsec > d.nsec;
        } else {
            return sec > d.sec;
        }
    }

    inline bool operator>=(const Duration& d) const {
        if (sec == d.sec) {
            return nsec >= d.nsec;
        } else {
            return sec > d.sec;
        }
    }

    inline Duration operator+(const Duration& d) const {
        int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
        return Duration(nsecs);
    }

    inline Duration operator-(const Duration& d) const {
        int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
        return Duration(nsecs);
    }

    inline Duration operator*(const double scale) const {
        int64_t nsecs = sec * NSECS_PER_SEC + nsec;
        nsecs *= scale;
        return Duration(nsecs);
    }

    inline Duration operator/(const double scale) const {
        int64_t nsecs = sec * NSECS_PER_SEC + nsec;
        nsecs /= scale;
        return Duration(nsecs);
    }

    inline Duration& operator+=(const Duration& d) {
        int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

    inline Duration& operator-=(const Duration& d) {
        int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

    inline Duration& operator*=(const double scale) {
        int64_t nsecs = sec * NSECS_PER_SEC + nsec;
        nsecs *= scale;
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

    inline Duration& operator/=(const double scale) {
        int64_t nsecs = sec * NSECS_PER_SEC + nsec;
        nsecs /= scale;
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

#ifdef ROS_VERSION

    Duration(const ros::Duration& d) : sec(d.sec), nsec(d.nsec) {}

    inline Duration& operator=(const ros::Duration& d) {
        sec = d.sec; nsec = d.nsec;
        return *this;
    }

    inline ros::Duration toROS() const {
        return ros::Duration(sec, nsec);
    }

#endif
};

/**
 * Time class based on ros::Time, with additional arithmetic operations
 */
struct Time {
public:
    int32_t sec;
    int32_t nsec;

    Time() : sec(0), nsec(0) {}
    Time(Time& other) : sec(other.sec), nsec(other.nsec) {}
    Time(const Time& other) : sec(other.sec), nsec(other.nsec) {}
    Time(int32_t sec, int32_t nsec) : sec(sec), nsec(nsec) {}
    Time(int64_t nsec) : sec(nsec/NSECS_PER_SEC), nsec(nsec%NSECS_PER_SEC) {}
    Time(double sec) : sec(sec), nsec((sec-this->sec)*NSECS_PER_SEC) {}

    static Time now();

    inline int64_t toNSec() { return sec * NSECS_PER_SEC + nsec; }
    inline double toSec() { return sec + ((double)nsec)/NSECS_PER_SEC; }

    inline bool isZero() const { return sec == 0 && nsec == 0; }

    inline bool operator==(const Time& t) const { return sec == t.sec && nsec == t.nsec; }

    inline bool operator!=(const Time& t) const { return sec != t.sec || nsec != t.nsec; }

    inline bool operator<(const Time& t) const {
        if (sec == t.sec) {
            return nsec < t.nsec;
        } else {
            return sec < t.sec;
        }
    }

    inline bool operator<=(const Time& t) const {
        if (sec == t.sec) {
            return nsec <= t.nsec;
        } else {
            return sec < t.sec;
        }
    }

    inline bool operator>(const Time& t) const {
        if (sec == t.sec) {
            return nsec > t.nsec;
        } else {
            return sec > t.sec;
        }
    }

    inline bool operator>=(const Time& t) const {
        if (sec == t.sec) {
            return nsec >= t.nsec;
        } else {
            return sec > t.sec;
        }
    }

    inline Time operator+(const Duration& d) const {
        int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
        return Time(nsecs);
    }

    inline Time operator-(const Duration& d) const {
        int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
        return Time(nsecs);
    }

    inline Duration operator-(const Time& t) const {
        int64_t nsecs = (sec - t.sec) * NSECS_PER_SEC + (nsec - t.nsec);
        return Duration(nsecs);
    }

    inline Time& operator+=(const Duration& d) {
        int64_t nsecs = (sec + d.sec) * NSECS_PER_SEC + (nsec + d.nsec);
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

    inline Time& operator-=(const Duration& d) {
        int64_t nsecs = (sec - d.sec) * NSECS_PER_SEC + (nsec - d.nsec);
        sec = nsecs / NSECS_PER_SEC; nsec = nsecs % NSECS_PER_SEC;
        return *this;
    }

    std::string format(const std::string format) const;
    std::string formatDate() const;
    std::string formatDateAndTime() const;

#ifdef ROS_VERSION

    Time(const ros::Time& t) : sec(t.sec), nsec(t.nsec) {}

    inline Time& operator=(const ros::Time& t) {
        sec = t.sec; nsec = t.nsec;
        return *this;
    }

    inline ros::Time toROS() const {
        return ros::Time(sec, nsec);
    }

#endif
};

} // namespace crosbot

#endif /* CROSBOT_DATA_TIME_HPP_ */
