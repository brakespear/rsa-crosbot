/*
 * data.cpp
 *
 *  Created on: 16/02/2012
 *      Author: rescue
 */

#include <ros/ros.h>

#include <sys/time.h>

#include <crosbot/data/time.hpp>

// TODO: remove
//#include <tf/tf.h>
//
//#include <crosbot/data.hpp>
//#include "image.hpp"
//#include <crosbot/utils.hpp>
//
//#include <string.h>
//
//#include <tf/transform_datatypes.h>

namespace crosbot {

Time Time::now() {
	timeval time;
	gettimeofday(&time, NULL);
	return Time(time.tv_sec, time.tv_usec*1000);
}

std::string Time::format(const std::string format) const {
	time_t nowtime;
	struct tm *nowtm;
	char tmbuf[1024];

	nowtime = sec;
	nowtm = localtime(&nowtime);
	strftime(tmbuf, sizeof tmbuf, format.c_str(), nowtm);
	std::string rval(tmbuf);
	return rval;
}

std::string Time::formatDate() const {
	time_t nowtime;
	struct tm *nowtm;
	char tmbuf[256];

	nowtime = sec;
	nowtm = localtime(&nowtime);
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d", nowtm);
	std::string rval(tmbuf);
	return rval;
}

std::string Time::formatDateAndTime() const {
	time_t nowtime;
	struct tm *nowtm;
	char tmbuf[256];

	nowtime = sec;
	nowtm = localtime(&nowtime);
	strftime(tmbuf, sizeof tmbuf, "%Y-%m-%d-%H-%M-%S", nowtm);

	std::string rval(tmbuf);
	snprintf(tmbuf, sizeof tmbuf, ".%03d", nsec / 1000000);
	rval.append(tmbuf);
	return rval;
}

} // namespace crosbot
