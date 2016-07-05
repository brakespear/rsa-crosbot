/*
 * control.cpp
 *
 *  Created on: 04/07/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot/controls/command.hpp>
#include <crosbot/controls/status.hpp>


namespace crosbot {

inline void _local_sendStatus(
    CrosbotStatusPtr status,
    crosbot_msgs::ControlStatus::_level_type level,
    const crosbot_msgs::ControlStatus::_args_type& args,
    const char* fmt, va_list vaList) {

    char* statusStr;
    vasprintf(&statusStr, fmt, vaList);
    status->sendStatus(statusStr, level);
}

void CrosbotStatus::sendStatus(crosbot_msgs::ControlStatus::_level_type level,
        const char* fmt, ...) {
    crosbot_msgs::ControlStatus::_args_type args;
    va_list listStatus;

    va_start(listStatus, fmt);
    _local_sendStatus(this, level, args, fmt, listStatus);
    va_end(listStatus);
}

void CrosbotStatus::sendStatus(crosbot_msgs::ControlStatus::_level_type level,
        const crosbot_msgs::ControlStatus::_args_type& args,
        const char* fmt, ...) {
    va_list listStatus;

    va_start(listStatus, fmt);
    _local_sendStatus(this, level, args, fmt, listStatus);
    va_end(listStatus);
}

} // namespace crosbot
