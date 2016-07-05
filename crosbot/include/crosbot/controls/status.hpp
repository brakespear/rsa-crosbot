/*
 * commandPublisher.hpp
 *
 *  Created on: 02/07/2016
 *      Author: rescue
 */

#ifndef CROSBOT_STATUSPUBLISHER_HPP_
#define CROSBOT_STATUSPUBLISHER_HPP_

#include <crosbot/handle.hpp>
#include <crosbot/controls/defines.hpp>
#include <crosbot_msgs/ControlStatus.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace crosbot {

/**
 * Class on which status callback is based
 */
class CrosbotStatusCallback : public HandledObject {
public:
    CrosbotStatusCallback() {};
    virtual ~CrosbotStatusCallback() {};

    virtual void callback_receivedStatus(const crosbot_msgs::ControlStatusPtr command) = 0;
};
typedef Handle<CrosbotStatusCallback> CrosbotStatusCallbackPtr;

/**
 * Common generalised class for sending and receiving crosbot control status messages.
 */
class CrosbotStatus : public HandledObject {
private:

    ros::Publisher publisher;
    ros::Subscriber subscriber;
    crosbot_msgs::ControlStatus::_stats_namespace_type nspace;
    CrosbotStatusCallbackPtr callback;

    bool send;
    bool receive;

    void _callback_receivedStatus(const crosbot_msgs::ControlStatusPtr command) {
        callback->callback_receivedStatus(command);
    };
public:

    /**
     * Create a new status that optionally has the capacity to send and receive.
     * The node is always possible of sending control commands, but only optionally may receive status
     *     messages.
     * All status messages are broadcast with an annotated namespace indicating where the
     *     status originated from
     * A status publisher sends from the configured namespace.
     * A status subscriber receives status from all namespaces (unlike control commands).
     *
     * @param send Enable sending of crosbot control status.
     * @param receive Enable receiving of crosbot control status.
     */
    CrosbotStatus(crosbot_msgs::ControlStatus::_stats_namespace_type nspace = "",
            CrosbotStatusCallbackPtr callback = NULL) :
        send(true),
        receive(false),
        nspace(nspace),
        callback(callback)
    {
        // Node handle with "root" namespace
        ros::NodeHandle nh("");
        if (send) {
            publisher = nh.advertise<crosbot_msgs::ControlStatus>(TOPIC_CROSBOT_CONTROL_STATUS, 1);
        }
        if (callback != NULL) {
            receive = true;
            subscriber = nh.subscribe(TOPIC_CROSBOT_CONTROL_STATUS, 1, &CrosbotStatus::_callback_receivedStatus, this);
        }
    };
    virtual ~CrosbotStatus() {
        if (send) {
            publisher.shutdown();
        }
        if (receive) {
            subscriber.shutdown();
        }
    };

    /**
     * Can this command system send crosbot control status.
     */
    bool canSend() { return send; };

    /**
     * Can this command system receive crosbot control status.
     */
    bool canReceive() { return receive; };

    /**
     * Send status with the provided args, under the given namespace
     */
    void sendStatus(crosbot_msgs::ControlStatus::_status_type status,
            crosbot_msgs::ControlStatus::_level_type level,
            const crosbot_msgs::ControlStatus::_args_type& args) {
        if (send) {
            crosbot_msgs::ControlStatus msg;
            msg.level = level;
            msg.stats_namespace = nspace;
            msg.status = status;
            msg.args = args;
            publisher.publish(msg);
        }
    }

    /**
     * Send status with zero arguments, under the given namespace
     */
    void sendStatus(crosbot_msgs::ControlStatus::_status_type status,
            crosbot_msgs::ControlStatus::_level_type level) {
        crosbot_msgs::ControlStatus::_args_type args;
        sendStatus(status, level, args);
    }
};
typedef Handle<CrosbotStatus> CrosbotStatusPtr;

} // namespace crosbot

#endif /* CROSBOT_STATUSPUBLISHER_HPP_ */
