/*
 * commandPublisher.hpp
 *
 *  Created on: 02/07/2016
 *      Author: rescue
 */

#ifndef CROSBOT_COMMANDPUBLISHER_HPP_
#define CROSBOT_COMMANDPUBLISHER_HPP_

#include <crosbot/handle.hpp>
#include <crosbot/controls/defines.hpp>
#include <crosbot_msgs/ControlCommand.h>

#include <ros/publisher.h>
#include <ros/subscriber.h>

namespace crosbot {

/**
 * Class on which command callback is based
 */
class CrosbotCommandCallback : public HandledObject {
public:
    CrosbotCommandCallback() {};
    virtual ~CrosbotCommandCallback() {};

    virtual void callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) = 0;
};
typedef Handle<CrosbotCommandCallback> CrosbotCommandCallbackPtr;

/**
 * Common generalised class for sending and receiving crosbot command messages.
 */
class CrosbotCommand : public HandledObject {
private:

    crosbot_msgs::ControlCommand::_cmd_namespace_type nspace;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    CrosbotCommandCallbackPtr callback;

    bool send;
    bool receive;

    void _callback_receivedCommand(const crosbot_msgs::ControlCommandPtr command) {
        if (command->cmd_namespace == "" ||
            command->cmd_namespace == nspace) {
            callback->callback_receivedCommand(command);
        }
    };

    /**
     * Send command with the provided args
     */
    void sendCommand(crosbot_msgs::ControlCommand::_cmd_namespace_type nspace,
            crosbot_msgs::ControlCommand::_command_type command,
            const crosbot_msgs::ControlCommand::_args_type& args) {
        if (send) {
            crosbot_msgs::ControlCommand msg;
            msg.cmd_namespace = nspace;
            msg.command = command;
            msg.args = args;
            publisher.publish(msg);
        }
    }
public:

    /**
     * Create a new command that optionally has the capacity to send and receive.
     * The node is always possible of sending control commands, but only optionally may receive command
     *     messages.
     * A command can either be broadcast to all crosbot nodes,
     *     or targeted to a given namespace
     * A command publisher can send command to either the "all" namespace, or its configured namespace.
     * A command subscriber receives commands from the "all" namespace and the provided namespace.
     *
     * @param nspace Namespace to subscribe to (in addition to "all")
     * @param callback Callback class for subscribing to command messages
     */
    CrosbotCommand(crosbot_msgs::ControlCommand::_cmd_namespace_type nspace = "",
            CrosbotCommandCallbackPtr callback = NULL) :
        send(true),
        receive(false),
        nspace(nspace),
        callback(callback)
    {
        // Node handle with "root" namespace
        ros::NodeHandle nh("");
        if (send) {
            publisher = nh.advertise<crosbot_msgs::ControlCommand>(TOPIC_CROSBOT_CONTROL_COMMANDS, 1);
        }
        if (callback != NULL) {
            receive = true;
            subscriber = nh.subscribe(TOPIC_CROSBOT_CONTROL_COMMANDS, 1, &CrosbotCommand::_callback_receivedCommand, this);
        }
    };
    virtual ~CrosbotCommand() {
        if (send) {
            publisher.shutdown();
        }
        if (receive) {
            subscriber.shutdown();
        }
    };

    /**
     * Can this command system send crosbot control commands.
     */
    bool canSend() { return send; };

    /**
     * Can this command system receive crosbot control commands.
     */
    bool canReceive() { return receive; };

    /**
     * Send command with arguments using "all" namespace
     */
    void sendCommandAll(crosbot_msgs::ControlCommand::_command_type command,
            const crosbot_msgs::ControlCommand::_args_type& args) {
        sendCommand("", command, args);
    }

    /**
     * Send command with zero arguments using "all" namespace
     */
    void sendCommandAll(crosbot_msgs::ControlCommand::_command_type command) {
        std::vector<std::string> args;
        sendCommandAll(command, args);
    }

    /**
     * Send command with zero arguments using the configured namespace
     */
    void sendCommandNamespace(crosbot_msgs::ControlCommand::_command_type command,
            const crosbot_msgs::ControlCommand::_args_type& args) {
        sendCommand(nspace, command, args);
    }

    /**
     * Send command with zero arguments using the configured namespace
     */
    void sendCommandNamespace(crosbot_msgs::ControlCommand::_command_type command) {
        std::vector<std::string> args;
        sendCommandNamespace(command, args);
    }

};
typedef Handle<CrosbotCommand> CrosbotCommandPtr;

} // namespace crosbot

#endif /* CROSBOT_COMMANDPUBLISHER_HPP_ */
