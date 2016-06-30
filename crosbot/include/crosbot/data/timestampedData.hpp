/*
 * timestampedData.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_DATA_TIMESTAMPEDDATA_HPP_
#define CROSBOT_DATA_TIMESTAMPEDDATA_HPP_

#include <crosbot/data/defines.hpp>
#include <crosbot/data/time.hpp>
#include <crosbot/handle.hpp>

namespace crosbot {

/**
 * A superclass used for more complex data classes, which
 * (1) Keeps a generated time stamp, and
 * (2) Extends crosbot::HandledObject to provided a managed data structure.
 */
class TimeStamptedData : public HandledObject {
public:
    Time timestamp;

    TimeStamptedData(Time stamp = Time::now()) :
        HandledObject(),
        timestamp(stamp)
    {}
    TimeStamptedData(TimeStamptedData& other) :
        HandledObject(),
        timestamp(other.timestamp)
    {}

    virtual ~TimeStamptedData() {}

    Time getTimestamp() const {
        return timestamp;
    }
};

} // namespace crosbot

#endif /* CROSBOT_DATA_TIMESTAMPEDDATA_HPP_ */
