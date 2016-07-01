/*
 * pointCloud.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GEOMETRY_POINTCLOUD_HPP_
#define CROSBOT_GEOMETRY_POINTCLOUD_HPP_

#include <cstdint>
#include <ostream>
#include <string>
#include <vector>

#include <crosbot/data/colour.hpp>
#include <crosbot/data/timestampedData.hpp>
#include <crosbot/geometry/defines.hpp>
#include <crosbot/geometry/points.hpp>
#include <crosbot/geometry/poses.hpp>

#ifdef ROS_VERSION

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>

#include <crosbot_msgs/ColourMsg.h>
#include <crosbot_msgs/PointCloudMsg.h>

using crosbot_msgs::ColourMsg;
using crosbot_msgs::ColouredPointMsg;
using crosbot_msgs::PointCloudMsg;
using crosbot_msgs::PointCloudMsgPtr;
using crosbot_msgs::PointCloudMsgConstPtr;

#endif

namespace crosbot {

class PointCloud;
typedef Handle<PointCloud> PointCloudPtr;

/**
 * Crosbot point cloud representation, containing a set of crosbot::Point3D
 * mapped to crosbot::Colour.
 * Note that for reasons of efficiency (that is if colour are not required)
 * crosbot::ColouredPoint is not used.
 */
class PointCloud : public TimeStamptedData {
public:
    /**
     * The frame of reference the points in the cloud are in.
     */
    std::string frameID;

    /**
     * The set of points in the cloud.
     */
    std::vector<Point> cloud;

    /**
     * The set colours for each point in the cloud.  This can be empty if the points aren't coloured.
     */
    std::vector<Colour> colours;

    PointCloud() : frameID("") {}
    PointCloud(std::string frameID) : frameID(frameID) {}
    PointCloud(const PointCloud& pc) : TimeStamptedData(pc.timestamp), frameID(pc.frameID) { cloud = pc.cloud; colours = pc.colours; }
    PointCloud(std::string frameID, const PointCloud&, Pose correction);

    inline bool hasNAN() {
        for (size_t i = 0; i < cloud.size(); i++) {
            if (cloud[i].hasNAN())
                return true;
        }

        return false;
    }

#ifdef ROS_VERSION

    PointCloud(const PointCloudMsg& c) {
        timestamp = c.header.stamp;
        frameID = c.header.frame_id;

        size_t n = c.points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c.points[i];
        }
        n = c.colours.size();
        colours.resize(n);
        for (size_t i = 0; i < n; i++) {
            colours[i] = c.colours[i];
        }
    }

    PointCloud(const PointCloudMsgConstPtr& c) {
        timestamp = c->header.stamp;
        frameID = c->header.frame_id;

        size_t n = c->points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c->points[i];
        }
        n = c->colours.size();
        colours.resize(n);
        for (size_t i = 0; i < n; i++) {
            colours[i] = c->colours[i];
        }
    }

    PointCloud(const sensor_msgs::PointCloud& c) {
        timestamp = c.header.stamp;
        frameID = c.header.frame_id;

        size_t n = c.points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c.points[i];
        }

        for (size_t ch = 0; ch < c.channels.size(); ++ch) {
            const sensor_msgs::ChannelFloat32& channel = c.channels[ch];
            if (channel.name == "rgb") {
                n = channel.values.size();
                colours.resize(n);
                for (size_t i = 0; i < n; i++) {
                    uint32_t cInt = *((uint32_t *)&channel.values[i]);
                    Colour& colour = colours[i];
                    colour.r = (cInt >> 16) & 0xFF;
                    colour.g = (cInt >> 8) & 0xFF;
                    colour.b = cInt & 0xFF;
                    colour.a = 0;
                }
            } else if (channel.name == "rgba") {
                n = channel.values.size();
                colours.resize(n);
                for (size_t i = 0; i < n; i++) {
                    uint32_t cInt = *((uint32_t *)&channel.values[i]);
                    Colour& colour = colours[i];
                    colour.r = (cInt >> 24) & 0xFF;
                    colour.g = (cInt >> 16) & 0xFF;
                    colour.b = (cInt >> 8) & 0xFF;
                    colour.a = cInt & 0xFF;
                }
            }
        }
    }

    PointCloud(const sensor_msgs::PointCloudConstPtr& c) {
        timestamp = c->header.stamp;
        frameID = c->header.frame_id;

        size_t n = c->points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c->points[i];
        }
    }

    PointCloud(const sensor_msgs::LaserScan& ls) {
        timestamp = ls.header.stamp;
        frameID = ls.header.frame_id;

        Point p, origin;
        float angle = ls.angle_min, angleInc = ls.angle_increment, range,
                rangeMin = ls.range_min, rangeMax = ls.range_max;

        size_t n = ls.ranges.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; ++i, angle += angleInc) {
            range = ls.ranges[i];
            if (range >= rangeMin && range <= rangeMax) {
                p.x = cos(angle) * range;
                p.y = sin(angle) * range;

                cloud[i] = p;
            } else {
                cloud[i] = origin;
            }
        }
    }

    PointCloud(const sensor_msgs::LaserScanConstPtr& ls, bool includeOutOfRange = false) {
        timestamp = ls->header.stamp;
        frameID = ls->header.frame_id;

        Point p;
        float angle = ls->angle_min, angleInc = ls->angle_increment, range,
                rangeMin = ls->range_min, rangeMax = ls->range_max;
        uint32_t discardedReadings = 0;

        size_t n = ls->ranges.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; ++i, angle += angleInc) {
            range = ls->ranges[i];
            if (includeOutOfRange || (range >= rangeMin && range <= rangeMax)) {
                p.x = cos(angle) * range;
                p.y = sin(angle) * range;

                cloud[i - discardedReadings] = p;
            } else {
                ++discardedReadings;
            }
        }
        cloud.resize(n - discardedReadings);
    }

    inline PointCloud& operator=(const sensor_msgs::PointCloud& c) {
        timestamp = c.header.stamp;
        frameID = c.header.frame_id;

        size_t n = c.points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c.points[i];
        }
        return *this;
    }

    inline PointCloud& operator=(const sensor_msgs::PointCloudConstPtr& c) {
        timestamp = c->header.stamp;
        frameID = c->header.frame_id;

        size_t n = c->points.size();
        cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            cloud[i] = c->points[i];
        }
        return *this;
    }

    inline PointCloudMsgPtr toROS() const {
        PointCloudMsgPtr rval(new PointCloudMsg());
        rval->header.stamp = timestamp.toROS();
        rval->header.frame_id = frameID;

        size_t n = cloud.size();
        rval->points.resize(n);
        for (size_t i = 0; i < n; i++) {
            rval->points[i] = cloud[i].toROS();
        }

        return rval;
    }

    inline sensor_msgs::PointCloudPtr toROS1() const {
        sensor_msgs::PointCloudPtr rval(new sensor_msgs::PointCloud());
        rval->header.stamp = timestamp.toROS();
        rval->header.frame_id = frameID;

        rval->points.resize(cloud.size());

        for (size_t i = 0; i < cloud.size(); ++i) {
            rval->points[i] = cloud[i].toROS32();
        }

        // Add colour channels
        if (colours.size() > 0) {
            rval->channels.resize(1);
            sensor_msgs::ChannelFloat32& channel = rval->channels[0];
            channel.name = "rgb";
            channel.values.resize(cloud.size());

            uint32_t c32;
            for (size_t i = 0; i < cloud.size(); ++i) {
                const Colour& colour = colours[i];
                c32 = ((uint32_t)colour.r) << 16 | ((uint32_t)colour.g) << 8 | ((uint32_t)colour.b);
                channel.values[i] = *(float*)(&c32);
            }
        }

        return rval;
    }

    PointCloud(const sensor_msgs::PointCloud2ConstPtr& c);
    PointCloud(const sensor_msgs::PointCloud2& c);
    PointCloud& operator=(const sensor_msgs::PointCloud2ConstPtr& c);
    PointCloud& operator=(const sensor_msgs::PointCloud2& c);

#endif
};

#ifdef ROS_VERSION
// XXX: Why does ROS not generate an equality operator?

inline bool operator==(const ColourMsg& c1, const ColourMsg& c2) {
    return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b && c1.a == c2.a;
}

inline bool operator==(const ColouredPointMsg& p1, const ColouredPointMsg& p2) {
    return p1.p == p2.p && p1.c == p2.c;
}

/**
 * Tests that a value isn't corrupt.
 */
inline bool hasNAN(const ColouredPointMsg& cp) {
    return hasNAN(cp.p);
}

inline bool hasNAN(const PointCloudMsg& pc) {
    for (size_t i = 0; i < pc.points.size(); i++) {
        if (hasNAN(pc.points[i]))
            return true;
    }
    return false;
}

#endif // ROS_VERSION

} // namespace crosbot


#endif /* CROSBOT_GEOMETRY_POINTCLOUD_HPP_ */
