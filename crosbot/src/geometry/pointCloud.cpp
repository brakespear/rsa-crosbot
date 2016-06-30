/*
 * pointCloud.cpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot/geometry/pointCloud.hpp>

namespace crosbot {

PointCloud::PointCloud(std::string frameID, const PointCloud& c, Pose correction) :
    TimeStamptedData(c.timestamp), frameID(frameID)
{
    cloud.resize(c.cloud.size());
    tf::Transform trans = correction.toTF();

    for (size_t i = 0; i < cloud.size(); ++i) {
        cloud[i] = trans * c.cloud[i].toTF();
    }
    colours = c.colours;
}


inline PointCloud& PointCloud_Pointcloud(PointCloud& cloud, const sensor_msgs::PointCloud2& c) {
    cloud.timestamp = c.header.stamp;
    cloud.frameID = c.header.frame_id;

    uint32_t numValues = c.width * c.height;
    cloud.cloud.resize(numValues);

    int Xoffset = -1, Yoffset = -1, Zoffset = -1, colourOffset = -1;
    bool haveRGBA = false;

    for (uint32_t f = 0; f < c.fields.size(); ++f) {
        const sensor_msgs::PointField& field = c.fields[f];
        if (strcasecmp(field.name.c_str(), "x") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
            Xoffset = field.offset;
        } else if (strcasecmp(field.name.c_str(), "y") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
            Yoffset = field.offset;
        } else if (strcasecmp(field.name.c_str(), "z") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
            Zoffset = field.offset;
        } else if (strcasecmp(field.name.c_str(), "rgb") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
            colourOffset= field.offset;
        } else if (strcasecmp(field.name.c_str(), "rgba") == 0 && field.datatype == sensor_msgs::PointField::FLOAT32) {
            colourOffset= field.offset;
            haveRGBA = true;
        }
    }

    if (colourOffset > 0) {
        cloud.colours.resize(numValues);
    }

    for (uint32_t Y = 0; Y < c.height; ++Y) {
        const uint8_t *point = &c.data[Y*c.row_step];

        for (uint32_t X = 0; X < c.height; ++X, point += c.point_step) {
            Point& p = cloud.cloud[Y*c.width+X];

            if (Xoffset >= 0) {
                p.x = *(float *)(point+Xoffset);
            }
            if (Yoffset >= 0) {
                p.y = *(float *)(point+Yoffset);
            }
            if (Zoffset >= 0) {
                p.z = *(float *)(point+Zoffset);
            }

            if (colourOffset >= 0) {
                Colour& colour = cloud.colours[Y*c.width+X];

                colour.b = *(point+colourOffset);
                colour.g = *(point+colourOffset+1);
                colour.r = *(point+colourOffset+2);

                if (haveRGBA)
                    colour.a = *(point+colourOffset+3);
                else
                    colour.a = 255;
            }
        }
    }

    return cloud;
}

PointCloud::PointCloud(const sensor_msgs::PointCloud2ConstPtr& c) {
    PointCloud_Pointcloud(*this, *c);
}

PointCloud::PointCloud(const sensor_msgs::PointCloud2& c) {
    PointCloud_Pointcloud(*this, c);
}

PointCloud& PointCloud::operator=(const sensor_msgs::PointCloud2ConstPtr& c) {
    return PointCloud_Pointcloud(*this, *c);
}

PointCloud& PointCloud::operator=(const sensor_msgs::PointCloud2& c) {
    return PointCloud_Pointcloud(*this, c);
}

} // namespace crosbot
