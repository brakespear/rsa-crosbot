/*
 * pointCloud.hpp
 *
 *  Created on: 29/06/2016
 *      Author: rescue
 */

#ifndef CROSBOT_SERIALIZATION_GEOMETRY_HPP_
#define CROSBOT_SERIALIZATION_GEOMETRY_HPP_

#include <crosbot/serialization.hpp>
#include <crosbot/geometry.hpp>

namespace crosbot {
namespace serialization {

CASROS_SIMPLE_SERIALIZER(Point2D);
CASROS_SIMPLE_SERIALIZER(Point3D);
CASROS_SIMPLE_SERIALIZER(Quaternion);
CASROS_SIMPLE_SERIALIZER(Pose2D);
CASROS_SIMPLE_SERIALIZER(Pose3D);
CASROS_SIMPLE_SERIALIZER(Index2D);

template <>
class Serializer<PointCloud> {
public:
    size_t write(const PointCloud& cloud, OutputStream& stream) throw (IOException) {
        Serializer<Time> serTime;
        Serializer<uint64_t> serUint64;
        Serializer<Point> serPoint;
        Serializer<std::string> serString;
        Serializer<Colour> serColour;

        size_t rval = serTime.write(cloud.timestamp, stream);
        rval += serString.write(cloud.frameID, stream);

        uint64_t n = cloud.cloud.size();
        rval += serUint64.write(n , stream);
        for (size_t i = 0; i < n; i++) {
            rval += serPoint.write(cloud.cloud[i], stream);
        }

        n = cloud.colours.size();
        rval += serUint64.write(n , stream);
        for (size_t i = 0; i < n; i++) {
            rval += serColour.write(cloud.colours[i], stream);
        }
        return rval;
    }

    size_t read(PointCloud& cloud, InputStream& stream) throw (IOException) {
        Serializer<Time> serTime;
        Serializer<uint64_t> serUint64;
        Serializer<Point> serPoint;
        Serializer<std::string> serString;
        Serializer<Colour> serColour;

        size_t rval = serTime.read(cloud.timestamp, stream);
        rval += serString.read(cloud.frameID, stream);

        uint64_t n;
        rval += serUint64.read(n , stream);
        cloud.cloud.resize(n);
        for (size_t i = 0; i < n; i++) {
            rval += serPoint.read(cloud.cloud[i], stream);
        }
        rval += serUint64.read(n , stream);
        cloud.colours.resize(n);
        for (size_t i = 0; i < n; i++) {
            rval += serColour.read(cloud.colours[i], stream);
        }
        return rval;
    }

    size_t serializedLength(const PointCloud& cloud) {
        Serializer<std::string> serString;
        return sizeof(Time) + serString.serializedLength(cloud.frameID) +
                sizeof(uint64_t) + sizeof(Point) * cloud.cloud.size() +
                sizeof(uint64_t) + sizeof(Colour) * cloud.colours.size();
    }
};

} // namespace serialization
} // namespace crosbot

#ifdef ROS_VERSION

namespace ros {
namespace serialization {

template<>
struct Serializer<crosbot::Point3D> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
      stream.next(t.x);
      stream.next(t.y);
      stream.next(t.z);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<crosbot::Quaternion> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
      stream.next(t.x);
      stream.next(t.y);
      stream.next(t.z);
      stream.next(t.w);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

template<>
struct Serializer<crosbot::Pose3D> {
  template<typename Stream, typename T>
  inline static void allInOne(Stream& stream, T t) {
      stream.next(t.position);
      stream.next(t.orientation);
  }
  ROS_DECLARE_ALLINONE_SERIALIZER;
};

} // namespace serialization
} // namespace ros

#endif /* ROS_VERSION */

#endif /* CROSBOT_SERIALIZATION_GEOMETRY_HPP_ */
