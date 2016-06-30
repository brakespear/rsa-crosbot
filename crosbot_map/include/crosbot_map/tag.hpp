/*
 * tags.hpp
 *
 *  Created on: 19/02/2013
 *      Author: mmcgill
 */

#ifndef CROSBOT_TAG_HPP_
#define CROSBOT_TAG_HPP_

#include <iostream>

#include <crosbot/data.hpp>
#include <crosbot/geometry.hpp>
#include <crosbot/serialization.hpp>
#include <crosbot/serialization/data.hpp>
#include <crosbot/serialization/geometry.hpp>

#include <crosbot/PointCloudMsg.h>
#include <crosbot_map/SnapMsg.h>

namespace crosbot {

class Tag;
typedef Handle<Tag> TagPtr;

typedef TagPtr (*TagFactoryFunc)(const UUID uuid);

class Tag : public TimeStamptedData {
public:
	static const UUID uuid;

	/**
	 * The location/pose of the item relative to the robot.
	 */
	Pose pose;

	/**
	 * The pose of the robot.
	 */
	Pose robot;

	/**
	 * A unique identifier.
	 */
	uint32_t id;

	virtual UUID getUUID() const { return uuid; }

	virtual size_t write(serialization::OutputStream& stream) const throw (IOException);

	virtual size_t read(serialization::InputStream& stream) throw (IOException);

	// Length of the tag not including the UUID
	virtual size_t serializedLength() const;

	static TagPtr createTag(const UUID& uuid);
protected:
	void addTagFactory(TagFactoryFunc);
};

class Snap : public Tag {
public:
	static const UUID uuid;
	virtual UUID getUUID() const { return uuid; }

	/**
	 * Possible values of a snap's status.
	 */
	enum Status {
		REJECTED    = -1,
		UNCONFIRMED = 0,
		CONFIRMED   = 1,
		DUPLICATE   = 2,
	};

	/**
	 * Possible snap types.
	 */
	enum Type {
		VICTIM      = 0,
		LANDMARK    = 1,
		SCAN        = 2,
		HOLE        = 3,
	};

	/**
	 * The type of object the snap represents.
	 */
	Type type;

	/**
	 * The status of the snap.
	 */
	Status status;

	/**
	 * A description of the snap.
	 */
	std::string description;

	/**
	 * Images related to the snap.
	 */
	std::vector<ImagePtr> images;

	/**
	 * Point clouds included in the snap.
	 */
	std::vector<PointCloudPtr> clouds;

	Snap() : type(LANDMARK), status(UNCONFIRMED) {}
	Snap(Type type, Status status=UNCONFIRMED) : type(type), status(status) {}
	virtual size_t write(serialization::OutputStream& stream) const throw (IOException);
	virtual size_t read(serialization::InputStream& stream) throw (IOException);
	virtual size_t serializedLength() const;

#ifdef ROS_VERSION
	inline Snap& operator=(const crosbot_map::SnapMsg& snp) {
		id = snp.id;
		timestamp = snp.header.stamp;
		type = (Snap::Type)snp.type;
		status = (Snap::Status)snp.status;

		robot = snp.robot;
		pose = snp.pose;

		description = snp.description;

		images.clear();
		for (size_t i = 0; i < snp.images.size(); i++) {
			ImagePtr img = new Image(snp.images[i]);
			images.push_back(img);
		}
		clouds.clear();
		for (size_t i = 0; i < snp.clouds.size(); i++) {
			PointCloudPtr cloud = new PointCloud(snp.clouds[i]);
			clouds.push_back(cloud);
		}

		return *this;
	}

	inline Snap& operator=(const crosbot_map::SnapMsgConstPtr& snp) {
		return *this = *snp;
	}

	Snap(const crosbot_map::SnapMsg& snp) {
		*this = snp;
	}

	Snap(const crosbot_map::SnapMsgConstPtr& snp) {
		*this = *snp;
	}

    inline crosbot_map::SnapMsgPtr toROS() const {
    	crosbot_map::SnapMsgPtr rval(new crosbot_map::SnapMsg());
    	rval->id = id;
    	rval->header.stamp = timestamp.toROS();
    	rval->type = type;
    	rval->status = status;
    	rval->robot = robot.toROS();
    	rval->pose = pose.toROS();
    	rval->description = description;

		for (size_t i = 0; i < images.size(); i++) {
			if (images[i] != NULL) {
				rval->images.push_back(*(images[i]->toROS()));
			}
		}
		for (size_t i = 0; i < clouds.size(); i++) {
			if (clouds[i] != NULL) {
				rval->clouds.push_back(*(clouds[i]->toROS()));
			}
		}

    	return rval;
    }

    inline crosbot_map::SnapMsgPtr toROSsmall() const {
    	crosbot_map::SnapMsgPtr rval(new crosbot_map::SnapMsg());

    	rval->id = id;
    	rval->header.stamp = timestamp.toROS();
    	rval->type = type;
    	rval->status = status;
    	rval->robot = robot.toROS();
    	rval->pose = pose.toROS();
    	rval->description = description;

    	return rval;
    }

#endif
};
typedef Handle<Snap> SnapPtr;

namespace serialization {

template <>
class Serializer<Tag> {
public:
	size_t write(const Tag& tag, OutputStream& stream) throw (IOException) {
		size_t rval = stream.next(tag.getUUID());
		uint64_t size = tag.serializedLength();
		rval += stream.next(size);
		return rval + tag.write(stream);
	}

	TagPtr read(InputStream& stream) throw (IOException) {
		UUID uuid;
		uint64_t tagSize;
		stream.next(uuid);
		stream.next(tagSize);
		size_t posn = stream.position();
		TagPtr rval = Tag::createTag(uuid);
		if (rval == NULL)
			rval = new Tag();
		uint64_t read = rval->read(stream);
		if (read > tagSize)
			throw IOException("Tag overread.");
		else if (read < tagSize) {
			// skip extra data in tag
			stream.seek(posn + tagSize);
		}

		return rval;
	}

	size_t read(Tag& tag, InputStream& stream) throw (IOException) {
		throw IOException("Wrong read method for tags.\n");
	}

	size_t serializedLength(const Tag& tag) {
		return sizeof(UUID) + sizeof(uint64_t) + tag.serializedLength();
	}
};

} // namespace serialization

} // namespace crosbot


#endif /* CROSBOT_TAG_HPP_ */
