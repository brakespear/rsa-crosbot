/*
 * tag.cpp
 *
 *  Created on: 19/02/2013
 *      Author: mmcgill
 */

#include <crosbot_map/tag.hpp>

namespace crosbot {

const UUID Tag::uuid("80d701c6-aa6b-47ad-b2a3-fe425fdc3c06");
const UUID Snap::uuid("b7868893-6bae-4f54-b6c7-4d4cb76d0a1c");

std::vector<TagFactoryFunc> tagFactories;
TagPtr Tag::createTag(const UUID& uuid) {
	TagPtr rval;

	if (uuid == Snap::uuid)
		rval = new Snap();
	else if (uuid == Tag::uuid)
		rval = new Tag();

	for (uint32_t i = 0; rval != NULL && i < tagFactories.size(); i++) {
		rval = tagFactories[i](uuid);
	}
	if (rval == NULL)
		rval = new Tag();
	return rval;
}

void Tag::addTagFactory(TagFactoryFunc factory) {
	tagFactories.push_back(factory);
}


size_t Tag::write(serialization::OutputStream& stream) const throw (IOException) {
	stream.next(pose);
	stream.next(robot);

	return sizeof(Pose) + sizeof(Pose);
}

size_t Tag::read(serialization::InputStream& stream) throw (IOException) {
	stream.next(pose);
	stream.next(robot);

	return sizeof(Pose) + sizeof(Pose);
}

// Length of the tag not including the UUID
size_t Tag::serializedLength() const {
	return sizeof(Pose) + sizeof(Pose);
}




size_t Snap::write(serialization::OutputStream& stream) const throw (IOException) {
	size_t rval = Tag::write(stream);
	serialization::Serializer<std::string> serString;
	serialization::Serializer<uint8_t> serUInt8;
	serialization::Serializer<uint64_t> serUInt64;
	serialization::Serializer<Image> serImage;
	serialization::Serializer<PointCloud> serPointCloud;

	rval += serString.write(description, stream);
	uint8_t uint8 = type;
	rval +=  serUInt8.write(uint8, stream);
	uint8 = status;
	rval +=  serUInt8.write(uint8, stream);

	uint64_t n = 0;
	for (uint32_t i = 0; i < images.size(); i++) {
		if (images[i] != NULL)
			++n;
	}
	rval += serUInt64.write(n, stream);
	for (uint32_t i = 0; i < images.size(); i++) {
		if (images[i] != NULL)
			rval += serImage.write(*(images[i]), stream);
	}

	n = 0;
	for (uint32_t i = 0; i < clouds.size(); i++) {
		if (clouds[i] != NULL)
			++n;
	}
	rval += serUInt64.write(n, stream);
	for (uint32_t i = 0; i < clouds.size(); i++) {
		if (clouds[i] != NULL)
			rval += serPointCloud.write(*(clouds[i]), stream);
	}

	return rval;
}

size_t Snap::read(serialization::InputStream& stream) throw (IOException) {
	size_t rval = Tag::read(stream);
	uint8_t uint8;
	serialization::Serializer<std::string> serString;
	serialization::Serializer<uint8_t> serUInt8;
	serialization::Serializer<uint64_t> serUInt64;
	serialization::Serializer<Image> serImage;
	serialization::Serializer<PointCloud> serPointCloud;

	rval += serString.read(description, stream);
	rval +=  serUInt8.read(uint8, stream);
	type = (Type)uint8;
	rval +=  serUInt8.read(uint8, stream);
	status = (Status)uint8;

	uint64_t n = 0;
	rval += serUInt64.read(n, stream);
	images.resize(n);
	for (uint32_t i = 0; i < n; i++) {
		images[i] = new Image();
		rval += serImage.read(*(images[i]), stream);
	}

	rval += serUInt64.read(n, stream);
	clouds.resize(n);
	for (uint32_t i = 0; i < n; i++) {
		clouds[i] = new PointCloud();
		rval += serPointCloud.read(*(clouds[i]), stream);
	}

	return rval;
}

// Length of the tag
size_t Snap::serializedLength() const {
	size_t rval = Tag::serializedLength();

	serialization::Serializer<std::string> serString;
	serialization::Serializer<Image> serImage;
	serialization::Serializer<PointCloud> serPointCloud;

	rval += serString.serializedLength(description) + sizeof(uint8_t) + sizeof(uint8_t);

	rval += sizeof(uint64_t);
	for (uint32_t i = 0; i < images.size(); i++) {
		if (images[i] != NULL)
			rval += serImage.serializedLength(*(images[i]));
	}
	rval += sizeof(uint64_t);
	for (uint32_t i = 0; i < clouds.size(); i++) {
		if (clouds[i] != NULL)
			rval += serPointCloud.serializedLength(*(clouds[i]));
	}

	return rval;
}

} // namespace crosbot
