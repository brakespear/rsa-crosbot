/*
 * map.hpp
 *
 *  Created on: 09/02/2012
 *      Author: rescue
 */

#ifndef CROSBOT_MAP_HPP_
#define CROSBOT_MAP_HPP_

#include <crosbot/data.hpp>
#include <crosbot/config.hpp>
#include <crosbot_map/tag.hpp>
#include <crosbot/thread.hpp>

#include <nav_msgs/OccupancyGrid.h>

namespace crosbot {

class Path : public HandledObject {
public:
	std::vector<Pose> path;
	std::vector<Time> timestamps;

	Path() {}
	Path(Pose pose, Time timstamp = Time(0,0)) {
		path.push_back(pose);
	}
};
typedef Handle<Path> PathPtr;

class Map;
typedef Handle<Map> MapPtr;

class MapListener {
public:
	virtual ~MapListener() {}
	virtual void motionTracked(MapPtr map)=0;
	virtual void mapUpdated(MapPtr map)=0;
	virtual void tagAdded(MapPtr map, TagPtr tag)=0;
	virtual void tagChanged(MapPtr map, TagPtr tag)=0;
};

// Map Class
class Map : public HandledObject {
public:
	struct TagLocation {
	public:
		TagPtr tag;
		Pose robot;
		Pose mapPose;

		TagLocation(TagPtr tag, Pose robot, Pose mapPose) : tag(tag), robot(robot), mapPose(mapPose) {}
	};

	class TagList;
	typedef Handle<TagList> TagListPtr;
	class TagList : public HandledObject {
	public:
		std::vector<TagLocation> tags;

		TagList() {}
		TagList(TagListPtr parent) : tags(parent->tags) {}
	};

	Map() {}
	virtual ~Map(){}

	/**
	 * Methods for configuring and starting/stopping.
	 */
	virtual void configure(ConfigElementPtr config) {}
	virtual void start() {}
	virtual void stop() {}

	/**
	 * Allow for map to be paused resumed.
	 */
	virtual void setPaused(bool pause)=0;
	virtual bool isPaused()=0;

	/**
	 * Allow map to be set in track only mode.
	 */
	virtual void setUpdateMap(bool update)=0;

	/**
	 * Map can be cloned, saved and loaded.
	 */
	virtual MapPtr clone()=0;
	virtual bool load(std::string filename) throw(IOException)=0;
	virtual bool save(std::string filename) throw(IOException)=0;

	/**
	 * Gets the current pose and position tracker pose.
	 */
	virtual Pose getCurrentPose(Pose* trackerPose = NULL)=0;

	/**
	 * Gets the tags in the map.
	 */
	virtual TagListPtr getTags() { return NULL; }

	/**
	 * Gets the path the robot has followed if the map allows it.
	 */
	virtual PathPtr getPath() {
		return new Path(getCurrentPose());
	}

	/**
	 * Allows the robots pose to be reset.
	 */
	virtual void resetPose(Pose3D pose = Pose())=0;

	/**
	 * Data for maps
	 */
	virtual void newPointCloud(PointCloudPtr cloud, Pose robot, Pose sensor)=0;
	virtual void addTag(TagPtr tag) {}

	/**
	 * Convert to a simple map type.
	 */
	virtual nav_msgs::OccupancyGridPtr asOccupancyGrid()=0;

protected:
	std::vector<MapListener *> listeners;
	ReadWriteLock rwListeners;

	// alert listeners to altered map
	void motionTracked() {
		std::vector<MapListener *> listeners;
		{{ Lock lock(rwListeners);
			listeners = this->listeners;
		}}
		for (unsigned int i = 0; i < listeners.size(); i++) {
			listeners[i]->motionTracked(this);
		}
	}

	// alert listeners to altered map
	void mapChanged() {
		std::vector<MapListener *> listeners;
		{{ Lock lock(rwListeners);
			listeners = this->listeners;
		}}
		for (unsigned int i = 0; i < listeners.size(); i++) {
			listeners[i]->mapUpdated(this);
		}
	}

	// alert listeners to altered tag
	void tagAdded(TagPtr tag) {
		std::vector<MapListener *> listeners;
		{{ Lock lock(rwListeners);
			listeners = this->listeners;
		}}
		for (unsigned int i = 0; i < listeners.size(); i++) {
			listeners[i]->tagAdded(this, tag);
		}
	}

public:
	// alert listeners to altered tag
	void tagChanged(TagPtr tag) {
		std::vector<MapListener *> listeners;
		{{ Lock lock(rwListeners);
			listeners = this->listeners;
		}}
		for (unsigned int i = 0; i < listeners.size(); i++) {
			listeners[i]->tagChanged(this, tag);
		}
	}

	/**
	 * To allow listeners to know when a map changes.
	 */
	void addListener(MapListener *listener) {
		Lock lock(rwListeners, true);
		listeners.push_back(listener);
	}

	void removeListener(MapListener *listener) {
		Lock lock(rwListeners, true);
		std::vector<MapListener *>::iterator it = listeners.begin();
		while (it !=listeners.end()) {
			if ((*it) == listener) {
				listeners.erase(it);
				it = listeners.begin();
			} else {
				it++;
			}
		}
	}
};

class MapManager {
protected:
	std::map<std::string, MapPtr> maps;
public:
	MapManager() {}
	~MapManager() { maps.clear(); }

	MapPtr getMap(std::string name);
	void addMap(std::string name, MapPtr map);
	void removeMap(std::string name);
	void removeMap(MapPtr map);
	std::string getName(MapPtr map);

	unsigned int getCount() { return maps.size(); }
	MapPtr getMap(unsigned int idx);

	void startMaps();
	void stopMaps();

	void clear() { maps.clear(); }
};

typedef MapPtr (*MapFactoryFunc)(ConfigElementPtr config);
class MapFactory {
public:
	static MapPtr createMap(ConfigElementPtr config);
	static void addFactory(MapFactoryFunc factory);
};

} // namespace crosbot

#endif /* CROSBOT_MAP_HPP_ */
