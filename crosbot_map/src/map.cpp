/*
 * map.cpp
 *
 *  Created on: 17/02/2012
 *      Author: rescue
 */

#include <ros/ros.h>

#include <crosbot_map/map.hpp>
#include <crosbot/utils.hpp>

namespace crosbot {

MapPtr MapManager::getMap(std::string name) {
	std::map<std::string, MapPtr>::iterator found = maps.find(name);
	if (found != maps.end())
		return found->second;
	return NULL;
}

void MapManager::addMap(std::string name, MapPtr map) {
	std::map<std::string, MapPtr>::iterator found = maps.find(name);

	if (found == maps.end()) {
		maps.insert(std::pair<std::string, MapPtr>(name, map));
	} else {
		found->second = map;
	}
}

void MapManager::removeMap(std::string name) {
	std::map<std::string, MapPtr>::iterator found = maps.find(name);
	if (found != maps.end()) {
		maps.erase(found);
	}
}

void MapManager::removeMap(MapPtr map) {
	std::map<std::string, MapPtr>::iterator it = maps.begin();
	while (it != maps.end()) {
		if (it->second == map) {
			maps.erase(it);
			it = maps.begin();
		} else {
			it++;
		}
	}
}

MapPtr MapManager::getMap(unsigned int idx) {
	std::map<std::string, MapPtr>::iterator it = maps.begin();
	while (it != maps.end() && idx > 0) {
		it++;
		idx--;
	}
	if (it == maps.end())
		return NULL;

	return it->second;
}

std::string MapManager::getName(MapPtr map) {
	std::map<std::string, MapPtr>::iterator it = maps.begin();
	while (it != maps.end() && it->second != map) {
		it++;
	}
	if (it == maps.end())
		return "";

	return it->first;
}

void MapManager::startMaps() {
	std::string getName(MapPtr map);
	std::map<std::string, MapPtr>::iterator it = maps.begin();
	while (it != maps.end()) {
		try {
			it->second->start();
		} catch (std::exception& exc) {
//			logger->log(LOG_ERROR, "Exception thrown when starting map %s.\n\t%s\n",
//					it->first.c_str(), exc.what());
		}

		it++;
	}
}

void MapManager::stopMaps() {
	std::map<std::string, MapPtr>::iterator it = maps.begin();
	while (it != maps.end()) {
//		logger->log(LOG_PROGRESS, "Stopping map %s.\n", it->second->getName().c_str());
		try {
			it->second->stop();
		} catch (std::exception& exc) {
			ERROR("Exception thrown when stopping map %s.\n\t%s\n",
					it->first.c_str(), exc.what());
		}
//		logger->log(LOG_PROGRESS, "Stopped map %s.\n", it->second->getName().c_str());

		it++;
	}
	maps.clear();
}

std::vector<MapFactoryFunc> mapFactories;

MapPtr MapFactory::createMap(ConfigElementPtr config) {
	MapPtr rval;
	for (int i = mapFactories.size()-1; rval == NULL && i >= 0; --i) {
		rval = mapFactories[i](config);
	}
	return rval;
}

void MapFactory::addFactory(MapFactoryFunc factory) {
	mapFactories.push_back(factory);
}

} // namespace crosbot
