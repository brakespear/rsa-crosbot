/*
 * module.cpp
 *
 *  Created on: 20/02/2013
 *      Author: mmcgill
 */

#include <crosbot_fastslam/module.hpp>

namespace crosbot {

namespace fastslam {

class FastSLAMFactory {
public:
	static MapPtr factory(ConfigElementPtr config) {
		std::string type = config->getParam(PARAM_TYPE, config->name);
		std::string name = config->getParam(PARAM_NAME, "");

		if (strcasecmp(type.c_str(), "fastslam") == 0) {
			ros::NodeHandle nh;
			FastSLAMModulePtr rval = new FastSLAMModule(name);
			rval->initialize(nh);
			rval->configure(config);
			return rval;
		}
		return NULL;
	}

	FastSLAMFactory() {
		MapFactory::addFactory(&factory);
	}
};

FastSLAMFactory fastSLAMFactory;

} // namespace fastslam

} // namespace crosbot
