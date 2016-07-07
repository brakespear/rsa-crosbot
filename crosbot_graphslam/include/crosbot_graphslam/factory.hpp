/*
 * factory.hpp
 *
 *  Created on: 07/07/2016
 *      Author: rescue
 */

#ifndef CROSBOT_GRAPHSLAM_FACTORY_HPP_
#define CROSBOT_GRAPHSLAM_FACTORY_HPP_


// Due to include orders, only specify existence of GraphSlam class
class GraphSlam;

/**
 * Simple factory for creating GraphSlam entities,
 *     typically as either the CPU or GPU implementations.
 * Factories should implement this method, returning a pointer to the GraphSlam object
 */
class FactoryGraphSlam {
public:
   virtual GraphSlam *makeGraphSlam() = 0;
};

#endif /* CROSBOT_GRAPHSLAM_FACTORY_HPP_ */
