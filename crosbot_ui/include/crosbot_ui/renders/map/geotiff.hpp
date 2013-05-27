/*
 * geotiff.h
 *
 *  Created on: 23/09/2009
 *      Author: rescue
 */

#ifndef CROSBOT_GEOTIFF_HPP_
#define CROSBOT_GEOTIFF_HPP_

#include <crosbot_ui/opengl.hpp>

namespace crosbot {
using namespace gui;

class GeoTIFFConstants {
public:
	
	/**
	 * The colours of the text/scale/orientation markers.
	 */
	static Colour4f textColour;
	static Colour4f scaleColour;
	static Colour4f orientationColour;
	
	/**
	 * The colours/size of the unexplored checker board pattern.
	 */
	static Colour4f unexploredLight;
	static Colour4f unexploredDark;
	static float unexploredCheckerBoardSize;	// In meters
	
	/**
	 * Explored grid colour/size.
	 */
	static Colour4f exploredGrid;
	static float exploredGridSize;	// In meters
	
	/**
	 * Mapped area colours.
	 */
	static Colour4f wallAndObstacleColour;
	static Colour4f searchedAreaLowConfidence;
	static Colour4f searchedAreaHighConfidence;
	static Colour4f clearedAreaLowConfidence;
	static Colour4f clearedAreaHighConfidence;
	static int maxClearViews;
	
	/**
	 * Detected item markers.
	 */
	static Colour4f markerTextColour;
	static Colour4f victimColour;
	static Colour4f victimUnconfirmedColour;
	static float victimMarkerDiameter; // In meters
	static Colour4f hazardColour;
	static float hazardMarkerSide; // In meters
	
	/**
	 * Robot path colours.
	 */
	static Colour4f initialPositionColour;
	static float robotPositionSize;
	static Colour4f robotPathColour;
	static float robotPathThickness; // In meters
};

} // namespace crosbot

#endif /* CROSBOT_GEOTIFF_HPP_ */
