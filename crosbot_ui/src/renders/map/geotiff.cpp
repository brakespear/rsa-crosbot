/*
 * geotiff.cpp
 *
 *  Created on: 23/09/2009
 *      Author: rescue
 */

#include <crosbot_ui/renders/map/geotiff.hpp>

namespace crosbot {

/**
 * The colours of the text/scale/orientation markers.
 */
Colour4f GeoTIFFConstants::textColour					= Colour4f(0, 44, 207);
Colour4f GeoTIFFConstants::scaleColour					= Colour4f(0, 50, 140);
Colour4f GeoTIFFConstants::orientationColour			= Colour4f(0, 50, 140);

/**
 * The colours/size of the unexplored checker board pattern.
 */
Colour4f GeoTIFFConstants::unexploredLight				= Colour4f(237, 237, 238);
Colour4f GeoTIFFConstants::unexploredDark				= Colour4f(226, 226, 227);
float GeoTIFFConstants::unexploredCheckerBoardSize		= 1.0;	// In meters

/**
 * Explored grid colour/size.
 */
Colour4f GeoTIFFConstants::exploredGrid					= Colour4f(190, 190, 191);
float GeoTIFFConstants::exploredGridSize				= 0.5;	// In meters

/**
 * Mapped area colours.
 */
Colour4f GeoTIFFConstants::wallAndObstacleColour		= Colour4f(0, 40, 120);
Colour4f GeoTIFFConstants::searchedAreaLowConfidence	= Colour4f(128, 128, 128);
Colour4f GeoTIFFConstants::searchedAreaHighConfidence	= Colour4f(255, 255, 255);
//Made low confidence a lighter shade of green
//Colour4f GeoTIFFConstants::clearedAreaLowConfidence		= Colour4f(180, 230, 180);
Colour4f GeoTIFFConstants::clearedAreaLowConfidence		= Colour4f(210, 230, 210);
Colour4f GeoTIFFConstants::clearedAreaHighConfidence	= Colour4f(130, 230, 130);
int GeoTIFFConstants::maxClearViews						= 50;

/**
 * Detected item markers.
 */
Colour4f GeoTIFFConstants::markerTextColour				= Colour4f(255, 255, 255);
Colour4f GeoTIFFConstants::victimColour					= Colour4f(240, 10, 10);
Colour4f GeoTIFFConstants::victimUnconfirmedColour		= Colour4f(240, 100, 10);
float GeoTIFFConstants::victimMarkerDiameter			= 0.35; // In meters
Colour4f GeoTIFFConstants::hazardColour					= Colour4f(255, 100, 30);
float GeoTIFFConstants::hazardMarkerSide				= 0.30; // In meters

/**
 * Robot path colours.
 */
Colour4f GeoTIFFConstants::initialPositionColour		= Colour4f(255, 200, 0);
float GeoTIFFConstants::robotPositionSize				= 0.2; // In meters
Colour4f GeoTIFFConstants::robotPathColour				= Colour4f(120, 0, 140);
float GeoTIFFConstants::robotPathThickness				= 0.02; // In meters

} // namespace crosbot
