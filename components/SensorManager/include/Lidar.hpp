/*
 * Lidar.hpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

#ifndef COMPONENTS_SENSORMANAGER_INCLUDE_LIDAR_HPP_
#define COMPONENTS_SENSORMANAGER_INCLUDE_LIDAR_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <vector>

/****************************************
 * STRUCT DEFINITION
 ****************************************/

struct Scanline
{
    Scanline() : layer(0), angle(0.0) {};

    int layer;

    float angle; //radians

    std::vector<float> ranges;
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class Lidar
{

public:

    Lidar()
    {
        numLayers = 0;
        numRanges = 0;
        horizontalRes = 0.0;
        verticalRes = 0.0;
        horizontalFOV = 0.0;
        verticalFOV = 0.0;
    }

    Scanline GetCenterScanline() const
    {
        return scanlines[8]; // close to center scanline
    }

public:

    int numLayers;
    int numRanges;

    float horizontalRes; //Degrees between each range
    float verticalRes; //Degrees
    float horizontalFOV; //Degrees
    float verticalFOV; //Degrees

    Scanline collapsedScanline;

    std::vector<Scanline> scanlines;
};



#endif /* COMPONENTS_SENSORMANAGER_INCLUDE_LIDAR_HPP_ */
