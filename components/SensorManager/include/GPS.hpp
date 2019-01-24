/*
 * GPS.hpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

#ifndef COMPONENTS_SENSORMANAGER_INCLUDE_GPS_HPP_
#define COMPONENTS_SENSORMANAGER_INCLUDE_GPS_HPP_

/****************************************
 * INCLUDES
 ****************************************/

/****************************************
 * ENUM DEFINITION
 ****************************************/

enum GPSFix
{
    NO_FIX = 0,
    STANDARD = 1,
    DIFFERENTIAL = 2
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class GPS
{

public:

    GPS()
    {
        northing = 0.0;
        easting = 0.0;
        latitude = 0.0;
        longitude = 0.0;
        altitude = 0.0;
        speed = 0.0;
        heading = 0.0;
        positionError = 0.0;
        speedError = 0.0;
        numSatellites = 0;
        gpsFix = NO_FIX;
    }

public:

    double northing;
    double easting;

    double latitude;
    double longitude;
    double altitude;

    float speed;
    float heading;

    GPSFix gpsFix;
    int numSatellites;

    float positionError;
    float speedError;
};



#endif /* COMPONENTS_SENSORMANAGER_INCLUDE_GPS_HPP_ */
