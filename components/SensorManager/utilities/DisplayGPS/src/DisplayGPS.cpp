/*
 * DisplayGPS.cpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "GPS.hpp"
#include "SensorManager.hpp"

#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include <iostream>

/****************************************
 * Main
 ****************************************/

int main()
{
    SensorManager gps;
    if(!gps.Initialize(GPS_FLAG))
    {
        std::cout<<"[ERROR]: could not initialize lidar manager"<<std::endl;
        return -1;
    }

    gps.Start();


    while(gps.IsRunning())
    {
        gps.Update();

        auto gpsData = gps.GetGPS();

        std::cout<<"latitude: "<<gpsData.latitude<<", longitude: "<<gpsData.longitude<<", altitude: "<<gpsData.altitude<<std::endl;
        std::cout<<"northing: "<<gpsData.northing<<", easting: "<<gpsData.easting<<", speed: "<<gpsData.speed<<", heading: "<<gpsData.heading<<std::endl;
    }

    gps.Stop();
}




