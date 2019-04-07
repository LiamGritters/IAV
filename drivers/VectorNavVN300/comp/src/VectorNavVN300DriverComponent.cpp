/*
 * VectorNavVN300DriverComponent.cpp
 *
 *  Created on: 2019-01-15
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "VectorNavVN300Driver.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

/****************************************
 * Main
 ****************************************/

int main()
{
    VectorNavVN300Driver driver;

    driver.Initialize();
    driver.SetDataFrequency(10);
    driver.Start();

    while(driver.IsRunning())
    {
        const auto gps = driver.GetGPSData();
        const auto imu = driver.GetIMUData();

        double time = driver.GetTimestamp();

        float imuXAcceleration = imu.xAcceleration;

        float a = driver._gpsAccelerationECEF[0];


        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    driver.Stop();
}



