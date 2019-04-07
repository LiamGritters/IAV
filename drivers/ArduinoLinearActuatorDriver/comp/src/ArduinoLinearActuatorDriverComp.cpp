/*
 * ArduinoLinearActuatorDriverComp.cpp
 *
 *  Created on: 2019-03-29
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "ArduinoLinearActuatorDriver.hpp"

#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string DeviceFileName = "/dev/ttyACM0";
//const std::string DeviceFileName = "/dev/ttyUSB0";

/****************************************
 * Main
 ****************************************/

int main(int argc, char* argv[])
{
    if((argc != 2)) return -1;

    ArduinoLinearActuatorDriver driver;

    if(!driver.Initialize(DeviceFileName)) return -1;
    driver.Start();

    float value  = atof(argv[1]);

    while(driver.IsRunning())
    {
        std::string msg = "<act," + std::to_string(value) + ">";
        driver.SendData(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }

    driver.Stop();
}



