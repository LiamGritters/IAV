/*
 * DisplayIMU.cpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "IMU.hpp"
#include "SensorManager.hpp"

#include <lcm/lcm-cpp.hpp>
#include <chrono>
#include <iostream>

/****************************************
 * Main
 ****************************************/

int main()
{
    SensorManager imu;
    if(!imu.Initialize(IMU_FLAG))
    {
        std::cout<<"[ERROR]: could not initialize lidar manager"<<std::endl;
        return -1;
    }

    imu.Start();


    while(imu.IsRunning())
    {
        imu.Update();

        auto imuData = imu.GetIMU();

        std::cout<<"roll: "<<imuData.roll<<", pitch : "<<imuData.pitch<<", yaw: "<<imuData.yaw<<std::endl;
        std::cout<<"Acceleration-> x: "<<imuData.xAcceleration<<", y: "<<imuData.yAcceleration<<", z: "<<imuData.zAcceleration<<std::endl;
    }

    imu.Stop();
}


