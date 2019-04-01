/*
 * JoystickControllerMain.cpp
 *
 *  Created on: 2019-03-30
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "LCMChannels.hpp"
#include "vehicle_controller_demand_t.hpp"
#include "LogitechJoystick.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * Main
 ****************************************/

int main()
{

    LogitechJoystick joystick;
    if(!joystick.Initialize())
    {
        std::cout<<"[ERROR]: Could not initialize the joystick object"<<std::endl;
        return -1;
    }

    joystick.Start();

    lcm::LCM lcm;

    if(!lcm.good()) return 1;

    iav_lcm::vehicle_controller_demand_t msg;
    msg.enabled = true;
    msg.name = "controller";

    while(joystick.IsRunning())
    {
        msg.timestamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
        msg.turningAngle = joystick.GetTurningRate();
        msg.velocity = joystick.GetVelocity();

        lcm.publish(ControllerChannel, &msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    msg.enabled = false;
}



