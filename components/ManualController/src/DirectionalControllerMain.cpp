/*
 * DirectionalControllerMain.cpp
 *
 *  Created on: 2018-11-08
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "DirectionalController.hpp"

#include "LCMChannels.hpp"
#include "vehicle_controller_demand_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr float DefaultVelocityRate = 1.0;

/****************************************
 * Main
 ****************************************/

int main()
{
    DirectionalController controller;

    controller.Start();

    lcm::LCM lcm;

    if(!lcm.good()) return 1;

    iav_lcm::vehicle_controller_demand_t msg;
    msg.enabled = true;
    msg.name = "controller";

    while(controller.IsRunning())
    {
        msg.turningAngle = controller.GetTurningRate(); //*DefaultVelocityRate;
        msg.velocity = controller.GetSpeedLevel()*DefaultVelocityRate;

        lcm.publish(ControllerChannel, &msg);

        controller.ResetTurningRate();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    msg.enabled = false;
}
