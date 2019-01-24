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
#include "controller_t.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>

/****************************************
 * Main
 ****************************************/

int main()
{
    DirectionalController controller;

    controller.Start();

    lcm::LCM lcm;

    if(!lcm.good()) return 1;

    exlcm::controller_t msg;
    msg.enabled = true;
    msg.name = "controller";

    while(controller.IsRunning())
    {
        msg.speedLevel = controller.GetSpeedLevel();
        msg.turningRate = controller.GetTurningRate();
        msg.velocity = controller.GetVelocity();

        lcm.publish(ControllerChannel, &msg);

        controller.ResetTurningRate();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    msg.enabled = false;
}
