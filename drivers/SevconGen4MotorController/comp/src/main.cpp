/*
 * main.cpp
 *
 *  Created on: 2019-02-28
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "CANWrapper.hpp"
#include "PIDController.hpp"

#include <string>
#include <iostream>

#include <vector>

#include <ios>
#include <sstream>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string DeviceName = "can0";

/****************************************
 * Main
 ****************************************/


int main(int argc, char **argv)
{
    CANWrapper can;

    if(!can.Initialize(DeviceName)) return -1;

    can.Start();

    PIDController controller;

    controller.Initialize(0.4, 0, 0, 100, -100, 0.1);

    double processVal = can.GetAngularVelocity();
    double setPoint = 200.0;

    for(int i = 0; i < 100; ++i)
    {
        processVal = can.GetAngularVelocity();
        std::cout<<"process val: "<<processVal<<std::endl;
        double inc = controller.Calculate(setPoint, processVal);
        processVal += inc;
        std::cout<<"inc + val: "<<processVal<<std::endl;
        can.SendSpeed((int)processVal);

        usleep(400000); //400 milliseconds
    }

    usleep(400000);
    can.SendSpeed(0);
}









