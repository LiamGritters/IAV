/*
 * SevconGen4MotorControllerDriver.cpp
 *
 *  Created on: 2019-02-25
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "SevconGen4MotorControllerDriver.hpp"

#include <iostream>
#include <string>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

SevconGen4Driver::SevconGen4Driver()
{
    this->_isRunning = false;
    this->_timestamp = 0.0;
}

SevconGen4Driver::~SevconGen4Driver()
{
    if(_isRunning)
    {
        Stop();
    }
}

bool
SevconGen4Driver::Initialize()
{


    return true;
}

void
SevconGen4Driver::Start()
{
    this->_isRunning = true;

}

void
SevconGen4Driver::Stop()
{
    this->_isRunning = false;

}





