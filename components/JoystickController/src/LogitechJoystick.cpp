/*
 * LogitechJoystick.cpp
 *
 *  Created on: 2019-03-31
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "LogitechJoystick.hpp"

#include <chrono>
#include <iostream>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr float  MaxSpeed =3;//max speed in m/s -- velocity controls
constexpr float  MaxAngle = 15;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

LogitechJoystick::LogitechJoystick()
{
    this->_velocity = 0.0;
    this->_turningRate = 0.0;
    this->_isRunning = false;
}

LogitechJoystick::~LogitechJoystick()
{
    Stop();
}

bool
LogitechJoystick::Initialize()
{
    return _joystick.Initialize();
}

void
LogitechJoystick::Start()
{
    this->_isRunning = true;
    this->_thread = std::thread(LogitechJoystick::controllerThread, this);
}

void
LogitechJoystick::Stop()
{
    this->_isRunning = false;
    if(_thread.joinable())
    {
        _thread.join();
    }
}

void
LogitechJoystick::controllerThread(LogitechJoystick *controller)
{
    while(controller->_isRunning)
    {
        auto& id = controller->_joystick.GetIDs();
        if (id.size() <= 0) continue;
        controller->getJoystickValues(id[0]);
    }
    controller->_isRunning = false;
}

void
LogitechJoystick::getJoystickValues(int id)
{
    int x = 0, y = 0, z = 0;

    bool one = false;

    if (!_joystick.GetX(id, x))
        x = 0;
    if (!_joystick.GetY(id, y))
        y = 0;
    if (!_joystick.GetZRot(id, z))
        z = 0;
    if (!_joystick.GetButton(id,JoystickLibrary::Extreme3DProButton::Trigger, one))
       one = false;

    if(!one)
    {
        x=0;
        y=0;
        z=0;
    }

    const float factorv = 100.0/MaxSpeed;
    _velocity= y/factorv;

    const float factora = 100/MaxAngle;
    _turningRate= x/factora;
    std::cout << "vel: "<<_velocity<<", angle: "<<_turningRate <<   std::endl;
}





