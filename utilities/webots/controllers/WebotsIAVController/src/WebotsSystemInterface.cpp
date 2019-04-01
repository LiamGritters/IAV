/*
 * WebotsSystemInterface.cpp
 *
 *  Created on: 2018-10-06
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsSystemInterface.hpp"
#include "WebotsPublishMessages.hpp"

#include "LCMChannels.hpp"

#include <iostream>
#include <chrono>

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

WebotsSystemInterface::WebotsSystemInterface()
{
    this->_timeStep = 0;
    this->_rightSpeed = 0;
    this->_leftSpeed = 0;
    this->_isRunning = false;
    this->_robot = nullptr;
    this->_webotsFlag = 0;
}

WebotsSystemInterface::~WebotsSystemInterface()
{
    Stop();
    if(_robot)
    {
        delete _robot;
    }
}

bool
WebotsSystemInterface::Initialize()
{
    this->_webotsFlag = 0;
    this->_robot = new webots::Robot();
    this->_timeStep = _robot->getBasicTimeStep();

    this->_sensors.Initialize(_robot);
    this->_controls.Initialize(_robot);

    return setupNetwork();
}

void
WebotsSystemInterface::Start()
{
    this->_isRunning = true;

    this->_robotProcessThread = std::thread(WebotsSystemInterface::robotProcess, this);
    this->_publishThread = std::thread(PublishData, &_lcm, &_sensors, std::ref(this->_isRunning));

    while(_webotsFlag != -1)
    {
        _lcm.handle();

        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void
WebotsSystemInterface::Stop()
{
    _isRunning = false;
    if(_robotProcessThread.joinable())
    {
        _robotProcessThread.join();
    }
    if(_publishThread.joinable())
    {
        _publishThread.join();
    }
}

bool
WebotsSystemInterface::setupNetwork()
{
    if(_lcm.good())
    {
        _lcm.subscribe(ControllerChannel, &WebotsSystemInterface::handleMessage, this);
        return true;
    }
    return false;
}

void
WebotsSystemInterface::robotProcess(WebotsSystemInterface *sysInterface)
{
    while(sysInterface->_isRunning)
    {
        sysInterface->_webotsFlag = sysInterface->_robot->step(sysInterface->_timeStep);

        sysInterface->_sensors.PopulateData();
        sysInterface->_controls.PopulateData();

        sysInterface->_controls.SetLeftMotorVelocity(sysInterface->_leftSpeed);
        sysInterface->_controls.SetRightMotorVelocity(sysInterface->_rightSpeed);

        std::cout<<"left speed: "<<sysInterface->_leftSpeed<<", right speed: "<<sysInterface->_rightSpeed<<std::endl;
    }
}

void
WebotsSystemInterface::handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const iav_lcm::vehicle_controller_demand_t *msg)
{
    this->_leftSpeed = (msg->velocity) + (msg->turningAngle);
    this->_rightSpeed = (msg->velocity) - (msg->turningAngle);
}

