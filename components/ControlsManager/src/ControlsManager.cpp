/*
 * ControlsManager.cpp
 *
 *  Created on: 2019-04-02
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "ControlsManager.hpp"
#include "LCMChannels.hpp"

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

ControlsManager::ControlsManager()
{
    this->_timestamp = 0.0;
    this->_velocity = 0.0;
    this->_turningAngle = 0.0;
    this->_msgEnabled = false;
    this->_isRunning = false;
}

ControlsManager::~ControlsManager()
{
    Stop();
}

bool
ControlsManager::Initialize()
{
    if(!_lcm.good()) return false;
    _lcm.subscribe(ControllerChannel, &ControlsManager::handleControlInputsMessage, this);
    return true;
}

void ControlsManager::Start()
{
    this->_isRunning = true;

    this->_thread = std::thread(ControlsManager::readMessage, this);
}

void
ControlsManager::readMessage(ControlsManager* controls)
{
    while(controls->_isRunning)
    {
        controls->_lcm.handle();

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void
ControlsManager::Stop()
{
    this->_isRunning = false;
    if(_thread.joinable())
    {
        _thread.join();
    }
}

void
ControlsManager::handleControlInputsMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const iav_lcm::vehicle_controller_demand_t* msg)
{
    _msgEnabled = msg->enabled;
    _velocity = msg->velocity;
    _turningAngle = msg->turningAngle;
    _timestamp = msg->timestamp;
}
