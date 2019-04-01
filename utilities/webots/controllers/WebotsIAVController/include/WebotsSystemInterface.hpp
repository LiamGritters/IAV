/*
 * WebotsSystemInterface.hpp
 *
 *  Created on: 2018-10-06
 *      Author: liam
 */

#ifndef WEBOTSSYSTEMINTERFACE_HPP_
#define WEBOTSSYSTEMINTERFACE_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsSensorManager.hpp"
#include "WebotsControlsManager.hpp"

#include "vehicle_controller_demand_t.hpp"

#include <lcm/lcm-cpp.hpp>

#include <webots/Robot.hpp>
#include <mutex>
#include <thread>
#include <atomic>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class WebotsSystemInterface
{

public:

    WebotsSystemInterface();
    ~WebotsSystemInterface();

    bool Initialize();
    void Start();
    void Stop();

private:

    static void robotProcess(WebotsSystemInterface *sysInterface);

    bool setupNetwork();

    void handleMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const iav_lcm::vehicle_controller_demand_t *msg);

private:

    std::atomic<bool> _isRunning;

    int _timeStep;
    int _webotsFlag;

    float _rightSpeed;
    float _leftSpeed;

    webots::Robot* _robot;

    std::thread _robotProcessThread;
    std::thread _publishThread;

    WebotsSensorManager _sensors;
    WebotsControlsManager _controls;

    lcm::LCM _lcm;
};



#endif /* WEBOTSSYSTEMINTERFACE_HPP_ */
