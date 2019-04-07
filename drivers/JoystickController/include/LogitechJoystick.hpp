/*
 * LogitechJoystick.hpp
 *
 *  Created on: 2019-03-31
 *      Author: liam
 */

#ifndef COMPONENTS_JOYSTICKCONTROLLER_INCLUDE_LOGITECHJOYSTICK_HPP_
#define COMPONENTS_JOYSTICKCONTROLLER_INCLUDE_LOGITECHJOYSTICK_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "Extreme3DProService.hpp"

#include <thread>
#include <atomic>
#include <mutex>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class LogitechJoystick
{

public:

    LogitechJoystick();
    ~LogitechJoystick();

    bool Initialize();

    void Start();

    void Stop();

    void Update();

    inline bool IsRunning() const {return _isRunning;};

    inline float GetTurningRate() const {return _turningRate;};
    inline float GetVelocity() const {return _velocity;};

private:

    void getJoystickValues(int id);

    static void controllerThread(LogitechJoystick *controller);

private:

    bool _isRunning;

    float _velocity;
    float _turningRate;

    JoystickLibrary::Extreme3DProService& _joystick = JoystickLibrary::Extreme3DProService::GetInstance();

    std::thread _thread;
};



#endif /* COMPONENTS_JOYSTICKCONTROLLER_INCLUDE_LOGITECHJOYSTICK_HPP_ */
