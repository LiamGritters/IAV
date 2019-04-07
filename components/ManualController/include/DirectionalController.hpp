/*
 * DirectionalController.hpp
 *
 *  Created on: 2018-11-08
 *      Author: liam
 */

#ifndef DIRECTIONALCONTROLLER_HPP_
#define DIRECTIONALCONTROLLER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <thread>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class DirectionalController
{

public:

    DirectionalController();
    ~DirectionalController();

    void Initialize();

    void Start();

    void Stop();

    inline bool IsRunning() const {return _isRunning;};

    void ResetTurningRate() {_turningRate = 0;};

    inline float GetTurningRate() const {return _turningRate;};
    inline int GetSpeedLevel() const {return _speedLevel;};

private:

    static void controllerThread(DirectionalController *controller);

private:

    bool _isRunning;

    float _turningRate;

    int _speedLevel; // 1 to 9, with 9 being the fastest

    std::thread _thread;
};


#endif /* DIRECTIONALCONTROLLER_HPP_ */
