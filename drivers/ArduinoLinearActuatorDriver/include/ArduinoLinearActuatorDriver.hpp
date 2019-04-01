/*
 * ArduinoLinearActuactorDriver.hpp
 *
 *  Created on: 2019-03-29
 *      Author: liam
 */

#ifndef DRIVERS_ARDUINOLINEARACTUATORDRIVER_INCLUDE_ARDUINOLINEARACTUATORDRIVER_HPP_
#define DRIVERS_ARDUINOLINEARACTUATORDRIVER_INCLUDE_ARDUINOLINEARACTUATORDRIVER_HPP_


/****************************************
 * INCLUDES
 ****************************************/

#include <stdio.h>
#include <fcntl.h>
#include <termios.h>
#include <thread>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class ArduinoLinearActuatorDriver
{

public:

    ArduinoLinearActuatorDriver();
    ~ArduinoLinearActuatorDriver();

    bool Initialize(std::string deviceFileName);

    void Start();
    void Stop();
    void SendData(std::string msg);

    inline bool IsRunning() const {return _isRunning;};

    float GetActuactorPosition() const {return _position;};


private:

    static void listenThread(ArduinoLinearActuatorDriver *driver);

private:

    bool _isRunning;

    double _timestamp;

    int _fd;

    termios _tty;

    std::thread _listenThread;

    float _position;
};



#endif /* DRIVERS_ARDUINOLINEARACTUATORDRIVER_INCLUDE_ARDUINOLINEARACTUATORDRIVER_HPP_ */
