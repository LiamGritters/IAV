/*
 * SevconGen4MotorControllerDriver.hpp
 *
 *  Created on: 2019-02-25
 *      Author: liam
 */

#ifndef SEVCONGEN4MOTORCONTROLLERDRIVER_HPP_
#define SEVCONGEN4MOTORCONTROLLERDRIVER_HPP_


/****************************************
 * INCLUDES
 ****************************************/


/****************************************
 * CLASS DEFINITION
 ****************************************/

class SevconGen4Driver
{

public:

    SevconGen4Driver();
    ~SevconGen4Driver();

    bool Initialize();

    void Start();
    void Stop();

    inline bool IsRunning() const {return _isRunning;};


private:


private:

    bool _isRunning;

    double _timestamp;

};


#endif /* SEVCONGEN4MOTORCONTROLLERDRIVER_HPP_ */
