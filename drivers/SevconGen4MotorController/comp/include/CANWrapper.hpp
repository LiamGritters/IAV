/*
 * CANWrapper.hpp
 *
 *  Created on: 2019-03-07
 *      Author: liam
 */

#ifndef DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANWRAPPER_HPP_
#define DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANWRAPPER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "can4linux.h"

#include <thread>
#include <string>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr int BufferSize = 100;
constexpr int TransmitBuffer = 256;

/****************************************
 * CLASS DEFINITION
 ****************************************/

class CANWrapper
{

public:
    CANWrapper();
    ~CANWrapper();

    bool Initialize(std::string canDeviceName);

    void Start();
    void Stop();
    bool IsRunning() const {return _isRunning;};

    void SendMessage();

private:

    static void listenThread(CANWrapper* wrapper);

public:

    bool _isRunning;

    std::string _deviceName;

    int _fd; //field descriptor

    canmsg_t _rx[BufferSize]; //receive can message
    canmsg_t _tx[TransmitBuffer]; //transmit can message

    std::thread _listenThread;

};

#endif /* DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANWRAPPER_HPP_ */
