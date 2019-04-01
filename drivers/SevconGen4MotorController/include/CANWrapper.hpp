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

#include "CANMessages.hpp"

#include <thread>
#include <string>
#include <vector>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <net/if.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>

#include <linux/can.h>
#include <linux/can/raw.h>

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

    void SendMessage(IAV::CANmsg msg);
    void SendSpeed(int speed);

    IAV::CANstate GetMotorControllerState() const {return _state;};

    int ReadAngularVelocity();

private:

    static void listenThread(CANWrapper* wrapper);

    void checkOperationState();
    void setBridge(bool flag);
    void setMaxTorque(uint16_t torque);

    void checkReadMsg();

    void sevconLogin();
    void forceToOP();
    void initializeMotorController();

    std::vector<uint8_t> convertToHex(std::string string);
    std::vector<uint8_t> convertToHex(int value);
    int convertHextoInt(std::vector<uint8_t> hexVector);

    void initializeOperationState(); //only works in slave mode
    void setContactor(bool flag); //only works in slave mode

public:

    bool _isRunning;

    std::string _deviceName;

    int _fd; //field descriptor

    IAV::CANmsg _rx; //[BufferSize];

    std::thread _listenThread;

    IAV::CANstate _state;

    sockaddr_can _addr;
    ifreq _ifr;

    int64_t _timeout;
};

#endif /* DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANWRAPPER_HPP_ */
