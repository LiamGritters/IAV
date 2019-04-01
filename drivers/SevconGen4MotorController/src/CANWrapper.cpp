/*
 * CANWrapper.cpp
 *
 *  Created on: 2019-03-07
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "CANWrapper.hpp"

#include <iostream>
#include <ios>
#include <sstream>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr int SleepTime = 100;
constexpr uint16_t MaxTorque = 1000;
constexpr uint64_t MaxTimeout = 1000;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

CANWrapper::CANWrapper()
{
    this->_fd = -1;
    this->_isRunning = false;
    this->_state = IAV::STOPPED;
    this->_timeout = 0.0;
}

CANWrapper::~CANWrapper()
{
    Stop();
}

bool
CANWrapper::Initialize(std::string canDeviceName)
{
    if(canDeviceName.empty()) return false;

    _fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if( _fd < 0)
    {
        std::cout<<"[ERROR]: Cannot Open Socket"<<std::endl;
        return false;
    }

    strcpy(_ifr.ifr_name, canDeviceName.c_str());
    ioctl(_fd, SIOCGIFINDEX, &_ifr);

    _addr.can_family  = AF_CAN;
    _addr.can_ifindex = _ifr.ifr_ifindex;

    printf("%s at index %d\n", canDeviceName.c_str(), _ifr.ifr_ifindex);

    if(bind(_fd, (struct sockaddr *)&_addr, sizeof(_addr)) < 0) {
        perror("Error in socket bind");
        return false;
    }

    usleep(10000);

    initializeMotorController();

    return true;
}

void
CANWrapper::initializeMotorController()
{
    sevconLogin();

    checkOperationState();
    while(_state != IAV::OP && _isRunning)
    {
        if (_state == IAV::PREOP)
        {
            forceToOP();
            std::cout<<"attempting to initialize operation state"<<std::endl;
            break;
        }
        usleep(1000);
        checkOperationState();
    }

    usleep(100000);
    setBridge(true);
    setMaxTorque(MaxTorque);
    usleep(10000);

    _timeout = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

void
CANWrapper::Stop()
{
    this->_isRunning = false;

    if(_listenThread.joinable())
    {
        _listenThread.join();
    }

    if(_fd >= 0)
    {
        close(_fd);
    }
}

void
CANWrapper::Start()
{
    this->_isRunning = true;

    this->_listenThread = std::thread(CANWrapper::listenThread, this);
}

void
CANWrapper::listenThread(CANWrapper* wrapper)
{
    int got;
    while(wrapper->_isRunning)
    {
        got = read(wrapper->_fd, &wrapper->_rx, sizeof(IAV::CANmsg));

        if(got > 0)
        {
        int j;
            printf("Received with ret=%d: id=%u/0x%08x \tlen=%d", got, wrapper->_rx.identity, wrapper->_rx.identity, wrapper->_rx.data_length);

            if(wrapper->_rx.data_length > 0 )
            {
                int length = wrapper->_rx.data_length;

                for(j = 0; j < length; j++)
                {
                    printf(" %02x", wrapper->_rx.data[j]);
                }

                std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
                if(wrapper->_rx.identity == IAV::HRBT)
                {
                    // if _state was previously off, must re-initialize motor controller
                    if(wrapper->_state == IAV::OFF) wrapper->initializeMotorController();
                    wrapper->_timeout = (int64_t)ms.count();
                }
                else
                {
                    if((uint64_t)((int64_t)ms.count() - wrapper->_timeout) > MaxTimeout)
                    {
                        wrapper->_state = IAV::OFF;
                    }
                }
            }
            printf("\n"); fflush(stdout);
        }
        else
        {
            printf("Received with ret=%d\n", got);
            fflush(stdout);
        }

        usleep(SleepTime);
    }
}

void
CANWrapper::SendMessage(IAV::CANmsg msg)
{
    const int sent = write(_fd, &msg, sizeof(can_frame));

    if (sent <= 0)
    {
        perror("sent");
        Stop();
    }

}

void
CANWrapper::initializeOperationState()
{
    IAV::CANmsg msg;

    msg.identity = IAV::NMT;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 2; //number of bytes
    msg.data[0] = 0x01;
    msg.data[1] = 0x00;

    SendMessage(msg);
}

int
CANWrapper::ReadAngularVelocity()
{
    IAV::CANmsg msg;
    msg.identity = IAV::RSDO;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8; //number of bytes, always 8 for SDO

    msg.data[0] = IAV::READ_REQUEST;
    msg.data[1] = IAV::ANGULAR_VELOCITY & 0xFF;
    msg.data[2] = IAV::ANGULAR_VELOCITY >> 8;
    msg.data[3] = IAV::ANGULAR_VELOCITY_SUBINDEX;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    IAV::CANmsg rx;
    int got = read(_fd, &rx, sizeof(IAV::CANmsg));
    if((got > 0) && (rx.data_length == 8))
    {
        if((rx.identity == IAV::TSDO) && (msg.data[1] == (IAV::ANGULAR_VELOCITY & 0xFF)) && (msg.data[2] == (IAV::ANGULAR_VELOCITY >> 8)))
        {
            std::vector<uint8_t> data;
            data.assign(rx.data+4, rx.data+8);
            return convertHextoInt(data);
        }
    }
    return -1;
}

int
CANWrapper::convertHextoInt(std::vector<uint8_t> hexVector)
{
    int value = 0;
    const int hexSize = hexVector.size();
    for(int i = 0; i < hexSize; ++i)
    {
        value += hexVector[i] << 8*i;
    }
    return value;
}

void
CANWrapper::forceToOP()
{
    IAV::CANmsg msg;
    msg.identity = IAV::RSDO;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8; //number of bytes, always 8 for SDO

    msg.data[0] = IAV::WRITE_REQUEST_1BYTE;
    msg.data[1] = IAV::STATE_MASTER & 0xFF;
    msg.data[2] = IAV::STATE_MASTER >> 8;
    msg.data[3] = IAV::STATE_MASTER_SUBINDEX;
    msg.data[4] = IAV::OP_MASTER;
    msg.data[5] = 0xCD; //filler value, unused
    msg.data[6] = 0xEF; //filler value, unused
    msg.data[7] = 0xFA; //filler value, unused
    SendMessage(msg);
    checkReadMsg();
    usleep(40000);

    IAV::CANmsg msg2;
    msg2.bufferFull = false;
    msg2.syncFlag = false;
    msg2.identity = IAV::RPDO_FAKEOUT;
    msg2.data_length = 3;
    msg2.data[0] = 0x00;
    msg2.data[1] = 0x00;
    msg2.data[2] = 0x00;

    for(int i = 0; i < 5; ++i)
    {
        SendMessage(msg2);
        usleep(40000);
    }
}

void
CANWrapper::checkOperationState()
{

    int got = read(_fd, &_rx, sizeof(can_frame));

    if(got > 0)
    {
        if(_rx.identity == IAV::HRBT)
        {
            printf("Received with ret=%d: id=%u/0x%08x ", got, _rx.identity, _rx.identity);

            printf("\tlen=%d", _rx.data_length);

            if(_rx.data_length > 0 )
            {
                int length = _rx.data_length;

                for(int j = 0; j < length; j++)
                {
                    printf(" %02x", _rx.data[j]);
                }
            }
            printf("\n"); fflush(stdout);

            switch(_rx.data[0])
            {
                case(IAV::INIT):
                    _state = IAV::INIT;
                    break;
                case(IAV::STOPPED):
                    _state = IAV::STOPPED;
                    break;
                case(IAV::PREOP):
                    _state = IAV::PREOP;
                    break;
                case(IAV::OP):
                    _state = IAV::OP;
                    break;
            }
        }
    }
}

void
CANWrapper::setContactor(bool flag)
{
    IAV::CANmsg msg;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8;
    msg.identity = IAV::RSDO;

    msg.data[0] = IAV::WRITE_REQUEST_2BYTES;
    msg.data[1] = IAV::CONTACTOR & 0xFF;
    msg.data[2] = IAV::CONTACTOR >> 8;
    msg.data[3] = IAV::CONTACTOR_SUBINDEX;
    msg.data[6] = 0xEA; //filler value, unused
    msg.data[7] = 0xFA; //filler value, unused

    if(flag)
    {
        msg.data[4] = IAV::CONTACTOR_ON & 0xFF;
        msg.data[5] = IAV::CONTACTOR_ON >> 8;
    }
    else
    {
        msg.data[4] = IAV::CONTACTOR_OFF & 0xFF;
        msg.data[5] = IAV::CONTACTOR_OFF >> 8;
    }
    SendMessage(msg);

    checkReadMsg();
}

void
CANWrapper::setBridge(bool flag)
{
    IAV::CANmsg msg;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8;
    msg.identity = IAV::RSDO;

    msg.data[0] = IAV::WRITE_REQUEST_2BYTES;
    msg.data[1] = IAV::BRIDGE & 0xFF;
    msg.data[2] = IAV::BRIDGE >> 8;
    msg.data[3] = IAV::BRIDGE_SUBINDEX;
    msg.data[6] = 0xEA;
    msg.data[7] = 0xFA;

    if(flag)
    {
        msg.data[4] = IAV::BRIDGE_ON & 0xFF;
        msg.data[5] = IAV::BRIDGE_ON >> 8;
    }
    else
    {
        msg.data[4] = IAV::BRIDGE_OFF & 0xFF;
        msg.data[5] = IAV::BRIDGE_OFF >> 8;
    }
    SendMessage(msg);

    checkReadMsg();
}

void
CANWrapper::setMaxTorque(uint16_t torque)
{
    IAV::CANmsg msg;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8;
    msg.identity = IAV::RSDO;

    msg.data[0] = IAV::WRITE_REQUEST_2BYTES;
    msg.data[1] = IAV::MAX_TORQUE & 0xFF;
    msg.data[2] = IAV::MAX_TORQUE >> 8;
    msg.data[3] = IAV::MAX_TORQUE_SUBINDEX;
    msg.data[6] = 0xEA;
    msg.data[7] = 0xFA;

    msg.data[4] = torque & 0xFF;
    msg.data[5] = torque >> 8;

    SendMessage(msg);

    checkReadMsg();
}

void
CANWrapper::checkReadMsg()
{
    int got = read(_fd, &_rx, sizeof(IAV::CANmsg));

    if(got > 0)
    {
    int j;
        printf("Received with ret=%d: id=%u/0x%08x ",
            got,_rx.identity, _rx.identity);

        printf("\tlen=%d", _rx.data_length);

        if(_rx.data_length > 0 )
        {
            int length = _rx.data_length;

            for(j = 0; j < length; j++)
            {
                printf(" %02x", _rx.data[j]);
            }
        }
        printf("\n"); fflush(stdout);
    }
}

void
CANWrapper::SendSpeed(int speed)
{
    IAV::CANmsg msg;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8;
    msg.identity = IAV::RSDO;

    msg.data[0] = IAV::WRITE_REQUEST_4BYTES;
    msg.data[1] = IAV::TARGET_SPEED & 0xFF;
    msg.data[2] = IAV::TARGET_SPEED >> 8;
    msg.data[3] = IAV::TARGET_SPEED_SUBINDEX;

    auto hexVector = convertToHex(speed);
    int hexSize = hexVector.size();

    for(int j = 4, k = 0; j < 8; j++, ++k)
    {
        msg.data[j] = hexVector[hexSize - 1 - k];
    }
    std::cout<<std::endl;

    SendMessage(msg);

    checkReadMsg();

    if(speed == 0)
    {
        setBridge(true);
        setMaxTorque(MaxTorque);
    }
}

std::vector<uint8_t>
CANWrapper::convertToHex(int value)
{
    std::stringstream ss;
    ss<< std::hex << (int)value; // int decimal_value
    std::string strHex ( ss.str() );

    return convertToHex(strHex);
}

std::vector<uint8_t>
CANWrapper::convertToHex(std::string string)
{

    size_t slength = string.length();

    if(slength < 8)
    {
        const int diff = 8 - slength;
        for(int i = 0; i < diff; ++i)
        {
            string.insert(string.begin(), '0');
        }
    }

    slength = string.length();

    auto cString = string.c_str();
    size_t dlength = slength / 2;

    std::vector<uint8_t> data(dlength);

    size_t index = 0;
    while (index < slength)
    {
        char c = cString[index];
        int value = 0;
        if (c >= '0' && c <= '9')
            value = (c - '0');
        else if (c >= 'A' && c <= 'F')
            value = (10 + (c - 'A'));
        else if (c >= 'a' && c <= 'f')
            value = (10 + (c - 'a'));

        data[(index / 2)] += value << (((index + 1) % 2) * 4);

        index++;
    }

    return data;
}

void
CANWrapper::sevconLogin()
{
    IAV::CANmsg msg;
    msg.bufferFull = false;
    msg.syncFlag = false;
    msg.data_length = 8;
    msg.identity = IAV::RSDO;

    for(int i = 0; i < 2 ; ++i)
    {
        msg.data[0] = IAV::READ_REQUEST;
        msg.data[1] = IAV::LOGIN_ENTRY & 0xFF;
        msg.data[2] = IAV::LOGIN_ENTRY >> 8;
        msg.data[3] = IAV::LOGIN_ACCESS_LEVEL_SUBINDEX;
        msg.data[4] = 0x00;
        msg.data[5] = 0x00;
        msg.data[6] = 0x00;
        msg.data[7] = 0x00;

        SendMessage(msg);
        checkReadMsg();
    }

    msg.data[0] = IAV::WRITE_REQUEST_2BYTES;
    msg.data[1] = IAV::LOGIN_ENTRY & 0xFF;
    msg.data[2] = IAV::LOGIN_ENTRY >> 8;
    msg.data[3] = IAV::LOGIN_USER_ID_SUBINDEX;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0xEA;
    msg.data[7] = 0xFA;

    SendMessage(msg);
    checkReadMsg();

    msg.data[0] = IAV::READ_REQUEST;
    msg.data[1] = IAV::PASSWORD_KEY & 0xFF;
    msg.data[2] = IAV::PASSWORD_KEY >> 8;
    msg.data[3] = IAV::PASSWORD_KEY_SUBINDEX;
    msg.data[4] = 0x00;
    msg.data[5] = 0x00;
    msg.data[6] = 0x00;
    msg.data[7] = 0x00;

    SendMessage(msg);
    checkReadMsg();

    usleep(40000);

    msg.data[0] = IAV::WRITE_REQUEST_2BYTES; //ADDED THIS
    msg.data[1] = IAV::LOGIN_ENTRY & 0xFF;
    msg.data[2] = IAV::LOGIN_ENTRY >> 8;
    msg.data[3] = IAV::LOGIN_PASSWORD_SUBINDEX;
    msg.data[4] = IAV::LOGIN_LEVEL_5 & 0xFF;
    msg.data[5] = IAV::LOGIN_LEVEL_5 >> 8;
    msg.data[6] = 0xEA;
    msg.data[7] = 0xFA;

    SendMessage(msg);
    checkReadMsg();
}
