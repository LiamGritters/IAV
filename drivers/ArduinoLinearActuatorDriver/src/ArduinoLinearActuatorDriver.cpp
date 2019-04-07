/*
 * ArduinoLinearActuactorDriver.cpp
 *
 *  Created on: 2019-03-29
 *      Author: liam
 */


/****************************************
 * INCLUDES
 ****************************************/

#include <iostream>
#include <string>
#include <string.h>
#include <unistd.h>
#include "../include/ArduinoLinearActuatorDriver.hpp"

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

ArduinoLinearActuatorDriver::ArduinoLinearActuatorDriver()
{
    this->_timestamp = 0.0;
    this->_isRunning = false;
    this->_fd = 0;
    this->_position =0.0;
}

ArduinoLinearActuatorDriver::~ArduinoLinearActuatorDriver()
{
    Stop();
}

bool
ArduinoLinearActuatorDriver::Initialize(std::string deviceFileName)
{
    if(deviceFileName.empty()) return false;

    _fd = open(deviceFileName.c_str(), O_RDWR | O_NOCTTY);

    if(_fd < 0)
    {
        std::cout<<"[ERROR]: could not open Arduino USB port"<<std::endl;
        return false;
    }
    else
    {
        std::cout<<"opened port successfully"<<std::endl;
    }

    /* Set Baud Rate */
    cfsetospeed (&_tty, (speed_t)B9600);
    cfsetispeed (&_tty, (speed_t)B9600);

    _tty.c_cflag     &=  ~PARENB;            // Make 8n1
    _tty.c_cflag     &=  ~CSTOPB;
    _tty.c_cflag     &=  ~CSIZE;
    _tty.c_cflag     |=  CS8;

    /* Make raw */
    cfmakeraw(&_tty);

    tcflush( _fd, TCIFLUSH );
    if ( tcsetattr ( _fd, TCSANOW, &_tty ) != 0)
    {
       std::cout << "Error " << errno << " from tcsetattr" << std::endl;
    }

    return true;
}

void
ArduinoLinearActuatorDriver::Start()
{
    this->_isRunning = true;

    this->_listenThread = std::thread(ArduinoLinearActuatorDriver::listenThread, this);
}

void
ArduinoLinearActuatorDriver::Stop()
{
    _isRunning = false;
    if(_listenThread.joinable())
    {
        _listenThread.join();
    }
}

void
ArduinoLinearActuatorDriver::listenThread(ArduinoLinearActuatorDriver *driver)
{
    /* Whole response*/
    char response[1024];
    memset(response, '\0', sizeof response);
    bool storePosition = false;
    std::string positionStr;

    while(driver->_isRunning)
    {
        int n = 0, spot = 0;
        char buf = '\0';

        do
        {
            n = read( driver->_fd, &buf, 1 );
            if(buf == '<') spot = 0;
            sprintf( &response[spot], "%c", buf );
            spot += n;

            if(buf == ':')
            {
                storePosition = true;
            }
            else if(storePosition)
            {
                if(buf == ' ')
                {
                    storePosition = false;
                }
                positionStr.append(sizeof(char), buf);
            }

        }
        while( buf != '>' && n > 0);

        if(!positionStr.empty())
        {
            driver->_position = atof(positionStr.c_str());
            positionStr.clear();
        }

        if (n < 0)
        {
            std::cout << "Error reading: " << strerror(errno) << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else if (n == 0)
        {
            std::cout << "Read nothing!" << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        else
        {
            std::cout << "Response: " << response << ", position: "<<driver->_position<<std::endl;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

}

void
ArduinoLinearActuatorDriver::SendData(std::string msg)
{
    auto cmd = msg.c_str();
    int n_written = 0, spot = 0;

    do
    {
        n_written = write( _fd, &cmd[spot], 1 );
        spot += n_written;
    }
    while (cmd[spot-1] != '>' && n_written > 0);
}


