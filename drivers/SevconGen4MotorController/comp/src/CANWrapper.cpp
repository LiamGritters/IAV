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
#include <fcntl.h>
#include <unistd.h>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr int SleepTime = 1000;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

CANWrapper::CANWrapper()
{
    this->_fd = -1;
    this->_isRunning = false;
}

CANWrapper::~CANWrapper()
{
    Stop();
}

bool
CANWrapper::Initialize(std::string canDeviceName)
{
    if(canDeviceName.empty()) return false;

    this->_fd = open(canDeviceName.c_str(), O_RDWR | O_NONBLOCK);

    if(_fd < 0)
    {
        std::cout<<"[ERROR]: Cannot open CAN device"<<std::endl;
        return false;
    }

    return true;
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
        got = read(wrapper->_fd, wrapper->_rx, BufferSize);

        if(got > 0)
        {
            int i,j;
            for(i = 0; i < got; i++)
            {
                printf("Received with ret=%d: %12lu.%06lu id=%u/0x%08x\n",
                    got,
                    wrapper->_rx[i].timestamp.tv_sec,
                    wrapper->_rx[i].timestamp.tv_usec,
                    wrapper->_rx[i].id, wrapper->_rx[i].id);

                printf("\tlen=%d", wrapper->_rx[i].length);
                printf(" flags=0x%02x", wrapper->_rx[i].flags );

                char* format = (char*)" : %c%c%c (%2d):";
                printf(format,
                    /* extended/base */
                        (wrapper->_rx[i].flags & MSG_EXT) ?
                        (wrapper->_rx[i].flags & MSG_SELF) ? 'E' : 'e'
                        :
                        (wrapper->_rx[i].flags & MSG_SELF) ? 'B' : 'b',
                    /* remote/data */
                        (wrapper->_rx[i].flags & MSG_RTR) ? 'R' : 'D',
                        (wrapper->_rx[i].flags & MSG_RBRS) ? 'F' : ' ',
                                wrapper->_rx[i].length );
                if( !(wrapper->_rx[i].flags & MSG_RTR) )
                {
                    int length = (wrapper->_rx[i].length > 8)? 8 : wrapper->_rx[i].length;

                    for(j = 0; j < length; j++)
                    {
                        printf(" %02x", wrapper->_rx[i].data[j]);
                    }
                }
                printf("\n"); fflush(stdout);
            }
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
CANWrapper::SendMessage()
{
    size_t msgSize = 16;
    const int sent = write(_fd, _tx, msgSize);

    if (sent <= 0)
    {
        perror("sent");
        Stop();
    }

}



