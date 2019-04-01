/*
 * DirectionalController.cpp
 *
 *  Created on: 2018-11-08
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "DirectionalController.hpp"

#include <iostream>
#include <ncurses.h>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

DirectionalController::DirectionalController()
{
//    this->_velocity = 0.0;
    this->_turningRate = 0.0;
    this->_isRunning = false;
    this->_speedLevel = 3; //Default Speed level
}

DirectionalController::~DirectionalController()
{
    Stop();
}

void
DirectionalController::Start()
{
    this->_isRunning = true;
    this->_thread = std::thread(DirectionalController::controllerThread, this);
}

void
DirectionalController::Stop()
{
    this->_isRunning = false;
    if(_thread.joinable())
    {
        _thread.join();
    }
}

void
DirectionalController::controllerThread(DirectionalController *controller)
{
    initscr();
    int c;

    raw();
    keypad(stdscr, TRUE);
    noecho();

    printw("Press the up and down arrows to change speed \n");
    printw("Press the left and right arrows to steer \n");
    printw("Press s to stop the vehicle and q to quit the controller \n");

    while(controller->_isRunning)
    {
        c = getch();
        switch(c)
        {   case KEY_UP:
                controller->_speedLevel++;
                break;
            case KEY_DOWN:
                controller->_speedLevel--;
                break;
            case KEY_LEFT:
                controller->_turningRate = -1.0;
                break;
            case KEY_RIGHT:
                controller->_turningRate = 1.0;
                break;
            default:
                refresh();
                break;
        }

        if(controller->_speedLevel > 9) controller->_speedLevel = 9;
        if(controller->_speedLevel < 0 ) controller->_speedLevel = 0;

        if(c == 's')
        {
            controller->_speedLevel = 0;
        }
        if((c == 'q') || (c == 27))
        {
            printw("Stopping Controller Thread \n");
            break;
        }
        refresh();
    }
    controller->_isRunning = false;

    refresh();
    endwin();
}
