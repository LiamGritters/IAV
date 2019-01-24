// File:          WebotsIAVController.cpp
// Date:
// Description:
// Author:
// Modifications:


#include "WebotsSystemInterface.hpp"

#include <webots/Robot.hpp>
#include <iostream>

int main(int argc, char **argv)
{
    WebotsSystemInterface interface;

    interface.Initialize();
    interface.Start();

    return 0;
}
