/*
 * PIDControllerTest.cpp
 *
 *  Created on: 2019-03-26
 *      Author: liam
 */


/****************************************
 * INCLUDES
 ****************************************/

#include "PIDController.hpp"

#include <iostream>

/****************************************
 * Main
 ****************************************/

int main()
{
    PIDController controller;

    controller.Initialize(0.1, 0.5, 0.01, 100, -100, 0.1);

    double val = 0.0;

    for(int i = 0; i < 100; ++i)
    {
        double inc = controller.Calculate(20, val);
        std::cout<<"val: "<<val<<", inc: "<<inc<<std::endl;
        val += inc;
    }
}


