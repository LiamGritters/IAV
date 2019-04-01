/*
 * PIDController.cpp
 *
 *  Created on: 2019-03-26
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "PIDController.hpp"

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

PIDController::PIDController()
{
    this->_kp = 0.0;
    this->_ki = 0.0;
    this->_kd = 0.0;
    this->_dt = 0.0;
    this->_max = 0.0;
    this->_min = 0.0;

    this->_integral = 0.0;
    this->_preError = 0.0;
}

PIDController::~PIDController()
{

}

bool
PIDController::Initialize(double Kp, double Ki, double Kd, double max, double min, double dt)
{
    if(dt <= 0) return false; //dt must be positive

    _dt = dt;
    _kp = Kp;
    _ki = Ki;
    _kd = Kd;
    _max = max;
    _min = min;

    return true;
}

double
PIDController::Calculate(double SetPoint, double ProcessValue)
{

    // Calculate error
    double error = SetPoint - ProcessValue;

    // Proportional term
    double Pout = _kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _ki * _integral;

    // Derivative term
    double derivative = (error - _preError) / _dt;
    double Dout = _kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if( output > _max )
        output = _max;
    else if( output < _min )
        output = _min;

    // Save error to previous error
    _preError = error;

    return output;
}
