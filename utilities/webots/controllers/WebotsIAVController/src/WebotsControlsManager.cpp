/*
 * WebotsControlsManager.cpp
 *
 *  Created on: 2018-10-18
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsControlsManager.hpp"

#include <cmath>
#include <iostream>
#include <string>
#include <chrono>

/****************************************
 * CONSTANTS
 ****************************************/

const std::string RightMotorName = "rotational motor right";
const std::string LeftMotorName = "rotational motor left";
const std::string RightEncoderName = "position sensor right";
const std::string LeftEncoderName = "position sensor left";

constexpr int EncoderSamplingPeriod = 10;
constexpr int EncoderResolution = 1024; //Number of ticks
constexpr float WheelDiameter = 0.17;
constexpr float WheelBase = 1.2;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

WebotsControlsManager::WebotsControlsManager()
{
    this->_rightEncoderData = 0;
    this->_leftEncoderData = 0;
    this->_odometry = 0.0;
    this->_turnRate = 0.0;
    this->_speed = 0.0;
    this->_robot= nullptr;
    this->_leftMotor = nullptr;
    this->_rightMotor = nullptr;
    this->_leftEncoder = nullptr;
    this->_rightEncoder = nullptr;
}

WebotsControlsManager::~WebotsControlsManager()
{
    //let WebotsSystemInterface delete robot object
}

bool
WebotsControlsManager::Initialize(webots::Robot* robot)
{
    this->_time = getTimestamp();

    this->_robot = robot;

    this->_rightMotor = _robot->getMotor(RightMotorName);
    this->_leftMotor = _robot->getMotor(LeftMotorName);
    this->_rightEncoder = _robot->getPositionSensor(RightEncoderName);
    this->_leftEncoder = _robot->getPositionSensor(LeftEncoderName);

    if((!_rightMotor) || (!_leftMotor))
    {
        std::cout<<"[ERROR]: Motors Could not be initialized"<<std::endl;
        return false;
    }

    if(_rightEncoder && _leftEncoder)
    {
        _rightEncoder->enable(EncoderSamplingPeriod);
        _leftEncoder->enable(EncoderSamplingPeriod);

        PopulateData();

    }

    SetLeftMotorVelocity(0.0);
    SetRightMotorVelocity(0.0);

    return true;
}

void
WebotsControlsManager::PopulateData()
{
    if(_rightEncoder && _leftEncoder)
    {
        int updatedRightEncoder = (int)((_rightEncoder->getValue() / (2*M_PI)) * EncoderResolution);
        int updatedLeftEncoder = (int)((_leftEncoder->getValue() / (2*M_PI)) * EncoderResolution);
        double updatedTime = getTimestamp();

        float rightDiff = updatedRightEncoder - _rightEncoderData;
        float leftDiff = updatedLeftEncoder - _leftEncoderData;
        double timeDiff = (updatedTime - _time);

        this->_odometry += abs(rightDiff + leftDiff)/2.0 * (2*M_PI/EncoderResolution) * (WheelDiameter/2.0);
        this->_turnRate = ((leftDiff - rightDiff)/2.0 * (2*M_PI/EncoderResolution)  * (WheelDiameter/2.0)) / ((WheelBase/2.0)*timeDiff);
        this->_speed = (abs(rightDiff + leftDiff)/2.0 * (2*M_PI/EncoderResolution) * (WheelDiameter/2.0)) / timeDiff;

        this->_rightEncoderData = updatedRightEncoder;
        this->_leftEncoderData = updatedLeftEncoder;
        this->_time = updatedTime;
    }
}

void
WebotsControlsManager::SetRightMotorVelocity(float velocity)
{
    if(_rightMotor)
    {
        _rightMotor->setPosition(INFINITY);
        _rightMotor->setVelocity(velocity);
    }
    else
    {
        std::cout<<"[ERROR]: Right Motor is not initialized"<<std::endl;
    }
}

void
WebotsControlsManager::SetLeftMotorVelocity(float velocity)
{
    if(_leftMotor)
    {
        _leftMotor->setPosition(INFINITY);
        _leftMotor->setVelocity(velocity);
    }
    else
    {
        std::cout<<"[ERROR]: Left Motor is not initialized"<<std::endl;
    }
}

double
WebotsControlsManager::getTimestamp()
{
    return (double)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count());
}

