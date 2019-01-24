/*
 * WebotsControlsManager.hpp
 *
 *  Created on: 2018-10-18
 *      Author: liam
 */

#ifndef WEBOTSCONTROLSMANAGER_HPP_
#define WEBOTSCONTROLSMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>

/****************************************
 * STRUCTS
 ****************************************/

/****************************************
 * CLASS DEFINITION
 ****************************************/

class WebotsControlsManager
{

public:

    WebotsControlsManager();
    ~WebotsControlsManager();

    bool Initialize(webots::Robot* robot);
    void PopulateData();

    float GetOdometry() const {return _odometry;};
    float GetSpeed() const {return _speed;};
    float GetTurningRate() const {return _turnRate;};

    void SetLeftMotorVelocity(float velocity);
    void SetRightMotorVelocity(float velocity);

private:

    double getTimestamp();

private:

    int _rightEncoderData;
    int _leftEncoderData;

    float _odometry;
    float _turnRate;
    float _speed;

    double _time;

    webots::Robot* _robot;
    webots::Motor* _rightMotor;
    webots::Motor* _leftMotor;
    webots::PositionSensor* _rightEncoder;
    webots::PositionSensor* _leftEncoder;
};



#endif /* WEBOTSCONTROLSMANAGER_HPP_ */
