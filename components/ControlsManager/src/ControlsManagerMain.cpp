/*
 * ControlsManagerMain.cpp
 *
 *  Created on: 2019-03-29
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "PIDController.hpp"
#include "CANWrapper.hpp"
#include "ArduinoLinearActuatorDriver.hpp"

#include "vehicle_controller_demand_t.hpp"
#include "driveline_t.hpp"
#include "LCMChannels.hpp"

#include "LinearActuatorControlValues.hpp"

#include <lcm/lcm-cpp.hpp>
#include <string>
#include <chrono>
#include <iostream>
#include <cmath>

/****************************************
 * CONSTANTS
 ****************************************/

//Speed Limit
constexpr float MaxVelocityDemand = 2.0; // [m/s]
constexpr float MaxVelocityDemandReverse = -2.0; // [m/s]

//Actuator Limits
constexpr float MaxTurningAngle =  15.0; // deg
constexpr float MinTurningAngle = -15.0; // deg

//PID Controller Parameters
constexpr double Kp = 0.1;
constexpr double Ki = 0.0;
constexpr double Kd = 0.0;
constexpr double MaxAllowableSpeedChange = 100.0;
constexpr double MinAllowableSpeedChange = -100.0;
constexpr double DeltaTime = 0.1;

//Motor Controller CAN Parameters
const std::string CanDevice = "can0"; //CAN port

//Arduino Parameters
const std::string DeviceFileName = "/dev/ttyACM0"; //usb Port

//Motor Constants
constexpr float GearReduction = 1.f/8.f;
constexpr float RPMToRadsPerSec = 2.f*M_PI/60.f;
constexpr float WheelRadius = 0.4572; // meters

/****************************************
 * CLASSES
 ****************************************/

class ReceiveControllerInputs
{
    public:
        ReceiveControllerInputs()
        {
            this->timestamp = 0.0;
            this->velocity = 0.0;
            this->turningAngle = 0.0;
            this->enabled = false;
        }
        ~ReceiveControllerInputs() {}

        void
        HandleControlInputsMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const iav_lcm::vehicle_controller_demand_t* msg)
        {
            enabled = msg->enabled;
            velocity = msg->velocity;
            turningAngle = msg->turningAngle;
            timestamp = msg->timestamp;
        }

    public:


        bool       enabled;
        float      velocity;
        float      turningAngle;
        int64_t    timestamp;
};

/****************************************
 * FUNCTIONS
 ****************************************/

float
convertVehicleSpeedToMotorSpeed(float vehicleSpeed)
{
    return ((1.f/(GearReduction*RPMToRadsPerSec)) / (WheelRadius)) * vehicleSpeed;
}

float
convertMotorSpeedToVehicleSpeed(float motorSpeed)
{
    return GearReduction*RPMToRadsPerSec*WheelRadius*motorSpeed;
}

float
convertTurningAngleToLinearActuatorPosition(float turningAngle)
{
    float position = 0.0;

    const int numWheelAngles = WheelAngles.size() - 1;
    for(int i = 0; i < numWheelAngles; ++i)
    {
        if((turningAngle >= WheelAngles[i]) && (turningAngle <= WheelAngles[i+1]))
        {
            position = (((turningAngle - WheelAngles[i]) * (ActuatorPositions[i+1] - ActuatorPositions[i])) / (WheelAngles[i+1] - WheelAngles[i])) + ActuatorPositions[i];
        }
    }

    if(turningAngle > WheelAngles[numWheelAngles]) position = ActuatorPositions[numWheelAngles]; //LIMITS MIGHT HAVE TO BE CHANGED
    if(turningAngle < WheelAngles[0]) position = ActuatorPositions[0];

    return position;
}

float
convertLinearActuatorPositionToTurningAngle(float position)
{
    float turningAngle = 0.0;

    const int numActuatorPositions = ActuatorPositions.size() - 1;
    for(int i = 0; i < numActuatorPositions; ++i)
    {
        if((position >= ActuatorPositions[i]) && (position <= ActuatorPositions[i+1]))
        {
            turningAngle = (((position - ActuatorPositions[i]) * (WheelAngles[i+1] - WheelAngles[i])) / (ActuatorPositions[i+1] - ActuatorPositions[i])) + WheelAngles[i];
        }
    }

    if(position > ActuatorPositions[numActuatorPositions]) turningAngle = WheelAngles[numActuatorPositions]; //LIMITS MIGHT HAVE TO BE CHANGED
    if(position < ActuatorPositions[0]) turningAngle = WheelAngles[0];

    return turningAngle;
}

/****************************************
 * Main
 ****************************************/

int main()
{
    PIDController controller;
    CANWrapper motor;
    ArduinoLinearActuatorDriver actuator;

    if(!controller.Initialize(Kp, Ki, Kd, MaxAllowableSpeedChange, MinAllowableSpeedChange, DeltaTime) ||
       !motor.Initialize(CanDevice) ||
       !actuator.Initialize(DeviceFileName))
    {
        std::cout<<"[ERROR]: could not initialize one or more components"<<std::endl;
        return false;
    }

    lcm::LCM lcm;
    if(!lcm.good()) return 1;

    ReceiveControllerInputs controlInputs;
    lcm.subscribe(ControllerChannel, &ReceiveControllerInputs::HandleControlInputsMessage, &controlInputs);
    lcm.handle();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    motor.Start();
    actuator.Start();

    iav_lcm::driveline_t msg;
    msg.enabled = true;
    msg.name = "driveline";

    uint64_t timeStamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

    while(controlInputs.enabled)
    {
        lcm.handle();

        if((motor.IsRunning()) && (actuator.IsRunning()) && (motor.GetMotorControllerState() != IAV::OFF))
        {
            //Convert velocity demand to motor velocity
            float velocityDemand = controlInputs.velocity;
            velocityDemand = (velocityDemand > MaxVelocityDemand) ? MaxVelocityDemand : velocityDemand;//Limit input speed, for safety ->change later
            velocityDemand = (velocityDemand < MaxVelocityDemandReverse) ? MaxVelocityDemandReverse : velocityDemand;

            float turningAngle = controlInputs.turningAngle;
            turningAngle = (turningAngle > MaxTurningAngle) ? MaxTurningAngle : turningAngle;
            turningAngle = (turningAngle < MinTurningAngle) ? MinTurningAngle : turningAngle;

            const float motorDemand = convertVehicleSpeedToMotorSpeed(velocityDemand);
            const float actuatorPositionDemand = convertTurningAngleToLinearActuatorPosition(turningAngle);

            double velocity = motor.ReadAngularVelocity();
            const double incVel = controller.Calculate(motorDemand, velocity);
            velocity += incVel; //new velocity

            double actuatorPosition = actuator.GetActuactorPosition();
            const double incPos = controller.Calculate(actuatorPositionDemand, actuatorPosition);
            actuatorPosition += incPos; //new actuator position

            motor.SendSpeed((int)velocity);
            actuator.SendData("<sendingData," + std::to_string(actuatorPosition) + ">");

            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            const uint64_t newTimestamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
            const uint64_t changeInTime = newTimestamp - timeStamp;
            timeStamp = newTimestamp;

            msg.timestamp = newTimestamp;
            msg.enabled = true;
            msg.speed = convertMotorSpeedToVehicleSpeed(motor.ReadAngularVelocity());
            msg.wheelAngle = convertLinearActuatorPositionToTurningAngle(actuator.GetActuactorPosition());
            msg.turningRate = msg.wheelAngle / changeInTime;
        }
        else
        {
            //wait for motor and actuator to come back up
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            timeStamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
            msg.timestamp = timeStamp;
            msg.enabled = false;

            std::cout<<"[ERROR]: Motor Controller or Actuator are not running"<<std::endl;
        }

        lcm.publish(DrivelineChannel, &msg);
    }

    msg.enabled = false;
}



