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

#include "ControlsManager.hpp"
#include "LCMChannels.hpp"

#include <string>
#include <chrono>
#include <iostream>
#include <cmath>

/****************************************
 * CONSTANTS
 ****************************************/

//Speed Limit
constexpr float MaxVelocityDemand = 5.0; // [m/s]
constexpr float MaxVelocityDemandReverse = -5.0; // [m/s]

//Actuator Limits
constexpr float MaxTurningAngle =  25.0; // deg
constexpr float MinTurningAngle = -25.0; // deg

//PID Controller Parameters
constexpr double Kp = 0.4;
constexpr double Ki = 0.0;
constexpr double Kd = 0.0;
constexpr double MaxAllowableSpeedChange = 100.0;
constexpr double MinAllowableSpeedChange = -100.0;
constexpr double DeltaTime = 0.05;

//Motor Controller CAN Parameters
const std::string CanDevice = "can0"; //CAN port

//Arduino Parameters
const std::string DeviceFileName = "/dev/ttyACM0"; //usb Port

//Motor Constants
constexpr float GearReduction = 1.f/8.f;
constexpr float RPMToRadsPerSec = 2.f*M_PI/60.f;
constexpr float WheelRadius = 0.4572/2.0; // meters

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

bool
checkActuatorPosition(float position)
{
    const int numActuatorPositions = ActuatorPositions.size() - 1;
    if(position > ActuatorPositions[numActuatorPositions]) return false;
    if(position < ActuatorPositions[0]) return false;
    return true;
}

/****************************************
 * Main
 ****************************************/

int main()
{
    PIDController motorController;
    CANWrapper motor;
    ArduinoLinearActuatorDriver actuator;

    ControlsManager controlsManager;
    if(!controlsManager.Initialize())
    {
        std::cout<<"[ERROR]: could not initialize controls manager"<<std::endl;
        return -1;
    }

    lcm::LCM lcm;
    if(!lcm.good())
    {
        std::cout<<"LCM could not start"<<std::endl;
        return -1;
    }

    iav_lcm::driveline_t msg;
    msg.enabled = true;
    msg.name = "driveline";

    if(!motorController.Initialize(Kp, Ki, Kd, MaxAllowableSpeedChange, MinAllowableSpeedChange, DeltaTime) ||
       !motor.Initialize(CanDevice) ||
       !actuator.Initialize(DeviceFileName))
    {
        std::cout<<"[ERROR]: could not initialize one or more components"<<std::endl;
        return -1;
    }

    motor.Start();

    for(int i = 0; i < 10; ++i) //must put motor into operable state by sending minimal speed command
    {
        motor.SendSpeed(30);
        usleep(40000);
    }
    motor.SendSpeed(0);

    controlsManager.Start();
    actuator.Start();
    uint64_t timeStamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();

    while(controlsManager.IsRunning())
    {
        if((motor.IsRunning()) && (actuator.IsRunning()))
        {
            //Convert velocity demand to motor velocity
            float velocityDemand = controlsManager.GetVelocity();
            velocityDemand = (velocityDemand > MaxVelocityDemand) ? MaxVelocityDemand : velocityDemand;//Limit input speed, for safety ->change later
            velocityDemand = (velocityDemand < MaxVelocityDemandReverse) ? MaxVelocityDemandReverse : velocityDemand;

            float turningAngle = controlsManager.GetTurningAngle();
            turningAngle = (turningAngle > MaxTurningAngle) ? MaxTurningAngle : turningAngle;
            turningAngle = (turningAngle < MinTurningAngle) ? MinTurningAngle : turningAngle;

            const float motorDemand = convertVehicleSpeedToMotorSpeed(velocityDemand);
            const float actuatorPositionDemand = convertTurningAngleToLinearActuatorPosition(turningAngle);

            std::cout<<"vel: "<<velocityDemand<<", act: "<<actuatorPositionDemand<<std::endl;

            if((motor.GetMotorControllerState() != IAV::OFF))
            {
                double velocity = motor.GetAngularVelocity();
                std::cout<<"read angular velocity: "<<velocity<<std::endl;
                const double incVel = motorController.Calculate(motorDemand, velocity);
                velocity += incVel; //new velocity

                if(fabs(motorDemand) < 0.1)
                {
                    velocity = 0;
                }

                motor.SendSpeed((int)velocity);
            }
            else
            {
                std::cout<<"Motor Controller is OFF"<<std::endl;
            }

            actuator.SendData("<act," + std::to_string(actuatorPositionDemand) + ">");

            const uint64_t newTimestamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
            const uint64_t changeInTime = newTimestamp - timeStamp;
            timeStamp = newTimestamp;

            msg.timestamp = newTimestamp;
            msg.enabled = true;
            msg.speed = convertMotorSpeedToVehicleSpeed(motor.GetAngularVelocity());
            msg.wheelAngle = convertLinearActuatorPositionToTurningAngle(actuator.GetActuactorPosition());
            msg.turningRate = msg.wheelAngle / changeInTime;

            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        else
        {
            //wait for motor and actuator to come back up
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));

            timeStamp = (uint64_t)(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())).count();
            msg.timestamp = timeStamp;
            msg.enabled = false;

            if(!motor.IsRunning())
            {
                std::cout<<"[ERROR]: Motor Controller is not running"<<std::endl;
            }
            if(!actuator.IsRunning())
            {
                std::cout<<"[ERROR]: Actuator is not running"<<std::endl;
            }
        }

        lcm.publish(DrivelineChannel, &msg);
    }

    std::cout<<"Controls Manager Stopped Running"<<std::endl;
    controlsManager.Stop();
    motor.Stop();
    actuator.Stop();

    msg.enabled = false;
    lcm.publish(DrivelineChannel, &msg);
}



