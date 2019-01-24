/*
 * SensorManager.cpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "SensorManager.hpp"
#include "LCMChannels.hpp"

#include <iostream>
#include <chrono>
#include <thread>

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

SensorManager::SensorManager()
{
    this->_isRunning = false;
    this->_timestamp = 0.0;

}

SensorManager::~SensorManager()
{
    if(_isRunning)
    {
        Stop();
    }
}

bool
SensorManager::Initialize(SensorFlags initializeSensorFlag)
{

    return setupNetwork(initializeSensorFlag);
}

void
SensorManager::Start()
{
    this->_isRunning = true;
}

void
SensorManager::Update()
{
    if(_isRunning)
    {
        _lcm.handle();
    }
}

void
SensorManager::Stop()
{
    this->_isRunning = false;
}

bool
SensorManager::setupNetwork(SensorFlags initializeSensorFlag)
{
    if(_lcm.good())
    {
        switch(initializeSensorFlag)
        {
            case ALL_FLAG:
                _lcm.subscribe(LidarChannel, &SensorManager::handleLidarMessage, this);
                _lcm.subscribe(GPSChannel, &SensorManager::handleGPSMessage, this);
                _lcm.subscribe(IMUChannel, &SensorManager::handleIMUMessage, this);
                break;
            case LIDAR_FLAG:
                _lcm.subscribe(LidarChannel, &SensorManager::handleLidarMessage, this);
                break;
            case GPS_FLAG:
                _lcm.subscribe(GPSChannel, &SensorManager::handleGPSMessage, this);
                break;
            case IMU_FLAG:
                _lcm.subscribe(IMUChannel, &SensorManager::handleIMUMessage, this);
                break;

        }
        return true;
    }
    return false;
}

void
SensorManager::handleLidarMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::lidar_t *msg)
{
    if(msg->enabled && this->_isRunning)
    {
        std::lock_guard<std::mutex> guard(_lidarDataLock);
        this->_timestamp = msg->timestamp;
        _lidar.horizontalFOV = msg->fov;
        _lidar.numRanges = msg->num_ranges;
        _lidar.collapsedScanline.ranges.resize(_lidar.numRanges);
        std::copy(msg->ranges.begin(), msg->ranges.end(), _lidar.collapsedScanline.ranges.begin());
    }
}

void
SensorManager::handleGPSMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::gps_t *msg)
{
    if(msg->enabled && this->_isRunning)
    {
        this->_timestamp = msg->timestamp;

        _gps.northing = msg->northing;
        _gps.easting = msg->easting;

        _gps.latitude = msg->latitude;
        _gps.longitude = msg->longitude;
        _gps.altitude = msg->altitude;

        _gps.speed = msg->speed;
        _gps.heading = msg->heading;

        _gps.gpsFix = (GPSFix)msg->gps_fix;
        _gps.numSatellites = msg->num_satellites;

        _gps.positionError = msg->position_error;
        _gps.speedError = msg->speed_error;
    }
}

void
SensorManager::handleIMUMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::imu_t *msg)
{
    if(msg->enabled && this->_isRunning)
    {
        this->_timestamp = msg->timestamp;

        _imu.roll = msg->roll;
        _imu.pitch = msg->pitch;
        _imu.yaw = msg->yaw;

        _imu.xAcceleration = msg->xAcceleration;
        _imu.yAcceleration = msg->yAcceleration;
        _imu.zAcceleration = msg->zAcceleration;
    }
}

