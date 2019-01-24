/*
 * SensorManager.hpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

#ifndef COMPONENTS_SENSORMANAGER_INCLUDE_SENSORMANAGER_HPP_
#define COMPONENTS_SENSORMANAGER_INCLUDE_SENSORMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "Lidar.hpp"
#include "GPS.hpp"
#include "IMU.hpp"

#include "lidar_t.hpp"
#include "gps_t.hpp"
#include "imu_t.hpp"

#include <lcm/lcm-cpp.hpp>

#include <string>
#include <mutex>

/****************************************
 * ENUM
 ****************************************/

enum SensorFlags
{
    ALL_FLAG,
    LIDAR_FLAG,
    GPS_FLAG,
    IMU_FLAG
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class SensorManager
{

public:

    SensorManager();
    ~SensorManager();

    bool Initialize(SensorFlags initializeSensorFlag);

    void Start();
    void Stop();
    void Update();
    inline bool IsRunning() const {return  _isRunning;};

    Lidar GetLidar() const {return _lidar;};
    IMU GetIMU() const {return _imu;};
    GPS GetGPS() const {return _gps;};

private:

    bool setupNetwork(SensorFlags initializeSensorFlag);

    void handleLidarMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::lidar_t *msg);
    void handleGPSMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::gps_t *msg);
    void handleIMUMessage(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const exlcm::imu_t *msg);

private:

    bool _isRunning;

    float _timestamp;

    Lidar _lidar;
    GPS _gps;
    IMU _imu;

    lcm::LCM _lcm;

    std::mutex _lidarDataLock;
};



#endif /* COMPONENTS_SENSORMANAGER_INCLUDE_SENSORMANAGER_HPP_ */
