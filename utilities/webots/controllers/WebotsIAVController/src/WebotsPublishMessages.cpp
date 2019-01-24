/*
 * WebotsPublishMessages.cpp
 *
 *  Created on: 2018-11-22
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsPublishMessages.hpp"
#include "LCMChannels.hpp"

#include "Lidar.hpp"
#include "GPS.hpp"
#include "IMU.hpp"

#include "lidar_t.hpp"
#include "imu_t.hpp"
#include "gps_t.hpp"

#include <chrono>
#include <thread>

/****************************************
 * FUNCTION IMPLEMENTATION
 ****************************************/

void
PublishData(lcm::LCM *lcm, WebotsSensorManager *sensors, std::atomic<bool>& isRunning)
{
    while(isRunning)
    {
        PublishLidarMessage(lcm, sensors);
        PublishIMUMessage(lcm, sensors);
        PublishGPSMessage(lcm, sensors);

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void
PublishLidarMessage(lcm::LCM *lcm, WebotsSensorManager *sensors)
{
    exlcm::lidar_t msg;
    msg.enabled = true;
    msg.name = "lidar";

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    msg.timestamp = (int64_t)ms.count();

    auto lidarData = sensors->getLidar();

    msg.num_ranges = lidarData.numRanges;
    msg.ranges.resize(msg.num_ranges);
    auto centerScanline = lidarData.GetCenterScanline();
    std::copy(centerScanline.ranges.begin(), centerScanline.ranges.end(), msg.ranges.begin());

    msg.fov = lidarData.horizontalFOV;

    lcm->publish(LidarChannel, &msg);

}

void
PublishIMUMessage(lcm::LCM *lcm, WebotsSensorManager *sensors)
{
    exlcm::imu_t msg;
    msg.enabled = true;
    msg.name = "imu";

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    msg.timestamp = (int64_t)ms.count();

    auto imuData = sensors->getIMU();

    msg.roll = imuData.roll;
    msg.pitch = imuData.pitch;
    msg.yaw = imuData.yaw;
    msg.xAcceleration = imuData.xAcceleration;
    msg.yAcceleration = imuData.yAcceleration;
    msg.zAcceleration = imuData.zAcceleration;

    lcm->publish(IMUChannel, &msg);
}

void
PublishGPSMessage(lcm::LCM *lcm, WebotsSensorManager *sensors)
{
    exlcm::gps_t msg;
    msg.enabled = true;
    msg.name = "gps";

    std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch());
    msg.timestamp = (int64_t)ms.count();

    auto gpsData = sensors->getGPS();

    msg.northing = gpsData.northing;
    msg.easting = gpsData.easting;

    msg.altitude = gpsData.altitude;
    msg.longitude = gpsData.longitude;
    msg.latitude = gpsData.latitude;

    msg.speed = gpsData.speed;
    msg.heading = gpsData.heading;

    msg.gps_fix = DIFFERENTIAL;
    msg.num_satellites = 5; //Default Value

    msg.speed_error = 0.01; //Default Value
    msg.position_error = 0.1; //Default Value

    lcm->publish(GPSChannel, &msg);
}

