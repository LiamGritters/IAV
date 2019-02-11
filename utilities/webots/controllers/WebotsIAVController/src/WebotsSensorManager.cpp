/*
 * WebotsSensorManager.cpp
 *
 *  Created on: 2018-10-11
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsSensorManager.hpp"
#include "UTMConverter.hpp"

#include <iostream>
#include <string>

/****************************************
 * CONSTANTS
 ****************************************/

// Webots Sensor Names -> set in the simulator
const std::string GpsName = "gps";
const std::string ImuName = "imu";
const std::string LidarName = "Velodyne VLP-16";
const std::string StereoCamName = "MultiSense S21";
const std::string LeftCamName = "cameraLeft";
const std::string RightCamName = "cameraRight";
const std::string CompassName = "compass";
const std::string AccelerometerName = "accelerometer";

// Sensors Sampling Periods (Hz)
constexpr int GpsSamplePeriod = 10;
constexpr int ImuSamplePeriod = 15;
constexpr int LidarSamplePeriod = 20;
constexpr int CamSamplePeriod = 15;

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

WebotsSensorManager::WebotsSensorManager()
{
    this->_robot = nullptr;
    this->_gps = nullptr;
    this->_imu = nullptr;
    this->_lidar = nullptr;
    this->_stereoCam = nullptr;
    this->_leftCam = nullptr;
    this->_rightCam = nullptr;
    this->_compass = nullptr;
    this->_accelerometer = nullptr;
};

WebotsSensorManager::~WebotsSensorManager()
{
    //let WebotsSystemInterface delete robot object
}

void
WebotsSensorManager::Initialize(webots::Robot* robot)
{
    this->_robot = robot;

    //Initialize all the sensors
    this->_gps = _robot->getGPS(GpsName);
    this->_imu = _robot->getInertialUnit(ImuName);
    this->_lidar = _robot->getLidar(LidarName);
//    this->_stereoCam = _robot->getCamera(StereoCamName);
    this->_leftCam = _robot->getCamera(LeftCamName);
    this->_rightCam = _robot->getCamera(RightCamName);
    this->_compass = _robot->getCompass(CompassName);
    this->_accelerometer = _robot->getAccelerometer(AccelerometerName);

    if(_compass)
    {
        _compass->enable(GpsSamplePeriod); //same as GPS

        populateCompass();
    }

    if(_accelerometer)
    {
        _accelerometer->enable(ImuSamplePeriod); //same as IMU

        populateAccelerometer();
    }

    if(_gps)
    {
        _gps->enable(GpsSamplePeriod);

        populateGPS();
    }

    if(_imu)
    {
        _imu->enable(ImuSamplePeriod);

        populateIMU();
    }

    if(_lidar)
    {
        _lidar->enable(LidarSamplePeriod);

        _lidarData.horizontalFOV = _lidar->getFov();
        _lidarData.verticalFOV = _lidar->getVerticalFov();
        _lidarData.numLayers = _lidar->getNumberOfLayers();
        _lidarData.numRanges = _lidar->getHorizontalResolution();
        _lidarData.horizontalRes = _lidarData.horizontalFOV / _lidarData.numRanges;

        if(_lidarData.numLayers > 0)
        {
            _lidarData.scanlines.resize(_lidarData.numLayers);
            _lidarData.collapsedScanline.ranges.resize(_lidarData.numLayers);
            for(auto &scan : _lidarData.scanlines)
            {
                scan.ranges.resize(_lidarData.numRanges);
            }
        }
        else
        {
            _lidar = nullptr;
        }
    }

//    if(_stereoCam)
//    {
//        _stereoCam->enable(CamSamplePeriod);
//    }
}

void
WebotsSensorManager::PopulateData()
{
    populateCompass();
    populateIMU();
    populateLidar();
    populateGPS();
    populateAccelerometer();
}

void
WebotsSensorManager::populateIMU()
{
    if(_imu)
    {
        const double* imuValues = _imu->getRollPitchYaw();
        _imuData.roll = imuValues[0];
        _imuData.pitch = imuValues[1];
        _imuData.yaw = imuValues[2];
    }
}

void
WebotsSensorManager::populateAccelerometer()
{
    if(_accelerometer)
    {
        const double *accelValues = _accelerometer->getValues();
        _imuData.xAcceleration = accelValues[0];
        _imuData.yAcceleration = accelValues[1];
        _imuData.zAcceleration = accelValues[2];
    }
}

void
WebotsSensorManager::populateLidar()
{
    if(_lidar)
    {
        const int numRanges = _lidarData.numRanges;

        for(int i = 0; i < _lidarData.numLayers; ++i)
        {
            const float* lidarValues = _lidar->getLayerRangeImage(i);

            if(lidarValues)
            {
                std::copy(lidarValues, lidarValues + numRanges, _lidarData.scanlines[i].ranges.begin());
            }
            else
            {
                std::cout<<"unable to retrieve LiDAR values"<<std::endl;
                break;
            }
        }
    }
}

void
WebotsSensorManager::populateGPS()
{
    if(_gps)
    {
        const double* gpsValues = _gps->getValues();
        _gpsData.latitude = gpsValues[0];
        _gpsData.longitude = gpsValues[1];
        _gpsData.altitude = gpsValues[2];

        LatLonToUTMXY(_gpsData.latitude,_gpsData.longitude, 0, _gpsData.easting, _gpsData.northing);

        _gpsData.speed = _gps->getSpeed();
    }
}

void
WebotsSensorManager::populateCompass()
{
    if(_compass)
    {
        const double* compassData = _compass->getValues();
        _gpsData.heading = (float)getBearing(compassData[0], compassData[2]);
    }
}

double
WebotsSensorManager::getBearing(double x, double z)
{
    const double rad = atan2(x, z);
    double bearing = (rad - 1.5708) / M_PI * 180.0;
    if (bearing < 0.0)
    {
      bearing = bearing + 360.0;
    }
    return bearing;
}
