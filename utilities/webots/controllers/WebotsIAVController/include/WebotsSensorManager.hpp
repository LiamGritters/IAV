/*
 * WebotsSensorManager.hpp
 *
 *  Created on: 2018-10-11
 *      Author: liam
 */

#ifndef WEBOTSSENSORMANAGER_HPP_
#define WEBOTSSENSORMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "Lidar.hpp"
#include "GPS.hpp"
#include "IMU.hpp"

#include <highgui/highgui.hpp>
#include <core/core.hpp>
#include <imgproc/imgproc.hpp>

#include <webots/Robot.hpp>
#include <webots/InertialUnit.hpp>
#include <webots/GPS.hpp>
#include <webots/Camera.hpp>
#include <webots/Lidar.hpp>
#include <webots/Compass.hpp>
#include <webots/Accelerometer.hpp>

#include <vector>

/****************************************
 * STRUCTS
 ****************************************/

struct stereoCamera
{
    stereoCamera(){};

    cv::Mat rgbd; //3D image-> red,green,blue pixels and depth
};

struct camera
{
    camera(){};

    cv::Mat rgb; //2D Image-> red,green,blue pixels
};

/****************************************
 * CLASS DEFINITION
 ****************************************/

class WebotsSensorManager
{

public:

    WebotsSensorManager();
    ~WebotsSensorManager();

    void Initialize(webots::Robot* robot);
    void PopulateData();

    Lidar getLidar() const {return _lidarData;};

    IMU getIMU() const {return _imuData;};
    GPS getGPS() const {return _gpsData;};
    stereoCamera getStereoCamera() const {return _stereoCamData;};
    camera getLeftCamera() const {return _camLeftData;};
    camera getRightCamera() const {return _camRightData;};

private:

    void populateIMU();
    void populateLidar();
    void populateGPS();
    void populateCompass();
    void populateAccelerometer();

    double getBearing(double x, double z); //in degrees

private:

    Lidar _lidarData;
    IMU _imuData;
    GPS _gpsData;
    stereoCamera _stereoCamData;
    camera _camLeftData;
    camera _camRightData;

    webots::Robot* _robot;
    webots::InertialUnit* _imu;
    webots::GPS* _gps;
    webots::Lidar* _lidar;
    webots::Camera* _stereoCam;
    webots::Camera* _leftCam;
    webots::Camera* _rightCam;
    webots::Compass* _compass; //use as gps heading
    webots::Accelerometer* _accelerometer; //used for imu

};

#endif /* WEBOTSSENSORMANAGER_HPP_ */
