/*
 * VectorNavVN300Driver.hpp
 *
 *  Created on: 2019-01-14
 *      Author: liam
 */

#ifndef VECTORNAVVN300DRIVER_HPP_
#define VECTORNAVVN300DRIVER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "GPS.hpp"
#include "IMU.hpp"

#include "vn/sensors.h"
#include "vn/thread.h"

#include <vector>

/****************************************
 * CLASS DEFINITION
 ****************************************/

class VectorNavVN300Driver
{

public:

    VectorNavVN300Driver();
    ~VectorNavVN300Driver();

    bool Initialize();

    void Start();
    void Stop();

    inline bool IsRunning() const {return _isRunning;};

    void SetDataFrequency(uint8_t freq);

    IMU GetIMUData() const {return _imu;};
    GPS GetGPSData() const {return _gps;};
    double GetTimestamp() const {return _timestamp;};

public:

    std::vector<double> _gpsPositionECEF;
    std::vector<float> _gpsVelocityECEF;
    std::vector<float> _gpsAccelerationECEF;
    std::vector<float> _gpsPositionECEFUncertainty;
    float _gpsVelocityECEFUncertainty;

    std::vector<double> _gpsPositionEstimatedECEF;
    std::vector<float> _gpsVelocityEstimatedECEF;

    float _positionEstimatedUncertainty;
    float _velocityEstimatedUncertainty;

private:

    static void asciiAsyncYPRMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);
    static void asciiAsyncGPSMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);
    static void binaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index);

    void binaryAsyncMessageReceivedFUNC(void* userData, vn::protocol::uart::Packet& p, size_t index);

private:

    bool _isRunning;

    vn::sensors::VnSensor _vn300;

    GPS _gps;
    IMU _imu;
    double _timestamp;

};

#endif /* VECTORNAVVN300DRIVER_HPP_ */
