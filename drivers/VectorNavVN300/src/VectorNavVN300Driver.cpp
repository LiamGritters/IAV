/*
 * VectorNavVN300Driver.cpp
 *
 *  Created on: 2019-01-14
 *      Author: liam
 */

/****************************************
 * INCLUDES
 ****************************************/

#include "VectorNavVN300Driver.hpp"
#include "vn/compositedata.h"

#include <iostream>
#include <string>

/****************************************
 * CONSTANTS
 ****************************************/

constexpr uint32_t SensorBaudrate = 921600;//115200;

const std::string SensorPort = "/dev/ttyUSB0";

/****************************************
 * CLASS IMPLEMENTATION
 ****************************************/

VectorNavVN300Driver::VectorNavVN300Driver()
{
    this->_isRunning = false;
    this->_timestamp = 0.0;
    this->_gpsVelocityECEFUncertainty = 0.0;
    this->_positionEstimatedUncertainty = 0.0;
    this->_velocityEstimatedUncertainty = 0.0;
}

VectorNavVN300Driver::~VectorNavVN300Driver()
{
    if(_isRunning)
    {
        Stop();
    }
    if(_vn300.isConnected())
    {
        _vn300.disconnect();
    }
}

bool
VectorNavVN300Driver::Initialize()
{
    this->_vn300.connect(SensorPort, SensorBaudrate);

    if(!_vn300.isConnected())
    {
        std::cout<<"Connecting..."<<std::endl;
        vn::xplat::Thread::sleepSec(3);
    }

    _gpsPositionECEF.resize(3);
    _gpsVelocityECEF.resize(3);
    _gpsAccelerationECEF.resize(3);
    _gpsPositionECEFUncertainty.resize(3);
    _gpsPositionEstimatedECEF.resize(3);
    _gpsVelocityEstimatedECEF.resize(3);

    std::cout<<"Binary output register"<<std::endl;

    vn::sensors::BinaryOutputRegister bor(
        vn::protocol::uart::ASYNCMODE_PORT1,
        100,
        vn::protocol::uart::COMMONGROUP_TIMEGPS
        | vn::protocol::uart::COMMONGROUP_YAWPITCHROLL
        | vn::protocol::uart::COMMONGROUP_ANGULARRATE
        | vn::protocol::uart::COMMONGROUP_VELOCITY
        | vn::protocol::uart::COMMONGROUP_ACCEL,
        vn::protocol::uart::TIMEGROUP_NONE,
        vn::protocol::uart::IMUGROUP_NONE,
        vn::protocol::uart::GPSGROUP_NUMSATS
        | vn::protocol::uart::GPSGROUP_FIX,
        vn::protocol::uart::ATTITUDEGROUP_NONE,
        vn::protocol::uart::INSGROUP_INSSTATUS
        | vn::protocol::uart::INSGROUP_POSLLA
        | vn::protocol::uart::INSGROUP_POSECEF
        | vn::protocol::uart::INSGROUP_POSU
        | vn::protocol::uart::INSGROUP_VELU
        );

    std::cout<<"write binary output 1"<<std::endl;

    _vn300.writeBinaryOutput1(bor);

    return true;
}

void
VectorNavVN300Driver::Start()
{
    this->_isRunning = true;

    std::cout<<"registering packets"<<std::endl;

    _vn300.registerAsyncPacketReceivedHandler(this, VectorNavVN300Driver::binaryAsyncMessageReceived);

    std::cout<<"registered packets"<<std::endl;
}

void
VectorNavVN300Driver::Stop()
{
    this->_isRunning = false;
    this->_vn300.unregisterAsyncPacketReceivedHandler();
}

void
VectorNavVN300Driver::SetDataFrequency(uint8_t freq)
{
    if (freq == 0 || freq > 40) return;

    this->_vn300.writeAsyncDataOutputFrequency(freq);
    uint32_t newHz = this->_vn300.readAsyncDataOutputFrequency();
    std::cout << "New Async Frequency: " << newHz << " Hz" << std::endl;
}

void
VectorNavVN300Driver::asciiAsyncYPRMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index)
{
    // Make sure we have an ASCII packet and not a binary packet.
    if (p.type() != vn::protocol::uart::Packet::TYPE_ASCII)
        return;

    // Make sure we have a VNYPR data packet.
    if (p.determineAsciiAsyncType() != vn::protocol::uart::VNYPR)
        return;

    // We now need to parse out the yaw, pitch, roll data.
    vn::math::vec3f ypr;
    p.parseVNYPR(&ypr);

    // Now print out the yaw, pitch, roll measurements.
    std::cout << "ASCII Async YPR: " << ypr << std::endl;
}

void
VectorNavVN300Driver::asciiAsyncGPSMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index)
{
    // Make sure we have an ASCII packet and not a binary packet.
    if (p.type() != vn::protocol::uart::Packet::TYPE_ASCII)
        return;

    // Make sure we have a VNGPS data packet.
    if (p.determineAsciiAsyncType() != vn::protocol::uart::VNGPS)
        return;

    double time;
    uint16_t week;
    uint8_t gpsFix, numSats;
    float speedAcc, timeAcc;
    vn::math::vec3d lla;
    vn::math::vec3f nedVel, nedAcc;

    p.parseVNGPS(&time, &week, &gpsFix, &numSats, &lla, &nedVel, &nedAcc, &speedAcc, &timeAcc);

    std::cout<<"time: "<<time<<", week: "<<week<<", gpsFix: "<<gpsFix<<", numSats: "<<numSats<<", lla: "<<lla<<std::endl;
}

void
VectorNavVN300Driver::binaryAsyncMessageReceivedFUNC(void* userData, vn::protocol::uart::Packet& p, size_t index)
{
    if (p.type() == vn::protocol::uart::Packet::TYPE_BINARY)
    {

        if (!p.isCompatible(
                vn::protocol::uart::COMMONGROUP_TIMEGPS
                | vn::protocol::uart::COMMONGROUP_YAWPITCHROLL
                | vn::protocol::uart::COMMONGROUP_ANGULARRATE
                | vn::protocol::uart::COMMONGROUP_VELOCITY
                | vn::protocol::uart::COMMONGROUP_ACCEL,
                vn::protocol::uart::TIMEGROUP_NONE,
                vn::protocol::uart::IMUGROUP_NONE,
                vn::protocol::uart::GPSGROUP_NUMSATS
                | vn::protocol::uart::GPSGROUP_FIX
                | vn::protocol::uart::GPSGROUP_POSECEF
                | vn::protocol::uart::GPSGROUP_VELECEF
                | vn::protocol::uart::GPSGROUP_POSU
                | vn::protocol::uart::GPSGROUP_VELU,
                vn::protocol::uart::ATTITUDEGROUP_NONE,
                vn::protocol::uart::INSGROUP_INSSTATUS
                | vn::protocol::uart::INSGROUP_POSLLA
                | vn::protocol::uart::INSGROUP_POSECEF
                | vn::protocol::uart::INSGROUP_VELECEF
                | vn::protocol::uart::INSGROUP_ACCELECEF
                | vn::protocol::uart::INSGROUP_POSU
                | vn::protocol::uart::INSGROUP_VELU))
            // Not the type of binary packet we are expecting.
            return;

        vn::sensors::CompositeData cd = vn::sensors::CompositeData::parse(p);

        if(cd.hasTimeGps()) _timestamp = cd.timeGps();
        if(cd.hasYawPitchRoll())
        {
            _imu.yaw = cd.yawPitchRoll().x;
            _imu.pitch = cd.yawPitchRoll().y;
            _imu.roll = cd.yawPitchRoll().z;
        }
        if(cd.hasAcceleration())
        {
            _imu.xAcceleration = cd.acceleration().x;
            _imu.yAcceleration = cd.acceleration().y;
            _imu.zAcceleration = cd.acceleration().z;
        }
        if(cd.hasPositionGpsEcef())
        {
            _gpsPositionECEF[0] = cd.positionGpsEcef().x;
            _gpsPositionECEF[1] = cd.positionGpsEcef().y;
            _gpsPositionECEF[2] = cd.positionGpsEcef().z;
        }
        if(cd.hasVelocityGpsEcef())
        {
            _gpsVelocityECEF[0] = cd.velocityGpsEcef().x;
            _gpsVelocityECEF[1] = cd.velocityGpsEcef().y;
            _gpsVelocityECEF[2] = cd.velocityGpsEcef().z;
        }
        if(cd.hasAccelerationEcef())
        {
            _gpsAccelerationECEF[0] = cd.velocityGpsEcef().x;
            _gpsAccelerationECEF[1] = cd.velocityGpsEcef().y;
            _gpsAccelerationECEF[2] = cd.velocityGpsEcef().z;
        }
        if(cd.hasPositionUncertaintyGpsEcef())
        {
            _gpsPositionECEFUncertainty[0] = cd.positionUncertaintyGpsEcef().x;
            _gpsPositionECEFUncertainty[1] = cd.positionUncertaintyGpsEcef().y;
            _gpsPositionECEFUncertainty[2] = cd.positionUncertaintyGpsEcef().z;
        }
        if(cd.hasVelocityUncertaintyGps())
        {
            _gpsVelocityECEFUncertainty = cd.velocityUncertaintyGps();
        }
        if(cd.hasPositionEstimatedEcef())
        {
            _gpsPositionEstimatedECEF[0] = cd.positionEstimatedEcef().x;
            _gpsPositionEstimatedECEF[1] = cd.positionEstimatedEcef().y;
            _gpsPositionEstimatedECEF[2] = cd.positionEstimatedEcef().z;
        }
        if(cd.hasVelocityEstimatedEcef())
        {
            _gpsVelocityEstimatedECEF[0] = cd.velocityEstimatedEcef().x;
            _gpsVelocityEstimatedECEF[1] = cd.velocityEstimatedEcef().y;
            _gpsVelocityEstimatedECEF[2] = cd.velocityEstimatedEcef().z;
        }
        if(cd.hasPositionUncertaintyEstimated())
        {
            _positionEstimatedUncertainty = cd.positionUncertaintyEstimated();
        }
        if(cd.hasVelocityUncertaintyEstimated())
        {
            _velocityEstimatedUncertainty = cd.velocityUncertaintyEstimated();
        }
    }
}

void
VectorNavVN300Driver::binaryAsyncMessageReceived(void* userData, vn::protocol::uart::Packet& p, size_t index)
{
    VectorNavVN300Driver *driver = static_cast<VectorNavVN300Driver*>(userData);
    driver->binaryAsyncMessageReceivedFUNC(userData, p, index);
}


