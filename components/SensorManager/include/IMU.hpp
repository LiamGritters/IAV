/*
 * IMU.hpp
 *
 *  Created on: 2019-01-22
 *      Author: liam
 */

#ifndef COMPONENTS_SENSORMANAGER_INCLUDE_IMU_HPP_
#define COMPONENTS_SENSORMANAGER_INCLUDE_IMU_HPP_

/****************************************
 * INCLUDES
 ****************************************/

/****************************************
 * CLASS DEFINITION
 ****************************************/

class IMU
{

public:

    IMU()
    {
        roll = 0.0;
        pitch = 0.0;
        yaw = 0.0;
        xAcceleration = 0.0;
        yAcceleration = 0.0;
        zAcceleration = 0.0;
    }

public:

    float roll;
    float pitch;
    float yaw;
    float xAcceleration;
    float yAcceleration;
    float zAcceleration;

};




#endif /* COMPONENTS_SENSORMANAGER_INCLUDE_IMU_HPP_ */
