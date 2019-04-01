/*
 * LinearActuatorControlValues.hpp
 *
 *  Created on: 2019-03-30
 *      Author: liam
 */

#ifndef COMPONENTS_CONTROLSMANAGER_INCLUDE_LINEARACTUATORCONTROLVALUES_HPP_
#define COMPONENTS_CONTROLSMANAGER_INCLUDE_LINEARACTUATORCONTROLVALUES_HPP_

/**
 * Used to convert the desired wheel angles to the linear actuator position on the buggy and vice-versa
 * Credit: Ryan Kidney
 */

/****************************************
 * INCLUDES
 ****************************************/

#include <vector>

/****************************************
 * ANGLES
 ****************************************/

const std::vector<float> WheelAngles
 {
//
//     -45,
//     -40,
//     -35,
//     -30,
     -25,
     -20,
     -15,
     -10,
     -5,
     0,
     5,
     10,
     15,
     20,
     25,
//     30,
//     35,
//     40,
//     45
 };

/****************************************
 * POSITONS
 ****************************************/

const std::vector<float> ActuatorPositions
 {
//     -0.09,
//     -0.08,
//     -0.07,
//     -0.06,
     -0.05,
     -0.04,
     -0.03,
     -0.02,
     -0.01,
     0.00,
     0.01,
     0.02,
     0.03,
     0.04,
     0.05,
//     0.06,
//     0.07,
//     0.08,
//     0.09
 };

#endif /* COMPONENTS_CONTROLSMANAGER_INCLUDE_LINEARACTUATORCONTROLVALUES_HPP_ */
