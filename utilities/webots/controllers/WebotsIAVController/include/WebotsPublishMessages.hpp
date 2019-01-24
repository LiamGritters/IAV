/*
 * WebotsPublishMessages.hpp
 *
 *  Created on: 2018-11-22
 *      Author: liam
 */

#ifndef UTILITIES_WEBOTS_CONTROLLERS_WEBOTSIAVCONTROLLER_INCLUDE_WEBOTSPUBLISHMESSAGES_HPP_
#define UTILITIES_WEBOTS_CONTROLLERS_WEBOTSIAVCONTROLLER_INCLUDE_WEBOTSPUBLISHMESSAGES_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "WebotsSensorManager.hpp"

#include <lcm/lcm-cpp.hpp>
#include <atomic>

/****************************************
 * FUNCTION DEFINITONS
 ****************************************/

void PublishData(lcm::LCM *lcm, WebotsSensorManager *sensors, std::atomic<bool>& isRunning);

void PublishLidarMessage(lcm::LCM *lcm, WebotsSensorManager *sensors);

void PublishIMUMessage(lcm::LCM *lcm, WebotsSensorManager *sensors);

void PublishGPSMessage(lcm::LCM *lcm, WebotsSensorManager *sensors);

#endif /* UTILITIES_WEBOTS_CONTROLLERS_WEBOTSIAVCONTROLLER_INCLUDE_WEBOTSPUBLISHMESSAGES_HPP_ */
