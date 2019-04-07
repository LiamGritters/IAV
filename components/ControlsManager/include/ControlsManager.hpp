/*
 * ControlsManager.hpp
 *
 *  Created on: 2019-04-02
 *      Author: liam
 */

#ifndef COMPONENTS_CONTROLSMANAGER_INCLUDE_CONTROLSMANAGER_HPP_
#define COMPONENTS_CONTROLSMANAGER_INCLUDE_CONTROLSMANAGER_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include "vehicle_controller_demand_t.hpp"
#include "driveline_t.hpp"

#include "LinearActuatorControlValues.hpp"

#include <lcm/lcm-cpp.hpp>
#include <thread>

/****************************************
 * CONSTANTS
 ****************************************/



/****************************************
 * CLASS DEFINITION
 ****************************************/

class ControlsManager
{
    public:
        ControlsManager();
        ~ControlsManager();

        bool Initialize();
        void Start();
        void Stop();

        inline bool IsRunning() const {return _isRunning;};
        inline bool IsMsgEnabled() const {return _msgEnabled;};
        inline float GetVelocity() const {return _velocity;};
        inline float GetTurningAngle() const {return _turningAngle;};
        inline uint64_t GetTimestamp() const {return _timestamp;};


    private:

        static void readMessage(ControlsManager* controls);

        void handleControlInputsMessage(const lcm::ReceiveBuffer* rbuf, const std::string& chan, const iav_lcm::vehicle_controller_demand_t* msg);


    private:

        bool       _isRunning;
        bool       _msgEnabled;
        float      _velocity;
        float      _turningAngle;
        int64_t    _timestamp;

        std::thread _thread;

        lcm::LCM _lcm;
};



#endif /* COMPONENTS_CONTROLSMANAGER_INCLUDE_CONTROLSMANAGER_HPP_ */
