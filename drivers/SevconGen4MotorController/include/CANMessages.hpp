/*
 * CANMessages.hpp
 *
 *  Created on: 2019-03-14
 *      Author: liam
 */

#ifndef DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANMESSAGES_HPP_
#define DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANMESSAGES_HPP_

/****************************************
 * INCLUDES
 ****************************************/

#include <string>

/****************************************
 * CONSTANTS
 ****************************************/
namespace IAV
{

constexpr int MotorNodeID = 1;

//CANopen Object Dictionary Operations
constexpr uint32_t TPDO1 = 0x180 + MotorNodeID;
constexpr uint32_t RPDO1 = 0x200 + MotorNodeID;
constexpr uint32_t RPDO_FAKEOUT = 0x200; //used to initialize motor controller in master mode

constexpr uint32_t TSDO = 0x580 + MotorNodeID;
constexpr uint32_t RSDO = 0x600 + MotorNodeID;

constexpr uint32_t NMT  = 0x000;
constexpr uint32_t SYNC = 0x080;
constexpr uint32_t EMC = 0x080 + MotorNodeID;
constexpr uint32_t HRBT = 0x700 + MotorNodeID;

/*******************INDEXES***********************/

//LINE CONTACTOR INFO
constexpr uint16_t CONTACTOR = 0x6C11;
constexpr uint16_t CONTACTOR_ON  = 0x0A00;
constexpr uint16_t CONTACTOR_OFF = 0x0000;
constexpr uint8_t CONTACTOR_SUBINDEX = 0x01;

//READ ANGULAR VELOCITY
constexpr uint16_t ANGULAR_VELOCITY = 0x606C;
constexpr uint8_t ANGULAR_VELOCITY_SUBINDEX = 0x00;

//OPERATIONAL STATE IN MASTER MODE
constexpr uint16_t STATE_MASTER = 0x2800;
constexpr uint8_t STATE_MASTER_SUBINDEX = 0x00;
constexpr uint8_t PREOP_MASTER = 0X01;
constexpr uint8_t OP_MASTER = 0x00;

//CONTROL WORD (BRIDGE)
constexpr uint16_t BRIDGE = 0x6040;
constexpr uint8_t BRIDGE_SUBINDEX = 0x00;
constexpr uint16_t BRIDGE_ON = 0x000F;
constexpr uint16_t BRIDGE_OFF = 0x8007;

//MAXIMUM TORQUE
constexpr uint16_t MAX_TORQUE = 0x6072;
constexpr uint8_t MAX_TORQUE_SUBINDEX = 0x00;

//TARGET SPEED
constexpr uint16_t TARGET_SPEED = 0x60FF;
constexpr uint8_t TARGET_SPEED_SUBINDEX = 0x00;

//MOTOR CONTROLLER LOGIN INFO
constexpr uint16_t LOGIN_ENTRY = 0x5000;
constexpr uint8_t LOGIN_ACCESS_LEVEL_SUBINDEX = 0x01;
constexpr uint8_t LOGIN_PASSWORD_SUBINDEX = 0x02;
constexpr uint8_t LOGIN_USER_ID_SUBINDEX = 0x03;
constexpr uint16_t LOGIN_LEVEL_5 = 0x968C;

//MOTOR CONTROLLER PASSWORD KEY
constexpr uint16_t PASSWORD_KEY = 0x5001;
constexpr uint8_t PASSWORD_KEY_SUBINDEX = 0x00;

/****************************************
 * ENUM
 ****************************************/

enum CANstate
{
    OP      = 0x05,
    PREOP   = 0x7F,
    INIT    = 0x00,
    STOPPED = 0x04,
    OFF     = -1
};

enum sdoCommandBytes
{
    READ_RESPONSE_4BYTES = 0x43,
    READ_RESPONSE_3BYTES = 0x47,
    READ_RESPONSE_2BYTES = 0x4B,
    READ_RESPONSE_1BYTE  = 0x4F,
    READ_REQUEST         = 0x40,

    WRITE_REQUEST_4BYTES = 0x23,
    WRITE_REQUEST_3BYTES = 0x27,
    WRITE_REQUEST_2BYTES = 0x2B,
    WRITE_REQUEST_1BYTE  = 0x2F,
    WRITE_REPSONSE       = 0x60,

    ERROR_REPONSE        = 0x80
};

/****************************************
 * STRUCT
 ****************************************/

struct CANmsg
{
    uint32_t identity;
    uint8_t data_length;
    uint8_t data[8] __attribute__((aligned(8)));
    volatile bool bufferFull;
    volatile bool syncFlag;
};


}//End namespace

#endif /* DRIVERS_SEVCONGEN4MOTORCONTROLLER_COMP_INCLUDE_CANMESSAGES_HPP_ */
