/*****************************************************************
 Copyright (c) 2020, Unitree Robotics.Co.Ltd. All rights reserved.
******************************************************************/

#ifndef _MOTOR_CRC_H_
#define _MOTOR_CRC_H_

#include <stdint.h>
#include <array>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/low_cmd.hpp"
#include "unitree_go/msg/motor_cmd.hpp"
#include "unitree_go/msg/bms_cmd.hpp"

constexpr int HIGHLEVEL = 0xee;
constexpr int LOWLEVEL = 0xff;
constexpr int TRIGERLEVEL = 0xf0;
constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

// joint index
constexpr int FR_0 = 0;
constexpr int FR_1 = 1;
constexpr int FR_2 = 2;

constexpr int FL_0 = 3;
constexpr int FL_1 = 4;
constexpr int FL_2 = 5;

constexpr int RR_0 = 6;
constexpr int RR_1 = 7;
constexpr int RR_2 = 8;

constexpr int RL_0 = 9;
constexpr int RL_1 = 10;
constexpr int RL_2 = 11;

/**
 * @struct BmsCmd
 * @brief Structure for Battery Management System (BMS) command.
 *
 * This structure is used to define commands related to the Battery Management System.
 */
typedef struct
{
	uint8_t off;					/**< Off command identifier, typically set to 0xA5. */
	std::array<uint8_t, 3> reserve; /**< Reserved bytes for future use. */
} BmsCmd;

/**
 * @struct MotorCmd
 * @brief Structure for Motor command.
 *
 * This structure contains the command parameters for controlling a motor.
 */
typedef struct
{
	uint8_t mode; // desired working mode
	float q;	  // desired angle (unit: radian)
	float dq;	  // desired velocity (unit: radian/second)
	float tau;	  // desired output torque (unit: N.m)
	float Kp;	  // desired position stiffness (unit: N.m/rad )
	float Kd;	  // desired velocity stiffness (unit: N.m/(rad/s) )
	std::array<uint32_t, 3> reserve;
} MotorCmd; // motor control

/**
 * @struct LowCmd
 * @brief Structure for Low level command.
 *
 * This structure defines the low-level command message sent to the motor controllers,
 * including motor commands, BMS commands, and additional system information.
 */
typedef struct
{
	std::array<uint8_t, 2> head; /**< Header bytes of the command message. */
	uint8_t levelFlag;			 /**< Command level flag. */
	uint8_t frameReserve;		 /**< Reserved byte for frame alignment. */

	std::array<uint32_t, 2> SN;				/**< Serial numbers. */
	std::array<uint32_t, 2> version;		/**< Firmware versions. */
	uint16_t bandWidth;						/**< Bandwidth information. */
	std::array<MotorCmd, 20> motorCmd;		/**< Array of motor commands. */
	BmsCmd bms;								/**< Battery Management System command. */
	std::array<uint8_t, 40> wirelessRemote; /**< Wireless remote control data. */
	std::array<uint8_t, 12> led;			/**< LED control data. */
	std::array<uint8_t, 2> fan;				/**< Fan control data. */
	uint8_t gpio;							/**< GPIO control data. */
	uint32_t reserve;						/**< Reserved bytes for future use. */

	uint32_t crc; /**< CRC value for error checking. */
} LowCmd;

/**
 * @brief Computes the CRC32 checksum for a given buffer.
 *
 * @param ptr Pointer to the buffer containing data.
 * @param len Length of the data in bytes.
 * @return The computed CRC32 checksum.
 */
uint32_t crc32_core(uint32_t *ptr, uint32_t len);

/**
 * @brief Computes and sets the CRC for a LowCmd message.
 *
 * @param msg Reference to a LowCmd message.
 */
void get_crc(unitree_go::msg::LowCmd &msg);

#endif