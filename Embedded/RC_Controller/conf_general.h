
/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef CONF_GENERAL_H_
#define CONF_GENERAL_H_

#include "datatypes.h"

#define MAIN_MODE_VEHICLE 				0
#define MAIN_MODE_MOTE_2400			1
#define MAIN_MODE_MOTE_400			2
#define MAIN_MODE_MOTE_HYBRID		3 // Use 400 for slow and critical communication and 2400 for the rest.
#define MAIN_MODE_M8T_BASE_2400		5
#define MAIN_MODE_M8T_BASE_400		6

// Main mode
#ifndef MAIN_MODE
#define MAIN_MODE					MAIN_MODE_VEHICLE
#endif

// Mode macros
#define MAIN_MODE_IS_MOTE			(MAIN_MODE == MAIN_MODE_MOTE_2400 || MAIN_MODE == MAIN_MODE_MOTE_400 || MAIN_MODE == MAIN_MODE_MOTE_HYBRID)
#define MAIN_MODE_IS_VEHICLE		(MAIN_MODE == MAIN_MODE_VEHICLE)
#define MAIN_MODE_IS_BASE			(MAIN_MODE == MAIN_MODE_M8T_BASE_2400 || MAIN_MODE == MAIN_MODE_M8T_BASE_400)

// Firmware version
#define FW_VERSION_MAJOR			30
#define FW_VERSION_MINOR			1

// IO BOARD
// #define IO_BOARD

//#define IS_DRANGEN
#define IS_MACTRAC

// MacTrac
// Steering Center: 210
// Valve: Low values: Turn right; high values: turn left
#ifdef IS_MACTRAC
#define HAS_HYDRAULIC_DRIVE			1

#define SERVO_VESC_S1				178.0 // Left
#define SERVO_VESC_S2				240.0 // Right
#define USE_ADCONV_FOR_VIN
#define SERVO_VESC_HYDRAULIC
#define HYDRAULIC_HAS_SPEED_SENSOR
#define SERVO_VESC_ID				0
#define SERVO_VESC_INVERTED			0
#define SERVO_VESC_DEADBAND_COMP    0.2
#define IS_F9_BOARD					1
#define ADDIO
//#define IO_BOARD
#ifdef CAN_ADDIO
#define FTR2_ANGLE                  1
#endif
#define SERVO_WRITE
#define SERVO_READ
#endif


#ifdef IS_DRANGEN
#define DIFF_THROTTLE_VESC_LEFT 28
#define DIFF_THROTTLE_VESC_RIGHT 36
#define DIFF_STEERING 16
#define SERVO_VESC_ID				0
#define VESC_ID				0
#define VOLTAGEFRACTION 1
#define SERVO_VESC_S1				178.0 // Left
#define SERVO_VESC_S2				240.0 // Right
//#define SERVO_VESC_S1				-20.0 // Left
//#define SERVO_VESC_S2				40.0 // Right
#define USE_ADCONV_FOR_VIN
#define SERVO_VESC_HYDRAULIC
#define HYDRAULIC_HAS_SPEED_SENSOR
#define SERVO_VESC_INVERTED			0
#define SERVO_VESC_DEADBAND_COMP    0.2
#define IS_F9_BOARD					1
#define IS_ALL_ELECTRIC             1
#define ANALOG_ANGLE
#define SERVO_WRITE
#define SERVO_READ
//#define TACHOATCARD
#endif

// Differential steering
#ifndef HAS_DIFF_STEERING
#define HAS_DIFF_STEERING			0
#endif
#ifndef DIFF_STEERING_VESC_LEFT
#define DIFF_STEERING_VESC_LEFT		0
#endif
#ifndef DIFF_STEERING_VESC_RIGHT
#define DIFF_STEERING_VESC_RIGHT	1
#endif

// Hydraulic drive
#ifndef HAS_HYDRAULIC_DRIVE
#define HAS_HYDRAULIC_DRIVE			0
#endif

// VESC for steering
// ID of VESC for steering.
// -1: No steering VESC
#ifndef SERVO_VESC_ID
#define SERVO_VESC_ID				-1
#endif
/*
 * Angle should be increasing from S1 to S2 (possibly
 * passing 0). The steering mapping is done on top
 * of S1 and S2.
 */
#ifndef SERVO_VESC_S1
#define SERVO_VESC_S1				331.0
#endif
#ifndef SERVO_VESC_S2
#define SERVO_VESC_S2				30.0
#endif
#ifndef SERVO_VESC_P_GAIN
#define SERVO_VESC_P_GAIN			2.0
#endif
#ifndef SERVO_VESC_I_GAIN
#define SERVO_VESC_I_GAIN			1.0
#endif
#ifndef SERVO_VESC_D_GAIN
#define SERVO_VESC_D_GAIN			0.1
#endif
#ifndef SERVO_VESC_D_FILTER
#define SERVO_VESC_D_FILTER			0.05
#endif
#ifndef SERVO_VESC_INVERTED
#define SERVO_VESC_INVERTED			0
#endif
#ifndef SERVO_VESC_ANGLE_INVERTED
#define SERVO_VESC_ANGLE_INVERTED	0
#endif
#ifndef SERVO_VESC_DEADBAND_COMP
#define SERVO_VESC_DEADBAND_COMP	0.0 // Range 0 - 1
#endif

// Ublox settings
#ifndef UBLOX_EN
#define UBLOX_EN					1
#endif
#ifndef UBLOX_USE_PPS
#define UBLOX_USE_PPS				1
#endif

// External PPS signal for accurate time synchronization and delay compensation on PD4.
#ifndef GPS_EXT_PPS
#define GPS_EXT_PPS					0
#endif

// CAN settings
#define CAN_EN_DW					1
#define CAN_ADDIO                   1
#define CAN_IO_BOARD                0

// Log configuration to enable. Choose one only.
//#define LOG_EN_CARREL
//#define LOG_EN_ITRANSIT

// CC2520 Settings
#define CC2520_RF_CHANNEL			12
#define CC2520_PAN_ID				0xfa11
#define CC2520_NODE_ADDRESS			0x001
#define CC2520_DEST_ADDRESS			0xffff // 0xffff = broadcast

// General settings
#define ID_ALL						255
#define ID_VEHICLE_CLIENT				254 // Packet for vehicle client only
#ifndef VESC_ID
#define VESC_ID						ID_ALL // id, or ID_ALL for any VESC (not used in diff steering mode)
#endif
#define ID_MOTE						254 // If the packet is for the mote and not to be forwarded in mote mode

// vehicle parameters
#ifndef BOARD_YAW_ROT
#define BOARD_YAW_ROT				-90.0
#endif

// Servo settings
#define SERVO_OUT_RATE_HZ			50
#define SERVO_OUT_PULSE_MIN_US		1000
#define SERVO_OUT_PULSE_MAX_US		2000

// Autopilot settings
#define AP_ROUTE_SIZE				1000

// Board-dependent settings
#if IS_F9_BOARD
#define HAS_CC2520					0
#define HAS_CC1120					0
#define UBLOX_IS_F9P				1
#define LED_RED_GPIO				GPIOC
#define LED_RED_PIN					10
#define LED_GREEN_GPIO				GPIOC
#define LED_GREEN_PIN				11
#define CAN1_RX_GPIO				GPIOB
#define CAN1_RX_PIN					8
#define CAN1_TX_GPIO				GPIOB
#define CAN1_TX_PIN					9
#define HAS_BMI160					1
#define HAS_ID_SW					0
#define VIN_R1						39000.0
#define VIN_R2						2200.0
#else
#define HAS_CC2520					1
#define HAS_CC1120					1
//#define UBLOX_IS_F9P				0
#define LED_RED_GPIO				GPIOE
#define LED_RED_PIN					0
#define LED_GREEN_GPIO				GPIOE
#define LED_GREEN_PIN				1
#define CAN1_RX_GPIO				GPIOD
#define CAN1_RX_PIN					0
#define CAN1_TX_GPIO				GPIOD
#define CAN1_TX_PIN					1
#define HAS_BMI160					0
#define HAS_ID_SW					1
#define VIN_R1						10000.0
#define VIN_R2						1500.0
#endif

// Global variables
extern MAIN_CONFIG main_config;
extern int main_id;

// Functions
void conf_general_init(void);
void conf_general_get_default_main_config(MAIN_CONFIG *conf);
void conf_general_read_main_conf(MAIN_CONFIG *conf);
bool conf_general_store_main_config(MAIN_CONFIG *conf);

#endif /* CONF_GENERAL_H_ */
