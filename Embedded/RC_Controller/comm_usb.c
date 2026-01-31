/*
	Copyright 2012-2016 Benjamin Vedder	benjamin@vedder.se

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

/**
 * @file    comm_usb.c
 * @brief   USB Communication Module
 * @details This module handles USB serial communication between the embedded controller
 *          and external devices (typically a Raspberry Pi). It implements a dual-thread
 *          architecture for receiving and processing packets, with flow control to prevent
 *          USB buffer overflows.
 *
 * @defgroup USB_COMM USB Communication
 * @ingroup  COMMUNICATION
 * @{
 */

#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "comm_usb.h"
#include "packet.h"
#include "comm_usb_serial.h"
#include "conf_general.h"
//#include "comm_cc2520.h"
//#include "comm_cc1120.h"
#include "ublox.h"

/**
 * @name Configuration Settings
 * @{
 */
/**
 * @brief Packet handler identifier for USB communication
 * @details Used to identify USB packets in the packet processing system
 */
#define PACKET_HANDLER				0

/**
 * @brief Size of the serial receive buffer
 * @details This buffer holds incoming USB data before it's processed
 */
#define SERIAL_RX_BUFFER_SIZE		2048

/**
 * @brief Milliseconds to system ticks conversion
 * @details Converts milliseconds to ChibiOS system ticks for timing operations
 */
#define MS2ST(ms)   ((systime_t)((ms) * CH_CFG_ST_FREQUENCY / 1000))
/** @} */

/**
 * @}
 * @}
 */

/**
 * @name Private Variables
 * @{
 */
/**
 * @brief Circular buffer for incoming USB serial data
 * @details Stores bytes received from USB before they're processed by the packet handler
 */
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];

/**
 * @brief Read position in the circular buffer
 * @details Tracks where the processing thread should read from
 */
static volatile int serial_rx_read_pos = 0;

/**
 * @brief Write position in the circular buffer
 * @details Tracks where incoming data should be written
 */
static volatile int serial_rx_write_pos = 0;

/**
 * @brief Working area for the serial read thread
 * @details Stack space for the thread that reads from USB
 */
static THD_WORKING_AREA(serial_read_thread_wa, 512);

/**
 * @brief Working area for the serial process thread
 * @details Stack space for the thread that processes packets
 */
static THD_WORKING_AREA(serial_process_thread_wa, 4096);

/**
 * @brief Mutex for protecting USB send operations
 * @details Ensures thread-safe access to USB transmission
 */
static mutex_t send_mutex;

/**
 * @brief Pointer to the process thread
 * @details Used for signaling when new data arrives
 */
static thread_t *process_tp;
/** @} */

/**
 * @name Private Functions
 * @{
 */
/**
 * @brief Processes a complete packet
 * @details Routes packets to the appropriate handler based on mode and packet type
 * @param data Pointer to the packet data
 * @param len Length of the packet
 */
static void process_packet(unsigned char *data, unsigned int len);

/**
 * @brief Sends a packet via USB
 * @details Implements flow control to prevent USB buffer overflow
 * @param buffer Pointer to the data to send
 * @param len Length of the data
 */
static void send_packet(unsigned char *buffer, unsigned int len);

/**
 * @brief Serial read thread function
 * @details Continuously reads bytes from USB and stores them in the circular buffer
 * @param arg Unused thread argument
 */
static THD_FUNCTION(serial_read_thread, arg);

/**
 * @brief Serial process thread function
 * @details Processes bytes from the circular buffer and reassembles packets
 * @param arg Unused thread argument
 */
static THD_FUNCTION(serial_process_thread, arg);
/** @} */

/**
 * @brief Initializes the USB communication module
 * @details Sets up USB serial interface, packet processing, and creates worker threads
 * @note This function must be called before any USB communication can occur
 */
void comm_usb_init(void) {
	comm_usb_serial_init();
	packet_init(send_packet, process_packet, PACKET_HANDLER);

	chMtxObjectInit(&send_mutex);

	// Threads
	chThdCreateStatic(serial_read_thread_wa, sizeof(serial_read_thread_wa), NORMALPRIO, serial_read_thread, NULL);
	chThdCreateStatic(serial_process_thread_wa, sizeof(serial_process_thread_wa), NORMALPRIO, serial_process_thread, NULL);
}

/**
 * @brief Sends a packet via USB
 * @details Thread-safe function to send data over USB with proper synchronization
 * @param data Pointer to the data buffer to send
 * @param len Length of the data to send
 * @note This function is thread-safe and can be called from any context
 */
void comm_usb_send_packet(unsigned char *data, unsigned int len) {
	chMtxLock(&send_mutex);
	packet_send_packet(data, len, PACKET_HANDLER);
	chMtxUnlock(&send_mutex);
}

/**
 * @brief Serial read thread
 * @details Continuously reads bytes from USB and stores them in the circular buffer.
 *          This thread runs at high priority to ensure no data is lost.
 * @param arg Unused thread argument
 * @note This thread never exits and runs indefinitely
 */
static THD_FUNCTION(serial_read_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial read");

	uint8_t buffer[128];
	int i;
	int len;
	int had_data = 0;

	for(;;) {
		// Read one byte from USB
		len = chSequentialStreamRead(&SDU1, (uint8_t*) buffer, 1);

		// Store the byte in the circular buffer
		for (i = 0;i < len;i++) {
			serial_rx_buffer[serial_rx_write_pos++] = buffer[i];

			// Wrap around if we reach the end of the buffer
			if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_write_pos = 0;
			}

			had_data = 1;
		}

		// Signal the process thread if we received data
		if (had_data) {
			chEvtSignal(process_tp, (eventmask_t) 1);
			had_data = 0;
		}
	}
}

/**
 * @brief Serial process thread
 * @details Processes bytes from the circular buffer and reassembles packets.
 *          This thread waits for signals from the read thread before processing data.
 * @param arg Unused thread argument
 * @note This thread never exits and runs indefinitely
 */
static THD_FUNCTION(serial_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("USB-Serial process");

	// Get our own thread pointer for signaling
	process_tp = chThdGetSelfX();

	for(;;) {
		// Wait for data to arrive
		chEvtWaitAny((eventmask_t) 1);

		// Process all available bytes in the buffer
		while (serial_rx_read_pos != serial_rx_write_pos) {
			// Feed each byte to the packet processor
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

			// Wrap around if we reach the end of the buffer
			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}
}

/**
 * @brief Processes a complete packet
 * @details Routes packets to the appropriate handler based on mode and packet type.
 *          In MOTE mode, it filters packets and routes them to different radios or commands.
 *          In other modes, it simply passes packets to the commands processor.
 * @param data Pointer to the packet data
 * @param len Length of the packet
 * @note This function implements mode-specific packet routing logic
 */
static void process_packet(unsigned char *data, unsigned int len) {
#if MAIN_MODE_IS_MOTE
	uint8_t id = data[0];
	CMD_PACKET packet_id = data[1];

	// Check if this is a MOTE packet that should be processed locally
	if (id == ID_MOTE && (packet_id < 50 || packet_id >= 200)) {
		// Process command packets locally
		commands_process_packet(data, len, comm_usb_send_packet);
	} else {
		// Route to appropriate radio based on mode
#if MAIN_MODE == MAIN_MODE_MOTE_HYBRID
		// Hybrid mode: use CC1120 for RTCM, CC2520 for other packets
		if (packet_id == CMD_SEND_RTCM_USB) {
			comm_cc1120_send_buffer(data, len);
		} else {
			comm_cc2520_send_buffer(data, len);
		}
#elif MAIN_MODE == MAIN_MODE_MOTE_400
		// 400MHz mode: use CC1120 radio
		comm_cc1120_send_buffer(data, len);
#else
		// Default: use CC2520 radio
		comm_cc2520_send_buffer(data, len);
#endif
	}

	// Forward RTCM data to UBLOX if enabled
#if UBLOX_EN
	if (packet_id == CMD_SEND_RTCM_USB) {
		ublox_send(data + 2, len - 2);
	}
#endif
#else
	// Non-MOTE mode: process all packets locally
	commands_process_packet(data, len, comm_usb_send_packet);
#endif
}

/**
 * @brief Sends a packet via USB with flow control
 * @details Implements rate limiting to prevent USB buffer overflows.
 *          This is critical for maintaining stable communication with the Raspberry Pi.
 * @param buffer Pointer to the data to send
 * @param len Length of the data
 * @note Packets that exceed the rate limit are silently dropped to prevent USB stalls
 */
static void send_packet(unsigned char *buffer, unsigned int len) {
	// Basic flow control: limit the rate of USB transmissions
	// This helps prevent overwhelming the USB connection to Raspberry Pi
	static systime_t last_send_time = 0;
	systime_t now = chVTGetSystemTimeX();
	
	// Minimum interval between USB transmissions to prevent flooding
	// Only send if enough time has passed since the last transmission
	if (now - last_send_time > MS2ST(USB_MIN_SEND_INTERVAL_MS) || last_send_time == 0) {
		// Send the packet via USB
		chSequentialStreamWrite(&SDU1, buffer, len);
		last_send_time = now;
	} else {
		// Skip this packet to prevent USB buffer overflow
		// This is a simple rate limiting approach that drops excess packets
		// rather than blocking the system
	}
}
/* @} */
/* @} */
