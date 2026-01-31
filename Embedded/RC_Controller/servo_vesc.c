/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "servo_vesc.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "bldc_interface.h"
#include "terminal.h"
#include "utils.h"
#include "comm_can.h"
#include "commands.h"
#include <math.h>
#include "autopilot.h"

/**
 * SPI Configuration for AS5047 Encoder
 * 
 * These definitions configure the GPIO pins used for SPI communication
 * with the AS5047 magnetic encoder. The implementation uses bit-banging
 * for SPI communication to avoid using the hardware SPI peripheral.
 */
#define SPI_SW_MISO_GPIO			GPIOC  /**< Master In Slave Out - Input from encoder */
#define SPI_SW_MISO_PIN				11     /**< Pin number for MISO */
#define SPI_SW_MOSI_GPIO			GPIOC  /**< Master Out Slave In - Output to encoder */
#define SPI_SW_MOSI_PIN				12     /**< Pin number for MOSI */
#define SPI_SW_SCK_GPIO				GPIOC  /**< Serial Clock - Output to encoder */
#define SPI_SW_SCK_PIN				10     /**< Pin number for SCK */
#define SPI_SW_CS_GPIO				GPIOD  /**< Chip Select - Output to encoder */
#define SPI_SW_CS_PIN				2      /**< Pin number for CS */

// Private functions
static void terminal_state(int argc, const char **argv);
static float as5047_read(bool *ok);
static bool spi_check_parity(uint16_t x);
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length);
static void spi_begin(void);
static void spi_end(void);
static void spi_delay(void);

/**
 * Private Variables
 * 
 * These variables store the state of the servo controller.
 * They are static to ensure they persist across function calls
 * and are only accessible within this module.
 */
static float m_pos_set = 0.5;           /**< Target position (0.0 to 1.0) */
static float m_pos_now = 0.0;           /**< Current measured position (0.0 to 1.0) */
static float m_pos_now_raw = 0.0;       /**< Raw encoder reading in degrees */
static int m_not_ok_cnt = 0;            /**< Counter for consecutive sensor failures */
static float m_out_last = 0.0;          /**< Last output value sent to actuator */

/**
 * Debug and Output Variables
 * 
 * These variables are exported for debugging purposes and can be
 * accessed from other modules to monitor the servo controller's
 * internal state.
 */
float servo_output;                     /**< Output value for external use */
extern float debugvalue;                /**< Raw encoder position */
extern float debugvalue2;               /**< P-term of PID controller */
extern float debugvalue3;               /**< I-term of PID controller */
extern float debugvalue4;               /**< D-term of PID controller */
extern float debugvalue5;               /**< Deadband compensation */
extern float debugvalue13;              /**< Target position */
extern float debugvalue14;              /**< Current position */
extern float debugvalue15;              /**< Position error */
extern int iDebug;                      /**< Debug mode selector */
extern bool m_is_active;                /**< Autopilot active flag */
extern bool m_kb_active;                /**< Keyboard control active flag */
extern MAIN_CONFIG main_config;         /**< Main configuration structure */
float i_term;                          /**< Integral term of PID controller (persists between calls) */

/**
 * Thread Configuration
 * 
 * The servo control thread runs at normal priority and has a stack size
 * of 1024 bytes. It implements a 100Hz control loop for position control.
 */
static THD_WORKING_AREA(servo_thread_wa, 1024);  /**< Thread working area */
static THD_FUNCTION(servo_thread, arg);           /**< Thread function prototype */

/**
 * Initialize the VESC servo controller
 * 
 * This function sets up the hardware interfaces, creates the control thread,
 * and registers the terminal command for debugging. It should be called
 * during system initialization.
 */
void servo_vesc_init(void) {
	// Configure GPIO pins for SPI communication
	palSetPadMode(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN, PAL_MODE_INPUT);
	palSetPadMode(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPadMode(SPI_SW_CS_GPIO, SPI_SW_CS_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);

	// Set MOSI to 1 (idle state for SPI)
	palSetPadMode(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
	palSetPad(SPI_SW_MOSI_GPIO, SPI_SW_MOSI_PIN);

	// Create the servo control thread
	chThdCreateStatic(servo_thread_wa, sizeof(servo_thread_wa), NORMALPRIO, servo_thread, NULL);

	// Register terminal command for debugging
	terminal_register_command_callback(
			"servo_vesc_state",
			"Print the state of the VESC servo for 30 seconds",
			"",
			terminal_state);

	// Initialize integral term to zero
	i_term = 0.0;
}

/**
 * Set the target position for the servo
 * 
 * @param pos Target position in range [0.0, 1.0]
 *            Values are truncated to this range if outside
 */
void servo_vesc_set_pos(float pos) {
	utils_truncate_number(&pos, 0.0, 1.0);
	m_pos_set = pos;
}

/**
 * Get the current measured position
 * 
 * @return Current position in range [0.0, 1.0]
 */
float servo_vesc_get_pos(void) {
	return m_pos_now;
}

/**
 * Reset the sensor fault counter
 * 
 * This function resets the counter that tracks consecutive sensor
 * failures. Should be called after a fault condition is resolved.
 */
void servo_vesc_reset_fault(void) {
	m_not_ok_cnt = 0;
}

/**
 * Get the target position
 * 
 * @return Target position in range [0.0, 1.0]
 */
float servo_vesc_get_pos_set(void) {
	return m_pos_set;
}

/**
 * Servo Control Thread
 * 
 * This is the main control loop that runs at 100Hz (10ms interval).
 * It implements a PID controller to maintain the desired position.
 * 
 * @param arg Unused thread argument
 */
static THD_FUNCTION(servo_thread, arg) {
	(void)arg;

	chRegSetThreadName("Servo VESC");

	// Initialize control loop state variables
	i_term = 0.0;                    /**< Integral term (persists between iterations) */
	float prev_error = 0.0;          /**< Previous error for derivative calculation */
	float dt_int = 0.0;              /**< Integrated time for derivative calculation */
	float d_filter = 0.0;            /**< Filtered derivative term */

	// Wait 5 seconds before starting control loop
	// This allows other systems to initialize first
	chThdSleepMilliseconds(5000);

	for(;;) {

	// TODO: Consider enabling this condition for better resource management
	// if (m_is_active || m_kb_active) - Only run when autopilot or keyboard control is active
	if (1)  // Always run for now
	{

		// Reset integral term when not in active mode
		// This prevents wind-up when the system is not controlling
		if (!m_is_active)
		{
			i_term = 0;
		}
		/**
		 * Read Position from Sensor
		 * 
		 * Depending on configuration, read position from either:
		 * - ADDIO board (via CAN)
		 * - IO_BOARD (via CAN)
		 * - Direct AS5047 encoder (via SPI)
		 */
		// Map s1 to 0.0 and s2 to 1.0
#ifdef SERVO_VESC_HYDRAULIC
		// Set VESC ID for CAN communication
		comm_can_set_vesc_id(SERVO_VESC_ID);

		// Prevent unused variable warning
		(void)as5047_read;
		
		#ifdef ADDIO
			// Read position from ADDIO board via CAN
			// Note: Offset of 8.0 degrees was tried but removed
			float pos_addio = comm_can_ftr2_angle();
			debugvalue = pos_addio;

			// Always assume valid reading from ADDIO
			bool ok = true;
			m_pos_now_raw = pos_addio;
		#elif defined(IO_BOARD)
			// Read position from IO_BOARD via CAN
			float pos_io_board = comm_can_io_board_as5047_angle();

			// Check if position changed (simple validity check)
			bool ok = pos_io_board != m_pos_now_raw;
			m_pos_now_raw = pos_io_board;
		#else
			// No hydraulic interface defined
			bool ok = false;
		#endif
#else
		// Non-hydraulic mode: read directly from AS5047 encoder
		bool ok = false;
		m_pos_now_raw = as5047_read(&ok);
#endif

		/**
		 * Convert Raw Encoder Reading to Normalized Position
		 * 
		 * The encoder provides absolute angle in degrees. We need to:
		 * 1. Subtract the left limit (S1) to get relative position
		 * 2. Handle angle wrapping at 360 degrees
		 * 3. Map to normalized range [0.0, 1.0]
		 */
		float pos = m_pos_now_raw;
		pos -= SERVO_VESC_S1;  /**< Convert to relative position from left limit */

		// Allow some margin after limit without wrapping around
		// This prevents issues when the encoder is near the limit
		// TODO: Verify if this approach is optimal
		
		if (pos < -20) {
			utils_norm_angle_360(&pos);  /**< Normalize angle to [0, 360) range */
		}

		// Calculate the valid range width
		float end = SERVO_VESC_S2 - SERVO_VESC_S1;
		utils_norm_angle_360(&end);  /**< Ensure end is in valid range */
		
		// Map from [S1, S2] degrees to [0.0, 1.0] normalized
		m_pos_now = utils_map(pos, 0.0, end, 0.0, 1.0);
		/**
		 * PID Controller Implementation
		 * 
		 * Calculate the error and compute P, I, D terms for position control
		 */
		// Run PID-controller on the output
		float error = m_pos_set - m_pos_now;  /**< Error = target - current */
		
		// Export values for debugging
		debugvalue13 = m_pos_set;    /**< Target position */
		debugvalue14 = m_pos_now;    /**< Current position */
		debugvalue15 = error;        /**< Position error */
		
		// Debug output when iDebug == 81
		if (iDebug == 81)
		{
			commands_printf("m_pos_set: %f", m_pos_set);
			commands_printf("m_pos_now: %f", m_pos_now);
			commands_printf("error: %f", error);
		}
		
		// Control loop time step (10ms = 0.01s)
		float dt = 0.01;
		
		// Proportional term: proportional to current error
		float p_term = error * main_config.vehicle.vesc_p_gain;
		
		// Integral term: sum of errors over time
		// This provides correction for steady-state errors
		i_term += error * (main_config.vehicle.vesc_i_gain * dt);

		/**
		 * Derivative Term Calculation
		 * 
		 * The derivative term provides damping based on the rate of change of error.
		 * Special handling is needed when the error doesn't change (e.g., at low
		 * speeds or when position resolution is limited).
		 * 
		 * When error is constant, we accumulate the time (dt_int) to get a better
		 * estimate of the derivative when it finally changes.
		 */
		float d_term = 0.0;
		
		// Accumulate time for derivative calculation
		dt_int += dt;
		
		// Check if error has changed
		if (error == prev_error) {
			// Error hasn't changed, set derivative to zero
			// but keep accumulating time for better future estimate
			d_term = 0.0;
		} else {
			// Error has changed, calculate derivative
			// d_term = (change in error) / (change in time)
			d_term = (error - prev_error) * (main_config.vehicle.vesc_d_gain / dt_int);
			
			// Reset accumulated time after calculating derivative
			dt_int = 0.0;
		}

		// Apply low-pass filter to derivative term for smoothing
		// This reduces noise in the derivative calculation
		UTILS_LP_FAST(d_filter, d_term, SERVO_VESC_D_FILTER);
		d_term = d_filter;

		/**
		 * PID Output Calculation and Wind-up Protection
		 * 
		 * Combine P, I, D terms and apply limits to prevent wind-up and ensure
		 * output stays within valid range [-1.0, 1.0].
		 */
		// I-term wind-up protection
		// Limit P term to prevent excessive integral wind-up
		utils_truncate_number_abs(&p_term, 1.0);
		
		// Limit I term based on remaining capacity after P term
		// This prevents the sum from exceeding the output limits
		utils_truncate_number_abs(&i_term, 1.0 - fabsf(p_term));

		// Store previous error for next iteration
		prev_error = error;

		// Calculate combined PID output
		float output = p_term + i_term + d_term;
		
		// Debug output when iDebug == 80
		if (iDebug == 80)
		{
			commands_printf("i gain: %f", main_config.vehicle.vesc_i_gain);
			commands_printf("Deadband: %f", main_config.vehicle.deadband);
		}
		
		// Add deadband compensation to overcome stiction
		// Deadband is added in the direction of motion
		output += SIGN(output) * main_config.vehicle.deadband;
		
		// Export PID terms for debugging
		debugvalue2 = p_term;                    /**< P-term */
		debugvalue3 = i_term;                    /**< I-term */
		debugvalue4 = d_term;                    /**< D-term */
		debugvalue5 = SIGN(output) * main_config.vehicle.deadband;  /**< Deadband compensation */
		
		// Ensure final output is within valid range
		utils_truncate_number(&output, -1.0, 1.0);

		/**
		 * Sensor Fault Handling
		 * 
		 * Track consecutive sensor failures. If too many failures occur,
		 * the system will stop sending commands to prevent unsafe operation.
		 */
		if (ok) {
			// Sensor reading was valid, reset fault counter
			m_not_ok_cnt = 0;
		} else {
			// Sensor reading failed, increment fault counter
			// #define INGENVINKELGIVARE - Uncomment for testing (disables fault counting)
#ifndef INGENVINKELGIVARE
			m_not_ok_cnt++;
#else
			m_not_ok_cnt = 0;  // FOR TESTING, REMOVE!!! - Disables fault detection
#endif
		}

		/**
		 * Output to Actuator
		 * 
		 * Send the calculated output to the actuator. The behavior depends
		 * on whether hydraulic mode is enabled and whether sensor faults
		 * have occurred.
		 * 
		 * Fault tolerance: If more than 100 consecutive sensor failures occur,
		 * the system stops sending commands to prevent unsafe operation.
		 */
#ifdef SERVO_VESC_HYDRAULIC
		if (m_not_ok_cnt < 100) {
			// No faults: send calculated output
			
			// Invert output if configured
			float output_scaled = SERVO_VESC_INVERTED ? output : -output;
			
			// Scale output (0.75 factor may be for safety margin)
			output_scaled *= 0.75;
			
			// Convert from [-1.0, 1.0] to [0.0, 1.0] range for valve control
			m_out_last = (output_scaled + 1.0) / 2.0;
			//servo_output = m_out_last;  // Uncomment if needed for external use
			
			// Send command via appropriate interface
			#ifdef ADDIO
				comm_can_addio_set_valve_duty(m_out_last);
			#elif defined(IO_BOARD)
				comm_can_io_board_set_pwm_duty(0, m_out_last);
			#endif
		} else {
			// Too many faults: set to safe middle position (0.5 = 50%)
			#ifdef ADDIO
				comm_can_addio_set_valve_duty(0.5);
			#elif defined(IO_BOARD)
				comm_can_io_board_set_pwm_duty(0, 0.5);
			#endif
		}
		
#else
		// Non-hydraulic mode: direct VESC control
		if (m_not_ok_cnt < 100) {
			// No faults: send calculated output
			m_out_last = SERVO_VESC_INVERTED ? output : -output;
			bldc_interface_set_duty_cycle(m_out_last);
		} else {
			// Too many faults: stop motor (commented out for safety)
			// bldc_interface_set_current(0.0);
			// bldc_interface_set_current(0.1);
		}
#endif
	} else
	{
		// Not in active mode: reset integral term to prevent wind-up
		i_term = 0;
	}
		
	// Sleep for 10ms to maintain 100Hz control loop frequency
	chThdSleepMilliseconds(10);
	}
}

/**
 * Terminal Command: servo_vesc_state
 * 
 * This function prints the servo state for 30 seconds (300 iterations at 100ms each).
 * It's useful for debugging and monitoring the servo controller's operation.
 * 
 * @param argc Number of arguments (unused)
 * @param argv Array of arguments (unused)
 */
static void terminal_state(int argc, const char **argv) {
	(void)argc;
	(void)argv;

	// Print servo state for 30 seconds
	for (int i = 0; i < 300; i++) {
		// Print: Raw encoder reading, processed position, fault counter, output value
		commands_printf("Raw: %.1f Processed: %.3f Not OK: %d: Out: %.3f",
				(double)m_pos_now_raw, (double)m_pos_now, m_not_ok_cnt, (double)m_out_last);
		
		// Wait 100ms between updates
		chThdSleepMilliseconds(100);
	}
}

/**
 * Read Position from AS5047 Encoder
 * 
 * This function communicates with the AS5047 magnetic encoder via SPI
 * to read the absolute position. The encoder provides 14-bit resolution
 * (16384 positions per revolution).
 * 
 * @param ok Pointer to boolean that indicates if reading was successful
 * @return Encoder position in degrees (0-360)
 */
static float as5047_read(bool *ok) {
	uint16_t pos;
	static float last_enc_angle = 0.0;
	float res = last_enc_angle;

	// Begin SPI communication
	spi_begin();
	
	// Read 1 word (16 bits) from encoder
	spi_transfer(&pos, 0, 1);
	
	// End SPI communication
	spi_end();

	// Debug output (can be removed in production)
	commands_printf("as5047_read: %f\n", (float)pos);

	// Validate the reading
	// Check parity and ensure not disconnected (0xFFFF = all ones = disconnect)
	if (spi_check_parity(pos) && pos != 0xffff) {
		// Mask to 14 bits (AS5047 has 14-bit resolution)
		pos &= 0x3FFF;
		
		// Convert to degrees: (raw_value / 16384) * 360
		last_enc_angle = ((float)pos * 360.0) / 16384.0;
		res = last_enc_angle;
		
		// Indicate successful reading
		if (ok) {
			*ok = true;
		}
	} else {
		// Reading failed - invalid parity or disconnected
		if (ok) {
			*ok = false;
		}
	}

	return res;
}

/**
 * Check Parity of 16-bit Value
 * 
 * This function calculates the parity bit for a 16-bit value using
 * XOR operations. The AS5047 encoder includes a parity bit in its
 * output, which can be used to detect communication errors.
 * 
 * @param x 16-bit value to check
 * @return true if parity is even (even number of 1 bits), false if odd
 */
static bool spi_check_parity(uint16_t x) {
	// XOR all bits together to calculate parity
	x ^= x >> 8;  // XOR byte 0 and byte 1
	x ^= x >> 4;  // XOR nibble pairs
	x ^= x >> 2;  // XOR bit pairs
	x ^= x >> 1;  // XOR adjacent bits
	
	// Return true if even parity (result is 0), false if odd parity (result is 1)
	return (~x) & 1;
}

/**
 * SPI Bit-Banging Transfer
 * 
 * This function implements SPI communication using GPIO bit-banging.
 * It transfers 'length' words (16 bits each) via SPI protocol.
 * 
 * @param in_buf Buffer to store received data (can be NULL if not needed)
 * @param out_buf Buffer containing data to send (use NULL to send 0xFFFF)
 * @param length Number of 16-bit words to transfer
 */
static void spi_transfer(uint16_t *in_buf, const uint16_t *out_buf, int length) {
	for (int i = 0; i < length; i++) {
		// Determine what to send
		uint16_t send = out_buf ? out_buf[i] : 0xFFFF;  // Send 0xFFFF if no output buffer
		uint16_t receive = 0;  // Initialize receive buffer

		// Transfer each bit (16 bits per word)
		for (int bit = 0; bit < 16; bit++) {
			// Shift out the next bit (MSB first)
			// palWritePad(HW_SPI_PORT_MOSI, HW_SPI_PIN_MOSI, send >> 15);
			send <<= 1;

			// Clock idle-to-active transition
			spi_delay();
			palSetPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();

			// Sample MISO three times and use majority voting for noise immunity
			int r1, r2, r3;
			r1 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r2 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);
			__NOP();
			r3 = palReadPad(SPI_SW_MISO_GPIO, SPI_SW_MISO_PIN);

			// Shift in the received bit (use majority of three samples)
			receive <<= 1;
			if (utils_middle_of_3_int(r1, r2, r3)) {
				receive |= 1;
			}

			// Clock active-to-idle transition
			palClearPad(SPI_SW_SCK_GPIO, SPI_SW_SCK_PIN);
			spi_delay();
		}

		// Store received data if buffer provided
		if (in_buf) {
			in_buf[i] = receive;
		}
	}
}

/**
 * SPI Chip Select Assert (Begin Transfer)
 * 
 * Asserts the chip select line to start SPI communication.
 */
static void spi_begin(void) {
	palClearPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

/**
 * SPI Chip Select Deassert (End Transfer)
 * 
 * Deasserts the chip select line to end SPI communication.
 */
static void spi_end(void) {
	palSetPad(SPI_SW_CS_GPIO, SPI_SW_CS_PIN);
}

/**
 * SPI Delay
 * 
 * Small delay to ensure proper SPI timing. The number of NOP instructions
 * determines the delay duration. This can be adjusted based on the
 * required SPI clock speed.
 */
static void spi_delay(void) {
	__NOP();
	__NOP();
	__NOP();
	__NOP();
}
