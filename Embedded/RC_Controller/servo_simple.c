/*
	Copyright 2012 - 2019 Benjamin Vedder	benjamin@vedder.se

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

#include "servo_simple.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "utils.h"
#include "servo_vesc.h"

#if SERVO_VESC_ID < 0
// Settings
#define TIM_CLOCK				1000000 // Hz
#define RAMP_LOOP_HZ			100 // Hz

// Private variables
static float m_pos_now;
static float m_pos_set;
static THD_WORKING_AREA(ramp_thread_wa, 128);

// Private functions
static THD_FUNCTION(ramp_thread, arg);
#endif

/**
 * Initialize the servo system
 * 
 * This function initializes either the VESC-based servo controller or
 * the simple PWM servo controller, depending on the SERVO_VESC_ID
 * configuration.
 * 
 * When SERVO_VESC_ID >= 0: Initializes VESC-based control with feedback
 * When SERVO_VESC_ID < 0: Initializes simple PWM servo on TIM3
 */
void servo_simple_init(void) {
#if SERVO_VESC_ID >= 0
	// Use VESC-based servo control
	servo_vesc_init();
#else
	// Simple PWM servo control
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	// Initialize position variables
	m_pos_now = 0.5;
	m_pos_set = 0.5;

	// Configure GPIOB pin 0 for TIM3 alternate function
	palSetPadMode(GPIOB, 0, PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OSPEED_HIGHEST | PAL_STM32_PUDR_FLOATING);

	// Enable TIM3 clock
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);

	// Configure timer base: period and prescaler
	TIM_TimeBaseStructure.TIM_Period = (uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)SERVO_OUT_RATE_HZ);
	TIM_TimeBaseStructure.TIM_Prescaler = (uint16_t)((168000000 / 2) / TIM_CLOCK) - 1;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	// Initialize timer
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

	// Configure output compare for PWM mode
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

	// Initialize output compare channel 3
	TIM_OC3Init(TIM3, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM3, TIM_OCPreload_Enable);

	// Enable auto-reload preload
	TIM_ARRPreloadConfig(TIM3, ENABLE);

	// Set initial position to 0.5 (50%)
	servo_simple_set_pos(0.5);

	// Enable timer
	TIM_Cmd(TIM3, ENABLE);

	// Create ramp thread for smooth position changes
	chThdCreateStatic(ramp_thread_wa, sizeof(ramp_thread_wa),
			NORMALPRIO, ramp_thread, NULL);
#endif
}

/**
 * Set servo position immediately
 * 
 * This function sets the servo position to the specified value.
 * The position is truncated to the range [0.0, 1.0].
 * 
 * @param pos Target position in range [0.0, 1.0]
 */
void servo_simple_set_pos(float pos) {
	utils_truncate_number(&pos, 0.0, 1.0);
#if SERVO_VESC_ID < 0
	// Simple PWM mode: update both current and target position
	m_pos_now = pos;
	m_pos_set = pos;

	// Convert normalized position [0.0, 1.0] to pulse width in microseconds
	float us = (float)SERVO_OUT_PULSE_MIN_US + pos * (float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
	
	// Convert microseconds to timer ticks
	us *= (float)TIM_CLOCK / 1000000.0;
	
	// Set PWM duty cycle
	TIM3->CCR3 = (uint32_t)us;
#else
	// VESC mode: delegate to VESC controller
	servo_vesc_set_pos(pos);
#endif
}

/**
 * Set servo position with ramping
 * 
 * This function sets the target servo position, which will be reached
 * smoothly via ramping. In VESC mode, if reset_fault is true,
 * the fault counter is reset.
 * 
 * @param pos Target position in range [0.0, 1.0]
 * @param reset_fault If true, reset fault counter (VESC mode only)
 */
void servo_simple_set_pos_ramp(float pos, bool reset_fault) {
	utils_truncate_number(&pos, 0.0, 1.0);
#if SERVO_VESC_ID < 0
	// Simple PWM mode: set target position for ramping
	m_pos_set = pos;
#else
	// VESC mode: reset fault if requested, then set position
	if (reset_fault) {
		servo_vesc_reset_fault();
	}
	servo_vesc_set_pos(pos);
#endif
}

/**
 * Get current servo position
 * 
 * @return Current position in range [0.0, 1.0]
 */
float servo_simple_get_pos_now(void) {
#if SERVO_VESC_ID < 0
	return m_pos_now;
#else
	return servo_vesc_get_pos();
#endif
}

/**
 * Get target servo position
 * 
 * @return Target position in range [0.0, 1.0]
 */
float servo_simple_get_pos_set(void) {
#if SERVO_VESC_ID < 0
	return m_pos_set;
#else
	return servo_vesc_get_pos_set();
#endif
}

/**
 * Ramp Thread for Smooth Position Changes
 * 
 * This thread runs at RAMP_LOOP_HZ frequency and smoothly ramps the
 * servo position from current to target. It's only used in simple
 * PWM mode (when SERVO_VESC_ID < 0).
 * 
 * The ramping speed is controlled by main_config.vehicle.steering_ramp_time.
 * 
 * @param arg Unused thread argument
 */
#if SERVO_VESC_ID < 0
static THD_FUNCTION(ramp_thread, arg) {
	(void)arg;

	chRegSetThreadName("Servo ramp");

	for(;;) {
		// Store previous position
		float pos_prev = m_pos_now;
		
		// Step towards target position
		// Step size = 1.0 / (ramp_frequency * ramp_time)
		utils_step_towards(&m_pos_now, m_pos_set, 1.0 / ((float)RAMP_LOOP_HZ * main_config.vehicle.steering_ramp_time));

		// Only update PWM if position changed
		if (m_pos_now != pos_prev) {
			// Convert normalized position to pulse width in microseconds
			float us = (float)SERVO_OUT_PULSE_MIN_US + m_pos_now * (float)(SERVO_OUT_PULSE_MAX_US - SERVO_OUT_PULSE_MIN_US);
			
			// Convert microseconds to timer ticks
			us *= (float)TIM_CLOCK / 1000000.0;
			
			// Update PWM duty cycle
			TIM3->CCR3 = (uint32_t)us;
		}

		// Sleep to maintain loop frequency
		chThdSleep(CH_CFG_ST_FREQUENCY / RAMP_LOOP_HZ);
	}
}
#endif
