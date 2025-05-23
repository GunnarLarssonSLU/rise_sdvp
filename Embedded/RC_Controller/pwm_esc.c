/*
	Copyright 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "pwm_esc.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "stm32f4xx_conf.h"
#include "pwm_esc.h"
#include "icu.h"

#include "chtypes.h"
#include "chsys.h"
#include "chvt.h"

//#include "io_board_adc.h"  // or wherever ADC_CNT_t is defined

extern ADC_CNT_t io_board_adc0_cnt;

static uint16_t last_capture = 0;
static systime_t last_tick = 0;


#define SERVO_READ

#ifndef SERVO_READ
#define SERVO_WRITE
#endif

// Settings
#define ESC_UPDATE_RATE			200	// Hz
#define TIM_CLOCK				5e6 // Hz
#define ALL_CHANNELS			0xFF

// Pins
#define SERVO1_GPIO				GPIOB
#define SERVO1_PIN				0
#define SERVO2_GPIO				GPIOB
#define SERVO2_PIN				1
#if IS_F9_BOARD
#define SERVO3_GPIO				GPIOA
#define SERVO3_PIN				2
#define SERVO4_GPIO				GPIOA
#define SERVO4_PIN				3
#else
#define SERVO3_GPIO				GPIOE
#define SERVO3_PIN				5
#define SERVO4_GPIO				GPIOE
#define SERVO4_PIN				6
#endif


#ifndef CORTEX_PRIORITY_BITS
#define CORTEX_PRIORITY_BITS 4
#endif

#ifndef CORTEX_PRIORITY_MASK
#define CORTEX_PRIORITY_MASK(n) ((n) << (8 - CORTEX_PRIORITY_BITS))
#endif


extern ADC_CNT_t io_board_adc0_cnt;

static PWMConfig pwmcfg3 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

static PWMConfig pwmcfg9 = {
		TIM_CLOCK,
		(uint16_t)((uint32_t)TIM_CLOCK / (uint32_t)ESC_UPDATE_RATE),
		NULL,
		{
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_ACTIVE_HIGH, NULL},
				{PWM_OUTPUT_DISABLED, NULL},
				{PWM_OUTPUT_DISABLED, NULL}
		},
		0,
		0,
#if STM32_PWM_USE_ADVANCED
		0
#endif
};

void pwm_esc_init(void) {
#ifdef SERVO_WRITE
	pwmStart(&PWMD3, &pwmcfg3);
	pwmStart(&PWMD9, &pwmcfg9);

	palSetPadMode(SERVO1_GPIO, SERVO1_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO2_GPIO, SERVO2_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM3) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO3_GPIO, SERVO3_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM9) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);
	palSetPadMode(SERVO4_GPIO, SERVO4_PIN,
			PAL_MODE_ALTERNATE(GPIO_AF_TIM9) |
			PAL_STM32_OTYPE_PUSHPULL |
			PAL_STM32_OSPEED_MID1);

	pwm_esc_set_all(0);
#endif
#ifdef SERVO_READ
	tach_input_init();
#endif
}

void pwm_esc_set_all(float pulse_width) {
	pwm_esc_set(ALL_CHANNELS, pulse_width);
}

/**
 * Set output pulsewidth.
 *
 * @param channel
 * Channel to use
 * Range: [0 - 3]
 * 0xFF: All Channels
 *
 * @param pulse_width
 * Pulsewidth to use
 * Range: [0.0 - 1.0]
 *
 */
void pwm_esc_set(uint8_t channel, float pulse_width) {
	uint32_t cnt_val;

	if (0) {
		// Always set zero if emergency stop is set.
		// TODO: Implement this
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)main_config.mr.motor_pwm_min_us) / 1e3;
	} else {
		cnt_val = ((TIM_CLOCK / 1e3) * (uint32_t)main_config.mr.motor_pwm_min_us) / 1e3 +
				((TIM_CLOCK / 1e3) * (uint32_t)(pulse_width * (float)(main_config.mr.motor_pwm_max_us -
						main_config.mr.motor_pwm_min_us))) / 1e3;
	}

	switch(channel) {
	case 0:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		break;

	case 1:
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		break;

	case 2:
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		break;

	case 3:
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		break;

	case ALL_CHANNELS:
		pwmEnableChannel(&PWMD3, 2, cnt_val);
		pwmEnableChannel(&PWMD3, 3, cnt_val);
		pwmEnableChannel(&PWMD9, 0, cnt_val);
		pwmEnableChannel(&PWMD9, 1, cnt_val);
		break;

	default:
		break;
	}
}


/*
static void icuwidthcb(ICUDriver *icup, icucnt_t width) {
	commands_printf("::Rising edge::\n");

	// Move current to last
    io_board_adc0_cnt.high_time_last = io_board_adc0_cnt.high_time_current;

    // Store new width (high pulse duration)
    io_board_adc0_cnt.high_time_current = (float)width;

    // Increment high edge count
    io_board_adc0_cnt.toggle_high_cnt++;

    // Update state
    io_board_adc0_cnt.is_high = true;
}

static void icuperiodcb(ICUDriver *icup, icucnt_t period) {
	commands_printf("::Falling edge::\n");
    // Compute low time as: period - high time
    float low_time = (float)period - io_board_adc0_cnt.high_time_current;

    // Move current to last
    io_board_adc0_cnt.low_time_last = io_board_adc0_cnt.low_time_current;

    // Store new low time
    io_board_adc0_cnt.low_time_current = low_time;

    // Increment low edge count
    io_board_adc0_cnt.toggle_low_cnt++;

    // Update state
    io_board_adc0_cnt.is_high = false;
}
*/
/*
static ICUConfig icucfg = {
    ICU_INPUT_ACTIVE_HIGH,
    1000000,            // 1 MHz ICU clock = 1 µs resolution
    icuwidthcb,
    icuperiodcb,
    NULL,
    ICU_CHANNEL_2,      // For PA9; change to CHANNEL_3 if using PA10
    0
};*/

/*
static ICUConfig icu_cfg = {
    .mode = ICU_INPUT_ACTIVE_HIGH,
    .frequency = 1000000, // 1 MHz = 1 µs resolution
    .width_cb = icuwidthcb,
    .period_cb = icuperiodcb,
    .overflow_cb = NULL,
    .channel = ICU_CHANNEL_3,
    .dier = 0
};
*/

void tach_input_init(void) {
	// Enable TIM2 clock
	rccEnableTIM2(TRUE);

	// Configure PA2 as TIM2_CH3 (AF1)
	palSetPadMode(GPIOA, 2, PAL_MODE_ALTERNATE(1));

	// Timer configuration
	TIM2->PSC = 84 - 1;        // 84 MHz / 84 = 1 MHz → 1 µs per count
	TIM2->ARR = 0xFFFF;        // Max period
	TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
	TIM2->CCMR2 |= TIM_CCMR2_CC3S_0;  // CC3 input, map to TI3
	TIM2->CCER &= ~TIM_CCER_CC3P;     // Rising edge
	TIM2->CCER |= TIM_CCER_CC3E;      // Enable input capture

	// Enable interrupt on CC3 match
	TIM2->DIER |= TIM_DIER_CC3IE;
	TIM2->CR1 |= TIM_CR1_CEN;         // Start the timer

	// Enable TIM2 IRQ in NVIC
	nvicEnableVector(STM32_TIM2_NUMBER, CORTEX_PRIORITY_MASK(7));
}

CH_IRQ_HANDLER(STM32_TIM2_HANDLER) {
    CH_IRQ_PROLOGUE();

    if (TIM2->SR & TIM_SR_CC3IF) {
        uint16_t capture = TIM2->CCR3;
        uint16_t delta = capture - last_capture;
        last_capture = capture;
 //   	commands_printf("::Got signal::\n");

        io_board_adc0_cnt.high_time_last = io_board_adc0_cnt.high_time_current;
        io_board_adc0_cnt.high_time_current = (float)delta;
    	commands_printf("Value: %u\n",delta);

        io_board_adc0_cnt.low_time_last = io_board_adc0_cnt.low_time_current;
        io_board_adc0_cnt.low_time_current = 0; // You can improve this later with a state machine

        io_board_adc0_cnt.toggle_high_cnt++;
        io_board_adc0_cnt.is_high = true;

        TIM2->SR &= ~TIM_SR_CC3IF; // Clear interrupt flag
    }
    CH_IRQ_EPILOGUE();
}
