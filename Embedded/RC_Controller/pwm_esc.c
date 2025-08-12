

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

// #include "io_board_adc.h"  // or wherever ADC_CNT_t is defined

//extern ADC_CNT_t io_board_adc0_cnt;
volatile bool new_pulse = false;


#ifdef SERVO_READ
//#include <time.h> // For time functions
#define TACHO_INPUT_PORT      GPIOA
ADC_CNT_t io_board_adc0_cnt = {1};
//static time_t last_reading_time = 0;
systime_t last_reading_time = 0;
static uint16_t last_capture = 2;
static systime_t last_tick = 0;
#endif


#ifdef PWMTEST
// Pin definition for PA2
#define TACHO_INPUT_PAD       2

static adcsample_t samplePWM = 0;
static ADCConversionGroup adcgrpcfg;

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
#ifdef PWMTEST
	adcgrpcfg.circular     = FALSE;
	adcgrpcfg.num_channels = 1;
	adcgrpcfg.end_cb       = NULL;
	adcgrpcfg.error_cb     = NULL;
	adcgrpcfg.cr1          = 0;
	adcgrpcfg.cr2          = ADC_CR2_SWSTART;
	adcgrpcfg.smpr1 = 0;       // Channel 2 isn't in SMPR1
	adcgrpcfg.smpr2 = 6 << 6;  // Set sample time for channel 2 (bits 8:6)
	adcgrpcfg.sqr1         = ADC_SQR1_NUM_CH(1);
	adcgrpcfg.sqr2         = 0;
	adcgrpcfg.sqr3         = ADC_SQR3_SQ1_N(ADC_CHANNEL_IN2);


	  adcStart(&ADCD1, NULL);  // Start ADC driver

    // Configure the pin as input with pull-down
    palSetPadMode(TACHO_INPUT_PORT, TACHO_INPUT_PAD, PAL_MODE_INPUT_PULLDOWN);

    // Start a debug thread
    static THD_WORKING_AREA(waTachoDebug, 128);
    chThdCreateStatic(waTachoDebug, sizeof(waTachoDebug), NORMALPRIO, tacho_debug_thread, NULL);
#endif
}

void pwm_esc_set_all(float pulse_width) {
	pwm_esc_set(ALL_CHANNELS, pulse_width);
}


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

#ifdef SERVO_READ

void tach_input_init(void) {
    // Enable TIM2 clock
    rccEnableTIM2(TRUE);

    // Configure PA2 as TIM2_CH3 (AF1)
    palSetPadMode(TACHO_INPUT_PORT, 2, PAL_MODE_ALTERNATE(1));

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

    last_capture = 0;

    // Debug: Print initialization complete
    commands_printf("Tachometer input initialized\n");
}


CH_IRQ_HANDLER(STM32_TIM2_HANDLER) {
    CH_IRQ_PROLOGUE();

    if (TIM2->SR & TIM_SR_CC3IF) {
        uint16_t capture = TIM2->CCR3;
        uint16_t delta = capture - last_capture;
        last_capture = capture;

        last_reading_time = chVTGetSystemTimeX();

        if (io_board_adc0_cnt.is_high) {
            io_board_adc0_cnt.high_time_last = io_board_adc0_cnt.high_time_current;
            io_board_adc0_cnt.high_time_current = 0.000001f * delta;
        } else {
            io_board_adc0_cnt.low_time_last = io_board_adc0_cnt.low_time_current;
            io_board_adc0_cnt.low_time_current = 0.000001f * delta;
        }

        io_board_adc0_cnt.toggle_high_cnt++;
        io_board_adc0_cnt.is_high = !io_board_adc0_cnt.is_high;

        new_pulse = true;  // signalera till tråden

        TIM2->SR &= ~TIM_SR_CC3IF;
    }

    CH_IRQ_EPILOGUE();
}
#endif

#ifdef PWMTEST
void read_adc_test(void) {
		adcConvert(&ADCD1, &adcgrpcfg, &samplePWM, 1);
//		commands_printf("Raw ADC value: %d", samplePWM);

}


static void tacho_debug_thread(void *arg) {
    (void)arg;
    chRegSetThreadName("tacho_debug");
    palSetPadMode(GPIOA, 2, PAL_MODE_INPUT_ANALOG); // Set PA2 to analog input

    while (true) {
        read_adc_test();
        chThdSleepMilliseconds(1000); // Adjust as needed
    }
}
#endif
