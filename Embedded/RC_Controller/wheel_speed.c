

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
#include "commands.h"
#include <math.h>


// #include "io_board_adc.h"  // or wherever ADC_CNT_t is defined

// Threads
static THD_WORKING_AREA(wheelspeed_thread_wa, 1024);
static THD_FUNCTION(wheelspeed_thread, arg);

//extern ADC_CNT_t io_board_adc0_cnt;
volatile bool new_pulse = false;
static volatile float m_speed_now = 0.0;
static volatile float m_distance_now = 0.0;

extern const float wheel_diam;
extern const float cnts_per_rev;

//#include <time.h> // For time functions
#define TACHO_INPUT_PORT      GPIOA
ADC_CNT_t io_board_adc0_cnt = {1};
systime_t last_reading_time = 0;
static volatile uint32_t timer_overflow_count = 0; // antal overflow
static volatile uint32_t last_capture = 0;         // 32-bitars senaste capture
//extern float time_last;
float m_speed_pwm = 0;

extern float m_speed_pwm;
extern float m_speed_filtered;


#define SPEED_BUFFER_LEN   6      // antal senaste mätningar att använda i medel

static float time_buffer[SPEED_BUFFER_LEN];
static int buf_index = 0;
int buf_count = 0; // hur många giltiga värden i bufferten
float last_valid_time = 0.0f;
float m_speed_filtered = 0.0f;
extern float m_throttle_set;

extern int buf_count;
#define SPEED_TIMEOUT_MS   400

extern int iDebug;

extern float debugvalue;
extern float debugvalue2;
extern float debugvalue3;

// Settings
#define ESC_UPDATE_RATE			200	// Hz
#define TIM_CLOCK				5e6 // Hz
#define ALL_CHANNELS			0xFF


#ifndef CORTEX_PRIORITY_BITS
	#define CORTEX_PRIORITY_BITS 4
#endif

#ifndef CORTEX_PRIORITY_MASK
	#define CORTEX_PRIORITY_MASK(n) ((n) << (8 - CORTEX_PRIORITY_BITS))
#endif

void tach_input_init(void) {
    // Enable TIM2 clock
    rccEnableTIM2(TRUE);

    // Configure PA2 as TIM2_CH3 (AF1)
    palSetPadMode(TACHO_INPUT_PORT, 2, PAL_MODE_ALTERNATE(1));

    // Timer configuration
    TIM2->PSC = 840 - 1;        // 84 MHz / 84 = 1 MHz → 1 µs per count
    TIM2->ARR = 0xFFFF;        // Max period
    TIM2->CCMR2 &= ~TIM_CCMR2_CC3S;
    TIM2->CCMR2 |= TIM_CCMR2_CC3S_0;  // CC3 input, map to TI3
    TIM2->CCER &= ~TIM_CCER_CC3P;     // Rising edge (clear bit for rising edge)
    TIM2->CCER |= TIM_CCER_CC3E;      // Enable input capture

    // Try capturing falling edges instead
    // TIM2->CCER |= TIM_CCER_CC3P;  // Uncomment to capture falling edge

    // Enable interrupt on CC3 match
    TIM2->DIER |= TIM_DIER_CC3IE;
    TIM2->CR1 |= TIM_CR1_CEN;         // Start the timer

    // Enable TIM2 IRQ in NVIC
    nvicEnableVector(STM32_TIM2_NUMBER, CORTEX_PRIORITY_MASK(7));

    last_capture = 0;

    // Initialize the state machine for the first pulse
    io_board_adc0_cnt.is_high = false;
    io_board_adc0_cnt.high_time_last = 0.0f;
    io_board_adc0_cnt.high_time_current = 0.0f;
    io_board_adc0_cnt.low_time_last = 0.0f;
    io_board_adc0_cnt.low_time_current = 0.0f;
    io_board_adc0_cnt.toggle_high_cnt = 0;
    io_board_adc0_cnt.toggle_low_cnt = 0;


    // Debug: Print initialization complete
    commands_printf("Tachometer input initialized\n");
    commands_printf("TIM2->CCER = 0x%08X\n", TIM2->CCER);
    commands_printf("TIM2->DIER = 0x%08X\n", TIM2->DIER);

	chThdCreateStatic(wheelspeed_thread_wa, sizeof(wheelspeed_thread_wa), NORMALPRIO, wheelspeed_thread, NULL);

}

static void update_speed_buffer(float period, float unused) {
    // Simplified version: just use the period directly
    // Debug: Print when we receive a pulse
  //  static uint32_t pulse_count = 0;
	commands_printf("in update_speed_buffer");

//    if (pulse_count++ % 3 == 0) {
//        commands_printf("In update_speed\n");
//		commands_printf("period: %f",period);
//        commands_printf("In update_speed: period=%lu us\n", delta);
//    }
    float time_last = period;
   // debugvalue2=time_last;
    if (time_last <= 0.0f) return;

    // Dynamiska toleranser beroende på hastighet
    float max_factor = MAX_RATIO;
    float min_factor = MIN_RATIO;
    if (m_speed_filtered < LOWSPEED) { // vid låg fart, tillåt större variation
        max_factor = MAX_RATIO_LOWSPEED;
        min_factor = MIN_RATIO_LOWSPEED;
    }

    // Kontrollera om värdet är rimligt jämfört med senaste giltiga
    if (last_valid_time > 0.0f) {
        float ratio = time_last / last_valid_time;
//		commands_printf("ratio: %f",ratio);
        if ((m_speed_filtered > 0.0f) && (ratio > max_factor || ratio < min_factor)) {
            return; // orimligt → ignorera
        }
    }

    // Spara som senaste giltiga
    last_valid_time = time_last;

    // Lägg in i ringbuffert
    time_buffer[buf_index] = time_last;
    buf_index = (buf_index + 1) % SPEED_BUFFER_LEN;
    if (buf_count < SPEED_BUFFER_LEN) buf_count++;

    // Beräkna medelvärde
    float sum_time = 0.0f;
    for (int i = 0; i < buf_count; i++) {
        sum_time += time_buffer[i];
    }

    float avg_time = sum_time / buf_count;
//	commands_printf("avt_time: %f",avg_time);
    // Ny hastighet
//    float new_speed = SIGN(m_throttle_set) * (wheel_diam * M_PI) / (avg_time * cnts_per_rev);
    float new_speed = (wheel_diam * M_PI) / (avg_time * cnts_per_rev);

    //	commands_printf("new_speed: %f",new_speed);
    // Lågpassfilter
    const float alpha = 0.3f;
    m_speed_filtered = m_speed_filtered * (1.0f - alpha) + new_speed * alpha;
    m_speed_pwm = m_speed_filtered;
	commands_printf("m_speed_pwm calculated");

	//    debugvalue2=m_speed_pwm;
//    m_speed_pwm = new_speed;
};



CH_IRQ_HANDLER(STM32_TIM2_HANDLER) {
    CH_IRQ_PROLOGUE();

	commands_printf("Triggered");
    debugvalue=4.0;
    debugvalue2=4.0;
    debugvalue3=4.0;

    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // rensa flagga
        timer_overflow_count++;
        debugvalue=5.0;
        debugvalue2=5.0;
        debugvalue3=5.0;
    }

    if (TIM2->SR & TIM_SR_CC3IF) {
        debugvalue=6.0;
        debugvalue2=6.0;
        debugvalue3=6.0;
        uint16_t capture16 = TIM2->CCR3;
        uint32_t capture32 = ((uint32_t)timer_overflow_count << 16) | capture16;

        uint32_t delta = capture32 - last_capture;
        last_capture = capture32;

        last_reading_time = chVTGetSystemTimeX();

        // Simplified: Just measure the time between rising edges (period)
        // This is simpler and more reliable than trying to measure both high and low times
 //       debugvalue2=delta;
        float period = 0.00001f * delta;  // Convert to seconds
        
        // Update speed buffer with the period
        // For a square wave, the period is the sum of high and low times
        update_speed_buffer(period, 0.0f);
        new_pulse = true;  // signalera till tråden

        TIM2->SR &= ~TIM_SR_CC3IF;
    }

    CH_IRQ_EPILOGUE();
}


static THD_FUNCTION(wheelspeed_thread, arg) {
	(void)arg;

//	event_listener_t el;
//    eventmask_t events;

	chRegSetThreadName("WheelSpeed");

//   chEvtRegister(&emergency_event, &el, EMERGENCY_STOP_EVENT);

//    const systime_t READING_TIMEOUT = TIME_MS2I(1000);// Timeout in milli seconds

    while (!chThdShouldTerminateX()) {
/*
		events = chEvtWaitAnyTimeout(EMERGENCY_STOP_EVENT, MS2ST(100));

		if (events & EMERGENCY_STOP_EVENT)
		{// Perform cleanup if necessary
            // ...
            break;
        }
*/
		if (fabsf(m_speed_now) > 0.01) {
			m_distance_now += m_speed_now * 0.01;
		}

		chThdSleepMilliseconds(10);

/*
		ADC_CNT_t cnt;
#ifdef SERVO_READ
		cnt=io_board_adc0_cnt;
#else
		cnt = *comm_can_io_board_adc0_cnt();
#endif
*/


        debugvalue=1.0;
        debugvalue2=1.0;
        debugvalue3=1.0;

        // Timeout → nolla
        if (chVTTimeElapsedSinceX(last_reading_time) > MS2ST(SPEED_TIMEOUT_MS)) {
            buf_count = 0;
            last_valid_time = 0.0f;
            m_speed_now = 0.0f;
            m_speed_filtered = 0.0f;
/*            debugvalue=2.0;
            debugvalue2=2.0;
            debugvalue3=2.0;*/
        }

        // Om ny puls från ISR
        if (new_pulse) {
            syssts_t sts = chSysGetStatusAndLockX();
            new_pulse = false;
            chSysRestoreStatusX(sts);
            debugvalue=3.0;
            debugvalue2=3.0;
            debugvalue3=3.0;
            m_speed_now=m_speed_pwm;

        	commands_printf("calculated speed");
        }
	}
//    chEvtUnregister(&emergency_event, &el);
//    chThdExit(MSG_OK);
}


/**
 * Get current speed
 *
 * @return
 * Speed in m/s
 */
float get_speed(void) {
	return m_speed_now;
}

/**
 * Get travel distance
 *
 * @param reset
 * Reset distance counter
 *
 * @return
 * Travel distance in meters
 */
float get_distance(bool reset) {
	float ret = m_distance_now;

	if (reset) {
		m_distance_now = 0.0;
	}

	return ret;
}

