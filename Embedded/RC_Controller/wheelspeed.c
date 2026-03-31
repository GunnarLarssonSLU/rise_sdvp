#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include <math.h>
#include "commands.h"
#include "utils.h"
#include "hydraulic.h"
#include "ch.h"
#include "hal.h"
#include "conf_general.h"
#include "hydraulic.h"

#define SPEED_BUFFER_LEN   6      // antal senaste mätningar att använda i medel
#define MAX_FACTOR_CHANGE  2.0f   // tillåten förändring mellan mätningar (högsta/lägsta tid)
#define MIN_FACTOR_CHANGE  0.5f   // lägsta tillåten förändring (för kort tid)

static float time_buffer[SPEED_BUFFER_LEN];
static int buf_index = 0;
int buf_count = 0; // hur många giltiga värden i bufferten
float last_valid_time = 0.0f;
float m_speed_filtered = 0.0f;
float m_speed_pwm = 0;
extern systime_t last_reading_time;

extern volatile float m_speed_now;
extern float m_speed_pwm;
extern float m_speed_filtered;
extern float last_valid_time;
extern int buf_count;
#define SPEED_TIMEOUT_MS   400

volatile bool new_pulse = false;

extern float m_throttle_set;

extern float debugvalue;
extern float debugvalue2;
extern float debugvalue3;
extern float debugvalue4;
extern float debugvalue5;

#ifdef IS_MACTRAC
	// Measure speed
	const float wheel_diam = 0.65;
	const float cnts_per_rev = 16.0;
#endif
#ifdef IS_DRANGEN
	// Measure speed
	const float wheel_diam = 0.30;
	const float cnts_per_rev = 16.0;
#endif

#ifndef CORTEX_PRIORITY_BITS
#define CORTEX_PRIORITY_BITS 4
#endif

#ifndef CORTEX_PRIORITY_MASK
#define CORTEX_PRIORITY_MASK(n) ((n) << (8 - CORTEX_PRIORITY_BITS))
#endif


#ifdef SERVO_READ
//#include <time.h> // For time functions
#define TACHO_INPUT_PORT      GPIOA
ADC_CNT_t io_board_adc0_cnt = {1};
systime_t last_reading_time = 0;
static volatile uint32_t timer_overflow_count = 0; // antal overflow
static volatile uint32_t last_capture = 0;         // 32-bitars senaste capture
static systime_t last_tick = 0;
//extern float time_last;
#endif

static THD_WORKING_AREA(wheelspeed_thread_wa, 1024);
static THD_FUNCTION(wheelspeed_thread, arg);

void wheelspeed_init(void) {
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

void update_speed_buffer(float period, float unused) {
    // Validate input
    if (period <= 0.0f || isnan(period) || isinf(period)) {
        return;
    }

    float time_last = period;
    
    // Check for division by zero in speed calculation
    if (time_last <= 0.0001f) {  // Minimum reasonable period (100us)
        return;
    }

    // Dynamic tolerances based on speed
    float max_factor = MAX_RATIO;
    float min_factor = MIN_RATIO;
    if (m_speed_filtered < LOWSPEED) {
        max_factor = MAX_RATIO_LOWSPEED;
        min_factor = MIN_RATIO_LOWSPEED;
    }

    // Validate ratio if we have previous valid time
    if (last_valid_time > 0.0f) {
        float ratio = time_last / last_valid_time;
        if (ratio <= 0.0f || isinf(ratio)) {
            return;
        }
        if ((m_speed_filtered > 0.0f) && (ratio > max_factor || ratio < min_factor)) {
            return;
        }
    }

    // Update buffer with thread-safe operations
    chSysLockFromISR();
    last_valid_time = time_last;
    time_buffer[buf_index] = time_last;
    buf_index = (buf_index + 1) % SPEED_BUFFER_LEN;
    if (buf_count < SPEED_BUFFER_LEN) {
        buf_count++;
    }
    chSysUnlockFromISR();

    // Calculate average time
    float sum_time = 0.0f;
    int valid_count = 0;
    for (int i = 0; i < buf_count; i++) {
        if (time_buffer[i] > 0.0f) {
            sum_time += time_buffer[i];
            valid_count++;
        }
    }

    if (valid_count == 0) {
        return;
    }

    float avg_time = sum_time / valid_count;
    
    // Calculate speed with safety checks
    if (avg_time <= 0.0001f) {
        m_speed_filtered = 0.0f;
        m_speed_pwm = 0.0f;
        return;
    }

    float new_speed = SIGN(m_throttle_set) * (wheel_diam * M_PI) / (avg_time * cnts_per_rev);
    
    // Update shared speed variables
    chSysLockFromISR();
    const float alpha = 0.3f;
    m_speed_filtered = m_speed_filtered * (1.0f - alpha) + new_speed * alpha;
    m_speed_pwm = m_speed_filtered;
    chSysUnlockFromISR();
};

CH_IRQ_HANDLER(STM32_TIM2_HANDLER) {
    CH_IRQ_PROLOGUE();
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; // rensa flagga
        timer_overflow_count++;
    }

    if (TIM2->SR & TIM_SR_CC3IF) {
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

	chRegSetThreadName("WheelSpeed");
    
	/*
    // Increase stack size check
    if (chThdGetSelfX()->p_ctx != NULL) {
        if (chThdGetSelfX()->p_ctx->sp < (void*)((uint32_t)wheelspeed_thread_wa + sizeof(wheelspeed_thread_wa) - 256)) {
            commands_printf("WARNING: WheelSpeed thread low on stack!\n");
        }
    }*/
    
    while (!chThdShouldTerminateX()) {
        chThdSleepMilliseconds(50);
        
        // Timeout → nolla
        if (chVTTimeElapsedSinceX(last_reading_time) > MS2ST(SPEED_TIMEOUT_MS)) {
            chSysLock();
            buf_count = 0;
            last_valid_time = 0.0f;
            m_speed_now = 0.0f;
            m_speed_filtered = 0.0f;
            chSysUnlock();
        }

        // Om ny puls från ISR
        if (new_pulse) {
            chSysLock();
            new_pulse = false;
            m_speed_now = m_speed_pwm;
            debugvalue=m_speed_now+1.0;
            debugvalue2=m_speed_now+2.0;
            debugvalue3=m_speed_now+3.0;
            chSysUnlock();
        }
    }
}

