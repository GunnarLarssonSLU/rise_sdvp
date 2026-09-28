/*
	Minimal main.c for troubleshooting USB communication
	
	This is a stripped-down version of main.c that only initializes:
	- HAL and ChibiOS
	- USB communication
	- Commands processing
	
	All other subsystems are disabled to isolate USB communication issues.
*/

#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "conf_general.h"
#include "comm_usb.h"
#include "commands.h"
#include "packet.h"
#include "timer.h"
#include "led.h"

#define MS2ST(ms)   ((systime_t)((ms) * CH_CFG_ST_FREQUENCY / 1000))

int main(void) {
	halInit();
	chSysInit();

	// Minimal initialization for troubleshooting
	timer_init();
	led_init();

	// Initialize configuration (needed for commands)
	conf_general_init();

	// Initialize USB communication
	comm_usb_init();

	// Initialize commands processing
	commands_init();

#if UBLOX_EN
	ublox_init();
#endif

	// Main loop - just process packets
	for(;;) {
		chThdSleepMilliseconds(10);
		packet_timerfunc();
	}
}
