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

#ifndef PWM_ESC_H_
#define PWM_ESC_H_

#include <stdint.h>
#include "ch.h"
#include "hal.h"

// Functions
void pwm_esc_init(void);
void pwm_esc_set(uint8_t id, float pulse_width);
void pwm_esc_set_all(float pulse_width);

static void tacho_debug_thread(void *arg);
void read_adc_test(void);

/*
 * static void icuwidthcb(ICUDriver *icup, icucnt_t width);
static void icuperiodcb(ICUDriver *icup, icucnt_t period);
void tach_input_init(void);
*/


#endif /* PWM_ESC_H_ */
