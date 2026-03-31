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

#ifndef WHEEL_SPEED_H_
#define WHEEL_SPEED_H_

#include "pwm_esc.h"

// Functions
void tach_input_init(void);
float get_speed(void);
float get_distance(bool reset);

static void update_speed_buffer(float high_t, float low_t);


extern volatile bool new_pulse;


#endif /* PWM_ESC_H_ */
