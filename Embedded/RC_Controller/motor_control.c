/*
	Copyright 2026 Gunnar Larsson

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
#include <math.h>
#include "motor_control.h"
#include "bldc_interface.h"
#include "hydraulic.h"
#include "utils.h"
#include "conf_general.h"
#include "autopilot.h"
#include "steering_control.h"
//#include "watchdog.h"

// Current motor direction: +1 (forward) or -1 (backward)
static volatile int current_motor_direction = 1;
static int mode;
extern float m_turn_rad_now;
int iCounterMotor=0;
extern int iDebug;

// Steering control configuration
static STEERING_CONTROL_CONFIG m_steering_config;
static bool m_steering_control_initialized = false;

void motor_set_mode(int _mode) {
	mode=_mode;
}

// Initialize motor control system
void motor_control_init(void) {
    // Configure steering control parameters
    m_steering_config.p_gain = 2.0;      // From servo_vesc.c
    m_steering_config.i_gain = 1.0;      // From servo_vesc.c
    m_steering_config.d_gain = 0.1;      // From servo_vesc.c
    m_steering_config.d_filter = 0.05;   // From servo_vesc.c
    m_steering_config.max_output = 1.0;  // Normalized output
    m_steering_config.deadband = main_config.vehicle.deadband;
    m_steering_config.max_i_term = 1.0;

    steering_control_init(&m_steering_config);
    m_steering_control_initialized = true;
}

// Common motor control functions

void motor_set_speed(float speed)
{
	if (iDebug==41)
		{
			iCounterMotor++;
			if (iCounterMotor == 50)
			{
				commands_printf("set_speed");
				commands_printf("Mode: %i",mode);
				commands_printf("Speed: %f",speed);
			iCounterMotor=0;
			}
		}
	/*
    // Check watchdog state - only allow motor operation if system is operational
    if (system_state != SYSTEM_STATE_OPERATIONAL) {
        // Force speed to 0 if watchdog indicates unsafe state
        speed = 0.0f;
    }
    */
    
    if (speed > 0.01f) {
        current_motor_direction = 1;
    } else if (speed < -0.01f) {
        current_motor_direction = -1;
    }
	if (iDebug==42)
	{
		iCounterMotor++;
		if (iCounterMotor == 50)
		{
			commands_printf("Mode: %i",RC_MODE_CURRENT);
			commands_printf("Speed: %f",speed);
			iCounterMotor=0;
		}
	}
#if HAS_HYDRAULIC_DRIVE
		// MacTrac uses this
		hydraulic_set_speed(speed);
#else
	#if HAS_DIFF_STEERING
			float rpm_r=0.0 ,rpm_l=0.0;
			void motor_diff_rpms(*rpm_r, *rpm_l, speed, m_turn_rad_now);
			comm_can_lock_vesc();
			comm_can_set_vesc_id(VESC_LEFT);
			bldc_interface_set_rpm((int)rpm_l);
			comm_can_set_vesc_id(VESC_RIGHT);
			bldc_interface_set_rpm((int)rpm_r);
			comm_can_unlock_vesc();
	#else
		float rpm = motor_rpm(speed);
		comm_can_lock_vesc();
		comm_can_set_vesc_id(VESC_LEFT);
		bldc_interface_set_rpm((int)(rpm));
		comm_can_set_vesc_id(VESC_RIGHT);
		bldc_interface_set_rpm((int)(rpm));
		comm_can_unlock_vesc();
	#endif
#endif
}


void motor_set_throttle_and_steering(float throttle,float steering,float frontangle)
{
	if (iDebug==42)
		{
			commands_printf("set_throttle_and_steering");
			commands_printf("Mode: %i",mode);
			commands_printf("Throttle: %f",throttle);
			commands_printf("Steering: %f",steering);
		}

    // Update direction - must be exactly +1 or -1
    if (throttle > 0.01f) {
        current_motor_direction = 1;
    } else if (throttle < -0.01f) {
        current_motor_direction = -1;
    }
    // If speed is near zero, keep previous direction

	#if HAS_HYDRAULIC_DRIVE
	if (iDebug==41)
	{
		iCounterMotor++;
		if (iCounterMotor == 50)
		{
			commands_printf("HHD");
			iCounterMotor=0;
		}
	}
		//KAN VARA HÄR
		hydraulic_set_throttle_raw(throttle);
		//hydraulic_set_speed(throttle / 10);
		steering = utils_map(steering, -1.0, 1.0,
			main_config.vehicle.steering_center + (main_config.vehicle.steering_range / 2.0),
			main_config.vehicle.steering_center - (main_config.vehicle.steering_range / 2.0));

		//Steering between 0-1
		servo_simple_set_pos_ramp(steering, true);
	#else
		#if HAS_DIFF_STEERING
			motor_diff_control(throttle, steering);
		#else
			motor_steering_control(throttle, steering,frontangle);
		#endif
	#endif
}

#if !HAS_HYDRAULIC_DRIVE

void motor_diff_control(float throttle, float steering)
{
	if (iDebug==41)
		{
			iCounterMotor++;
			if (iCounterMotor == 50)
			{
				commands_printf("motor_diff_control");
				commands_printf("Mode: %i",mode);
				commands_printf("Throttle: %f",throttle);
				commands_printf("Steering: %f",steering);
				iCounterMotor=0;
			}
		}
	comm_can_lock_vesc();
	switch (mode) {
		case RC_MODE_CURRENT:
			comm_can_set_vesc_id(VESC_LEFT);
			bldc_interface_set_current(throttle + throttle * steering);
			comm_can_set_vesc_id(VESC_RIGHT);
			bldc_interface_set_current(throttle - throttle * steering);
			break;
		case RC_MODE_DUTY:
			comm_can_set_vesc_id(VESC_LEFT);
			bldc_interface_set_duty_cycle(throttle + throttle * steering);
			comm_can_set_vesc_id(VESC_RIGHT);
			bldc_interface_set_duty_cycle(throttle - throttle * steering);
			break;
	}
	comm_can_unlock_vesc();
}

void motor_steering_control(float throttle, float steering,float  frontangle)
{
	if (iDebug==41)
		{
			iCounterMotor++;
			if (iCounterMotor == 50)
			{
				commands_printf("motor_steering_control");
				commands_printf("Mode: %i",mode);
				commands_printf("Throttle: %f",throttle);
				commands_printf("Steering: %f",steering);
				iCounterMotor=0;
			}
		}

	comm_can_lock_vesc();
	switch (mode) {
		case RC_MODE_CURRENT:
			comm_can_set_vesc_id(VESC_LEFT);
			bldc_interface_set_current(throttle);
			comm_can_set_vesc_id(VESC_RIGHT);
			bldc_interface_set_current(throttle);
			break;
		case RC_MODE_DUTY:
			comm_can_set_vesc_id(VESC_LEFT);
			bldc_interface_set_duty_cycle(throttle);
			comm_can_set_vesc_id(VESC_RIGHT);
			bldc_interface_set_duty_cycle(throttle);
			break;
	}
	comm_can_unlock_vesc();
	float okdirections=SIGN(frontangle)==-SIGN(steering);
	bool nottooextreme;
	#ifdef STEERINGANGLE_MAX
		nottooextreme=fabs(frontangle)<STEERINGANGLE_MAX;
	#else
		nottooextreme=true;
	#endif

if (nottooextreme || okdirections)
	{
		comm_can_lock_vesc();
		comm_can_set_vesc_id(VESC_STEERING);
		bldc_interface_set_duty_cycle(steering*VOLTAGEFRACTION);
		comm_can_unlock_vesc();
	}
}
#endif

float motor_rpm(float speed)
{
	return speed / (main_config.vehicle.gear_ratio
			* (2.0 / main_config.vehicle.motor_poles) * (1.0 / 60.0)
			* main_config.vehicle.wheel_diam * M_PI);
}

void motor_diff_rpms(float *rpm_r, float *rpm_l, float speed, float m_turn_rad_now)
{
    float diff_speed_half = 0.0;
    if (fabsf(m_turn_rad_now) > 0.1) {
        diff_speed_half = speed * (main_config.vehicle.axis_distance / (2.0 * m_turn_rad_now));
    }

    *rpm_r = (speed + diff_speed_half) / (main_config.vehicle.gear_ratio
            * (2.0 / main_config.vehicle.motor_poles) * (1.0 / 60.0)
            * main_config.vehicle.wheel_diam * M_PI);

    *rpm_l = (speed - diff_speed_half) / (main_config.vehicle.gear_ratio
            * (2.0 / main_config.vehicle.motor_poles) * (1.0 / 60.0)
            * main_config.vehicle.wheel_diam * M_PI);
}

int motor_get_direction(void) {
    return current_motor_direction;
}

// Autopilot-specific functions
void motor_set_steering_autopilot(float steering_angle, float circle_radius) {
    #if HAS_DIFF_STEERING
        // For differential steering, use turn radius
        autopilot_set_turn_rad(circle_radius);
    #else
        // Scale maximum steering by speed
        float max_rad = main_config.vehicle.steering_max_angle_rad * autopilot_get_steering_scale();

        if (steering_angle >= max_rad) {
            steering_angle = max_rad;
        } else if (steering_angle <= -max_rad) {
            steering_angle = -max_rad;
        }

        #if HAS_HYDRAULIC_DRIVE
            // Use existing servo_vesc control for hydraulic systems
            float servo_pos = steering_angle
                    / ((2.0 * main_config.vehicle.steering_max_angle_rad)
                            / main_config.vehicle.steering_range)
                            + main_config.vehicle.steering_center;
            servo_simple_set_pos_ramp(servo_pos, true);
        #else
            #if HAS_DIFF_STEERING
                // Use generic steering control with frontangle feedback

                // Set target and get control output
                steering_control_set_target(steering_angle);
                float control_output = steering_control_update(m_turn_rad_now);

                // Apply control output to steering motor
                comm_can_lock_vesc();
                comm_can_set_vesc_id(VESC_STEERING);
                bldc_interface_set_duty_cycle(control_output * VOLTAGEFRACTION);
                comm_can_unlock_vesc();
            #endif
        #endif
    #endif
}

void motor_set_speed_autopilot(float speed) {
    #if HAS_HYDRAULIC_DRIVE
        hydraulic_set_speed(speed);
    #else
        #if HAS_DIFF_STEERING
            float rpm_r, rpm_l;
            motor_diff_rpms(&rpm_r, &rpm_l, speed, m_turn_rad_now);
            comm_can_lock_vesc();
            comm_can_set_vesc_id(VESC_LEFT);
            bldc_interface_set_rpm((int)rpm_l);
            comm_can_set_vesc_id(VESC_RIGHT);
            bldc_interface_set_rpm((int)rpm_r);
            comm_can_unlock_vesc();
        #else
            #ifdef VESC_LEFT
                float rpm = motor_rpm(speed);
                comm_can_lock_vesc();
                comm_can_set_vesc_id(VESC_LEFT);
                bldc_interface_set_rpm((int)(rpm));
                comm_can_set_vesc_id(VESC_RIGHT);
                bldc_interface_set_rpm((int)(rpm));
                comm_can_unlock_vesc();
            #endif
        #endif
    #endif
}

void motor_handle_route_end(void) {
    // Center steering
    #if HAS_HYDRAULIC_DRIVE
        servo_simple_set_pos_ramp(main_config.vehicle.steering_center, false);
    #else
        #if HAS_DIFF_STEERING
            // Differential steering vehicles don't need steering centering
        #else
            #ifdef VESC_STEERING
                comm_can_lock_vesc();
                comm_can_set_vesc_id(VESC_STEERING);
                bldc_interface_set_duty_cycle(main_config.vehicle.steering_center * VOLTAGEFRACTION);
                comm_can_unlock_vesc();
            #endif
        #endif
    #endif
    
    // Apply brake
    if (!main_config.vehicle.disable_motor) {
        bldc_interface_set_current_brake(10.0);
    }
}
