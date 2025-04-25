/*
	Copyright 2016 - 2018 Benjamin Vedder	benjamin@vedder.se

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

#include "commands.h"
#include "ch.h"
#include "hal.h"
#include "packet.h"
#include "pos.h"
#include "buffer.h"
#include "terminal.h"
#include "bldc_interface.h"
#include "servo_simple.h"
#include "utils.h"
#include "autopilot.h"
#include "comm_cc2520.h"
#include "comm_usb.h"
#include "timeout.h"
#include "log.h"
#include "ublox.h"
#include "comm_cc1120.h"
#include "adconv.h"
#include "motor_sim.h"
#include "m8t_base.h"
#include "pos_uwb.h"
#include "fi.h"
#include "comm_can.h"
#include "hydraulic.h"

#include <math.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// Defines
#define FWD_TIME		20000

// Private variables
static uint8_t m_send_buffer[PACKET_MAX_PL_LEN];
static void(*m_send_func)(unsigned char *data, unsigned int len) = 0;
static virtual_timer_t vt;
static mutex_t m_print_gps;
static bool m_init_done = false;

// Private functions
static void stop_forward(void *p);
static void rtcm_rx(uint8_t *data, int len, int type);
static void rtcm_base_rx(rtcm_ref_sta_pos_t *pos);

// Private variables
static rtcm3_state m_rtcm_state;
uint16_t last_sensorvalue;
float debugvalue;
float angle;
static bool arduino_connected = false;

extern float io_board_as5047_angle;
extern float servo_output;
extern int iDebug;

float sign(float input)
	{
	return input/fabs(input);
	};

void commands_init(void) {
	m_send_func = 0;
	chMtxObjectInit(&m_print_gps);
	chVTObjectInit(&vt);

	rtcm3_init_state(&m_rtcm_state);
	rtcm3_set_rx_callback(rtcm_rx, &m_rtcm_state);
	rtcm3_set_rx_callback_1005_1006(rtcm_base_rx, &m_rtcm_state);

#if MAIN_MODE != MAIN_MODE_vehicle
	(void)stop_forward;
#endif

	last_sensorvalue=7;
	debugvalue=0.5;
	m_init_done = true;
}

/**
 * Provide a function to use the next time there are packets to be sent.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len)) {
	m_send_func = func;
}

/**
 * Send a packet using the set send function.
 *
 * @param data
 * The packet data.
 *
 * @param len
 * The data length.
 */
void commands_send_packet(unsigned char *data, unsigned int len) {
	if (m_send_func) {
		m_send_func(data, len);
	}
}

/**
 * Process a received buffer with commands and data.
 *
 * @param data
 * The buffer to process.
 *
 * @param len
 * The length of the buffer.
 *
 * @param func
 * A pointer to the packet sending function.
 */
void commands_process_packet(unsigned char *data, unsigned int len,
		void (*func)(unsigned char *data, unsigned int len)) {
	if (!len) {
		return;
	}
	if (data[0] == RTCM3PREAMB) {
		for (unsigned int i = 0;i < len;i++) {
			rtcm3_input_data(data[i], &m_rtcm_state);
		}
		return;
	}

	CMD_PACKET packet_id;
	uint8_t id = 0;

	id = data[0];
	data++;
	len--;

	packet_id = data[0];
	data++;
	len--;

	if (id == main_id || id == ID_ALL || id == ID_VEHICLE_CLIENT) {
		int id_ret = main_id;

		if (id == ID_VEHICLE_CLIENT) {
			id_ret = ID_VEHICLE_CLIENT;
		}
		switch (packet_id) {
		// ==================== General commands ==================== //
		case CMD_TERMINAL_CMD: {
			timeout_reset();
			commands_set_send_func(func);

			data[len] = '\0';
			terminal_process_string((char*)data);
		} break;

		// ==================== Vehicle commands ==================== //
#if MAIN_MODE_IS_VEHICLE

		case CMD_ARDUINO_STATUS   :
			if (data[0])
			{
				arduino_connected=true;
			} else
			{
				arduino_connected=false;
			}
			break;

		case CMD_GETANGLE:
			uint16_t sensorvalue=data[0]*256+data[1];
//			uint16_t sensorvalue=data[1]*256+data[0];
			last_sensorvalue=sensorvalue;

            angle=(sensorvalue-main_config.vehicle.sensorcentre)*(main_config.vehicle.degreeinterval/main_config.vehicle.sensorinterval);
            //GUNNAR CHECK
            comm_can_io_board_as5047_setangle(angle);
			break;

		case CMD_SET_POS:
		case CMD_SET_POS_ACK: {
			timeout_reset();

			float x, y, angle;
			int32_t ind = 0;
			x = buffer_get_float32(data, 1e4, &ind);
			y = buffer_get_float32(data, 1e4, &ind);
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_xya(x, y, angle);
			pos_uwb_set_xya(x, y, angle);

			if (packet_id == CMD_SET_POS_ACK) {
				commands_set_send_func(func);
				// Send ack
				int32_t send_index = 0;
				m_send_buffer[send_index++] = id_ret;
				m_send_buffer[send_index++] = packet_id;
				commands_send_packet(m_send_buffer, send_index);
			}
		} break;

		case CMD_SET_ENU_REF: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			double lat, lon, height;
			lat = buffer_get_double64(data, D(1e16), &ind);
			lon = buffer_get_double64(data, D(1e16), &ind);
			height = buffer_get_float32(data, 1e3, &ind);
			pos_set_enu_ref(lat, lon, height);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_ENU_REF: {
			timeout_reset();
			commands_set_send_func(func);

			double llh[3];
			pos_get_enu_ref(llh);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_GET_ENU_REF;
			buffer_append_double64(m_send_buffer, llh[0], D(1e16), &send_index);
			buffer_append_double64(m_send_buffer, llh[1], D(1e16), &send_index);
			buffer_append_float32(m_send_buffer, llh[2], 1e3, &send_index);
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_ADD_POINTS: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			bool first = true;

			while (ind < (int32_t)len) {
				ROUTE_POINT p;
				p.px = buffer_get_float32(data, 1e4, &ind);
				p.py = buffer_get_float32(data, 1e4, &ind);
				p.pz = buffer_get_float32(data, 1e4, &ind);
				p.speed = buffer_get_float32(data, 1e6, &ind);
				p.time = buffer_get_int32(data, &ind);
				p.attributes = buffer_get_uint32(data, &ind);
				bool res = autopilot_add_point(&p, first);
				first = false;

				if (!res) {
					break;
				}
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_REMOVE_LAST_POINT: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_remove_last_point();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_CLEAR_POINTS: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_clear_route();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_GET_ROUTE_PART: {
			int32_t ind = 0;
			int first = buffer_get_int32(data, &ind);
			int num = data[ind++];

			if (num > 20) {
				break;
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_AP_GET_ROUTE_PART;

			int route_len = autopilot_get_route_len();
			buffer_append_int32(m_send_buffer, route_len, &send_index);

			for (int i = first;i < (first + num);i++) {
				ROUTE_POINT rp = autopilot_get_route_point(i);
				buffer_append_float32_auto(m_send_buffer, rp.px, &send_index);
				buffer_append_float32_auto(m_send_buffer, rp.py, &send_index);
				buffer_append_float32_auto(m_send_buffer, rp.pz, &send_index);
				buffer_append_float32_auto(m_send_buffer, rp.speed, &send_index);
				buffer_append_int32(m_send_buffer, rp.time, &send_index);
				buffer_append_uint32(m_send_buffer, rp.attributes, &send_index);
			}

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_SET_ACTIVE: {
			timeout_reset();
			commands_set_send_func(func);

			autopilot_set_active(data[0]);
			if (data[1])
				autopilot_reset_state();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_REPLACE_ROUTE: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			int first = true;

			while (ind < (int32_t)len) {
				ROUTE_POINT p;
				p.px = buffer_get_float32(data, 1e4, &ind);
				p.py = buffer_get_float32(data, 1e4, &ind);
				p.pz = buffer_get_float32(data, 1e4, &ind);
				p.speed = buffer_get_float32(data, 1e6, &ind);
				p.time = buffer_get_int32(data, &ind);
				p.attributes = buffer_get_uint32(data, &ind);

				if (first) {
					first = !autopilot_replace_route(&p);
				} else {
					autopilot_add_point(&p, false);
				}
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_AP_SYNC_POINT: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			int32_t point = buffer_get_int32(data, &ind);
			int32_t time = buffer_get_int32(data, &ind);
			int32_t min_diff = buffer_get_int32(data, &ind);

			autopilot_sync_point(point, time, min_diff);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SEND_RTCM_USB: {
			for (unsigned int i = 0;i < len;i++) {
				rtcm3_input_data(data[i], &m_rtcm_state);
			}
		} break;

		case CMD_SEND_NMEA_RADIO: {
#if !UBLOX_EN
			// Only enable this command if the board is configured without the ublox
			char *curLine = (char*)data;
			while(curLine) {
				char *nextLine = strchr(curLine, '\n');
				if (nextLine) {
					*nextLine = '\0';
				}

				bool found = pos_input_nmea(curLine);

				// Only send the lines that pos decoded
				if (found && main_config.gps_send_nmea) {
					int32_t send_index = 0;
					m_send_buffer[send_index++] = id_ret;
					m_send_buffer[send_index++] = packet_id;
					int len_line = strlen(curLine);
					memcpy(m_send_buffer + send_index, curLine, len_line);
					send_index += len_line;

					commands_send_packet(m_send_buffer, send_index);
				}

				if (nextLine) {
					*nextLine = '\n';
				}

				curLine = nextLine ? (nextLine + 1) : NULL;
			}
#endif
		} break;

		case CMD_SET_YAW_OFFSET:
		case CMD_SET_YAW_OFFSET_ACK: {
			timeout_reset();

			float angle;
			int32_t ind = 0;
			angle = buffer_get_float32(data, 1e6, &ind);
			pos_set_yaw_offset(angle);

			if (packet_id == CMD_SET_YAW_OFFSET_ACK) {
				commands_set_send_func(func);
				// Send ack
				int32_t send_index = 0;
				m_send_buffer[send_index++] = id_ret;
				m_send_buffer[send_index++] = packet_id;
				commands_send_packet(m_send_buffer, send_index);
			}
		} break;

		case CMD_SET_MS_TODAY: {
			timeout_reset();

			int32_t time;
			int32_t ind = 0;
			time = buffer_get_int32(data, &ind);
			pos_set_ms_today(time);
		} break;

		case CMD_SET_SYSTEM_TIME: {
			commands_set_send_func(func);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_usb_send_packet(m_send_buffer, send_index);

			// Send ack
			send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_SET_SYSTEM_TIME_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_REBOOT_SYSTEM: {
			commands_set_send_func(func);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			memcpy(m_send_buffer + send_index, data, len);
			send_index += len;
			comm_usb_send_packet(m_send_buffer, send_index);

			// Send ack
			send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_REBOOT_SYSTEM_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_SET_MAIN_CONFIG: {
			timeout_reset();
			commands_set_send_func(func);

			int32_t ind = 0;
			main_config.mag_use = data[ind++];
			main_config.mag_comp = data[ind++];
			main_config.yaw_mag_gain = buffer_get_float32_auto(data, &ind);

			main_config.mag_cal_cx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_cz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_xz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_yz = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zx = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zy = buffer_get_float32_auto(data, &ind);
			main_config.mag_cal_zz = buffer_get_float32_auto(data, &ind);

			main_config.gps_ant_x = buffer_get_float32_auto(data, &ind);
			main_config.gps_ant_y = buffer_get_float32_auto(data, &ind);
			main_config.gps_comp = data[ind++];
			main_config.gps_req_rtk = data[ind++];
			main_config.gps_use_rtcm_base_as_enu_ref = data[ind++];
			main_config.gps_corr_gain_stat = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_dyn = buffer_get_float32_auto(data, &ind);
			main_config.gps_corr_gain_yaw = buffer_get_float32_auto(data, &ind);
			main_config.gps_send_nmea = data[ind++];
			main_config.gps_use_ubx_info = data[ind++];
			main_config.gps_ubx_max_acc = buffer_get_float32_auto(data, &ind);

			main_config.uwb_max_corr = buffer_get_float32_auto(data, &ind);

			main_config.ap_repeat_routes = data[ind++];
			main_config.ap_base_rad = buffer_get_float32_auto(data, &ind);
			main_config.ap_rad_time_ahead = buffer_get_float32_auto(data, &ind);
			main_config.ap_mode_time = data[ind++];
			main_config.ap_max_speed = buffer_get_float32_auto(data, &ind);
			main_config.ap_time_add_repeat_ms = buffer_get_int32(data, &ind);

			main_config.log_rate_hz = buffer_get_int16(data, &ind);
			main_config.log_en = data[ind++];
			strcpy(main_config.log_name, (const char*)(data + ind));
			ind += strlen(main_config.log_name) + 1;
			main_config.log_mode_ext = data[ind++];
			main_config.log_uart_baud = buffer_get_uint32(data, &ind);

			log_set_rate(main_config.log_rate_hz);
			log_set_enabled(main_config.log_en);
			log_set_name(main_config.log_name);
			log_set_ext(main_config.log_mode_ext, main_config.log_uart_baud);

			// vehicle settings
			main_config.vehicle.yaw_use_odometry = data[ind++];
			main_config.vehicle.yaw_imu_gain = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.disable_motor = data[ind++];
			main_config.vehicle.simulate_motor = data[ind++];
			main_config.vehicle.clamp_imu_yaw_stationary = data[ind++];
			main_config.vehicle.use_uwb_pos = data[ind++];

			main_config.vehicle.gear_ratio = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.wheel_diam = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.motor_poles = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.steering_max_angle_rad = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.steering_center = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.steering_range = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.steering_ramp_time = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.axis_distance = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.vesc_p_gain = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.vesc_i_gain = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.vesc_d_gain = buffer_get_float32_auto(data, &ind);

			main_config.vehicle.sensorcentre = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.sensorinterval = buffer_get_float32_auto(data, &ind);
			main_config.vehicle.degreeinterval = buffer_get_float32_auto(data, &ind);


#if MAIN_MODE == MAIN_MODE_vehicle
			motor_sim_set_running(main_config.vehicle.simulate_motor);
#endif


			conf_general_store_main_config(&main_config);
			// Doing this while driving will get wrong as there is so much accelerometer noise then.
			//pos_reset_attitude();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_GET_MAIN_CONFIG:
		case CMD_GET_MAIN_CONFIG_DEFAULT: {
			timeout_reset();
			commands_set_send_func(func);

			MAIN_CONFIG main_cfg_tmp;

			if (packet_id == CMD_GET_MAIN_CONFIG) {
				main_cfg_tmp = main_config;
			} else {
				conf_general_get_default_main_config(&main_cfg_tmp);
			}

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;

			m_send_buffer[send_index++] = main_cfg_tmp.mag_use;
			m_send_buffer[send_index++] = main_cfg_tmp.mag_comp;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.yaw_mag_gain, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_cz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_xz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_yz, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zx, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zy, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.mag_cal_zz, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_x, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ant_y, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_comp;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_req_rtk;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_rtcm_base_as_enu_ref;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_stat, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_dyn, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_corr_gain_yaw, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.gps_send_nmea;
			m_send_buffer[send_index++] = main_cfg_tmp.gps_use_ubx_info;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.gps_ubx_max_acc, &send_index);

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.uwb_max_corr, &send_index);

			m_send_buffer[send_index++] = main_cfg_tmp.ap_repeat_routes;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_base_rad, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_rad_time_ahead, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.ap_mode_time;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.ap_max_speed, &send_index);
			buffer_append_int32(m_send_buffer, main_cfg_tmp.ap_time_add_repeat_ms, &send_index);

			buffer_append_int16(m_send_buffer, main_cfg_tmp.log_rate_hz, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.log_en;
			strcpy((char*)(m_send_buffer + send_index), main_cfg_tmp.log_name);
			send_index += strlen(main_config.log_name) + 1;
			m_send_buffer[send_index++] = main_cfg_tmp.log_mode_ext;
			buffer_append_uint32(m_send_buffer, main_cfg_tmp.log_uart_baud, &send_index);

			// vehicle settings
			m_send_buffer[send_index++] = main_cfg_tmp.vehicle.yaw_use_odometry;
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.yaw_imu_gain, &send_index);
			m_send_buffer[send_index++] = main_cfg_tmp.vehicle.disable_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.vehicle.simulate_motor;
			m_send_buffer[send_index++] = main_cfg_tmp.vehicle.clamp_imu_yaw_stationary;
			m_send_buffer[send_index++] = main_cfg_tmp.vehicle.use_uwb_pos;

			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.gear_ratio, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.wheel_diam, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.motor_poles, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.steering_max_angle_rad, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.steering_center, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.steering_range, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.steering_ramp_time, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.axis_distance, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.vesc_p_gain, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.vesc_i_gain, &send_index);
			buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.vesc_d_gain, &send_index);

		    buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.sensorcentre, &send_index);
		    buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.sensorinterval, &send_index);
		    buffer_append_float32_auto(m_send_buffer, main_cfg_tmp.vehicle.degreeinterval, &send_index);

			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_ADD_UWB_ANCHOR: {
			int32_t ind = 0;
			UWB_ANCHOR a;

			a.id = buffer_get_int16(data, &ind);
			a.px = buffer_get_float32_auto(data, &ind);
			a.py = buffer_get_float32_auto(data, &ind);
			a.height = buffer_get_float32_auto(data, &ind);
			a.dist_last = 0.0;
			pos_uwb_add_anchor(a);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_CLEAR_UWB_ANCHORS: {
			pos_uwb_clear_anchors();

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_IO_BOARD_SET_PWM_DUTY: {
			int32_t ind = 0;
			uint8_t board = data[ind++];
			float value = buffer_get_float32_auto(data, &ind);
			comm_can_io_board_set_pwm_duty(board, value);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_IO_BOARD_SET_VALVE: {
			comm_can_io_board_set_valve(data[0], data[1], data[2]);

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = packet_id;
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_HYDRAULIC_MOVE:
			// L1,L2 R1,R2
			hydraulic_move(data[0], data[1]);
			break;

		case CMD_GET_ANGLE:
//			io_board_as5047_angle = buffer_get_float32(data, 1e3, &ind);
			// io_board_as5047_angle = buffer_get_float32_auto(data, &ind);
			break;

		// ==================== vehicle commands ==================== //
#if MAIN_MODE == MAIN_MODE_vehicle
		case CMD_GET_STATE: {
			timeout_reset();

			POS_STATE pos, pos_uwb;
			mc_values mcval;
			float accel[3];
			float gyro[3];
			float mag[3];
			ROUTE_POINT rp_goal;

			commands_set_send_func(func);

			pos_get_imu(accel, gyro, mag);
			pos_get_pos(&pos);
			pos_get_mc_val(&mcval);
			autopilot_get_goal_now(&rp_goal);
			pos_uwb_get_pos(&pos_uwb);

			fi_inject_fault_float("px", &pos.px);

			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret; // 1
			m_send_buffer[send_index++] = CMD_GET_STATE; // 2
			m_send_buffer[send_index++] = FW_VERSION_MAJOR; // 3
			m_send_buffer[send_index++] = FW_VERSION_MINOR; // 4
			buffer_append_float32(m_send_buffer, pos.roll, 1e6, &send_index); // 8
			buffer_append_float32(m_send_buffer, pos.pitch, 1e6, &send_index); // 12
			if (main_config.vehicle.use_uwb_pos) {
				buffer_append_float32(m_send_buffer, pos_uwb.yaw, 1e6, &send_index); // 16
			} else {
				buffer_append_float32(m_send_buffer, pos.yaw, 1e6, &send_index); // 16
			}
			buffer_append_float32(m_send_buffer, accel[0], 1e6, &send_index); // 20
			buffer_append_float32(m_send_buffer, accel[1], 1e6, &send_index); // 24
			buffer_append_float32(m_send_buffer, accel[2], 1e6, &send_index); // 28
			buffer_append_float32(m_send_buffer, gyro[0], 1e6, &send_index); // 32
			buffer_append_float32(m_send_buffer, gyro[1], 1e6, &send_index); // 36
			buffer_append_float32(m_send_buffer, gyro[2], 1e6, &send_index); // 40
			buffer_append_float32(m_send_buffer, mag[0], 1e6, &send_index); // 44
			buffer_append_float32(m_send_buffer, mag[1], 1e6, &send_index); // 48
			buffer_append_float32(m_send_buffer, mag[2], 1e6, &send_index); // 52
			if (main_config.vehicle.use_uwb_pos) {
				buffer_append_float32(m_send_buffer, pos_uwb.px, 1e4, &send_index); // 56
				buffer_append_float32(m_send_buffer, pos_uwb.py, 1e4, &send_index); // 60
			} else {
				if(iDebug==7) {
					commands_printf("sending x: %f,y: %f\n", pos.px, pos.py);
				};

				buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index); // 56
				buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index); // 60
			}
			buffer_append_float32(m_send_buffer, pos.speed, 1e6, &send_index); // 64
			#ifdef USE_ADCONV_FOR_VIN
				buffer_append_float32(m_send_buffer, adconv_get_vin(), 1e6, &send_index); // 68
			#else
				#ifdef IS_DRANGEN
						buffer_append_float32(m_send_buffer, mcval.v_in, 1e6, &send_index); // 68
				#else
						buffer_append_float32(m_send_buffer, mcval.v_in, 1e6, &send_index); // 68
				#endif
			#endif
			buffer_append_float32(m_send_buffer, mcval.temp_mos, 1e6, &send_index); // 72
			m_send_buffer[send_index++] = mcval.fault_code; // 73
			buffer_append_float32(m_send_buffer, pos.px_gps, 1e4, &send_index); // 77
			buffer_append_float32(m_send_buffer, pos.py_gps, 1e4, &send_index); // 81
			buffer_append_float32(m_send_buffer, rp_goal.px, 1e4, &send_index); // 85
			buffer_append_float32(m_send_buffer, rp_goal.py, 1e4, &send_index); // 89
			buffer_append_float32(m_send_buffer, autopilot_get_rad_now(), 1e6, &send_index); // 93
			buffer_append_int32(m_send_buffer, pos_get_ms_today(), &send_index); // 97
			buffer_append_int16(m_send_buffer, autopilot_get_route_left(), &send_index); // 99
			if (main_config.vehicle.use_uwb_pos) {
				buffer_append_float32(m_send_buffer, pos.px, 1e4, &send_index); // 103
				buffer_append_float32(m_send_buffer, pos.py, 1e4, &send_index); // 107
			} else {
				buffer_append_float32(m_send_buffer, pos_uwb.px, 1e4, &send_index); // 103
				buffer_append_float32(m_send_buffer, pos_uwb.py, 1e4, &send_index); // 107
			}
//			buffer_append_float32(m_send_buffer, (float) packet_id , 1e4, &send_index); // 111
			buffer_append_float32(m_send_buffer, io_board_as5047_angle, 1e4, &send_index); // 111
			buffer_append_float32(m_send_buffer, servo_output, 1e4, &send_index); // 115
			buffer_append_uint16(m_send_buffer, last_sensorvalue, &send_index); // 119
			buffer_append_float32(m_send_buffer, debugvalue, 1e4, &send_index); // 121
			commands_send_packet(m_send_buffer, send_index);
		} break;

		case CMD_VESC_FWD:
			timeout_reset();
			commands_set_send_func(func);

			bldc_interface_set_forward_func(commands_forward_vesc_packet);
			bldc_interface_send_packet(data, len);
			chVTSet(&vt, MS2ST(FWD_TIME), stop_forward, NULL);
			break;

		case CMD_RC_CONTROL: {
			timeout_reset();

			RC_MODE mode;
			float throttle, steering;
			int32_t ind = 0;
			mode = data[ind++];
			throttle = buffer_get_float32(data, 1e4, &ind);
			steering = buffer_get_float32(data, 1e6, &ind);
	
			utils_truncate_number(&steering, -1.0, 1.0);

			//TODO: Could be an issue without speed sensor 
			// steering *= autopilot_get_steering_scale();

			autopilot_set_active(false);

			debugvalue=10.0*mode+0.2;

			switch (mode) {
			case RC_MODE_CURRENT:
				if (!main_config.vehicle.disable_motor) {
					#if IS_ALL_ELECTRIC
						float okdirections=sign(angle)==-sign(steering);
						float nottooextreme=fabs(angle)<25.0;
						if (iDebug==10)
						{
							commands_printf("Throttle: %f. (%f,%f)\n", throttle,angle,steering);
							commands_printf("Signs: (%f,%f)\n", sign(angle),sign(steering));
							commands_printf("okdirections: %f, nottooextreme: %f\n", okdirections,nottooextreme);
						}
						comm_can_lock_vesc();
						comm_can_set_vesc_id(DIFF_THROTTLE_VESC_LEFT);
						bldc_interface_set_current(throttle);
						comm_can_set_vesc_id(DIFF_THROTTLE_VESC_RIGHT);
						bldc_interface_set_current(throttle);
//						if ((angle>-30) && (angle<30)) && (angle<30))
						if ( (iDebug==10) && ((okdirections) || (nottooextreme)))
						{
							commands_printf("steering: %f.\n", steering);
							commands_printf("angle: %f.\n", angle);
							comm_can_set_vesc_id(DIFF_STEERING);
							bldc_interface_set_duty_cycle(steering*VOLTAGEFRACTION);
						}
						comm_can_unlock_vesc();
					#endif

					#if HAS_DIFF_STEERING
						comm_can_lock_vesc();
						comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
						bldc_interface_set_current(throttle + throttle * steering);
						comm_can_set_vesc_id(DIFF_STEERING_VESC_RIGHT);
						bldc_interface_set_current(throttle - throttle * steering);
						comm_can_unlock_vesc();
					#else
						#if HAS_HYDRAULIC_DRIVE
							//KAN VARA HÄR
							hydraulic_set_speed(throttle / 10);
/*						#else
							comm_can_lock_vesc();
							comm_can_set_vesc_id(DIFF_THROTTLE_VESC_LEFT);
							bldc_interface_set_duty_cycle(throttle);
							comm_can_set_vesc_id(DIFF_THROTTLE_VESC_RIGHT);
							bldc_interface_set_duty_cycle(throttle);
							comm_can_set_vesc_id(DIFF_STEERING);
							bldc_interface_set_duty_cycle(steering*VOLTAGEFRACTION);
							comm_can_unlock_vesc();
*/						#endif
					#endif
				}
				break;

			case RC_MODE_DUTY:
				// Left stick up/down Movement
				utils_truncate_number(&throttle, -1.0, 1.0);
				if (!main_config.vehicle.disable_motor) {
					#if HAS_DIFF_STEERING
						comm_can_lock_vesc();
						comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
						bldc_interface_set_duty_cycle(throttle + throttle * steering);
//						comm_can_set_vesc_id(DIFF_STEERING_VESC_RIGHT);
//						bldc_interface_set_duty_cycle(throttle - throttle * steering);
						comm_can_unlock_vesc();
					#else
						#if HAS_HYDRAULIC_DRIVE
						//					hydraulic_set_speed(throttle * 10);
							#ifdef IS_MACTRAC
								hydraulic_set_throttle_raw(throttle);
							#else
								hydraulic_set_throttle_raw(throttle / 0.15);
							#endif
						#else
							comm_can_lock_vesc();
							comm_can_set_vesc_id(DIFF_THROTTLE_VESC_LEFT);
							bldc_interface_set_duty_cycle(throttle);
							comm_can_set_vesc_id(DIFF_THROTTLE_VESC_RIGHT);
							bldc_interface_set_duty_cycle(throttle);
//							if ((io_board_as5047_angle>-30) && (io_board_as5047_angle<30))
//							if (1==2)
//							{
							comm_can_set_vesc_id(DIFF_STEERING);
							bldc_interface_set_duty_cycle(steering*VOLTAGEFRACTION);
//							}
							comm_can_unlock_vesc();

						#endif
					#endif
				}
				break;

			case RC_MODE_PID: // In m/s
				#if HAS_DIFF_STEERING
					if (steering < 0.001) {
						autopilot_set_turn_rad(1e6);
					} else {
						autopilot_set_turn_rad(1.0 / steering);
					}
				#endif
				autopilot_set_motor_speed(throttle);
				break;

			case RC_MODE_CURRENT_BRAKE:
				if (!main_config.vehicle.disable_motor) {
					#if HAS_DIFF_STEERING
						comm_can_lock_vesc();
						comm_can_set_vesc_id(ID_ALL);
						bldc_interface_set_current_brake(throttle);
						comm_can_unlock_vesc();
					#else
						#if HAS_HYDRAULIC_DRIVE
							hydraulic_set_speed(0.0);
						#else
							comm_can_set_vesc_id(VESC_ID);
							bldc_interface_set_current_brake(throttle);
						#endif
					#endif
				}
				break;

			default:
				break;
			}

			#if !HAS_DIFF_STEERING

			steering = utils_map(steering, -1.0, 1.0,
				main_config.vehicle.steering_center + (main_config.vehicle.steering_range / 2.0),
				main_config.vehicle.steering_center - (main_config.vehicle.steering_range / 2.0));
			
			//Steering between 0-1
			servo_simple_set_pos_ramp(steering, true);
				
			#endif
		} break;

		case CMD_SET_SERVO_DIRECT: {
			timeout_reset();
	//		if ((io_board_as5047_angle>-30) && (io_board_as5047_angle<30))
	//		{

			int32_t ind = 0;
			float steering = buffer_get_float32(data, 1e6, &ind);
			utils_truncate_number(&steering, 0.0, 1.0);
			servo_simple_set_pos_ramp(steering, true);
		} break;
#endif
#endif

		// ==================== Mote commands ==================== //
#if MAIN_MODE_IS_MOTE
		case CMD_MOTE_UBX_START_BASE: {
			commands_set_send_func(func);

			ubx_cfg_tmode3 cfg;
			memset(&cfg, 0, sizeof(ubx_cfg_tmode3));
			int32_t ind = 0;

			cfg.mode = data[ind++];
			cfg.lla = true;
			cfg.ecefx_lat = buffer_get_double64(data, D(1e16), &ind);
			cfg.ecefy_lon = buffer_get_double64(data, D(1e16), &ind);
			cfg.ecefz_alt = buffer_get_float32_auto(data, &ind);
			cfg.fixed_pos_acc = buffer_get_float32_auto(data, &ind);
			cfg.svin_min_dur = buffer_get_uint32(data, &ind);
			cfg.svin_acc_limit = buffer_get_float32_auto(data, &ind);

			if (cfg.mode == 0) {
				m8t_base_stop();
				ublox_cfg_tmode3(&cfg);

				// Switch off RTCM messages, set rate to 5 Hz and time reference to UTC
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 0);
				ublox_cfg_rate(200, 1, 0);

				// Automotive dynamic model
				ubx_cfg_nav5 nav5;
				memset(&nav5, 0, sizeof(ubx_cfg_nav5));
				nav5.apply_dyn = true;
				nav5.dyn_model = 4;
				ublox_cfg_nav5(&nav5);
			} else if (cfg.mode == 1 || cfg.mode == 2) {
				ublox_cfg_tmode3(&cfg);

				// Switch on RTCM messages, set rate to 1 Hz and time reference to UTC
				ublox_cfg_rate(1000, 1, 0);
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, 1); // Every second
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, 1); // Every second
				ublox_cfg_msg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, 1); // Every second

				// Stationary dynamic model
				ubx_cfg_nav5 nav5;
				memset(&nav5, 0, sizeof(ubx_cfg_nav5));
				nav5.apply_dyn = true;
				nav5.dyn_model = 2;
				ublox_cfg_nav5(&nav5);
			} else if (cfg.mode == 3) {
				m8t_base_set_min_acc_samples(cfg.svin_acc_limit, cfg.svin_min_dur);
				m8t_base_start();
			} else if (cfg.mode == 4) {
				m8t_base_set_pos(cfg.ecefx_lat, cfg.ecefy_lon, cfg.ecefz_alt);
				m8t_base_start();
			}

			// Send ack
			int32_t send_index = 0;
			m_send_buffer[send_index++] = id_ret;
			m_send_buffer[send_index++] = CMD_MOTE_UBX_START_BASE_ACK;
			commands_send_packet(m_send_buffer, send_index);
		} break;
#endif

		default:
			break;
		}
	}
}

void commands_printf(const char* format, ...) {
	if (!m_init_done) {
		return;
	}

	chMtxLock(&m_print_gps);
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[512];

	print_buffer[0] = main_id;
	print_buffer[1] = CMD_PRINTF;
	len = vsnprintf(print_buffer + 2, 509, format, arg);
	va_end (arg);

	if(len > 0) {
		commands_send_packet((unsigned char*)print_buffer, (len<509) ? len + 2: 512);
	}
	chMtxUnlock(&m_print_gps);
}

void commands_printf_log_usb(char* format, ...) {
	va_list arg;
	va_start (arg, format);
	int len;
	static char print_buffer[255];

	print_buffer[0] = ID_VEHICLE_CLIENT;
	print_buffer[1] = CMD_LOG_LINE_USB;
	len = vsnprintf(print_buffer + 2, 253, format, arg);
	va_end (arg);

	if(len > 0) {
		comm_usb_send_packet((unsigned char*)print_buffer, (len<253) ? len + 2: 255);
	}
}

void commands_forward_vesc_packet(unsigned char *data, unsigned int len) {
	m_send_buffer[0] = main_id;
	m_send_buffer[1] = CMD_VESC_FWD;
	memcpy(m_send_buffer + 2, data, len);
	commands_send_packet((unsigned char*)m_send_buffer, len + 2);
}

void commands_send_nmea(unsigned char *data, unsigned int len) {
	if (main_config.gps_send_nmea) {
		int32_t send_index = 0;
		m_send_buffer[send_index++] = main_id;
		m_send_buffer[send_index++] = CMD_SEND_NMEA_RADIO;
		memcpy(m_send_buffer + send_index, data, len);
		send_index += len;
		commands_send_packet(m_send_buffer, send_index);
	}
}

void commands_send_log_ethernet(unsigned char *data, int len) {
	int32_t ind = 0;
	m_send_buffer[ind++] = ID_VEHICLE_CLIENT;
	m_send_buffer[ind++] = CMD_LOG_ETHERNET;
	memcpy(m_send_buffer + ind, data, len);
	ind += len;
	comm_usb_send_packet(m_send_buffer, ind);
}

static void stop_forward(void *p) {
	(void)p;
	bldc_interface_set_forward_func(0);
}

static void rtcm_rx(uint8_t *data, int len, int type) {
	(void)type;

#if UBLOX_EN
	ublox_send(data, len);
	(void)m_send_buffer;
#else
	int32_t send_index = 0;
	m_send_buffer[send_index++] = main_id;
	m_send_buffer[send_index++] = CMD_SEND_RTCM_USB;
	memcpy(m_send_buffer + send_index, data, len);
	send_index += len;
	comm_usb_send_packet(m_send_buffer, send_index);
#endif
}

static void rtcm_base_rx(rtcm_ref_sta_pos_t *pos) {
	if (main_config.gps_use_rtcm_base_as_enu_ref) {
		pos_set_enu_ref(pos->lat, pos->lon, pos->height);
	}
}

rtcm3_state* commands_get_rtcm3_state(void) {
	return &m_rtcm_state;
}
