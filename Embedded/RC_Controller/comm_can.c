/*
	Copyright 2016 Benjamin Vedder	benjamin@vedder.se

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

#include <string.h>
#include "comm_can.h"
#include "ch.h"
#include "hal.h"
#include "stm32f4xx_conf.h"
#include "datatypes.h"
#include "buffer.h"
#include "crc.h"
#include "packet.h"
#include "bldc_interface.h"
#include "commands.h"
#include "motor_sim.h"

// Settings
#define CANDx						CAND1
#define RX_FRAMES_SIZE				100
#define RX_BUFFER_SIZE				PACKET_MAX_PL_LEN
#define CAN_STATUS_MSGS_TO_STORE	10

//ADDIO CAN
#define CAN_ADDIO_MASTER			0x28
#define CAN_ANGLE					0x19F

// Threads
static THD_WORKING_AREA(cancom_read_thread_wa, 512);
static THD_WORKING_AREA(cancom_process_thread_wa, 4096);
static THD_FUNCTION(cancom_read_thread, arg);
static THD_FUNCTION(cancom_process_thread, arg);

// Functions
static void cmd_terminal_useeid(int argc, const char **argv);

// Variables
static can_status_msg stat_msgs[CAN_STATUS_MSGS_TO_STORE];
static mutex_t can_mtx;
static mutex_t vesc_mtx_ext;
static uint8_t rx_buffer[RX_BUFFER_SIZE];
static unsigned int rx_buffer_last_id;
static CANRxFrame rx_frames[RX_FRAMES_SIZE];
static int rx_frame_read;
static int rx_frame_write;
static thread_t *process_tp;
static int vesc_id = VESC_ID;

int iEid;


// IO Board
static float io_board_adc_voltages[8] = {0};
static bool io_board_lim_sw[8] = {0};
//static float io_board_as5047_angle = 0.0; //static?
float io_board_as5047_angle = 0.0; //static?
static float can_ftr2_angle = 0.0;

#ifndef SERVO_READ
ADC_CNT_t io_board_adc0_cnt = {1};
#endif

extern int iDebug;

// ADDIO
static bool addio_lim_sw[8] = {1};
static uint8_t pvg32_node_id = 0;
static bool ftr2_activated = false;
static int ftr2_id = 0x61F;
static uint8_t ftr2_frame[8] = {0x2f, 0x00, 0x18, 0x02, 0xFE, 0x00, 0x00, 0x00};

#ifdef ADDIO
/*
 * 5125KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
		CAN_BTR_TS1(8) | CAN_BTR_BRP(27)
};
#else
/*
 * 500KBaud, automatic wakeup, automatic recover
 * from abort mode.
 * See section 22.7.7 on the STM32 reference manual.
 */
static const CANConfig cancfg = {
		CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
		CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
		CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};
#endif

// Private functions
static void send_packet_wrapper(unsigned char *data, unsigned int len);
static void printf_wrapper(char *str);
static uint8_t update_addio_outputs(int valve, bool set);
static void prop_valve_nmt_sm(uint8_t node_id, uint8_t state);
static void prop_valve_status_sm(uint8_t node_id, uint8_t* data);

// Function pointers
static void(*m_range_func)(uint8_t id, uint8_t dest, float range) = 0;
static void(*m_dw_ping_func)(uint8_t id) = 0;
static void(*m_dw_uptime_func)(uint8_t id, uint32_t uptime) = 0;

void comm_can_init(void) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		stat_msgs[i].id = -1;
	}

	rx_frame_read = 0;
	rx_frame_write = 0;
	vesc_id = VESC_ID;

	terminal_register_command_callback(
			"eid",
			"Set value for eid filter for debug info",
			0,
			cmd_terminal_useeid);


	chMtxObjectInit(&can_mtx);
	chMtxObjectInit(&vesc_mtx_ext);

	palSetPadMode(CAN1_RX_GPIO, CAN1_RX_PIN, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));
	palSetPadMode(CAN1_TX_GPIO, CAN1_TX_PIN, PAL_MODE_ALTERNATE(GPIO_AF_CAN1));

	canStart(&CANDx, &cancfg);

	#ifndef ADDIO
	bldc_interface_init(send_packet_wrapper);
	bldc_interface_set_rx_printf_func(printf_wrapper);
	#endif
	chThdCreateStatic(cancom_read_thread_wa, sizeof(cancom_read_thread_wa), NORMALPRIO + 1,
			cancom_read_thread, NULL);
	chThdCreateStatic(cancom_process_thread_wa, sizeof(cancom_process_thread_wa), NORMALPRIO,
			cancom_process_thread, NULL);
}

void comm_can_set_vesc_id(int id) {
	vesc_id = id;

	if (vesc_id == DIFF_STEERING_VESC_LEFT) {
		motor_sim_set_motor(0);
	} else if (vesc_id == DIFF_STEERING_VESC_RIGHT) {
		motor_sim_set_motor(1);
	}
}

void comm_can_lock_vesc(void) {
	chMtxLock(&vesc_mtx_ext);
}

void comm_can_unlock_vesc(void) {
	chMtxUnlock(&vesc_mtx_ext);
}

static THD_FUNCTION(cancom_read_thread, arg) {
	(void)arg;
	chRegSetThreadName("CAN read");

	event_listener_t el;
	CANRxFrame rxmsg;

	chEvtRegister(&CANDx.rxfull_event, &el, 0);

	while(!chThdShouldTerminateX()) {
		if (chEvtWaitAnyTimeout(ALL_EVENTS, MS2ST(10)) == 0) {
			continue;
		}

		msg_t result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);

		while (result == MSG_OK) {
			rx_frames[rx_frame_write++] = rxmsg;
			if (rx_frame_write == RX_FRAMES_SIZE) {
				rx_frame_write = 0;
			}

			chEvtSignal(process_tp, (eventmask_t) 1);

			result = canReceive(&CANDx, CAN_ANY_MAILBOX, &rxmsg, TIME_IMMEDIATE);
		}
	}

	chEvtUnregister(&CANDx.rxfull_event, &el);
}

/*
 * void find_ones_positions(unsigned long number) {
    unsigned long mask = 1;
    int position = 1;

    commands_printf("Positions of '1's in the binary representation of %lu are: ", number);

    while (number > 0) {
        if (number & mask) {
        	commands_printf("%d ", position);
        }
        number >>= 1;
        position++;
    }
    commands_printf("\n");
}*/

static void cmd_terminal_useeid(int argc, const char **argv) {
	sscanf(argv[1], "%i", &iEid);
	commands_printf("Eid: %i\n",iEid);
}


static THD_FUNCTION(cancom_process_thread, arg) {
	(void)arg;

	chRegSetThreadName("CAN process");
	process_tp = chThdGetSelfX();

	int32_t ind = 0;
	unsigned int rxbuf_len;
	unsigned int rxbuf_ind;
	uint8_t crc_low;
	uint8_t crc_high;
	bool commands_send;

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (rx_frame_read != rx_frame_write) {
			CANRxFrame rxmsg = rx_frames[rx_frame_read++];
//			if ((rxmsg.EID > 0) && (rxmsg.EID!=211) && (rxmsg.EID!=212) && (rxmsg.EID!=213) && (rxmsg.EID!=214) && (rxmsg.EID!=415)  && (rxmsg.EID!=415)   && (rxmsg.EID!=416) && (rxmsg.EID!=672) && (rxmsg.EID!=1824)) {

				if ((iDebug==20) && (rxmsg.EID==iEid))
				{
				commands_printf("EID: %u\n",rxmsg.EID);
				commands_printf("SID: %u\n",rxmsg.SID);
				commands_printf("IDE: %u\n",rxmsg.IDE);
				commands_printf("DLC: %u\n",rxmsg.DLC);
				commands_printf("TIME_u: %u\n",rxmsg.TIME);
				commands_printf("TIME_i: %i\n",rxmsg.TIME);
				commands_printf("TIME_f: %f\n",rxmsg.TIME);
				for (int ii=0;ii<rxmsg.DLC;ii++)
				{
					commands_printf("data[%i]: %i\n",ii,rxmsg.data8[ii]);
				}
//				CAN_PACKET_ID cmd = rxmsg.EID >> 8;
//				commands_printf("cmd: %u\n",cmd);
//				commands_printf("cmd: %u\n",rxmsg.EID & 255);
//				find_ones_positions(rxmsg.EID);
	//			};
			}
			if (rxmsg.IDE == CAN_IDE_EXT) {
				// Process extended IDs (VESC Communication)
				uint8_t id = rxmsg.EID & 0xFF;
				CAN_PACKET_ID cmd = rxmsg.EID >> 8;
				can_status_msg *stat_tmp;

				switch (cmd) {
				case CAN_PACKET_FILL_RX_BUFFER:
					memcpy(rx_buffer + rxmsg.data8[0], rxmsg.data8 + 1, rxmsg.DLC - 1);
					break;

				case CAN_PACKET_FILL_RX_BUFFER_LONG:
					rxbuf_ind = (unsigned int)rxmsg.data8[0] << 8;
					rxbuf_ind |= rxmsg.data8[1];
					if (rxbuf_ind < RX_BUFFER_SIZE) {
						memcpy(rx_buffer + rxbuf_ind, rxmsg.data8 + 2, rxmsg.DLC - 2);
					}
					break;

				case CAN_PACKET_PROCESS_RX_BUFFER:
					ind = 0;
					rx_buffer_last_id = rxmsg.data8[ind++];
					commands_send = rxmsg.data8[ind++];
					rxbuf_len = (unsigned int)rxmsg.data8[ind++] << 8;
					rxbuf_len |= (unsigned int)rxmsg.data8[ind++];

					if (rxbuf_len > RX_BUFFER_SIZE) {
						break;
					}

					crc_high = rxmsg.data8[ind++];
					crc_low = rxmsg.data8[ind++];

					if (crc16(rx_buffer, rxbuf_len)
							== ((unsigned short) crc_high << 8
									| (unsigned short) crc_low)) {

						(void)commands_send;
						bldc_interface_process_packet(rx_buffer, rxbuf_len);
					}
					break;

				case CAN_PACKET_PROCESS_SHORT_BUFFER:
					ind = 0;
					rx_buffer_last_id = rxmsg.data8[ind++];
					commands_send = rxmsg.data8[ind++];
					(void)commands_send;
					bldc_interface_process_packet(rxmsg.data8 + ind, rxmsg.DLC - ind);
					break;

				case CAN_PACKET_STATUS:
					for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
						stat_tmp = &stat_msgs[i];
						if (stat_tmp->id == id || stat_tmp->id == -1) {
							ind = 0;
							stat_tmp->id = id;
							stat_tmp->rx_time = chVTGetSystemTime();
							stat_tmp->rpm = (float)buffer_get_int32(rxmsg.data8, &ind);
							stat_tmp->current = (float)buffer_get_int16(rxmsg.data8, &ind) / 10.0;
							stat_tmp->duty = (float)buffer_get_int16(rxmsg.data8, &ind) / 1000.0;
							break;
						}
					}
					break;

				default:
					break;
				}
			} else if (rxmsg.IDE == CAN_IDE_STD) {
				// Process standard IDs
#if CAN_EN_DW
				if ((rxmsg.SID & 0x700) == CAN_MASK_DW) {
					switch (rxmsg.data8[0]) {
					case CMD_DW_RANGE: {
						int32_t ind = 1;
						uint8_t id = rxmsg.SID & 0xFF;
						uint8_t dest = rxmsg.data8[ind++];
						float range = (float)buffer_get_int32(rxmsg.data8, &ind) / 1000.0;

						if (m_range_func) {
							m_range_func(id, dest, range);
						}
					} break;

					case CMD_DW_PING: {
						if (m_dw_ping_func) {
							uint8_t id = rxmsg.SID & 0xFF;
							m_dw_ping_func(id);
						}
					} break;

					case CMD_DW_UPTIME: {
						int32_t ind = 1;
						uint8_t id = rxmsg.SID & 0xFF;
						uint32_t uptime = buffer_get_uint32(rxmsg.data8, &ind);

						if (m_dw_uptime_func) {
							m_dw_uptime_func(id, uptime);
						}
					} break;

					default:
						break;
					}
				}
#endif
#if CAN_IO_BOARD
				if ((rxmsg.SID & 0x700) == CAN_MASK_IO_BOARD) {
//					uint16_t id = rxmsg.SID & 0x0F;
					uint16_t msg = (rxmsg.SID >> 4) & 0x0F;
					int32_t ind = 0;

					switch (msg) {
/*					case CAN_IO_PACKET_ADC_VOLTAGES_0_1_2_3:
						ind = 0;
						io_board_adc_voltages[0] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[1] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[2] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[3] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);

						if (io_board_adc_voltages[0] >= 9.9) {
							io_board_adc0_cnt.is_high = true;
						} else if (io_board_adc_voltages[0] < 3.3) {
							io_board_adc0_cnt.is_high = false;
						}
						break;
*/
					case CAN_IO_PACKET_ADC_VOLTAGES_4_5_6_7:
						ind = 0;
						io_board_adc_voltages[4] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[5] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[6] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						io_board_adc_voltages[7] = buffer_get_float16(rxmsg.data8, 0.5e3, &ind);
						break;
#ifndef ANALOG_ANGLE
					case CAN_IO_PACKET_AS5047_ANGLE:
						ind = 0;
						//	commands_printf("CAN_IO_PACKET_AS5047_ANGLE\n");
						io_board_as5047_angle = buffer_get_float32(rxmsg.data8, 1e3, &ind);
						break;
#endif
					case CAN_IO_PACKET_LIM_SW:
						for (int i = 0;i < rxmsg.DLC; i++) {
							io_board_lim_sw[i] = rxmsg.data8[i];
						}
						break;

					case CAN_IO_PACKET_ADC0_HIGH_TIME:
						ind = 0;
						io_board_adc0_cnt.high_time_last = buffer_get_float32_auto(rxmsg.data8, &ind);
						io_board_adc0_cnt.high_time_current = buffer_get_float32_auto(rxmsg.data8, &ind);
						break;

					case CAN_IO_PACKET_ADC0_LOW_TIME:
						ind = 0;
						io_board_adc0_cnt.low_time_last = buffer_get_float32_auto(rxmsg.data8, &ind);
						io_board_adc0_cnt.low_time_current = buffer_get_float32_auto(rxmsg.data8, &ind);
						break;

					case CAN_IO_PACKET_ADC0_HIGH_LOW_CNT:
						ind = 0;
						io_board_adc0_cnt.toggle_high_cnt = buffer_get_uint32(rxmsg.data8, &ind);
						io_board_adc0_cnt.toggle_low_cnt = buffer_get_uint32(rxmsg.data8, &ind);
						break;

					default:
						break;
					}
				}
#endif
#if CAN_ADDIO
				if (rxmsg.SID == CAN_ANGLE) {
					ftr2_activated = true;
					can_ftr2_angle = ((rxmsg.data8[1] << 8) | rxmsg.data8[0]) / 10;
				}
				if ((rxmsg.SID & 0x700) == CAN_MASK_PVG32 && rxmsg.SID != 0x71F) {
					prop_valve_nmt_sm(rxmsg.SID - 0x700, rxmsg.data8[0]); 
				}
				if (rxmsg.SID == (0x180 + pvg32_node_id)) {
					prop_valve_status_sm(pvg32_node_id, rxmsg.data8);
				}
				if (rxmsg.SID == CAN_ADDIO_MASTER) {
					for (int i = 0;i < rxmsg.DLC; i++) {
						addio_lim_sw[i] = (rxmsg.data8[0] >> i) & 0x01;
					}
				}
#endif
			}

			if (rx_frame_read == RX_FRAMES_SIZE) {
				rx_frame_read = 0;
			}
		}
	}
}


void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len) {
	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_EXT;
	txmsg.EID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
	chMtxUnlock(&can_mtx);
}

void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len) {
	CANTxFrame txmsg;
	txmsg.IDE = CAN_IDE_STD;
	txmsg.SID = id;
	txmsg.RTR = CAN_RTR_DATA;
	txmsg.DLC = len;
	memcpy(txmsg.data8, data, len);

	chMtxLock(&can_mtx);
	canTransmit(&CANDx, CAN_ANY_MAILBOX, &txmsg, MS2ST(20));
	chMtxUnlock(&can_mtx);
}

/**
 * Send a buffer up to RX_BUFFER_SIZE bytes as fragments. If the buffer is 6 bytes or less
 * it will be sent in a single CAN frame, otherwise it will be split into
 * several frames.
 *
 * @param controller_id
 * The controller id to send to.
 *
 * @param data
 * The payload.
 *
 * @param len
 * The payload length.
 *
 * @param send
 * If true, this packet will be passed to the send function of commands.
 * Otherwise, it will be passed to the process function (DON'T CARE HERE, only for VESC).
 */
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send) {
	uint8_t send_buffer[8];

	if (len <= 6) {
		uint32_t ind = 0;
		send_buffer[ind++] = main_id + 128;
		send_buffer[ind++] = send;
		memcpy(send_buffer + ind, data, len);
		ind += len;
		comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PROCESS_SHORT_BUFFER << 8), send_buffer, ind);
	} else {
		unsigned int end_a = 0;
		for (unsigned int i = 0;i < len;i += 7) {
			if (i > 255) {
				break;
			}

			end_a = i + 7;

			uint8_t send_len = 7;
			send_buffer[0] = i;

			if ((i + 7) <= len) {
				memcpy(send_buffer + 1, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 1, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER << 8), send_buffer, send_len + 1);
		}

		for (unsigned int i = end_a;i < len;i += 6) {
			uint8_t send_len = 6;
			send_buffer[0] = i >> 8;
			send_buffer[1] = i & 0xFF;

			if ((i + 6) <= len) {
				memcpy(send_buffer + 2, data + i, send_len);
			} else {
				send_len = len - i;
				memcpy(send_buffer + 2, data + i, send_len);
			}

			comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_FILL_RX_BUFFER_LONG << 8), send_buffer, send_len + 2);
		}

		uint32_t ind = 0;
		send_buffer[ind++] = main_id + 128;
		send_buffer[ind++] = send;
		send_buffer[ind++] = len >> 8;
		send_buffer[ind++] = len & 0xFF;
		unsigned short crc = crc16(data, len);
		send_buffer[ind++] = (uint8_t)(crc >> 8);
		send_buffer[ind++] = (uint8_t)(crc & 0xFF);

		comm_can_transmit_eid(controller_id | ((uint32_t)CAN_PACKET_PROCESS_RX_BUFFER << 8), send_buffer, ind++);
	}
}

/**
 * Start ranging with a DW node on the CAN bus.
 *
 * @param id
 * The ID of the DW node.
 *
 * @param dest
 * The ID of the node to range to.
 *
 * @param samples
 * How many samples to average over.
 */
void comm_can_dw_range(uint8_t id, uint8_t dest, int samples) {
	uint8_t buffer[8];
	int32_t ind = 0;

	buffer[ind++] = CMD_DW_RANGE;
	buffer[ind++] = dest;
	buffer[ind++] = samples;

	comm_can_transmit_sid(((uint32_t)id | CAN_MASK_DW), buffer, ind);
}

void comm_can_dw_ping(uint8_t id) {
	uint8_t buffer[8];
	int32_t ind = 0;
	buffer[ind++] = CMD_DW_PING;
	comm_can_transmit_sid(((uint32_t)id | CAN_MASK_DW), buffer, ind);
}

void comm_can_dw_reboot(uint8_t id) {
	uint8_t buffer[8];
	int32_t ind = 0;
	buffer[ind++] = CMD_DW_REBOOT;
	comm_can_transmit_sid(((uint32_t)id | CAN_MASK_DW), buffer, ind);
}

void comm_can_dw_get_uptime(uint8_t id) {
	uint8_t buffer[8];
	int32_t ind = 0;
	buffer[ind++] = CMD_DW_UPTIME;
	comm_can_transmit_sid(((uint32_t)id | CAN_MASK_DW), buffer, ind);
}

/**
 * Set the function to be called when ranging is done.
 *
 * @param func
 * A pointer to the function.
 */
void comm_can_set_range_func(void(*func)(uint8_t id, uint8_t dest, float range)) {
	m_range_func = func;
}

/**
 * Set the function to be called when a ping is received from the UWB board.
 *
 * @param func
 * A pointer to the function.
 */
void comm_can_set_dw_ping_func(void(*func)(uint8_t uptime)) {
	m_dw_ping_func = func;
}

/**
 * Set the function to be called when a uptime result is received from the UWB board.
 *
 * @param func
 * A pointer to the function.
 */
void comm_can_set_dw_uptime_func(void(*func)(uint8_t id, uint32_t uptime)) {
	m_dw_uptime_func = func;
}

/**
 * Get status message by index.
 *
 * @param index
 * Index in the array
 *
 * @return
 * The message or 0 for an invalid index.
 */
can_status_msg *comm_can_get_status_msg_index(int index) {
	if (index < CAN_STATUS_MSGS_TO_STORE) {
		return &stat_msgs[index];
	} else {
		return 0;
	}
}

/**
 * Get status message by id.
 *
 * @param id
 * Id of the controller that sent the status message.
 *
 * @return
 * The message or 0 for an invalid id.
 */
can_status_msg *comm_can_get_status_msg_id(int id) {
	for (int i = 0;i < CAN_STATUS_MSGS_TO_STORE;i++) {
		if (stat_msgs[i].id == id) {
			return &stat_msgs[i];
		}
	}

	return 0;
}

bool comm_can_addio_lim_sw(int sw) {
	return addio_lim_sw[sw];
}

float comm_can_ftr2_angle(void) {
	if (!ftr2_activated) {
		uint8_t packet[8] = {0};
		uint8_t ind = 8;

		comm_can_transmit_sid(ftr2_id, ftr2_frame, 8);
	}
	return can_ftr2_angle;
}

void comm_can_addio_set_valve(int valve, bool set){
	uint8_t packet[2] = {0};
	int32_t ind = 2;

	packet[0] = 0x24;
	packet[1] = update_addio_outputs(valve, set);

	comm_can_transmit_sid(0x1F, packet, ind);
}

static uint8_t update_addio_outputs(int valve, bool set) {
	static uint8_t addio_board_outputs_bm = 0;
	if (valve > 7) return addio_board_outputs_bm;

	if (set) {
		addio_board_outputs_bm |= (1 << valve);
	} else {
		addio_board_outputs_bm &= ~(1 << valve);
	}

	return addio_board_outputs_bm;
}

static void prop_valve_status_sm(uint8_t node_id, uint8_t* data){
	uint8_t packet[8] = {0};
	uint8_t ind = 8;

	switch (data[0])
	{
	case PVG32_ERROR:
		packet[0] = 0x81; //Reset
		packet[1] = node_id; //node-ide
		comm_can_transmit_sid(0x00, packet, ind);
		// comm_can_transmit_sid(0x300 + node_id, packet, ind);
		break;
	case PVG32_INIT:
		packet[0] = PVG32_DISABLED;
		packet[2] = 0x01;
		comm_can_transmit_sid(0x300 + node_id, packet, ind);
		break;
	case PVG32_DISABLED:
		packet[0] = PVG32_HOLD;
		packet[2] = 0x01;
		comm_can_transmit_sid(0x300 + node_id, packet, ind);
		break;
	case PVG32_HOLD:
		packet[0] = PVG32_ACTIVE;
		packet[2] = 0x01;
		comm_can_transmit_sid(0x300 + node_id, packet, ind);
		break;
	case PVG32_ACTIVE:
		break;
	
	default:
		break;
	}
}

static void prop_valve_nmt_sm(uint8_t node_id, uint8_t state){
	uint8_t packet[2];
	uint8_t ind = 2;
	switch (state)
	{
	case PVG32_BOOTUP:
		break;
	case PVG32_STOPPED:
		pvg32_node_id = node_id;
		packet[0] = 0x01;
		packet[1] = node_id;

		comm_can_transmit_sid(0x00, packet, ind);
		break;
	case PVG32_PREOPERATIONAL:
		pvg32_node_id = node_id;
		packet[0] = 0x01;
		packet[1] = node_id;

		comm_can_transmit_sid(0x00, packet, ind);
		break;
	case PVG32_OPERATIONAL:
		pvg32_node_id = node_id;
		break;
	
	default:
		break;
	}
}

void comm_can_addio_set_valve_duty(float duty) {
	uint8_t packet[8] = {0};
	int32_t ind = 8;

    // Convert duty (-1.0 - 1.0) to Vpoc setpoint (-16384 to +16384)
    int16_t vpoc_value = (int16_t)((duty * 32768.0f) - 16384.0);

    // Construct RxPDO1 message
	packet[2] = (uint8_t)(vpoc_value & 0xFF);
	packet[3] = (uint8_t)((vpoc_value >> 8) & 0xFF);

    // Send RxPDO1 message (0x200 + NodeID)
	comm_can_transmit_sid(0x200 + pvg32_node_id, packet, ind);
}

float comm_can_io_board_adc_voltage(int ch) {
	if (ch < 0 || ch >= 8) {
		return 0.0;
	}
	return io_board_adc_voltages[ch];
}

float comm_can_io_board_as5047_angle(void) {
	return io_board_as5047_angle;
}

void comm_can_io_board_as5047_setangle(float angle) {
	io_board_as5047_angle=angle;
}


bool comm_can_io_board_lim_sw(int sw) {
	if (sw < 0 || sw >= 8) {
		return false;
	}
	return io_board_lim_sw[sw];
}

#ifndef SERVO_READ
ADC_CNT_t* comm_can_io_board_adc0_cnt(void) {
	return &io_board_adc0_cnt;
}
#endif


void comm_can_io_board_set_valve(int board, int valve, bool set) {
	uint8_t packet[8];
	int32_t ind = 0;
	packet[ind++] = valve;
	packet[ind++] = set;
	comm_can_transmit_sid((board & 0x0F) | CAN_MASK_IO_BOARD | (CAN_IO_PACKET_SET_VALVE << 4), packet, ind);
}

void comm_can_io_board_set_pwm_duty(int board, float duty) {
	uint8_t packet[8];
	int32_t ind = 0;
	buffer_append_float16(packet, duty, 1e3, &ind);
	comm_can_transmit_sid((board & 0x0F) | CAN_MASK_IO_BOARD | (CAN_IO_PACKET_SET_VALVE_PWM_DUTY << 4), packet, ind);
}

static void send_packet_wrapper(unsigned char *data, unsigned int len) {
	comm_can_send_buffer(vesc_id, data, len, false);
}

static void printf_wrapper(char *str) {
	commands_printf(str);
}

