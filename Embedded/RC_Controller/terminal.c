/*
	Copyright 2016 - 2017 Benjamin Vedder	benjamin@vedder.se

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

#include "ch.h"
#include "hal.h"
#include "terminal.h"
#include "commands.h"
#include "utils.h"
#include "bldc_interface.h"
#include "cc1120.h"
#include "pos.h"
#include "comm_can.h"
#include "mpu9150.h"
#include "led.h"

#include <string.h>
#include <stdio.h>
#include <math.h>

// Settings
#define CALLBACK_LEN						80

// Private types
typedef struct _terminal_callback_struct {
	const char *command;
	const char *help;
	const char *arg_names;
	void(*cbf)(int argc, const char **argv);
} terminal_callback_struct;

// Private variables
static terminal_callback_struct callbacks[CALLBACK_LEN];
static int callback_write = 0;

// Private functions
static void dw_range_callback(uint8_t id, uint8_t dest, float range);
static void dw_ping_callback(uint8_t id);
static void dw_uptime_callback(uint8_t id, uint32_t uptime);

void terminal_process_string(char *str) {
	enum { kMaxArgs = 64 };
	int argc = 0;
	char *argv[kMaxArgs];

#if MAIN_MODE == MAIN_MODE_vehicle
	static char buffer[256];
#endif

	char *p2 = strtok(str, " ");
	while (p2 && argc < kMaxArgs) {
		argv[argc++] = p2;
		p2 = strtok(0, " ");
	}

	if (argc == 0) {
		commands_printf("No command received\n");
		return;
	}

	if (strcmp(argv[0], "ping") == 0) {
		commands_printf("pong\n");
		uint16_t test2 = 1200000000000;
		uint16_t test3 = 3400000000000;
		uint16_t test4 = 5600000000000;
		uint16_t test5 = 7800000000000;
//		float test = 1234.5f;
		float test = fmaxf(test2,test3)+ fmaxf(test4,test5);
		commands_printf("Test = %f\n", test);
	} else if (strcmp(argv[0], "mem") == 0) {
		size_t n, size;
		n = chHeapStatus(NULL, &size);
		commands_printf("core free memory : %u bytes", chCoreGetStatusX());
		commands_printf("heap fragments   : %u", n);
		commands_printf("heap free total  : %u bytes\n", size);
	} else if (strcmp(argv[0], "threads") == 0) {
		thread_t *tp;
		static const char *states[] = {CH_STATE_NAMES};
		commands_printf("    addr    stack prio refs     state           name time    ");
		commands_printf("-------------------------------------------------------------");
		tp = chRegFirstThread();
		do {
			commands_printf("%.8lx %.8lx %4lu %4lu %9s %14s %lu",
					(uint32_t)tp, (uint32_t)tp->p_ctx.r13,
					(uint32_t)tp->p_prio, (uint32_t)(tp->p_refs - 1),
					states[tp->p_state], tp->p_name, (uint32_t)tp->p_time);
			tp = chRegNextThread(tp);
		} while (tp != NULL);
		commands_printf(" ");
	}

#if MAIN_MODE == MAIN_MODE_vehicle
	else if (strcmp(argv[0], "vesc") == 0) {
		buffer[0] = '\0';
		int ind = 0;
		for (int i = 1;i < argc;i++) {
			sprintf(buffer + ind, " %s", argv[i]);
			ind += strlen(argv[i]) + 1;
		}
		bldc_interface_terminal_cmd(buffer);
	}
#endif

	else if (strcmp(argv[0], "reset_att") == 0) {
		pos_reset_attitude();
	} else if (strcmp(argv[0], "reset_enu") == 0) {
		pos_reset_enu_ref();
	} else if (strcmp(argv[0], "cc1120_state") == 0) {
		commands_printf("%s\n", cc1120_state_name());
	} else if (strcmp(argv[0], "cc1120_update_rf") == 0) {
		if (argc != 2) {
			commands_printf("Invalid number of arguments\n");
		} else {
			int set = -1;
			sscanf(argv[1], "%d", &set);

			if (set < 0) {
				commands_printf("Invalid argument\n");
			} else {
				cc1120_update_rf(set);
				commands_printf("Done\n");
			}
		}
	} else if (strcmp(argv[0], "dw_range") == 0) {
		if (argc != 2) {
			commands_printf("Invalid number of arguments\n");
		} else {
			int dest = -1;
			sscanf(argv[1], "%d", &dest);

			if (dest < 0 || dest > 254) {
				commands_printf("Invalid argument\n");
			} else {
				comm_can_set_range_func(dw_range_callback);
				comm_can_dw_range(CAN_DW_ID_ANY, dest, 5);
			}
		}
	} else if (strcmp(argv[0], "dw_ping") == 0) {
		comm_can_set_dw_ping_func(dw_ping_callback);
		comm_can_dw_ping(CAN_DW_ID_ANY);
	} else if (strcmp(argv[0], "dw_reboot") == 0) {
		comm_can_dw_reboot(CAN_DW_ID_ANY);
	} else if (strcmp(argv[0], "dw_uptime") == 0) {
		comm_can_set_dw_uptime_func(dw_uptime_callback);
		comm_can_dw_get_uptime(CAN_DW_ID_ANY);
	} else if (strcmp(argv[0], "zero_gyro") == 0) {
#if !HAS_BMI160
		led_write(LED_RED, 1);
		mpu9150_sample_gyro_offsets(100);
		led_write(LED_RED, 0);
#else
		commands_printf("TODO: Implement for BMI160\n");
#endif
	} else if (strcmp(argv[0], "io_board_read") == 0) {
		for (int i = 0;i < 100;i++) {
			commands_printf("ADC: %.2f %.2f %.2f %.2f %.2f %.2f %.2f",
					(double)comm_can_io_board_adc_voltage(0),
					(double)comm_can_io_board_adc_voltage(1),
					(double)comm_can_io_board_adc_voltage(2),
					(double)comm_can_io_board_adc_voltage(3),
					(double)comm_can_io_board_adc_voltage(4),
					(double)comm_can_io_board_adc_voltage(5),
					(double)comm_can_io_board_adc_voltage(6));
			commands_printf("IOs: %d %d %d %d",
					comm_can_io_board_lim_sw(0),
					comm_can_io_board_lim_sw(1),
					comm_can_io_board_lim_sw(2),
					comm_can_io_board_lim_sw(3));
			commands_printf("AS5047: %.1f",
					(double)comm_can_io_board_as5047_angle());
/*			ADC_CNT_t cnt = *comm_can_io_board_adc0_cnt();
			commands_printf("ADC0 CNT: HCurr: %.2f HLast: %.2f LCurr: %.2f LLast: %.2f HCNT: %d LCNT: %d\n",
					(double)cnt.high_time_current, (double)cnt.high_time_last,
					(double)cnt.low_time_current, (double)cnt.low_time_last,
					cnt.toggle_high_cnt, cnt.toggle_low_cnt);
			*/
			chThdSleepMilliseconds(100);
		}
		commands_printf("Done\n");
	} else if (strcmp(argv[0], "io_board_set_valve") == 0) {
		if (argc != 3) {
			commands_printf("Invalid number of arguments\n");
		} else {
			int valve = -1;
			sscanf(argv[1], "%d", &valve);
			int state = -1;
			sscanf(argv[2], "%d", &state);

			if (valve < 0 || state < 0) {
				commands_printf("Invalid argument\n");
			} else {
				comm_can_io_board_set_valve(0, valve, state);
				commands_printf("Set valve %d to %d\n", valve, state);
			}
		}
	} else if (strcmp(argv[0], "addio_read") == 0) {
		for (int i = 0;i < 100;i++) {
			commands_printf("IOs: %d %d %d %d",
					comm_can_addio_lim_sw(0),
					comm_can_addio_lim_sw(1),
					comm_can_addio_lim_sw(2),
					comm_can_addio_lim_sw(3));
			commands_printf("FTR2: %.1f",
					(double)comm_can_ftr2_angle());
			chThdSleepMilliseconds(100);
		}
		commands_printf("Done\n");
	} else if (strcmp(argv[0], "addio_set_valve") == 0) {
		if (argc != 3) {
			commands_printf("Invalid number of arguments\n");
		} else {
			int valve = -1;
			sscanf(argv[1], "%d", &valve);
			int state = -1;
			sscanf(argv[2], "%d", &state);

			if (valve < 0 || state < 0) {
				commands_printf("Invalid argument\n");
			} else {
				comm_can_addio_set_valve(valve, state);
				commands_printf("Set valve %d to %d\n", valve, state);
			}
		}
	}

	// The help command
	else if (strcmp(argv[0], "help") == 0) {
		commands_printf("Valid commands are:");
		commands_printf("help");
		commands_printf("  Show this help");

		commands_printf("ping");
		commands_printf("  Print pong here to see if the reply works");

		commands_printf("mem");
		commands_printf("  Show memory usage");

		commands_printf("threads");
		commands_printf("  List all threads");

#if MAIN_MODE == MAIN_MODE_vehicle
		commands_printf("vesc");
		commands_printf("  Forward command to VESC");
#endif

		commands_printf("reset_att");
		commands_printf("  Re-initialize the attitude estimation");

		commands_printf("reset_enu");
		commands_printf("  Re-initialize the ENU reference on the next GNSS sample");

		commands_printf("cc1120_state");
		commands_printf("  Print the state of the CC1120");

		commands_printf("cc1120_update_rf [rf_setting]");
		commands_printf("  Set one of the cc1120 RF settings");
		int ind = 0;
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_1_2K_2FSK_BW25K_4K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_1_2K_2FSK_BW50K_20K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_1_2K_2FSK_BW10K_4K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_50K_2GFSK_BW100K_25K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_100K_4FSK_BW100K_25K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_4_8K_2FSK_BW40K_9K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_4_8K_2FSK_BW50K_14K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_4_8K_2FSK_BW100K_39K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_434_0M_9_6K_2FSK_BW50K_12K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_452_0M_9_6K_2GFSK_BW33K_2_4K");
		commands_printf("  %d: %s", ind++, "CC1120_SET_452_0M_9_6K_2GFSK_BW50K_2_4K");

		commands_printf("dw_range [dest]");
		commands_printf("  Measure the distance to DW module [dest] with ultra wideband.");

		commands_printf("dw_ping");
		commands_printf("  Ping the UWB module to see if is present in the CAN bus.");

		commands_printf("dw_reboot");
		commands_printf("  Reboot the UWB module.");

		commands_printf("dw_uptime");
		commands_printf("  Ask the UWB module how long it has been running.");

		commands_printf("zero_gyro");
		commands_printf("  Zero the gyro bias. Note: The PCB must be completely still when running this command.");

		commands_printf("io_board_read");
		commands_printf("  Read and print all IO board inputs for 10 seconds.");

		commands_printf("io_board_set_valve [valve] [state]");
		commands_printf("  Set IO board valve [valve] to [state]");

		commands_printf("addio_read");
		commands_printf("  Read and print all addio inputs for 10 seconds.");

		commands_printf("addio_set_valve [valve] [state]");
		commands_printf("  Set addio valve [valve] to [state]");

		for (int i = 0;i < callback_write;i++) {
			if (callbacks[i].arg_names) {
				commands_printf("%s %s", callbacks[i].command, callbacks[i].arg_names);
			} else {
				commands_printf(callbacks[i].command);
			}

			if (callbacks[i].help) {
				commands_printf("  %s", callbacks[i].help);
			} else {
				commands_printf("  There is no help available for this command.");
			}
		}

//		commands_printf(" ");
	} else {
		bool found = false;
		for (int i = 0;i < callback_write;i++) {
			if (strcmp(argv[0], callbacks[i].command) == 0) {
				callbacks[i].cbf(argc, (const char**)argv);
				found = true;
				break;
			}
		}

		if (!found) {
			commands_printf("Invalid command: %s\n"
					"type help to list all available commands\n", argv[0]);
		}
	}
}

/**
 * Register a custom command  callback to the terminal. If the command
 * is already registered the old command callback will be replaced.
 *
 * @param command
 * The command name.
 *
 * @param help
 * A help text for the command. Can be NULL.
 *
 * @param arg_names
 * The argument names for the command, e.g. [arg_a] [arg_b]
 * Can be NULL.
 *
 * @param cbf
 * The callback function for the command.
 */
void terminal_register_command_callback(
		const char* command,
		const char *help,
		const char *arg_names,
		void(*cbf)(int argc, const char **argv)) {

	int callback_num = callback_write;

	for (int i = 0;i < callback_write;i++) {
		// First check the address in case the same callback is registered more than once.
		if (callbacks[i].command == command) {
			callback_num = i;
			break;
		}

		// Check by string comparison.
		if (strcmp(callbacks[i].command, command) == 0) {
			callback_num = i;
			break;
		}
	}

	callbacks[callback_num].command = command;
	callbacks[callback_num].help = help;
	callbacks[callback_num].arg_names = arg_names;
	callbacks[callback_num].cbf = cbf;

	if (callback_num == callback_write) {
		callback_write++;
		if (callback_write >= CALLBACK_LEN) {
			callback_write = 0;
		}
	}
}

static void dw_range_callback(uint8_t id, uint8_t dest, float range) {
	commands_printf("Distance between %d (connected over CAN) and %d: %.1f cm\n",
			id, dest, (double)range * D(100.0));
}

static void dw_ping_callback(uint8_t id) {
	commands_printf("Ping RX from %d\n", id);
}

static void dw_uptime_callback(uint8_t id, uint32_t uptime) {
	commands_printf("UWB module %d has been up for %d ms.\n", id, uptime);
}
