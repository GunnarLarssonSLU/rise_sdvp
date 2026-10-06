/*
	BMI270 (IMU på ROV_MCU-kortet, Upwis MP101_323) på I2C2-benen PB10/PB11 via
	mjukvaru-I2C, samma buss som BMI160 på F9-kortet. Samma gränssnitt och enheter
	som bmi160_wrapper, så pos.c behöver bara välja drivrutin.

	Start enligt Bosch (datablad och BMI270_SensorAPI): chip-ID 0x24, mjukstart,
	stäng av energisparläget, ladda upp konfigurationsfilen (8 kB, bmi270_config.c)
	i bitar med adress i INIT_ADDR_0/1, INIT_CTRL = 1, kontrollera INTERNAL_STATUS,
	och slå sedan på accel/gyro med 200 Hz, ±16 g och ±2000 °/s.
 */

#include "bmi270_wrapper.h"
#include "i2c_bb.h"
#include <string.h>

#ifndef BMI270_I2C_ADDR
#define BMI270_I2C_ADDR			0x68	// SDO till GND på ROV_MCU
#endif

// Register (bmi2_defs.h)
#define REG_CHIP_ID				0x00
#define REG_ACC_X_LSB			0x0C	// accel x,y,z och sedan gyro x,y,z: 12 byte
#define REG_INTERNAL_STATUS		0x21
#define REG_ACC_CONF			0x40
#define REG_ACC_RANGE			0x41
#define REG_GYR_CONF			0x42
#define REG_GYR_RANGE			0x43
#define REG_INIT_CTRL			0x59
#define REG_INIT_ADDR_0			0x5B
#define REG_INIT_DATA			0x5E
#define REG_PWR_CONF			0x7C
#define REG_PWR_CTRL			0x7D
#define REG_CMD					0x7E

#define BMI270_CHIP_ID			0x24
#define CMD_SOFT_RESET			0xB6
#define CONFIG_CHUNK			32		// jämnt antal byte, som Boschs read_write_len

extern const uint8_t bmi270_config_file[];
extern const uint16_t bmi270_config_file_len;

// Threads
static THD_FUNCTION(bmi270_thread, arg);
static THD_WORKING_AREA(bmi270_thread_wa, 2048);

// Private
static i2c_bb_state m_i2c_bb;
static void(*read_callback)(float *accel, float *gyro, float *mag) = 0;
static int rate_hz;
static bool m_ok = false;

static bool reg_write(uint8_t reg, const uint8_t *data, uint16_t len) {
	uint8_t txbuf[CONFIG_CHUNK + 1];
	if (len > CONFIG_CHUNK) {
		return false;
	}
	m_i2c_bb.has_error = 0;
	txbuf[0] = reg;
	memcpy(txbuf + 1, data, len);
	return i2c_bb_tx_rx(&m_i2c_bb, BMI270_I2C_ADDR, txbuf, len + 1, 0, 0);
}

static bool reg_write1(uint8_t reg, uint8_t val) {
	return reg_write(reg, &val, 1);
}

static bool reg_read(uint8_t reg, uint8_t *data, uint16_t len) {
	m_i2c_bb.has_error = 0;
	return i2c_bb_tx_rx(&m_i2c_bb, BMI270_I2C_ADDR, &reg, 1, data, len);
}

static bool init_bmi270(void) {
	uint8_t id = 0;
	// Första läsningen efter spänningspåslag kan misslyckas; läs två gånger.
	reg_read(REG_CHIP_ID, &id, 1);
	if (!reg_read(REG_CHIP_ID, &id, 1) || id != BMI270_CHIP_ID) {
		return false;
	}

	reg_write1(REG_CMD, CMD_SOFT_RESET);
	chThdSleepMilliseconds(3);
	reg_read(REG_CHIP_ID, &id, 1);	// efter reset: en läsning för att väcka I2C

	// Energisparläget av, konfigurationsladdning av, sedan uppladdning.
	if (!reg_write1(REG_PWR_CONF, 0x00)) {
		return false;
	}
	chThdSleepMilliseconds(1);	// ≥ 450 µs
	if (!reg_write1(REG_INIT_CTRL, 0x00)) {
		return false;
	}

	for (uint16_t index = 0; index < bmi270_config_file_len; index += CONFIG_CHUNK) {
		uint16_t n = bmi270_config_file_len - index;
		if (n > CONFIG_CHUNK) {
			n = CONFIG_CHUNK;
		}
		uint8_t addr[2];
		addr[0] = (uint8_t)((index / 2) & 0x0F);
		addr[1] = (uint8_t)((index / 2) >> 4);
		if (!reg_write(REG_INIT_ADDR_0, addr, 2) ||
				!reg_write(REG_INIT_DATA, bmi270_config_file + index, n)) {
			return false;
		}
	}

	if (!reg_write1(REG_INIT_CTRL, 0x01)) {
		return false;
	}
	chThdSleepMilliseconds(25);	// databladet: ≤ 20 ms

	uint8_t status = 0;
	if (!reg_read(REG_INTERNAL_STATUS, &status, 1) || (status & 0x0F) != 0x01) {
		return false;
	}

	// Accel, gyro och temperatur på. 200 Hz, normal bandbredd, prestandaläge.
	bool ok = reg_write1(REG_PWR_CTRL, 0x0E);
	ok = ok && reg_write1(REG_ACC_CONF, 0xA9);	// filter_perf | bwp normal | 200 Hz
	ok = ok && reg_write1(REG_ACC_RANGE, 0x03);	// ±16 g
	ok = ok && reg_write1(REG_GYR_CONF, 0xE9);	// filter_perf | noise_perf | bwp normal | 200 Hz
	ok = ok && reg_write1(REG_GYR_RANGE, 0x00);	// ±2000 °/s
	chThdSleepMilliseconds(50);	// gyro startar

	return ok;
}

void bmi270_wrapper_init(int samp_rate_hz) {
	rate_hz = samp_rate_hz;

	m_i2c_bb.sda_gpio = GPIOB;
	m_i2c_bb.sda_pin = 11;
	m_i2c_bb.scl_gpio = GPIOB;
	m_i2c_bb.scl_pin = 10;
	i2c_bb_init(&m_i2c_bb);

	m_ok = init_bmi270();
	if (m_ok) {
		chThdCreateStatic(bmi270_thread_wa, sizeof(bmi270_thread_wa),
				NORMALPRIO, bmi270_thread, NULL);
	}
}

void bmi270_wrapper_set_read_callback(void(*func)(float *accel, float *gyro, float *mag)) {
	read_callback = func;
}

bool bmi270_wrapper_is_ok(void) {
	return m_ok;
}

static THD_FUNCTION(bmi270_thread, arg) {
	(void)arg;

	chRegSetThreadName("BMI Sampling");

	for(;;) {
		uint8_t d[12];

		if (!reg_read(REG_ACC_X_LSB, d, sizeof(d))) {
			chThdSleepMilliseconds(5);
			continue;
		}

		float tmp_accel[3], tmp_gyro[3], tmp_mag[3];

		for (int i = 0; i < 3; i++) {
			int16_t a = (int16_t)((uint16_t)d[2 * i] | ((uint16_t)d[2 * i + 1] << 8));
			int16_t g = (int16_t)((uint16_t)d[6 + 2 * i] | ((uint16_t)d[6 + 2 * i + 1] << 8));
			tmp_accel[i] = (float)a * 16.0 / 32768.0;
			tmp_gyro[i] = (float)g * 2000.0 / 32768.0;
		}

		memset(tmp_mag, 0, sizeof(tmp_mag));

		if (read_callback) {
			read_callback(tmp_accel, tmp_gyro, tmp_mag);
		}

		chThdSleepMicroseconds(1000000 / rate_hz);
	}
}
