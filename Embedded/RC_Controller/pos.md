Functions related to calculating the vehicles position.

The summary is based on that the unit has a [BMI160](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/) (which the F9-card used by Macbot & Drängen has). Though almost all will be true matter if it hasn't.


Functions called during initiation:
- ahrs_init_attitude_info
- bmi160_wrapper_init (with a sample rate of 500 hz)
- bmi160_wrapper_set_read_callback (with function mpu9150_read)
- bldc_interface_set_rx_value_func (with mc_values_received)
- ublox_set_rx_callback_relposned(with ublox_relposned_rx)
- ublox_set_rx_callback_rawx (with ubx_rx_rawx)
- save_pos_history()

During initiation it also sets a large number of variable to zero and terminal commands list below are initiated.

## Important functions

| Function | Description | Input (description) (type) | Output (description) (type) |
| --- | --- | --- | --- |
| pos_init |  | | |
| pos_pps_cb | | (EXTDriver*, expchannel_t) (extp, channel) | |
| pos_get_imu | | (float *, float *, float *) (accel, gyro, mag) | |
| pos_get_quaternions | (float *) (q) | |
| pos_get_pos | (POS_STATE *) (p) | |
| pos_get_gps | (GPS_STATE *) (p) | |
| pos_get_speed | | | (float) |
| pos_set_xya | | (float, float, float) (x, y, angle) | |
| pos_set_yaw_offset | | (float) (angle) | |
| pos_set_enu_ref | | (double, double, double) (lat, lon, height) | |
| pos_get_enu_ref | | | (array double) (llh);
| pos_reset_enu_ref | | | |
void pos_get_mc_val(mc_values *v);
int32_t pos_get_ms_today(void);
void pos_set_ms_today(int32_t ms);
bool pos_input_nmea(const char *data);
void pos_reset_attitude(void);
int pos_time_since_gps_corr(void);
void pos_base_rtcm_obs(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num);

### mpu9150_read

This function does quite a lot of processing to handle both imu data


## Registered terminal commands
| Command | Description | Calls function |
| --- | --- | --- |
| pos_delay_info | Print and plot delay information when doing GNSS position correction | cmd_terminal_delay_info |
| pos_gnss_corr_info | Print and plot correction information when doing GNSS position correction | cmd_terminal_gps_corr_info |
| pos_delay_comp | Enable or disable delay compensation | cmd_terminal_delay_comp |
| pos_print_sat_info | Print all satellite information | cmd_terminal_print_sat_info |
| pos_sat_info | Print and plot information about a satellite | cmd_terminal_sat_info |

