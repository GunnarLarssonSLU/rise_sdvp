Below you find information on important files in the folder and what to find where. (in progress..)

# Main

The main function initiates all threads running on the card. Logging and timeout settings are set directly in the file


# Autopilot

Functions related to the autopilot. 

## Registered terminal commands
| Command | Description | Calls function |
| --- | --- | --- |
| ap_state | Print the state of the autopilot | terminal_state |
| ap_print_closest | Print and plot distance to closest route point. terminal_print_closest |
| ap_dynamic_rad | Enable or disable dynamic radius. | terminal_dynamic_rad | 
| ap_ang_dist_comp | Enable or disable steering angle distance compensation | terminal_angle_dist_comp |
| ap_set_look_ahead | Set the look-ahead distance along the route in points | terminal_look_ahead |


# BLDC interface

Interface with motor control (https://github.com/vedderb/bldc)

# Conf. general

Configuration of vehicle

# Commands

Functions that process commands sent to the machine.

## Important functions

| Function | Description |
| --- | --- |
| commands_init |                 initiate gps |
| commands_process_packet |       process received command |
| commands_printf |               print information in (program internal) terminal |
| commands_forward_vesc_packet |  forward command to VESC |
| comm_usb_send_packet  | Send command to Raspberry Pi (via USB) |

## Messages

| Message   | Description   |   Input (description) (type) | Output (description) (type) |
|--------|---------|---------|---------|
| CMD_TERMINAL_CMD | Terminal command  |  (command) |  |
| CMD_SET_POS | Set vehicle position | (x,y,angle) (float32,float32,float32) |  |
| CMD_SET_ENU_REF | Set local reference | (lat,long,height) (double64, double64,float32) |
| CMD_GET_ENU_REF | Get local reference |   | (lat,long,height) (double64, double64,float32) |
| CMD_AP_ADD_POINTS | Add autopilot point |   (point) (ROUTE_POINT) |  |
| CMD_AP_REMOVE_LAST_POINT | Remove last point |  |  |
| CMD_AP_CLEAR_POINTS | Remove all points |  |  |
| CMD_AP_GET_ROUTE_PART | Get selected points | (first, last) (int, int) | array ROUTE_POINT |
| CMD_AP_REPLACE_ROUTE | Replace current route with another route(?) | array ROUTE_POINT | |
| CMD_AP_SET_ACTIVE   | Set route as active | (routeid) (int) | |
| CMD_SET_MAIN_CONFIG | Set configuration of vehicle | *See separate item* | |
| CMD_GET_MAIN_CONFIG | Get configuration of vehicle |  | *See separate item* |
| CMD_REBOOT_SYSTEM | Reboot the Raspberry Pi |  |  |
| CMD_SET_SYSTEM_TIME | Set time of the Raspberry Pi | data? |  |
| CMD_RC_CONTROL | Set vehicle throttle and steering directly | (throttle, steering) (float32, float32) | | |
| CMD_HYDRAULIC_MOVE | Change state hydraulic implements | (item, state) (int, int) | |
| CMD_GET_STATE  | Get current vehicle state | | *See separate item* |
| CMD_VESC_FWD   | Forward command to VESC | all data | all data |

## Types

### ROUTE_POINT

| Variable   | Type    |
| ---        | ---     |
| px         | float32 |
| py         | float32 |
| pz         | float32 |
| speed      | float32 |
| time       | int32   |
| attributes | uint32  |

# Hydraulic

Starts thread "Hydraulic".

Currently controls the vehicle powertrain via the controller card whilst the hydraulic implements (front/back) is controlled via a bespoke IO-card.

## Important functions

| Function | Description | Input (description) (type) | Output (description) (type) |
| --- | --- | --- | --- |
| THD_FUNCTION |  .. | .. | .. |
| hydraulic_set_speed | Sets vehicles speed (m/s) | (float) (speed) | |
| hydraulic_move | Change state hydraulic implements | (item, state) (int, int) | |

# Pos

Functions related to calculating the vehicles position

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
void pos_set_enu_ref(double lat, double lon, double height);
void pos_get_enu_ref(double *llh);
void pos_reset_enu_ref(void);
void pos_get_mc_val(mc_values *v);
int32_t pos_get_ms_today(void);
void pos_set_ms_today(int32_t ms);
bool pos_input_nmea(const char *data);
void pos_reset_attitude(void);
int pos_time_since_gps_corr(void);
void pos_base_rtcm_obs(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num);

## Registered terminal commands
| Command | Description | Calls function |
| --- | --- | --- |
| pos_delay_info | Print and plot delay information when doing GNSS position correction | cmd_terminal_delay_info |
| pos_gnss_corr_info | Print and plot correction information when doing GNSS position correction | cmd_terminal_gps_corr_info |
| pos_delay_comp | Enable or disable delay compensation | cmd_terminal_delay_comp |
| pos_print_sat_info | Print all satellite information | cmd_terminal_print_sat_info |
| pos_sat_info | Print and plot information about a satellite | cmd_terminal_sat_info |


# Timeout

## Important functions
- timeout_reset      Resets the timeout counter. Can be called from anywhere in the project. Is called in commands.c by various functions, most importantly 
- THD_FUNCTION       Stops autopilot unless timeout_reset has been called within a certain set time interval 

## Settings

- m_timeout_msec = 2000
- m_last_update_time = 0
- m_timeout_brake_current = 0.0
- m_has_timeout = false

# Utils

Various useful functions
