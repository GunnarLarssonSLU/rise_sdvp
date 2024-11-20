Below you find information on important files in the folder and what to find where. (in progress..)

# Main

The main function initiates all threads running on the card. Logging and timeout settings are set directly in the file


# Autopilot

Functions related to the autopilot. 

# Commands

Functions that process commands sent to the machine.

## Functions

| Function | Description |
| --- | --- |
| commands_init |                 initiate gps |
| commands_process_packet |       process received command |
| commands_printf |               print information in (program internal) terminal |
| commands_forward_vesc_packet |  forward command to VESC |

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
| CMD_RC_CONTROL | Set vehicle throttle and steering directly | (throttle, steering) (float32, float32) | | |
| CMD_HYDRAULIC_MOVE | ... | (item, state) (int, int) | |
| CMD_GET_STATE  | Get current vehicle state | | *See separate item* |

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

# Timeout

## Functions
- timeout_reset      Resets the timeout counter. Can be called from anywhere in the project. Is called in commands.c by various functions, most importantly 
- THD_FUNCTION       Stops autopilot unless timeout_reset has been called within a certain set time interval 

## Settings

- m_timeout_msec = 2000
- m_last_update_time = 0
- m_timeout_brake_current = 0.0
- m_has_timeout = false

