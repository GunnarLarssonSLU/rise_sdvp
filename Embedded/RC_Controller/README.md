Below you find information on important files in the folder and what to find where. (in progress..)

# Main

The main function initiates all threads running on the card. Logging and timeout settings are set directly in the file


# Autopilot

Functions related to the autopilot. 

# Commands

Functions that process commands sent to the machine.

## Functions

-**commands_init**                 initiate gps
-**commands_process_packet**       process received command
-**commands_printf**               print information in (program internal) terminal
-**commands_forward_vesc_packet**  forward command to VESC

## Messages

| Message   | Description   |   Input (description) (type) | Output (description) (type) |
|--------|---------|---------|---------|
| CMD_TERMINAL_CMD | Terminal command  |  (command) |  |
| CMD_SET_POS | Set vehicle position | (x,y,angle) (float32,float32,float32) |  |
| CMD_SET_ENU_REF | Set local reference | (lat,long,height) (double64, double64,float32) |
| CMD_GET_ENU_REF | Get local reference |   | (lat,long,height) (double64, double64,float32) |


#Timeout

##Functions
*timeout_reset      Resets the timeout counter. Can be called from anywhere in the project. Is called in commands.c by various functions, most importantly 
*THD_FUNCTION       Stops autopilot unless timeout_reset has been called within a certain set time interval 

##Settings

*m_timeout_msec = 2000;
*m_last_update_time = 0;
*m_timeout_brake_current = 0.0;
*m_has_timeout = false;

