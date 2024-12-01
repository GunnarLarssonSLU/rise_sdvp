Below you find information on important files in the folder and what to find where. (in progress..)

The is the code used to run the bespoke IO-card on the Macbot

# Main

The main function initiates the cards. It configures the card, turns off valves and led lights

Called functions:
- halInit (Chibios function)
- chSysInit (Chibios function)
- comm_can_init (initiates CAN)
- adc_read_init 
- as5047_init (initiates angle sensor)


## Registered terminal commands

None

# Threads

ADC read


