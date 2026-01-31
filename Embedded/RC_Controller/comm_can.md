# CAN Bus Communication Module Documentation

## Overview

This document provides comprehensive documentation for the CAN bus communication module in the RC_Controller firmware, specifically focusing on the `comm_can.c` file.

## File Structure

### Location
- **File**: `comm_can.c`
- **Purpose**: CAN bus communication for the RC_Controller system
- **Architecture**: Dual-thread design with circular buffer for reliable message handling

## Key Features

1. **Multi-Device Support**
   - VESC motor controllers
   - IO boards with sensors and valves
   - UWB (DW) modules for ranging
   - ADDIO valve controllers

2. **Dual-Thread Architecture**
   - Separate threads for reading and processing CAN messages
   - Efficient handling of CAN communication without blocking

3. **Circular Buffer**
   - 100-frame buffer for incoming CAN messages
   - Prevents message loss during high-throughput scenarios

4. **Thread Safety**
   - Mutex-protected transmit operations
   - Safe for multi-threaded access

5. **Packet Fragmentation**
   - Supports sending packets larger than CAN's 8-byte limit
   - Automatic reassembly of fragmented packets

6. **Status Monitoring**
   - Tracks status from multiple VESC controllers
   - Stores recent status messages for quick access

## Detailed Documentation

### Configuration Settings

#### `CANDx`
- **Type**: Macro
- **Value**: `CAND1`
- **Purpose**: Selects which CAN peripheral to use
- **Note**: Currently configured for CAN1

#### `RX_FRAMES_SIZE`
- **Type**: Macro
- **Value**: 100
- **Purpose**: Maximum number of CAN frames to buffer
- **Considerations**:
  - Determines the size of the circular buffer
  - Must be large enough to handle bursty CAN traffic
  - Trade-off: Larger values use more RAM but reduce message loss

#### `RX_BUFFER_SIZE`
- **Type**: Macro
- **Value**: `PACKET_MAX_PL_LEN`
- **Purpose**: Size of the receive buffer for packet data
- **Requirements**:
  - Must be at least `PACKET_MAX_PL_LEN` to handle maximum packet size
  - Used for reassembling fragmented packets

#### `CAN_STATUS_MSGS_TO_STORE`
- **Type**: Macro
- **Value**: 10
- **Purpose**: Number of status messages to store
- **Usage**: Tracks status information from VESC controllers
- **Note**: Each VESC controller can store its status

#### `CAN_ADDIO_MASTER`
- **Type**: Macro
- **Value**: 0x28
- **Purpose**: CAN ID for ADDIO master device
- **Usage**: Used for communicating with ADDIO valve controllers

#### `CAN_ANGLE`
- **Type**: Macro
- **Value**: 0x19F
- **Purpose**: CAN ID for FTR2 angle sensor
- **Usage**: Receives angle data from FTR2 sensors

### CAN Configuration

#### ADDIO Mode (5125KBaud)
```c
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(8) | CAN_BTR_BRP(27)
};
```

**Settings**:
- **Baud Rate**: 5125KBaud
- **Automatic Bus-Off Management (ABOM)**: Enabled
- **Automatic Wakeup (AWUM)**: Enabled
- **Transmit FIFO Priority (TXFP)**: Enabled
- **Synchronization Jump Width (SJW)**: 0
- **Time Segment 2 (TS2)**: 1
- **Time Segment 1 (TS1)**: 8
- **Baud Rate Prescaler (BRP)**: 27

#### Default Mode (500KBaud)
```c
static const CANConfig cancfg = {
    CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP,
    CAN_BTR_SJW(0) | CAN_BTR_TS2(1) |
    CAN_BTR_TS1(8) | CAN_BTR_BRP(6)
};
```

**Settings**:
- **Baud Rate**: 500KBaud
- **Automatic Bus-Off Management (ABOM)**: Enabled
- **Automatic Wakeup (AWUM)**: Enabled
- **Transmit FIFO Priority (TXFP)**: Enabled
- **Synchronization Jump Width (SJW)**: 0
- **Time Segment 2 (TS2)**: 1
- **Time Segment 1 (TS1)**: 8
- **Baud Rate Prescaler (BRP)**: 6

### Private Variables

#### `stat_msgs`
- **Type**: `can_status_msg[CAN_STATUS_MSGS_TO_STORE]`
- **Purpose**: Array for storing CAN status messages
- **Usage**: Tracks status information from VESC controllers
- **Access**: Thread-safe (protected by mutex when needed)

#### `can_mtx`
- **Type**: `mutex_t`
- **Purpose**: Mutex for protecting CAN transmit operations
- **Usage**: Locked during `canTransmit()` calls
- **Importance**: Prevents concurrent CAN transmissions

#### `vesc_mtx_ext`
- **Type**: `mutex_t`
- **Purpose**: External mutex for VESC operations
- **Usage**: Used by external code to synchronize VESC access
- **Functions**: `comm_can_lock_vesc()`, `comm_can_unlock_vesc()`

#### `rx_buffer`
- **Type**: `uint8_t[RX_BUFFER_SIZE]`
- **Purpose**: Buffer for reassembling fragmented packets
- **Usage**: Stores packet data during fragmentation/reassembly
- **Size**: `PACKET_MAX_PL_LEN` bytes

#### `rx_buffer_last_id`
- **Type**: `unsigned int`
- **Purpose**: Stores the last received packet ID
- **Usage**: Used for debugging and packet tracking

#### `rx_frames`
- **Type**: `CANRxFrame[RX_FRAMES_SIZE]`
- **Purpose**: Circular buffer for incoming CAN messages
- **Usage**: Stores received CAN frames before processing
- **Size**: 100 frames

#### `rx_frame_read` and `rx_frame_write`
- **Type**: `int`
- **Purpose**: Track positions in the circular buffer
- **Synchronization**:
  - `read`: Modified by process thread
  - `write`: Modified by read thread
  - No mutex needed (single writer/single reader)

#### `process_tp`
- **Type**: `thread_t*`
- **Purpose**: Pointer to process thread for signaling
- **Usage**: Read thread signals process thread when new data arrives

#### `vesc_id`
- **Type**: `int`
- **Purpose**: Stores the VESC controller ID
- **Default**: `VESC_ID`
- **Usage**: Used for addressing VESC controllers on CAN bus

### Thread Configuration

#### `cancom_read_thread_wa`
- **Type**: `THD_WORKING_AREA`
- **Size**: 512 bytes
- **Purpose**: Stack space for the CAN read thread
- **Priority**: `NORMALPRIO + 1` (higher than process thread)

#### `cancom_process_thread_wa`
- **Type**: `THD_WORKING_AREA`
- **Size**: 4096 bytes
- **Purpose**: Stack space for the CAN process thread
- **Priority**: `NORMALPRIO`

### Public Functions

#### `comm_can_init()`
```c
void comm_can_init(void)
```

**Purpose**: Initializes the CAN communication module

**Steps**:
1. Initializes status message array
2. Resets circular buffer positions
3. Sets default VESC ID
4. Initializes mutexes
5. Configures CAN GPIO pins
6. Starts CAN peripheral
7. Initializes BLDC interface (non-ADDIO mode)
8. Creates read and process threads

**Requirements**:
- Must be called before any CAN communication
- Should be called after CAN hardware initialization
- Thread-safe (can be called from initialization context)

**Thread Creation**:
- Read thread: 512-byte stack, `NORMALPRIO + 1`
- Process thread: 4096-byte stack, `NORMALPRIO`

#### `comm_can_set_vesc_id()`
```c
void comm_can_set_vesc_id(int id)
```

**Purpose**: Sets the VESC controller ID

**Parameters**:
- `id`: VESC controller ID to use

**Effects**:
- Updates `vesc_id` variable
- Configures motor simulation based on ID
- `DIFF_STEERING_VESC_LEFT`: Sets motor 0
- `DIFF_STEERING_VESC_RIGHT`: Sets motor 1

**Usage**:
```c
comm_can_set_vesc_id(VESC_ID);
// Or for specific steering configuration
comm_can_set_vesc_id(DIFF_STEERING_VESC_LEFT);
```

#### `comm_can_lock_vesc()` and `comm_can_unlock_vesc()`
```c
void comm_can_lock_vesc(void);
void comm_can_unlock_vesc(void);
```

**Purpose**: External mutex functions for VESC synchronization

**Usage**:
```c
comm_can_lock_vesc();
// Critical section accessing VESC
comm_can_unlock_vesc();
```

**Note**: These functions provide external access to the VESC mutex

### Thread Functions

#### `cancom_read_thread()`
```c
static THD_FUNCTION(cancom_read_thread, arg)
```

**Purpose**: Continuously reads CAN messages and stores them in circular buffer

**Behavior**:
1. Registers for CAN RX full event
2. Waits for CAN messages or timeout
3. Reads all available CAN messages
4. Stores messages in circular buffer
5. Signals process thread when new data arrives
6. Runs indefinitely (never exits)

**Thread Name**: "CAN read"

**Priority**: `NORMALPRIO + 1`

**Key Features**:
- Event-driven reading (waits for CAN events)
- Processes all available messages in batch
- Efficient circular buffer management
- Low-latency signaling to process thread

**Timeout**: 10ms (prevents thread starvation)

#### `cancom_process_thread()`
```c
static THD_FUNCTION(cancom_process_thread, arg)
```

**Purpose**: Processes CAN messages from circular buffer and routes them appropriately

**Behavior**:
1. Gets own thread pointer for signaling
2. Waits for signal from read thread
3. Processes all available messages in buffer
4. Routes messages based on ID type (extended or standard)
5. Handles VESC communication, IO board messages, UWB messages, etc.
6. Runs indefinitely (never exits)

**Thread Name**: "CAN process"

**Priority**: `NORMALPRIO`

**Key Features**:
- Event-driven processing (waits for data)
- Processes all available data when signaled
- Routes messages to appropriate handlers
- Supports multiple CAN device types

### Message Processing

#### Extended ID Messages (VESC Communication)

The module handles several extended ID message types:

**`CAN_PACKET_FILL_RX_BUFFER`**
- Fills receive buffer with data
- Used for packet fragmentation

**`CAN_PACKET_FILL_RX_BUFFER_LONG`**
- Fills receive buffer with data at specific position
- Used for larger packet fragmentation

**`CAN_PACKET_PROCESS_RX_BUFFER`**
- Processes complete packet from buffer
- Verifies CRC before processing
- Passes to BLDC interface

**`CAN_PACKET_PROCESS_SHORT_BUFFER`**
- Processes short packet (< 7 bytes)
- Directly passes to BLDC interface

**`CAN_PACKET_STATUS`**
- Updates VESC status information
- Stores RPM, current, duty cycle
- Tracks reception time

#### Standard ID Messages

**DW (UWB) Messages**
- **Mask**: `CAN_MASK_DW` (0x700)
- **Commands**:
  - `CMD_DW_RANGE`: Range measurement
  - `CMD_DW_PING`: Ping request
  - `CMD_DW_UPTIME`: Uptime request

**IO Board Messages**
- **Mask**: `CAN_MASK_IO_BOARD` (0x700)
- **Message Types**:
  - ADC voltages (channels 4-7)
  - AS5047 angle sensor
  - Limit switches
  - ADC timing information

**ADDIO Messages**
- **Angle Sensor**: `CAN_ANGLE` (0x19F)
- **Master**: `CAN_ADDIO_MASTER` (0x28)
- **Valve Control**: Node-specific IDs
- **Status Messages**: Node-specific IDs

### Packet Transmission

#### `comm_can_transmit_eid()`
```c
void comm_can_transmit_eid(uint32_t id, uint8_t *data, uint8_t len)
```

**Purpose**: Transmits a CAN message with extended ID

**Parameters**:
- `id`: Extended CAN ID (29-bit)
- `data`: Pointer to data buffer
- `len`: Length of data (1-8 bytes)

**Behavior**:
1. Creates CANTxFrame structure
2. Sets IDE to extended
3. Sets EID to provided ID
4. Copies data to frame
5. Locks CAN mutex
6. Transmits frame with timeout
7. Unlocks CAN mutex

**Timeout**: 20ms

#### `comm_can_transmit_sid()`
```c
void comm_can_transmit_sid(uint32_t id, uint8_t *data, uint8_t len)
```

**Purpose**: Transmits a CAN message with standard ID

**Parameters**:
- `id`: Standard CAN ID (11-bit)
- `data`: Pointer to data buffer
- `len`: Length of data (1-8 bytes)

**Behavior**: Similar to `comm_can_transmit_eid()` but uses standard ID

#### `comm_can_send_buffer()`
```c
void comm_can_send_buffer(uint8_t controller_id, uint8_t *data, unsigned int len, bool send)
```

**Purpose**: Sends a buffer as CAN messages, handling fragmentation if needed

**Parameters**:
- `controller_id`: Destination controller ID
- `data`: Pointer to data buffer
- `len`: Length of data
- `send`: Whether to send or process the packet

**Fragmentation Strategy**:
- **Short packets** (<= 6 bytes):
  - Sent in single frame
  - Includes header information
  
- **Long packets** (> 6 bytes):
  1. Fill buffer in 7-byte chunks (first 256 bytes)
  2. Fill buffer in 6-byte chunks (remaining bytes)
  3. Send process command with CRC

**Example**:
```c
uint8_t packet[64];
// Fill packet...
comm_can_send_buffer(VESC_ID, packet, sizeof(packet), false);
```

### UWB (DW) Functions

#### `comm_can_dw_range()`
```c
void comm_can_dw_range(uint8_t id, uint8_t dest, int samples)
```

**Purpose**: Start ranging with a DW node

**Parameters**:
- `id`: Source DW node ID
- `dest`: Destination node ID
- `samples`: Number of samples to average

**Usage**: Initiates range measurement between two DW nodes

#### `comm_can_dw_ping()`
```c
void comm_can_dw_ping(uint8_t id)
```

**Purpose**: Send ping to DW node

**Parameters**:
- `id`: DW node ID

**Usage**: Checks if DW node is responsive

#### `comm_can_dw_reboot()`
```c
void comm_can_dw_reboot(uint8_t id)
```

**Purpose**: Reboot DW node

**Parameters**:
- `id`: DW node ID

**Usage**: Reboots the specified DW node

#### `comm_can_dw_get_uptime()`
```c
void comm_can_dw_get_uptime(uint8_t id)
```

**Purpose**: Get uptime from DW node

**Parameters**:
- `id`: DW node ID

**Usage**: Requests uptime information from DW node

#### Callback Functions

```c
void comm_can_set_range_func(void(*func)(uint8_t id, uint8_t dest, float range));
void comm_can_set_dw_ping_func(void(*func)(uint8_t uptime));
void comm_can_set_dw_uptime_func(void(*func)(uint8_t id, uint32_t uptime));
```

**Purpose**: Set callback functions for DW events

**Usage**:
```c
void my_range_callback(uint8_t id, uint8_t dest, float range) {
    // Handle range result
}

comm_can_set_range_func(my_range_callback);
```

### Status Message Functions

#### `comm_can_get_status_msg_index()`
```c
can_status_msg *comm_can_get_status_msg_index(int index)
```

**Purpose**: Get status message by index

**Parameters**:
- `index`: Index in the status message array (0-9)

**Return**: Pointer to status message or NULL for invalid index

**Usage**:
```c
can_status_msg *status = comm_can_get_status_msg_index(0);
if (status) {
    // Use status information
}
```

#### `comm_can_get_status_msg_id()`
```c
can_status_msg *comm_can_get_status_msg_id(int id)
```

**Purpose**: Get status message by VESC ID

**Parameters**:
- `id`: VESC controller ID

**Return**: Pointer to status message or NULL for invalid ID

**Usage**:
```c
can_status_msg *status = comm_can_get_status_msg_id(VESC_ID);
if (status) {
    // Use status information
}
```

### IO Board Functions

#### `comm_can_io_board_adc_voltage()`
```c
float comm_can_io_board_adc_voltage(int ch)
```

**Purpose**: Get ADC voltage from IO board

**Parameters**:
- `ch`: ADC channel (0-7)

**Return**: Voltage reading or 0.0 for invalid channel

**Usage**:
```c
float voltage = comm_can_io_board_adc_voltage(4);
```

#### `comm_can_io_board_as5047_angle()`
```c
float comm_can_io_board_as5047_angle(void)
```

**Purpose**: Get angle from AS5047 sensor

**Return**: Angle in degrees

**Usage**:
```c
float angle = comm_can_io_board_as5047_angle();
```

#### `comm_can_io_board_as5047_setangle()`
```c
void comm_can_io_board_as5047_setangle(float angle)
```

**Purpose**: Set angle for AS5047 sensor (for testing)

**Parameters**:
- `angle`: Angle to set

**Usage**:
```c
comm_can_io_board_as5047_setangle(45.0);
```

#### `comm_can_io_board_lim_sw()`
```c
bool comm_can_io_board_lim_sw(int sw)
```

**Purpose**: Get limit switch state

**Parameters**:
- `sw`: Limit switch number (0-7)

**Return**: State of limit switch

**Usage**:
```c
bool limit_state = comm_can_io_board_lim_sw(0);
```

#### `comm_can_io_board_adc0_cnt()`
```c
ADC_CNT_t* comm_can_io_board_adc0_cnt(void)
```

**Purpose**: Get ADC0 counter information

**Return**: Pointer to ADC counter structure

**Usage**:
```c
ADC_CNT_t *cnt = comm_can_io_board_adc0_cnt();
// Access counter fields
```

### ADDIO Functions

#### `comm_can_addio_lim_sw()`
```c
bool comm_can_addio_lim_sw(int sw)
```

**Purpose**: Get ADDIO limit switch state

**Parameters**:
- `sw`: Limit switch number (0-7)

**Return**: State of limit switch

**Usage**:
```c
bool limit_state = comm_can_addio_lim_sw(0);
```

#### `comm_can_ftr2_angle()`
```c
float comm_can_ftr2_angle(void)
```

**Purpose**: Get angle from FTR2 sensor

**Return**: Angle in degrees

**Behavior**:
- If sensor not activated, sends request frame
- Returns cached angle value

**Usage**:
```c
float angle = comm_can_ftr2_angle();
```

#### `comm_can_addio_set_valve()`
```c
void comm_can_addio_set_valve(int valve, bool set)
```

**Purpose**: Set ADDIO valve state

**Parameters**:
- `valve`: Valve number (0-7)
- `set`: Valve state (true = open, false = closed)

**Usage**:
```c
comm_can_addio_set_valve(1, true);  // Open valve 1
comm_can_addio_set_valve(1, false); // Close valve 1
```

#### `comm_can_addio_set_valve_duty()`
```c
void comm_can_addio_set_valve_duty(float duty)
```

**Purpose**: Set ADDIO valve duty cycle

**Parameters**:
- `duty`: Duty cycle (-1.0 to 1.0)

**Conversion**: Duty cycle converted to Vpoc setpoint (-16384 to +16384)

**Usage**:
```c
comm_can_addio_set_valve_duty(0.5);  // 50% duty cycle
comm_can_addio_set_valve_duty(-0.3); // -30% duty cycle
```

### IO Board Control Functions

#### `comm_can_io_board_set_valve()`
```c
void comm_can_io_board_set_valve(int board, int valve, bool set)
```

**Purpose**: Set IO board valve state

**Parameters**:
- `board`: IO board number (0-15)
- `valve`: Valve number
- `set`: Valve state

**Usage**:
```c
comm_can_io_board_set_valve(0, 1, true);
```

#### `comm_can_io_board_set_pwm_duty()`
```c
void comm_can_io_board_set_pwm_duty(int board, float duty)
```

**Purpose**: Set IO board PWM duty cycle

**Parameters**:
- `board`: IO board number (0-15)
- `duty`: Duty cycle

**Usage**:
```c
comm_can_io_board_set_pwm_duty(0, 0.5);
```

## Communication Flow

### Data Reception Path

1. **CAN Hardware** → Receives CAN messages
2. **Read Thread** → Reads messages from CAN peripheral
3. **Read Thread** → Stores messages in circular buffer
4. **Read Thread** → Signals process thread
5. **Process Thread** → Reads messages from buffer
6. **Process Thread** → Routes messages based on ID type
7. **Process Thread** → Calls appropriate handler functions

### Data Transmission Path

1. **Application** → Calls transmit function
2. **Transmit Function** → Locks CAN mutex
3. **CAN Peripheral** → Transmits CAN message
4. **Transmit Function** → Unlocks CAN mutex

## Thread Interaction

### Thread Communication

**Read Thread → Process Thread**:
- **Mechanism**: Event signaling (`chEvtSignal`)
- **Trigger**: When CAN message is stored in buffer
- **Purpose**: Notify process thread to process data

**Process Thread → Read Thread**:
- **Mechanism**: Circular buffer positions
- **Read Position**: Process thread reads from buffer
- **Write Position**: Read thread writes to buffer
- **Synchronization**: No explicit synchronization needed (single writer/single reader)

### Thread Priorities

- **Read Thread**: `NORMALPRIO + 1` (higher priority)
- **Process Thread**: `NORMALPRIO`

**Rationale**:
- Read thread has higher priority to ensure no CAN messages are lost
- Process thread can wait for data without affecting CAN reception

## Error Handling

### CAN Errors
- **Detection**: CAN peripheral handles hardware errors
- **Recovery**: Automatic retry on next transmission
- **Logging**: Minimal (errors may be silently handled)

### Buffer Overflow
- **Detection**: Circular buffer wrap-around
- **Recovery**: Oldest messages overwritten by new messages
- **Prevention**: 100-frame buffer should handle typical loads

### Packet Loss
- **Fragmentation**: May lose packets if buffer overflows
- **Status Messages**: May lose oldest status if buffer full
- **Impact**: Minimal if CAN traffic is within expected limits

## Configuration Options

### CAN Baud Rate
- **ADDIO Mode**: 5125KBaud
- **Default Mode**: 500KBaud
- **Configuration**: Defined by `cancfg` structure

### Buffer Sizes
- **Frame Buffer**: 100 frames
- **Packet Buffer**: `PACKET_MAX_PL_LEN` bytes
- **Status Messages**: 10 messages

### Thread Priorities
- **Read Thread**: `NORMALPRIO + 1`
- **Process Thread**: `NORMALPRIO`
- **Adjustable**: Can be changed based on system requirements

## Usage Examples

### Initialization
```c
// Standard initialization sequence
can_hw_init();      // Initialize CAN hardware
comm_can_init();    // Initialize CAN communication

// Set VESC ID if needed
comm_can_set_vesc_id(VESC_ID);

// Set callback functions
comm_can_set_range_func(my_range_callback);
```

### Sending Data
```c
// Simple CAN message
uint8_t data[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
comm_can_transmit_eid(0x123456, data, 8);

// Long packet (automatic fragmentation)
uint8_t long_data[128];
// Fill data...
comm_can_send_buffer(VESC_ID, long_data, sizeof(long_data), false);
```

### Receiving Data
```c
// Data reception is automatic
// Status messages can be accessed
can_status_msg *status = comm_can_get_status_msg_id(VESC_ID);
if (status) {
    float rpm = status->rpm;
    float current = status->current;
    float duty = status->duty;
}

// IO board data
float angle = comm_can_io_board_as5047_angle();
float voltage = comm_can_io_board_adc_voltage(4);
```

### UWB Ranging
```c
// Set up callback
void my_range_callback(uint8_t id, uint8_t dest, float range) {
    commands_printf("Range: %f meters", range);
}

comm_can_set_range_func(my_range_callback);

// Start ranging
comm_can_dw_range(1, 2, 10);  // Node 1 to node 2, 10 samples
```

### Valve Control
```c
// ADDIO valve control
comm_can_addio_set_valve(1, true);   // Open valve 1
comm_can_addio_set_valve_duty(0.5);  // 50% duty cycle

// IO board valve control
comm_can_io_board_set_valve(0, 1, true);
```

## Performance Considerations

### Throughput
- **Limited by**: CAN bus speed (500KBaud or 5125KBaud)
- **Actual**: Much lower due to protocol overhead
- **Fragmentation**: Adds overhead for packets > 8 bytes

### Latency
- **Read Thread**: Very low (event-driven)
- **Process Thread**: Depends on signaling latency
- **Total**: Typically < 1ms for complete message processing

### Memory Usage
- **Frame Buffer**: 100 × sizeof(CANRxFrame) ≈ 1.6KB
- **Packet Buffer**: `PACKET_MAX_PL_LEN` ≈ 256 bytes
- **Status Messages**: 10 × sizeof(can_status_msg) ≈ 0.5KB
- **Threads**: 4608 bytes total stack space
- **Mutexes**: Minimal overhead
- **Total**: ~6.5KB for CAN communication module

### CAN Bus Load
- **Typical**: Low to moderate
- **Peak**: Can handle bursty traffic with 100-frame buffer
- **Limitations**: No adaptive rate control (fixed baud rate)

## Compatibility

### Hardware
- **CAN Interface**: Standard CAN 2.0
- **Baud Rates**: 500KBaud or 5125KBaud
- **Devices**:
  - VESC motor controllers
  - IO boards with sensors
  - UWB (DW) modules
  - ADDIO valve controllers
  - FTR2 angle sensors

### Software
- **ChibiOS**: Required for thread and synchronization primitives
- **CAN Driver**: Required for low-level CAN access
- **Packet Framework**: Required for packet processing

## Future Improvements

### Potential Enhancements
1. **Dynamic Baud Rate**: Adjust based on network conditions
2. **Larger Buffers**: For higher throughput applications
3. **Error Reporting**: Notify application of lost messages
4. **Priority Messages**: Support for time-critical messages
5. **Adaptive Rate Control**: Adjust transmission rate dynamically
6. **Message Filtering**: Hardware filtering for specific IDs

### Current Limitations
1. **Fixed Buffer Size**: Cannot adapt to different traffic patterns
2. **Simple Error Handling**: No sophisticated error recovery
3. **No Priority**: All messages treated equally
4. **No Flow Control**: Assumes CAN bus can handle traffic
5. **Limited Status Tracking**: Only 10 VESC status messages

## Troubleshooting

### Common Issues

**Issue**: CAN communication stops working
- **Possible Causes**:
  - CAN bus off (too many errors)
  - Wrong baud rate configuration
  - Electrical issues (termination, noise)
- **Solutions**:
  - Check CAN bus termination
  - Verify baud rate settings match all devices
  - Check for short circuits or noise
  - Monitor CAN error counters

**Issue**: Messages lost or corrupted
- **Possible Causes**:
  - Buffer overflow
  - CAN bus errors
  - Electrical issues
- **Solutions**:
  - Increase buffer sizes
  - Check CAN bus termination
  - Reduce CAN bus load
  - Check for electrical issues

**Issue**: High latency
- **Possible Causes**:
  - Process thread blocked
  - High system load
  - CAN bus saturation
- **Solutions**:
  - Increase thread priority
  - Reduce CAN bus load
  - Check for system bottlenecks
  - Monitor CAN bus utilization

**Issue**: VESC not responding
- **Possible Causes**:
  - Wrong VESC ID
  - VESC not powered
  - CAN connection issue
- **Solutions**:
  - Verify VESC ID with `comm_can_set_vesc_id()`
  - Check VESC power
  - Verify CAN wiring
  - Check status messages

### Debugging Tips

1. **Check Status Messages**:
   ```c
   can_status_msg *status = comm_can_get_status_msg_id(VESC_ID);
   if (!status) {
       // VESC not responding
   }
   ```

2. **Monitor CAN Traffic**: Use CAN analyzer to verify messages

3. **Check Error Counters**: Monitor CAN error counters in status register

4. **Test with Simple Messages**: Send simple test messages to verify basic connectivity

5. **Isolate Devices**: Disconnect devices one by one to identify faulty hardware

## References

### Related Files
- `comm_can.h`: Header file with function declarations
- `bldc_interface.c`: BLDC interface for VESC communication
- `packet.c`: Packet processing framework
- `commands.c`: Command processing module

### External Dependencies
- ChibiOS RTOS: Threads, mutexes, events
- STM32 CAN Driver: Low-level CAN access
- VESC Firmware: For VESC controllers
- IO Board Firmware: For IO boards
- UWB Firmware: For DW modules

### CAN Standards
- **CAN 2.0A**: Standard CAN (11-bit IDs)
- **CAN 2.0B**: Extended CAN (29-bit IDs)
- **CAN FD**: Not currently supported

## Conclusion

The CAN communication module provides a robust and efficient way to communicate between the embedded controller and various CAN devices. Its dual-thread architecture with circular buffering ensures reliable message handling even under moderate load conditions. The module supports multiple device types and provides comprehensive status monitoring.

The module is well-suited for the RC_Controller application, providing reliable communication with VESC motor controllers, IO boards, UWB modules, and ADDIO valve controllers. The fragmentation and reassembly capabilities allow for efficient data transfer even with CAN's 8-byte message limit.
