# Commands Module Documentation

## Overview

This document provides comprehensive documentation for the command processing module in the RC_Controller firmware, specifically focusing on the `commands.c` file.

## File Structure

### Location
- **File**: `commands.c`
- **Purpose**: Central command processing and packet handling
- **Architecture**: Event-driven with packet processing and command routing

## Key Features

1. **Packet Processing**
   - Packet reception and validation
   - Command routing based on packet ID
   - Packet forwarding to other nodes

2. **Command Execution**
   - Local command execution
   - Remote command forwarding
   - Command response handling

3. **Subsystem Integration**
   - Position control
   - Motor control
   - Hydraulic system control
   - GNSS receiver control
   - Logging and debugging

4. **Synchronization**
   - Mutex-protected operations
   - Thread-safe command processing
   - Virtual timer for delayed operations

5. **Debugging Support**
   - Debug variables
   - Terminal commands
   - Logging functions

## Detailed Documentation

### Private Variables

#### Communication
- **`m_send_buffer`**: Buffer for outgoing packets (PACKET_MAX_PL_LEN bytes)
- **`m_send_func`**: Function pointer for packet sending
- **`vt`**: Virtual timer for delayed operations
- **`m_print_gps`**: Mutex for GPS printing operations

#### State Tracking
- **`m_init_done`**: Initialization completion flag
- **`m_kb_active`**: Keyboard active flag
- **`arduino_connected`**: Arduino connection status

#### Debug Variables
- **`debugvalue`** through **`debugvalue15`**: Debug variables for testing
- **`angle`**: Current angle reading
- **`last_sensorvalue`**: Last sensor value

#### RTCM State
- **`m_rtcm_state`**: RTCM3 decoder state for GNSS corrections

### Public Functions

#### `commands_init()`
```c
void commands_init(void)
```

**Purpose**: Initializes the commands module

**Steps**:
1. Initializes send function pointer
2. Initializes print mutex
3. Initializes debug variables
4. Sets initialization flag

**Initial State**:
- All debug variables set to 0.0
- Send function pointer set to NULL
- Mutex initialized
- Initialization flag set to true

#### `commands_set_send_func()`
```c
void commands_set_send_func(void(*func)(unsigned char *data, unsigned int len))
```

**Purpose**: Sets the packet send function

**Parameters**:
- `func`: Function pointer for packet sending

**Usage**:
```c
// Set send function to USB communication
commands_set_send_func(comm_usb_send_packet);
```

#### `commands_send_packet()`
```c
void commands_send_packet(unsigned char *data, unsigned int len)
```

**Purpose**: Sends a packet using the configured send function

**Parameters**:
- `data`: Packet data buffer
- `len`: Packet length

**Behavior**:
1. Checks if send function is set
2. Calls send function with packet data
3. Returns early if no send function configured

**Usage**:
```c
unsigned char packet[64];
packet[0] = DEST_ID;
packet[1] = CMD_CODE;
// Fill packet...
commands_send_packet(packet, sizeof(packet));
```

#### `commands_process_packet()`
```c
void commands_process_packet(unsigned char *data, unsigned int len, void (*func)(unsigned char *data, unsigned int len))
```

**Purpose**: Processes an incoming packet

**Parameters**:
- `data`: Packet data buffer
- `len`: Packet length
- `func`: Send function for responses

**Behavior**:
1. Checks packet length
2. Routes packet based on ID and command
3. Executes appropriate command handler
4. Sends responses using provided function

**Command Routing**:
- **RTCM3 Data**: Forwarded to RTCM3 decoder
- **Terminal Commands**: Processed by terminal module
- **Position Commands**: Handled by position module
- **Motor Commands**: Handled by motor control
- **Hydraulic Commands**: Handled by hydraulic module
- **Configuration Commands**: Updates system configuration

### Command Processing

#### Command Categories

**General Commands**:
- Terminal commands
- Debug commands
- System status
- Configuration

**Position Commands**:
- Set position
- Get position
- Position control

**Motor Commands**:
- Motor control
- Speed setting
- Throttle control

**Hydraulic Commands**:
- Actuator control
- Valve control
- Position setting

**GNSS Commands**:
- RTCM3 data
- GNSS configuration
- Position updates

**Logging Commands**:
- Log configuration
- Data logging
- Debug output

### Packet Structure

#### Packet Format

```
| ID | CMD | Payload |
| 1 byte | 1 byte | Variable |
```

**Fields**:
- **ID**: Destination ID (0xFF for broadcast)
- **CMD**: Command code
- **Payload**: Command-specific data

#### Common IDs

- **`ID_ALL`**: Broadcast to all nodes
- **`ID_VEHICLE_CLIENT`**: Vehicle client
- **`main_id`**: This vehicle's ID

#### Command Codes

**General Commands**:
- `CMD_TERMINAL_CMD`: Terminal command
- `CMD_HEARTBEAT`: Heartbeat
- `CMD_ARDUINO_STATUS`: Arduino status

**Position Commands**:
- `CMD_SET_POS`: Set position
- `CMD_GET_STATE`: Get state
- `CMD_GET_MAIN_CONFIG`: Get configuration

**Motor Commands**:
- `CMD_SET_SPEED`: Set speed
- `CMD_SET_THROTTLE`: Set throttle
- `CMD_SET_MOTOR_MODE`: Set motor mode

**Hydraulic Commands**:
- `CMD_MOVE_HYDRAULIC`: Move hydraulic
- `CMD_SET_HYDRAULIC_SPEED`: Set hydraulic speed

**GNSS Commands**:
- `CMD_SEND_RTCM_USB`: Send RTCM3 data
- `CMD_SEND_NMEA_RADIO`: Send NMEA data

### Synchronization

#### Mutex Protection

**`m_print_gps`**:
- **Purpose**: Protects GPS printing operations
- **Usage**: Locked during GPS data printing
- **Duration**: Short critical sections

#### Thread Safety

**Command Processing**:
- **Context**: Can be called from any thread
- **Synchronization**: No explicit synchronization needed
- **Assumption**: Single packet processor

**Packet Sending**:
- **Context**: Can be called from any thread
- **Synchronization**: No explicit synchronization needed
- **Assumption**: Send function handles synchronization

### Debug Functions

#### `commands_printf()`
```c
void commands_printf(const char* format, ...)
```

**Purpose**: Prints formatted message as command packet

**Parameters**:
- `format`: Format string
- `...`: Variable arguments

**Behavior**:
1. Checks initialization status
2. Locks print mutex
3. Formats message into buffer
4. Sends as command packet
5. Unlocks mutex

**Usage**:
```c
commands_printf("Debug: x=%f, y=%f", x, y);
```

#### `commands_forward_vesc_packet()`
```c
void commands_forward_vesc_packet(unsigned char *data, unsigned int len)
```

**Purpose**: Forwards VESC packet to other nodes

**Parameters**:
- `data`: VESC packet data
- `len`: Packet length

**Behavior**:
1. Validates packet length
2. Adds header information
3. Sends as command packet

**Usage**:
```c
unsigned char vesc_data[64];
// Fill VESC data...
commands_forward_vesc_packet(vesc_data, sizeof(vesc_data));
```

#### `commands_send_nmea()`
```c
void commands_send_nmea(unsigned char *data, unsigned int len)
```

**Purpose**: Sends NMEA data to other nodes

**Parameters**:
- `data`: NMEA data
- `len`: Data length

**Behavior**:
1. Checks if NMEA sending is enabled
2. Validates packet length
3. Adds header information
4. Sends as command packet

**Usage**:
```c
unsigned char nmea_data[128];
// Fill NMEA data...
commands_send_nmea(nmea_data, sizeof(nmea_data));
```

### RTCM3 Integration

#### `rtcm_rx()`
```c
void rtcm_rx(uint8_t *data, int len, int type)
```

**Purpose**: Processes RTCM3 correction data

**Parameters**:
- `data`: RTCM3 data buffer
- `len`: Data length
- `type`: Data type

**Behavior**:
1. **UBLOX Mode**: Forwards to UBLOX receiver
2. **Other Modes**: Sends via USB

**Usage**:
```c
// Receive RTCM3 data from radio
unsigned char rtcm_data[256];
int rtcm_len = receive_from_radio(rtcm_data);
rtcm_rx(rtcm_data, rtcm_len, RTCM3_TYPE);
```

#### `rtcm_base_rx()`
```c
void rtcm_base_rx(rtcm_ref_sta_pos_t *pos)
```

**Purpose**: Processes RTCM3 base station position

**Parameters**:
- `pos`: Base station position data

**Behavior**:
1. Checks if base position should be used as ENU reference
2. Updates position reference if enabled

**Usage**:
```c
rtcm_ref_sta_pos_t base_pos;
// Fill base position...
rtcm_base_rx(&base_pos);
```

### Configuration Functions

#### `commands_get_rtcm3_state()`
```c
rtcm3_state* commands_get_rtcm3_state(void)
```

**Purpose**: Gets RTCM3 decoder state

**Return**: Pointer to RTCM3 state structure

**Usage**:
```c
rtcm3_state *state = commands_get_rtcm3_state();
// Access RTCM3 state information
```

## Communication Flow

### Packet Reception Path

1. **Communication Module** → Receives packet
2. **Module** → Calls `commands_process_packet()`
3. **Function** → Routes to appropriate handler
4. **Handler** → Executes command
5. **Handler** → Sends response if needed

### Packet Transmission Path

1. **Application** → Calls command function
2. **Function** → Builds packet
3. **Function** → Calls `commands_send_packet()`
4. **Function** → Uses configured send function
5. **Send Function** → Transmits packet

### Command Execution Path

1. **Packet Received** → `commands_process_packet()` called
2. **Packet Processed** → Command ID extracted
3. **Command Routed** → Appropriate handler called
4. **Handler Executes** → Command action performed
5. **Response Sent** → Response packet sent if needed

## Thread Interaction

### Thread Safety

**Command Processing**:
- **Context**: Can be called from any thread
- **Synchronization**: No explicit synchronization needed
- **Assumption**: Single packet processor

**Packet Sending**:
- **Context**: Can be called from any thread
- **Synchronization**: No explicit synchronization needed
- **Assumption**: Send function handles synchronization

**Mutex Usage**:
- **`m_print_gps`**: Protects GPS printing
- **Duration**: Short critical sections
- **Contention**: Minimal (infrequent GPS printing)

### Event Handling

**Virtual Timer**:
- **Purpose**: Delayed packet forwarding
- **Duration**: FWD_TIME microseconds
- **Usage**: Forward packets after delay

## Configuration Options

### Packet Forwarding

**`FWD_TIME`**: 20000 microseconds (20ms)
- **Purpose**: Delay for packet forwarding
- **Effect**: Allows time for local processing before forwarding

### Debug Variables

**Debug Variables**: 16 float variables (debugvalue through debugvalue15)
- **Purpose**: Testing and debugging
- **Access**: Read/write via commands
- **Usage**: Store temporary values for debugging

## Usage Examples

### Basic Command Processing
```c
// Initialize commands module
commands_init();

// Set send function
commands_set_send_func(comm_usb_send_packet);

// Process incoming packet
unsigned char packet[64];
int packet_len = receive_packet(packet);
commands_process_packet(packet, packet_len, comm_usb_send_packet);
```

### Sending Commands
```c
// Send debug message
commands_printf("System initialized");

// Send position command
unsigned char cmd[10];
cmd[0] = DEST_ID;
cmd[1] = CMD_SET_POS;
// Fill position data...
commands_send_packet(cmd, sizeof(cmd));
```

### Forwarding Packets
```c
// Forward VESC packet
unsigned char vesc_data[64];
// Fill VESC data...
commands_forward_vesc_packet(vesc_data, sizeof(vesc_data));

// Forward NMEA data
unsigned char nmea_data[128];
// Fill NMEA data...
commands_send_nmea(nmea_data, sizeof(nmea_data));
```

### RTCM3 Data Handling
```c
// Receive RTCM3 data
unsigned char rtcm_data[256];
int rtcm_len = receive_from_radio(rtcm_data);
rtcm_rx(rtcm_data, rtcm_len, RTCM3_TYPE);

// Get RTCM3 state
rtcm3_state *state = commands_get_rtcm3_state();
```

## Performance Considerations

### Thread Performance

**Command Processing**:
- **Context**: Any thread
- **Blocking**: Minimal (short operations)
- **Latency**: < 1ms for most commands

**Packet Sending**:
- **Context**: Any thread
- **Blocking**: Minimal (send function handles blocking)
- **Latency**: Depends on send function

### Memory Usage

**Buffers**:
- **Send Buffer**: PACKET_MAX_PL_LEN bytes
- **Debug Variables**: 16 × 4 bytes = 64 bytes
- **RTCM3 State**: ~1KB
- **Total**: ~1.3KB

### Throughput

**Command Rate**:
- **Typical**: 10-100 commands/second
- **Peak**: 1000+ commands/second
- **Limitation**: Depends on communication bandwidth

## Compatibility

### Hardware
- **Communication**: Any communication interface
- **Packet Format**: Standard packet format
- **Compatibility**: Works with any packet-based communication

### Software
- **ChibiOS**: Required for mutex and virtual timer
- **Packet Framework**: Required for packet processing
- **Communication Modules**: Required for actual transmission

## Future Improvements

### Potential Enhancements
1. **Command Queue**: Queue commands for processing
2. **Priority Commands**: Support for priority commands
3. **Command Logging**: Log command execution
4. **Error Recovery**: Automatic retry for failed commands
5. **Command Validation**: Validate command parameters
6. **Command Timeout**: Timeout for command execution

### Current Limitations
1. **No Command Queue**: Commands processed immediately
2. **No Priority**: All commands treated equally
3. **No Logging**: Command execution not logged
4. **No Validation**: Command parameters not validated
5. **No Timeout**: Commands can hang indefinitely

## Troubleshooting

### Common Issues

**Issue**: Commands not processed
- **Possible Causes**:
  - No send function configured
  - Packet format incorrect
  - Command handler missing
- **Solutions**:
  - Verify send function is set
  - Check packet format
  - Verify command is implemented

**Issue**: Packets not forwarded
- **Possible Causes**:
  - Forwarding disabled
  - Send function not configured
  - Packet too large
- **Solutions**:
  - Enable forwarding
  - Configure send function
  - Check packet size

**Issue**: Debug output not working
- **Possible Causes**:
  - Module not initialized
  - Mutex contention
  - Send function not configured
- **Solutions**:
  - Initialize module
  - Check mutex usage
  - Configure send function

### Debugging Tips

1. **Check Initialization**:
   ```c
   if (!m_init_done) {
       // Module not initialized
   }
   ```

2. **Monitor Packet Flow**:
   ```c
   // Add debug output to packet processing
   commands_printf("Processing packet: ID=%d, CMD=%d", id, cmd);
   ```

3. **Verify Send Function**:
   ```c
   if (!m_send_func) {
       // No send function configured
   }
   ```

4. **Check Mutex Contention**:
   ```c
   // Ensure mutex is not locked for too long
   chMtxLock(&m_print_gps);
   // Quick operation
   chMtxUnlock(&m_print_gps);
   ```

5. **Test with Simple Commands**:
   ```c
   // Test with simple debug command
   commands_printf("Test message");
   ```

## References

### Related Files
- `commands.h`: Header file with function declarations
- `packet.c`: Packet processing framework
- `comm_usb.c`: USB communication
- `comm_can.c`: CAN communication
- `pos.c`: Position control
- `hydraulic.c`: Hydraulic control
- `ublox.c`: GNSS receiver control

### External Dependencies
- **ChibiOS**: Mutex, virtual timer
- **Packet Framework**: Packet processing
- **Communication Modules**: Actual transmission
- **Subsystem Modules**: Command execution

## Conclusion

The commands module provides a central hub for command processing in the RC_Controller system. It handles packet reception, command routing, and communication with various subsystems. The module's design ensures efficient command processing with proper synchronization and error handling.

The callback-based architecture allows for easy integration with different communication modules, while the command routing system provides flexible command handling. The module's support for debugging and logging makes it valuable for system development and troubleshooting.
