# UBLOX GNSS Module Documentation

## Overview

This document provides comprehensive documentation for the UBLOX GNSS module in the RC_Controller firmware, specifically focusing on the `ublox.c` file.

## File Structure

### Location
- **File**: `ublox.c`
- **Purpose**: Communication with UBLOX GNSS receivers (F9P, M8N, etc.)
- **Architecture**: Single-thread design with UART communication and message parsing

## Key Features

1. **GNSS Protocol Support**
   - UBX binary protocol parsing
   - NMEA sentence parsing
   - RTCM3 correction data handling

2. **Multi-Constellation Support**
   - GPS
   - GLONASS
   - Galileo
   - BeiDou
   - QZSS

3. **Message Processing**
   - Navigation solutions (NAV-SOL)
   - Relative position (RELPOSNED)
   - Raw measurements (RAWX)
   - Satellite status (NAV-SAT)
   - Configuration acknowledgments (ACK/NAK)

4. **Configuration**
   - GNSS constellation enable/disable
   - Message rate configuration
   - UART baud rate configuration
   - Reset and initialization

5. **Thread-Based Processing**
   - Dedicated thread for message processing
   - Circular buffer for UART data
   - Event-driven processing

## Detailed Documentation

### Hardware Configuration

#### UART Settings
- **Device**: `UARTD6`
- **TX Pin**: GPIOC, Pin 6
- **RX Pin**: GPIOC, Pin 7
- **Reset Pin**: GPIOC, Pin 9

#### Baud Rates
- **Default (Unconfigured)**: 38400 (F9P) or 9600 (other)
- **Operational**: 115200
- **Configuration**: Automatic baud rate switching

### Buffer Configuration

#### Buffer Sizes
- **Serial RX Buffer**: 1024 bytes
- **Line Buffer**: 256 bytes (for NMEA)
- **UBX Buffer**: 3000 bytes (for UBX messages)

#### Circular Buffer
- **Purpose**: Store incoming UART data
- **Size**: 1024 bytes
- **Access**: Thread-safe via interrupt service
- **Overflow**: Wraps around (oldest data lost)

### Private Types

#### `decoder_state`
```c
typedef struct {
    uint8_t line[LINE_BUFFER_SIZE];
    uint8_t ubx[UBX_BUFFER_SIZE];
    int line_pos;
    int ubx_pos;
    uint8_t ubx_class;
    uint8_t ubx_id;
    uint8_t ubx_ck_a;
    uint8_t ubx_ck_b;
    int ubx_len;
} decoder_state;
```

**Purpose**: Tracks state during message decoding

**Fields**:
- `line`: NMEA sentence buffer
- `ubx`: UBX message buffer
- `line_pos`: Current position in line buffer
- `ubx_pos`: Current position in UBX buffer
- `ubx_class`: Current UBX message class
- `ubx_id`: Current UBX message ID
- `ubx_ck_a`, `ubx_ck_b`: Checksum bytes
- `ubx_len`: Expected message length

### Thread Configuration

#### `process_thread`
- **Stack Size**: 4096 bytes
- **Priority**: NORMALPRIO
- **Purpose**: Process incoming UART data
- **Behavior**: Event-driven (waits for data)

**Main Loop**:
1. Wait for data event
2. Process all available bytes
3. Parse NMEA or UBX messages
4. Route messages to appropriate handlers
5. Call callback functions

### Private Variables

#### UART Communication
- **`m_serial_rx_buffer`**: Circular buffer for UART data (1024 bytes)
- **`m_serial_rx_read_pos`**: Read position in buffer
- **`m_serial_rx_write_pos`**: Write position in buffer

#### RTCM State
- **`m_rtcm_state`**: RTCM3 decoder state
- **Purpose**: Process RTCM3 correction data
- **Integration**: With `rtcm3_simple` module

#### Print Flags
- **`m_print_next_nav_sol`**: Print next NAV-SOL message
- **`m_print_next_relposned`**: Print next RELPOSNED message
- **`m_print_next_rawx`**: Print next RAWX message
- **`m_print_next_svin`**: Print next SVIN message
- **`m_print_next_nav_sat`**: Print next NAV-SAT message
- **`m_print_next_mon_ver`**: Print next MON-VER message
- **`m_print_next_cfg_gnss`**: Print next CFG-GNSS message

#### Decoder State
- **`m_decoder_state`**: Current decoder state
- **Purpose**: Track parsing state across iterations

### Public Functions

#### `ublox_init()`
```c
void ublox_init(void)
```

**Purpose**: Initializes the UBLOX module

**Steps**:
1. Resets decoder state
2. Configures GPIO pins
3. Initializes UART with default baud rate
4. Creates process thread
5. Configures GNSS receiver
6. Sets operational baud rate

**Initialization Sequence**:
1. **Default Baud Rate**: 38400 or 9600
2. **Configuration**: Sends UBX configuration messages
3. **Baud Rate Change**: Switches to 115200
4. **Final Configuration**: Additional settings

**Thread Creation**:
- Process thread: 4096-byte stack, NORMALPRIO

#### `ublox_send()`
```c
void ublox_send(unsigned char *data, unsigned int len)
```

**Purpose**: Sends RTCM3 correction data to UBLOX

**Parameters**:
- `data`: RTCM3 data buffer
- `len`: Length of data

**Behavior**:
1. Calls `rtcm_rx()` to process data
2. Forwards to `rtcm3_input_data()`
3. No direct UART transmission (handled by RTCM3 module)

**Usage**:
```c
// Send RTCM3 correction data
ublox_send(rtcm_data, rtcm_len);
```

#### Callback Registration Functions

```c
void ublox_set_rx_callback_nav_sol(void(*func)(ubx_nav_sol *sol));
void ublox_set_rx_callback_relposned(void(*func)(ubx_nav_relposned *pos));
void ublox_set_rx_callback_rawx(void(*func)(ubx_rxm_rawx *rawx));
void ublox_set_rx_callback_svin(void(*func)(ubx_nav_svin *svin));
void ublox_set_rx_callback_nav_sat(void(*func)(ubx_nav_sat *sat));
void ublox_set_rx_callback_cfg_gnss(void(*func)(ubx_cfg_gnss *gnss));
```

**Purpose**: Register callback functions for GNSS messages

**Usage**:
```c
// Set callback for navigation solution
void my_nav_sol_callback(ubx_nav_sol *sol) {
    // Handle navigation solution
}

ublox_set_rx_callback_nav_sol(my_nav_sol_callback);
```

#### `ublox_poll()`
```c
void ublox_poll(uint8_t msg_class, uint8_t id)
```

**Purpose**: Poll for a specific UBX message

**Parameters**:
- `msg_class`: UBX message class
- `id`: UBX message ID

**Behavior**:
1. Encodes UBX-POLL message
2. Sends to receiver
3. Receiver responds with requested data

**Usage**:
```c
// Poll for navigation solution
ublox_poll(UBX_CLASS_NAV, UBX_NAV_SOL);
```

### Configuration Functions

#### GNSS Constellation Configuration

```c
void ublox_cfg_append_enable_gps(unsigned char *buffer, int *ind, bool enable);
void ublox_cfg_append_enable_gal(unsigned char *buffer, int *ind, bool enable);
void ublox_cfg_append_enable_bds(unsigned char *buffer, int *ind, bool enable);
void ublox_cfg_append_enable_glo(unsigned char *buffer, int *ind, bool enable);
void ublox_cfg_append_enable_qzss(unsigned char *buffer, int *ind, bool enable);
```

**Purpose**: Enable/disable GNSS constellations

**Parameters**:
- `buffer`: Configuration message buffer
- `ind`: Current position in buffer
- `enable`: Enable (true) or disable (false)

**Usage**:
```c
unsigned char cfg_msg[256];
int ind = 0;

// Enable GPS and Galileo
ublox_cfg_append_enable_gps(cfg_msg, &ind, true);
ublox_cfg_append_enable_gal(cfg_msg, &ind, true);

// Send configuration
ublox_encode_send(UBX_CLASS_CFG, UBX_CFG_GNSS, cfg_msg, ind);
```

#### Message Rate Configuration

```c
void ublox_cfg_append_set_rate(unsigned char *buffer, int *ind, uint16_t meas_rate, uint16_t nav_rate, uint16_t timeref);
```

**Purpose**: Set GNSS measurement and navigation rates

**Parameters**:
- `buffer`: Configuration message buffer
- `ind`: Current position in buffer
- `meas_rate`: Measurement rate (ms)
- `nav_rate`: Navigation rate (cycles)
- `timeref`: Time reference

**Usage**:
```c
unsigned char cfg_msg[256];
int ind = 0;

// Set 10Hz measurement rate, 1 navigation solution per cycle
ublox_cfg_append_set_rate(cfg_msg, &ind, 100, 1, 0);

// Send configuration
ublox_encode_send(UBX_CLASS_CFG, UBX_CFG_RATE, cfg_msg, ind);
```

### Message Processing

#### UBX Message Decoding

The module decodes various UBX message types:

**Navigation Messages**:
- **NAV-SOL**: Navigation solution (position, velocity, time)
- **RELPOSNED**: Relative position (for RTK)
- **SVIN**: Survey-in status

**Receiver Messages**:
- **RAWX**: Raw measurements (pseudorange, carrier phase)
- **NAV-SAT**: Satellite status and signal information

**Configuration Messages**:
- **ACK**: Configuration acknowledgment
- **NAK**: Configuration negative acknowledgment
- **CFG-GNSS**: GNSS configuration

**Monitoring Messages**:
- **MON-VER**: Module version information

#### Message Flow

1. **UART Reception**: Bytes received via UART interrupt
2. **Buffer Storage**: Bytes stored in circular buffer
3. **Thread Notification**: Process thread signaled
4. **Message Parsing**: Thread parses NMEA or UBX messages
5. **Checksum Verification**: UBX messages verified
6. **Message Routing**: Messages routed to appropriate handler
7. **Callback Invocation**: Registered callbacks called

### UBX Protocol Details

#### Message Format

```
| Sync1 | Sync2 | Class | ID | Length | Payload | Checksum A | Checksum B |
| 0xB5  | 0x62  | 1 byte | 1 byte | 2 bytes | Variable | 1 byte | 1 byte |
```

**Fields**:
- **Sync**: 0xB5, 0x62 (UBX sync characters)
- **Class**: Message class (e.g., 0x01 for NAV)
- **ID**: Message ID (e.g., 0x06 for NAV-SOL)
- **Length**: Payload length (2 bytes, little-endian)
- **Payload**: Message data
- **Checksum**: CRC8 checksum (A and B)

#### Checksum Calculation

```c
ubx_ck_a = 0;
ubx_ck_b = 0;

for (int i = 2; i < len; i++) {
    ubx_ck_a += msg[i];
    ubx_ck_b += ubx_ck_a;
}
```

### NMEA Protocol Support

The module also supports NMEA sentences:
- **Format**: ASCII text with checksum
- **Termination**: `\r\n`
- **Checksum**: Two hex digits at end

**Example**: `$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47`

### RTCM3 Integration

#### RTCM3 Correction Data

- **Purpose**: Differential GNSS corrections
- **Source**: External RTCM3 provider
- **Processing**: Handled by `rtcm3_simple` module
- **Output**: Sent to UBLOX receiver via `ublox_send()`

**Flow**:
1. RTCM3 data received (e.g., from radio)
2. Data forwarded to `ublox_send()`
3. Data processed by RTCM3 decoder
4. Corrections applied by UBLOX receiver

### Thread Functions

#### `process_thread()`
```c
static THD_FUNCTION(process_thread, arg)
```

**Purpose**: Main processing thread for UBLOX communication

**Behavior**:
1. Gets own thread pointer
2. Enters infinite loop
3. Waits for data event
4. Processes all available bytes
5. Parses NMEA or UBX messages
6. Routes to appropriate handlers

**Event Handling**:
- **Trigger**: UART data reception
- **Mechanism**: Event signaling
- **Processing**: All available data processed

**Message Parsing**:
- **NMEA**: Line-based parsing
- **UBX**: State machine with checksum verification
- **RTCM3**: Forwarded to RTCM3 decoder

### UART Callbacks

#### `rxchar()`
```c
static void rxchar(UARTDriver *uartp, uint16_t c)
```

**Purpose**: UART receive character callback (ISR)

**Behavior**:
1. Stores character in circular buffer
2. Wraps around if buffer full
3. Signals process thread
4. Returns to UART driver

**Thread Safety**:
- **ISR Context**: Uses ISR-safe functions
- **Signaling**: `chEvtSignalI()` for thread notification
- **Buffer Access**: No mutex needed (single writer)

### Type Getters and Setters

#### Getter Functions

```c
static uint8_t ubx_get_U1(uint8_t *msg, int *ind);
static int8_t ubx_get_I1(uint8_t *msg, int *ind);
static uint16_t ubx_get_U2(uint8_t *msg, int *ind);
static int16_t ubx_get_I2(uint8_t *msg, int *ind);
static uint32_t ubx_get_U4(uint8_t *msg, int *ind);
static int32_t ubx_get_I4(uint8_t *msg, int *ind);
static float ubx_get_R4(uint8_t *msg, int *ind);
static double ubx_get_R8(uint8_t *msg, int *ind);
```

**Purpose**: Extract data from UBX messages

**Parameters**:
- `msg`: Message buffer
- `ind`: Current position (updated)

**Behavior**:
- Reads data at current position
- Updates position pointer
- Handles little-endian format

#### Setter Functions

```c
static void ubx_put_U1(uint8_t *msg, int *ind, uint8_t data);
static void ubx_put_I1(uint8_t *msg, int *ind, int8_t data);
static void ubx_put_U2(uint8_t *msg, int *ind, uint16_t data);
static void ubx_put_I2(uint8_t *msg, int *ind, int16_t data);
static void ubx_put_U4(uint8_t *msg, int *ind, uint32_t data);
static void ubx_put_I4(uint8_t *msg, int *ind, int32_t data);
static void ubx_put_R4(uint8_t *msg, int *ind, float data);
static void ubx_put_R8(uint8_t *msg, int *ind, double data);
```

**Purpose**: Write data to UBX messages

**Parameters**:
- `msg`: Message buffer
- `ind`: Current position (updated)
- `data`: Data to write

**Behavior**:
- Writes data at current position
- Updates position pointer
- Handles little-endian format

## Communication Flow

### Data Reception Path

1. **UART Hardware** → Receives bytes
2. **rxchar ISR** → Stores in circular buffer
3. **rxchar ISR** → Signals process thread
4. **Process Thread** → Waits for event
5. **Process Thread** → Reads from buffer
6. **Process Thread** → Parses NMEA or UBX
7. **Process Thread** → Routes to handler
8. **Handler** → Calls registered callback

### Data Transmission Path

1. **Application** → Calls configuration function
2. **Function** → Builds UBX message
3. **Function** → Calls `ubx_encode_send()`
4. **ubx_encode_send** → Adds headers and checksum
5. **UART Driver** → Transmits message

### RTCM3 Data Path

1. **External Source** → Provides RTCM3 data
2. **Application** → Calls `ublox_send()`
3. **ublox_send** → Calls `rtcm_rx()`
4. **rtcm_rx** → Processes RTCM3 data
5. **RTCM3 Decoder** → Extracts corrections
6. **UBLOX Receiver** → Applies corrections

## Thread Interaction

### Thread Communication

**UART ISR → Process Thread**:
- **Mechanism**: Event signaling (`chEvtSignalI`)
- **Trigger**: Character received
- **Purpose**: Notify process thread to process data

**Process Thread → Application**:
- **Mechanism**: Callback functions
- **Trigger**: Message received and parsed
- **Purpose**: Deliver parsed data to application

### Thread Safety

**UART ISR**:
- **Context**: Interrupt service routine
- **Access**: Circular buffer (single writer)
- **Synchronization**: ISR-safe functions

**Process Thread**:
- **Context**: Normal thread
- **Access**: Circular buffer (single reader)
- **Synchronization**: No mutex needed

**Application**:
- **Context**: Any thread
- **Access**: Callback functions
- **Synchronization**: Callback must be thread-safe

## Configuration Options

### GNSS Constellations

The module supports configuration of multiple GNSS constellations:

**GPS**:
- **Enable**: `ublox_cfg_append_enable_gps()`
- **Default**: Enabled

**GLONASS**:
- **Enable**: `ublox_cfg_append_enable_glo()`
- **Default**: Enabled

**Galileo**:
- **Enable**: `ublox_cfg_append_enable_gal()`
- **Default**: Enabled

**BeiDou**:
- **Enable**: `ublox_cfg_append_enable_bds()`
- **Default**: Enabled

**QZSS**:
- **Enable**: `ublox_cfg_append_enable_qzss()`
- **Default**: Enabled

### Message Rates

**Measurement Rate**:
- **Range**: 100ms to 10000ms
- **Typical**: 100ms (10Hz)
- **Function**: `ublox_cfg_append_set_rate()`

**Navigation Rate**:
- **Range**: 1 to 255 cycles
- **Typical**: 1 cycle
- **Function**: `ublox_cfg_append_set_rate()`

### Baud Rate

**Default**: 38400 (F9P) or 9600 (other)

**Operational**: 115200

**Configuration**: Automatic switching during initialization

## Usage Examples

### Basic Initialization
```c
// Initialize UBLOX module
ublox_init();

// Set callback for navigation solutions
void my_nav_sol_callback(ubx_nav_sol *sol) {
    // Handle navigation solution
    pos_set_xya(sol->ecefX, sol->ecefY, sol->ecefZ);
}

ublox_set_rx_callback_nav_sol(my_nav_sol_callback);
```

### Configuration
```c
// Build configuration message
unsigned char cfg_msg[256];
int ind = 0;

// Enable GPS and Galileo
ublox_cfg_append_enable_gps(cfg_msg, &ind, true);
ublox_cfg_append_enable_gal(cfg_msg, &ind, true);

// Set 10Hz measurement rate
ublox_cfg_append_set_rate(cfg_msg, &ind, 100, 1, 0);

// Send configuration
ublox_encode_send(UBX_CLASS_CFG, UBX_CFG_GNSS, cfg_msg, ind);
```

### Polling for Data
```c
// Poll for navigation solution
ublox_poll(UBX_CLASS_NAV, UBX_NAV_SOL);

// Poll for satellite status
ublox_poll(UBX_CLASS_NAV, UBX_NAV_SAT);
```

### RTCM3 Correction Data
```c
// Receive RTCM3 data from radio
unsigned char rtcm_data[256];
int rtcm_len = receive_from_radio(rtcm_data);

// Send to UBLOX
ublox_send(rtcm_data, rtcm_len);
```

### Terminal Commands
```c
// Poll for navigation solution via terminal
terminal_cmd("ubx poll nav-sol");

// Poll for satellite status
terminal_cmd("ubx poll nav-sat");

// Poll for receiver version
terminal_cmd("ubx poll mon-ver");
```

## Performance Considerations

### Thread Performance

**Thread Loop**: Event-driven (no fixed interval)

**CPU Usage**:
- Low when no data
- Moderate during message processing
- Efficient parsing algorithms

**Memory Usage**:
- **Stack**: 4096 bytes
- **Buffers**: ~4.5KB total
- **Variables**: ~1KB
- **Total**: ~5.5KB

### Throughput

**UART**: 115200 baud
- **Max**: ~11.5 KB/s
- **Typical**: 1-2 KB/s (with GNSS data)
- **RTCM3**: Additional bandwidth for corrections

**Message Rates**:
- **Navigation**: 1-10Hz
- **Raw Measurements**: 1-10Hz
- **Satellite Status**: 1-10Hz

### Latency

**UART Reception**: < 1ms
- **Thread Wakeup**: < 1ms
- **Message Processing**: < 10ms
- **Callback Execution**: Depends on callback

## Compatibility

### Hardware
- **UBLOX Receivers**: F9P, M8N, M8T, etc.
- **UART Interface**: Standard asynchronous UART
- **Baud Rates**: 9600, 38400, 115200
- **GPIO**: Reset pin for receiver control

### Software
- **ChibiOS**: Required for threading and UART driver
- **UART Driver**: STM32 UART driver
- **RTCM3 Module**: `rtcm3_simple` for correction data
- **Position Module**: `pos.c` for position updates

## Future Improvements

### Potential Enhancements
1. **Dynamic Baud Rate**: Adjust based on data rate
2. **Message Filtering**: Hardware filtering for specific messages
3. **Error Recovery**: Automatic recovery from errors
4. **Diagnostics**: Comprehensive error reporting
5. **Logging**: Detailed logging of GNSS data
6. **Calibration**: Automatic calibration procedures

### Current Limitations
1. **Fixed Buffers**: Cannot adapt to different data rates
2. **Simple Error Handling**: No sophisticated error recovery
3. **No Priority**: All messages treated equally
4. **No Flow Control**: Assumes UART can handle traffic
5. **Limited Diagnostics**: Minimal error reporting

## Troubleshooting

### Common Issues

**Issue**: No GNSS data received
- **Possible Causes**:
  - Receiver not powered
  - Wrong baud rate
  - Antenna not connected
  - No satellite view
- **Solutions**:
  - Check power and connections
  - Verify baud rate settings
  - Check antenna connection
  - Verify satellite visibility

**Issue**: Configuration not accepted
- **Possible Causes**:
  - Wrong message format
  - Receiver in wrong state
  - Checksum error
- **Solutions**:
  - Verify message format
  - Check receiver state
  - Verify checksum calculation

**Issue**: High position error
- **Possible Causes**:
  - No RTCM3 corrections
  - Poor satellite geometry
  - Multipath interference
- **Solutions**:
  - Enable RTCM3 corrections
  - Improve antenna placement
  - Check for interference sources

**Issue**: UART errors
- **Possible Causes**:
  - Wrong baud rate
  - Electrical issues
  - Noise on UART lines
- **Solutions**:
  - Verify baud rate
  - Check wiring
  - Add noise filtering

### Debugging Tips

1. **Check Receiver Status**:
   ```c
   // Poll for receiver version
   ublox_poll(UBX_CLASS_MON, UBX_MON_VER);
   ```

2. **Monitor Satellite Status**:
   ```c
   // Set callback for satellite status
   ublox_set_rx_callback_nav_sat(my_sat_callback);
   ```

3. **Check Configuration**:
   ```c
   // Poll for current configuration
   ublox_poll(UBX_CLASS_CFG, UBX_CFG_GNSS);
   ```

4. **Enable Debug Output**:
   ```c
   // Enable printing of specific messages
   m_print_next_nav_sol = true;
   m_print_next_nav_sat = true;
   ```

5. **Check UART Communication**:
   ```c
   // Verify UART is receiving data
   // Check circular buffer fill level
   ```

## References

### Related Files
- `ublox.h`: Header file with function declarations and types
- `rtcm3_simple.c`: RTCM3 correction data processing
- `pos.c`: Position estimation and navigation
- `commands.c`: Command processing and terminal interface

### External Dependencies
- **ChibiOS**: Threading, UART driver, synchronization
- **UBLOX Protocol**: UBX binary protocol specification
- **NMEA Protocol**: NMEA 0183 specification
- **RTCM3 Protocol**: RTCM3 correction data specification

### UBLOX Documentation
- **UBX Protocol**: UBX-13003221
- **Receiver Specs**: Device-specific datasheets
- **Configuration**: UBX-18010854 (for F9P)

## Conclusion

The UBLOX GNSS module provides comprehensive support for UBLOX GNSS receivers, enabling precise position estimation and navigation. Its thread-based architecture ensures reliable message processing while maintaining low CPU usage. The module supports multiple GNSS constellations, RTCM3 corrections, and flexible configuration options.

The module's design ensures efficient data processing with minimal overhead, making it suitable for real-time navigation applications. The callback-based architecture allows for easy integration with the rest of the system, while the configuration functions provide fine-grained control over receiver behavior.
