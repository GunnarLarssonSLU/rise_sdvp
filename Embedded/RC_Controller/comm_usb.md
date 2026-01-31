# USB Communication Module Documentation

## Overview

This document provides comprehensive documentation for the USB communication module in the RC_Controller firmware, specifically focusing on the `comm_usb.c` file.

## File Structure

### Location
- **File**: `comm_usb.c`
- **Purpose**: USB serial communication between embedded controller and external devices (typically Raspberry Pi)
- **Architecture**: Dual-thread design with circular buffer for reliable data transfer

## Key Features

1. **Dual-Thread Architecture**
   - Separate threads for reading and processing data
   - Efficient handling of USB communication without blocking

2. **Circular Buffer**
   - 2048-byte buffer for incoming USB data
   - Prevents data loss during high-throughput scenarios

3. **Flow Control**
   - Rate limiting to prevent USB buffer overflows
   - Configurable minimum send interval

4. **Mode-Specific Routing**
   - Different behavior for MOTE vs non-MOTE modes
   - Supports multiple radio types (CC2520, CC1120)

5. **Thread Safety**
   - Mutex-protected send operations
   - Safe for multi-threaded access

## Detailed Documentation

### Configuration Settings

#### `PACKET_HANDLER`
- **Type**: Macro
- **Value**: 0
- **Purpose**: Identifies USB packets in the packet processing system
- **Usage**: Used by the packet framework to route USB-specific packets

#### `SERIAL_RX_BUFFER_SIZE`
- **Type**: Macro
- **Value**: 2048 bytes
- **Purpose**: Size of the circular buffer for incoming USB data
- **Considerations**: 
  - Large enough to handle bursty data
  - Small enough to fit in limited RAM
  - Must be a power of 2 for efficient circular buffer operations

#### `MS2ST(ms)`
- **Type**: Macro
- **Purpose**: Converts milliseconds to ChibiOS system ticks
- **Formula**: `(systime_t)((ms) * CH_CFG_ST_FREQUENCY / 1000)`
- **Usage**: Used for timing operations in flow control

### Private Variables

#### `serial_rx_buffer`
- **Type**: `uint8_t[SERIAL_RX_BUFFER_SIZE]`
- **Purpose**: Circular buffer storing incoming USB data
- **Access**: Shared between read and process threads
- **Synchronization**: Read position by process thread, write position by read thread

#### `serial_rx_read_pos` and `serial_rx_write_pos`
- **Type**: `volatile int`
- **Purpose**: Track positions in the circular buffer
- **Synchronization**: 
  - `read_pos`: Modified only by process thread
  - `write_pos`: Modified only by read thread
  - No mutex needed due to single-writer/single-reader pattern

#### Thread Working Areas
- **`serial_read_thread_wa`**: 512 bytes for read thread
- **`serial_process_thread_wa`**: 4096 bytes for process thread
- **Purpose**: Stack space for thread execution

#### `send_mutex`
- **Type**: `mutex_t`
- **Purpose**: Protects USB send operations
- **Usage**: Locked during `packet_send_packet()` calls
- **Importance**: Prevents concurrent USB transmissions

#### `process_tp`
- **Type**: `thread_t*`
- **Purpose**: Pointer to process thread for signaling
- **Usage**: Read thread signals process thread when data arrives

### Public Functions

#### `comm_usb_init()`
```c
void comm_usb_init(void)
```

**Purpose**: Initializes the USB communication module

**Steps**:
1. Initializes USB serial interface via `comm_usb_serial_init()`
2. Sets up packet processing with `packet_init()`
3. Initializes send mutex
4. Creates read and process threads

**Requirements**:
- Must be called before any USB communication
- Should be called after USB hardware initialization
- Thread-safe (can be called from initialization context)

**Thread Creation**:
- Both threads run at `NORMALPRIO`
- Read thread: 512-byte stack
- Process thread: 4096-byte stack (larger due to packet processing)

#### `comm_usb_send_packet()`
```c
void comm_usb_send_packet(unsigned char *data, unsigned int len)
```

**Purpose**: Thread-safe function to send data over USB

**Parameters**:
- `data`: Pointer to data buffer
- `len`: Length of data to send

**Behavior**:
1. Locks send mutex to prevent concurrent transmissions
2. Calls `packet_send_packet()` with USB handler
3. Unlocks mutex when complete

**Thread Safety**:
- Safe to call from any thread context
- Protects against concurrent USB transmissions
- Minimal blocking time (only during actual transmission)

**Usage Example**:
```c
unsigned char buffer[64];
buffer[0] = 0xAA;
buffer[1] = 0x55;
comm_usb_send_packet(buffer, 2);
```

### Thread Functions

#### `serial_read_thread()`
```c
static THD_FUNCTION(serial_read_thread, arg)
```

**Purpose**: Continuously reads bytes from USB and stores them in circular buffer

**Behavior**:
1. Reads one byte at a time from USB (`chSequentialStreamRead`)
2. Stores byte in circular buffer at `write_pos`
3. Wraps around when reaching buffer end
4. Signals process thread when data arrives
5. Runs indefinitely (never exits)

**Thread Name**: "USB-Serial read"

**Priority**: `NORMALPRIO`

**Key Features**:
- Minimal blocking (reads one byte at a time)
- Efficient circular buffer management
- Low-latency signaling to process thread

#### `serial_process_thread()`
```c
static THD_FUNCTION(serial_process_thread, arg)
```

**Purpose**: Processes bytes from circular buffer and reassembles packets

**Behavior**:
1. Waits for signal from read thread
2. Processes all available bytes in buffer
3. Feeds each byte to packet processor (`packet_process_byte`)
4. Wraps around when reaching buffer end
5. Runs indefinitely (never exits)

**Thread Name**: "USB-Serial process"

**Priority**: `NORMALPRIO`

**Key Features**:
- Event-driven processing (waits for data)
- Processes all available data when signaled
- Efficient packet reassembly

### Packet Processing

#### `process_packet()`
```c
static void process_packet(unsigned char *data, unsigned int len)
```

**Purpose**: Routes packets based on mode and packet type

**Behavior**:

**MOTE Mode** (`MAIN_MODE_IS_MOTE`):
1. Extracts packet ID and destination
2. **Local processing** (ID_MOTE with specific packet IDs):
   - Routes to `commands_process_packet()`
3. **Radio forwarding** (other packets):
   - Hybrid mode: CC1120 for RTCM, CC2520 for others
   - 400MHz mode: CC1120 radio
   - Default: CC2520 radio
4. **UBLOX forwarding** (if enabled):
   - Forwards RTCM data to UBLOX module

**Non-MOTE Mode**:
- All packets processed locally by `commands_process_packet()`

**Packet ID Ranges**:
- Local processing: ID_MOTE with packet_id < 50 or >= 200
- Radio forwarding: All other packets

### Flow Control

#### `send_packet()`
```c
static void send_packet(unsigned char *buffer, unsigned int len)
```

**Purpose**: Sends packets via USB with rate limiting

**Flow Control Algorithm**:
1. Tracks time of last transmission (`last_send_time`)
2. Gets current time (`chVTGetSystemTimeX()`)
3. Checks if minimum interval has passed
4. **If interval passed**:
   - Sends packet via `chSequentialStreamWrite()`
   - Updates `last_send_time`
5. **If interval not passed**:
   - Silently drops packet
   - Prevents USB buffer overflow

**Rate Limiting**:
- Configurable via `USB_MIN_SEND_INTERVAL_MS`
- Prevents overwhelming Raspberry Pi USB connection
- Simple but effective approach

**Trade-offs**:
- Pros: Prevents USB stalls, simple implementation
- Cons: May drop packets during high load

## Communication Flow

### Data Reception Path

1. **USB Hardware** → Reads bytes from USB
2. **Read Thread** → Stores bytes in circular buffer
3. **Read Thread** → Signals process thread
4. **Process Thread** → Reads bytes from buffer
5. **Process Thread** → Feeds bytes to packet processor
6. **Packet Processor** → Reassembles complete packets
7. **process_packet()** → Routes packet to appropriate handler

### Data Transmission Path

1. **Application** → Calls `comm_usb_send_packet()`
2. **Send Mutex** → Locks to prevent concurrent access
3. **packet_send_packet()** → Prepares packet for transmission
4. **send_packet()** → Applies flow control
5. **USB Hardware** → Transmits packet over USB
6. **Send Mutex** → Unlocks when complete

## Thread Interaction

### Thread Communication

**Read Thread → Process Thread**:
- **Mechanism**: Event signaling (`chEvtSignal`)
- **Trigger**: When data is written to buffer
- **Purpose**: Notify process thread to process data

**Process Thread → Read Thread**:
- **Mechanism**: Circular buffer positions
- **Read Position**: Process thread reads from buffer
- **Write Position**: Read thread writes to buffer
- **Synchronization**: No explicit synchronization needed (single writer/single reader)

### Thread Priorities

Both threads run at `NORMALPRIO`:
- **Read Thread**: Higher priority would ensure no data loss
- **Process Thread**: Higher priority would reduce packet processing latency
- **Current**: Balanced approach, suitable for typical USB data rates

## Error Handling

### USB Errors
- **Detection**: USB driver handles hardware errors
- **Recovery**: Automatic retry on next transmission
- **Logging**: Minimal (errors may be silently dropped)

### Buffer Overflow
- **Detection**: Circular buffer wrap-around
- **Recovery**: Oldest data overwritten by new data
- **Prevention**: 2048-byte buffer should handle typical loads

### Packet Loss
- **Flow Control**: Rate limiting may drop packets
- **Reason**: Prevent USB buffer overflow
- **Impact**: Minimal if data rate exceeds USB capacity

## Configuration Options

### USB_MIN_SEND_INTERVAL_MS
- **Purpose**: Minimum time between USB transmissions
- **Default**: Not specified in this file (defined elsewhere)
- **Effect**: Controls USB bandwidth usage
- **Trade-off**: Higher values = more stable but lower throughput

### SERIAL_RX_BUFFER_SIZE
- **Purpose**: Size of receive buffer
- **Default**: 2048 bytes
- **Effect**: Controls how much data can be buffered
- **Trade-off**: Larger = more data buffered but higher RAM usage

## Usage Examples

### Sending Data
```c
// Simple packet send
unsigned char packet[10];
packet[0] = DEST_ID;
packet[1] = CMD_CODE;
// ... fill packet ...
comm_usb_send_packet(packet, sizeof(packet));

// Thread-safe from any context
chSysLock();
comm_usb_send_packet(data, len);
chSysUnlock();
```

### Receiving Data
```c
// Data reception is automatic
// Packets are processed by packet framework
// Application receives processed commands via commands_process_packet()
```

### Initialization
```c
// Standard initialization sequence
usb_hw_init();  // Initialize USB hardware
comm_usb_init();  // Initialize USB communication
commands_init();  // Initialize command processing
// ... other initialization ...
```

## Performance Considerations

### Throughput
- **Limited by**: USB bandwidth (~12 Mbps for USB 2.0)
- **Actual**: Much lower due to protocol overhead
- **Flow Control**: Further limits to prevent overflow

### Latency
- **Read Thread**: Very low (reads one byte at a time)
- **Process Thread**: Depends on signaling latency
- **Total**: Typically < 1ms for complete packet processing

### Memory Usage
- **Buffer**: 2048 bytes for receive buffer
- **Threads**: 4608 bytes total stack space
- **Mutex**: Minimal overhead
- **Total**: ~6KB for USB communication module

## Compatibility

### Hardware
- **USB Interface**: Standard USB 2.0
- **Protocol**: Serial over USB (CDC-ACM or similar)
- **Host**: Raspberry Pi (tested)
- **Other Hosts**: Should work with any USB host

### Software
- **ChibiOS**: Required for thread and synchronization primitives
- **Packet Framework**: Required for packet processing
- **USB Serial Driver**: Required for low-level USB access

## Future Improvements

### Potential Enhancements
1. **Dynamic Rate Limiting**: Adjust based on actual USB performance
2. **Larger Buffers**: For higher throughput applications
3. **Error Reporting**: Notify application of dropped packets
4. **Flow Control**: Implement proper USB flow control (XON/XOFF)
5. **Priority Boosting**: Increase thread priority during high load

### Current Limitations
1. **Silent Packet Dropping**: No notification when packets are dropped
2. **Fixed Buffer Size**: Cannot adapt to different data rates
3. **Simple Rate Limiting**: No adaptive algorithm
4. **No Error Recovery**: Assumes USB is always available

## Troubleshooting

### Common Issues

**Issue**: USB communication stops working
- **Possible Causes**:
  - USB buffer overflow
  - Rate limiting too aggressive
  - USB hardware issues
- **Solutions**:
  - Check `USB_MIN_SEND_INTERVAL_MS` value
  - Verify USB cable and connection
  - Check for error messages in logs

**Issue**: Data corruption
- **Possible Causes**:
  - Buffer overflow
  - Race conditions
  - USB errors
- **Solutions**:
  - Increase buffer size
  - Check for proper synchronization
  - Verify USB hardware

**Issue**: High latency
- **Possible Causes**:
  - Process thread blocked
  - High system load
  - USB bandwidth saturation
- **Solutions**:
  - Increase thread priority
  - Reduce data rate
  - Check for system bottlenecks

## References

### Related Files
- `comm_usb.h`: Header file with function declarations
- `comm_usb_serial.c`: Low-level USB serial driver
- `packet.c`: Packet processing framework
- `commands.c`: Command processing module

### External Dependencies
- ChibiOS RTOS: Threads, mutexes, streams
- USB Hardware Driver: Low-level USB access
- Packet Framework: Packet reassembly and routing

## Conclusion

The USB communication module provides a robust and efficient way to communicate between the embedded controller and external devices. Its dual-thread architecture with circular buffering ensures reliable data transfer even under moderate load conditions. The flow control mechanism prevents USB buffer overflows, making it suitable for long-running applications.

The module is well-suited for the RC_Controller application, providing reliable communication with the Raspberry Pi for command and data exchange.
