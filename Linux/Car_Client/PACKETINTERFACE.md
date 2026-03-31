# PacketInterface Documentation

## Overview

**PacketInterface** is a communication protocol handler that manages packet-based communication between the CarClient and controller cards. It implements a custom binary protocol with CRC checksumming, acknowledgment handling, and retry mechanisms.

## Protocol Specification

### Packet Structure

Each packet follows this format:
```
[START_BYTE][LENGTH][PAYLOAD][CRC16]
```

- **START_BYTE**: 0xAA (1 byte) - Packet start marker
- **LENGTH**: 2 bytes (little-endian) - Length of payload
- **PAYLOAD**: Variable length - Command and data
- **CRC16**: 2 bytes (little-endian) - CRC checksum covering payload

### Command Codes

The protocol supports various command types defined in `datatypes.h`:
- `CMD_PACKET`: Standard packet command
- `CMD_ACK`: Acknowledgement
- `CMD_NACK`: Negative acknowledgement
- `CMD_STATE`: State information
- And many more...

## Key Features

### 1. Packet Assembly/Disassembly
- **processData()**: Processes incoming byte stream, reassembles packets
- **processPacket()**: Handles complete packets, extracts commands and data
- **sendPacket()**: Creates and sends packets with CRC checksum

### 2. Reliable Communication
- **Acknowledgment system**: `sendPacketAck()` with configurable retries
- **Timeout handling**: Automatic retry on failed acknowledgments
- **CRC checking**: Ensures data integrity

### 3. UDP Communication Modes
- **Client mode**: Connect to a specific host and port
- **Server mode**: Listen on a specific port for incoming connections
- **Broadcast mode**: Send to multiple recipients

### 4. High-Level Commands
Convenience methods for common operations:
- Route management (set, get, clear route points)
- Configuration management
- Position and orientation commands
- System commands (reboot, time synchronization)
- UWB anchor management
- Logging commands

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
PacketInterface(QObject *parent = 0)
~PacketInterface()
```

#### Core Packet Methods
```cpp
bool sendPacket(const unsigned char *data, unsigned int len_packet)
bool sendPacket(QByteArray data)
bool sendPacketAck(const unsigned char *data, unsigned int len_packet,
                   int retries, int timeoutMs = 200)
void processData(QByteArray &data)
void processPacket(const unsigned char *data, int len)
```

#### UDP Connection Management
```cpp
void startUdpConnection(QHostAddress ip, int port)
void startUdpConnection2(QHostAddress ip)
void startUdpConnectionServer(int port)
void stopUdpConnection()
bool isUdpConnected()
```

#### Route Management
```cpp
bool setRoutePoints(quint8 id, QList<LocPoint> points, int retries = 10)
bool replaceRoute(quint8 id, QList<LocPoint> points, int retries = 10)
bool removeLastRoutePoint(quint8 id, int retries = 10)
bool clearRoute(quint8 id, int retries = 10)
bool getRoutePart(quint8 id, qint32 first, quint8 num, QList<LocPoint> &points,
                  int &routeLen, int retries = 10)
bool getRoute(quint8 id, QList<LocPoint> &points, int retries = 10)
```

#### Configuration and Control
```cpp
bool setApActive(quint8 id, bool active, int retries = 10)
bool setConfiguration(quint8 id, MAIN_CONFIG &conf, int retries = 10)
bool setPosAck(quint8 id, double x, double y, double angle, int retries = 10)
bool setYawOffsetAck(quint8 id, double angle, int retries = 10)
bool setEnuRef(quint8 id, double *llh, int retries = 10)
```

#### System Commands
```cpp
bool setSystemTime(quint8 id, qint32 sec, qint32 usec, int retries = 10)
bool sendReboot(quint8 id, bool powerOff, int retries = 10)
```

#### Synchronization
```cpp
bool setSyncPoint(quint8 id, int point, int time, int min_time_diff,
                  bool ack = true, int retries = 10)
```

#### UWB Management
```cpp
bool addUwbAnchor(quint8 id, UWB_ANCHOR a, int retries = 10)
bool clearUwbAnchors(quint8 id, int retries = 10)
bool sendMoteUbxBase(int mode, double pos_acc = 10.0, int svin_min_dur = 120,
                     double svin_acc_limit = 10.0, double lat = 0, double lon = 0,
                     double height = 0, int retries = 10)
```

### Signals

The PacketInterface emits signals when various events occur:

```cpp
void dataToSend(QByteArray &data)
void packetReceived(quint8 id, CMD_PACKET cmd, const QByteArray &data)
void printReceived(quint8 id, QString str)
void stateReceived(quint8 id, CAR_STATE state)
void mrStateReceived(quint8 id, MULTIROTOR_STATE state)
void vescFwdReceived(quint8 id, QByteArray data)
void ackReceived(quint8 id, CMD_PACKET cmd, QString msg)
void rtcmUsbReceived(quint8 id, QByteArray data)
void nmeaRadioReceived(quint8 id, QByteArray data)
void configurationReceived(quint8 id, MAIN_CONFIG conf)
void enuRefReceived(quint8 id, double lat, double lon, double height)
void logLineUsbReceived(quint8 id, QString str)
void plotInitReceived(quint8 id, QString xLabel, QString yLabel)
void plotDataReceived(quint8 id, double x, double y)
void plotAddGraphReceived(quint8 id, QString name)
void plotSetGraphReceived(quint8 id, int graph)
void systemTimeReceived(quint8 id, qint32 sec, qint32 usec)
void rebootSystemReceived(quint8 id, bool powerOff)
void routePartReceived(quint8 id, int len, const QList<LocPoint> &route)
void logEthernetReceived(quint8 id, QByteArray data)
```

## Implementation Details

### Packet Processing

The `processData()` method implements a state machine for packet reassembly:

1. **State 0**: Looking for start byte (0xAA)
2. **State 1**: Reading length bytes
3. **State 2**: Reading payload
4. **State 3**: Reading CRC bytes

When a complete packet is received, it's validated with CRC and passed to `processPacket()`.

### CRC Calculation

Uses CRC-16-CCITT algorithm with a precomputed lookup table for efficiency:

```cpp
unsigned short crc16_tab[256]
```

### Acknowledgement System

The `sendPacketAck()` method:
1. Sends a packet
2. Waits for acknowledgment
3. Retries if no ACK received within timeout
4. Gives up after configured number of retries

### Timer System

A 10ms timer drives the acknowledgment retry logic and other time-based operations.

## Usage Examples

### Sending a Simple Packet

```cpp
QByteArray packet;
packet.append(0x01);  // Car ID
packet.append(CMD_TEST);  // Command
packet.append("Hello");  // Data
packetInterface->sendPacket(packet);
```

### Sending with Acknowledgement

```cpp
QByteArray packet;
packet.append(0x01);  // Car ID
packet.append(CMD_CONFIG);  // Command
// ... add configuration data
if (!packetInterface->sendPacketAck(packet.constData(), packet.length(), 5)) {
    qDebug() << "Failed to send configuration";
}
```

### Setting Route Points

```cpp
QList<LocPoint> routePoints;
// ... populate routePoints
if (!packetInterface->setRoutePoints(1, routePoints)) {
    qDebug() << "Failed to set route";
}
```

### Receiving Packets

Connect to signals:

```cpp
connect(packetInterface, &PacketInterface::packetReceived,
        this, &CarClient::carPacketRx);
connect(packetInterface, &PacketInterface::ackReceived,
        this, &CarClient::handleAck);
```

## Error Handling

The PacketInterface handles various error conditions:
- **CRC errors**: Discards corrupted packets
- **Timeout errors**: Retries or fails after configured attempts
- **Connection errors**: Detects UDP disconnections
- **Buffer overflows**: Prevents memory corruption

## Threading Model

The PacketInterface is designed to work in Qt's single-threaded event loop:
- All methods are thread-safe when called from the main thread
- UDP I/O is handled asynchronously via signals and slots
- Timer-based operations use Qt's timer system

## Dependencies

- Qt (Core, Network)
- `datatypes.h` for command codes and data structures
- `utility.h` for helper functions

## Best Practices

1. **Use ACK for critical commands**: Always use `sendPacketAck()` for important operations
2. **Handle errors gracefully**: Check return values and handle failures
3. **Connect to signals**: Use signals for asynchronous event handling
4. **Validate data**: Ensure packets contain valid data before sending
5. **Configure timeouts appropriately**: Balance responsiveness with reliability

## Performance Considerations

- **Packet size**: Keep packets under 1400 bytes for UDP reliability
- **Retry count**: Use minimal retries for non-critical commands
- **Timeout values**: Shorter timeouts for local networks, longer for WAN
- **Buffer sizes**: Default buffers are sized for typical use cases

## Future Enhancements

Potential improvements:
- TCP transport option for more reliable communication
- Packet fragmentation for large data transfers
- Compression for efficient data transmission
- Encryption for secure communication
- Quality of Service (QoS) markings for network prioritization
