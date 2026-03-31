# RtcmClient Documentation

## Overview

**RtcmClient** is a class for receiving RTCM (Radio Technical Commission for Maritime Services) correction data. It supports multiple connection methods including NTRIP servers, TCP servers, and serial ports. RTCM data is used for Differential GPS (DGPS) and Real-Time Kinematic (RTK) positioning to improve GPS accuracy.

## Key Features

### 1. Multiple Connection Methods
- **NTRIP (Networked Transport of RTCM via Internet Protocol)**: Connect to NTRIP casters
- **TCP Server**: Connect to RTCM TCP servers
- **Serial Port**: Receive RTCM data from serial devices

### 2. RTCM Protocol Support
- Decodes RTCM3 messages
- Handles various RTCM message types
- Extracts reference station positions
- Supports GPS and GLONASS constellations

### 3. Authentication Support
- NTRIP server authentication
- Username and password support

### 4. Connection Management
- Connection status monitoring
- Automatic reconnection handling
- Graceful disconnection

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
RtcmClient(QObject *parent = 0)
```

#### Connection Methods
```cpp
bool connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80)
bool connectTcp(QString server, int port = 80)
bool connectSerial(QString port, int baudrate = 115200)
```

#### Connection Status
```cpp
bool isTcpConnected()
bool isSerialConnected()
```

#### Disconnection Methods
```cpp
void disconnectTcpNtrip()
void disconnectSerial()
```

#### Configuration
```cpp
void setGpsOnly(bool isGpsOnly)
```

#### Static Methods
```cpp
static QByteArray encodeBasePos(double lat, double lon, double height, double antenna_height = 0)
```

### Signals

```cpp
void rtcmReceived(QByteArray data, int type, bool sync = false)
void refPosReceived(double lat, double lon, double height, double antenna_height)
```

### Slots

```cpp
void tcpInputConnected()
void tcpInputDisconnected()
void tcpInputDataAvailable()
void tcpInputError(QAbstractSocket::SocketError socketError)
void serialDataAvailable()
void serialPortError(QSerialPort::SerialPortError error)
```

## Implementation Details

### RTCM Message Types

The RtcmClient handles various RTCM3 message types:

- **RTCM 1005**: Stationary ARP (Antenna Reference Point) information
- **RTCM 1006**: Reference station position information
- **RTCM 1077**: GPS L1-only RTK observations
- **RTCM 1087**: GLONASS L1-only RTK observations
- **RTCM 1127**: GPS/GLONASS L1-only RTK observations

### NTRIP Protocol

NTRIP is a protocol for distributing RTCM data over the internet:
1. Connect to NTRIP caster
2. Send HTTP GET request with authentication
3. Receive RTCM data stream
4. Process RTCM messages

### Connection Management

The class maintains:
- TCP socket for network connections
- Serial port for direct connections
- Connection state tracking
- Automatic cleanup on disconnection

### RTCM Decoding

Uses the `rtcm3_simple` library for RTCM decoding:
- Callback-based decoding
- Message type identification
- Reference position extraction
- Observation data processing

## Usage Examples

### Connecting to NTRIP Server

```cpp
RtcmClient *rtcmClient = new RtcmClient(this);

// Connect to NTRIP server
if (!rtcmClient->connectNtrip(
    "ntrip.example.com",
    "MOUNTPOINT",
    "username",
    "password",
    2101
)) {
    qDebug() << "Failed to connect to NTRIP server";
}
```

### Connecting to TCP Server

```cpp
// Connect to RTCM TCP server
if (!rtcmClient->connectTcp("192.168.1.100", 8200)) {
    qDebug() << "Failed to connect to TCP server";
}
```

### Connecting to Serial Port

```cpp
// Connect to RTCM serial device
if (!rtcmClient->connectSerial("/dev/ttyUSB1", 9600)) {
    qDebug() << "Failed to connect to serial port";
}
```

### Handling RTCM Data

```cpp
// Connect to RTCM received signal
connect(rtcmClient, &RtcmClient::rtcmReceived, this, &MyClass::handleRtcmData);

void MyClass::handleRtcmData(QByteArray data, int type, bool sync) {
    qDebug() << "Received RTCM message type:" << type;
    qDebug() << "Data length:" << data.length();
    
    // Forward to GPS receiver
    forwardToGpsReceiver(data);
}
```

### Handling Reference Position

```cpp
// Connect to reference position signal
connect(rtcmClient, &RtcmClient::refPosReceived, this, &MyClass::handleRefPos);

void MyClass::handleRefPos(double lat, double lon, double height, double antenna_height) {
    qDebug() << "Reference station position:";
    qDebug() << "  Latitude:" << lat;
    qDebug() << "  Longitude:" << lon;
    qDebug() << "  Height:" << height;
    qDebug() << "  Antenna height:" << antenna_height;
}
```

### Checking Connection Status

```cpp
// Check TCP connection status
if (rtcmClient->isTcpConnected()) {
    qDebug() << "Connected to RTCM server";
}

// Check serial connection status
if (rtcmClient->isSerialConnected()) {
    qDebug() << "Connected to RTCM serial device";
}
```

### Disconnecting

```cpp
// Disconnect from NTRIP/TCP
rtcmClient->disconnectTcpNtrip();

// Disconnect from serial
rtcmClient->disconnectSerial();
```

### Encoding Base Position

```cpp
// Encode base station position for RTCM 1005 message
QByteArray basePos = RtcmClient::encodeBasePos(
    59.3293,  // Latitude
    18.0686,  // Longitude
    10.5,     // Height
    1.2       // Antenna height
);

// Send to GPS receiver
sendToGpsReceiver(basePos);
```

## Error Handling

### Connection Errors

The class handles various connection errors:
- **NTRIP errors**: Authentication failures, server unavailability
- **TCP errors**: Connection refused, timeout, network issues
- **Serial errors**: Port not found, permission denied, device errors

### Error Signals

```cpp
// Handle TCP errors
connect(rtcmClient, &RtcmClient::tcpInputError, this, &MyClass::handleTcpError);

void MyClass::handleTcpError(QAbstractSocket::SocketError error) {
    switch (error) {
        case QAbstractSocket::ConnectionRefusedError:
            qDebug() << "Connection refused";
            break;
        case QAbstractSocket::RemoteHostClosedError:
            qDebug() << "Server closed connection";
            break;
        case QAbstractSocket::HostNotFoundError:
            qDebug() << "Host not found";
            break;
        default:
            qDebug() << "TCP error:" << error;
    }
}
```

### Serial Errors

```cpp
// Handle serial errors
connect(rtcmClient, &RtcmClient::serialPortError, this, &MyClass::handleSerialError);

void MyClass::handleSerialError(QSerialPort::SerialPortError error) {
    qDebug() << "Serial error:" << error;
}
```

## Best Practices

1. **Check connection status**: Verify connections before sending data
2. **Handle errors**: Connect to error signals for proper error handling
3. **Validate data**: Check RTCM message types and data integrity
4. **Connection management**: Disconnect properly when done
5. **Authentication**: Use proper credentials for NTRIP servers
6. **Port configuration**: Use appropriate baud rates for serial devices

## Performance Considerations

- **Buffer sizes**: Qt handles buffer management internally
- **Data rates**: RTCM data rates are typically low (kbps range)
- **Processing**: RTCM decoding is efficient with callback-based approach
- **Memory usage**: Minimal overhead for connection management

## Threading Model

The RtcmClient uses Qt's signal-slot mechanism:
- All methods are thread-safe when called from the main thread
- Network I/O is handled asynchronously
- Signal emission is thread-safe
- No explicit threading required

## Dependencies

- Qt (Core, Network, SerialPort)
- `rtcm3_simple.h` for RTCM decoding
- `datatypes.h` for data structures

## RTCM Message Reference

### RTCM 1005: Stationary ARP Information
Contains antenna reference point information including antenna type and serial number.

### RTCM 1006: Reference Station Position
Contains reference station position (latitude, longitude, height) and antenna height.

### RTCM 1077: GPS L1-only RTK Observations
Contains GPS L1 observations for RTK positioning.

### RTCM 1087: GLONASS L1-only RTK Observations
Contains GLONASS L1 observations for RTK positioning.

### RTCM 1127: GPS/GLONASS L1-only RTK Observations
Contains combined GPS and GLONASS L1 observations.

## Future Enhancements

Potential improvements:
- Support for more RTCM message types
- Multiple simultaneous connections
- Connection retry logic
- Quality of service monitoring
- Statistics tracking (packets received, errors, etc.)
- Support for RTCM2 messages
- Configurable RTCM decoding options
