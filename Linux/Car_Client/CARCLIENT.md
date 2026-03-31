# CarClient Documentation

## Overview

**CarClient** is the main application class that serves as the central communication hub for the vehicle control system. It acts as an interface between:
- The laptop program (RControlStation)
- The controller card (via serial port)
- Optional Arduino devices
- Various network services (RTCM, NMEA, UBX, logging)

The application is designed to run on a Raspberry Pi and handles all data routing, protocol conversion, and system management for autonomous vehicle control.

## Main Components

### 1. Communication Interfaces

#### Serial Communication
- **Controller Card**: Primary interface via `SerialPort`
- **RTCM GPS Data**: Via `QSerialPort` (typically 9600 baud)
- **Arduino**: Via `QSerialPort` (typically 9600 baud)

#### Network Communication
- **TCP Server**: Default port 8300 for laptop communication
- **TCP Client**: For connecting to external services
- **UDP Server**: For broadcast messages
- **RTCM Server**: Broadcasts RTCM correction data (default port 8200)
- **UBX Server**: Broadcasts UBX GPS data (default port 8210)
- **Log Server**: For logging data (default port 8410)

#### NMEA Server
- Connects to NMEA data sources for GPS and navigation data

### 2. Key Features

#### RTCM Handling
- Processes RTCM3 correction data
- Supports both USB and NTRIP sources
- Can inject base station positions
- Broadcasts to multiple clients

#### System Management
- Reboot system (with optional power off)
- Battery monitoring (configurable cell count)
- Time synchronization
- Logging to USB storage

#### Simulation Support
- Car simulation with configurable number of vehicles
- UWB emulation for testing
- Dynamic simulation parameters

#### Protocol Support
- Packet-based communication via `PacketInterface`
- NMEA 0183 protocol
- UBX protocol for u-blox GPS receivers
- RTCM3 protocol for GPS corrections

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
CarClient(QObject *parent = 0)
```
Creates the CarClient instance and initializes all communication interfaces.

#### Connection Methods
```cpp
void connectSerial(QString port, int baudrate = 115200)
void connectSerialRtcm(QString port, int baudrate = 9600)
void connectSerialArduino(QString port, int baudrate = 9600)
void connectNmea(QString server, int port = 2948)
void connectNtrip(QString server, QString stream, QString user = "", QString pass = "", int port = 80)
```

#### Server Methods
```cpp
void startRtcmServer(int port = 8200)
void startUbxServer(int port = 8210)
void startLogServer(int port = 8410)
void startUdpServer(int port = 8300)
bool startTcpServer(int port = 8300, QHostAddress addr = QHostAddress::Any)
```

#### Logging Methods
```cpp
bool enableLogging(QString directory)
void logStop()
```

#### System Methods
```cpp
Q_INVOKABLE void rebootSystem(bool powerOff = false)
Q_INVOKABLE QVariantList getNetworkAddresses()
Q_INVOKABLE int getBatteryCells()
void setBatteryCells(int cells)
```

#### Car Management
```cpp
void addSimulatedCar(int id)
CarSim *getSimulatedCar(int id)
quint8 carId()
void setCarId(quint8 id)
```

### Private Members

#### Communication Objects
- `mPacketInterface`: Handles packet-based communication
- `mSerialPort`: Primary serial port for controller card
- `mSerialPortRtcm`: Serial port for RTCM GPS data
- `mSerialPortArduino`: Serial port for Arduino
- `mTcpSocket`: TCP client socket
- `mTcpServer`: TCP server for incoming connections
- `mUdpSocket`: UDP socket for broadcast messages
- `mRtcmClient`: NTRIP client for RTCM data
- `mUblox`: UBX protocol handler

#### Configuration
- `mSettings`: Structure containing all connection settings
- `mCarId`: Current car ID (0-254, 255 = undefined)
- `mCarIdToSet`: Pending car ID to set
- `mBatteryCells`: Number of battery cells for voltage monitoring
- `mRtcmBaseLat/Lon/Height`: Base station position for RTCM
- `mRtcmSendBase`: Whether to send base position

#### State Tracking
- `mTcpConnected`: TCP connection status
- `mRtklibRunning`: Whether RTKLIB is running
- `mOverrideUwbPos`: Whether to override UWB position
- `mOverrideUwbX/Y`: Override UWB coordinates

#### Timers
- `mReconnectTimer`: For automatic reconnection attempts
- `mLogFlushTimer`: For periodic log flushing

## Signal-Slot Connections

The CarClient uses Qt's signal-slot mechanism extensively for event-driven programming:

### Serial Port Signals
- `readyRead()` → `serialDataAvailable()`
- `readyRead()` (RTCM) → `serialRtcmDataAvailable()`
- `readyRead()` (Arduino) → `serialArduinoDataAvailable()`
- `error()` → `serialPortError()`

### Network Signals
- `readyRead()` (TCP) → `tcpDataAvailable()`
- `connected()` → `tcpConnected()`
- `disconnected()` → `tcpDisconnected()`
- `readyRead()` (UDP) → `readPendingDatagrams()`

### Packet Interface
- `packetDataToSend()` → Handles outgoing packets
- `carPacketRx()` → Processes incoming car packets

### System Signals
- `rebootSystemReceived()` → Handles reboot requests
- `systemTimeReceived()` → Updates system time

## Data Flow

### Incoming Data Path

1. **TCP/UDP**: `tcpDataAvailable()` → `mPacketInterface->sendNmeaRadio()`
2. **Serial**: `serialDataAvailable()` → `processCarData()`
3. **RTCM**: `serialRtcmDataAvailable()` → `rtcmRx()` → RTCM3 decoder
4. **NTRIP**: `rtcmReceived()` → RTCM3 decoder

### Outgoing Data Path

1. **To Controller**: `packetDataToSend()` → serial port
2. **To Network**: `tcpRx()` → `mPacketInterface->sendPacket()` → UDP/TCP
3. **Broadcast**: Various broadcasters send data to subscribed clients

## Configuration

The CarClient can be configured via command-line arguments:

```bash
./carclient \
    --tcpport 8300 \
    --tcprtcmserver 8200 \
    --tcpubxserver 8210 \
    --logusbdir /media/usb/logs \
    --ntrip server:stream:user:pass:80 \
    --batterycells 10 \
    --simulatecars 2:1
```

## Error Handling

The application handles various error conditions:
- Serial port errors (disconnection, permission issues)
- TCP connection failures (with automatic reconnection)
- RTCM decoding errors
- Logging failures
- System reboot failures

## Threading Model

The application uses Qt's event loop and is single-threaded by design. All I/O operations are handled asynchronously via signals and slots.

## Dependencies

- Qt 5/6 (Core, Network, SerialPort)
- RTCM3 library
- UBX protocol library
- PacketInterface for custom packet protocol

## File Structure

The CarClient manages several file operations:
- USB logging to configured directory
- Log file rotation
- Periodic log flushing
- Camera image capture and storage

## Advanced Features

### Car Simulation
The CarClient can simulate multiple vehicles for testing:
- Configurable number of simulated cars
- Each car has its own ID and state
- Can emulate UWB positioning
- Supports dynamic route simulation

### NTRIP Client
For receiving RTCM correction data over the internet:
- Supports authentication
- Handles connection failures
- Reconnects automatically

### Time Synchronization
- Can set system time from GPS
- Handles leap seconds
- Supports Unix time format

## Command Reference

The CarClient processes various commands:
- **Camera commands**: Trigger image capture
- **System commands**: Reboot, shutdown
- **GPS commands**: Configure RTCM, UBX settings
- **Logging commands**: Start/stop logging
- **Simulation commands**: Control simulated vehicles

## Debugging

The application provides extensive debugging via qDebug():
- Connection status messages
- Data flow tracking
- Error conditions
- System state changes

## Best Practices

1. **Connection Management**: Always check connection status before sending data
2. **Error Handling**: Handle serial port and network errors gracefully
3. **Thread Safety**: All Qt operations must be in the main thread
4. **Memory Management**: Use Qt's parent-child ownership system
5. **Configuration**: Validate all configuration parameters

## Future Enhancements

Potential improvements:
- Multi-threaded processing for high data rates
- More robust error recovery
- Additional protocol support
- Improved simulation features
- Remote configuration interface
