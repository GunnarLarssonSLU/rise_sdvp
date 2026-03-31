# TcpServerSimple Documentation

## Overview

**TcpServerSimple** is a simple TCP server class that handles single client connections. It provides basic TCP server functionality with optional packet framing support. This class is designed for point-to-point communication, typically used for laptop-to-vehicle communication in the autonomous vehicle control system.

## Key Features

### 1. Single Client Connection
- Accepts one TCP connection at a time
- Manages connection state
- Automatic cleanup on disconnection

### 2. Data Transmission
- Send data to connected client
- Receive data from connected client
- Connection status notifications

### 3. Optional Packet Framing
- Integrates with `Packet` class for framed communication
- Toggle packet mode on/off
- Automatic packet processing

### 4. ROS 2 Integration
- Forward data to ROS 2 channel
- Local socket communication
- Optional ROS 2 messaging

### 5. Error Handling
- Error string reporting
- Connection error notifications
- Graceful shutdown

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
TcpServerSimple(QObject *parent = 0)
```

#### Server Management
```cpp
bool startServer(int port, QHostAddress addr = QHostAddress::Any)
void stopServer()
QString errorString()
```

#### Data Transmission
```cpp
bool sendData(const QByteArray &data)
```

#### Packet Interface
```cpp
Packet *packet()
bool usePacket() const
void setUsePacket(bool usePacket)
```

### Signals

```cpp
void dataRx(const QByteArray &data)
void connectionChanged(bool connected, QString address)
```

### Slots

```cpp
void newTcpConnection()
void tcpInputDisconnected()
void tcpInputDataAvailable()
void tcpInputError(QAbstractSocket::SocketError socketError)
void dataToSend(QByteArray &data)
```

## Implementation Details

### Server Architecture

The TcpServerSimple uses Qt's `QTcpServer` for server functionality:
- Listens on specified port and address
- Accepts single connection
- Manages connection state
- Provides error reporting

### Connection Management

The class maintains a single connection:
- `QTcpSocket *mTcpSocket` stores the current connection
- Null when no client is connected
- Automatically cleaned up on disconnection
- Connection state changes emit signals

### Packet Mode

When packet mode is enabled:
- Data is processed through the `Packet` class
- Packet framing is added to outgoing data
- Incoming data is reassembled into packets
- Complete packets are emitted via signals

### ROS 2 Integration

The class can forward data to a ROS 2 channel:
- Uses `QLocalSocket` for inter-process communication
- Connects to "carclient_ros2_channel"
- Forwards data to ROS 2 components
- Optional feature, can be disabled

## Usage Examples

### Starting TCP Server

```cpp
TcpServerSimple *server = new TcpServerSimple(this);

// Start server on port 8300
if (!server->startServer(8300)) {
    qDebug() << "Failed to start server:" << server->errorString();
}
```

### Sending Data

```cpp
QByteArray data = "Hello, client!";
if (!server->sendData(data)) {
    qDebug() << "No client connected";
}
```

### Using Packet Mode

```cpp
// Enable packet mode
server->setUsePacket(true);

// Get packet interface
Packet *packet = server->packet();
connect(packet, &Packet::packetReceived, this, &MyClass::handlePacket);

// Send packet
QByteArray packetData;
packetData.append("Packet content");
packet->sendPacket(packetData);
```

### Handling Incoming Data

```cpp
connect(server, &TcpServerSimple::dataRx, this, &MyClass::handleData);
connect(server, &TcpServerSimple::connectionChanged, this, &MyClass::handleConnection);

void MyClass::handleData(const QByteArray &data) {
    qDebug() << "Received:" << data;
}

void MyClass::handleConnection(bool connected, QString address) {
    if (connected) {
        qDebug() << "Client connected:" << address;
    } else {
        qDebug() << "Client disconnected";
    }
}
```

### Stopping Server

```cpp
// Stop server and clean up
server->stopServer();
```

## Error Handling

### Error Codes

The `errorString()` method returns error strings from Qt's `QTcpServer`:
- "Connection refused"
- "Address already in use"
- "Permission denied"
- "Network unreachable"
- And other socket errors

### Handling Errors

```cpp
if (!server->startServer(8300)) {
    QString error = server->errorString();
    qDebug() << "Error starting server:" << error;
    // Handle error appropriately
}
```

### Connection Errors

```cpp
connect(server, &TcpServerSimple::tcpInputError, this, &MyClass::handleSocketError);

void MyClass::handleSocketError(QAbstractSocket::SocketError error) {
    switch (error) {
        case QAbstractSocket::RemoteHostClosedError:
            qDebug() << "Client disconnected";
            break;
        case QAbstractSocket::ConnectionRefusedError:
            qDebug() << "Connection refused";
            break;
        default:
            qDebug() << "Socket error:" << error;
    }
}
```

## Best Practices

1. **Check return values**: Verify server starts successfully
2. **Handle errors**: Check error messages when operations fail
3. **Manage connections**: Monitor connection state via signals
4. **Use appropriate ports**: Default port is 8300
5. **Packet mode**: Enable for reliable communication
6. **Cleanup resources**: Call `stopServer()` before destruction

## Performance Considerations

- **Single connection**: Designed for one-to-one communication
- **Buffer sizes**: Qt handles buffer management internally
- **Data size**: Keep individual packets under 1400 bytes for reliability
- **Packet mode**: Adds overhead but improves reliability
- **Memory usage**: Minimal overhead for single connection

## Threading Model

The TcpServerSimple uses Qt's signal-slot mechanism:
- All methods are thread-safe when called from the main thread
- Network I/O is handled asynchronously
- Signal emission is thread-safe
- No explicit threading required

## Comparison with TcpBroadcast

The `TcpServerSimple` class vs. `TcpBroadcast`:

**TcpServerSimple**:
- Single client connection
- Point-to-point communication
- Packet framing support
- ROS 2 integration
- Simpler API

**TcpBroadcast**:
- Multiple client support
- One-to-many broadcasting
- No packet framing
- No ROS 2 integration
- More complex client management

## Future Enhancements

Potential improvements:
- Configurable buffer sizes
- Connection timeouts
- Keep-alive mechanism
- SSL/TLS support for encryption
- Compression for efficient data transfer
- Statistics tracking (bytes sent/received)
- Heartbeat mechanism for connection monitoring
