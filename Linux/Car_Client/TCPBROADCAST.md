# TcpBroadcast Documentation

## Overview

**TcpBroadcast** is a TCP server class that broadcasts data to multiple connected clients. It's designed for efficient one-to-many communication, commonly used for streaming data like GPS corrections (RTCM), UBX data, or log messages.

## Key Features

### 1. Multi-Client Support
- Accepts multiple TCP connections
- Maintains list of connected clients
- Automatically removes disconnected clients

### 2. Data Broadcasting
- Broadcasts data to all connected clients
- Efficient data distribution
- Non-blocking I/O operations

### 3. Logging Support
- Log data to file simultaneously with broadcasting
- Configurable log file location
- Automatic log file management

### 4. Error Handling
- Error reporting via `getLastError()`
- Graceful server shutdown
- Automatic cleanup of disconnected clients

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
TcpBroadcast(QObject *parent = 0)
~TcpBroadcast()
```

#### Server Management
```cpp
bool startTcpServer(int port)
QString getLastError()
void stopServer()
```

#### Data Broadcasting
```cpp
void broadcastData(QByteArray data)
```

#### Logging
```cpp
bool logToFile(QString file)
void logStop()
```

### Signals

```cpp
void dataReceived(QByteArray &data)
```

### Slots

```cpp
void newTcpConnection()
void readyRead()
```

## Implementation Details

### Server Architecture

The TcpBroadcast uses Qt's `QTcpServer` for server functionality:
- Listens on specified port
- Accepts incoming connections
- Maintains list of connected sockets
- Automatically cleans up disconnected clients

### Data Flow

1. **Broadcast**: `broadcastData()` sends data to all connected clients
2. **Receive**: `readyRead()` slot receives data from clients
3. **Logging**: Data is optionally written to log file
4. **Cleanup**: Disconnected clients are removed from the list

### Client Management

The class maintains a list of connected clients:
- `QList<QTcpSocket*>` stores all active connections
- Disconnected clients are removed during broadcast
- New connections are added via `newTcpConnection()` slot

## Usage Examples

### Starting TCP Server

```cpp
TcpBroadcast *broadcaster = new TcpBroadcast(this);

// Start server on port 8200
if (!broadcaster->startTcpServer(8200)) {
    qDebug() << "Failed to start server:" << broadcaster->getLastError();
}
```

### Broadcasting Data

```cpp
QByteArray data = "Hello, clients!";
broadcaster->broadcastData(data);
```

### Logging to File

```cpp
// Start logging to file
if (!broadcaster->logToFile("/var/log/broadcast.log")) {
    qDebug() << "Failed to start logging";
}

// Stop logging
broadcaster->logStop();
```

### Stopping Server

```cpp
// Stop server and clean up
broadcaster->stopServer();
```

### Receiving Client Data

```cpp
connect(broadcaster, &TcpBroadcast::dataReceived, this, &MyClass::handleClientData);

void MyClass::handleClientData(QByteArray &data) {
    qDebug() << "Received from client:" << data;
}
```

## Error Handling

### Error Codes

The `getLastError()` method returns error strings from Qt's `QTcpServer`:
- "Connection refused"
- "Address already in use"
- "Permission denied"
- "Network unreachable"
- And other socket errors

### Handling Errors

```cpp
if (!broadcaster->startTcpServer(8200)) {
    QString error = broadcaster->getLastError();
    qDebug() << "Error starting server:" << error;
    // Handle error appropriately
}
```

## Best Practices

1. **Check return values**: Verify server starts successfully
2. **Handle errors**: Check error messages when operations fail
3. **Cleanup resources**: Call `stopServer()` before destruction
4. **Manage connections**: Monitor connected clients
5. **Use appropriate ports**: Avoid well-known port numbers
6. **Log appropriately**: Enable logging for debugging

## Performance Considerations

- **Buffer sizes**: Qt handles buffer management internally
- **Client count**: No hard limit on number of clients
- **Data size**: Keep individual packets under 1400 bytes for reliability
- **Broadcast efficiency**: Single write operation to all clients
- **Memory usage**: Clients are stored in a list with minimal overhead

## Threading Model

The TcpBroadcast uses Qt's signal-slot mechanism:
- All methods are thread-safe when called from the main thread
- Network I/O is handled asynchronously
- Signal emission is thread-safe
- No explicit threading required

## Comparison with QTCPSERVER

The `TcpBroadcast` class vs. raw `QTcpServer`:

**TcpBroadcast**:
- Simplified API for broadcasting
- Built-in client management
- Automatic cleanup
- Logging support
- Signal for received data

**QTcpServer**:
- More low-level control
- Manual client management
- No built-in broadcasting
- No logging support
- More code required

## Future Enhancements

Potential improvements:
- Configurable buffer sizes
- Connection limits
- Client authentication
- Encryption support
- Compression for efficient data transfer
- Statistics tracking (bytes sent, clients connected, etc.)
- Heartbeat mechanism for client monitoring
