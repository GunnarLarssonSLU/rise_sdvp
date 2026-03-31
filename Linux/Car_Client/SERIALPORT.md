# SerialPort Documentation

## Overview

**SerialPort** is a Qt-based serial port communication class that provides thread-safe serial communication with Linux systems. It extends `QThread` to handle serial I/O in a separate thread, preventing UI blocking.

## Key Features

### 1. Thread-Safe Communication
- Runs in a separate thread
- Uses mutexes and wait conditions for thread synchronization
- Safe for use from the main UI thread

### 2. Comprehensive Serial Configuration
- Configurable baud rates
- Configurable data bits (5-8)
- Configurable parity (none, even, odd)
- Configurable stop bits (1, 2)

### 3. Robust Data Handling
- Circular buffer for incoming data
- Non-blocking I/O operations
- Data capture with timeouts
- Pre-transmit data support

### 4. Error Handling
- Signal emission on errors
- Graceful port closure
- Error code reporting

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
SerialPort(QObject *parent = 0)
~SerialPort()
```

#### Port Management
```cpp
int openPort(const QString& port, int baudrate = 115200,
             SerialDataBits dataBits = DATA_8,
             SerialStopBits stopBits = STOP_1,
             SerialParity parity = PARITY_NONE)
void closePort()
bool isOpen()
```

#### Configuration Methods
```cpp
bool setBaudrate(int baudrate)
bool setParity(SerialParity parity)
bool setStopBits(SerialStopBits stopBits)
bool setDataBits(SerialDataBits dataBits)
```

#### Data Reading
```cpp
bool readByte(char& byte)
int readBytes(char* buffer, int bytes)
int readString(QString& string, int length)
QByteArray readAll()
int bytesAvailable()
```

#### Data Writing
```cpp
int writeData(const char* data, int length, bool block = true)
int writeData(const QByteArray &data, bool block = true)
bool writeByte(char byte, bool block = true)
bool writeString(const QString& string, bool block = true)
```

#### Data Capture
```cpp
int captureBytes(char* buffer, int num, int timeoutMs = 0, const QString& preTransmit = "")
int captureBytes(char* buffer, int num, int timeoutMs = 0, const char* preTransmit = 0, int preTransLen = 0)
```

### Enumerations

```cpp
enum SerialParity {PARITY_NONE, PARITY_EVEN, PARITY_ODD}
enum SerialDataBits {DATA_5, DATA_6, DATA_7, DATA_8}
enum SerialStopBits {STOP_1, STOP_2}
```

### Signals

```cpp
void serial_data_available()
void serial_port_error(int e)
```

## Implementation Details

### Thread Model

The SerialPort runs as a separate QThread:
- Main thread calls `openPort()`, `writeData()`, etc.
- Serial I/O happens in the worker thread
- Signals are emitted to notify the main thread
- Mutexes protect shared data structures

### Circular Buffer

Incoming data is stored in a circular buffer:
- Size: 32KB (configurable via `mBufferSize`)
- Prevents data loss from overflow
- Efficient memory usage
- Thread-safe access

### Non-Blocking I/O

The serial port is configured for non-blocking operations:
- `O_NDELAY` flag enables non-blocking mode
- `FNDELAY` flag for file control
- Timeout-based operations for blocking when needed

### Error Handling

The class handles various error conditions:
- Port open failures
- Configuration errors
- Read/write errors
- Timeout errors

## Usage Examples

### Basic Usage

```cpp
SerialPort *serial = new SerialPort(this);

// Connect signals
connect(serial, &SerialPort::serial_data_available, this, &MyClass::handleSerialData);
connect(serial, &SerialPort::serial_port_error, this, &MyClass::handleSerialError);

// Open port
if (serial->openPort("/dev/ttyUSB0", 115200) != 0) {
    qDebug() << "Failed to open serial port";
}
```

### Reading Data

```cpp
// Check if data is available
if (serial->bytesAvailable() > 0) {
    // Read all available data
    QByteArray data = serial->readAll();
    processData(data);
}

// Read specific number of bytes
char buffer[256];
int bytesRead = serial->readBytes(buffer, 256);
```

### Writing Data

```cpp
// Write QByteArray
QByteArray data;
data.append("Hello");
serial->writeData(data);

// Write string
serial->writeString("Hello, World!");

// Write single byte
serial->writeByte(0xAA);
```

### Data Capture with Timeout

```cpp
char buffer[100];
int bytesRead = serial->captureBytes(buffer, 100, 1000); // 1 second timeout

if (bytesRead > 0) {
    processCapturedData(buffer, bytesRead);
}
```

### Pre-Transmit Data

```cpp
char buffer[100];
char preTransmit[] = {0xAA, 0xBB};

// Send pre-transmit data, then capture response
int bytesRead = serial->captureBytes(buffer, 100, 1000, preTransmit, 2);
```

### Configuration Changes

```cpp
// Change baud rate
serial->setBaudrate(57600);

// Change parity
serial->setParity(SerialPort::PARITY_EVEN);

// Change data bits
serial->setDataBits(SerialPort::DATA_7);

// Change stop bits
serial->setStopBits(SerialPort::STOP_2);
```

## Error Handling

### Error Codes

The `serial_port_error` signal provides error codes:
- `-1`: Port open failed
- `-2`: Reading serial port options failed
- `-3`: Setting serial port options failed
- `-4`: Setting baud rate failed
- And other system error codes

### Handling Errors

```cpp
void MyClass::handleSerialError(int error) {
    if (error == -1) {
        qDebug() << "Port open failed";
    } else if (error == -2) {
        qDebug() << "Reading serial port options failed";
    } else {
        qDebug() << "Serial port error:" << error;
    }
}
```

## Best Practices

1. **Always check return values**: Verify port open succeeded
2. **Handle errors gracefully**: Connect to error signals
3. **Check bytes available**: Before reading data
4. **Use appropriate timeouts**: Balance responsiveness with reliability
5. **Close ports properly**: Call `closePort()` before destruction
6. **Thread safety**: All methods are thread-safe from main thread

## Performance Considerations

- **Buffer size**: 32KB default is sufficient for most applications
- **Non-blocking I/O**: Minimizes thread blocking
- **Circular buffer**: Efficient memory usage
- **Timeouts**: Configurable for different use cases

## Comparison with QSerialPort

The custom `SerialPort` class vs. Qt's `QSerialPort`:

**Custom SerialPort**:
- Thread-based architecture
- Circular buffer for data
- Pre-transmit support
- Capture with timeout
- More control over low-level operations

**QSerialPort**:
- Built into Qt
- Signal-based API
- Simpler API
- Less control over buffering

## Future Enhancements

Potential improvements:
- Configurable buffer sizes
- Flow control support
- Hardware handshaking
- More comprehensive error reporting
- Asynchronous I/O support
- Multiple port support
