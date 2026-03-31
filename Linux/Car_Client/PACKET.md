# Packet Documentation

## Overview

**Packet** is a lightweight packet framing and processing class that handles binary packet communication. It provides basic packet assembly, CRC checksumming, and packet reception with timeout handling.

## Protocol Specification

### Packet Structure

Each packet follows this format:
```
[START_BYTE][LENGTH][PAYLOAD][CRC16][END_BYTE]
```

- **START_BYTE**: 2, 3, or 4 - Indicates packet size
  - 2: Packet ≤ 255 bytes
  - 3: Packet ≤ 65535 bytes  
  - 4: Packet > 65535 bytes
- **LENGTH**: Variable length - Actual length based on start byte
  - For start=2: 1 byte (8-bit)
  - For start=3: 2 bytes (16-bit, big-endian)
  - For start=4: 3 bytes (24-bit, big-endian)
- **PAYLOAD**: Variable length - Application data
- **CRC16**: 2 bytes - CRC checksum covering payload
- **END_BYTE**: 3 - Packet end marker

## Key Features

### 1. Packet Framing
- **sendPacket()**: Adds framing, length, CRC, and end marker
- **Supports variable packet sizes**: Up to 16MB
- **CRC-16-CCITT checksum**: Ensures data integrity

### 2. Packet Reception
- **processData()**: State machine for packet reassembly
- **Timeout handling**: Detects incomplete packets
- **CRC validation**: Rejects corrupted packets

### 3. Signal-Based Architecture
- **dataToSend()**: Emitted when packet is ready to send
- **packetReceived()**: Emitted when complete packet is received

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
Packet(QObject *parent = 0)
```

#### Packet Sending
```cpp
void sendPacket(const QByteArray &data)
```

#### CRC Calculation
```cpp
static unsigned short crc16(const unsigned char *buf, unsigned int len)
```

### Signals

```cpp
void dataToSend(QByteArray &data)
void packetReceived(QByteArray &packet)
```

### Slots

```cpp
void processData(QByteArray data)
void timerSlot()
```

## Implementation Details

### Packet Processing State Machine

The `processData()` method implements a state machine:

1. **State 0**: Looking for start byte (2, 3, or 4)
2. **State 1**: Reading length bytes
3. **State 2**: Reading payload
4. **State 3**: Reading CRC (high byte)
5. **State 4**: Reading CRC (low byte)
6. **State 5**: Reading end byte
7. **State 6**: Validating CRC and emitting packet

### CRC Calculation

Uses CRC-16-CCITT algorithm with a precomputed lookup table:

```cpp
unsigned short crc16_tab[256]
```

The CRC is calculated over the payload only (not including framing bytes).

### Timer System

A 10ms timer drives the timeout logic:
- Resets timeout counter on each byte received
- Detects incomplete packets
- Prevents hanging on corrupted data

## Usage Examples

### Sending a Packet

```cpp
QByteArray data;
data.append("Hello, World!");

Packet *packetHandler = new Packet(this);
connect(packetHandler, &Packet::dataToSend, this, &MyClass::sendOverSerial);

packetHandler->sendPacket(data);
```

### Receiving Packets

```cpp
Packet *packetHandler = new Packet(this);
connect(packetHandler, &Packet::packetReceived, this, &MyClass::handlePacket);

// When data arrives from serial port
void MyClass::serialDataReceived(QByteArray data) {
    packetHandler->processData(data);
}

// Handle received packet
void MyClass::handlePacket(QByteArray packet) {
    qDebug() << "Received packet:" << packet;
}
```

### Handling Serial Data

```cpp
void MyClass::readSerialData() {
    QByteArray data = serialPort->readAll();
    if (!data.isEmpty()) {
        packetHandler->processData(data);
    }
}
```

## Error Handling

The Packet class handles various error conditions:
- **Timeout errors**: Incomplete packets are discarded
- **CRC errors**: Corrupted packets are rejected
- **Invalid framing**: Malformed packets are ignored
- **Buffer overflows**: Prevents memory corruption

## Threading Model

The Packet class is designed to work in Qt's single-threaded event loop:
- All methods are thread-safe when called from the main thread
- Timer-based operations use Qt's timer system
- Signal emission is thread-safe

## Dependencies

- Qt (Core)
- No external libraries required

## Best Practices

1. **Connect signals**: Always connect `dataToSend` and `packetReceived` signals
2. **Handle timeouts**: Configure appropriate byte timeout for your application
3. **Validate data**: Check packet content after reception
4. **Error handling**: Handle cases where packets are discarded
5. **Buffer management**: Ensure sufficient buffer size for expected packet sizes

## Performance Considerations

- **Packet size**: Keep packets under 1400 bytes for serial reliability
- **CRC calculation**: Uses precomputed table for efficiency
- **State machine**: Minimal overhead for packet processing
- **Memory usage**: Fixed-size buffers for predictable memory usage

## Comparison with PacketInterface

The `Packet` class is a simpler, lighter-weight version of `PacketInterface`:

**Packet**:
- Basic framing and CRC
- No acknowledgment system
- No command parsing
- Simpler API
- Lower overhead

**PacketInterface**:
- Full protocol implementation
- Acknowledgments and retries
- Command parsing
- UDP support
- More features but higher overhead

## Future Enhancements

Potential improvements:
- Configurable timeout values
- Packet fragmentation for large data
- Compression support
- Encryption for secure communication
- Flow control mechanisms
