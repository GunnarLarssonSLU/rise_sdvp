# Utility Documentation

## Overview

The **utility** namespace provides a collection of helper functions for data conversion, mathematical operations, coordinate transformations, and system utilities. These functions are used throughout the CarClient application for various low-level operations.

## Categories of Functions

### 1. Buffer Operations
Functions for reading and writing binary data to/from buffers with various data types and scaling.

### 2. Mathematical Operations
Mathematical helper functions including mapping, rounding, and logarithmic calculations.

### 3. Coordinate Transformations
Functions for converting between different coordinate systems (LLH, XYZ, ENU).

### 4. System Utilities
Functions for system operations, signal waiting, and command execution.

## Buffer Operations

### Writing Functions

```cpp
void buffer_append_int64(uint8_t* buffer, int64_t number, int32_t *index)
void buffer_append_uint64(uint8_t *buffer, uint64_t number, int32_t *index)
void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index)
void buffer_append_uint32(uint8_t* buffer, uint32_t number, int32_t *index)
void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index)
void buffer_append_uint16(uint8_t* buffer, uint16_t number, int32_t *index)
void buffer_append_double16(uint8_t* buffer, double number, double scale, int32_t *index)
void buffer_append_double32(uint8_t* buffer, double number, double scale, int32_t *index)
void buffer_append_double64(uint8_t* buffer, double number, double scale, int32_t *index)
void buffer_append_double32_auto(uint8_t* buffer, double number, int32_t *index)
```

**Purpose**: Append various data types to a buffer with proper byte ordering.

**Parameters**:
- `buffer`: Output buffer
- `number`: Value to append
- `scale`: Scaling factor for floating-point values
- `index`: Pointer to current buffer index (updated by function)

### Reading Functions

```cpp
int16_t buffer_get_int16(const uint8_t *buffer, int32_t *index)
uint16_t buffer_get_uint16(const uint8_t *buffer, int32_t *index)
int32_t buffer_get_int32(const uint8_t *buffer, int32_t *index)
uint32_t buffer_get_uint32(const uint8_t *buffer, int32_t *index)
uint64_t buffer_get_uint64(const uint8_t *buffer, int32_t *index)
int64_t buffer_get_int64(const uint8_t *buffer, int32_t *index)
double buffer_get_double16(const uint8_t *buffer, double scale, int32_t *index)
double buffer_get_double32(const uint8_t *buffer, double scale, int32_t *index)
double buffer_get_double64(const uint8_t *buffer, double scale, int32_t *index)
double buffer_get_double32_auto(const uint8_t *buffer, int32_t *index)
```

**Purpose**: Read various data types from a buffer with proper byte ordering.

**Parameters**:
- `buffer`: Input buffer
- `scale`: Scaling factor for floating-point values
- `index`: Pointer to current buffer index (updated by function)

**Returns**: The extracted value

## Mathematical Operations

### Mapping Function

```cpp
double map(double x, double in_min, double in_max, double out_min, double out_max)
```

**Purpose**: Map a value from one range to another.

**Parameters**:
- `x`: Input value
- `in_min`: Minimum value of input range
- `in_max`: Maximum value of input range
- `out_min`: Minimum value of output range
- `out_max`: Maximum value of output range

**Returns**: Mapped value

### Rounding Function

```cpp
inline double roundDouble(double x)
```

**Purpose**: Round a double to the nearest integer.

**Parameters**:
- `x`: Value to round

**Returns**: Rounded value

### Logarithm Function

```cpp
double logn(double base, double number)
```

**Purpose**: Calculate logarithm with custom base.

**Parameters**:
- `base`: Logarithm base
- `number`: Number to calculate logarithm of

**Returns**: Logarithm result

### Sign Function

```cpp
double sign(double x)
```

**Purpose**: Get the sign of a value (-1, 0, or 1).

**Parameters**:
- `x`: Input value

**Returns**: Sign of the value

### Truncation Function

```cpp
int truncateNumber(double *number, double min, double max)
```

**Purpose**: Truncate a number to a specified range.

**Parameters**:
- `number`: Pointer to number to truncate
- `min`: Minimum allowed value
- `max`: Maximum allowed value

**Returns**: 1 if truncated, 0 otherwise

### Angle Normalization

```cpp
void normAngle(double *angle)
```

**Purpose**: Normalize an angle to the range [-π, π].

**Parameters**:
- `angle`: Pointer to angle to normalize

### Step Towards

```cpp
void stepTowards(double *value, double goal, double step)
```

**Purpose**: Move a value towards a goal by a maximum step size.

**Parameters**:
- `value`: Pointer to current value
- `goal`: Target value
- `step`: Maximum step size

## Coordinate Transformations

### LLH to XYZ

```cpp
void llhToXyz(double lat, double lon, double height, double *x, double *y, double *z)
```

**Purpose**: Convert latitude, longitude, height to Earth-centered Earth-fixed (ECEF) coordinates.

**Parameters**:
- `lat`: Latitude in degrees
- `lon`: Longitude in degrees
- `height`: Height above WGS84 ellipsoid in meters
- `x`, `y`, `z`: Output ECEF coordinates

### XYZ to LLH

```cpp
void xyzToLlh(double x, double y, double z, double *lat, double *lon, double *height)
```

**Purpose**: Convert ECEF coordinates to latitude, longitude, height.

**Parameters**:
- `x`, `y`, `z`: Input ECEF coordinates
- `lat`: Output latitude in degrees
- `lon`: Output longitude in degrees
- `height`: Output height above WGS84 ellipsoid in meters

### ENU Matrix Creation

```cpp
void createEnuMatrix(double lat, double lon, double *enuMat)
```

**Purpose**: Create a rotation matrix for converting between ENU and XYZ coordinates.

**Parameters**:
- `lat`: Reference latitude in degrees
- `lon`: Reference longitude in degrees
- `enuMat`: Output 3x3 rotation matrix

### LLH to ENU

```cpp
void llhToEnu(const double *iLlh, const double *llh, double *xyz)
```

**Purpose**: Convert LLH coordinates to East-North-Up relative to a reference point.

**Parameters**:
- `iLlh`: Reference LLH coordinates
- `llh`: Input LLH coordinates
- `xyz`: Output ENU coordinates

### ENU to LLH

```cpp
void enuToLlh(const double *iLlh, const double *xyz, double *llh)
```

**Purpose**: Convert ENU coordinates to LLH coordinates.

**Parameters**:
- `iLlh`: Reference LLH coordinates
- `xyz`: Input ENU coordinates
- `llh`: Output LLH coordinates

## System Utilities

### Signal Waiting

```cpp
bool waitSignal(QObject *sender, const char *signal, int timeoutMs)
```

**Purpose**: Wait for a signal to be emitted with a timeout.

**Parameters**:
- `sender`: Object that emits the signal
- `signal`: Signal name (e.g., "dataReady()")
- `timeoutMs`: Timeout in milliseconds

**Returns**: true if signal was received, false on timeout

### System Command Execution

```cpp
std::string systemcmd(const char* command)
```

**Purpose**: Execute a system command and return its output.

**Parameters**:
- `command`: Command to execute

**Returns**: Command output as string

### UTC Time

```cpp
int getTimeUtcToday()
```

**Purpose**: Get current UTC time in seconds since midnight.

**Returns**: Seconds since midnight UTC

### CRC-16 Calculation

```cpp
unsigned short crc16(const unsigned char *buf, unsigned int len)
```

**Purpose**: Calculate CRC-16 checksum using precomputed lookup table.

**Parameters**:
- `buf`: Data buffer
- `len`: Length of data

**Returns**: CRC-16 checksum

## Usage Examples

### Buffer Operations

```cpp
uint8_t buffer[100];
int32_t index = 0;

// Write data to buffer
utility::buffer_append_int32(buffer, 12345, &index);
utility::buffer_append_double32(buffer, 3.14159, 1000.0, &index);

// Read data from buffer
index = 0;
int32_t value = utility::buffer_get_int32(buffer, &index);
double dvalue = utility::buffer_get_double32(buffer, 1000.0, &index);
```

### Coordinate Transformations

```cpp
// Convert LLH to XYZ
double lat = 59.3293, lon = 18.0686, height = 10.5;
double x, y, z;
utility::llhToXyz(lat, lon, height, &x, &y, &z);

// Convert back to LLH
double lat2, lon2, height2;
utility::xyzToLlh(x, y, z, &lat2, &lon2, &height2);
```

### ENU Coordinates

```cpp
// Set reference point
double refLat = 59.3293, refLon = 18.0686;
double enuMat[9];
utility::createEnuMatrix(refLat, refLon, enuMat);

// Convert LLH to ENU relative to reference
double pointLat = 59.3294, pointLon = 18.0687, pointHeight = 10.5;
double enu[3];
utility::llhToEnu(&refLat, &pointLat, enu);
```

### Signal Waiting

```cpp
// Wait for a signal with timeout
if (utility::waitSignal(myObject, "dataReady()", 5000)) {
    // Signal received within 5 seconds
    processData();
} else {
    // Timeout occurred
    qDebug() << "Timeout waiting for signal";
}
```

### System Command

```cpp
// Execute command and get output
std::string output = utility::systemcmd("ls -la /tmp");
qDebug() << "Command output:" << QString::fromStdString(output);
```

## Best Practices

1. **Buffer management**: Always track buffer index when reading/writing
2. **Coordinate systems**: Be clear about which coordinate system is being used
3. **Error handling**: Check return values for functions that return status
4. **Thread safety**: Most functions are thread-safe, but signal waiting is not
5. **Precision**: Understand scaling factors for floating-point operations

## Performance Considerations

- **Buffer operations**: Optimized for speed with direct memory access
- **CRC calculation**: Uses precomputed lookup table for efficiency
- **Coordinate transformations**: Uses standard WGS84 ellipsoid parameters
- **Signal waiting**: Uses Qt's event loop for efficient waiting

## Dependencies

- C++ standard library
- Qt (for signal waiting and datetime)
- Math library (for trigonometric functions)

## Future Enhancements

Potential improvements:
- More coordinate systems (UTM, MGRS, etc.)
- Additional mathematical functions
- Buffer overflow checking
- Endianness detection and handling
- More comprehensive error checking
- Support for additional data types
