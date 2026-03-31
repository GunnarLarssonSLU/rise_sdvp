# NmeaServer Documentation

## Overview

**NmeaServer** is a class for handling NMEA 0183 protocol communication. It provides both server functionality (broadcasting NMEA sentences) and client functionality (connecting to NMEA servers). The class supports various NMEA sentence types including GGA, RMC, ZDA, and raw NMEA messages.

## Key Features

### 1. NMEA Protocol Support
- **GGA (Global Positioning System Fix Data)**: Position and fix quality
- **RMC (Recommended Minimum Navigation Information)**: Position, velocity, time
- **ZDA (Time & Date)**: UTC time and date
- **Raw NMEA**: Any NMEA sentence

### 2. Server Functionality
- TCP broadcast server for NMEA data
- Multiple client support
- Configurable port

### 3. Client Functionality
- TCP client for connecting to NMEA servers
- Connection status monitoring
- Automatic reconnection

### 4. Logging
- Log NMEA data to file
- Timestamped log files
- Log file rotation

### 5. Data Parsing
- Parse GGA sentences to extract position data
- Extract latitude, longitude, altitude, satellite count, etc.

## Data Structures

### NMEA GGA Information
```cpp
typedef struct {
    double lat;        // Latitude in degrees
    double lon;        // Longitude in degrees
    double height;     // Altitude in meters
    double t_tow;      // Time of week in seconds
    int n_sat;         // Number of satellites
    int fix_type;      // Fix type (0=no fix, 1=GPS, 2=DGPS)
    double h_dop;      // Horizontal dilution of precision
    double diff_age;   // Differential GPS age
} nmea_gga_info_t;
```

### NMEA RMC Information
```cpp
typedef struct {
    double lat;        // Latitude in degrees
    double lon;        // Longitude in degrees
    double t_tow;      // Time of week in seconds
    quint16 t_wn;      // Week number
    double vel_x;      // Velocity in x direction (m/s)
    double vel_y;      // Velocity in y direction (m/s)
    double vel_z;      // Velocity in z direction (m/s)
} nmea_rmc_info_t;
```

## Class Structure

### Public Interface

#### Construction and Initialization
```cpp
NmeaServer(QObject *parent = 0)
~NmeaServer()
```

#### Server Methods
```cpp
bool startTcpServer(int port)
```

#### NMEA Sending Methods
```cpp
bool sendNmeaGga(nmea_gga_info_t &nmea)
bool sendNmeaZda(quint16 wn, double tow)
bool sendNmeaRmc(nmea_rmc_info_t &nmea)
bool sendNmeaRaw(QString msg)
```

#### Logging Methods
```cpp
bool logToFile(QString file)
void logStop()
```

#### Client Methods
```cpp
bool connectClientTcp(QString server, int port = 80)
bool isClientTcpConnected()
void disconnectClientTcp()
```

#### Parsing Methods
```cpp
static int decodeNmeaGGA(QByteArray data, nmea_gga_info_t &gga)
```

### Signals

```cpp
void clientGgaRx(int fields, NmeaServer::nmea_gga_info_t gga)
```

## Implementation Details

### NMEA Sentence Format

NMEA 0183 sentences follow this format:
```
$<talker><message>,<data>...*<checksum><CR><LF>
```

Example GGA sentence:
```
$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47
```

### Checksum Calculation

The checksum is calculated as the bitwise XOR of all characters between `$` and `*` (excluding these characters).

### Coordinate Conversion

NMEA uses degrees and decimal minutes format:
- `4807.038,N` = 48° 7.038' N = 48.1173° N
- `01131.000,E` = 11° 31.000' E = 11.5167° E

## Usage Examples

### Starting NMEA Server

```cpp
NmeaServer *nmeaServer = new NmeaServer(this);

// Start TCP server on port 2948
if (!nmeaServer->startTcpServer(2948)) {
    qDebug() << "Failed to start NMEA server";
}
```

### Sending NMEA GGA Sentence

```cpp
NmeaServer::nmea_gga_info_t gga;
gga.lat = 59.3293;      // Latitude
gga.lon = 18.0686;      // Longitude  
gga.height = 10.5;      // Altitude in meters
gga.t_tow = 123456.789; // Time of week
gga.n_sat = 8;          // Number of satellites
gga.fix_type = 1;       // GPS fix

nmeaServer->sendNmeaGga(gga);
```

### Sending NMEA RMC Sentence

```cpp
NmeaServer::nmea_rmc_info_t rmc;
rmc.lat = 59.3293;
rmc.lon = 18.0686;
rmc.t_tow = 123456.789;
rmc.t_wn = 2000;        // Week number
rmc.vel_x = 1.5;        // Velocity in x direction
rmc.vel_y = 0.2;        // Velocity in y direction
rmc.vel_z = 0.0;        // Velocity in z direction

nmeaServer->sendNmeaRmc(rmc);
```

### Connecting to NMEA Server

```cpp
// Connect to external NMEA server
if (!nmeaServer->connectClientTcp("192.168.1.100", 2948)) {
    qDebug() << "Failed to connect to NMEA server";
}

// Check connection status
if (nmeaServer->isClientTcpConnected()) {
    qDebug() << "Connected to NMEA server";
}
```

### Logging NMEA Data

```cpp
// Start logging to file
if (!nmeaServer->logToFile("/var/log/nmea.log")) {
    qDebug() << "Failed to start logging";
}

// Stop logging
nmeaServer->logStop();
```

### Parsing GGA Sentences

```cpp
QByteArray nmeaData = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";

NmeaServer::nmea_gga_info_t gga;
int fields = NmeaServer::decodeNmeaGGA(nmeaData, gga);

if (fields > 0) {
    qDebug() << "Latitude:" << gga.lat;
    qDebug() << "Longitude:" << gga.lon;
    qDebug() << "Altitude:" << gga.height;
}
```

## Error Handling

The NmeaServer handles various error conditions:
- **Server startup failures**: Returns false from `startTcpServer()`
- **Connection failures**: Returns false from `connectClientTcp()`
- **Logging failures**: Returns false from `logToFile()`
- **Socket errors**: Emitted via `tcpInputError()` signal

## Best Practices

1. **Check return values**: Verify operations succeed
2. **Handle errors**: Connect to error signals
3. **Validate data**: Check parsed NMEA data is reasonable
4. **Connection management**: Check connection status before sending
5. **Logging**: Rotate log files periodically

## Performance Considerations

- **Buffer sizes**: Use appropriate buffer sizes for NMEA data
- **Checksum calculation**: Efficient XOR-based checksum
- **Parsing**: Optimized parsing of NMEA sentences
- **Network**: Use appropriate timeouts for network operations

## Protocol Reference

### GGA Sentence
```
$GPGGA,hhmmss.ss,lat,lat_N/S,lon,lon_E/W,fix_quality,num_sats,HDOP,altitude,M,geoid,M,age,diff_ref,*checksum<CR><LF>
```

### RMC Sentence
```
$GPRMC,hhmmss.ss,A,lat,lat_N/S,lon,lon_E/W,speed,knots,track,ddmmyy,mag_var,mag_var_E/W,*checksum<CR><LF>
```

### ZDA Sentence
```
$GPZDA,hhmmss.ss,dd,mm,yyyy,xx,yy,*checksum<CR><LF>
```

## Future Enhancements

Potential improvements:
- Support for more NMEA sentence types (VTG, GSA, GSV, etc.)
- UDP server support
- NMEA 2.0 support
- More robust error recovery
- Configurable checksum calculation
- Support for different talker IDs (GP, GL, GA, etc.)
