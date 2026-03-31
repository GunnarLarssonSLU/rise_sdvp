# Main Application Documentation

## Overview

The **main.cpp** file contains the entry point for the CarClient application. It handles command-line argument parsing, application initialization, and signal handling. The application is designed to run on a Raspberry Pi and serves as the central communication hub for autonomous vehicle control systems.

## Command-Line Arguments

The application accepts numerous command-line arguments to configure its behavior:

### Basic Options
```
-h, --help : Show help text
-p, --ttyport : Serial port, e.g. /dev/ttyUSB0
-b, --baudrate : Serial baud rate, e.g. 9600
-l, --log : Log to file, e.g. /tmp/logfile.bin
```

### Server Configuration
```
--tcprtcmserver [port] : Start RTCM server on [port] (e.g. 8200)
--tcpubxserver [port] : Start server for UBX data on [port] (e.g. 8210)
--tcplogserver [port] : Start log server on [port] (e.g. 8410)
--tcpnmeasrv : NMEA server address
--tcpnmeaport : NMEA server port
--useudp : Use UDP server
--udpport : Port to use for the UDP server
--usetcp : Use TCP server (will be used by default)
--notcp : Do not use TCP server
--tcpport : Specify port to use for the TCP server (default: 8300)
```

### Logging
```
--logusb : Store log files
--logusbdir : Directory to store USB logs to
```

### RTCM Configuration
```
--inputrtcm : Input RTCM data from serial port
--ttyportrtcm : Serial port for RTCM, e.g. /dev/ttyUSB0
--rtcmbaud : RTCM port baud rate, e.g. 9600
--ntrip [server]:[stream]:[user]:[password]:[port] : Connect to ntrip server
--rtcmbasepos [lat]:[lon]:[height] : Inject RTCM base position message
```

### CHRONOS Configuration
```
--chronos : Run CHRONOS client
--chronossettxid : Set CHRONOS transmitter id
--chronoshostaddr [address] : Use network interface with address (default: any interface)
```

### Simulation
```
--simulatecars [num]:[firstid] : Simulate num cars where the first car has ID firstid
--dynosim : Run simulator together with output from AWITAR dyno instead of MotorSim
--simaprepeatroutes [1 or 0] : Repeat routes setting for the simulation (default: 1)
--simlogen [rateHz] : Enable simulator logging output at rateHz Hz
--simuwben [port]:[rateHz] : Enable simulator UWB emulation output on TCP port port at rateHz Hz
```

### System Configuration
```
--batterycells : Number of cells in series, e.g. for the GUI battery indicator
--setid [id] : Override ID of board
--broadcastcarstate : Broadcast state received from car on TCP Port 2102
```

### GUI (if enabled)
```
--usegui : Use QML GUI
```

## Application Flow

### 1. Initialization
- Parse command-line arguments
- Set up signal handlers (SIGINT, SIGTERM)
- Initialize Qt application (QCoreApplication or QApplication)
- Create CarClient instance

### 2. Configuration
- Configure serial ports based on arguments
- Start TCP/UDP servers as specified
- Set up RTCM and NMEA connections
- Configure logging
- Initialize simulation if enabled

### 3. Main Loop
- Qt event loop handles all communication
- Signals and slots manage data flow
- Clean shutdown on exit

## Signal Handling

The application sets up signal handlers for graceful shutdown:

```cpp
signal(SIGINT, m_cleanup);
signal(SIGTERM, m_cleanup);
```

When a signal is received:
1. The signal handler calls `qApp->quit()`
2. The application exits cleanly
3. A "Bye :)" message is printed

## Usage Examples

### Basic Usage
```bash
./carclient --ttyport /dev/ttyUSB0
```

### With RTCM Server
```bash
./carclient --ttyport /dev/ttyUSB0 --tcprtcmserver 8200
```

### With NMEA Server Connection
```bash
./carclient --tcpnmeasrv 192.168.1.100 --tcpnmeaport 2948
```

### With USB Logging
```bash
./carclient --ttyport /dev/ttyUSB0 --logusb --logusbdir /media/usb/logs
```

### With Simulation
```bash
./carclient --simulatecars 2:1 --simlogen 10
```

### With NTRIP Server
```bash
./carclient --ttyport /dev/ttyUSB0 --ntrip server.example.com:MOUNTPOINT:user:pass:80
```

### With RTCM Base Position
```bash
./carclient --ttyport /dev/ttyUSB0 --rtcmbasepos 59.3293:18.0686:10.5
```

## Error Handling

The application handles various error conditions:
- **Invalid arguments**: Shows help and exits
- **Serial port failures**: Logs errors and continues
- **Network failures**: Logs errors and continues
- **Signal interrupts**: Graceful shutdown

## Logging

The application uses qDebug() for logging:
- Command-line arguments are logged
- Configuration changes are logged
- Connection status is logged
- Errors are logged with qWarning() or qCritical()

## Build Configuration

The application can be built in different configurations:

### GUI Mode
```cpp
#ifdef HAS_GUI
#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
#endif
```

When GUI is enabled, the application can display a QML-based interface.

### Non-GUI Mode
```cpp
#else
#include <QCoreApplication>
#endif
```

The default mode is headless (no GUI), suitable for running on embedded systems.

## Dependencies

- Qt 5/6 (Core, Network)
- CarClient class
- Chronos class
- CarStateBroadcaster class
- Various system libraries (signal.h, etc.)

## Best Practices

1. **Use help option**: `--help` shows all available options
2. **Validate arguments**: Check that required arguments are provided
3. **Handle errors gracefully**: The application continues running after non-critical errors
4. **Use appropriate ports**: Default ports are provided but can be customized
5. **Configure logging**: Enable logging for debugging and troubleshooting

## Performance Considerations

- **Argument parsing**: Efficient parsing of command-line arguments
- **Signal handling**: Minimal overhead for signal handling
- **Resource management**: Proper cleanup on exit
- **Error recovery**: Continues running after non-critical errors

## Future Enhancements

Potential improvements:
- More comprehensive argument validation
- Config file support for persistent settings
- Environment variable support
- Subcommand support (e.g., `carclient start`, `carclient config`)
- Better error messages and suggestions
- Interactive mode for configuration
- Remote configuration interface
