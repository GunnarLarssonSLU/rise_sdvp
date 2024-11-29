**Car_Client** is a program ran at the Raspberry Pi at start up. As describe in the [main documentation](../../README.md). It can take a large number of arguments:
- **tcprtcmserver \[port\]**: Start RTCM server on \[port\] (e.g. 8200)
- **tcpubxserver \[port\]***: Start server for UBX data on \[port\] (e.g. 8210)
- **tcplogserver \[port\]**: Start log server on \[port\] (e.g. 8410)
- **tcpnmeasrv**: NMEA server address
- **tcpnmeaport**: NMEA server port
- **useudp**: Use UDP server"
- **udpport**: Port to use for the UDP server"
- **usetcp**: Use TCP server (will be used by default)
- **notcp**: Do not use TCP server"
- **tcpport**: Specify port to use for the TCP server (default: 8300)
- **logusb**: Store log files
- **logusbdir**: Directory to store USB logs to
- **inputrtcm**: Input RTCM data from serial port
- **ttyportrtcm**: Serial port for RTCM, e.g. /dev/ttyUSB0
- **rtcmbaud**: RTCM port baud rate, e.g. 9600
- **chronos**: Run CHRONOS client
- **chronossettxid**: Set CHRONOS transmitter id
- **chronoshostaddr \[address\]*** : Use network interface with address (default: any interface)
- **ntrip \[server\]:\[stream\]:\[user\]:\[password\]:\[port\] : Connect to ntrip server
- **rtcmbasepos \[lat\]:\[lon\]:\[height\]**: Inject RTCM base position message
- **batterycells**: Number of cells in series, e.g. for the GUI battery indicator
- **simulatecars \[num\]:\[firstid\]**: Simulate num cars where the first car has ID firstid
- **dynosim**: Run simulator together with output from AWITAR dyno instead of MotorSim
- **simaprepeatroutes \[1 or 0\]**: Repeat routes setting for the simulation (default: 1)
- **simlogen \[rateHz\]**: Enable simulator logging output at rateHz Hz
- **simuwben \[port\]:\[rateHz\]**: Enable simulator UWB emulation output on TCP port port at rateHz Hz
- **setid \[id\]**: Override ID of board
- **broadcastcarstate**: Broadcast state received from car on TCP Port 2102

The program works as an interface between the laptop program (RControlStation) and the controller card (and an Arduino if there is one). It transmits data to/from the laptop via TCP and to/from the controller card (and Arduino) via a serial port.

The main function initiate an object of class **Car_Client**. The constructor of that class:
- Initiate RTCM
- Initiate TCP server
- Create a large number of [Signal and slots](https://en.wikipedia.org/wiki/Signals_and_slots) connections. Some of particular importance are:
  - readyRead from serial ports that triggers corresponding functions in the class. 
  - readyRead from the TCP socker that triggers the tcpDataAvailable function in the class


