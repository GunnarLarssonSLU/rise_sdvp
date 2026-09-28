# USB Communication Troubleshooting Guide for Car_Client

## Overview

This document describes the USB communication flow in Car_Client and provides troubleshooting guidance for when the program gets stuck due to USB communication issues.

## Communication Flow

The USB communication in Car_Client follows this path:

```
main.cpp (with -p /dev/vehicle)
  ↓
CarClient::connectSerial(port, baudrate)
  ↓
SerialPort::openPort(port, baudrate)
  ↓
SerialPort::run() (read thread)
  ↓
CarClient::serialDataAvailable()
  ↓
CarClient::processCarData()
  ↓
PacketInterface::processData()
  ↓
PacketInterface::processPacket()
```

For outgoing data:
```
CarClient::packetDataToSend()
  ↓
PacketInterface::sendPacket()
  ↓
SerialPort::writeData()
```

## Added Debug Messages

The following qdebug messages have been added to help diagnose USB communication issues:

### serialport.cpp
- **SerialPort::openPort**: Logs port opening attempts, file descriptor, and configuration steps
- **SerialPort::run**: Logs read thread startup, bytes read, and read errors
- **SerialPort::writeData**: Logs write attempts, bytes written, and write errors

### carclient.cpp
- **CarClient::connectSerial**: Logs connection attempts, results, and state changes
- **CarClient::serialDataAvailable**: Logs when data is available and how many bytes are read
- **CarClient::serialPortError**: Logs serial port errors with error codes
- **CarClient::processCarData**: Logs data processing

### packetinterface.cpp
- **PacketInterface::processData**: Logs packet processing state and byte counts
- **PacketInterface::sendPacket**: Logs packet sending via UDP or serial
- **PacketInterface::processPacket**: Logs packet reception and CRC verification

## Potential Causes of USB Communication Failures

### 1. Device File Issues

**Symptoms**: Program fails to open /dev/vehicle, gets stuck at startup

**Possible Causes**:
- `/dev/vehicle` does not exist
- The device file exists but points to the wrong device
- Permission issues (user doesn't have read/write access)
- The device is already opened by another process

**Debugging Steps**:
1. Check if the device exists: `ls -la /dev/vehicle`
2. Check if it's a symlink: `ls -la /dev/vehicle` (should point to actual tty device)
3. Check permissions: Ensure user has rw access
4. Check if device is already in use: `lsof /dev/vehicle`
5. Check actual serial devices: `ls /dev/tty*`

**Solutions**:
- Create symlink if missing: `sudo ln -s /dev/ttyUSB0 /dev/vehicle` (or appropriate device)
- Fix permissions: `sudo chmod 666 /dev/ttyUSB0` or add user to dialout group
- Kill conflicting processes using the device

### 2. Wrong Baud Rate

**Symptoms**: Port opens but no data is received, or garbage data is received

**Possible Causes**:
- Baud rate mismatch between Car_Client and the device
- Default baud rate (115200) doesn't match device configuration

**Debugging Steps**:
1. Check what baud rate the device expects (consult device documentation)
2. Try different baud rates: 9600, 19200, 38400, 57600, 115200, 230400
3. Use command line: `./Car_Client -p /dev/vehicle -b 57600`

**Solutions**:
- Specify correct baud rate with `-b` or `--baudrate` parameter
- Check device configuration and match it

### 3. Serial Port Configuration Issues

**Symptoms**: Port opens but communication doesn't work, errors during configuration

**Possible Causes**:
- Data bits, stop bits, or parity mismatch
- Flow control issues
- Non-standard baud rate not supported by driver

**Debugging Steps**:
1. Check the qdebug output for configuration errors
2. Look for messages like "Setting baudrate failed" or "Reading serial port options failed"
3. Check if custom baud rate is needed (error message will suggest this)

**Solutions**:
- Ensure device and Car_Client use same serial settings
- For custom baud rates, the driver needs to support it

### 4. Device Not Responding

**Symptoms**: Port opens, but no data is ever received (read thread times out repeatedly)

**Possible Causes**:
- Device is powered off or not connected
- Device is in a bootloader mode or other non-communication state
- Device requires a reset or specific initialization sequence
- Wrong device selected (e.g., /dev/ttyUSB0 vs /dev/ttyUSB1)

**Debugging Steps**:
1. Check if device is powered: Look for LEDs on the device
2. Check device connection: `lsusb` to see if device is detected
3. Try different /dev/ttyUSB* devices
4. Check if device needs reset: Try unplugging and replugging
5. Check if device is in bootloader: Some devices need to be reset to exit bootloader

**Solutions**:
- Power cycle the device
- Try different /dev/tty* devices
- Check device documentation for initialization requirements

### 5. Read Thread Getting Stuck

**Symptoms**: Program starts but becomes unresponsive, read thread stops processing

**Possible Causes**:
- pselect() call blocks indefinitely (shouldn't happen with timeout)
- Read operation blocks (non-blocking mode should prevent this)
- Too many consecutive failed reads (after 3 failures, port is closed)

**Debugging Steps**:
1. Check qdebug output for "pselect failed" or "Reading failed" messages
2. Check for "Too many consecutive failed reads" message
3. Check if read thread is still running: Look for periodic timeout messages

**Solutions**:
- The code now closes the port after 3 consecutive failed reads
- Check physical connection and device power
- Check for electrical issues (bad USB cable, loose connections)

### 6. Write Operations Failing

**Symptoms**: Commands are sent but not executed, write errors in logs

**Possible Causes**:
- Serial port closed during write
- Write buffer full (device not reading fast enough)
- Permission issues on write

**Debugging Steps**:
1. Check for "Serial port not open" messages in writeData
2. Check for "Writing to serial port failed" messages
3. Check if device is receiving data (device-side debugging)

**Solutions**:
- Ensure port is open before writing
- Check device is reading data from its serial port
- Reduce data rate if buffer is overflowing

### 7. Packet Parsing Issues

**Symptoms**: Data is received but not processed correctly, CRC errors

**Possible Causes**:
- CRC mismatch (data corruption)
- Invalid start bytes
- State machine getting stuck
- Buffer overflow

**Debugging Steps**:
1. Check for "CRC mismatch" messages in PacketInterface
2. Check for "Invalid start byte" messages (commented out by default)
3. Check for "Expected end byte 3" messages
4. Check packet state transitions

**Solutions**:
- Enable more verbose packet debugging
- Check for electrical noise or interference
- Verify baud rate matches on both ends

### 8. Device-Specific Issues

**Symptoms**: Various issues specific to the connected device

**Possible Causes for /dev/vehicle**:
- The device might be a custom embedded controller (RC_Controller)
- It might need specific initialization commands
- It might have specific timing requirements
- It might be running firmware that expects a specific protocol

**Debugging Steps**:
1. Check if device is a VESC/RC_Controller: Look for VEDDER in any output
2. Try sending a simple command like "help" via terminal
3. Check if device responds to ping or status requests

**Solutions**:
- Consult device-specific documentation
- Check firmware version on device
- Try updating device firmware

## Recommended Debugging Procedure

1. **Start with verbose logging**: Run with `QT_LOGGING_RULES=*.debug=true ./Car_Client -p /dev/vehicle`

2. **Check basic connectivity**:
   ```bash
   ls -la /dev/vehicle
   lsusb
   ls /dev/tty*
   ```

3. **Test with screen/minicom**:
   ```bash
   sudo apt install screen
   screen /dev/vehicle 115200
   ```
   (Press Ctrl+A then : to exit screen)

4. **Check for errors in dmesg**:
   ```bash
   dmesg | tail -20
   ```

5. **Check if device is detected by USB subsystem**:
   ```bash
   lsusb -v | grep -A 10 "Your Device"
   ```

6. **Test with different baud rates**:
   ```bash
   for baud in 9600 19200 38400 57600 115200 230400; do
     echo "Testing $baud..."
     timeout 2 ./Car_Client -p /dev/vehicle -b $baud 2>&1 | grep -i "connected\|failed\|error" || echo "No connection"
   done
   ```

## Common Error Messages and Their Meanings

| Error Message | Meaning | Solution |
|--------------|---------|----------|
| "Opening serial port failed" | Cannot open /dev/vehicle | Check device exists and permissions |
| "Reading serial port options failed" | Cannot get port settings | Check device is a valid serial port |
| "Writing serial port options failed" | Cannot configure port | Check baud rate is supported |
| "Setting baudrate failed" | Baud rate not supported | Try standard baud rate or check custom divisor |
| "pselect failed in read thread" | Select system call failed | Check for system errors |
| "Reading failed" | Read system call failed | Check device connection |
| "Reading serial port returned 0" | No data available | Check device is sending data |
| "Too many consecutive failed reads" | Port closed due to errors | Check physical connection |
| "Serial port not open" | Attempt to write to closed port | Check port is open before writing |
| "Writing to serial port failed" | Write system call failed | Check device is accepting data |
| "CRC mismatch" | Data corruption detected | Check baud rate and electrical connection |
| "Invalid state" | Packet parser state machine error | Enable more debugging |

## Electrical Troubleshooting

1. **Check USB cable**: Try a different, high-quality USB cable
2. **Check USB port**: Try a different USB port on the computer
3. **Check power**: Ensure device is properly powered
4. **Check for noise**: USB communication can be affected by electrical noise
5. **Try USB hub**: Some devices work better through a powered USB hub
6. **Check grounding**: Ensure proper grounding between devices

## Performance Considerations

1. **Baud rate**: Higher baud rates require better quality cables and connections
2. **Buffer sizes**: The SerialPort uses a 32KB circular buffer
3. **Timeouts**: Read thread uses 10ms timeout for pselect
4. **Failed reads**: Port closes after 3 consecutive failed reads

## Additional Tools

1. **strace**: Trace system calls to see what's happening at OS level
   ```bash
   strace -f -o debug.log ./Car_Client -p /dev/vehicle
   ```

2. **usbmon**: Monitor USB traffic (requires root)
   ```bash
   sudo modprobe usbmon
   sudo ls /sys/kernel/debug/usb/usbmon/
   sudo cat /sys/kernel/debug/usb/usbmon/0u > usb.log &
   ```

3. **serial test tools**:
   - `screen`: Simple terminal emulator
   - `minicom`: More advanced terminal emulator
   - `cu`: Another terminal emulator
   - `socat`: Can test serial connections

## Summary

The most common causes of USB communication getting stuck are:

1. **Device file doesn't exist or wrong device** - Check /dev/vehicle exists and points to correct device
2. **Permission issues** - Ensure user has read/write access to the device
3. **Wrong baud rate** - Verify baud rate matches device configuration
4. **Device not powered or not connected** - Check physical connection and power
5. **Device in wrong mode** - Device might need reset or firmware update

The added qdebug messages will help identify exactly where the communication is failing, making it much easier to diagnose and fix issues.
