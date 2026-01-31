# Position Tracking System Documentation

## Overview

The `pos.c` module implements a comprehensive position tracking system that integrates data from multiple sensors (IMU, GPS, wheel encoders) to provide accurate position, orientation, and velocity information for autonomous navigation. This system is critical for vehicle localization and navigation tasks.

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                     POSITION TRACKING SYSTEM                              │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌─────────────┐    ┌─────────────┐    ┌───────────────────────────────┐  │
│  │   IMU       │    │    GPS      │    │   Wheel Encoders/Odometry     │  │
│  │ (BMI160/    │    │ (u-blox)    │    │ (VESC, Hydraulic, IO Board)   │  │
│  │  MPU9150)   │    │             │    │                               │  │
│  └─────────────┘    └─────────────┘    └───────────────────────────────┘  │
│         │                  │                  │                          │
│         ▼                  ▼                  ▼                          │
│  ┌───────────────────────────────────────────────────────────────────┐  │
│  │                     POSITION FUSION ENGINE                        │  │
│  │                                                                   │  │
│  │  ┌─────────────┐    ┌─────────────┐    ┌─────────────────────────┐  │  │
│  │  │ Attitude    │    │ Local ENU   │    │ Dead Reckoning         │  │  │
│  │  │ Estimation  │    │ Frame       │    │ (Odometry Integration) │  │  │
│  │  │ (Mahony/     │    │ (GPS to     │    │                         │  │  │
│  │  │ Madgwick)   │    │ local ENU)  │    │                         │  │  │
│  │  └─────────────┘    └─────────────┘    └─────────────────────────┘  │  │
│  │                                                                   │  │
│  │  ┌─────────────────────────────────────────────────────────────┐  │  │
│  │  │                     POSITION STATE                           │  │  │
│  │  │                                                                 │  │  │
│  │  │  - Position (x, y, z)                                         │  │  │
│  │  │  - Orientation (yaw, pitch, roll)                             │  │  │
│  │  │  - Velocity (vx, vy, speed)                                   │  │  │
│  │  │  - Quaternion (q0, q1, q2, q3)                                │  │  │
│  │  │  - GPS corrections                                            │  │  │
│  │  └─────────────────────────────────────────────────────────────┘  │  │
│  └───────────────────────────────────────────────────────────────────┘  │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

## Hardware Components

### IMU Sensors

The system supports two IMU options:

1. **BMI160** (Preferred)
   - Digital 9-axis IMU (accelerometer, gyroscope, magnetometer)
   - I2C/SPI interface
   - Configurable sample rate (500Hz in this implementation)
   - Built-in sensor fusion

2. **MPU9150** (Legacy)
   - Digital 9-axis IMU (accelerometer, gyroscope, magnetometer)
   - I2C interface
   - Requires external magnetometer calibration
   - Note: MPU9250 footprint is rotated 180° compared to MPU9150

### GPS Receiver

- **u-blox modules** supported
- NMEA and UBX protocol support
- RTCM3 correction capability
- PPS (Pulse Per Second) for precise time synchronization
- GPS antenna offset compensation

### Odometry Sources

1. **VESC-based** (Wheel encoders)
   - Reads RPM from VESC controllers
   - Supports differential steering
   - Calculates distance from wheel rotation

2. **Hydraulic drive**
   - Reads distance and speed from hydraulic sensors
   - Calculates position based on fluid flow

3. **IO Board**
   - Alternative sensor interface
   - Provides position and velocity data

## Sensor Fusion Algorithms

### Attitude Estimation

The system uses sensor fusion algorithms to estimate orientation:

1. **Mahony Filter**
   - Simple, fast attitude estimation
   - Gyroscope-based prediction
   - Accelerometer/magnetometer correction
   - Quaternion output

2. **Madgwick Filter**
   - Enhanced version of Mahony
   - Better handling of magnetometer disturbances
   - More robust in dynamic conditions

**Initialization Process:**
1. Uses accelerometer to determine gravity vector
2. Uses magnetometer to determine magnetic north
3. Calculates initial quaternion orientation
4. Transitions to continuous estimation mode

### Position Correction

GPS position corrections are applied periodically:

1. **Position Correction Algorithm:**
   ```
   position_error = position_gps - position_odometry
   correction_gain = gain_static + gain_dynamic * distance_traveled
   position_corrected += correction_gain * position_error
   ```

2. **Yaw Correction Algorithm:**
   ```
   yaw_gps = atan2(dy, dx)  // From GPS position difference
   yaw_error = yaw_gps - yaw_imu
   imu_yaw_offset += yaw_error * yaw_correction_gain
   ```

## Coordinate Systems

### WGS84 (World Geodetic System 1984)
- Global earth model
- Latitude, longitude, height
- Used for GPS positioning

### ECEF (Earth-Centered, Earth-Fixed)
- Cartesian coordinates relative to Earth center
- X: Intersection of Equator and Prime Meridian
- Y: 90°E longitude
- Z: North Pole

### ENU (East-North-Up)
- Local coordinate system
- **X**: East
- **Y**: North
- **Z**: Up
- Right-handed coordinate system

**Transformation Equations:**
```
// WGS84 to ECEF
x = (v + height) * cos(lat) * cos(lon)
y = (v + height) * cos(lat) * sin(lon)
z = (v * (1 - e²) + height) * sin(lat)

// ECEF to ENU
lx = r1c1*(x-ix) + r1c2*(y-iy) + r1c3*(z-iz)
ly = r2c1*(x-ix) + r2c2*(y-iy) + r2c3*(z-iz)
lz = r3c1*(x-ix) + r3c2*(y-iy) + r3c3*(z-iz)
```

## Initialization Process

### pos_init()

This function initializes all position-related subsystems:

1. **IMU Initialization:**
   - Configures BMI160 or MPU9150
   - Sets sample rate (500Hz)
   - Calibrates gyroscope offsets
   - Registers read callback

2. **GPS Initialization:**
   - Sets up NMEA and UBX callbacks
   - Configures PPS interrupt
   - Initializes coordinate transformations

3. **Timer Configuration:**
   - 50kHz timer for position updates
   - ~5kHz effective update rate

4. **State Initialization:**
   - Clears position and GPS state
   - Initializes history buffer
   - Sets up mutexes for thread safety

5. **Terminal Commands:**
   - Registers debugging commands
   - Enables monitoring and configuration

## Key Functions

### Data Access Functions

#### pos_get_imu(float *accel, float *gyro, float *mag)
- Retrieves raw IMU sensor data
- Provides direct access to unprocessed sensor readings
- Parameters can be NULL if not needed

#### pos_get_quaternions(float *q)
- Gets current orientation as quaternion
- Quaternion: q0 + q1i + q2j + q3k
- Thread-safe with mutex protection

#### pos_get_pos(POS_STATE *p)
- Retrieves complete position state
- Includes position, orientation, velocity
- Thread-safe with mutex protection

#### pos_get_gps(GPS_STATE *p)
- Retrieves complete GPS state
- Includes WGS84, ECEF, and ENU coordinates
- Thread-safe with mutex protection

#### pos_get_speed()
- Gets current speed in m/s
- Calculated from odometry data

### Position Setting Functions

#### pos_set_xya(float x, float y, float angle)
- Manually sets position and yaw
- Useful for initialization or reset
- Recalculates IMU yaw offset

#### pos_set_yaw_offset(float angle)
- Sets yaw offset between IMU and vehicle
- Used for calibration

#### pos_set_enu_ref(double lat, double lon, double height)
- Sets local ENU reference frame origin
- Calculates rotation matrix for ECEF to ENU
- Essential for multi-location operation

### GPS Processing Functions

#### pos_input_nmea(const char *data)
- Processes NMEA GPS sentences
- Handles GGA, GPGSV, GLGSV
- Updates GPS state and position

#### pos_pps_cb(EXTDriver *extp, expchannel_t channel)
- PPS interrupt handler
- Provides precise time synchronization
- Aligns GPS corrections with correct time

### Utility Functions

#### pos_reset_attitude()
- Resets attitude estimation
- Re-initializes orientation

#### pos_reset_enu_ref()
- Resets local ENU reference
- Used when moving to new location

#### pos_time_since_gps_corr()
- Gets time since last GPS correction
- Useful for monitoring correction frequency

## Odometry Integration

### Wheel-Based Odometry

For vehicles with wheel encoders:

1. **Distance Calculation:**
   ```
   distance = rpm * gear_ratio * (2/motor_poles) * (1/60) * wheel_diameter * π
   ```

2. **Position Update:**
   ```
   angle_rad = -yaw * π/180
   px += cos(angle_rad) * distance
   py += sin(angle_rad) * distance
   ```

3. **Turning Compensation:**
   - Calculates turn radius from steering angle
   - Adjusts position for curved paths
   - Updates yaw based on turning

### Differential Steering

For vehicles with independent left/right wheels:

1. **Dual VESC Reading:**
   - Alternates between left and right VESC
   - Reads RPM from each wheel

2. **Distance Calculation:**
   ```
   distance_left = ...
   distance_right = ...
   distance_diff = distance_right - distance_left
   ```

3. **Turn Radius Calculation:**
   ```
   turn_rad_rear = axis_distance * (distance_left + distance_right) / (2 * distance_diff)
   angle_diff = (distance_diff) / axis_distance
   ```

4. **Position Update:**
   ```
   px += turn_rad_rear * (sin(angle_rad + angle_diff) - sin(angle_rad))
   py += turn_rad_rear * (cos(angle_rad - angle_diff) - cos(angle_rad))
   yaw += angle_diff
   ```

## GPS Processing

### NMEA Sentence Processing

The system processes several NMEA sentence types:

1. **GGA (Global Positioning System Fix Data):**
   - Position, time, fix quality
   - Number of satellites
   - Horizontal dilution of precision

2. **GPGSV (GPS Satellites in View):**
   - Satellite PRN numbers
   - Elevation and azimuth
   - Signal-to-noise ratio

3. **GLGSV (GLONASS Satellites in View):**
   - Similar to GPGSV for GLONASS satellites

### Coordinate Transformations

1. **WGS84 to ECEF:**
   ```
   x = (v + height) * cos(lat) * cos(lon)
   y = (v + height) * cos(lat) * sin(lon)
   z = (v * (1 - e²) + height) * sin(lat)
   ```

2. **ECEF to ENU:**
   ```
   lx = r1c1*(x-ix) + r1c2*(y-iy) + r1c3*(z-iz)
   ly = r2c1*(x-ix) + r2c2*(y-iy) + r2c3*(z-iz)
   lz = r3c1*(x-ix) + r3c2*(y-iy) + r3c3*(z-iz)
   ```

3. **GPS Antenna Offset:**
   ```
   px -= cos(yaw) * gps_ant_x - sin(yaw) * gps_ant_y
   py -= sin(yaw) * gps_ant_x + cos(yaw) * gps_ant_y
   ```

## Configuration Parameters

### Main Configuration (conf_general.h)

```c
// GPS Configuration
#define GPS_ANT_X                    0.0    // GPS antenna X offset (m)
#define GPS_ANT_Y                    0.0    // GPS antenna Y offset (m)
#define GPS_GROUND_LEVEL             0.0    // Ground level offset (m)
#define GPS_COMP                    1      // Enable GPS position correction
#define GPS_REQ_RTK                 0      // Require RTK fix for correction
#define GPS_USE_UBX_INFO            0      // Use u-blox quality indication
#define GPS_UBX_MAX_ACC             10.0   // Maximum u-blox accuracy (m)

// GPS Correction Gains
#define GPS_CORR_GAIN_STAT          0.1    // Static correction gain
#define GPS_CORR_GAIN_DYN           0.01   // Dynamic correction gain
#define GPS_CORR_GAIN_YAW           0.01   // Yaw correction gain

// Magnetometer Calibration
#define MAG_COMP                    1      // Enable magnetometer calibration
#define MAG_USE                     1      // Use magnetometer for yaw
#define YAW_MAG_GAIN                10.0   // Magnetometer influence gain

// Vehicle Parameters
#define YAW_USE_ODOMETRY            1      // Use odometry for yaw
#define YAW_IMU_GAIN                0.1    // IMU yaw influence gain
#define CLAMP_IMU_YAW_STATIONARY    1      // Clamp yaw when stationary
```

## Terminal Commands

The position system provides several terminal commands for debugging and configuration:

### Position Correction Commands

| Command | Description | Usage |
|---------|-------------|-------|
| `pos_delay_info [0/1]` | Enable/disable delay information printing | `pos_delay_info 1` |
| `pos_gnss_corr_info [0/1]` | Enable/disable GNSS correction information | `pos_gnss_corr_info 1` |
| `pos_delay_comp [0/1]` | Enable/disable delay compensation | `pos_delay_comp 1` |

### Satellite Information

| Command | Description | Usage |
|---------|-------------|-------|
| `pos_print_sat_info` | Print all satellite information | `pos_print_sat_info` |
| `pos_sat_info [prn]` | Print information about specific satellite | `pos_sat_info 12` |

### Position Control

| Command | Description | Usage |
|---------|-------------|-------|
| `setpos x y angle` | Manually set position and yaw | `setpos 0.0 0.0 0.0` |
| `debug code` | Set debug mode | `debug 1` |
| `pos_print_testGL` | Output test information | `pos_print_testGL` |

## Debugging and Monitoring

### Debug Modes

Set `iDebug` to enable specific debug output:

| Value | Output |
|-------|--------|
| 1 | GPS initialization messages |
| 2 | Position correction details |
| 4 | GPS correction status |
| 6 | Yaw angle monitoring |
| 8 | Yaw drift accumulation |
| 11 | Extended GPS initialization |
| 16 | Yaw odometry usage |
| 44 | GPS angle correction |

### Debug Variables

The system exports debug values for monitoring:

| Variable | Description |
|----------|-------------|
| `iDebug` | Debug mode selector |
| `debugvalue6` | IMU yaw angle |
| `m_pos` | Current position state |
| `m_gps` | Current GPS state |
| `m_mag`, `m_gyro`, `m_accel` | Raw sensor data |

## Performance Characteristics

| Parameter | Value |
|-----------|-------|
| Update Rate | ~5kHz (200μs) |
| Position History | 100 entries |
| GPS Update Rate | 1-10Hz (NMEA) |
| IMU Update Rate | 50kHz timer |
| Correction Rate | ~1-10Hz |
| Yaw Correction Gain | 0.01 (adaptive) |
| Position Correction Gain | 0.1-0.2 (adaptive) |

## Error Handling

### Fault Detection

1. **GPS Quality Check:**
   - Validates u-blox position quality
   - Checks accuracy thresholds
   - Verifies fix type

2. **Sensor Validation:**
   - Magnetometer calibration
   - Accelerometer range checking
   - Gyroscope bias compensation

3. **Position Validation:**
   - Distance thresholding
   - Speed validation
   - Yaw rate limits

### Recovery Strategies

1. **Attitude Reset:** `pos_reset_attitude()`
2. **Position Reset:** `pos_set_xya()`
3. **Reference Reset:** `pos_reset_enu_ref()`
4. **Yaw Offset Adjustment:** `pos_set_yaw_offset()`

## Typical Workflow

When operating the vehicle at different locations, each with its own base station, the system's functions in pos.c handle this scenario through reinitialization and recalibration processes.

### At Each New Location:

1. **System Start:** Power on the vehicle, initiating the positioning system.
2. **Initialization:** `pos_init()` is called to reset and prepare the system.
3. **Set ENU Reference:** `pos_set_enu_ref()` establishes the local coordinate system based on the current base station.
4. **Position Retrieval:** `pos_get_pos()` can now be used to obtain accurate positional data in the context of the new location.

By following this sequence, the system effectively manages operations across different locations and base stations, ensuring accurate and reliable positioning data for each session.

## Integration with Other Systems

### VESC Communication

The position system integrates with VESC controllers:
- Reads motor values (RPM, current, duty cycle)
- Supports differential steering
- Uses CAN bus for communication

### Hydraulic Drive

For hydraulic vehicles:
- Reads distance and speed from hydraulic sensors
- Calculates turn radius from steering angle
- Integrates position based on hydraulic flow

### UWB Positioning

The system provides position data to UWB positioning:
- `pos_uwb_update_dr()` updates dead reckoning
- Provides yaw, position, and velocity
- Enables UWB-GPS fusion

## Troubleshooting

### Common Issues

1. **Position Drift:**
   - Check GPS correction enabled
   - Verify magnetometer calibration
   - Ensure proper yaw offset

2. **Erratic Yaw:**
   - Check magnetometer calibration
   - Verify IMU orientation
   - Adjust yaw correction gains

3. **Slow Updates:**
   - Check timer configuration
   - Verify IMU communication
   - Ensure sufficient CPU resources

4. **GPS Not Updating:**
   - Check NMEA sentence parsing
   - Verify PPS configuration
   - Test GPS receiver

### Debugging Steps

1. **Check IMU Data:**
   ```
   pos_print_testGL
   ```

2. **Monitor Position:**
   ```
   pos_delay_info 1
   pos_gnss_corr_info 1
   ```

3. **Verify GPS:**
   ```
   pos_print_sat_info
   ```

4. **Check Specific Satellite:**
   ```
   pos_sat_info 12  // Monitor satellite PRN 12
   ```

## References

- **Mahony Filter**: https://x-io.co.uk/open-source-imu-and-ahrs-algorithms/
- **Madgwick Filter**: http://www.x-io.co.uk/node/8
- **u-blox Protocol**: https://www.u-blox.com/en/product-resources
- **RTCM3 Protocol**: http://rtcm3.org/
- **WGS84 Datum**: https://en.wikipedia.org/wiki/World_Geodetic_System
- **ENU Coordinate System**: https://en.wikipedia.org/wiki/East_north_up
- **BMI160 Datasheet**: https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/
- **MPU9150 Datasheet**: https://www.invensense.com/products/motion-tracking/9-axis/mpu-9150/

## Future Improvements

1. **Kalman Filter**: Replace Mahony/Madgwick with EKF for better accuracy
2. **Multiple GPS**: Support for multiple GPS receivers
3. **INS/GNSS Fusion**: Tightly-coupled GPS/IMU fusion
4. **Wheel Slip Detection**: Detect and compensate for wheel slip
5. **Terrain Adaptation**: Adapt correction gains based on terrain
6. **Battery Compensation**: Compensate for battery voltage effects
7. **Temperature Compensation**: Compensate for temperature effects on sensors
8. **Dynamic Model**: Vehicle dynamics model for better prediction

## Code Structure

### Main Functions

- `pos_init()`: System initialization
- `mpu9150_read()`: IMU data reading and processing
- `update_orientation_angles()`: Attitude estimation
- `correct_pos_gps()`: GPS position correction
- `vehicle_update_pos()`: Odometry integration
- `mc_values_received()`: VESC data processing

### Callback Functions

- `pos_pps_cb()`: PPS interrupt handler
- `ublox_relposned_rx()`: u-blox relative position callback
- `ubx_rx_rawx()`: u-blox raw observation callback

### Terminal Commands

- `cmd_terminal_delay_info()`: Delay information printing
- `cmd_terminal_gps_corr_info()`: GPS correction information
- `cmd_terminal_delay_comp()`: Delay compensation control
- `cmd_terminal_print_sat_info()`: Satellite information printing
- `cmd_terminal_sat_info()`: Specific satellite monitoring
- `cmd_terminal_setpos()`: Manual position setting
- `cmd_terminal_debug()`: Debug mode control
- `cmd_terminal_testGL()`: Test information printing

## Data Flow

1. **IMU Data Flow:**
   ```
   IMU Sensor → mpu9150_read() → update_orientation_angles() → POS_STATE
   ```

2. **GPS Data Flow:**
   ```
   GPS Receiver → pos_input_nmea() → GPS_STATE → correct_pos_gps() → POS_STATE
   ```

3. **Odometry Data Flow:**
   ```
   VESC/Hydraulic → mc_values_received() → vehicle_update_pos() → POS_STATE
   ```

4. **PPS Data Flow:**
   ```
   GPS PPS → pos_pps_cb() → Time Synchronization → Position Correction
   ```

## Thread Safety

The position system uses mutexes to ensure thread-safe access:

- `m_mutex_pos`: Protects POS_STATE access
- `m_mutex_gps`: Protects GPS_STATE access

All position and GPS state accesses are protected by these mutexes.

## Performance Optimization

1. **Timer-Based Updates**: Uses TIM6 for precise timing
2. **Efficient Math**: Uses fast trigonometric functions
3. **Selective Updates**: Only updates when necessary
4. **History Buffer**: Circular buffer for position history
5. **Adaptive Correction**: Gain increases with distance traveled

## Limitations

1. **GPS Latency**: Position corrections have inherent latency
2. **IMU Drift**: Long-term IMU drift requires GPS correction
3. **Wheel Slip**: Odometry accuracy affected by wheel slip
4. **Magnetic Interference**: Magnetometer affected by metal objects
5. **GPS Multipath**: GPS accuracy affected by signal reflections

## Best Practices

1. **Calibration**: Always calibrate magnetometer and IMU
2. **Initialization**: Initialize position at known location
3. **Monitoring**: Regularly monitor position accuracy
4. **Configuration**: Tune correction gains for specific vehicle
5. **Testing**: Test in various environments and conditions
6. **Logging**: Enable debug output during development

## Function Reference

| Function | Description | Input | Output |
|----------|-------------|-------|--------|
| `pos_init()` | Initialize position system | | |
| `pos_pps_cb()` | PPS interrupt handler | (EXTDriver*, expchannel_t) | |
| `pos_get_imu()` | Get raw IMU data | (float*, float*, float*) | |
| `pos_get_quaternions()` | Get orientation as quaternion | (float*) | |
| `pos_get_pos()` | Get position state | (POS_STATE*) | |
| `pos_get_gps()` | Get GPS state | (GPS_STATE*) | |
| `pos_get_speed()` | Get current speed | | (float) |
| `pos_set_xya()` | Set position manually | (float, float, float) | |
| `pos_set_yaw_offset()` | Set yaw offset | (float) | |
| `pos_set_enu_ref()` | Set ENU reference | (double, double, double) | |
| `pos_get_enu_ref()` | Get ENU reference | | (double[3]) |
| `pos_reset_enu_ref()` | Reset ENU reference | | |
| `pos_get_mc_val()` | Get motor controller values | (mc_values*) | |
| `pos_get_ms_today()` | Get time of day | | (int32_t) |
| `pos_set_ms_today()` | Set time of day | (int32_t) | |
| `pos_input_nmea()` | Process NMEA sentence | (const char*) | (bool) |
| `pos_reset_attitude()` | Reset attitude estimation | | |
| `pos_time_since_gps_corr()` | Time since GPS correction | | (int) |
| `pos_base_rtcm_obs()` | Process RTCM3 observations | (rtcm_obs_header_t*, rtcm_obs_t*, int) | |
