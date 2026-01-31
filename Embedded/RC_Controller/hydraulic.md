# Hydraulic System Documentation

## Overview

This document provides comprehensive documentation for the hydraulic system control module in the RC_Controller firmware, specifically focusing on the `hydraulic.c` file.

## File Structure

### Location
- **File**: `hydraulic.c`
- **Purpose**: Control hydraulic actuators and vehicle propulsion
- **Architecture**: Single-thread design with periodic control loop

## Key Features

1. **Vehicle Propulsion Control**
   - Speed control via hydraulic servos
   - Throttle mapping and conversion
   - Distance tracking

2. **Hydraulic Actuator Control**
   - Front, rear, and extra hydraulic positions
   - Limit switch monitoring
   - Timeout-based safety

3. **Multi-Vehicle Support**
   - MacTrac configuration
   - Standard configuration
   - Configurable servo assignments

4. **Safety Features**
   - Timeout-based shutdown
   - Limit switch monitoring
   - Emergency stop support

5. **IO Integration**
   - CAN-based IO board control
   - ADDIO valve controller support
   - Speed sensor integration

## Detailed Documentation

### Configuration Settings

#### Vehicle-Specific Configuration

**MacTrac Configuration**
```c
#define SERVO_LEFT			10  // Only one servo
#define SERVO_RIGHT		1
#define SPEED_M_S			1.1
```

**Standard Configuration**
```c
#define SERVO_LEFT			1
#define SERVO_RIGHT		2
#define SPEED_M_S			0.6
```

**Key Differences**:
- **MacTrac**: Uses single servo, higher speed (1.1 m/s)
- **Standard**: Uses dual servos, lower speed (0.6 m/s)

#### Timeout Settings

**`TIMEOUT_SECONDS`** (2.0 seconds)
- **Purpose**: Speed control timeout
- **Behavior**: If no speed command received for 2 seconds, hydraulic stops
- **Safety**: Prevents unintended movement

**`TIMEOUT_SECONDS_MOVE`** (10.0 seconds)
- **Purpose**: Hydraulic move command timeout
- **Behavior**: If no move command received for 10 seconds, hydraulic stops
- **Safety**: Prevents actuators from being left in extended position

### Private Variables

#### Speed and Distance Tracking

**`m_speed_now`**
- **Type**: `volatile float`
- **Purpose**: Current vehicle speed in m/s
- **Source**: Speed sensor or calculated from throttle
- **Access**: Updated by hydraulic thread, read by other modules

**`m_distance_now`**
- **Type**: `volatile float`
- **Purpose**: Accumulated travel distance in meters
- **Calculation**: `distance += speed * time_interval`
- **Reset**: Can be reset via `hydraulic_get_distance(true)`

**`m_throttle_set`**
- **Type**: `volatile float`
- **Purpose**: Current throttle setting (-1.0 to 1.0)
- **Range**: -1.0 (full reverse) to 1.0 (full forward)
- **Usage**: Used for speed calculation when no sensor available

#### Timeout Counters

**`m_timeout_cnt`**
- **Type**: `volatile float`
- **Purpose**: Tracks time since last speed command
- **Increment**: +0.01 per 10ms loop iteration
- **Reset**: Set to 0.0 when speed command received

**`m_move_timeout_cnt`**
- **Type**: `volatile float`
- **Purpose**: Tracks time since last move command
- **Increment**: +0.01 per 10ms loop iteration
- **Reset**: Set to 0.0 when move command received

#### Hydraulic Actuator States

**`m_move_front`**, **`m_move_rear`**, **`m_move_extra`**
- **Type**: `volatile HYDRAULIC_MOVE`
- **Purpose**: Current state of hydraulic actuators
- **Values**:
  - `HYDRAULIC_MOVE_STOP` - Stopped
  - `HYDRAULIC_MOVE_UP` - Moving up/out
  - `HYDRAULIC_MOVE_DOWN` - Moving down/in
  - `HYDRAULIC_MOVE_OUT` - Moving out (extra)
  - `HYDRAULIC_MOVE_IN` - Moving in (extra)

### Public Functions

#### `hydraulic_init()`
```c
void hydraulic_init(void)
```

**Purpose**: Initializes the hydraulic system

**Steps**:
1. Configures PWM settings for MacTrac vs standard
2. Initializes PWM ESC interface
3. Sets all servos to neutral position (0.5)
4. Creates hydraulic control thread

**PWM Configuration**:
- **MacTrac**: 1000-2000 μs range
- **Standard**: 1000-2100 μs range

**Initial State**:
- All servos set to 0.5 (neutral)
- Speed and distance counters initialized to 0
- Timeout counters initialized to 0

#### `hydraulic_get_speed()`
```c
float hydraulic_get_speed(void)
```

**Purpose**: Gets current vehicle speed

**Return**: Speed in meters per second

**Usage**:
```c
float speed = hydraulic_get_speed();
commands_printf("Speed: %.2f m/s", speed);
```

#### `hydraulic_get_distance()`
```c
float hydraulic_get_distance(bool reset)
```

**Purpose**: Gets accumulated travel distance

**Parameters**:
- `reset`: If true, resets distance counter to 0

**Return**: Distance in meters

**Usage**:
```c
// Get distance without reset
float dist = hydraulic_get_distance(false);

// Get distance and reset counter
float dist = hydraulic_get_distance(true);
```

#### `hydraulic_set_speed()`
```c
void hydraulic_set_speed(float speed)
```

**Purpose**: Sets vehicle speed

**Parameters**:
- `speed`: Desired speed in m/s (positive = forward, negative = reverse)

**Behavior**:
1. Resets timeout counter
2. **If speed < 0.01**: Stops hydraulic servos
3. **If speed > 0.01**:
   - **MacTrac**: Single servo control with position mapping
   - **Standard**: Dual servo control (one for forward, one for reverse)
   - Updates speed tracking (if no sensor)

**Safety Check**:
- Returns early if system not in OPERATIONAL state

**Usage**:
```c
// Drive forward at 0.5 m/s
hydraulic_set_speed(0.5);

// Drive backward at 0.3 m/s
hydraulic_set_speed(-0.3);

// Stop
hydraulic_set_speed(0.0);
```

#### `hydraulic_set_throttle_raw()`
```c
void hydraulic_set_throttle_raw(float throttle)
```

**Purpose**: Sets hydraulic throttle directly

**Parameters**:
- `throttle`: Throttle value (-1.0 to 1.0)

**Behavior**:
1. Resets timeout counter
2. Truncates throttle to absolute value <= 1.0
3. Updates throttle setting
4. Maps throttle to servo positions
5. Updates speed tracking (if no sensor)

**Usage**:
```c
// Full forward
hydraulic_set_throttle_raw(1.0);

// Half forward
hydraulic_set_throttle_raw(0.5);

// Full reverse
hydraulic_set_throttle_raw(-1.0);
```

#### `hydraulic_move()`
```c
void hydraulic_move(HYDRAULIC_POS pos, HYDRAULIC_MOVE move)
```

**Purpose**: Controls hydraulic actuators

**Parameters**:
- `pos`: Hydraulic position (FRONT, REAR, EXTRA)
- `move`: Desired movement (STOP, UP, DOWN, OUT, IN)

**Behavior**:
1. Resets move timeout counter
2. Updates appropriate move state variable
3. Actual movement controlled by hydraulic thread

**Positions**:
- **FRONT**: Front hydraulic actuator
- **REAR**: Rear hydraulic actuator
- **EXTRA**: Extra/auxiliary hydraulic actuator

**Movements**:
- **STOP**: Stop actuator movement
- **UP/OUT**: Extend/raise actuator
- **DOWN/IN**: Retract/lower actuator

**Usage**:
```c
// Raise front actuator
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);

// Lower rear actuator
hydraulic_move(HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_DOWN);

// Stop all actuators
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_STOP);
hydraulic_move(HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_STOP);
hydraulic_move(HYDRAULIC_POS_EXTRA, HYDRAULIC_MOVE_STOP);
```

### Thread Function

#### `hydro_thread()`
```c
static THD_FUNCTION(hydro_thread, arg)
```

**Purpose**: Main hydraulic control loop

**Behavior**:
1. Sets thread name to "Hydraulic"
2. Initializes state tracking variables
3. Enters infinite loop (until thread termination)
4. **Main Loop** (10ms interval):
   - Updates distance based on current speed
   - Updates timeout counters
   - Applies timeouts if exceeded
   - Updates move timeout
   - Resets move commands if timeout exceeded
   - Resets move states periodically
   - Reads speed sensor data (if available)
   - Controls hydraulic actuators (MacTrac only)
   - Updates valve states based on move commands

**Loop Timing**: 10ms (100Hz)

**Key Operations**:
- **Distance Tracking**: `distance += speed * 0.01`
- **Timeout Check**: If timeout exceeded, stop servos
- **Limit Switch Monitoring**: Stops movement if limit switch triggered
- **Valve Control**: Updates valve states via CAN

**Safety Features**:
- Automatic shutdown on timeout
- Limit switch monitoring
- Periodic state reset

## Hydraulic Actuator Control

### MacTrac Configuration

**Actuator Mapping**:
- **Front Actuator**: Valves 0 (UP) and 1 (DOWN)
- **Rear Actuator**: Valves 2 (UP) and 3 (DOWN)
- **Extra Actuator**: Valves 4 (OUT) and 5 (IN)

**Limit Switches**:
- **ADDIO**: Limit switches 0-3
- **IO Board**: Limit switches 0-3

**Behavior**:
- Movement stops when limit switch triggered
- Automatic valve control based on move commands
- Timeout-based safety shutdown

### Standard Configuration

**Actuator Mapping**:
- Similar to MacTrac but with different IO board assignments
- Front: Valves 1 (UP) and 2 (DOWN)
- Rear: Valves 5 (UP) and 6 (DOWN)
- Extra: Valves 4 (OUT) and 3 (IN)

## Speed Sensor Integration

### SERVO_READ Mode

When `SERVO_READ` is defined, the system uses speed sensor data:

**Sensor Data**:
- Reads from `io_board_adc0_cnt`
- Uses pulse timing for speed calculation
- Implements timeout for sensor data

**Timeout Handling**:
- If no valid reading for `SPEED_TIMEOUT_MS` (400ms)
- Resets speed counters
- Sets speed to 0.0

**Speed Calculation**:
- Uses PWM-based speed measurement
- Filters speed values
- Tracks last valid reading time

### Fallback Mode

When no speed sensor is available (`HYDRAULIC_HAS_SPEED_SENSOR` not defined):

**Speed Estimation**:
- Calculated from throttle setting
- `speed = SIGN(throttle) * SPEED_M_S`
- Speed set to 0 when throttle < 0.9

**Distance Tracking**:
- Still maintained based on estimated speed

## Safety Features

### Timeout-Based Shutdown

**Speed Control Timeout** (2.0 seconds):
- If no speed command received for 2 seconds
- Hydraulic servos set to neutral
- Speed counter reset to 0
- Prevents unintended movement

**Move Command Timeout** (10.0 seconds):
- If no move command received for 10 seconds
- Hydraulic actuators stopped
- Prevents actuators from being left extended

### Limit Switch Monitoring

**MacTrac with ADDIO**:
- Monitors limit switches 0-3
- Stops movement when limit switch triggered
- Prevents actuator damage

**Standard with IO Board**:
- Monitors limit switches 0-3
- Stops movement when limit switch triggered
- Prevents actuator damage

### Emergency Stop Support

**Emergency Event Handling**:
- Listens for emergency stop events
- Can terminate hydraulic thread
- Integrated with system emergency stop

## Communication Flow

### Speed Control Path

1. **Application** → Calls `hydraulic_set_speed()`
2. **Function** → Updates throttle setting
3. **Hydraulic Thread** → Reads throttle in main loop
4. **Thread** → Sets servo positions
5. **Servos** → Control hydraulic pumps
6. **Vehicle** → Moves at desired speed

### Distance Tracking Path

1. **Hydraulic Thread** → Reads current speed
2. **Thread** → Calculates distance increment
3. **Thread** → Updates `m_distance_now`
4. **Application** → Calls `hydraulic_get_distance()`
5. **Function** → Returns accumulated distance

### Hydraulic Actuator Path

1. **Application** → Calls `hydraulic_move()`
2. **Function** → Updates move state variable
3. **Hydraulic Thread** → Reads move state
4. **Thread** → Checks limit switches
5. **Thread** → Updates valve states via CAN
6. **Valves** → Control hydraulic actuators

## Thread Interaction

### Thread Communication

**Application → Hydraulic Thread**:
- **Mechanism**: Shared volatile variables
- **Variables**: `m_speed_now`, `m_throttle_set`, `m_move_*`
- **Synchronization**: No explicit synchronization (single writer pattern)

**Hydraulic Thread → Application**:
- **Mechanism**: Shared volatile variables
- **Variables**: `m_speed_now`, `m_distance_now`
- **Synchronization**: Volatile access for thread safety

### Thread Safety

**Read Operations**:
- Safe from any thread context
- Uses volatile variables for atomic access

**Write Operations**:
- Performed only by hydraulic thread
- No concurrent writes needed

**Critical Sections**:
- None (simple variable access)
- Volatile ensures visibility across threads

## Configuration Options

### Vehicle Type

**`IS_MACTRAC`**
- **Effect**: Uses MacTrac-specific configuration
- **Servos**: Single servo (10, 1)
- **Speed**: 1.1 m/s
- **PWM Range**: 1000-2000 μs

**Standard Configuration**
- **Effect**: Uses standard configuration
- **Servos**: Dual servos (1, 2)
- **Speed**: 0.6 m/s
- **PWM Range**: 1000-2100 μs

### Speed Sensor

**`SERVO_READ`**
- **Effect**: Enables speed sensor integration
- **Source**: IO board ADC
- **Timeout**: 400ms for sensor data
- **Accuracy**: High (actual speed measurement)

**Fallback Mode**
- **Effect**: Uses throttle-based speed estimation
- **Accuracy**: Lower (estimated from throttle)

### IO Board Type

**`ADDIO`**
- **Effect**: Uses ADDIO valve controllers
- **Communication**: CAN bus
- **Valves**: 8 valves via CAN
- **Limit Switches**: 8 switches via CAN

**`IO_BOARD`**
- **Effect**: Uses standard IO boards
- **Communication**: CAN bus
- **Valves**: Multiple valves per board
- **Limit Switches**: 8 switches per board

## Usage Examples

### Basic Speed Control
```c
// Initialize hydraulic system
hydraulic_init();

// Drive forward
hydraulic_set_speed(0.5);  // 0.5 m/s forward

// Drive backward
hydraulic_set_speed(-0.3); // 0.3 m/s backward

// Stop
hydraulic_set_speed(0.0);  // Stop

// Get current speed
float speed = hydraulic_get_speed();
```

### Distance Tracking
```c
// Reset distance counter
hydraulic_get_distance(true);

// Drive for 10 meters
hydraulic_set_speed(0.5);

// Wait or perform other operations

// Check distance traveled
float dist = hydraulic_get_distance(false);
if (dist >= 10.0) {
    hydraulic_set_speed(0.0);  // Stop when reached
}
```

### Hydraulic Actuator Control
```c
// Raise front actuator
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);

// Wait for actuator to reach position
chThdSleepMilliseconds(2000);

// Lower front actuator
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_DOWN);

// Stop all actuators
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_STOP);
hydraulic_move(HYDRAULIC_POS_REAR, HYDRAULIC_MOVE_STOP);
hydraulic_move(HYDRAULIC_POS_EXTRA, HYDRAULIC_MOVE_STOP);
```

### Combined Operation
```c
// Drive while controlling actuators
hydraulic_set_speed(0.3);  // Drive forward

// Raise front actuator
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_UP);

// Wait and monitor distance
while (hydraulic_get_distance(false) < 5.0) {
    chThdSleepMilliseconds(100);
}

// Stop and lower actuator
hydraulic_set_speed(0.0);
hydraulic_move(HYDRAULIC_POS_FRONT, HYDRAULIC_MOVE_DOWN);
```

## Performance Considerations

### Thread Performance

**Thread Loop**: 10ms (100Hz)

**CPU Usage**:
- Low (simple calculations)
- Minimal blocking
- Efficient control loop

**Memory Usage**:
- **Stack**: 1024 bytes
- **Variables**: ~50 bytes
- **Total**: ~1KB

### Timing Characteristics

**Control Loop**: 10ms interval
- **Distance Update**: Every 10ms
- **Timeout Check**: Every 10ms
- **Valve Update**: Only when state changes

**Latency**:
- **Speed Control**: < 10ms
- **Actuator Control**: < 10ms
- **Distance Query**: Instant

### Power Consumption

**Servo Control**:
- PWM signals at 50Hz
- Low power consumption
- Efficient hydraulic control

**CAN Communication**:
- Minimal traffic (only on state changes)
- Efficient valve control
- Low bandwidth usage

## Compatibility

### Hardware
- **Servo Motors**: Standard PWM servos
- **Hydraulic Pumps**: Compatible with servo control
- **Valves**: CAN-controlled hydraulic valves
- **IO Boards**: CAN-based IO boards
- **ADDIO**: CAN-based ADDIO controllers

### Software
- **ChibiOS**: Required for threading
- **PWM ESC**: Required for servo control
- **CAN Driver**: Required for valve control
- **Packet Framework**: For CAN communication

## Future Improvements

### Potential Enhancements
1. **Dynamic Speed Control**: Adaptive speed based on load
2. **Position Feedback**: Actual actuator position sensing
3. **Pressure Monitoring**: Hydraulic pressure feedback
4. **Advanced Safety**: Emergency stop with confirmation
5. **Energy Efficiency**: Optimized power usage
6. **Diagnostics**: Comprehensive fault detection

### Current Limitations
1. **No Position Feedback**: Actuator position not monitored
2. **Simple Speed Estimation**: No load compensation
3. **Fixed Timeouts**: Not adaptive to conditions
4. **Basic Safety**: Limited fault detection
5. **No Calibration**: Fixed speed conversion factors

## Troubleshooting

### Common Issues

**Issue**: Vehicle not moving
- **Possible Causes**:
  - Servos not powered
  - Wrong servo assignments
  - PWM range incorrect
  - System not in OPERATIONAL state
- **Solutions**:
  - Check servo power
  - Verify servo assignments
  - Check PWM configuration
  - Verify system state

**Issue**: Hydraulic stops unexpectedly
- **Possible Causes**:
  - Timeout exceeded
  - Limit switch triggered
  - System state changed
- **Solutions**:
  - Check timeout values
  - Verify limit switches
  - Monitor system state

**Issue**: Incorrect speed reading
- **Possible Causes**:
  - Speed sensor not working
  - Wrong speed conversion factor
  - Sensor timeout
- **Solutions**:
  - Check sensor connection
  - Verify speed factor
  - Monitor sensor timeout

**Issue**: Actuators not responding
- **Possible Causes**:
  - CAN communication issue
  - Wrong valve assignments
  - Power issue
- **Solutions**:
  - Check CAN bus
  - Verify valve assignments
  - Check valve power

### Debugging Tips

1. **Check System State**:
   ```c
   if (system_state != SYSTEM_STATE_OPERATIONAL) {
       // System not ready
   }
   ```

2. **Monitor Speed**:
   ```c
   float speed = hydraulic_get_speed();
   commands_printf("Speed: %.2f m/s", speed);
   ```

3. **Check Distance**:
   ```c
   float dist = hydraulic_get_distance(false);
   commands_printf("Distance: %.2f m", dist);
   ```

4. **Verify Servo Positions**:
   ```c
   float left = pwm_esc_get(SERVO_LEFT);
   float right = pwm_esc_get(SERVO_RIGHT);
   commands_printf("Left: %.2f, Right: %.2f", left, right);
   ```

5. **Check Limit Switches**:
   ```c
   #ifdef ADDIO
   bool sw0 = comm_can_addio_lim_sw(0);
   #elif IO_BOARD
   bool sw0 = comm_can_io_board_lim_sw(0);
   #endif
   commands_printf("Limit switch 0: %d", sw0);
   ```

## References

### Related Files
- `hydraulic.h`: Header file with function declarations and types
- `pwm_esc.c`: PWM ESC control for servo motors
- `comm_can.c`: CAN communication for valve control
- `pos.c`: Position estimation and navigation
- `watchdog.c`: System watchdog and emergency stop

### External Dependencies
- **ChibiOS**: Threading and synchronization
- **PWM Driver**: Servo motor control
- **CAN Driver**: Valve control communication
- **IO Board Firmware**: For sensor and valve control
- **ADDIO Firmware**: For valve control

## Conclusion

The hydraulic system module provides comprehensive control for vehicle propulsion and hydraulic actuators. It supports multiple vehicle configurations, integrates with various IO systems, and includes robust safety features. The module's design ensures reliable operation while maintaining flexibility for different hardware setups.

The hydraulic control thread runs efficiently at 100Hz, providing responsive control while maintaining low CPU usage. The system's timeout-based safety features and limit switch monitoring ensure safe operation even in fault conditions.
