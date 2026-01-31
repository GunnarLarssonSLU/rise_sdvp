# PWM ESC Control Module Documentation

## Overview

This document provides comprehensive documentation for the PWM ESC control module in the RC_Controller firmware, specifically focusing on the `pwm_esc.c` file.

## File Structure

### Location
- **File**: `pwm_esc.c`
- **Purpose**: Control PWM signals for ESC (Electronic Speed Controllers) and servo motors
- **Architecture**: Timer-based PWM generation with optional speed measurement

## Key Features

1. **PWM Signal Generation**
   - Precise pulse width modulation
   - Configurable update rate (200Hz)
   - Multiple servo channels (up to 4)

2. **Speed Measurement**
   - Tachometer input for speed sensing
   - Pulse timing measurement
   - Speed filtering and averaging

3. **Vehicle Support**
   - MacTrac configuration
   - Standard configuration
   - Configurable wheel diameter

4. **Integration**
   - Works with hydraulic system
   - Supports motor simulation
   - Compatible with various ESC types

## Detailed Documentation

### Configuration Settings

#### PWM Settings

**`ESC_UPDATE_RATE`**: 200 Hz
- **Purpose**: PWM update frequency
- **Effect**: Determines how often PWM signals are updated
- **Trade-off**: Higher rate = smoother control but more CPU usage

**`TIM_CLOCK`**: 5MHz
- **Purpose**: Timer clock frequency
- **Effect**: Determines PWM resolution
- **Resolution**: ~200ns (5MHz clock)

#### Pin Definitions

**Servo Pins**:
- **Servo 1**: GPIOB, Pin 0
- **Servo 2**: GPIOB, Pin 1
- **Servo 3**: GPIOA, Pin 2 (F9 board) or GPIOE, Pin 5 (other)
- **Servo 4**: GPIOA, Pin 3 (F9 board) or GPIOE, Pin ? (other)

**Tachometer Input**:
- **MacTrac**: GPIOA, Pin 2
- **PWM Test**: GPIOA, Pin 2

### Private Variables

#### Speed Measurement

**`new_pulse`**: `volatile bool`
- **Purpose**: Flag indicating new pulse received
- **Usage**: Set by ISR when pulse detected
- **Access**: Read by main thread

**`timer_overflow_count`**: `volatile uint32_t`
- **Purpose**: Counts timer overflows
- **Usage**: Used for pulse timing calculation
- **Access**: Modified by ISR

**`last_capture`**: `volatile uint32_t`
- **Purpose**: Stores last capture value
- **Usage**: Used for pulse timing calculation
- **Access**: Modified by ISR

**`last_tick`**: `systime_t`
- **Purpose**: Last system tick time
- **Usage**: Used for timing calculations
- **Access**: Modified by main thread

#### Speed Filtering

**`time_buffer`**: `float[SPEED_BUFFER_LEN]`
- **Purpose**: Buffer for speed measurements
- **Size**: 6 measurements
- **Usage**: Used for filtering and averaging

**`buf_index`**: `int`
- **Purpose**: Current buffer index
- **Usage**: Tracks position in circular buffer

**`buf_count`**: `int`
- **Purpose**: Number of valid measurements in buffer
- **Usage**: Tracks how many measurements are valid

**`last_valid_time`**: `float`
- **Purpose**: Time of last valid measurement
- **Usage**: Used for timeout detection

**`m_speed_filtered`**: `float`
- **Purpose**: Filtered speed value
- **Usage**: Provides smoothed speed reading

**`m_speed_pwm`**: `float`
- **Purpose**: Raw speed from PWM measurement
- **Usage**: Direct speed reading from tachometer

### Public Functions

#### `pwm_esc_init()`
```c
void pwm_esc_init(void)
```

**Purpose**: Initializes the PWM ESC module

**Steps**:
1. Configures GPIO pins for PWM output
2. Initializes TIM3 for PWM generation
3. Configures PWM channels
4. Sets all servos to neutral position (1.5ms)
5. **MacTrac**: Configures tachometer input
6. **SERVO_READ**: Configures ADC for speed measurement

**Initial State**:
- All servos set to 1.5ms (neutral)
- PWM timer configured
- Speed measurement initialized (if enabled)

#### `pwm_esc_set_all()`
```c
void pwm_esc_set_all(float pulse_width)
```

**Purpose**: Sets all servos to the same pulse width

**Parameters**:
- `pulse_width`: Pulse width in milliseconds (0.5 to 2.5ms typical)

**Behavior**:
1. Truncates pulse width to valid range (0.5 to 2.5ms)
2. Sets all PWM channels to specified width
3. Updates hardware registers

**Usage**:
```c
// Set all servos to neutral
pwm_esc_set_all(1.5);

// Set all servos to full forward
pwm_esc_set_all(2.0);
```

#### `pwm_esc_set()`
```c
void pwm_esc_set(uint8_t channel, float pulse_width)
```

**Purpose**: Sets a specific servo to a pulse width

**Parameters**:
- `channel`: Servo channel (0-3)
- `pulse_width`: Pulse width in milliseconds

**Behavior**:
1. Truncates pulse width to valid range (0.5 to 2.5ms)
2. Sets specified PWM channel to specified width
3. Updates hardware register

**Channels**:
- **0**: Servo 1 (GPIOB, Pin 0)
- **1**: Servo 2 (GPIOB, Pin 1)
- **2**: Servo 3 (GPIOA, Pin 2 or GPIOE, Pin 5)
- **3**: Servo 4 (GPIOA, Pin 3 or GPIOE, Pin ?)

**Usage**:
```c
// Set servo 1 to full forward
pwm_esc_set(0, 2.0);

// Set servo 2 to neutral
pwm_esc_set(1, 1.5);
```

#### `pwm_esc_get()`
```c
float pwm_esc_get(uint8_t channel)
```

**Purpose**: Gets current pulse width for a servo

**Parameters**:
- `channel`: Servo channel (0-3)

**Return**: Current pulse width in milliseconds

**Usage**:
```c
// Get current pulse width for servo 1
float width = pwm_esc_get(0);
commands_printf("Servo 1: %.2f ms", width);
```

### PWM Signal Characteristics

#### Pulse Width Range

**Typical Range**: 0.5ms to 2.5ms

**Standard Mapping**:
- **0.5ms**: Minimum pulse (full reverse)
- **1.5ms**: Neutral position
- **2.5ms**: Maximum pulse (full forward)

**MacTrac Mapping**:
- **1.0ms**: Minimum pulse
- **1.5ms**: Neutral position
- **2.0ms**: Maximum pulse

#### Update Rate

**Frequency**: 200Hz (5ms period)

**Benefits**:
- Smooth control
- Responsive to changes
- Compatible with most ESCs

### Speed Measurement

#### MacTrac Configuration

**Wheel Diameter**: 0.65 meters

**Counts per Revolution**: 16.0

**Calculation**:
```
speed = (wheel_diam * π) / pulse_period
```

#### SERVO_READ Configuration

**Buffer Size**: 6 measurements

**Filtering**: Moving average

**Timeout**: 400ms for valid readings

**Speed Calculation**:
```
speed = wheel_diam * π / pulse_period
```

### Thread Interaction

#### ISR Context

**Pulse Detection ISR**:
- **Trigger**: Tachometer pulse
- **Action**: Updates capture register
- **Signaling**: Sets `new_pulse` flag

**Timer Overflow ISR**:
- **Trigger**: Timer overflow
- **Action**: Increments overflow counter
- **Signaling**: None

#### Main Thread

**Speed Calculation**:
- **Trigger**: Periodic (10ms)
- **Action**: Reads pulse timing
- **Calculation**: Computes speed
- **Filtering**: Applies moving average

### Communication Flow

#### PWM Signal Path

1. **Application** → Calls `pwm_esc_set()`
2. **Function** → Validates pulse width
3. **Function** → Updates PWM register
4. **Timer** → Generates PWM signal
5. **Servo/ESC** → Receives PWM signal

#### Speed Measurement Path

1. **Tachometer** → Generates pulse
2. **ISR** → Captures pulse timing
3. **Main Thread** → Reads capture value
4. **Main Thread** → Calculates speed
5. **Main Thread** → Filters speed
6. **Application** → Reads filtered speed

## Usage Examples

### Basic PWM Control
```c
// Initialize PWM ESC
pwm_esc_init();

// Set all servos to neutral
pwm_esc_set_all(1.5);

// Control individual servos
pwm_esc_set(0, 1.8);  // Servo 1: 80% forward
pwm_esc_set(1, 1.2);  // Servo 2: 60% reverse
```

### Hydraulic Control
```c
// Set hydraulic servos
pwm_esc_set(SERVO_LEFT, 2.0);    // Full forward
pwm_esc_set(SERVO_RIGHT, 0.5);   // Full reverse
```

### Speed Measurement
```c
// Read current speed
float speed = m_speed_filtered;
commands_printf("Speed: %.2f m/s", speed);

// Check if speed is valid
if (buf_count > 0 && chVTTimeElapsedSinceX(last_valid_time) < MS2ST(400)) {
    // Speed is valid
}
```

### Differential Steering
```c
// Left motor forward, right motor reverse
pwm_esc_set(DIFF_STEERING_VESC_LEFT, 2.0);
pwm_esc_set(DIFF_STEERING_VESC_RIGHT, 0.5);

// Both motors forward
pwm_esc_set(DIFF_STEERING_VESC_LEFT, 2.0);
pwm_esc_set(DIFF_STEERING_VESC_RIGHT, 2.0);

// Both motors stop
pwm_esc_set_all(1.5);
```

## Performance Considerations

### PWM Generation

**Timer Configuration**:
- **Clock**: 5MHz
- **Prescaler**: 84
- **Period**: 60000 (for 200Hz)
- **Resolution**: ~200ns

**CPU Usage**:
- **PWM Generation**: Minimal (hardware timer)
- **Speed Measurement**: Low (periodic calculation)
- **Total**: Negligible impact

### Speed Measurement

**Update Rate**: 100Hz (10ms interval)

**CPU Usage**:
- **Calculation**: Simple arithmetic
- **Filtering**: Moving average
- **Total**: Minimal impact

**Accuracy**:
- **Resolution**: Depends on timer clock
- **Filtering**: Reduces noise
- **Timeout**: Detects invalid readings

## Compatibility

### Hardware
- **Microcontroller**: STM32F4 series
- **Timers**: TIM3 for PWM generation
- **GPIO**: Configurable pins for PWM output
- **ADC**: For speed measurement (optional)

### Software
- **ChibiOS**: Required for timing and synchronization
- **HAL**: Required for GPIO and timer configuration
- **STM32 Drivers**: Required for hardware access

## Future Improvements

### Potential Enhancements
1. **Dynamic Update Rate**: Adjust based on requirements
2. **Advanced Filtering**: Kalman filter for speed
3. **Multiple Timers**: Support for more servos
4. **PWM Capture**: For servo position feedback
5. **Fault Detection**: ESC fault monitoring

### Current Limitations
1. **Fixed Update Rate**: Cannot adapt dynamically
2. **Simple Filtering**: Basic moving average
3. **Limited Channels**: 4 PWM channels maximum
4. **No Feedback**: No position or load feedback
5. **Basic Configuration**: Fixed timer settings

## Troubleshooting

### Common Issues

**Issue**: Servos not responding
- **Possible Causes**:
  - Wrong PWM range
  - Timer not configured
  - GPIO pins incorrect
  - Power not connected
- **Solutions**:
  - Verify PWM range (0.5-2.5ms)
  - Check timer configuration
  - Verify GPIO pins
  - Check power connection

**Issue**: Erratic servo behavior
- **Possible Causes**:
  - Noise on PWM signals
  - Wrong update rate
  - Electrical interference
- **Solutions**:
  - Add filtering capacitors
  - Check update rate
  - Verify wiring

**Issue**: Speed measurement not working
- **Possible Causes**:
  - Tachometer not connected
  - Wrong pin configuration
  - Timer not configured
  - No pulses detected
- **Solutions**:
  - Verify tachometer connection
  - Check pin configuration
  - Verify timer settings
  - Check for pulses

**Issue**: PWM glitches
- **Possible Causes**:
  - CPU load too high
  - Timer interrupts blocked
  - Electrical noise
- **Solutions**:
  - Reduce CPU load
  - Check interrupt priorities
  - Add filtering

### Debugging Tips

1. **Check PWM Output**:
   ```c
   // Read back PWM value
   float width = pwm_esc_get(0);
   commands_printf("PWM: %.2f ms", width);
   ```

2. **Monitor Speed**:
   ```c
   // Check speed measurement
   if (buf_count > 0) {
       commands_printf("Speed: %.2f m/s", m_speed_filtered);
   } else {
       commands_printf("No speed data");
   }
   ```

3. **Verify Timer**:
   ```c
   // Check if timer is running
   if (palReadPad(GPIOB, 0) == PAL_HIGH) {
       // PWM is high
   }
   ```

4. **Test with Simple Code**:
   ```c
   // Simple test: set servo to neutral
   pwm_esc_init();
   pwm_esc_set_all(1.5);
   chThdSleepMilliseconds(1000);
   ```

5. **Check Connections**:
   ```c
   // Verify all connections
   // Check power, ground, and signal wires
   // Use multimeter to check signals
   ```

## References

### Related Files
- `pwm_esc.h`: Header file with function declarations
- `hydraulic.c`: Hydraulic system control
- `motor_sim.c`: Motor simulation
- `bldc_interface.c`: BLDC motor control
- `pos.c`: Position control

### External Dependencies
- **ChibiOS**: Timing and synchronization
- **STM32 HAL**: GPIO and timer drivers
- **STM32 CMSIS**: Core peripheral access

### PWM Standards
- **Standard Servo**: 0.5ms to 2.5ms pulses
- **ESC**: Similar to servo but higher current
- **Brushed Motor**: Typically 1ms to 2ms
- **Brushless Motor**: Similar to brushed

## Conclusion

The PWM ESC module provides reliable control for servo motors and ESC devices. Its timer-based architecture ensures precise PWM signal generation with minimal CPU usage. The optional speed measurement feature enables closed-loop control and performance monitoring.

The module's design ensures compatibility with various vehicle configurations and ESC types, making it suitable for different applications within the RC_Controller system. The simple API allows for easy integration with other modules while maintaining precise control over motor and servo operation.
