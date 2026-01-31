# Autopilot System Documentation

## Overview

The `autopilot.c` module implements a waypoint-based navigation system for autonomous vehicles. It uses a pure pursuit algorithm to guide the vehicle along a predefined route with support for dynamic look-ahead distance, speed control, and differential steering.

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                     AUTOPILOT SYSTEM                                     │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                     CONTROL THREAD (100Hz)                          │      │
│  │                                                                   │      │
│  │  ┌─────────────────────────────────────────────────────────────┐  │      │
│  │  │ Pure Pursuit Algorithm                                        │  │      │
│  │  │                                                                 │  │      │
│  │  │  1. Find closest route point                                 │  │      │
│  │  │  2. Calculate look-ahead point                                │  │      │
│  │  │  3. Compute turn radius                                        │  │      │
│  │  │  4. Update steering and speed                                  │  │      │
│  │  └─────────────────────────────────────────────────────────────┘  │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                     ROUTE MANAGEMENT                                │      │
│  │                                                                   │      │
│  │  ┌─────────────────────────────────────────────────────────────┐  │      │
│  │  │ Route Points (AP_ROUTE_SIZE = 200)                           │  │      │
│  │  │                                                                 │  │      │
│  │  │  - Position (x, y)                                             │  │      │
│  │  │  - Speed limit                                                 │  │      │
│  │  │  - Attributes (flags)                                          │  │      │
│  │  └─────────────────────────────────────────────────────────────┘  │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                     VEHICLE CONTROL                                │      │
│  │                                                                   │      │
│  │  ┌─────────────────────────────────────────────────────────────┐  │      │
│  │  │ Steering Control                                             │  │      │
│  │  │                                                                 │  │      │
│  │  │  - Turn radius calculation                                    │  │      │
│  │  │  - Pure pursuit look-ahead distance                           │  │      │
│  │  │  - Differential steering support                              │  │      │
│  │  └─────────────────────────────────────────────────────────────┘  │      │
│  │                                                                   │      │
│  │  ┌─────────────────────────────────────────────────────────────┐  │      │
│  │  │ Speed Control                                                 │  │      │
│  │  │                                                                 │  │      │
│  │  │  - Route point speed limits                                   │  │      │
│  │  │  - Speed override                                              │  │      │
│  │  │  - Dynamic speed adjustment                                    │  │      │
│  │  └─────────────────────────────────────────────────────────────┘  │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

## Pure Pursuit Algorithm

The autopilot uses a pure pursuit algorithm to follow the route:

1. **Find Closest Point**: Identify the route point closest to current position
2. **Look-Ahead Point**: Select a target point based on look-ahead distance
3. **Turn Radius**: Calculate required turn radius to reach target point
4. **Steering**: Set vehicle steering based on turn radius

### Look-Ahead Distance

The look-ahead distance determines how far ahead the vehicle "looks" when calculating the turn:

```
look_ahead_distance = m_route_look_ahead * vehicle_speed
```

**Default**: 8 meters (configurable via `m_route_look_ahead`)

### Turn Radius Calculation

The turn radius is calculated based on the vehicle's position relative to the look-ahead point:

```
turn_radius = distance_to_look_ahead_point / sin(angle_to_look_ahead_point)
```

## Route Management

### Route Structure

```c
typedef struct {
    float x, y;           // Position in local ENU frame
    float speed;          // Speed limit at this point
    uint32_t attributes; // Flags (e.g., sharp turn, stop)
} ROUTE_POINT;
```

### Route Storage

- **Maximum points**: AP_ROUTE_SIZE (200)
- **Storage**: Static array `m_route[AP_ROUTE_SIZE]`
- **Access**: Thread-safe via mutex `m_ap_lock`

### Route Navigation

1. **Current Point**: `m_point_now` - Current target point
2. **Last Point**: `m_point_last` - Last point in route
3. **Route Started**: `m_is_route_started` - Route initialization flag
4. **Route End**: `m_route_end` - Route completion flag

## Control Loop

### Frequency

- **Control frequency**: 100Hz (10ms interval)
- **Timer**: CH_CFG_ST_FREQUENCY / AP_HZ

### Control Sequence

1. **Check Active**: Verify autopilot is active
2. **Find Closest Point**: Locate nearest route point
3. **Calculate Look-Ahead**: Determine target point
4. **Compute Turn Radius**: Calculate required turn
5. **Update Steering**: Set vehicle steering
6. **Update Speed**: Adjust vehicle speed
7. **Advance Route**: Move to next point if needed

## Steering Control

### Differential Steering

For vehicles with independent left/right wheels:

```c
#if HAS_DIFF_STEERING
    // Calculate turn radius for differential steering
    m_turn_rad_now = ...;
    
    // Set left and right wheel speeds based on turn radius
    float left_speed = base_speed * (1 - turn_radius_compensation);
    float right_speed = base_speed * (1 + turn_radius_compensation);
#endif
```

### Pure Pursuit Steering

For standard vehicles:

```c
// Calculate steering angle based on turn radius
float steering_angle = atan(vehicle_axis_distance / turn_radius);

// Set servo position
servo_simple_set_pos_ramp(steering_angle_normalized, false);
```

## Speed Control

### Speed Limits

1. **Route Point Speed**: Speed limit at each route point
2. **Override Speed**: Manual speed override
3. **Dynamic Speed**: Speed adjusted based on turn radius

### Speed Override

```c
if (m_is_speed_override) {
    // Use override speed
    target_speed = m_override_speed;
} else {
    // Use route point speed or dynamic speed
    target_speed = route_point_speed;
}
```

### Dynamic Speed Adjustment

Speed is reduced for sharp turns based on turn radius:

```c
if (turn_radius < min_turn_radius) {
    // Reduce speed for sharp turns
    target_speed = target_speed * (turn_radius / min_turn_radius);
}
```

## Terminal Commands

The autopilot provides several terminal commands for debugging and control:

### Autopilot State Commands

| Command | Description | Usage |
|---------|-------------|-------|
| `ap_state` | Print autopilot state | `ap_state` |
| `ap_print_closest` | Print distance to closest point | `ap_print_closest [rate]` |
| `ap_dynamic_rad` | Enable/disable dynamic radius | `ap_dynamic_rad [0/1]` |
| `ap_ang_dist_comp` | Enable/disable angle-distance compensation | `ap_ang_dist_comp [0/1]` |
| `ap_look_ahead` | Set look-ahead distance | `ap_look_ahead [distance]` |

### Route Control Commands

| Command | Description | Usage |
|---------|-------------|-------|
| `ap_start` | Start autopilot | `ap_start` |
| `ap_stop` | Stop autopilot | `ap_stop` |
| `ap_reset` | Reset autopilot state | `ap_reset` |
| `ap_speed_override` | Override speed | `ap_speed_override [speed]` |
| `ap_speed_override_disable` | Disable speed override | `ap_speed_override_disable` |

### Route Management Commands

| Command | Description | Usage |
|---------|-------------|-------|
| `ap_route_start` | Start route following | `ap_route_start` |
| `ap_route_end` | End route following | `ap_route_end` |
| `ap_route_reset` | Reset route | `ap_route_reset` |
| `ap_route_sync` | Synchronize route | `ap_route_sync` |

## Configuration Parameters

### Main Configuration (conf_general.h)

```c
// Autopilot parameters
#define AP_ROUTE_SIZE               200     // Maximum route points
#define AP_HZ                       100     // Control frequency (Hz)
#define AP_LOOK_AHEAD               8.0     // Look-ahead distance (m)
#define AP_MIN_TURN_RADIUS          5.0     // Minimum turn radius (m)
#define AP_SHARP_TURN_ANGLE         30.0    // Sharp turn angle (degrees)
#define AP_SHARP_TURN_RADIUS        3.0     // Sharp turn radius (m)

// Vehicle parameters
#define VEHICLE_AXIS_DISTANCE        0.5     // Wheelbase (m)
#define VEHICLE_STEERING_CENTER     0.5     // Steering center position
#define VEHICLE_STEERING_RANGE      1.0     // Steering range
#define VEHICLE_STEERING_MAX_ANGLE  45.0    // Maximum steering angle (degrees)
```

## Thread Safety

The autopilot system uses mutexes to ensure thread-safe access:

- **Mutex**: `m_ap_lock`
- **Protection**: All route and state variables
- **Locking**: Held during control loop execution

## Error Handling

### Common Error Cases

1. **Empty Route**: No route points defined
2. **Invalid Point**: Route point out of bounds
3. **No Closest Point**: Vehicle outside route area
4. **Route Wrapping**: Route points wrap around

### Recovery Strategies

1. **Reset State**: `reset_state()` function
2. **Stop Autopilot**: `autopilot_stop()` function
3. **Clear Route**: Reset route points

## Performance Characteristics

| Parameter | Value |
|-----------|-------|
| Control Frequency | 100Hz |
| Look-Ahead Distance | 8m (configurable) |
| Route Points | 200 |
| Update Rate | 10ms |
| Steering Update | Continuous |
| Speed Update | Continuous |

## Algorithm Details

### Finding Closest Point

The algorithm finds the route point closest to the vehicle's current position:

```c
// Calculate distance to each route point
for (int i = 0; i < len; i++) {
    float dx = m_route[i].x - pos.px;
    float dy = m_route[i].y - pos.py;
    float dist = sqrt(dx*dx + dy*dy);
    
    if (dist < min_dist) {
        min_dist = dist;
        closest_point = i;
    }
}
```

### Calculating Look-Ahead Point

The look-ahead point is selected based on distance from the closest point:

```c
// Calculate look-ahead distance based on speed
float look_ahead_dist = m_route_look_ahead * pos.speed;

// Find point at look-ahead distance
int look_ahead_point = closest_point;
while (look_ahead_point < len) {
    float dx = m_route[look_ahead_point].x - m_route[closest_point].x;
    float dy = m_route[look_ahead_point].y - m_route[closest_point].y;
    float dist = sqrt(dx*dx + dy*dy);
    
    if (dist >= look_ahead_dist) {
        break;
    }
    look_ahead_point++;
}
```

### Computing Turn Radius

The turn radius is calculated using the pure pursuit formula:

```c
// Calculate vector from vehicle to look-ahead point
float dx = m_route[look_ahead_point].x - pos.px;
float dy = m_route[look_ahead_point].y - pos.py;
float dist = sqrt(dx*dx + dy*dy);

// Calculate angle to look-ahead point
float angle = atan2(dy, dx) - pos.yaw * M_PI / 180.0;

// Calculate turn radius
m_rad_now = dist / sin(angle);
```

## Integration with Other Systems

### Position System

The autopilot relies on position data from the position system:
- Current position (x, y, yaw)
- Vehicle speed
- Orientation (quaternions)

### Servo System

The autopilot controls vehicle steering via the servo system:
- Sets target steering position
- Uses servo_simple_set_pos_ramp() for smooth transitions
- Supports differential steering for dual-motor vehicles

### Vehicle Control

The autopilot sets vehicle speed and steering:
- **Steering**: Turn radius → steering angle
- **Speed**: Route point speed or override
- **Acceleration**: Smooth speed transitions

## Best Practices

### Route Design

1. **Point Spacing**: Keep points 1-5 meters apart
2. **Turn Radius**: Ensure minimum turn radius is respected
3. **Speed Limits**: Set appropriate speed limits for turns
4. **Smooth Transitions**: Avoid sharp angle changes between points

### Tuning Parameters

1. **Look-Ahead Distance**: Start with 8m, adjust based on speed
2. **Turn Radius**: Ensure vehicle can physically achieve the radius
3. **Speed Limits**: Set conservative limits for safety
4. **Control Frequency**: 100Hz is sufficient for most applications

### Testing

1. **Static Test**: Verify steering at fixed position
2. **Straight Line**: Test straight line following
3. **Curves**: Test various curve radii
4. **Speed Changes**: Test speed limit transitions
5. **Emergency Stop**: Test emergency stop functionality

## Examples

### Example 1: Simple Route

```c
// Define a simple rectangular route
ROUTE_POINT route[4] = {
    {0.0, 0.0, 1.0, 0},      // Start point
    {10.0, 0.0, 1.0, 0},     // Move forward
    {10.0, 10.0, 0.5, 0},    // Turn left (slower)
    {0.0, 10.0, 1.0, 0}      // Complete rectangle
};

// Load route
for (int i = 0; i < 4; i++) {
    autopilot_set_route_point(i, &route[i]);
}

// Start autopilot
autopilot_start();
```

### Example 2: Dynamic Speed Adjustment

```c
// Enable dynamic speed adjustment
m_en_dynamic_rad = true;

// Set minimum turn radius
float min_turn_radius = 5.0;

// In control loop:
if (m_rad_now < min_turn_radius) {
    // Reduce speed for sharp turns
    float speed_reduction = 1.0 - (min_turn_radius - m_rad_now) / min_turn_radius;
    target_speed = route_speed * (1.0 - speed_reduction);
}
```

### Example 3: Differential Steering

```c
#if HAS_DIFF_STEERING
    // Calculate turn radius
    m_turn_rad_now = calculate_turn_radius();
    
    // Set left and right wheel speeds
    float base_speed = get_target_speed();
    float turn_factor = vehicle_axis_distance / m_turn_rad_now;
    
    float left_speed = base_speed * (1.0 - turn_factor);
    float right_speed = base_speed * (1.0 + turn_factor);
    
    // Apply speeds to motors
    set_left_motor_speed(left_speed);
    set_right_motor_speed(right_speed);
#endif
```

## Troubleshooting

### Common Issues

1. **Vehicle Not Following Route**: Check position data accuracy
2. **Oscillating Steering**: Reduce look-ahead distance
3. **Slow Speed**: Check speed limits and overrides
4. **Route Not Advancing**: Verify point spacing and speed
5. **Sharp Turns**: Increase minimum turn radius

### Debugging Steps

1. **Check Position**: Verify position data is accurate
2. **Monitor State**: Use `ap_state` to monitor autopilot state
3. **Visualize Route**: Plot route points and vehicle position
4. **Adjust Parameters**: Tune look-ahead distance and turn radius
5. **Test Components**: Verify steering and speed control separately

## References

- **Pure Pursuit Algorithm**: https://www.ri.cmu.edu/pub_files/pub3/cowley_jeffrey_1988_1/cowley_jeffrey_1988_1.pdf
- **Waypoint Navigation**: https://www.researchgate.net/publication/224192566_A_Review_of_Waypoint_Navigation_and_Control_Algorithms_for_Autonomous_Underwater_Vehicles
- **Path Planning**: https://www.cs.cmu.edu/~motionplanning/

## Future Improvements

1. **Predictive Look-Ahead**: Use vehicle dynamics for better prediction
2. **Adaptive Look-Ahead**: Adjust look-ahead based on vehicle speed
3. **Obstacle Avoidance**: Integrate with obstacle detection
4. **Dynamic Route Adjustment**: Modify route based on real-time conditions
5. **Energy Optimization**: Optimize for energy efficiency
6. **Terrain Adaptation**: Adapt control based on terrain type
7. **Multi-Sensor Fusion**: Use multiple sensors for better positioning
8. **Machine Learning**: Learn optimal control parameters

## Code Structure

### Main Functions

- `autopilot_init()`: System initialization
- `autopilot_start()`: Start autopilot
- `autopilot_stop()`: Stop autopilot
- `autopilot_reset()`: Reset autopilot state
- `autopilot_set_route_point()`: Set route point
- `autopilot_get_state()`: Get autopilot state

### Control Functions

- `ap_thread()`: Main control thread
- `find_closest_point()`: Find closest route point
- `calculate_look_ahead()`: Calculate look-ahead point
- `calculate_turn_radius()`: Calculate turn radius
- `update_steering()`: Update vehicle steering
- `update_speed()`: Update vehicle speed

### Terminal Commands

- `terminal_state()`: Print autopilot state
- `terminal_print_closest()`: Print closest point info
- `terminal_dynamic_rad()`: Toggle dynamic radius
- `terminal_angle_dist_comp()`: Toggle angle-distance compensation
- `terminal_look_ahead()`: Set look-ahead distance

### Utility Functions

- `reset_state()`: Reset autopilot state
- `advance_route()`: Advance to next route point
- `check_route_end()`: Check if route is completed

## Data Flow

1. **Position Data Flow:**
   ```
   Position System → ap_thread() → Closest Point Calculation → Look-Ahead Point → Turn Radius
   ```

2. **Control Flow:**
   ```
   ap_thread() → Update Steering → servo_simple_set_pos_ramp() → Vehicle Steering
   ```

3. **Speed Flow:**
   ```
   ap_thread() → Update Speed → Vehicle Control → Vehicle Speed
   ```

4. **Route Flow:**
   ```
   Route Points → find_closest_point() → calculate_look_ahead() → calculate_turn_radius() → Steering Command
   ```

## Summary

The autopilot system provides a robust waypoint-based navigation solution for autonomous vehicles. It uses a pure pursuit algorithm for smooth path following, with support for dynamic speed control, differential steering, and comprehensive debugging capabilities. The system is designed to be flexible and configurable, allowing it to adapt to various vehicle types and route requirements.
