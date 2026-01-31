# Terminal System Documentation

## Overview

The `terminal.c` module implements a command-line interface for the RC_Controller system. It provides a flexible framework for executing commands, registering custom callbacks, and interacting with the system through a text-based interface.

## System Architecture

```
┌───────────────────────────────────────────────────────────────────────────────┐
│                     TERMINAL SYSTEM                                      │
├───────────────────────────────────────────────────────────────────────────────┤
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                     COMMAND PROCESSING                              │      │
│  │                                                                   │      │
│  │  ┌─────────────┐    ┌───────────────────────────────────────────┐  │      │
│  │  │ Built-in    │    │ Custom Callbacks                            │  │      │
│  │  │ Commands    │    │ (Registered via terminal_register_command_  │  │      │
│  │  └─────────────┘    │ callback)                                   │  │      │
│  │                     └───────────────────────────────────────────┘  │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                               │
│  ┌───────────────────────────────────────────────────────────────────┐      │
│  │                     OUTPUT HANDLING                                 │      │
│  │                                                                   │      │
│  │  ┌─────────────────────────────────────────────────────────────┐  │      │
│  │  │ commands_printf() - Formatted output to terminal            │  │      │
│  │  └─────────────────────────────────────────────────────────────┘  │      │
│  └───────────────────────────────────────────────────────────────────┘      │
│                                                                               │
└───────────────────────────────────────────────────────────────────────────────┘
```

## Key Features

1. **Built-in Commands**: Predefined commands for system monitoring and control
2. **Custom Callbacks**: Extensible framework for adding new commands
3. **Argument Parsing**: Flexible argument handling (up to 64 arguments)
4. **Help System**: Automatic help text generation
5. **Error Handling**: Graceful handling of invalid commands

## Command Processing

### Input Parsing

The terminal system processes commands using the following workflow:

1. **Tokenization**: Input string is split into command and arguments
2. **Command Identification**: Command name is matched against built-in and custom commands
3. **Argument Extraction**: Arguments are extracted and passed to the handler
4. **Execution**: Appropriate handler function is called

**Example:**
```
Input: "setpos 10.5 -3.2 45.0"

Tokenization:
- Command: "setpos"
- Arguments: ["10.5", "-3.2", "45.0"]

Execution:
- Handler: pos_set_xya()
- Parameters: x=10.5, y=-3.2, angle=45.0
```

### Built-in Commands

The system provides several built-in commands:

| Command | Description | Usage |
|---------|-------------|-------|
| `help` | Show help for all commands | `help` |
| `ping` | Test terminal responsiveness | `ping` |
| `mem` | Show memory usage | `mem` |
| `threads` | List all threads | `threads` |
| `vesc` | Forward command to VESC | `vesc command` |
| `reset_att` | Re-initialize attitude estimation | `reset_att` |
| `reset_enu` | Re-initialize ENU reference | `reset_enu` |
| `cc1120_state` | Print CC1120 state | `cc1120_state` |
| `cc1120_update_rf` | Update CC1120 RF settings | `cc1120_update_rf [setting]` |
| `dw_range` | Measure UWB distance | `dw_range [dest]` |
| `dw_ping` | Ping UWB module | `dw_ping` |
| `dw_reboot` | Reboot UWB module | `dw_reboot` |
| `dw_uptime` | Get UWB uptime | `dw_uptime` |
| `zero_gyro` | Zero gyroscope bias | `zero_gyro` |
| `io_board_read` | Read IO board inputs | `io_board_read` |
| `io_board_set_valve` | Set IO board valve | `io_board_set_valve [valve] [state]` |
| `addio_read` | Read ADDIO inputs | `addio_read` |
| `addio_set_valve` | Set ADDIO valve | `addio_set_valve [valve] [state]` |

## Custom Commands

### Command Registration

Custom commands can be registered using `terminal_register_command_callback()`:

```c
/**
 * Register a terminal command callback
 * 
 * @param command Command name (e.g., "my_command")
 * @param help Help text describing the command's purpose
 * @param arg_names Argument names for help display (e.g., "[arg1] [arg2]")
 * @param cbf Callback function to execute when command is called
 */
void terminal_register_command_callback(
    const char* command,
    const char *help,
    const char *arg_names,
    void(*cbf)(int argc, const char **argv)
);
```

### Example: Registering a Custom Command

```c
// Callback function
void my_custom_command(int argc, const char **argv) {
    commands_printf("My custom command called with %d arguments\n", argc);
    
    for (int i = 0; i < argc; i++) {
        commands_printf("Arg %d: %s\n", i, argv[i]);
    }
}

// Register the command
terminal_register_command_callback(
    "my_command",
    "My custom command that does something useful",
    "[arg1] [arg2] [arg3]",
    my_custom_command
);
```

### Command Overwriting

If the same command is registered multiple times, the latest registration overwrites the previous one. The system checks for duplicates by:

1. **Address comparison**: Fast check using string literal addresses
2. **String comparison**: Slower but more thorough check

This allows the same command to be registered with different string literals.

## Callback Management

### Callback Structure

```c
typedef struct _terminal_callback_struct {
    const char *command;      // Command name
    const char *help;         // Help text
    const char *arg_names;    // Argument names
    void(*cbf)(int argc, const char **argv);  // Callback function
} terminal_callback_struct;
```

### Callback Array

- **Maximum callbacks**: 80 (defined by `CALLBACK_LEN`)
- **Storage**: Static array in terminal.c
- **Management**: Circular buffer with write pointer

### Callback Registration Process

1. Check for existing registration (by address or string)
2. Use existing slot if found, or allocate new slot
3. Store command information
4. Increment write pointer if new slot was allocated

## Input Processing

### terminal_process_string()

This function processes terminal input:

```c
void terminal_process_string(char *str);
```

**Processing Steps:**

1. **Tokenization**: Split input string by spaces
2. **Argument Limit**: Maximum 64 arguments
3. **Command Matching**: Check against built-in and custom commands
4. **Execution**: Call appropriate handler
5. **Error Handling**: Display error for invalid commands

**Example Processing:**
```
Input: "my_command arg1 arg2 arg3"

1. Tokenize: ["my_command", "arg1", "arg2", "arg3"]
2. Match: "my_command" found in callbacks
3. Execute: my_callback(4, ["my_command", "arg1", "arg2", "arg3"])
```

## Output Handling

### commands_printf()

The terminal system uses `commands_printf()` for output:

```c
#include "commands.h"

void commands_printf(const char *format, ...);
```

**Features:**
- Formatted output (similar to printf)
- Thread-safe
- Output to terminal interface
- Supports floating-point formatting

**Example:**
```c
commands_printf("Position: x=%.2f, y=%.2f, yaw=%.1f", 
                (double)pos.px, (double)pos.py, (double)pos.yaw);
```

## Help System

### Automatic Help Generation

The help command automatically generates documentation from registered commands:

```
Valid commands are:
help
  Show this help

ping
  Print pong here to see if the reply works

mem
  Show memory usage

threads
  List all threads

vesc
  Forward command to VESC

... (additional built-in commands)

my_command [arg1] [arg2] [arg3]
  My custom command that does something useful
```

### Help Text Format

- **Command name**: First line
- **Help text**: Second line (indented)
- **Arguments**: Shown in brackets after command name

## Error Handling

### Common Error Cases

1. **No Command**: "No command received"
2. **Invalid Command**: No output (command is ignored)
3. **Too Many Arguments**: Extra arguments are ignored
4. **Too Many Callbacks**: "Error: Too many callbacks registered"

### Debugging Tips

1. **Test Connectivity**: Use `ping` command
2. **Check Memory**: Use `mem` command
3. **List Threads**: Use `threads` command
4. **Verify Commands**: Use `help` to list available commands

## Performance Considerations

### Efficiency

- **Tokenization**: Simple strtok() based
- **Command Lookup**: Linear search through callbacks
- **Argument Limit**: 64 arguments (sufficient for most use cases)
- **Callback Limit**: 80 callbacks (configurable via CALLBACK_LEN)

### Optimization Opportunities

1. **Command Hashing**: Use hash table for faster lookup
2. **Binary Search**: Sort commands alphabetically for binary search
3. **Command Caching**: Cache frequently used commands
4. **Lazy Parsing**: Parse arguments only when needed

## Integration with Other Systems

### VESC Communication

The terminal system forwards VESC commands:

```
User Input: "vesc get_values"

Processing:
1. terminal_process_string() identifies "vesc" command
2. Arguments are forwarded to VESC interface
3. VESC response is displayed via commands_printf()
```

### Position System

Position-related commands:
- `reset_att`: Re-initialize attitude estimation
- `reset_enu`: Reset ENU reference frame
- `setpos`: Manually set position

### UWB System

UWB-related commands:
- `dw_range`: Measure distance to UWB module
- `dw_ping`: Test UWB connectivity
- `dw_uptime`: Get UWB module uptime
- `dw_reboot`: Reboot UWB module

### IO Board

IO board commands:
- `io_board_read`: Read all inputs
- `io_board_set_valve`: Control valves

### ADDIO

ADDIO commands:
- `addio_read`: Read all inputs
- `addio_set_valve`: Control valves

## Best Practices

### Command Design

1. **Clear Names**: Use descriptive command names
2. **Consistent Format**: Follow existing naming conventions
3. **Help Text**: Always provide helpful descriptions
4. **Argument Names**: Specify argument names for help display

### Error Handling

1. **Validate Input**: Check argument count and values
2. **Error Messages**: Provide clear error messages
3. **Graceful Degradation**: Handle errors without crashing

### Documentation

1. **Help Text**: Write clear, concise help text
2. **Examples**: Include usage examples in help
3. **Parameters**: Document all parameters

## Examples

### Example 1: Simple Command

```c
void echo_command(int argc, const char **argv) {
    commands_printf("Echo: ");
    for (int i = 1; i < argc; i++) {
        commands_printf("%s ", argv[i]);
    }
    commands_printf("\n");
}

terminal_register_command_callback(
    "echo",
    "Echo back the provided arguments",
    "[text...]",
    echo_command
);
```

### Example 2: Position Query

```c
void pos_query(int argc, const char **argv) {
    POS_STATE pos;
    pos_get_pos(&pos);
    
    commands_printf("Position: x=%.3f, y=%.3f, z=%.3f\n", 
                   (double)pos.px, (double)pos.py, (double)pos.pz);
    commands_printf("Orientation: yaw=%.1f, pitch=%.1f, roll=%.1f\n",
                   (double)pos.yaw, (double)pos.pitch, (double)pos.roll);
    commands_printf("Speed: %.2f m/s\n", (double)pos.speed);
}

terminal_register_command_callback(
    "pos",
    "Query current position and orientation",
    "",
    pos_query
);
```

### Example 3: Configuration Command

```c
void set_steering_center(int argc, const char **argv) {
    if (argc != 2) {
        commands_printf("Usage: set_steering_center [center_value]\n");
        return;
    }
    
    float center;
    if (sscanf(argv[1], "%f", &center) != 1) {
        commands_printf("Invalid center value\n");
        return;
    }
    
    main_config.vehicle.steering_center = center;
    commands_printf("Steering center set to %.2f\n", (double)center);
}

terminal_register_command_callback(
    "set_steering_center",
    "Set steering center position",
    "[center_value]",
    set_steering_center
);
```

## Debugging Commands

### System Monitoring

| Command | Purpose |
|---------|---------|
| `mem` | Check memory usage and fragmentation |
| `threads` | List all active threads with status |
| `ping` | Test terminal responsiveness |

### Sensor Diagnostics

| Command | Purpose |
|---------|---------|
| `zero_gyro` | Calibrate gyroscope bias |
| `reset_att` | Re-initialize attitude estimation |
| `io_board_read` | Monitor IO board inputs |
| `addio_read` | Monitor ADDIO inputs |

### Position Diagnostics

| Command | Purpose |
|---------|---------|
| `reset_enu` | Reset ENU reference frame |
| `setpos` | Manually set position |
| `pos` | Query current position |

### Communication Diagnostics

| Command | Purpose |
|---------|---------|
| `vesc` | Forward commands to VESC |
| `cc1120_state` | Check CC1120 radio state |
| `dw_ping` | Test UWB connectivity |
| `dw_range` | Measure UWB distance |

## Configuration

### Maximum Callbacks

```c
#define CALLBACK_LEN				80
```

This defines the maximum number of custom commands that can be registered.

### Maximum Arguments

```c
enum { kMaxArgs = 64 };
```

This defines the maximum number of arguments that can be passed to a command.

## Thread Safety

The terminal system is designed to be thread-safe:

- **Command Registration**: Thread-safe (no concurrent modification)
- **Command Execution**: Handled by terminal thread
- **Output**: Thread-safe via commands_printf()

## Limitations

1. **Argument Count**: Maximum 64 arguments per command
2. **Callback Count**: Maximum 80 custom commands
3. **String Length**: Limited by static buffer sizes
4. **No Command History**: No persistent command history
5. **No Tab Completion**: No interactive completion

## Future Improvements

1. **Command History**: Persistent command history with up/down arrows
2. **Tab Completion**: Interactive command and argument completion
3. **Command Aliases**: Shortcut names for frequently used commands
4. **Command Groups**: Organize commands into logical groups
5. **Batch Mode**: Execute multiple commands from a file
6. **Scripting**: Support for simple scripting
7. **Remote Access**: Network-based terminal access
8. **Logging**: Automatic command logging for debugging

## References

- **ChibiOS Terminal**: ChibiOS console framework
- **printf Format**: Standard C printf formatting
- **String Tokenization**: strtok() function
- **Command Patterns**: Unix command-line interface patterns

## Code Structure

### Main Functions

- `terminal_process_string()`: Process terminal input
- `terminal_register_command_callback()`: Register custom commands
- `commands_printf()`: Formatted output (in commands.c)

### Callback Functions

- `dw_range_callback()`: UWB range measurement callback
- `dw_ping_callback()`: UWB ping callback
- `dw_uptime_callback()`: UWB uptime callback

### Data Structures

- `terminal_callback_struct`: Command registration structure
- `callbacks[]`: Array of registered commands
- `callback_write`: Current write position

## Data Flow

1. **Input Flow:**
   ```
   User Input → terminal_process_string() → Command Matching → Handler Execution
   ```

2. **Output Flow:**
   ```
   Handler → commands_printf() → Terminal Output
   ```

3. **Registration Flow:**
   ```
   Module Initialization → terminal_register_command_callback() → callbacks[] Array
   ```

## Best Practices for Developers

1. **Command Naming**: Use lowercase with underscores (e.g., `my_command`)
2. **Help Text**: Keep it concise but informative
3. **Error Handling**: Validate all inputs
4. **Output Format**: Use consistent formatting
5. **Testing**: Test commands thoroughly before deployment
6. **Documentation**: Document all custom commands
7. **Consistency**: Follow existing command patterns

## Example Integration

```c
// In your module initialization function
void my_module_init(void) {
    // Register commands
    terminal_register_command_callback(
        "my_command",
        "My module command",
        "[arg1] [arg2]",
        my_command_handler
    );
    
    // Other initialization...
}

// Command handler
void my_command_handler(int argc, const char **argv) {
    // Process arguments
    if (argc < 2) {
        commands_printf("Usage: %s [arg1] [arg2]\n", argv[0]);
        return;
    }
    
    // Do something useful
    float value1, value2;
    if (sscanf(argv[1], "%f", &value1) != 1 || 
        sscanf(argv[2], "%f", &value2) != 1) {
        commands_printf("Invalid arguments\n");
        return;
    }
    
    // Process the values
    float result = process_values(value1, value2);
    
    // Output result
    commands_printf("Result: %.2f\n", (double)result);
}
```

## Troubleshooting

### Common Issues

1. **Command Not Found**: Check spelling and registration
2. **Invalid Arguments**: Verify argument count and types
3. **No Output**: Check if handler is properly registered
4. **Crash on Command**: Add error handling to command handler

### Debugging Steps

1. **List Commands**: Use `help` to verify command registration
2. **Test Simple Commands**: Verify basic commands work
3. **Check Registration**: Ensure callback is registered before use
4. **Add Debug Output**: Add commands_printf() for debugging
5. **Validate Input**: Check argument parsing in handler

## Summary

The terminal system provides a flexible, extensible command-line interface for the RC_Controller system. It supports both built-in and custom commands, making it easy to add new functionality and debugging capabilities. The system is designed to be thread-safe and efficient, with clear error handling and comprehensive help support.
