# ACK System Issue Analysis

## Problem Summary

The code has problems with settings/uploads but works fine during normal operation (like driving). This is related to the acknowledgment (ACK) system.

## Root Causes

### Issue 1: Incorrect ACK Signal Emission (CRITICAL)

The `processPacket()` function in `packetinterface.cpp` emits the `ackReceived()` signal for **non-ACK commands**:

```cpp
case CMD_AP_ADD_POINTS:
    emit ackReceived(id, cmd, "CMD_AP_ADD_POINTS");
    break;
case CMD_AP_REMOVE_LAST_POINT:
    emit ackReceived(id, cmd, "CMD_AP_REMOVE_LAST_POINT");
    break;
// ... and many more
case CMD_SET_ENU_REF:  // <-- This is NOT an ACK command!
    emit ackReceived(id, cmd, "CMD_SET_ENU_REF");
    break;
```

**Problem**: These commands don't have `_ACK` suffixes in the `CMD_PACKET` enum. When they're received as responses, they emit `ackReceived`, which can:
- Incorrectly trigger ACK waits for unrelated commands
- Cause the wrong command to be acknowledged

**Correct ACK commands** (that should emit `ackReceived`):
- `CMD_SET_POS_ACK`
- `CMD_SET_YAW_OFFSET_ACK`
- `CMD_SET_SYSTEM_TIME_ACK`
- `CMD_REBOOT_SYSTEM_ACK`
- `CMD_MOTE_UBX_START_BASE_ACK`

### Issue 2: Global mWaitingAck Flag Creates Bottleneck

In `sendPacketAck()`:

```cpp
bool PacketInterface::sendPacketAck(...) {
    if (mWaitingAck) {
        qDebug() << "Already waiting for packet";
        return false;  // <-- FAILS if already waiting!
    }
    
    mWaitingAck = true;
    // ... send and wait for ACK ...
    mWaitingAck = false;
}
```

**Problem**: This global flag prevents concurrent ACK waits. If you:
1. Call `setPosAck()` → sets `mWaitingAck = true`
2. Call `setYawOffsetAck()` while waiting → **FAILS IMMEDIATELY**

This creates a bottleneck where only ONE ACK command can be in flight at a time.

### Issue 3: Missing ACK Commands in Enum

Looking at `datatypes.h`, there's no `CMD_SET_ENU_REF_ACK` defined, but the code tries to use it via `setEnuRef()` which calls `sendPacketAck()`.

## Why It Works During Normal Operation

During normal operation (driving), the code uses **non-ACK versions** of the functions:
- `setPos()` instead of `setPosAck()`
- `setYawOffset()` instead of `setYawOffsetAck()`
- `setRcControlCurrent()` etc.

These functions use `sendPacket()` which **doesn't wait for ACK** and doesn't use `mWaitingAck`.

## Why It Fails During Settings/Uploads

When uploading settings, the code likely uses the **ACK versions**:
- `setPosAck()`
- `setYawOffsetAck()`
- `setSystemTime()`
- `sendReboot()`
- `setEnuRef()`

These use `sendPacketAck()` which:
1. Has the global `mWaitingAck` bottleneck
2. Can receive incorrect ACK signals from non-ACK commands

## Recommended Fixes

### Fix 1: Only Emit ackReceived for Actual ACK Commands

In `packetinterface.cpp`, change the `processPacket()` function to only emit `ackReceived` for commands that end with `_ACK`:

```cpp
// Remove these incorrect emissions:
case CMD_AP_ADD_POINTS:
    emit ackReceived(id, cmd, "CMD_AP_ADD_POINTS");  // REMOVE
    break;
case CMD_SET_MAIN_CONFIG:
    emit ackReceived(id, cmd, "CMD_SET_MAIN_CONFIG");  // REMOVE
    break;
case CMD_SET_ENU_REF:
    emit ackReceived(id, cmd, "CMD_SET_ENU_REF");  // REMOVE
    break;
// etc.
```

### Fix 2: Remove Global mWaitingAck Bottleneck

Option A: Remove the check entirely (allows concurrent ACK waits):
```cpp
bool PacketInterface::sendPacketAck(...) {
    // Remove: if (mWaitingAck) { return false; }
    // Remove: mWaitingAck = true;
    // Remove: mWaitingAck = false;
    
    // Use a per-command tracking instead
}
```

Option B: Use a queue system for ACK commands instead of a single flag.

### Fix 3: Add Missing ACK Commands

Add `CMD_SET_ENU_REF_ACK` to the `CMD_PACKET` enum in `datatypes.h`.

## Impact

These issues cause:
- Settings/uploads to fail intermittently
- "Already waiting for packet" errors
- Incorrect ACK matching
- Sequential ACK commands to fail

Normal operation works because it doesn't use the ACK system.
