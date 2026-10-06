# ACK System Issue - Final Analysis

## Problem Statement
Settings/uploads have problems, but normal operation (like driving) works fine. Suspected to be related to ack-s (acknowledgment system).

## Root Cause

**The global `mWaitingAck` flag in `PacketInterface::sendPacketAck()` creates a bottleneck that prevents concurrent ACK operations.**

### How It Manifests

When uploading settings, the code calls functions that use `sendPacketAck()`:
- `setEnuRef(quint8 id, double *llh, int retries)` - sends `CMD_SET_ENU_REF`
- `setSystemTime(quint8 id, qint32 sec, qint32 usec, int retries)` - sends `CMD_SET_SYSTEM_TIME`
- `sendReboot(quint8 id, bool powerOff, int retries)` - sends `CMD_REBOOT_SYSTEM`
- `setPosAck(quint8 id, double x, double y, double angle, int retries)` - sends `CMD_SET_POS_ACK`
- `setYawOffsetAck(quint8 id, double angle, int retries)` - sends `CMD_SET_YAW_OFFSET_ACK`

All of these functions call `sendPacketAck()`, which has this code:

```cpp
bool PacketInterface::sendPacketAck(...) {
    if (mWaitingAck) {
        qDebug() << "Already waiting for packet";
        return false;  // <-- FAILS HERE if already waiting!
    }
    
    mWaitingAck = true;
    // ... send packet and wait for ACK ...
    mWaitingAck = false;
}
```

**If you call any two of these functions in sequence, the second one will FAIL with "Already waiting for packet"** because `mWaitingAck` is still true from the first call.

### Why Normal Operation Works

During normal operation (driving), the code uses **non-ACK versions** of the functions:
- `setPos(quint8 id, double x, double y, double angle)` - uses `sendPacket()`, no ACK wait
- `setYawOffset(quint8 id, double angle)` - uses `sendPacket()`, no ACK wait
- `setRcControlCurrent(quint8 id, double current, double steering)` - uses `sendPacket()`, no ACK wait

These functions use `sendPacket()` which does NOT use `mWaitingAck`, so they can be called concurrently without issues.

### Evidence from Code

In `chronos.cpp` line 196:
```cpp
mPacket->setEnuRef(255, mLlhRef);
```

This is called when processing OSEM messages. If another setting is uploaded at the same time, it will fail.

## Secondary Issue: Incorrect ACK Signal Emission

The `processPacket()` function emits `ackReceived` for regular commands that are not actually ACK commands:

```cpp
case CMD_SET_ENU_REF:
    emit ackReceived(id, cmd, "CMD_SET_ENU_REF");  // Wrong! This is not an ACK command
    break;
case CMD_SET_MAIN_CONFIG:
    emit ackReceived(id, cmd, "CMD_SET_MAIN_CONFIG");  // Wrong!
    break;
// ... and others
```

This can cause:
- A response for one command to incorrectly trigger ACK completion for a different command
- Race conditions in the ACK wait logic

However, this appears to be **intentional** - the car firmware sends back the same command as an acknowledgment. So when you send `CMD_SET_ENU_REF`, the car responds with `CMD_SET_ENU_REF` to acknowledge it.

## The Fix

### Primary Fix (Required)
Remove the global `mWaitingAck` flag from `sendPacketAck()` in `packetinterface.cpp`:

```cpp
bool PacketInterface::sendPacketAck(const unsigned char *data, unsigned int len_packet,
                                    int retries, int timeoutMs) {
    // REMOVE: if (mWaitingAck) { return false; }
    // REMOVE: mWaitingAck = true;
    
    unsigned char *buffer = new unsigned char[len_packet];
    bool ok = false;
    memcpy(buffer, data, len_packet);

    for (int i = 0;i < retries;i++) {
        QEventLoop loop;
        QTimer timeoutTimer;
        timeoutTimer.setSingleShot(true);
        timeoutTimer.start(timeoutMs);
        connect(this, SIGNAL(ackReceived(quint8, CMD_PACKET, QString)), &loop, SLOT(quit()));
        connect(&timeoutTimer, SIGNAL(timeout()), &loop, SLOT(quit()));

        QTimer::singleShot(0, [this, buffer, len_packet]() {
            sendPacket(buffer, len_packet);
        });

        loop.exec();

        if (timeoutTimer.isActive()) {
            ok = true;
            break;
        }

        qDebug() << "Retrying to send packet...";
    }

    // REMOVE: mWaitingAck = false;
    delete[] buffer;
    return ok;
}
```

Also remove the `mWaitingAck` member variable from `packetinterface.h`.

### Secondary Fix (Recommended)
Clean up the `ackReceived` emissions to only emit for actual ACK commands:

```cpp
// In processPacket(), change from:
case CMD_SET_ENU_REF:
    emit ackReceived(id, cmd, "CMD_SET_ENU_REF");
    break;

// To:
case CMD_SET_ENU_REF:
    // Don't emit ackReceived - this is not an ACK command
    break;
```

But note: This might break things if the car firmware is designed to send back the same command as ACK. In that case, keep the emissions but understand that it's by design.

## Impact

After the fix:
- ✅ Settings/uploads will work reliably even when called in sequence
- ✅ No more "Already waiting for packet" errors
- ✅ Normal operation continues to work as before
- ⚠️ Multiple ACK commands can now be in flight concurrently (this is good, but ensure the car firmware can handle it)

## Testing

After applying the fix, test:
1. Upload multiple settings in sequence (ENU ref, system time, etc.)
2. Verify all settings are applied correctly
3. Test normal driving operation
4. Test concurrent operations (upload settings while driving)
