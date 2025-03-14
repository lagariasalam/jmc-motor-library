# JMC-RC-Library
JMC RC Motor Arduino Library : Arduino Mega compatible, UDP-assigned


# Motor Control System Command Set

## Overview
This project implements a motor control system using Arduino with Ethernet(UDP) communication. It allows precise control of motors through UDP commands, supporting various operation modes like position control, velocity control, and homing.

## Features
- UDP-based command interface
- Multiple operation modes (Position, Velocity, Homing)
- Real-time status monitoring
- Configurable acceleration/deceleration profiles
- Emergency stop functionality
- Position and velocity tracking
- Enhanced status reporting in multiple formats

## Command Set Reference

### Position Commands (P)
- `P:?` - Query current position
- `P:VEL,<value>` - Set default velocity
- `P:ABS,<pos>,<vel>` - Move to absolute position
- `P:REL,<pos>,<vel>` - Move relative position

### Velocity Commands (V)
- `V:?` - Query current velocity
- `V:PRF,<vel>,<acc>,<dec>` - Set velocity profile
- `V:<value>` - Set velocity directly

### Homing Commands (H)
- `H` - Start default homing
- `H,<method>,<offset>,<speed>` - Custom homing

### Brake Commands (B)
- `B` - Normal stop
- `B,1` - Quick stop with deceleration
- `B,2` - Fast quick stop
- `B,3` - Immediate stop

### Status Commands (S)
- `S` - Get raw status value (returns both decimal and hex format)
- `S:DETAIL` - Get detailed human-readable status information
- `S:RAW` - Get raw status word only
- `S:FLAGS` - Get detailed breakdown of each status bit
- `S:COMPACT` - Get compact status format for efficient monitoring

#### Status Response Formats

**Raw Status Response:**
```
STATUS:16387|0x4003
```
Format: `STATUS:<decimal>|0x<hexadecimal>`

**Detailed Status Response:**
```
Motor Status (0x4003):
Operation: RUNNING
Position: TARGET REACHED
HOMING COMPLETE
Current Position: 1000
Current Velocity: 0.0
```

**Flags Status Response:**
```
Status Flags (0x4003):
Bit 0 (Ready to init): 1
Bit 1 (Init complete): 1
...
Bit 15 (CCW limit): 0
```

**Compact Status Response:**
```
M1:RT:1000:1
```
Format: `M<id>:<state><flags>:<position>:<mode>`

Where:
- `<state>` can be: E (Error), R (Running), H (Halted), S (Standby), D (Disabled)
- `<flags>` may include: W (Warning), T (Target reached), O (Origin/Home), C (CW limit), A (CCW limit)
- `<mode>` is the current operation mode: 1 (Position), 3 (Velocity), 6 (Homing)

### Mode Commands (M)
- `M` - Query current operation mode
- `M,1` - Set to Position mode
- `M,3` - Set to Velocity mode
- `M,6` - Set to Homing mode

### Acceleration Commands (A)
- `A,<acc>,<dec>` - Set acceleration and deceleration values

### Quick Stop Commands (Q)
- `Q` - Default quick stop (deceleration based)
- `Q,1` - Quick stop with deceleration
- `Q,2` - Fast quick stop
- `Q,3` - Immediate stop

### Zero Commands (Z)
- `Z` - Set current position as zero
- `Z:OFS,<offset>` - Move to offset position and set as zero

### Reset Commands (R)
- `R` - Reset motor and clear alarms

### Info Commands (I)
- `I` - Display comprehensive motor information

### System Reset (X)
- `X` - Reset the Arduino controller

## Installation
1. Install required libraries:
   ```cpp
   #include <SPI.h>
   #include "JMCMotor.h"
   #include <Ethernet.h>
   #include <EthernetUdp.h>
   #include <ModbusRTUMaster.h>
   ```

## Usage Example
```
// Initialize motor
motor.Setup(47, 47);

// Move to absolute position
Send: "P:ABS,1000,5.0"
Response: "Moving to absolute position 1000 at velocity 5.0"

// Query current position
Send: "P:?"
Response: "P: 1000"

// Get detailed status
Send: "S:DETAIL"
Response: "Motor Status (0x4003):
Operation: RUNNING
Position: TARGET REACHED
HOMING COMPLETE
Current Position: 1000
Current Velocity: 0.0"
```

## Notes
This is a personal project. Usage and distribution are restricted unless explicitly permitted by the author.
