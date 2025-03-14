# JMC-RC-Library
JMC RC Motor Arduino Library : Arduino Mega compatible, UDP-assigned


# Motor Control System Command Set

## Overview
This project implements a motor control system using Arduino with Ethernet(UDP) communication. It allows precise control of JMC RC stepper motors through UDP commands, supporting various operation modes like position control, velocity control, and homing.

## Features
- UDP-based command interface
- Multiple operation modes (Position, Velocity, Homing)
- Real-time status monitoring
- Configurable acceleration/deceleration profiles
- Emergency stop functionality
- Position and velocity tracking
- Enhanced status reporting in multiple formats
- Motor lock/unlock capability
- Microstepping configuration
- Current control (running and holding)
- Position window settings
- Jog mode for manual movement
- Error reporting and diagnostics
- Motor information readback

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

### Lock Commands (L)
- `L` - Query current lock/brake status
- `L:ON` or `L:1` - Lock motor (engage brake to prevent movement)
- `L:OFF` or `L:0` - Unlock motor (release brake to allow free movement)

The lock commands control the motor's brake. The behavior might seem counterintuitive:

- When `L:ON` is sent, the motor is **locked** (brake engaged) - the motor actively resists movement
- When `L:OFF` is sent, the motor is **unlocked** (brake released) - the motor is free to move/rotate

This is due to the motor's control system where the brake is engaged when the motor is energized.

### Microstepping Commands (MS)
- `MS` - Query current microstepping setting
- `MS:<value>` - Set microstepping resolution (1, 2, 4, 8, 16, or 32)

These commands configure the microstepping resolution of the stepper motor.
Higher resolution provides smoother operation but may reduce torque.

**Example:**
```
MS:16
```
Sets the motor to 1/16 microstepping mode.

### Current Commands (C)
- `C` - Query current settings (running and holding)
- `C:RUN,<percent>` - Set running current as a percentage (0-100)
- `C:HOLD,<percent>` - Set holding current as a percentage (0-100)

These commands control the current supplied to the motor. Lower holding current 
can reduce heat when the motor is stationary.

**Example:**
```
C:RUN,80
C:HOLD,40
```
Sets running current to 80% and holding current to 40%.

### Position Window Commands (PW)
- `PW` - Query position window settings
- `PW:<value>,<time_ms>` - Set position window and time

These commands configure when a position is considered "reached" by specifying 
the acceptable error and time within that error.

**Example:**
```
PW:10,500
```
Sets position window to ±10 units with 500ms time requirement.

### Jog Commands (J)
- `J:CW,<steps>,<speed>` - Jog clockwise by specified steps
- `J:CCW,<steps>,<speed>` - Jog counter-clockwise by specified steps
- `J:STOP` - Stop jog movement

These commands provide manual incremental movement at a specific speed.
Useful for testing and manual positioning.

**Example:**
```
J:CW,100,5.0
```
Jogs the motor 100 steps clockwise at a velocity of 5.0.

### Zero Commands (Z)
- `Z` - Set current position as zero
- `Z:OFS,<offset>` - Move to offset position and set as zero
- `Z:SET,<position>` - Set current position to specified value without moving

These commands allow you to set the position counter to specific values without 
physically moving the motor. This is useful for calibration and establishing 
reference points.

**Examples:**
```
Z
```
Sets the current position as zero.

```
Z:SET,1000
```
Sets the current position to 1000 without moving the motor.

### Motor Information Commands (MI)
- `MI` - Display comprehensive motor configuration information

Returns motor type, microstepping settings, current settings,
position window settings, and other configuration parameters.

### Error Commands (E)
- `E` - Display current error code and description
- `E:HISTORY` - Display alarm history (last 5 alarms)

Provides detailed error diagnostics for troubleshooting.

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

### Reset Commands (R)
- `R` - Reset motor and clear alarms

### Info Commands (I)
- `I` - Display comprehensive motor information

### System Reset (X)
- `X` - Reset the Arduino controller

## Motor Hardware Connections

The system is designed to work with JMC IHSS86 series integrated RC stepper motors using Modbus RTU over RS485.

### Wiring Diagram
```
Arduino Mega         IHSS86 RC Motor
--------------       ---------------
Pin 16 (TX2) ------> RS485 RX
Pin 17 (RX2) <------ RS485 TX
Pin 9 (DE/RE) -----> RS485 DE/RE
GND ------------- GND
```

### Network Configuration
The default network settings for the Arduino Ethernet shield are:
- IP Address: 192.168.1.2
- Remote Server: 192.168.1.100
- Local Port: 8002
- Remote Port: 8000

## Microstepping and Step Count

For the IHSS86 RC stepper motor with a standard 1.8° step angle:

| Microstepping | Total Steps for 360° | Step Size |
|---------------|----------------------|-----------|
| Full Step (1) | 200 | 1.8° |
| Half Step (2) | 400 | 0.9° |
| Quarter Step (4) | 800 | 0.45° |
| Eighth Step (8) | 1,600 | 0.225° |
| 1/16 Step (16) | 3,200 | 0.1125° |
| 1/32 Step (32) | 6,400 | 0.05625° |

## Installation
1. Install required libraries:
   ```cpp
   #include <SPI.h>
   #include "JMCMotor.h"
   #include <Ethernet.h>
   #include <EthernetUdp.h>
   #include <ModbusRTUMaster.h>
   ```

2. Upload the JMC_Lib_test.ino sketch to your Arduino Mega

3. Connect the hardware according to the wiring diagram

4. Ensure your PC is on the same network as the Arduino (default: 192.168.1.x)

5. Use a UDP client (like NetCat, Hercules, or a custom application) to send commands

## Usage Example
```
// Initialize motor
motor.Setup(47, 47);

// Set microstepping resolution
Send: "MS:16"
Response: "Microstepping set to: Sixteenth step (1/16)"

// Set motor currents
Send: "C:RUN,80"
Response: "Running current set to 80%"
Send: "C:HOLD,40"
Response: "Holding current set to 40%"

// Jog the motor
Send: "J:CW,100,5.0"
Response: "Jogging CW 100 steps at speed 5.0"

// Set current position as a reference point
Send: "Z:SET,1000"
Response: "Current position set to: 1000"

// Move to absolute position
Send: "P:ABS,2000,5.0"
Response: "Moving to absolute position 2000 at velocity 5.0"

// Check motor information
Send: "MI"
Response: 
"Motor Information:
Type: IHSS86 Series
Microstepping: 1/16
Running Current: 80.0%
Holding Current: 40.0%
Position Window: 10 units
Position Window Time: 500ms
Actual Mode: 1"
```

## Troubleshooting
- If motor doesn't respond, check the Modbus address (default is 1) and RS485 wiring
- Use the status commands (S:DETAIL) to check for errors or warnings
- The error command (E) can provide more detailed diagnostic information
- If the motor is not moving as expected, check the current settings and microstepping configuration
- Use the debug serial output for monitoring communication

## Notes
This is a personal project. Usage and distribution are restricted unless explicitly permitted by the author.
