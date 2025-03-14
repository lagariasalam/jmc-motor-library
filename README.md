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

## Installation
1. Install required libraries:
   ```cpp
   #include <SPI.h>
   #include "JMCMotor.h"
   #include <Ethernet.h>
   #include <EthernetUdp.h>
   #include <ModbusRTUMaster.h>


Usage Example
------------------

// Initialize motor
motor.Setup(47, 47);

// Move to absolute position
Send: "P:ABS,1000,5.0"
Response: "Moving to absolute position 1000 at velocity 5.0"

// Query current position
Send: "P:?"
Response: "P: 1000"



Notes
This is a personal project. Usage and distribution are restricted unless explicitly permitted by the author.
