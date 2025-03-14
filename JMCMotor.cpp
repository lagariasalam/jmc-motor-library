#include "JMCMotor.h"

JMCMotor::JMCMotor(int address, ModbusRTUMaster* modbus) {
    _slaveID = address;
    _modbus = modbus;
    _formatMSBFirst = true;
    _currentMode = MODE_POSITION;
}

void JMCMotor::Setup(float acceleration, float deceleration) {
    _accel = acceleration;
    _decel = deceleration;

    drive_Init();
    loosenBrake();
    PMOE();
    setOperationMode(MODE_POSITION);
    targetAccDec(_accel, _decel);
}

// Basic Motor Control
void JMCMotor::drive_Init() {
    WSHR(controlWord(), 0x01);  // Initialize drive
}

void JMCMotor::PMOE() {
    WSHR(controlWord(), 0x0F);  // Enable operation
}

void JMCMotor::loosenBrake() {
    WSHR(controlWord(), 0x03);  // Release brake
}

void JMCMotor::motorStart() {
    PMOE();
    setOperationMode(_currentMode);
}

void JMCMotor::motorStop() {
    WSHR(controlWord(), 0x010F);  // Controlled stop
}

void JMCMotor::EStop() {
    WSHR(controlWord(), 0x02);  // Quick stop
}

void JMCMotor::AlarmReset() {
    WSHR(controlWord(), 0x80);  // Reset alarm
}

// Enhanced Position Control
void JMCMotor::moveToPosition(long position, float velocity, bool absolute) {
    setOperationMode(MODE_POSITION);
    
    if (!absolute) {
        // Calculate the new target position by adding the relative offset
        position += getCurrentPosition();  // Add current position to the relative offset
    }
    
    // Configure for absolute or relative positioning
    if (absolute) {
        WSHR(controlWord(), 0x0F);  // Absolute positioning
    } else {
        WSHR(controlWord(), 0x4F);  // Relative positioning (bit 6 set)
    }
    
    targetSpeed(velocity);
    targetPosi(position);  // Write the calculated position to the target position register
    
    // Execute movement
    sampling_one();
    sampling_two();
}

void JMCMotor::setPositionProfile(float velocity, float acceleration, float deceleration) {
    targetAccDec(acceleration, deceleration);
    targetSpeed(velocity);
}

bool JMCMotor::isPositionReached() {
    uint16_t status = motorStats();
    return (status & 0x0400) != 0;  // Check target reached bit
}

// Enhanced Velocity Control
void JMCMotor::setVelocity(float velocity) {
    setOperationMode(MODE_VELOCITY);
    targetSpeed(velocity);
    WSHR(controlWord(), 0x0F);  // Enable operation
}

void JMCMotor::setVelocityProfile(float acceleration, float deceleration) {
    targetAccDec(acceleration, deceleration);
}

void JMCMotor::targetSpeed(float spd) {
    WMHR(targVel(), spd * 10);  // Convert to internal velocity units
}

void JMCMotor::targetPosi(unsigned long degree) {
    if (_formatMSBFirst) {
        unsigned long part1 = degree / 65535;
        unsigned long part2 = degree - part1 * 65535 - part1;
        WMHR(targetPos(), part2, part1);
    } else {
        unsigned long part1 = degree & 0xFFFF;
        unsigned long part2 = (degree >> 16) & 0xFFFF;
        WMHR(targetPos(), part1, part2);
    }
}

void JMCMotor::targetAccDec(float acc, float dec) {
    _accel = acc;
    _decel = dec;
    WSHR(accel(), acc * 10);
    WSHR(decel(), dec * 10);
}

// Homing Control
void JMCMotor::startHoming(HomingMethod method) {
    setOperationMode(MODE_HOMING);
    setHomingMethod(method);
    WSHR(controlWord(), 0x1F);  // Start homing operation
}

void JMCMotor::setHomingMethod(HomingMethod method) {
    WSHR(homingMethod(), method);
}

void JMCMotor::setHomingOffset(long offset) {
    // First move to the offset position
    setOperationMode(MODE_POSITION);
    WSHR(controlWord(), 0x0F);  // Absolute positioning
    targetPosi(offset);
    delay(500);  // Wait for movement to complete
    
    // Then set the home offset
    WMHR(homeOffset(), offset);
}

void JMCMotor::setHomingSpeed(float speed) {
    WMHR(homingSpeed(), speed * 10);
}

void JMCMotor::setZeroPosition() {
    setOperationMode(MODE_HOMING);
    WSHR(homingMethod(), HOMING_METHOD_0);
    WMHR(homeOffset(), 0);
    startHoming(HOMING_METHOD_0);
    delay(100);  // Wait for completion
}

// Mode Control
void JMCMotor::setOperationMode(OperationMode mode) {
    _currentMode = mode;
    PMOE();
    WSHR(modeOperation(), mode);
    delay(10);  // Allow mode to change
}

JMCMotor::OperationMode JMCMotor::getCurrentMode() {
    return static_cast<OperationMode>(readHoldingRegister(modeOperation()));
}

// Status Methods
uint16_t JMCMotor::motorStats() {
    uint16_t readFromRegister[1];
    delay(10);
    _modbus->readHoldingRegisters(_slaveID, statusWord(), readFromRegister, 1);
    return readFromRegister[0];
}

bool JMCMotor::isRunning() {
    return (motorStats() & 0x0200) != 0;
}

void JMCMotor::sampling_one() {
    WSHR(controlWord(), 0x23F);
}

void JMCMotor::sampling_two() {
    WSHR(controlWord(), 0x26F);
}

// Register Access Methods
void JMCMotor::WSHR(long regAdd, long val) {
    delay(10);
    _modbus->writeSingleHoldingRegister(_slaveID, regAdd, val);
}

void JMCMotor::WMHR(long regAdd, unsigned long val2, unsigned long val1) {
    delay(10);
    uint16_t holdingRegisters[2] = {(uint16_t)val1, (uint16_t)val2};
    _modbus->writeMultipleHoldingRegisters(_slaveID, regAdd, holdingRegisters, 2);
}

uint16_t JMCMotor::readHoldingRegister(long regAdd) {
    uint16_t readFromRegister[1];
    delay(10);
    _modbus->readHoldingRegisters(_slaveID, regAdd, readFromRegister, 1);
    return readFromRegister[0];
}

void JMCMotor::readMultipleRegisters(long regAdd, uint16_t* data, int count) {
    delay(10);
    _modbus->readHoldingRegisters(_slaveID, regAdd, data, count);
}

long JMCMotor::readPosition() {
    uint16_t readFromRegister[2];
    readMultipleRegisters(posVal(), readFromRegister, 2);
    
    if (_formatMSBFirst) {
        return ((long)readFromRegister[0] << 16) | readFromRegister[1];
    } else {
        return ((long)readFromRegister[1] << 16) | readFromRegister[0];
    }
}

long JMCMotor::readVelocity() {
    uint16_t readFromRegister[2];
    readMultipleRegisters(veloVal(), readFromRegister, 2);
    
    long rawValue;
    if (_formatMSBFirst) {
        rawValue = ((long)readFromRegister[0] << 16) | readFromRegister[1];
    } else {
        rawValue = ((long)readFromRegister[1] << 16) | readFromRegister[0];
    }
    return rawValue / 10;  // Convert from internal units to rps
}


// Add these implementations

long JMCMotor::getCurrentPosition() {
    return readPosition();  // Use existing readPosition() method
}

float JMCMotor::getCurrentVelocity() {
    return readVelocity() / 10.0f;  // Convert from internal units if needed
}

void JMCMotor::quickStop(QuickStopMode stopMode) {
    switch(stopMode) {
        case QUICK_STOP_DECEL:
            WSHR(controlWord(), 0x0002);  // Controlled deceleration
            break;
        case QUICK_STOP_FAST:
            WSHR(controlWord(), 0x0004);  // Fast stop
            break;
        case QUICK_STOP_IMMEDIATE:
            WSHR(controlWord(), 0x0006);  // Immediate stop
            break;
    }
}

JMCMotor::MotorStatus JMCMotor::getDetailedStatus() {
    uint16_t status = motorStats();
    MotorStatus detailedStatus;
    
    // Parse status word bits into the structure
    detailedStatus.readyToInit = (status & 0x0001) != 0;
    detailedStatus.initComplete = (status & 0x0002) != 0;
    detailedStatus.motorEnabled = (status & 0x0004) != 0;
    detailedStatus.errorStatus = (status & 0x0008) != 0;
    detailedStatus.driveWorking = (status & 0x0010) != 0;
    detailedStatus.quickStopActive = (status & 0x0020) != 0;
    detailedStatus.initState = (status & 0x0040) != 0;
    detailedStatus.warning = (status & 0x0080) != 0;
    detailedStatus.motorHalt = (status & 0x0100) != 0;
    detailedStatus.motorRunning = (status & 0x0200) != 0;
    detailedStatus.targetReached = (status & 0x0400) != 0;
    detailedStatus.swMechanicalOrigin = (status & 0x0800) != 0;
    detailedStatus.homingComplete = (status & 0x1000) != 0;
    detailedStatus.overPositionError = (status & 0x2000) != 0;
    detailedStatus.cwLimit = (status & 0x4000) != 0;
    detailedStatus.ccwLimit = (status & 0x8000) != 0;
    
    return detailedStatus;
}
