#ifndef JMCMotor_h
#define JMCMotor_h

#include "Arduino.h"
#include <ModbusRTUMaster.h>

class JMCMotor {
  public:
    // Operation modes
    enum OperationMode {
        MODE_POSITION = 1,
        MODE_VELOCITY = 3,
        MODE_HOMING = 6
    };

    // Quick stop modes
    enum QuickStopMode {
        QUICK_STOP_DECEL = 1,
        QUICK_STOP_FAST = 2,
        QUICK_STOP_IMMEDIATE = 3
    };

    // Homing methods
    enum HomingMethod {
        HOMING_METHOD_0 = 0,  // No movement
        HOMING_METHOD_1 = 1,  // Move left to CW limit
        HOMING_METHOD_2 = 2,  // Move right to CCW limit
        HOMING_METHOD_3 = 3,  // Move left to mechanical origin
        HOMING_METHOD_4 = 4,  // Move right to mechanical origin
        HOMING_METHOD_5 = 5,  // Move left with quick reduction
        HOMING_METHOD_6 = 6   // Move right with quick reduction
    };

    // Status structure
    struct MotorStatus {
        bool readyToInit;
        bool initComplete;
        bool motorEnabled;
        bool errorStatus;
        bool driveWorking;
        bool quickStopActive;
        bool initState;
        bool warning;
        bool motorHalt;
        bool motorRunning;
        bool targetReached;
        bool swMechanicalOrigin;
        bool homingComplete;
        bool overPositionError;
        bool cwLimit;
        bool ccwLimit;
    };

    // Constructor
    JMCMotor(int address, ModbusRTUMaster* modbus);
    
    // Basic setup and control
    void Setup(float acceleration, float deceleration);
    void motorStart();
    void motorStop();
    void EStop();
    void loosenBrake();
    void AlarmReset();
    void quickStop(QuickStopMode stopMode);
    
    // Position control
    void moveToPosition(long position, float velocity, bool absolute);
    void setPositionProfile(float velocity, float acceleration, float deceleration);
    long getCurrentPosition();
    bool isPositionReached();
    
    // Velocity control
    void setVelocity(float velocity);
    void setVelocityProfile(float acceleration, float deceleration);
    float getCurrentVelocity();
    
    // Status and configuration
    uint16_t motorStats();
    MotorStatus getDetailedStatus();
    bool isRunning();
    void targetAccDec(float acc, float dec);
    
    // Mode control
    void setOperationMode(OperationMode mode);
    OperationMode getCurrentMode();
    
    // Homing control
    void startHoming(HomingMethod method = HOMING_METHOD_1);
    void setHomingMethod(HomingMethod method);
    void setHomingOffset(long offset);
    void setHomingSpeed(float speed);
    void setZeroPosition();
    

  private:
    int _slaveID;
    float _accel;
    float _decel;
    bool _formatMSBFirst;
    ModbusRTUMaster* _modbus;
    OperationMode _currentMode;

    // Internal methods
    void drive_Init();
    void PMOE();
    void sampling_one();
    void sampling_two();
    void targetSpeed(float spd);
    void targetPosi(unsigned long degree);
    long readPosition();
    long readVelocity();
    
    // Register access
    void WSHR(long regAdd, long val);
    void WMHR(long regAdd, unsigned long val2, unsigned long val1 = 0);
    uint16_t readHoldingRegister(long regAdd);
    void readMultipleRegisters(long regAdd, uint16_t* data, int count);

    // Register addresses
    long controlWord() { return 0x6040; }
    long statusWord() { return 0x6041; }
    long modeOperation() { return 0x6060; }
    long posVal() { return 0x6064; }
    long veloVal() { return 0x606C; }
    long targetPos() { return 0x607A; }
    long targVel() { return 0x6081; }
    long accel() { return 0x6083; }
    long decel() { return 0x6084; }
    long homingMethod() { return 0x6098; }
    long homingSpeed() { return 0x6099; }
    long homeOffset() { return 0x607C; }
};

#endif
