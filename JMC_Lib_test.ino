#include <SPI.h>
#include "JMCMotor.h"
#include <avr/wdt.h>
#include "Arduino.h"
#include <Ethernet.h>
#include <EthernetUdp.h>
#include <ModbusRTUMaster.h>

// Network Configuration
byte mac[] = { 0xA8, 0x61, 0x0A, 0xAE, 0xF3, 0x7F };
IPAddress ip(192, 168, 1, 2);            // Arduino IP
IPAddress remoteServer(192, 168, 1, 100); // PC IP
unsigned int localPort = 8002;            // Arduino listening port
unsigned int remotePort = 8000;           // PC listening port

// Motor Configuration
#define MOTOR_ID 1
#define DE_RE_PIN 9
#define LED_PIN 12

// Global variables
EthernetUDP Udp;
ModbusRTUMaster modbus(Serial2, DE_RE_PIN);
JMCMotor motor(MOTOR_ID, &modbus);
char *saveptr;
const int BUFFER_SIZE = 255;
char cmdBuffer[BUFFER_SIZE];
float defaultVelocity = 5.0;  // Initial default velocity

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  pinMode(DE_RE_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  Ethernet.begin(mac, ip);
  Udp.begin(localPort);

  modbus.begin(115200);
  motor.Setup(47, 47);

  sendUDPMessage("System ready for commands");
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  handleUDPCommands();
}

void handleUDPCommands() {
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    memset(cmdBuffer, 0, BUFFER_SIZE);
    Udp.read(cmdBuffer, BUFFER_SIZE);
    digitalWrite(LED_PIN, HIGH);

    String command = strtok_r(cmdBuffer, ":", &saveptr);
    sendUDPMessage("RCV: " + String(cmdBuffer));

    if (command == "P") handlePositionCommand();
    else if (command == "V") handleVelocityCommand();
    else if (command == "H") handleHomingCommand();
    else if (command == "B") handleBrakeCommand();
    else if (command == "S") handleStatusCommand();
    else if (command == "M") handleModeCommand();
    else if (command == "A") handleAccelerationCommand();
    else if (command == "Q") handleQuickStopCommand();
    else if (command == "Z") handleZeroCommand();
    else if (command == "R") handleResetCommand();
    else if (command == "I") handleInfoCommand();
    else if (command == "X") {
      wdt_enable(WDTO_15MS);  // Set WDT to reset after 15 milliseconds

      while (1);  // Infinite loop until WDT triggers a reset
    }

    digitalWrite(LED_PIN, LOW);
  }
}

void handlePositionCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    sendUDPMessage("Invalid position command");
    return;
  }

  if (strcmp(subCmd, "?") == 0) {
    long pos = motor.getCurrentPosition();
    sendUDPMessage("P: " + String(pos));
  }
  else if (strcmp(subCmd, "VEL") == 0) {  // New velocity command
    char* velStr = strtok_r(NULL, ",", &saveptr);
    if (velStr) {
      float newVel = atof(velStr);
      if (newVel > 0) {  // Ensure velocity is positive
        defaultVelocity = newVel;
        sendUDPMessage("Default velocity set to " + String(defaultVelocity));
      } else {
        sendUDPMessage("Invalid velocity value: Must be greater than 0");
      }
    } else {
      // If no value provided, return current default velocity
      sendUDPMessage("Current default velocity: " + String(defaultVelocity));
    }
  }
  else if (strcmp(subCmd, "ABS") == 0) {
    char* posStr = strtok_r(NULL, ",", &saveptr);
    char* velStr = strtok_r(NULL, ",", &saveptr);

    if (posStr) {
      unsigned long pos = strtoul(posStr, NULL, 10);
      float vel = velStr ? atof(velStr) : defaultVelocity;  // Use defaultVelocity instead of hard-coded value

      motor.moveToPosition(pos, vel, true);
      sendUDPMessage("Moving to absolute position " + String(pos) + " at velocity " + String(vel));
    } else {
      sendUDPMessage("Invalid absolute position command: Missing position value");
    }
  }
  else if (strcmp(subCmd, "REL") == 0) {
    char* posStr = strtok_r(NULL, ",", &saveptr);
    char* velStr = strtok_r(NULL, ",", &saveptr);

    if (posStr) {
      long pos = atol(posStr);
      float vel = velStr ? atof(velStr) : defaultVelocity;  // Use defaultVelocity instead of hard-coded value

      motor.moveToPosition(pos, vel, false);
      sendUDPMessage("Moving relative position " + String(pos) + " at velocity " + String(vel));
    } else {
      sendUDPMessage("Invalid relative position command: Missing position value");
    }
  }
  else {
    // Handle legacy/direct position command
    unsigned long pos = strtoul(subCmd, NULL, 10);
    motor.moveToPosition(pos, defaultVelocity, true);
    sendUDPMessage("Moving to position " + String(pos));
  }
}

void handleVelocityCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    sendUDPMessage("Invalid velocity command");
    return;
  }

  if (strcmp(subCmd, "?") == 0) {
    float vel = motor.getCurrentVelocity();
    sendUDPMessage("V: " + String(vel));
  }
  else if (strcmp(subCmd, "PRF") == 0) {
    char* velStr = strtok_r(NULL, ",", &saveptr);
    char* accStr = strtok_r(NULL, ",", &saveptr);
    char* decStr = strtok_r(NULL, ",", &saveptr);
    if (velStr && accStr && decStr) {
      float vel = atof(velStr);
      float acc = atof(accStr);
      float dec = atof(decStr);
      motor.setVelocityProfile(acc, dec);
      motor.setVelocity(vel);
      sendUDPMessage("Set velocity profile: V=" + String(vel) + " A=" + String(acc) + " D=" + String(dec));
    }
  }
  else {
    float vel = atof(subCmd);
    motor.setVelocity(vel);
    sendUDPMessage("Set velocity to " + String(vel));
  }
}

void handleHomingCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    motor.startHoming(JMCMotor::HOMING_METHOD_1);
    sendUDPMessage("H:I{" + String(MOTOR_ID) + "}");
    return;
  }

  int method = atoi(subCmd);
  char* offsetStr = strtok_r(NULL, ",", &saveptr);
  char* speedStr = strtok_r(NULL, ",", &saveptr);

  long offset = offsetStr ? atol(offsetStr) : 0;
  float speed = speedStr ? atof(speedStr) : 5.0;

  motor.setHomingMethod(static_cast<JMCMotor::HomingMethod>(method));
  motor.setHomingOffset(offset);
  motor.setHomingSpeed(speed);
  motor.startHoming(static_cast<JMCMotor::HomingMethod>(method));
  sendUDPMessage("Starting homing method " + String(method));
}

void handleBrakeCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    motor.motorStop();
    sendUDPMessage("QB " + String(MOTOR_ID));
    return;
  }

  int brakeType = atoi(subCmd);
  switch (brakeType) {
    case 1: motor.quickStop(JMCMotor::QUICK_STOP_DECEL); break;
    case 2: motor.quickStop(JMCMotor::QUICK_STOP_FAST); break;
    case 3: motor.quickStop(JMCMotor::QUICK_STOP_IMMEDIATE); break;
    default: motor.motorStop();
  }
  sendUDPMessage("Brake type " + String(brakeType) + " applied");
}

void handleStatusCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  uint16_t status = motor.motorStats(); // Get raw status word
  
  if (!subCmd) {
    // Return both raw status and hex formatted status
    sendUDPMessage("STATUS:" + String(status) + "|0x" + String(status, HEX));
    return;
  }

  if (strcmp(subCmd, "DETAIL") == 0) {
    JMCMotor::MotorStatus stat = motor.getDetailedStatus();
    
    // Create human-readable status report
    String details = "Motor Status (0x" + String(status, HEX) + "):\n";
    
    // Operation status
    details += "Operation: ";
    if (stat.motorRunning) details += "RUNNING";
    else if (stat.motorHalt) details += "HALTED";
    else if (stat.motorEnabled) details += "ENABLED";
    else details += "DISABLED";
    details += "\n";
    
    // Error and warning status
    if (stat.errorStatus) details += "ERROR ACTIVE\n";
    if (stat.warning) details += "WARNING ACTIVE\n";
    if (stat.overPositionError) details += "POSITION ERROR\n";
    
    // Position status
    details += "Position: ";
    if (stat.targetReached) details += "TARGET REACHED";
    else details += "IN MOTION";
    details += "\n";
    
    // Limit switches
    if (stat.cwLimit) details += "CW LIMIT ACTIVE\n";
    if (stat.ccwLimit) details += "CCW LIMIT ACTIVE\n";
    
    // Homing status
    if (stat.homingComplete) details += "HOMING COMPLETE\n";
    if (stat.swMechanicalOrigin) details += "AT MECHANICAL ORIGIN\n";
    
    // Other flags
    if (stat.quickStopActive) details += "QUICK STOP ACTIVE\n";
    if (stat.readyToInit) details += "READY TO INITIALIZE\n";
    if (stat.initComplete) details += "INITIALIZATION COMPLETE\n";
    if (stat.driveWorking) details += "DRIVE WORKING\n";
    
    // Add current position and velocity
    details += "Current Position: " + String(motor.getCurrentPosition()) + "\n";
    details += "Current Velocity: " + String(motor.getCurrentVelocity());
    
    sendUDPMessage(details);
  }
  else if (strcmp(subCmd, "RAW") == 0) {
    // Return just the raw status word in both decimal and hex
    sendUDPMessage("RAW_STATUS:" + String(status) + "|0x" + String(status, HEX));
  }
  else if (strcmp(subCmd, "FLAGS") == 0) {
    // Return status with individual bits explained
    JMCMotor::MotorStatus stat = motor.getDetailedStatus();
    String flagDetails = "Status Flags (0x" + String(status, HEX) + "):\n";
    
    // Add each flag with its state
    flagDetails += "Bit 0 (Ready to init): " + String(stat.readyToInit ? "1" : "0") + "\n";
    flagDetails += "Bit 1 (Init complete): " + String(stat.initComplete ? "1" : "0") + "\n";
    flagDetails += "Bit 2 (Motor enabled): " + String(stat.motorEnabled ? "1" : "0") + "\n";
    flagDetails += "Bit 3 (Error status): " + String(stat.errorStatus ? "1" : "0") + "\n";
    flagDetails += "Bit 4 (Drive working): " + String(stat.driveWorking ? "1" : "0") + "\n";
    flagDetails += "Bit 5 (Quick stop active): " + String(stat.quickStopActive ? "1" : "0") + "\n";
    flagDetails += "Bit 6 (Init state): " + String(stat.initState ? "1" : "0") + "\n";
    flagDetails += "Bit 7 (Warning): " + String(stat.warning ? "1" : "0") + "\n";
    flagDetails += "Bit 8 (Motor halt): " + String(stat.motorHalt ? "1" : "0") + "\n";
    flagDetails += "Bit 9 (Motor running): " + String(stat.motorRunning ? "1" : "0") + "\n";
    flagDetails += "Bit 10 (Target reached): " + String(stat.targetReached ? "1" : "0") + "\n";
    flagDetails += "Bit 11 (SW mechanical origin): " + String(stat.swMechanicalOrigin ? "1" : "0") + "\n";
    flagDetails += "Bit 12 (Homing complete): " + String(stat.homingComplete ? "1" : "0") + "\n";
    flagDetails += "Bit 13 (Over position error): " + String(stat.overPositionError ? "1" : "0") + "\n";
    flagDetails += "Bit 14 (CW limit): " + String(stat.cwLimit ? "1" : "0") + "\n";
    flagDetails += "Bit 15 (CCW limit): " + String(stat.ccwLimit ? "1" : "0");
    
    sendUDPMessage(flagDetails);
  }
  else if (strcmp(subCmd, "COMPACT") == 0) {
    // Create a compact but informative status string
    JMCMotor::MotorStatus stat = motor.getDetailedStatus();
    String compactStatus = "M" + String(MOTOR_ID) + ":";
    
    // Basic operation state
    if (stat.errorStatus) compactStatus += "E";
    else if (stat.motorRunning) compactStatus += "R";
    else if (stat.motorHalt) compactStatus += "H";
    else if (stat.motorEnabled) compactStatus += "S"; // Standby
    else compactStatus += "D"; // Disabled
    
    // Important flags
    if (stat.warning) compactStatus += "W";
    if (stat.targetReached) compactStatus += "T";
    if (stat.homingComplete) compactStatus += "O"; // Origin
    if (stat.cwLimit) compactStatus += "C";
    if (stat.ccwLimit) compactStatus += "A";
    
    // Position and mode
    compactStatus += ":" + String(motor.getCurrentPosition());
    compactStatus += ":" + String(motor.getCurrentMode());
    
    sendUDPMessage(compactStatus);
  }
}

void handleModeCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    JMCMotor::OperationMode currentMode = motor.getCurrentMode();
    sendUDPMessage("Current mode: " + String(currentMode));
    return;
  }

  int mode = atoi(subCmd);
  switch (mode) {
    case 1:
      motor.setOperationMode(JMCMotor::MODE_POSITION);
      sendUDPMessage("Set to Position mode");
      break;
    case 3:
      motor.setOperationMode(JMCMotor::MODE_VELOCITY);
      sendUDPMessage("Set to Velocity mode");
      break;
    case 6:
      motor.setOperationMode(JMCMotor::MODE_HOMING);
      sendUDPMessage("Set to Homing mode");
      break;
    default:
      sendUDPMessage("Invalid mode");
  }
}

void handleAccelerationCommand() {
  char* accStr = strtok_r(NULL, ",", &saveptr);
  char* decStr = strtok_r(NULL, ",", &saveptr);

  if (!accStr || !decStr) {
    sendUDPMessage("Invalid acceleration command");
    return;
  }

  float acc = atof(accStr);
  float dec = atof(decStr);
  motor.targetAccDec(acc, dec);
  sendUDPMessage("Set Acc: " + String(acc) + " Dec: " + String(dec));
}

void handleQuickStopCommand() {
  char* typeStr = strtok_r(NULL, ",", &saveptr);
  int stopType = typeStr ? atoi(typeStr) : 1;

  switch (stopType) {
    case 1: motor.quickStop(JMCMotor::QUICK_STOP_DECEL); break;
    case 2: motor.quickStop(JMCMotor::QUICK_STOP_FAST); break;
    case 3: motor.quickStop(JMCMotor::QUICK_STOP_IMMEDIATE); break;
    default: motor.quickStop(JMCMotor::QUICK_STOP_DECEL);
  }
  sendUDPMessage("Quick stop type " + String(stopType) + " executed");
}

void handleZeroCommand() {
  char* subCmd = strtok_r(NULL, ",", &saveptr);
  if (!subCmd) {
    motor.setZeroPosition();
    sendUDPMessage("Position set to zero");
    return;
  }

  if (strcmp(subCmd, "OFS") == 0) {
    char* offsetStr = strtok_r(NULL, ",", &saveptr);
    if (offsetStr) {
      long offset = atol(offsetStr);
      // First move to the offset position
      motor.moveToPosition(offset, defaultVelocity, true);
      delay(500);  // Wait for movement to complete
      // Then set zero at this position
      motor.setZeroPosition();
      sendUDPMessage("Position set to zero with offset: " + String(offset));
    }
  }
}

void handleResetCommand() {
  motor.AlarmReset();
  motor.Setup(47, 47);
  sendUDPMessage("Motor reset complete");
}

void handleInfoCommand() {
  uint16_t statusWord = motor.motorStats();
  JMCMotor::MotorStatus stat = motor.getDetailedStatus();
  
  // Get current operation mode as text
  String modeText;
  switch(motor.getCurrentMode()) {
    case JMCMotor::MODE_POSITION:
      modeText = "Position";
      break;
    case JMCMotor::MODE_VELOCITY:
      modeText = "Velocity";
      break;
    case JMCMotor::MODE_HOMING:
      modeText = "Homing";
      break;
    default:
      modeText = "Unknown";
  }
  
  // Get state as text
  String stateText;
  if (stat.errorStatus) stateText = "ERROR";
  else if (stat.motorRunning) stateText = "RUNNING";
  else if (stat.motorHalt) stateText = "HALTED";
  else if (stat.motorEnabled) stateText = "ENABLED";
  else stateText = "DISABLED";
  
  // Format comprehensive info
  String info = "Motor " + String(MOTOR_ID) + " Info:\n";
  info += "-------------------\n";
  info += "Status: " + stateText + " (0x" + String(statusWord, HEX) + ")\n";
  info += "Mode: " + modeText + " (" + String(motor.getCurrentMode()) + ")\n";
  info += "Position: " + String(motor.getCurrentPosition()) + "\n";
  info += "Velocity: " + String(motor.getCurrentVelocity()) + "\n";
  info += "-------------------\n";
  
  // Additional status flags
  if (stat.targetReached) info += "Target Reached: YES\n";
  if (stat.homingComplete) info += "Homing Complete: YES\n";
  if (stat.warning) info += "Warning: YES\n";
  if (stat.cwLimit) info += "CW Limit: ACTIVE\n";
  if (stat.ccwLimit) info += "CCW Limit: ACTIVE\n";
  if (stat.quickStopActive) info += "Quick Stop: ACTIVE\n";
  info += "-------------------\n";
  
  // Add config information
  info += "Default Velocity: " + String(defaultVelocity);
  
  sendUDPMessage(info);
}

void sendUDPMessage(const String &message) {
  Udp.beginPacket(remoteServer, remotePort);
  Udp.write(message.c_str());
  Udp.endPacket();
  
  // Also print to Serial for debugging
  Serial.println("UDP>>> " + message);
}
