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
  if (!subCmd) {
    uint16_t status = motor.motorStats();
    sendUDPMessage(String(status) + "|" + String(status));
    return;
  }

  if (strcmp(subCmd, "DETAIL") == 0) {
    JMCMotor::MotorStatus status = motor.getDetailedStatus();
    String details = "Status:\n";
    details += "Running: " + String(status.motorRunning) + "\n";
    details += "Error: " + String(status.errorStatus) + "\n";
    details += "Position reached: " + String(status.targetReached) + "\n";
    details += "Homing complete: " + String(status.homingComplete);
    sendUDPMessage(details);
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
  String info = "Motor Info:\n";
  info += "Position: " + String(motor.getCurrentPosition()) + "\n";
  info += "Velocity: " + String(motor.getCurrentVelocity()) + "\n";
  info += "Mode: " + String(motor.getCurrentMode()) + "\n";
  info += "Status: " + String(motor.motorStats());
  sendUDPMessage(info);
}

void sendUDPMessage(const String &message) {
  Udp.beginPacket(remoteServer, remotePort);
  Udp.write(message.c_str());
  Udp.endPacket();
}
