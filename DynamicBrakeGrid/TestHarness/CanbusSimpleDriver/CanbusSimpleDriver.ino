#include <SPI.h>
#include <mcp2515.h>

// CAN Pins
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2
#define DEVICE_ID 0x01

// Command IDs
#define CMD_EMERGENCY_STOP   0x00
#define CMD_DISABLE_OUTPUT   0x01
#define CMD_ENABLE_OUTPUT    0x02
#define CMD_RESET_FAULTS     0x03
#define CMD_GET_FAULT_COUNT  0x04
#define CMD_GET_FAULT_LIST   0x05
#define CMD_GET_VOLTAGE_IN   0x06
#define CMD_GET_VOLTAGE_OUT  0x07
#define CMD_GET_CURRENT      0x08
#define CMD_GET_TEMPERATURE  0x09
#define CMD_GET_FAN_SPEED    0x0A
#define CMD_GET_POWER        0x0B
#define CMD_SET_VOLTAGE      0x0C
#define CMD_HEARTBEAT        0x0D

MCP2515 mcp2515(CAN_CS_PIN);
volatile bool canMessageReceived = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  SPI.begin();
  
  // Initialize CAN controller
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("Error resetting MCP2515");
    while(1);
  }
  
  if (mcp2515.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
    Serial.println("Error setting bitrate");
    while(1);
  }
  
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("Error setting normal mode");
    while(1);
  }

  pinMode(CAN_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), onCANInterrupt, FALLING);

  Serial.println("Test Harness Ready");
  printHelp();
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    processCommand(input);
  }

  if (canMessageReceived) {
    readCANResponse();
    canMessageReceived = false;
  }
}

void processCommand(String cmd) {
  if (cmd == "help") {
    printHelp();
  } 
  else if (cmd == "enable") {
    enableOutput();
  }
  else if (cmd == "disable") {
    disableOutput();
  }
  else if (cmd == "emergency") {
    emergencyStop();
  }
  else if (cmd == "reset") {
    resetFaults();
  }
  else if (cmd == "faults") {
    getFaultCount();
  }
  else if (cmd == "faultlist") {
    getFaultList();
  }
  else if (cmd == "vin") {
    getVoltageIn();
  }
  else if (cmd == "vout") {
    getVoltageOut();
  }
  else if (cmd == "current") {
    getCurrent();
  }
  else if (cmd == "temp") {
    getTemperature();
  }
  else if (cmd == "fan") {
    getFanSpeed();
  }
  else if (cmd == "power") {
    getPower();
  }
  else if (cmd.startsWith("set ")) {
    float voltage = cmd.substring(4).toFloat();
    setVoltage(voltage);
  }
  else {
    Serial.println("Unknown command. Type 'help' for options.");
  }
}

void sendCANCommand(uint8_t cmd, uint8_t* data = nullptr, uint8_t len = 0) {
  struct can_frame frame;
  frame.can_id = (DEVICE_ID << 5) | cmd;
  frame.can_dlc = len;
  
  if (data && len > 0) {
    memcpy(frame.data, data, len);
  }

  Serial.print("Sending CAN command: 0x");
  Serial.print(cmd, HEX);
  if (len > 0) {
    Serial.print(" Data:");
    for (int i = 0; i < len; i++) {
      Serial.print(" 0x");
      Serial.print(data[i], HEX);
    }
  }
  Serial.println();

  if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
    Serial.println("Error sending command");
  }
}

void readCANResponse() {
  struct can_frame frame;
  if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
    uint8_t commandId = frame.can_id & 0x1F;  // Extract command ID from lower 5 bits
    uint8_t deviceId = frame.can_id >> 5;     // Extract device ID from upper bits
    
    Serial.print("Device ID: 0x");
    Serial.print(deviceId, HEX);
    Serial.print(" | Response to CMD 0x");
    Serial.print(commandId, HEX);
    Serial.print(": ");
    
    switch(commandId) {
      case CMD_GET_CURRENT:  // 0x08
        if(frame.can_dlc == 4) {
          float current;
          memcpy(&current, frame.data, 4);
          Serial.print("Current: ");
          Serial.print(current, 3);
          Serial.println(" A");
          
          // Debug output of raw bytes
          Serial.print("Raw bytes: ");
          for(int i = 0; i < 4; i++) {
            Serial.print(frame.data[i], HEX);
            Serial.print(" ");
          }
          Serial.println();
        } else {
          Serial.print("Unexpected data length for current: ");
          Serial.println(frame.can_dlc);
        }
        break;
        
      case 0x1F: // ACK
        Serial.println("Command acknowledged");
        break;
        
      case CMD_GET_FAULT_COUNT:
        Serial.print("Fault count: ");
        Serial.println(frame.data[0]);
        break;
        
      case CMD_GET_VOLTAGE_IN:
        if(frame.can_dlc == 4) {
          float voltage;
          memcpy(&voltage, frame.data, 4);
          Serial.print("Input Voltage: ");
          Serial.print(voltage, 2);
          Serial.println(" V");
        }
        break;
        
      case CMD_GET_VOLTAGE_OUT:
        if(frame.can_dlc == 4) {
          float voltage;
          memcpy(&voltage, frame.data, 4);
          Serial.print("Output Voltage: ");
          Serial.print(voltage, 2);
          Serial.println(" V");
        }
        break;
        
        
      case CMD_GET_POWER:
        if(frame.can_dlc == 4) {
          float power;
          memcpy(&power, frame.data, 4);
          Serial.print("Power: ");
          Serial.print(power, 2);
          Serial.println(" W");
        }
        break;
        
      case CMD_GET_TEMPERATURE:
        if(frame.can_dlc == 4) {
          float temp;
          memcpy(&temp, frame.data, 4);
          Serial.print("Temperature: ");
          Serial.print(temp, 1);
          Serial.println(" °C");
        }
        break;
        
      case CMD_GET_FAN_SPEED:
        if(frame.can_dlc == 2) {
          uint16_t rpm = (frame.data[0] << 8) | frame.data[1];
          Serial.print("Fan Speed: ");
          Serial.print(rpm);
          Serial.println(" RPM");
        } else if(frame.can_dlc == 4) {
          float rpm;
          memcpy(&rpm, frame.data, 4);
          Serial.print("Fan Speed: ");
          Serial.print(rpm, 0);
          Serial.println(" RPM");
        }
        break;
        
      case CMD_GET_FAULT_LIST:
        Serial.print("Fault codes: ");
        for (int i = 0; i < frame.can_dlc; i++) {
          Serial.print("0x");
          Serial.print(frame.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
        break;
        
      default:
        // Print raw data for unhandled responses
        Serial.print("Raw data: ");
        for (int i = 0; i < frame.can_dlc; i++) {
          Serial.print(frame.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
    }
  }
}

// Command implementations (unchanged from original)
void enableOutput() {
  sendCANCommand(CMD_ENABLE_OUTPUT);
}

void disableOutput() {
  sendCANCommand(CMD_DISABLE_OUTPUT);
}

void emergencyStop() {
  sendCANCommand(CMD_EMERGENCY_STOP);
}

void resetFaults() {
  sendCANCommand(CMD_RESET_FAULTS);
}

void getFaultCount() {
  sendCANCommand(CMD_GET_FAULT_COUNT);
}

void getFaultList() {
  sendCANCommand(CMD_GET_FAULT_LIST);
}

void getVoltageIn() {
  sendCANCommand(CMD_GET_VOLTAGE_IN);
}

void getVoltageOut() {
  sendCANCommand(CMD_GET_VOLTAGE_OUT);
}

void getCurrent() {
  sendCANCommand(CMD_GET_CURRENT);
}

void getTemperature() {
  sendCANCommand(CMD_GET_TEMPERATURE);
}

void getFanSpeed() {
  sendCANCommand(CMD_GET_FAN_SPEED);
}

void getPower() {
  sendCANCommand(CMD_GET_POWER);
}

void setVoltage(float voltage) {
  uint8_t data[4];
  memcpy(data, &voltage, sizeof(voltage));
  sendCANCommand(CMD_SET_VOLTAGE, data, sizeof(data));
  Serial.print("Set voltage to: ");
  Serial.print(voltage);
  Serial.println("V");
}

void printHelp() {
  Serial.println("Available Commands:");
  Serial.println("  enable       - Enable output");
  Serial.println("  disable      - Disable output");
  Serial.println("  emergency    - Emergency stop");
  Serial.println("  reset        - Reset faults");
  Serial.println("  faults       - Get fault count");
  Serial.println("  faultlist    - Get fault list");
  Serial.println("  vin          - Get input voltage");
  Serial.println("  vout         - Get output voltage");
  Serial.println("  current      - Get current");
  Serial.println("  temp         - Get temperature");
  Serial.println("  fan          - Get fan speed");
  Serial.println("  power        - Get power");
  Serial.println("  set [value]  - Set voltage (e.g., 'set 24.5')");
  Serial.println("  help         - Show this help");
}

void onCANInterrupt() {
  canMessageReceived = true;
}
