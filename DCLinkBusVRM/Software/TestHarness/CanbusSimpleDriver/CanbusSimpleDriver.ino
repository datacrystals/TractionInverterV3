#include <SPI.h>
#include <mcp2515.h>

// Define Pins
#define CAN_INT_PIN 2         // Pin for CAN bus interrupt
#define CAN_CS_PIN 10         // Pin for CAN bus chip select

#define DEVICE_ID 0x01        // Unique device ID (0x00 to 0x3F)

// CAN bus object
MCP2515 mcp2515(CAN_CS_PIN);

// Function prototypes
void sendCANCommand(uint8_t commandID);
void readCANResponse();
void enableOutput();
void disableOutput();
void emergencyStop();
void resetFaults();
void getNumberOfFaults();
void getFaultsList();
void getVinVoltage();
void getVoutVoltage();
void getPhase1Current();
void getPhase2Current();
void getPhase1Temperature();
void getPhase2Temperature();
void getFanAirflow();
void getFanRPM();
void getPower();
void getCANFaultStatus();
void enableHeartbeatMode();
void sendHeartbeat();
void printHelpMessage();
void onCANInterrupt();

volatile bool canMessageReceived = false; // Flag for CAN message reception
unsigned long previousMillis = 0;         // Variable to store the last time a heartbeat was sent
const long heartbeatInterval = 1000;      // Interval for sending heartbeat pings (1 second)

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  while (!Serial);

  // Initialize SPI and CAN bus
  SPI.begin();
  if (mcp2515.reset() != MCP2515::ERROR_OK) {
    Serial.println("Error resetting MCP2515.");
    while (1);
  }
  if (mcp2515.setBitrate(CAN_250KBPS) != MCP2515::ERROR_OK) { // Updated to 500 kbps
    Serial.println("Error setting CAN bitrate.");
    while (1);
  }
  if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
    Serial.println("Error setting normal mode.");
    while (1);
  }

  // Attach interrupt for CAN message reception
  pinMode(CAN_INT_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(CAN_INT_PIN), onCANInterrupt, FALLING);

  Serial.println("CAN bus initialized.");
  printHelpMessage();
  delay(2000); // Allow device to start
}

void loop() {
  unsigned long currentMillis = millis();

  // Send heartbeat ping periodically
  if (currentMillis - previousMillis >= heartbeatInterval) {
    previousMillis = currentMillis;
    sendHeartbeat();
  }

  // Check for serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input == "enable") {
      enableOutput();
    } else if (input == "disable") {
      disableOutput();
    } else if (input == "emergency") {
      emergencyStop();
    } else if (input == "reset") {
      resetFaults();
    } else if (input == "faults") {
      getNumberOfFaults();
    } else if (input == "faultlist") {
      getFaultsList();
    } else if (input == "vin") {
      getVinVoltage();
    } else if (input == "vout") {
      getVoutVoltage();
    } else if (input == "current1") {
      getPhase1Current();
    } else if (input == "current2") {
      getPhase2Current();
    } else if (input == "temp1") {
      getPhase1Temperature();
    } else if (input == "temp2") {
      getPhase2Temperature();
    } else if (input == "airflow") {
      getFanAirflow();
    } else if (input == "rpm") {
      getFanRPM();
    } else if (input == "power") {
      getPower();
    } else if (input == "canstatus") {
      getCANFaultStatus();
    } else if (input == "heartbeatmode") {
      enableHeartbeatMode();
    } else if (input == "help") {
      printHelpMessage();
    } else {
      Serial.println("Unknown command. Type 'help' for a list of commands.");
    }
    delay(50);
    readCANResponse();
  }
}

// Send a CAN command
void sendCANCommand(uint8_t commandID) {
  struct can_frame canMsg;
  canMsg.can_id = (DEVICE_ID << 5) | commandID; // Construct CAN ID as (DEVICE_ID << 5) | COMMAND_ID
  canMsg.can_dlc = 1;                          // Data length (1 byte for command ID)
  canMsg.data[0] = 0x00;                       // Optional: Add data if needed

  // Send the CAN message
  if (mcp2515.sendMessage(&canMsg) != MCP2515::ERROR_OK) {
    Serial.println("Error sending CAN command.");
  }
}

// Enable Output
void enableOutput() {
  Serial.println("Enabling output...");
  sendCANCommand(0x02); // Command ID for Enable Output
}

// Disable Output
void disableOutput() {
  Serial.println("Disabling output...");
  sendCANCommand(0x01); // Command ID for Regular Stop (Disable Output)
}

// Emergency Stop
void emergencyStop() {
  Serial.println("Emergency stop...");
  sendCANCommand(0x00); // Command ID for Emergency Stop (Disable Output)
}

// Reset Faults
void resetFaults() {
  Serial.println("Resetting faults...");
  sendCANCommand(0x03); // Command ID for Reset Faults
}

// Get Number of Faults
void getNumberOfFaults() {
  Serial.println("Getting number of faults...");
  sendCANCommand(0x04); // Command ID for Get Number of Faults
}

// Get Faults List
void getFaultsList() {
  Serial.println("Getting faults list...");
  sendCANCommand(0x05); // Command ID for Get Faults List
}

// Get Vin Voltage
void getVinVoltage() {
  Serial.println("Getting Vin voltage...");
  sendCANCommand(0x06); // Command ID for Get Vin Voltage
}

// Get Vout Voltage
void getVoutVoltage() {
  Serial.println("Getting Vout voltage...");
  sendCANCommand(0x07); // Command ID for Get Vout Voltage
}

// Get Phase 1 Current
void getPhase1Current() {
  Serial.println("Getting Phase 1 current...");
  sendCANCommand(0x08); // Command ID for Get Phase 1 Current
}

// Get Phase 2 Current
void getPhase2Current() {
  Serial.println("Getting Phase 2 current...");
  sendCANCommand(0x09); // Command ID for Get Phase 2 Current
}

// Get Phase 1 Temperature
void getPhase1Temperature() {
  Serial.println("Getting Phase 1 temperature...");
  sendCANCommand(0x0A); // Command ID for Get Phase 1 Temperature
}

// Get Phase 2 Temperature
void getPhase2Temperature() {
  Serial.println("Getting Phase 2 temperature...");
  sendCANCommand(0x0B); // Command ID for Get Phase 2 Temperature
}

// Get Fan Airflow
void getFanAirflow() {
  Serial.println("Getting fan airflow...");
  sendCANCommand(0x0C); // Command ID for Get Fan Airflow (LFM)
}

// Get Fan RPM
void getFanRPM() {
  Serial.println("Getting fan RPM...");
  sendCANCommand(0x0D); // Command ID for Get Fan RPM
}

// Get Power
void getPower() {
  Serial.println("Getting power...");
  sendCANCommand(0x0E); // Command ID for Get Power
}

// Get CAN Fault Status
void getCANFaultStatus() {
  Serial.println("Getting CAN fault status...");
  sendCANCommand(0x0F); // Command ID for Get CAN Fault Status
}

// Enable Heartbeat Mode
void enableHeartbeatMode() {
  Serial.println("Enabling heartbeat mode...");
  sendCANCommand(0x10); // Command ID for Enable Heartbeat Mode
}

// Send heartbeat ping
void sendHeartbeat() {
  sendCANCommand(0x11); // Command ID for Heartbeat Ping
  delay(50);
  readCANResponse();
}

// Read CAN response
void readCANResponse() {
  if (canMessageReceived) {
    struct can_frame canMsg;
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      // Filter for acknowledgment packet (CAN ID 0x31)
      if (canMsg.can_id == 0x31 && canMsg.can_dlc == 1 && canMsg.data[0] == 0x01) {
        // Serial.print("Acknowledgment received: ");
        // Serial.println(canMsg.data[0], HEX);
      } else {
        Serial.print("Response received: ");
        for (int i = 0; i < canMsg.can_dlc; i++) {
          Serial.print(canMsg.data[i], HEX);
          Serial.print(" ");
        }
        Serial.println();
      }
    } else {
      Serial.println("Error reading CAN message.");
    }
    canMessageReceived = false; // Reset flag immediately
  }
}

// Print help message
void printHelpMessage() {
  Serial.println("Available commands:");
  Serial.println("  enable       - Enable output");
  Serial.println("  disable      - Disable output");
  Serial.println("  emergency    - Emergency stop");
  Serial.println("  reset        - Reset faults");
  Serial.println("  faults       - Get number of faults");
  Serial.println("  faultlist    - Get faults list");
  Serial.println("  vin          - Get Vin voltage");
  Serial.println("  vout         - Get Vout voltage");
  Serial.println("  current1     - Get Phase 1 current");
  Serial.println("  current2     - Get Phase 2 current");
  Serial.println("  temp1        - Get Phase 1 temperature");
  Serial.println("  temp2        - Get Phase 2 temperature");
  Serial.println("  airflow      - Get fan airflow");
  Serial.println("  rpm          - Get fan RPM");
  Serial.println("  power        - Get power");
  Serial.println("  canstatus    - Get CAN fault status");
  Serial.println("  heartbeatmode- Enable heartbeat mode");
  Serial.println("  help         - Print this help message");
}

// CAN interrupt handler
void onCANInterrupt() {
  canMessageReceived = true;
}
