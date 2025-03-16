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
void sendHeartbeat();
void queryParameter(uint8_t parameterID);
void printFaultBlinkCode(const char* blinkCode);
void onCANInterrupt();

volatile bool canMessageReceived = false; // Flag for CAN message reception

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
  if (mcp2515.setBitrate(CAN_1000KBPS) != MCP2515::ERROR_OK) { // Updated to 500 kbps
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
  delay(2000); // Allow device to start
}

void loop() {
  // Example usage of the functions
  sendHeartbeat();
  delay(1000); // Add a delay between messages

  // Add more commands as needed
}

// Send a CAN command
void sendCANCommand(uint8_t commandID) {
  struct can_frame canMsg;
  canMsg.can_id = (DEVICE_ID << 5) | commandID; // Construct CAN ID as (DEVICE_ID << 5) | COMMAND_ID
  canMsg.can_dlc = 1;                          // Data length (1 byte for command ID)
  canMsg.data[0] = 0x00;                       // Optional: Add data if needed

  // Debug: Print CAN ID
  Serial.print("Sending CAN ID: 0x");
  Serial.println(canMsg.can_id, HEX);

  // Send the CAN message
  if (mcp2515.sendMessage(&canMsg) == MCP2515::ERROR_OK) {
    Serial.print("Command sent: 0x");
    Serial.println(commandID, HEX);
  } else {
    Serial.println("Error sending CAN command.");
  
}

// Read CAN response
void readCANResponse() {
  if (canMessageReceived) {
    struct can_frame canMsg;
    if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
      // Filter for acknowledgment packet (CAN ID 0x31)
      if (canMsg.can_id == 0x31 && canMsg.can_dlc == 1 && canMsg.data[0] == 0x01) {
        Serial.print("Acknowledgment received: ");
        Serial.println(canMsg.data[0], HEX);
      } else {
        Serial.println("Unexpected CAN message received.");
      }
    } else {
      Serial.println("Error reading CAN message.");
    }
    canMessageReceived = false; // Reset flag immediately
  } else {
    Serial.println("No response received.");
  }
}

// Send heartbeat ping
void sendHeartbeat() {
  Serial.println("Sending heartbeat ping...");
  sendCANCommand(0x11); // Command ID for Heartbeat Ping
  delay(50);
  readCANResponse();
}

// CAN interrupt handler
void onCANInterrupt() {
  canMessageReceived = true;
}