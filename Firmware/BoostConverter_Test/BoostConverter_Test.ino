#include <SPI.h>
#include "CANBusManager.h"
#include "MessageCode.h"

// Define Pins
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2
#define DEVICE_ID 0x01

CANBusManager canBus(DEVICE_ID, CAN_CS_PIN, CAN_INT_PIN);

unsigned long previousMillis = 0;         // Variable to store the last time a heartbeat was sent
const long heartbeatInterval = 1000;      // Interval for sending heartbeat pings (1 second)

// Function prototypes
void OnMessage(const CANMessage& msg);
void printHelpMessage();

void setup() {
    // Initialize Serial for debugging
    Serial.begin(9600);
    while (!Serial);

    // Initialize CANBusManager
    canBus.Begin();
    canBus.SetCallback(OnMessage);

    Serial.println("CAN bus initialized.");
    printHelpMessage();
    delay(2000); // Allow device to start
}

void loop() {
    canBus.Update();

    unsigned long currentMillis = millis();

    // Send heartbeat ping periodically
    if (currentMillis - previousMillis >= heartbeatInterval) {
        previousMillis = currentMillis;
        canBus.SendCode(HEARTBEAT_PING);
    }

    // Check for serial input
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');
        input.trim();

        if (input == "enable") {
            Serial.println("Enabling output...");
            canBus.SendCode(ENABLE_OUTPUT);
        } else if (input == "disable") {
            Serial.println("Disabling output...");
            canBus.SendCode(REGULAR_STOP);
        } else if (input == "emergency") {
            Serial.println("Emergency stop...");
            canBus.SendCode(EMERGENCY_STOP);
        } else if (input == "reset") {
            Serial.println("Resetting faults...");
            canBus.SendCode(RESET_FAULTS);
        } else if (input == "faults") {
            Serial.println("Getting number of faults...");
            canBus.SendCode(GET_FAULT_COUNT);
        } else if (input == "faultlist") {
            Serial.println("Getting faults list...");
            canBus.SendCode(GET_FAULT_LIST);
        } else if (input == "vin") {
            Serial.println("Getting Vin voltage...");
            canBus.SendCode(GET_VIN_VOLTAGE);
        } else if (input == "vout") {
            Serial.println("Getting Vout voltage...");
            canBus.SendCode(GET_VOUT_VOLTAGE);
        } else if (input == "current1") {
            Serial.println("Getting Phase 1 current...");
            canBus.SendCode(GET_PHASE1_CURRENT);
        } else if (input == "current2") {
            Serial.println("Getting Phase 2 current...");
            canBus.SendCode(GET_PHASE2_CURRENT);
        } else if (input == "temp1") {
            Serial.println("Getting Phase 1 temperature...");
            canBus.SendCode(GET_PHASE1_TEMP);
        } else if (input == "temp2") {
            Serial.println("Getting Phase 2 temperature...");
            canBus.SendCode(GET_PHASE2_TEMP);
        } else if (input == "airflow") {
            Serial.println("Getting fan airflow...");
            canBus.SendCode(GET_FAN_AIRFLOW);
        } else if (input == "rpm") {
            Serial.println("Getting fan RPM...");
            canBus.SendCode(GET_FAN_RPM);
        } else if (input == "power") {
            Serial.println("Getting power...");
            canBus.SendCode(GET_POWER);
        } else if (input == "canstatus") {
            Serial.println("Getting CAN fault status...");
            canBus.SendCode(GET_CAN_FAULT_STATUS);
        } else if (input == "heartbeatmode") {
            Serial.println("Enabling heartbeat mode...");
            canBus.SendCode(ENABLE_HEARTBEAT);
        } else if (input == "help") {
            printHelpMessage();
        } else {
            Serial.println("Unknown command. Type 'help' for a list of commands.");
        }
        delay(50);
    }
}

// Callback for handling received CAN messages
void OnMessage(const CANMessage& msg) {
    MessageCode code = static_cast<MessageCode>(msg.id);

    switch (code) {
        case HEARTBEAT_PING:
            Serial.println("Heartbeat ping received.");
            break;
        case ENABLE_OUTPUT:
            Serial.println("Output enabled.");
            break;
        case REGULAR_STOP:
            Serial.println("Output disabled.");
            break;
        case EMERGENCY_STOP:
            Serial.println("Emergency stop activated.");
            break;
        case RESET_FAULTS:
            Serial.println("Faults reset.");
            break;
        case GET_FAULT_COUNT:
            Serial.print("Number of faults: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_FAULT_LIST:
            Serial.print("Fault list: ");
            Serial.println((char*)msg.data);
            break;
        case GET_VIN_VOLTAGE:
            Serial.print("Vin Voltage: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_VOUT_VOLTAGE:
            Serial.print("Vout Voltage: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_PHASE1_CURRENT:
            Serial.print("Phase 1 Current: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_PHASE2_CURRENT:
            Serial.print("Phase 2 Current: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_PHASE1_TEMP:
            Serial.print("Phase 1 Temperature: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_PHASE2_TEMP:
            Serial.print("Phase 2 Temperature: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_FAN_AIRFLOW:
            Serial.print("Fan Airflow: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_FAN_RPM:
            Serial.print("Fan RPM: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_POWER:
            Serial.print("Power: ");
            Serial.println(CANBusManager::ReadInt(msg));
            break;
        case GET_CAN_FAULT_STATUS:
            Serial.print("CAN Fault Status: ");
            Serial.println(CANBusManager::ReadBool(msg) ? "Error" : "OK");
            break;
        default:
            Serial.println("Unknown message received.");
            break;
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
