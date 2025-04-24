#include "CANFramework/src/CANBusManager.h"
#include "CANFramework/src/MessageCode.h"

// CAN Pins
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2
#define DEVICE_ID 0x01

CANBusManager canManager(DEVICE_ID, CAN_CS_PIN, CAN_INT_PIN);

volatile bool canMessageReceived = false;
bool pollingActive = false;
unsigned long lastPollTime = 0;
const unsigned long pollInterval = 500; // 500ms = 0.5 second

// Variables to store polled values
float inputVoltage = 0.0;
float temperature = 0.0;
float power = 0.0;
float fanSpeed = 0.0;
bool newDataAvailable = false;

void handleCANMessage(const CANMessage& msg) {
    switch (msg.code) {
        case MessageCode::GET_VIN_VOLTAGE:
            if (msg.length == 4) {
                memcpy(&inputVoltage, msg.data, 4);
                newDataAvailable = true;
            }
            break;

        case MessageCode::TEMPERATURE:
            if (msg.length == 4) {
                memcpy(&temperature, msg.data, 4);
                newDataAvailable = true;
            }
            break;

        case MessageCode::GET_POWER:
            if (msg.length == 4) {
                memcpy(&power, msg.data, 4);
                newDataAvailable = true;
            }
            break;

        case MessageCode::GET_FAN_AIRFLOW:
            if (msg.length == 4) {
                memcpy(&fanSpeed, msg.data, 4);
                newDataAvailable = true;
            }
            break;

        case MessageCode::GET_FAULT_COUNT:
            if (msg.length == 4) {
                uint32_t faultCount;
                memcpy(&faultCount, msg.data, 4);
                Serial.print("Fault Count: ");
                Serial.println(faultCount);
            }
            break;

        case MessageCode::GET_FAULT_LIST:
            if (msg.length > 0) {
                Serial.print("Fault List: ");
                Serial.write(msg.data, msg.length);
                Serial.println();
            }
            break;

        case MessageCode::RESET_FAULTS:
            Serial.println("Faults reset acknowledged");
            break;

        case MessageCode::HEARTBEAT:
            Serial.println("Heartbeat received");
            break;

        default:
            Serial.println("Unhandled CAN message");
            break;
    }
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    canManager.Begin();
    canManager.SetCallback(handleCANMessage);

    Serial.println("Test Harness Ready");
    printHelp();
}

void loop() {
    if (Serial.available()) {
        String input = Serial.readStringUntil('\n');
        input.trim();
        processCommand(input);
    }

    canManager.Update();

    // Handle polling if active
    if (pollingActive && millis() - lastPollTime >= pollInterval) {
        pollStats();
        lastPollTime = millis();
    }

    // Print collected data if new data is available
    if (newDataAvailable) {
        printPolledData();
        newDataAvailable = false;
    }
}

void printPolledData() {
    Serial.print("Vin: ");
    Serial.print(inputVoltage, 2);
    Serial.print("V | Temp: ");
    Serial.print(temperature, 1);
    Serial.print("°C | Power: ");
    Serial.print(power, 2);
    Serial.print("W | Fan: ");
    Serial.print(fanSpeed, 0);
    Serial.println(" RPM");
}

void processCommand(String cmd) {
    if (cmd == "help") {
        printHelp();
    } 
    else if (cmd == "enable") {
        canManager.SendCode(MessageCode::ENABLE_OUTPUT);
    }
    else if (cmd == "disable") {
        canManager.SendCode(MessageCode::REGULAR_STOP);
    }
    else if (cmd == "emergency") {
        canManager.SendCode(MessageCode::EMERGENCY_STOP);
    }
    else if (cmd == "reset") {
        canManager.SendCode(MessageCode::RESET_FAULTS);
    }
    else if (cmd == "faults") {
        canManager.SendCode(MessageCode::GET_FAULT_COUNT);
    }
    else if (cmd == "vin") {
        canManager.SendCode(MessageCode::GET_VIN_VOLTAGE);
    }
    else if (cmd == "temp") {
        canManager.SendCode(MessageCode::TEMPERATURE);
    }
    else if (cmd == "power") {
        canManager.SendCode(MessageCode::GET_POWER);
    }
    else if (cmd == "fan") {
        canManager.SendCode(MessageCode::GET_FAN_AIRFLOW);
    }
    else if (cmd == "poll") {
        pollingActive = !pollingActive; // Toggle polling state
        Serial.print("Polling ");
        Serial.println(pollingActive ? "started" : "stopped");
        if (pollingActive) {
            lastPollTime = millis() - pollInterval; // Force immediate poll
            // Clear previous values
            inputVoltage = 0;
            temperature = 0;
            power = 0;
            fanSpeed = 0;
        }
    }
    else {
        Serial.println("Unknown command. Type 'help' for options.");
    }
}

void pollStats() {
    // Request all stats
    canManager.SendCode(MessageCode::GET_VIN_VOLTAGE);
    canManager.SendCode(MessageCode::TEMPERATURE);
    canManager.SendCode(MessageCode::GET_POWER);
    canManager.SendCode(MessageCode::GET_FAN_AIRFLOW);
}

void printHelp() {
    Serial.println("Available Commands:");
    Serial.println("  enable       - Enable output");
    Serial.println("  disable      - Disable output");
    Serial.println("  emergency    - Emergency stop");
    Serial.println("  reset        - Reset faults");
    Serial.println("  faults       - Get fault count");
    Serial.println("  vin          - Get input voltage");
    Serial.println("  temp         - Get temperature");
    Serial.println("  power        - Get power");
    Serial.println("  fan          - Get fan speed");
    Serial.println("  poll         - Toggle continuous polling of stats");
    Serial.println("  help         - Show this help");
}
