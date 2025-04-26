#include "CANFramework/CANBusManager.cpp"
#include "CANFramework/MessageCode.h"

// CAN Pins
#define CAN_CS_PIN 10
#define CAN_INT_PIN 2
#define DEVICE_ID 0x02

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
    MessageCode code = CANBusManager::GetMessageCode(msg);
    Serial.print(F("Received CAN message with code: "));
    Serial.println(code);
    Serial.print(F("int: "));
    Serial.println((int)code);
    switch (code) {
        case MessageCode::GET_VIN_VOLTAGE:
            inputVoltage = CANBusManager::ReadFloat(msg);
            newDataAvailable = true;            
            break;

        case MessageCode::TEMPERATURE:
            temperature = CANBusManager::ReadFloat(msg);
            newDataAvailable = true;            
            break;

        case MessageCode::GET_POWER:
            power = CANBusManager::ReadFloat(msg);
            newDataAvailable = true;            
            break;

        case MessageCode::GET_FAN_AIRFLOW:
            fanSpeed = CANBusManager::ReadFloat(msg);
            newDataAvailable = true;            
            break;

        case MessageCode::GET_FAULT_COUNT:
            uint32_t faultCount = CANBusManager::ReadInt(msg);
            Serial.print(F("Fault Count: "));
            Serial.println(faultCount);
            break;

        case MessageCode::GET_FAULT_LIST:
            if (msg.length > 0) {
                Serial.print(F("Fault List: "));
                Serial.write(msg.data, msg.length);
                Serial.println();
            }
            break;

        case MessageCode::RESET_FAULTS:
            Serial.println(F("Faults reset acknowledged"));
            break;

        case MessageCode::HEARTBEAT:
            Serial.println(F("Heartbeat received"));
            break;

        case MessageCode::GET_VOUT_VOLTAGE:
            float voutVoltage = CANBusManager::ReadFloat(msg);
            Serial.print(F("Vout Voltage: "));
            Serial.println(voutVoltage, 2);            
            break;

        case MessageCode::GET_PHASE1_CURRENT:
            float phase1Current = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 1 Current: "));
            Serial.println(phase1Current, 2);            
            break;

        case MessageCode::GET_PHASE2_CURRENT:
            float phase2Current = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 2 Current: "));
            Serial.println(phase2Current, 2);            
            break;

        case MessageCode::GET_PHASE1_TEMP:
            float phase1Temp = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 1 Temperature: "));
            Serial.println(phase1Temp, 2);            
            break;

        case MessageCode::GET_PHASE2_TEMP:
            float phase2Temp = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 2 Temperature: "));
            Serial.println(phase2Temp, 2);            
            break;

        case MessageCode::GET_CAN_FAULT_STATUS:
            uint32_t canFaultStatus = CANBusManager::ReadInt(msg);
            Serial.print(F("CAN Fault Status: "));
            Serial.println(canFaultStatus);            
            break;

        case MessageCode::GET_FAN_SPEED:
            float fanSpeedCFM = CANBusManager::ReadFloat(msg);
            Serial.print(F("Fan Speed (CFM): "));
            Serial.println(fanSpeedCFM, 2);            
            break;

        case MessageCode::REQUEST_FIRMWARE_VERSION:
            Serial.println(F("Firmware version request received"));
            break;

        case MessageCode::FIRMWARE_VERSION_RESPONSE:
            if (msg.length > 0) {
                char firmwareVersion[9] = {0};
                memcpy(firmwareVersion, msg.data, msg.length);
                Serial.print(F("Firmware Version: "));
                Serial.println(firmwareVersion);
            }
            break;

        case MessageCode::GET_FAULT_COUNT_RESPONSE:
            uint32_t faultCount_response = CANBusManager::ReadInt(msg);
            Serial.print(F("Fault Count Response: "));
            Serial.println(faultCount_response);            
            break;

        case MessageCode::GET_VIN_VOLTAGE_RESPONSE:
            Serial.println(F("case MessageCode::GET_VIN_VOLTAGE_RESPONSE - start")); 
            Delay(100);
            float vinVoltage = CANBusManager::ReadFloat(msg);
            Delay(100);
            Serial.print(F("Vin Voltage Response: "));
            Delay(100);
            Serial.println(vinVoltage, 2);    
            Delay(100);
            Serial.println(F("case MessageCode::GET_VIN_VOLTAGE_RESPONSE - end"));        
            Delay(100);
            break;

        case MessageCode::GET_VOUT_VOLTAGE_RESPONSE:
            float voutVoltage_response = CANBusManager::ReadFloat(msg);
            Serial.print(F("Vout Voltage Response: "));
            Serial.println(voutVoltage_response, 2);            
            break;

        case MessageCode::GET_PHASE1_CURRENT_RESPONSE:
            float phase1Current_response = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 1 Current Response: "));
            Serial.println(phase1Current_response, 2);
            break;

        case MessageCode::GET_PHASE2_CURRENT_RESPONSE:
            float phase2Current_response = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 2 Current Response: "));
            Serial.println(phase2Current_response, 2);            
            break;

        case MessageCode::GET_PHASE1_TEMP_RESPONSE:
            float phase1Temp_response = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 1 Temperature Response: "));
            Serial.println(phase1Temp_response, 2);            
            break;

        case MessageCode::GET_PHASE2_TEMP_RESPONSE:
            float phase2Temp_response = CANBusManager::ReadFloat(msg);
            Serial.print(F("Phase 2 Temperature Response: "));
            Serial.println(phase2Temp_response, 2);
            break;

        case MessageCode::GET_POWER_RESPONSE:
            float power = CANBusManager::ReadFloat(msg);
            Serial.print(F("Power Response: "));
            Serial.println(power, 2);
            break;

        case MessageCode::GET_FAN_SPEED_RESPONSE:
            float fanSpeed = CANBusManager::ReadFloat(msg);
            Serial.print(F("Fan Speed Response: "));
            Serial.println(fanSpeed, 2);
            break;

        default:
            Serial.println(String(F("Unhandled CAN message: ")) + String((int)code));
            break;
    }
}

void OnCanStringMessage(MessageCode code, const char* message) {
    Serial.print(F("Received string message with code "));
    Serial.print((int)code);
    Serial.print(F(": "));
    Serial.println(message);
}

void setup() {
    Serial.begin(9600);
    while (!Serial);

    canManager.Begin();
    canManager.SetCallback(handleCANMessage);
    canManager.SetStringCallback(OnCanStringMessage); // Register the string callback

    Serial.println(F("Test Harness Ready"));
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
    Serial.print(F("Vin: "));
    Serial.print(inputVoltage, 2);
    Serial.print(F("V | Temp: "));
    Serial.print(temperature, 1);
    Serial.print(F("°C | Power: "));
    Serial.print(power, 2);
    Serial.print(F("W | Fan: "));
    Serial.print(fanSpeed, 0);
    Serial.println(F(" RPM"));
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
    else if (cmd == "faultlist") {
        canManager.SendCode(MessageCode::GET_FAULT_LIST);
    }
    else if (cmd == "vout") {
        canManager.SendCode(MessageCode::GET_VOUT_VOLTAGE);
    }
    else if (cmd == "current1") {
        canManager.SendCode(MessageCode::GET_PHASE1_CURRENT);
    }
    else if (cmd == "current2") {
        canManager.SendCode(MessageCode::GET_PHASE2_CURRENT);
    }
    else if (cmd == "temp1") {
        canManager.SendCode(MessageCode::GET_PHASE1_TEMP);
    }
    else if (cmd == "temp2") {
        canManager.SendCode(MessageCode::GET_PHASE2_TEMP);
    }
    else if (cmd == "rpm") {
        canManager.SendCode(MessageCode::GET_FAN_RPM);
    }
    else if (cmd == "canstatus") {
        canManager.SendCode(MessageCode::GET_CAN_FAULT_STATUS);
    }
    else if (cmd == "heartbeatmode") {
        canManager.SendCode(MessageCode::ENABLE_HEARTBEAT);
    }
    else if (cmd == "ping") {
        canManager.SendCode(MessageCode::HEARTBEAT_PING);
    }
    else if (cmd == "poll") {
        pollingActive = !pollingActive; // Toggle polling state
        Serial.print(F("Polling "));
        Serial.println(pollingActive ? F("started") : F("stopped"));
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
        Serial.println(F("Unknown command. Type 'help' for options."));
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
    Serial.println(F("Available Commands:"));
    Serial.println(F("  enable       - Enable output"));
    Serial.println(F("  disable      - Disable output"));
    Serial.println(F("  emergency    - Emergency stop"));
    Serial.println(F("  reset        - Reset faults"));
    Serial.println(F("  faults       - Get fault count"));
    Serial.println(F("  vin          - Get input voltage"));
    Serial.println(F("  temp         - Get temperature"));
    Serial.println(F("  power        - Get power"));
    Serial.println(F("  fan          - Get fan speed"));
    Serial.println(F("  poll         - Toggle continuous polling of stats"));
    Serial.println(F("  help         - Show this help"));
}
