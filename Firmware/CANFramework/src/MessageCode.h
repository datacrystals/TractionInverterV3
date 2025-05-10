#ifndef MESSAGE_CODES_H
#define MESSAGE_CODES_H

enum MessageCode : uint8_t {

    MSG_NULL = 0, // No message code
    EMPTY = 1,

    // Existing codes
    ENABLE = 2,
    VOLTAGE = 3,
    HEARTBEAT = 4,
    TOGGLE_LED = 5,
    TEMPERATURE = 6,
    STATUS_FLOAT = 7,
    DESCRIPTION = 8,
    
    STRING_START = 10,
    STRING_PART = 11,
    STRING_END = 12,

    API_VERSION_REQUEST = 13,
    API_VERSION_RESPONSE = 14,

    // BOOST Converter things
    EMERGENCY_STOP   = 20,     // Emergency Stop (Disable Output)
    REGULAR_STOP     = 21,     // Regular Stop (Disable Output)
    ENABLE_OUTPUT    = 22,     // Enable Output
    RESET_FAULTS     = 23,     // Reset Faults
    GET_FAULT_COUNT  = 24,     // Get Number of Faults
    GET_FAULT_LIST   = 25,     // Get Faults List
    GET_VIN_VOLTAGE  = 26,     // Get Vin Voltage
    GET_VOUT_VOLTAGE = 27,     // Get Vout Voltage
    GET_PHASE1_CURRENT = 28,   // Get Phase 1 Current
    GET_PHASE2_CURRENT = 29,   // Get Phase 2 Current
    GET_PHASE1_TEMP = 30,      // Get Phase 1 Temperature
    GET_PHASE2_TEMP = 31,      // Get Phase 2 Temperature
    GET_FAN_AIRFLOW = 32,      // Get Fan Airflow (LFM)
    GET_FAN_RPM     = 33,      // Get Fan RPM
    GET_POWER       = 34,      // Get Power
    GET_CAN_FAULT_STATUS = 35, // Get CAN Fault Status
    ENABLE_HEARTBEAT = 36,     // Enable Heartbeat Mode
    HEARTBEAT_PING = 37,       // Heartbeat Ping
    GET_FAULT_LIST_RESPONSE = 38, // Faults List Response
    GET_FAN_SPEED = 39, // Get Fan Speed (CFM)

    // DYNAMIC BRAKE things
    DISABLE_OUTPUT = 40,
    GET_VIN = 42,
    GET_VOUT = 43,
    SET_VOLTAGE = 44,

    // Add equivalents for CAN IDs
    REQUEST_STATUS = 50,      // 0x50 - Request Status
    STATUS_MESSAGE = 51,      // 0x51 - Status Message
    FAULT_MESSAGE = 52,       // 0x52 - Fault Message
    SYSTEM_INFO = 53,         // 0x53 - System Info
    SET_VOLTAGE_SETPOINT = 54,// 0x54 - Set Voltage Setpoint
    RESET_FAULTS_CMD = 55,    // 0x55 - Reset Faults    
    SYSTEM_COMMAND = 56,      // 0x56 - System Command
    REQUEST_FIRMWARE_VERSION = 57,    // 0x57 - Firmware Version
    FIRMWARE_VERSION_RESPONSE = 58,   // 0x58 - Firmware Version Response
    GET_FAULT_COUNT_RESPONSE = 59,    // 0x59 - Fault Count Response
    GET_VIN_VOLTAGE_RESPONSE = 60,    // 60 - Vin Voltage Response
    GET_VOUT_VOLTAGE_RESPONSE = 61,   // 61 - Vout Voltage Response
    GET_PHASE1_CURRENT_RESPONSE = 62, // 62 - Phase 1 Current Response
    GET_PHASE2_CURRENT_RESPONSE = 63, // 0x63 - Phase 2 Current Response
    GET_PHASE1_TEMP_RESPONSE = 64,    // 0x64 - Phase 1 Temperature Response
    GET_PHASE2_TEMP_RESPONSE = 65,    // 0x65 - Phase 2 Temperature Response
    GET_POWER_RESPONSE = 66,            // 0x66 - Power Response
    GET_FAN_SPEED_RESPONSE = 67,       // 0x67 - Fan Speed Response
};

#endif
