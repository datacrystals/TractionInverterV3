#ifndef MESSAGE_CODES_H
#define MESSAGE_CODES_H

enum MessageCode : uint8_t {

    EMPTY = 1,

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
    // END BOOST Converter things

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
};

#endif
