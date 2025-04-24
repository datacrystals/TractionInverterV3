#ifndef MESSAGE_CODES_H
#define MESSAGE_CODES_H

enum MessageCode : uint8_t {
    ENABLE = 0x01,
    VOLTAGE = 0x02,

    HEARTBEAT = 0x03,
    TOGGLE_LED = 0x04,
    TEMPERATURE = 0x05,
    STATUS_FLOAT = 0x06,
    DESCRIPTION = 0x07,
    
    STRING_START = 0x10,
    STRING_PART = 0x11,
    STRING_END = 0x12
};

#endif
