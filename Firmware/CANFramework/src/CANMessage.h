#ifndef CAN_MESSAGE_H
#define CAN_MESSAGE_H

#include <stdint.h>

struct CANMessage {
    uint16_t id;
    uint8_t length;
    uint8_t data[8];
};

#endif