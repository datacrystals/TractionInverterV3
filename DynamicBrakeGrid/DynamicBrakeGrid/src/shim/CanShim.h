#pragma once 

#include <stdint.h>

typedef unsigned char __u8;
typedef unsigned short __u16;
typedef unsigned long __u32;
typedef __u32 canid_t;

#define CAN_MAX_DLEN 8

struct can_frame {
    canid_t can_id;  /* 32 bit CAN_ID + EFF/RTR/ERR flags */
    __u8    can_dlc; /* frame payload length in byte (0 .. CAN_MAX_DLEN) */
    __u8    data[CAN_MAX_DLEN] __attribute__((aligned(8)));
};

