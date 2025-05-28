// CANBusManagerPico.cpp
// Standalone implementation of CANBusManager for Raspberry Pi Pico using MCP2515

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <string.h>
#include <stdio.h>

#define TEST_CAN // Uncomment to disable test script

// MCP2515 Register and SPI command definitions
#define MCP2515_RESET       0xC0
#define MCP2515_READ        0x03
#define MCP2515_WRITE       0x02
#define MCP2515_RTS         0x80
#define MCP2515_READ_STATUS 0xA0

#define MCP_CANCTRL         0x0F
#define MCP_CANSTAT         0x0E
#define MCP_CANINTE         0x2B
#define MCP_CANINTF         0x2C
#define MCP_RXB0CTRL        0x60
#define MCP_RXB0SIDH        0x61
#define MCP_RXB0SIDL        0x62
#define MCP_RXB0DLC         0x65
#define MCP_RXB0DATA        0x66
#define MCP_TXB0CTRL        0x30
#define MCP_TXB0SIDH        0x31
#define MCP_TXB0SIDL        0x32
#define MCP_TXB0DLC         0x35
#define MCP_TXB0DATA        0x36

#define MCP_MODE_NORMAL     0x00
#define MCP_MODE_CONFIG     0x80

#define MAX_DATA_BYTES      8

struct CANFrame {
    uint16_t id;
    uint8_t length;
    uint8_t data[MAX_DATA_BYTES];
};

class CANBusManager {
public:
    CANBusManager(spi_inst_t* spi, uint csPin);
    void begin();
    bool send(const CANFrame& frame);
    bool receive(CANFrame& frame);

private:
    spi_inst_t* spi;
    uint csPin;

    void select();
    void deselect();
    void reset();
    void writeRegister(uint8_t address, uint8_t value);
    uint8_t readRegister(uint8_t address);
    void bitModify(uint8_t address, uint8_t mask, uint8_t data);
    uint8_t readStatus();
};

CANBusManager::CANBusManager(spi_inst_t* spi, uint csPin) : spi(spi), csPin(csPin) {}

void CANBusManager::select() {
    gpio_put(csPin, 0);
}

void CANBusManager::deselect() {
    gpio_put(csPin, 1);
}

void CANBusManager::reset() {
    select();
    uint8_t cmd = MCP2515_RESET;
    spi_write_blocking(spi, &cmd, 1);
    deselect();
    sleep_ms(10);
}

void CANBusManager::writeRegister(uint8_t address, uint8_t value) {
    select();
    uint8_t buf[3] = { MCP2515_WRITE, address, value };
    spi_write_blocking(spi, buf, 3);
    deselect();
}

uint8_t CANBusManager::readRegister(uint8_t address) {
    select();
    uint8_t buf[3] = { MCP2515_READ, address, 0x00 };
    spi_write_read_blocking(spi, buf, buf, 3);
    deselect();
    return buf[2];
}

void CANBusManager::bitModify(uint8_t address, uint8_t mask, uint8_t data) {
    select();
    uint8_t buf[4] = { 0x05, address, mask, data };
    spi_write_blocking(spi, buf, 4);
    deselect();
}

uint8_t CANBusManager::readStatus() {
    select();
    uint8_t cmd = MCP2515_READ_STATUS;
    uint8_t res[2];
    spi_write_read_blocking(spi, &cmd, res, 2);
    deselect();
    return res[1];
}

void CANBusManager::begin() {
    spi_init(spi, 1000 * 1000);
    gpio_set_function(2, GPIO_FUNC_SPI);
    gpio_set_function(3, GPIO_FUNC_SPI);
    gpio_set_function(4, GPIO_FUNC_SPI);

    gpio_init(csPin);
    gpio_set_dir(csPin, GPIO_OUT);
    gpio_put(csPin, 1);

    reset();

    writeRegister(MCP_CANCTRL, MCP_MODE_CONFIG);
    writeRegister(MCP_CANCTRL, MCP_MODE_NORMAL);
}

bool CANBusManager::send(const CANFrame& frame) {
    uint8_t sidh = (frame.id >> 3) & 0xFF;
    uint8_t sidl = (frame.id << 5) & 0xE0;

    writeRegister(MCP_TXB0SIDH, sidh);
    writeRegister(MCP_TXB0SIDL, sidl);
    writeRegister(MCP_TXB0DLC, frame.length);
    for (uint8_t i = 0; i < frame.length; i++) {
        writeRegister(MCP_TXB0DATA + i, frame.data[i]);
    }

    select();
    uint8_t rts = MCP2515_RTS | 0x01;
    spi_write_blocking(spi, &rts, 1);
    deselect();

    return true;
}

bool CANBusManager::receive(CANFrame& frame) {
    uint8_t status = readStatus();
    if (!(status & 0x01)) return false; // RX0IF not set

    uint8_t sidh = readRegister(MCP_RXB0SIDH);
    uint8_t sidl = readRegister(MCP_RXB0SIDL);
    frame.id = (sidh << 3) | (sidl >> 5);
    frame.length = readRegister(MCP_RXB0DLC) & 0x0F;
    for (uint8_t i = 0; i < frame.length; i++) {
        frame.data[i] = readRegister(MCP_RXB0DATA + i);
    }

    bitModify(MCP_CANINTF, 0x01, 0x00); // Clear RX0IF
    return true;
}

// Test script (main.cpp)
#ifdef TEST_CAN
int main() {
    stdio_init_all();
    CANBusManager can(spi0, 5); // CS pin 5
    can.begin();

    CANFrame tx = { 0x123, 2, { 0xDE, 0xAD } };
    can.send(tx);

    while (true) {
        CANFrame rx;
        if (can.receive(rx)) {
            printf("Received ID: 0x%03X, Length: %d, Data:", rx.id, rx.length);
            for (int i = 0; i < rx.length; i++) {
                printf(" %02X", rx.data[i]);
            }
            printf("\n");
        }
        sleep_ms(100);
    }
    return 0;
}
#endif
