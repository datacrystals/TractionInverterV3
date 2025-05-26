// ucc5870_driver.c
// SPI driver for UCC5870-Q1 gate driver using Raspberry Pi Pico (RP2040)

#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>
#include <string.h>

// ============================
// SPI and GPIO configuration
// ============================
#define SPI_PORT spi0
#define PIN_MISO 3
#define PIN_CS   9
#define PIN_SCK  2
#define PIN_MOSI 4

// ============================
// SPI Initialization
// ============================
void ucc5870_spi_init() {
    spi_init(SPI_PORT, 4 * 1000 * 1000); // 4 MHz max
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS,   GPIO_FUNC_SIO);
    gpio_set_function(PIN_SCK,  GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1); // CS inactive
}

// ============================
// SPI Transaction (16-bit)
// ============================
uint16_t ucc5870_spi_tx(uint16_t data) {
    uint8_t tx_buf[2] = { data >> 8, data & 0xFF };
    uint8_t rx_buf[2] = { 0 };

    gpio_put(PIN_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    gpio_put(PIN_CS, 1);

    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

// ============================
// Register Access Wrappers
// ============================
void ucc5870_write_register(uint8_t reg, uint16_t value) {
    ucc5870_spi_tx(0xC000 | reg);         // WR_RA
    ucc5870_spi_tx(0xA000 | (value >> 8));// WRH
    ucc5870_spi_tx(0xB000 | (value & 0xFF)); // WRL
}

uint16_t ucc5870_read_register(uint8_t reg) {
    ucc5870_spi_tx(0x8800 | reg); // RD_DATA
    return ucc5870_spi_tx(0x2A2); // NOP to clock out data
}

// ============================
// CRC8 Computation (Polynomial: x^8 + x^2 + x + 1)
// ============================
uint8_t crc8_calc(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// ============================
// Full Configuration
// ============================
void ucc5870_configure() {
    ucc5870_write_register(0x00, 0x582A); // CFG1 (example)
    ucc5870_write_register(0x01, 0x0000); // CFG2
    ucc5870_write_register(0x02, 0x0000); // CFG3
    ucc5870_write_register(0x03, 0x0000); // CFG4
    ucc5870_write_register(0x04, 0x0000); // CFG5
    ucc5870_write_register(0x05, 0x0000); // CFG6
    ucc5870_write_register(0x06, 0x0000); // CFG7
    ucc5870_write_register(0x07, 0x0000); // CFG8
    ucc5870_write_register(0x08, 0x0000); // CFG9
    ucc5870_write_register(0x09, 0x0000); // CFG10
    ucc5870_write_register(0x0A, 0x0000); // CONTROL1
    ucc5870_write_register(0x0B, 0x0000); // CONTROL2
    ucc5870_write_register(0x0C, 0x0000); // CONTROL3

    // Compute CRC from written config (stubbed data)
    uint8_t config_data[] = {
        0x58, 0x2A, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00
    };
    uint8_t crc = crc8_calc(config_data, sizeof(config_data));
    ucc5870_write_register(0x13, crc);
}

// ============================
// Initialization Sequence
// ============================
void ucc5870_init() {
    ucc5870_spi_tx(0x02C8); // Software reset
    sleep_ms(10);

    ucc5870_spi_tx(0x0228); // Enter CFG_IN
    sleep_ms(1);

    ucc5870_configure();    // Configure registers and CRC

    ucc5870_spi_tx(0x0209); // Enable driver
}

// ============================
// Print Key Status Bits
// ============================
void ucc5870_print_status() {
    uint16_t status1 = ucc5870_read_register(0x16);
    uint16_t status2 = ucc5870_read_register(0x17);
    uint16_t status4 = ucc5870_read_register(0x19);

    uint8_t opm = (status1 >> 12) & 0xF;
    bool pri_rdy = (status2 >> 6) & 0x1;
    bool sec_rdy = (status4 >> 6) & 0x1;

    bool bist_pri_fault = (status2 >> 3) & 0x1;
    bool bist_sec_fault = (status4 >> 3) & 0x1;

    bool trim_crc_pri_fault = (status2 >> 2) & 0x1;
    bool trim_crc_sec_fault = (status4 >> 2) & 0x1;

    bool cfg_crc_pri_fault = (status2 >> 1) & 0x1;
    bool cfg_crc_sec_fault = (status4 >> 1) & 0x1;

    printf("\n=== UCC5870 STATUS ===\n");
    printf("STATUS1[OPM]: %d\n", opm);
    printf("STATUS2[PRI_RDY]: %d\n", pri_rdy);
    printf("STATUS4[SEC_RDY]: %d\n", sec_rdy);
    printf("STATUS2[BIST_PRI_FAULT]: %d\n", bist_pri_fault);
    printf("STATUS4[BIST_SEC_FAULT]: %d\n", bist_sec_fault);
    printf("STATUS2[TRIM_CRC_PRI_FAULT]: %d\n", trim_crc_pri_fault);
    printf("STATUS4[TRIM_CRC_SEC_FAULT]: %d\n", trim_crc_sec_fault);
    printf("STATUS2[CFG_CRC_PRI_FAULT]: %d\n", cfg_crc_pri_fault);
    printf("STATUS4[CFG_CRC_SEC_FAULT]: %d\n", cfg_crc_sec_fault);
}

// ============================
// Gate Drive Test and Fault Handling
// ============================
void ucc5870_test_drive() {
    gpio_put(25, 1); // Example activity indicator (LED)
    while (1) {
        ucc5870_print_status();
        sleep_ms(1000);
    }
}

// ============================
// Main Application
// ============================
int main() {
    stdio_init_all();
    ucc5870_spi_init();
    gpio_init(25); gpio_set_dir(25, GPIO_OUT);

    ucc5870_init();
    ucc5870_test_drive();
    return 0;
}
