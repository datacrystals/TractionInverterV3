#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <stdio.h>

// === PIN DEFINITIONS ===
#define PIN_SPI_SCK     10  // GPIO10
#define PIN_SPI_MOSI    11  // GPIO11
#define PIN_SPI_MISO    12  // GPIO12
#define PIN_SPI_CS      9   // GPIO9

// === SPI SETTINGS ===
#define SPI_PORT spi0
#define SPI_BAUDRATE 1 * 1000 * 1000  // 1 MHz

// === ADS8688A CHANNEL SELECT COMMANDS ===
#define CMD_MAN_CH0 0xC000
#define CMD_MAN_CH1 0xC400
#define CMD_MAN_CH2 0xC800
#define CMD_MAN_CH3 0xCC00
#define CMD_MAN_CH4 0xD000
#define CMD_MAN_CH5 0xD400
#define CMD_MAN_CH6 0xD800
#define CMD_MAN_CH7 0xDC00

// === FUNCTION DECLARATIONS ===
void ads8688a_init();
void ads8688a_send_command(uint16_t command, uint8_t *rx_buf);
uint16_t ads8688a_read_channel(uint16_t command);
void ads8688a_read_all_channels();

// === GLOBALS ===
const uint16_t channel_cmds[8] = {
    CMD_MAN_CH0, CMD_MAN_CH1, CMD_MAN_CH2, CMD_MAN_CH3,
    CMD_MAN_CH4, CMD_MAN_CH5, CMD_MAN_CH6, CMD_MAN_CH7
};

const char *channel_labels[8] = {
    "AIN_0P (pin 16)", "AIN_1P (pin 18)", "AIN_2P (pin 21)", "AIN_3P (pin 23)",
    "AIN_4P (pin 25)", "AIN_5P (pin 27)", "AIN_6P (pin 12)", "AIN_7P (pin 14)"
};

// === FUNCTION IMPLEMENTATIONS ===
void ads8688a_init() {
    spi_init(SPI_PORT, SPI_BAUDRATE);
    gpio_set_function(PIN_SPI_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SPI_MISO, GPIO_FUNC_SPI);

    gpio_init(PIN_SPI_CS);
    gpio_set_dir(PIN_SPI_CS, GPIO_OUT);
    gpio_put(PIN_SPI_CS, 1); // Deselect chip
}

void ads8688a_send_command(uint16_t command, uint8_t *rx_buf) {
    uint8_t tx_buf[2] = { (command >> 8) & 0xFF, command & 0xFF };
    gpio_put(PIN_SPI_CS, 0);
    spi_write_read_blocking(SPI_PORT, tx_buf, rx_buf, 2);
    gpio_put(PIN_SPI_CS, 1);
}

uint16_t ads8688a_read_channel(uint16_t command) {
    uint8_t rx_buf[2];
    ads8688a_send_command(command, rx_buf);
    return ((uint16_t)rx_buf[0] << 8) | rx_buf[1];
}

void ads8688a_read_all_channels() {
    printf("Reading ADS8688A Channels:\n");
    for (int i = 0; i < 8; ++i) {
        uint16_t value = ads8688a_read_channel(channel_cmds[i]);
        printf("  Channel %d [%s] = %u\n", i, channel_labels[i], value);
    }
    printf("\n");
}

// === MAIN ===
int main() {
    stdio_init_all();
    sleep_ms(1000);  // Wait for USB serial to come up

    printf("ADS8688A SPI Test\n");

    ads8688a_init();

    while (true) {
        ads8688a_read_all_channels();
        sleep_ms(1000);
    }

    return 0;
}
