/**
 * @file ucc5870_interface.c
 * @brief Interface to UCC5870-Q1 gate driver via SPI using Raspberry Pi Pico.
 */

 #include <stdio.h>
 #include "pico/stdlib.h"
 #include "hardware/spi.h"
 #include "hardware/gpio.h"
 #include "hardware/pwm.h"
 
 #define SPI_PORT spi0
 #define PIN_SCK  2
 #define PIN_MISO 3
 #define PIN_MOSI 4
 #define PIN_CS   9 // Chip Select pin W Low
 
 #define PWM_OUTH  22 // Chip Select pin W High
 #define PWM_OUTL  19 // Chip Select pin W Low
 #define FAULT_PIN 26
 
 #define CHIP_ADDR      0x0
 #define CMD_WRITE      0x2
 #define CMD_READ       0x1
 #define CMD_DRV_EN     0x3
 #define CMD_DRV_DIS    0x4
 #define CMD_CFG_IN     0x5
 #define CMD_CFG_OUT    0x6
 
 #define REG_CFG6       0x26
 #define REG_CFG7       0x27
 #define REG_STATUS2    0x32
 #define REG_STATUS3    0x33
 #define REG_ADCDATA1   0x2A
 #define REG_ADCDATA2   0x2B
 
 static uint16_t pwm_top = 1249;
 
 void cs_select() {
     gpio_put(PIN_CS, 0);
     asm volatile("nop \n nop \n nop");
 }
 
 void cs_deselect() {
     asm volatile("nop \n nop \n nop");
     gpio_put(PIN_CS, 1);
     asm volatile("nop \n nop \n nop");
 }
 
 uint16_t spi_transfer16(uint16_t val) {
     uint8_t tx[2] = {val >> 8, val & 0xFF};
     uint8_t rx[2];
     cs_select();
     spi_write_read_blocking(SPI_PORT, tx, rx, 2);
     cs_deselect();
     return (rx[0] << 8) | rx[1];
 }
 
 void write_register(uint8_t reg, uint8_t val) {
     uint16_t cmd = (CHIP_ADDR << 12) | (CMD_WRITE << 8) | val;
     spi_transfer16(cmd);
 }
 
 uint8_t read_register(uint8_t reg) {
     uint16_t cmd = (CHIP_ADDR << 12) | (CMD_READ << 8) | reg;
     uint16_t resp = spi_transfer16(cmd);
     return resp & 0xFF;
 }
 
 void gate_driver_on() {
     spi_transfer16((CHIP_ADDR << 12) | (CMD_DRV_EN << 8));
 }
 
 void gate_driver_off() {
     spi_transfer16((CHIP_ADDR << 12) | (CMD_DRV_DIS << 8));
 }
 
 void enter_configuration() {
     spi_transfer16((CHIP_ADDR << 12) | (CMD_CFG_IN << 8));
     sleep_ms(1);
 }
 
 void exit_configuration() {
     spi_transfer16((CHIP_ADDR << 12) | (CMD_CFG_OUT << 8));
     sleep_ms(1);
 }
 
 void set_desat_threshold(float mV) {
     uint8_t value;
     if (mV <= 500) value = 0;
     else if (mV <= 750) value = 1;
     else if (mV <= 1000) value = 2;
     else value = 3;
     write_register(REG_CFG6, value << 2);
 }
 
 uint16_t read_adc(uint8_t channel) {
     if (channel == 1) return read_register(REG_ADCDATA1);
     if (channel == 2) return read_register(REG_ADCDATA2);
     return 0;
 }
 
 uint8_t read_supply_faults() {
     uint8_t vcc1_status = read_register(REG_STATUS2);
     uint8_t vcc2_status = read_register(REG_STATUS3);
     return (vcc1_status << 4) | (vcc2_status & 0x0F);
 }
 
 bool read_fault_pin(uint gpio) {
     return gpio_get(gpio);
 }
 
 void set_pwm_duty_cycle(uint gpio, float duty_cycle) {
     uint slice = pwm_gpio_to_slice_num(gpio);
     uint chan = pwm_gpio_to_channel(gpio);
     uint16_t level = (uint16_t)((duty_cycle / 100.0f) * (pwm_top + 1));
     pwm_set_chan_level(slice, chan, level);
 }
 
 void ucc5870_init() {
     stdio_init_all();
 
     spi_init(SPI_PORT, 1000 * 1000);
     gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
     gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
     gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
     gpio_init(PIN_CS);
     gpio_set_dir(PIN_CS, GPIO_OUT);
     cs_deselect();
     spi_set_format(SPI_PORT, 16, SPI_CPOL_0, SPI_CPHA_1, SPI_MSB_FIRST);
 
     gpio_set_function(PWM_OUTH, GPIO_FUNC_PWM);
     gpio_set_function(PWM_OUTL, GPIO_FUNC_PWM);
 
     uint slice_outh = pwm_gpio_to_slice_num(PWM_OUTH);
     uint slice_outl = pwm_gpio_to_slice_num(PWM_OUTL);
     pwm_config cfg = pwm_get_default_config();
     pwm_config_set_wrap(&cfg, pwm_top);
     pwm_config_set_clkdiv(&cfg, 1.0f);
     pwm_init(slice_outh, &cfg, true);
     if (slice_outl != slice_outh) pwm_init(slice_outl, &cfg, true);
 
     gpio_init(FAULT_PIN); gpio_set_dir(FAULT_PIN, GPIO_IN);
 }
 
 void start() {
     set_pwm_duty_cycle(PWM_OUTH, 50.0);
     set_pwm_duty_cycle(PWM_OUTL, 0.0);
     gate_driver_on();
 }
 
 void stop() {
     set_pwm_duty_cycle(PWM_OUTH, 0.0);
     set_pwm_duty_cycle(PWM_OUTL, 0.0);
     gate_driver_off();
 }
 
 void read_status() {
     uint8_t faults = read_supply_faults();
     bool f1 = read_fault_pin(FAULT_PIN);
     uint16_t a1 = read_adc(1);
     uint16_t a2 = read_adc(2);
     printf("FAULTS: 0x%02X | FLT1: %d | ADC1: %u ADC2: %u\n", faults, f1, a1, a2);
 }

 void force_fault() {
    // Simulate a logic fault by writing an invalid command or toggling control lines
    // Here we write an invalid command to test fault detection
    printf("Forcing fault by sending invalid SPI command...\n");
    spi_transfer16(0xFFFF); // Intentionally invalid command frame
}
 
 int main() {
    printf("Startup - UCC5870-Q1 Interface Test\n");
     ucc5870_init();
     enter_configuration();
     set_desat_threshold(1000);
     exit_configuration();

     int loop_counter = 0;
     while (true) {
         start();
         read_status();
         sleep_ms(5000);
         stop();
         sleep_ms(5000);

         loop_counter++;
         if (loop_counter == 3) {
            force_fault();
         }
     }
 }
 