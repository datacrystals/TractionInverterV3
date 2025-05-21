/**
 * @file ucc5870_interface.h
 * @brief Header for UCC5870-Q1 gate driver interface on Raspberry Pi Pico.
 *
 * This interface abstracts gate driver control (startup, config, faults, PWM) via SPI.
 */

 #ifndef UCC5870_INTERFACE_H
 #define UCC5870_INTERFACE_H
 
 #include <stdint.h>
 #include <stdbool.h>
 
 #ifdef __cplusplus
 extern "C" {
 #endif
 
 /**
  * @brief Initializes SPI, GPIO, and PWM hardware for UCC5870-Q1.
  * Call this once at startup before any other function.
  */
 void ucc5870_init(void);
 
 /**
  * @brief Enters the configuration mode of the gate driver.
  * Must be called before modifying configuration registers.
  */
 void enter_configuration(void);
 
 /**
  * @brief Exits the configuration mode of the gate driver.
  * Call after all desired configuration is completed.
  */
 void exit_configuration(void);
 
 /**
  * @brief Enables gate drive output (driver ON).
  * Typically used after setup to begin switching.
  */
 void gate_driver_on(void);
 
 /**
  * @brief Disables gate drive output (driver OFF).
  * Safely shuts down gate control.
  */
 void gate_driver_off(void);
 
 /**
  * @brief Starts PWM and enables the gate driver.
  * Recommended for typical operational use.
  */
 void start(void);
 
 /**
  * @brief Stops PWM and disables the gate driver.
  * Use this to shut down output safely.
  */
 void stop(void);
 
 /**
  * @brief Sets the desaturation fault threshold voltage.
  * @param mV Threshold in millivolts (500, 750, 1000, 1250).
  */
 void set_desat_threshold(float mV);
 
 /**
  * @brief Reads the supply voltage fault status.
  * @return 8-bit status: upper nibble = VCC1, lower nibble = VCC2/VEE2.
  */
 uint8_t read_supply_faults(void);
 
 /**
  * @brief Reads the state of a fault input pin (e.g., nFLT1 or nFLT2).
  * @param gpio GPIO number of the fault pin.
  * @return true if high, false if low (fault asserted).
  */
 bool read_fault_pin(uint gpio);
 
 /**
  * @brief Reads an internal ADC value from the gate driver.
  * @param channel ADC channel (1 or 2).
  * @return 10-bit ADC value.
  */
 uint16_t read_adc(uint8_t channel);
 
 /**
  * @brief Sets the PWM duty cycle on a given output.
  * @param gpio PWM GPIO (e.g., OUTH or OUTL).
  * @param duty_cycle Duty cycle percentage (0.0 to 100.0).
  */
 void set_pwm_duty_cycle(uint gpio, float duty_cycle);
 
 /**
  * @brief Reads faults and ADC values and prints them.
  * Useful for runtime diagnostics.
  */
 void read_status(void);
 
 #ifdef __cplusplus
 }
 #endif
 
 #endif // UCC5870_INTERFACE_H
 