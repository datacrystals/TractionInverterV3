// Define Pins In Headers
#define LED_FAULT_PIN 4       // Fault Indicator Light
#define LED_INDICATOR1_PIN 6  // Power Indicator Light
#define LED_INDICATOR2_PIN 8  // Output Enabled Indicator
#define LED_INDICATOR3_PIN 9  // Temperature OK
#define FAN_TACH 3            // Fan Tachometer
#define FAN_PWM 5             // Fan PWM duty cycle control
#define OUTPUT_ENABLE_PIN 7   // Pin which enables output, must be high for output to be on
#define PHASE_1_TEMP_SENSOR_PIN A4 // Phase A Temp Sensor
#define PHASE_2_TEMP_SENSOR_PIN A5 // Phase A Temp Sensor
#define CAN_INT_PIN 2         // Pin for canbus interrupt
#define CAN_CS_PIN 10         // Pin for canbus chip select
#define CAN_SI_PIN 11         // Pin for canbus signal in
#define CAN_SO_PIN 12         // Pin for canbus signal out
#define CAN_SCK_PIN 13        // Pin for canbus clock

#define DEVICE_ID 0x01 // Set this to a unique value (0x00 to 0x3F)

// Indicator Defines
#define LONG_BLINK_DURATION 1000  // Long blink duration in ms
#define SHORT_BLINK_DURATION 500  // Short blink duration in ms
#define BLINK_GAP_DURATION 500    // Gap between blinks in ms
#define END_GAP_DURATION 1750     // End gap after sequence in ms

// Constants for airflow calculation
#define MAX_RPM 5500          // Maximum RPM at 100% PWM
#define MAX_AIRFLOW_CFM 225.0 // Maximum airflow in CFM at 100% PWM
#define DUCT_AREA_SQFT 0.15   // Cross-sectional area of the duct in square feet

// Thresholds for airflow in LFM
#define MIN_AIRFLOW_LFM -12000.0  // Minimum acceptable airflow in LFM
#define MAX_AIRFLOW_LFM 12000.0 // Maximum acceptable airflow in LFM

// Temperature and fan speed settings
#define MIN_TEMP -20.0         // Minimum temperature for fan speed adjustment
#define MAX_TEMP 65.0         // Maximum temperature for fan speed adjustment
#define TEMP_EXPONENT 2     // Fan exponent
#define NUM_SAMPLES 10        // Number of samples to use to base the temp off of
#define BASE_FAN_SPEED 25     // Base fan speed (0-255)

// Voltage Sensor settings
#define VIN_MIN_V 20
#define VIN_MAX_V 185
#define VOUT_MIN_V 20
#define VOUT_MAX_V 400
#define VSENSE_GRACE_TIME_MS 2000 // Time which voltage is allowed to be out of spec before error is asserted

// Current Sensor Settings
#define PHASE_1_CURRENT_SENSE_PIN A0 // Pin which is used to sample the current for phase 1
#define PHASE_2_CURRENT_SENSE_PIN A1 // Pin which is used to sample the current for phase 2
#define CURRENT_MIN_VALUE_A -180     // Min current value below which current error thrown
#define CURRENT_MAX_VALUE_A 180      // Max current value above which current error thrown

// Fault management constants
#define MIN_RESET_TIME_SEC 5      // Minimum time a fault must be active before it can auto-reset
#define MAX_AUTO_RESET_COUNT 3    // Maximum number of times a fault can auto-reset
#define AUTO_RESET_FAULT_TIME 60  // Time after which an auto-reset fault is forgotten
#define FAULT_BUFFER_SIZE 4       // Size of the fault buffer
#define FAULT_DURATION_THRESHOLD 5000 // Duration in milliseconds to consider a fault as proper

#define FAULT_BUFFER_OVERFLOW "FAULT_BUFFER_OVERFLOW"
#define FAULT_NAME_LENGTH 32
#define BLINK_CODE_LENGTH 8

#define HEARTBEAT_SHUTDOWN_TIMER 3000 // Will automaticalyl disable output when in heartbeat mode after this time has elapsed

// Debug Info
#define ENABLE_SERIAL_PRINT

#define FIRMWARE_VERSION "V0.5"
bool IGNORE_ERRORS = false;



#include "Common/Indicator.h"
#include "Common/VoltageSensor.h"
#include "Common/FanController.h"
#include "Common/FaultManager.h"
#include "Common/TMP36Sensor.h"
#include "Common/ACS758CurrentSensor.h"

// #include "Easy-CANopen/Src/Easy_CANopen/Easy_CANopen.h"

#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515/archive/master.zip



/* MANUAL INFORMATION FOR CANBUS, BLINK CODE

## Fault Blink Codes

The system uses blink codes to indicate specific faults. Each fault is associated with a unique blink pattern (e.g., "SLS" for Short-Long-Short). Below is a table of fault blink codes:

| Fault Name                     | Blink Code | Description                                                                 |
|--------------------------------|------------|-----------------------------------------------------------------------------|
| `AIRFLOW_OUT_OF_RANGE`         | `SSSL`     | Airflow is below or above the acceptable range.                             |
| `TEMPSENSE_1_OUT_OF_RANGE`     | `SSLS`     | Temperature sensor 1 is out of range.                                       |
| `TEMPSENSE_2_OUT_OF_RANGE`     | `SSLL`     | Temperature sensor 2 is out of range.                                       |
| `VIN_UNDERVOLTAGE`             | `SLSS`     | Input voltage (Vin) is below the minimum threshold.                         |
| `VIN_OVERVOLTAGE`              | `SLSL`     | Input voltage (Vin) is above the maximum threshold.                         |
| `VOUT_UNDERVOLTAGE`            | `SLLS`     | Output voltage (Vout) is below the minimum threshold.                       |
| `VOUT_OVERVOLTAGE`             | `SLLL`     | Output voltage (Vout) is above the maximum threshold.                       |
| `PHASE_A_OVERCURRENT`          | `LSSS`     | Current on Phase 1 is above the maximum threshold.                          |
| `PHASE_B_OVERCURRENT`          | `LSSL`     | Current on Phase 2 is above the maximum threshold.                          |
| `CANBUS_COMM_ERROR`            | `LSLS`     | CANbus communication error detected.                                        |
| `CANBUS_REMOTE_ESTOP`          | `LSLL`     | Remote emergency stop command received.                                     |
| `CANBUS_HEARTBEAT_TIMEOUT`     | `LLSS`     | Heartbeat ping not received within the timeout period.                      |
| `FAULT_BUFFER_OVERFLOW`        | `SSSS`     | Fault buffer overflow (too many faults).                                    |

## CANbus Command List

| Command ID | Description                     | Data Format                  | Response Format             | Notes                                                                 |
|------------|---------------------------------|------------------------------|-----------------------------|-----------------------------------------------------------------------|
| `0x00`     | Emergency Stop (Disable Output) | None                         | `0x01` (Acknowledgment)     | Disables output and asserts a non-resettable fault.                   |
| `0x01`     | Regular Stop (Disable Output)   | None                         | `0x01` (Acknowledgment)     | Disables output without asserting a fault.                            |
| `0x02`     | Enable Output                   | None                         | `0x01` (Acknowledgment)     | Enables output if no faults are active.                               |
| `0x03`     | Reset Faults                    | None                         | `0x01` (Acknowledgment)     | Resets all faults in the fault buffer.                                |
| `0x04`     | Get Number of Faults            | None                         | `uint8_t` (Number of faults)| Returns the number of active faults.                                  |
| `0x05`     | Get Faults List                 | None                         | `char[]` (Fault names)      | Returns a comma-separated list of active fault names.                 |
| `0x06`     | Get Vin Voltage                 | None                         | `uint16_t` (Vin in volts)   | Returns the input voltage (Vin) in volts.                             |
| `0x07`     | Get Vout Voltage                | None                         | `uint16_t` (Vout in volts)  | Returns the output voltage (Vout) in volts.                           |
| `0x08`     | Get Phase 1 Current             | None                         | `uint16_t` (Current in A)   | Returns the current on Phase 1 in amperes.                            |
| `0x09`     | Get Phase 2 Current             | None                         | `uint16_t` (Current in A)   | Returns the current on Phase 2 in amperes.                            |
| `0x0A`     | Get Phase 1 Temperature         | None                         | `uint16_t` (Temp in °C)     | Returns the temperature of Phase 1 in Celsius.                        |
| `0x0B`     | Get Phase 2 Temperature         | None                         | `uint16_t` (Temp in °C)     | Returns the temperature of Phase 2 in Celsius.                        |
| `0x0C`     | Get Fan Airflow (LFM)           | None                         | `uint16_t` (Airflow in LFM) | Returns the airflow in Linear Feet per Minute (LFM).                  |
| `0x0D`     | Get Fan RPM                     | None                         | `uint16_t` (RPM)            | Returns the fan speed in RPM.                                         |
| `0x0E`     | Get Power                       | None                         | `uint16_t` (Power in W)     | Returns the total power (Vin * (I1 + I2)) in watts.                   |
| `0x0F`     | Get CAN Fault Status            | None                         | `uint8_t` (0x00 or 0x01)    | Returns `0x01` if a CANbus error is detected, otherwise `0x00`.       |
| `0x10`     | Enable Heartbeat Mode           | None                         | `0x01` (Acknowledgment)     | Enables heartbeat mode, requiring periodic pings to keep output on.   |
| `0x11`     | Heartbeat Ping                  | None                         | `0x01` (Acknowledgment)     | Resets the heartbeat timer to keep output enabled.                    |

---

## Notes
- **Boolean Responses**: For commands like `0x01F` (Get CAN Fault Status), the response is a single byte (`0x00` or `0x01`) indicating the status.
- **Acknowledgment**: Most commands return an acknowledgment byte (`0x01`) to confirm receipt and execution.
- **Error Handling**: Unknown commands return an error byte (`0x02`) and use the original command ID in the response.
- **Blink Code Format**: Each blink code consists of a sequence of short (`S`) and long (`L`) blinks. For example, `SSSL` means three short blinks followed by one long blink.
- **Heartbeat Mode**: When enabled, the system requires periodic pings (command `0x021`) to keep the output enabled. If no ping is received within the timeout period (`HEARTBEAT_SHUTDOWN_TIMER`), the output is disabled, and a fault is asserted.
- **Fault Auto-Reset**: Some faults can auto-reset after a specified duration (`AUTO_RESET_FAULT_TIME`). Others require manual reset (command `0x013`).

*/



class CANController {
  public:
      CANController(int csPin, int interruptPin)
          : csPin(csPin), interruptPin(interruptPin), mcp2515(csPin), hasError(false) {}

      void begin() {
          SPI.begin();
          pinMode(csPin, OUTPUT);
          digitalWrite(csPin, HIGH);

          // Reset the MCP2515 and check for errors
          if (mcp2515.reset() != MCP2515::ERROR_OK) {
              hasError = true;
  #ifdef ENABLE_SERIAL_PRINT
              Serial.println("MCP2515 Reset Failed");
  #endif
              return;
          }

          // Set bitrate and mode
          if (mcp2515.setBitrate(CAN_250KBPS) != MCP2515::ERROR_OK) {
              hasError = true;
  #ifdef ENABLE_SERIAL_PRINT
              Serial.println("MCP2515 Set Bitrate Failed");
  #endif
              return;
          }

          if (mcp2515.setNormalMode() != MCP2515::ERROR_OK) {
              hasError = true;
  #ifdef ENABLE_SERIAL_PRINT
              Serial.println("MCP2515 Set Normal Mode Failed");
  #endif
              return;
          }

          // Set up interrupt pin
          pinMode(interruptPin, INPUT);
          attachInterrupt(digitalPinToInterrupt(interruptPin), handleInterruptStatic, FALLING);

          setInstance(this);
      }

      void registerCallback(void (*callback)(struct can_frame*)) {
          this->callback = callback;
      }

      void sendMessage(uint32_t id, uint8_t* data = nullptr, uint8_t len = 0) {
          struct can_frame frame;
          frame.can_id = id;
          frame.can_dlc = len;
          if (data != nullptr && len > 0) {
              memcpy(frame.data, data, len);
          }

          // Send the message and check for errors
          if (mcp2515.sendMessage(&frame) != MCP2515::ERROR_OK) {
              hasError = true;
      #ifdef ENABLE_SERIAL_PRINT
              Serial.println(F("Failed to Send CAN Message"));
      #endif
          }
      }

      bool getError() const {
          return hasError;
      }

      static CANController& instance() {
          return *instancePtr;
      }

  private:
      int csPin;
      int interruptPin;
      MCP2515 mcp2515;
      void (*callback)(struct can_frame*);
      bool hasError; // Tracks if an error has occurred

      static void handleInterruptStatic() {
          instance().handleInterrupt();
      }

      void handleInterrupt() {
          struct can_frame frame;
          if (mcp2515.readMessage(&frame) == MCP2515::ERROR_OK) {
              if (callback != nullptr) {
                  callback(&frame);
              }
          } else {
              hasError = true; // Set error flag if reading fails
  #ifdef ENABLE_SERIAL_PRINT
              Serial.println("Failed to Read CAN Message");
  #endif
          }
      }

      static void setInstance(CANController* ptr) {
          instancePtr = ptr;
      }

      static CANController* instancePtr;
};

CANController* CANController::instancePtr = nullptr;



// Define the static member
FanController* FanController::instance = nullptr;






// -- Define Global Objects --//
Indicator IndicatorFault(LED_FAULT_PIN);
Indicator IndicatorPower(LED_INDICATOR1_PIN);
Indicator IndicatorOutput(LED_INDICATOR2_PIN);
Indicator IndicatorTempOK(LED_INDICATOR3_PIN);

VoltageSensor VoltageSensorVIN(A2, 6000.0, 120.0, VIN_MIN_V, VIN_MAX_V, VSENSE_GRACE_TIME_MS);
VoltageSensor VoltageSensorVOUT(A3, 6000.0, 40.0, VOUT_MIN_V, VOUT_MAX_V, VSENSE_GRACE_TIME_MS);

TMP36Sensor Phase1TempSense(PHASE_1_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);
TMP36Sensor Phase2TempSense(PHASE_2_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);

ACS758CurrentSensor Phase1CurrentSense(PHASE_1_CURRENT_SENSE_PIN);
ACS758CurrentSensor Phase2CurrentSense(PHASE_2_CURRENT_SENSE_PIN);

FanController SystemFan(FAN_TACH, FAN_PWM, TEMP_EXPONENT, NUM_SAMPLES);

FaultManager faultManager;

CANController canController(CAN_CS_PIN, CAN_INT_PIN);



bool OutputEnabled = false; // Indicates if the output has been enabled or not
bool HeartbeatOutput = false; // If true, a heartbeat must be recieved every n ms, else output is disabled.

float CurrentTemp_C = 30.0; // Temperature which is used to drive the fan control
unsigned long lastKeepAliveTime = 0;


float VIN, VOUT; // Voltage Values
float Phase1Temp_C, Phase2Temp_C; // Temperature values
float Phase1Current_A, Phase2Current_A; // Current values



int freeMemory() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}


// Canbus Handler
void handleCanResponse(struct can_frame* frame) {
    uint8_t dataBuffer[8]; // Buffer to hold CAN data
    uint16_t value;        // Temporary variable for 16-bit values

    // Debug: Print full CAN ID
    // Serial.print(F("Received CAN ID: 0x"));
    // Serial.println(frame->can_id, HEX);

    // Extract the target device ID (bits 10 to 5)
    uint8_t targetDeviceID = (frame->can_id >> 5) & 0x3F;
    
    // Debug: Print target device ID
    // Serial.print(F("Target Device ID: 0x"));
    // Serial.println(targetDeviceID, HEX);

    // Ignore messages not intended for this device
    if (targetDeviceID != DEVICE_ID && targetDeviceID != 0x00) {
        Serial.println(F("Message ignored (wrong device ID)."));
        return;
    }

    // Extract the command ID (bits 4 to 0)
    uint8_t commandID = frame->can_id & 0x1F;

    // Debug: Print command ID
    // Serial.print(F("Command ID: 0x"));
    // Serial.println(commandID, HEX);


    switch (commandID) {
        case 0x00: // Emergency Stop (Disable Output)
            faultManager.assertFault("CANBUS_REMOTE_ESTOP", "LSLL", false);
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Estop Disable Output Command Received"));
#endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x00, dataBuffer, 1);
            break;

        case 0x01: // Regular Stop (Disable Output)
            OutputEnabled = false;
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Regular Stop Command Received"));
#endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x01, dataBuffer, 1);
            break;

        case 0x02: // Enable Output
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Enable Output Command Received"));
#endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x02, dataBuffer, 1);
            OutputEnabled = true;
            break;

        case 0x03: // Reset Faults
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Reset Faults Command Received"));
#endif
            faultManager.resetFaults();
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x03, dataBuffer, 1);
            break;

        case 0x04: { // Get Number of Faults
            uint8_t numFaults = faultManager.getFaultCount();
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Number of Faults: "));
            Serial.println(numFaults);
#endif
            dataBuffer[0] = numFaults;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x04, dataBuffer, 1);
            break;
        }

        case 0x05: { // Get Faults List
            char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
            faultManager.getFaultNames(faultNames, sizeof(faultNames));
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Faults:"));
            Serial.println(faultNames);
#endif
            // Send the fault names over CANbus
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x05, (uint8_t*)faultNames, strlen(faultNames));
            break;
        }

        case 0x06: { // Get Vin Voltage
            value = int(VIN);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Vin Value: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x06, dataBuffer, 2);
            break;
        }

        case 0x07: { // Get Vout Voltage
            value = int(VOUT);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Vout Value: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x07, dataBuffer, 2);
            break;
        }

        case 0x08: { // Get Phase 1 Current
            value = int(Phase1Current_A);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Ph1 Current: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x08, dataBuffer, 2);
            break;
        }

        case 0x09: { // Get Phase 2 Current
            value = int(Phase2Current_A);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Ph2 Current: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x09, dataBuffer, 2);
            break;
        }

        case 0x0A: { // Get Phase 1 Temperature
            value = int(Phase1Temp_C);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Ph1 Temperature: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0A, dataBuffer, 2);
            break;
        }

        case 0x0B: { // Get Phase 2 Temperature
            value = int(Phase2Temp_C);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Ph2 Temperature: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0B, dataBuffer, 2);
            break;
        }

        case 0x0C: { // Get Fan Airflow LFM
            value = SystemFan.getAirflowLFM();
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Fan Airflow (LFM): "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0C, dataBuffer, 2);
            break;
        }

        case 0x0D: { // Get Fan RPM
            value = SystemFan.getRPM();
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Fan RPM: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0D, dataBuffer, 2);
            break;
        }

        case 0x0E: { // Get Power
            value = VIN * (Phase1Current_A + Phase2Current_A);
#ifdef ENABLE_SERIAL_PRINT
            Serial.print(F("Power: "));
            Serial.println(value);
#endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0E, dataBuffer, 2);
            break;
        }

        case 0x0F: { // Get CAN Fault Status
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Checking CAN Faults..."));
            if (CANController::instance().getError()) {
                Serial.println(F("CAN Error Detected"));
            } else {
                Serial.println(F("CAN Communication OK"));
            }
#endif
            dataBuffer[0] = CANController::instance().getError() ? 0x01 : 0x00; // Boolean response
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x0F, dataBuffer, 1);
            break;
        }

        case 0x10: // Enable Heartbeat Mode
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Heartbeat enable received."));
#endif
            HeartbeatOutput = true;
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x10, dataBuffer, 1);
            break;

        case 0x11: // Heartbeat Ping
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Heartbeat ping received."));
#endif
            lastKeepAliveTime = millis();
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | 0x11, dataBuffer, 1);
            break;

        default: // Unknown Command
#ifdef ENABLE_SERIAL_PRINT
            Serial.println(F("Unknown Command"));
#endif
            dataBuffer[0] = 0x02; // Error byte
            CANController::instance().sendMessage((DEVICE_ID << 5) | (frame->can_id & 0x1F), dataBuffer, 1);
            break;
    }
}


//-- System Run Modes --//
void setup() {


    // Setup Output Enable Pin
    pinMode(OUTPUT_ENABLE_PIN, OUTPUT);
    digitalWrite(OUTPUT_ENABLE_PIN, true); // disable output immediately (active low)


    // Initialize the static instance pointer
    FanController::instance = &SystemFan;

    // Register Canbus Handler
    canController.begin();
    canController.registerCallback(handleCanResponse);

    // Initialize serial communication
#ifdef ENABLE_SERIAL_PRINT
    Serial.begin(9600);
#endif

    // Info about this module via serial
    Serial.println(F("Traction Inverter DC Link Bus Voltage Regulator Module"));
    Serial.print(F("Firmware Version: "));
    Serial.print(F(FIRMWARE_VERSION));
    if (IGNORE_ERRORS) {
      Serial.println(F("- NoErrorDetection"));
    } else {
      Serial.println(F(""));
    }
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("System Initializing..."));

    // Led Self Test
    IndicatorFault.SetState(true);
    IndicatorPower.SetState(true);
    IndicatorOutput.SetState(true);
    IndicatorTempOK.SetState(true);
    delay(1000);
    IndicatorFault.SetState(false);
    IndicatorPower.SetState(false);
    IndicatorOutput.SetState(false);
    IndicatorTempOK.SetState(false);

    // Power is okay, since the LED is on...
    IndicatorPower.SetState(true);

    // Fan Baseline Duty Cycle
    // SystemFan.setSpeed(BASE_FAN_SPEED);
    SystemFan.setSpeed(175);

    // Init Current Sensors
    Phase1CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);
    Phase2CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);



OutputEnabled = true;

    // Init done
    Serial.println(F("Initialization complete"));

}



void loop() {
    // Update Indicators
    // IndicatorFault.SetState(faultManager.getFaultCount() > 0);
    IndicatorPower.Blink(750);

    // Get the blink code for the most recent fault
    char blinkCode[BLINK_CODE_LENGTH];
    if (faultManager.getFaultCount() > 0) {
      faultManager.getMostRecentFaultBlinkCode(blinkCode, BLINK_CODE_LENGTH);
      if (blinkCode[0] != '\0') {
          IndicatorFault.BlinkCode(blinkCode);
      }
    } else {
      IndicatorFault.SetState(false);
    }



    // Adjust fan speed based on temperature
    if (!OutputEnabled) {
      SystemFan.setFanSpeed(CurrentTemp_C);
    } else {
      SystemFan.setSpeed(125); // for now just peg the fan, the temp sense is screwed when output is on due to noise
    }


    // Print airflow in LFM every second
    static unsigned long lastPrintTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastPrintTime >= 1000) {
        lastPrintTime = currentTime;
        SystemFan.printAirflowLFM();

        // Check for airflow errors
        if (SystemFan.checkError()) {
            faultManager.assertFault("AIRFLOW_OUT_OF_RANGE", "SSSL");
        }


        // ---- UPDATE VOLTAGE SENSOR DATA ---- //
        // Read Voltage Sensors
        VIN  = VoltageSensorVIN.readVoltage();
        VOUT = VoltageSensorVOUT.readVoltage();

#ifdef ENABLE_SERIAL_PRINT
        Serial.print(F("VIN Voltage: "));
        Serial.print(VIN);
        Serial.println("V");

        Serial.print(F("VOUT Voltage: "));
        Serial.print(VOUT);
        Serial.println("V");
#endif




        // ---- UPDATE TEMPERATURE SENSOR DATA ---- //
        // Read Temperature Sensors
        Phase1Temp_C = Phase1TempSense.readTemperature();
        Phase2Temp_C = Phase2TempSense.readTemperature();

        // Select Max Temperature
        if (Phase1Temp_C > Phase2Temp_C) {
          CurrentTemp_C = Phase1Temp_C;
        } else {
          CurrentTemp_C = Phase2Temp_C;
        }

#ifdef ENABLE_SERIAL_PRINT
        Serial.print(F("Phase1 Temp: "));
        Serial.print(Phase1Temp_C);
        Serial.println("C");

        Serial.print(F("Phase2 Temp: "));
        Serial.print(Phase2Temp_C);
        Serial.println("C");
#endif
        
        // Update thermal indicator led
        IndicatorTempOK.SetState(!(Phase1TempSense.getFault() || Phase2TempSense.getFault() || SystemFan.checkError()));




        // ---- Current Sensor Monitoring ---- //
        Phase1Current_A = Phase1CurrentSense.getCurrent();
        Phase2Current_A = Phase2CurrentSense.getCurrent();

#ifdef ENABLE_SERIAL_PRINT
        Serial.print(F("Phase1 Current: "));
        Serial.print(Phase1Current_A);
        Serial.println("A");

        Serial.print(F("Phase2 Current: "));
        Serial.print(Phase2Current_A);
        Serial.println("A");
#endif        

        

        // -- Ignore errors warning -- //
        if (IGNORE_ERRORS) {
          Serial.println(F("Warning, this firmware ignores all errors! Please monitor carefully, the output will always be enabled."));
          faultManager.resetFaults();
        }

        // ---- SERIAL LOG FAULTS ---- //
        // Print current faults
        if (faultManager.getFaultCount() > 0) {
            char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
            faultManager.getFaultNames(faultNames, sizeof(faultNames));
            Serial.print(F("Current Faults: "));
            Serial.println(faultNames);
        }

#ifdef ENABLE_SERIAL_PRINT
    Serial.print(F("Free RAM: "));
    Serial.println(freeMemory());
#endif        





    }


    // Heartbeat Logic Check
    // if (millis() - lastKeepAliveTime > HEARTBEAT_SHUTDOWN_TIMER) {
    //   OutputEnabled = false;
    //   faultManager.assertFault("CANBUS_HEARTBEAT_TIMEOUT", "LLSS");
    // }


    // -- Get Faults -- //

    // Get voltage sense errors
    if (VoltageSensorVOUT.isUndervoltage() && OutputEnabled) {
        faultManager.assertFault("VOUT_UNDERVOLTAGE", "SLLS");
    }
    if (VoltageSensorVOUT.isOvervoltage()) {
        faultManager.assertFault("VOUT_OVERVOLTAGE", "SLLL");
    }
    if (VoltageSensorVIN.isUndervoltage()) {
        faultManager.assertFault("VIN_UNDERVOLTAGE", "SLSS");
    }
    if (VoltageSensorVIN.isOvervoltage()) {
        faultManager.assertFault("VIN_OVERVOLTAGE", "SLSL");
    }


    // // Temperature Sensor Faults
    // if (Phase1TempSense.getFault()) {
    //     faultManager.assertFault("TEMPSENSE_1_OUT_OF_RANGE", "SSLS");
    // }
    // if (Phase2TempSense.getFault()) {
    //     faultManager.assertFault("TEMPSENSE_2_OUT_OF_RANGE", "SSLL");
    // }


    // Current sense faults
    if (Phase1CurrentSense.getError()) {
        faultManager.assertFault("PHASE_A_OVERCURRENT", "LSSS");
    }
    if (Phase2CurrentSense.getError()) {
        faultManager.assertFault("PHASE_B_OVERCURRENT", "LSSL");
    }


    // ---- CANBUS FAULT CHECK ---- //
    // if (canController.getError()) {
    //     faultManager.assertFault("CANBUS_COMM_ERROR", "LSLS");
    // }



    // Debugging ignore errors, always keep output on for debugging.
    if (IGNORE_ERRORS) {
      faultManager.resetFaults();
      OutputEnabled = true;
    }
    // We need to double check the voltage isnt too high *even* if we ignore errors
    // had a board already blow up which woldnt have happened with this check
    if (VoltageSensorVOUT.readVoltage() > VOUT_MAX_V) {
      OutputEnabled = false;
      IGNORE_ERRORS = false;
    }

    // Check that there are no errors, and disable output if there are
    if (faultManager.getFaultCount() != 0) {
      OutputEnabled = false;
    }
    IndicatorOutput.SetState(OutputEnabled);
    digitalWrite(OUTPUT_ENABLE_PIN, !OutputEnabled); // invert output enabled as the low state activates the output

    // Clear and forget auto-reset faults
    faultManager.clearAutoResetFaults();
    faultManager.forgetAutoResetFaults();




}
