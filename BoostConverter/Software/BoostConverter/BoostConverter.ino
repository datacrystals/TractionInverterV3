#include <SPI.h>
#include <mcp2515.h> // https://github.com/autowp/arduino-mcp2515/archive/master.zip


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


// Constants for airflow calculation
#define MAX_RPM 5500          // Maximum RPM at 100% PWM
#define MAX_AIRFLOW_CFM 225.0 // Maximum airflow in CFM at 100% PWM
#define DUCT_AREA_SQFT 0.15   // Cross-sectional area of the duct in square feet

// Thresholds for airflow in LFM
#define MIN_AIRFLOW_LFM 50.0  // Minimum acceptable airflow in LFM
#define MAX_AIRFLOW_LFM 2500.0 // Maximum acceptable airflow in LFM

// Temperature and fan speed settings
#define MIN_TEMP 30.0         // Minimum temperature for fan speed adjustment
#define MAX_TEMP 120.0         // Maximum temperature for fan speed adjustment
#define BASE_FAN_SPEED 25     // Base fan speed (0-255)

// Voltage Sensor settings
#define VIN_MIN_V 24
#define VIN_MAX_V 185
#define VOUT_MIN_V 320
#define VOUT_MAX_V 360
#define VSENSE_GRACE_TIME_MS 2000 // Time which voltage is allowed to be out of spec before error is asserted

// Current Sensor Settings
#define PHASE_1_CURRENT_SENSE_PIN A0 // Pin which is used to sample the current for phase 1
#define PHASE_2_CURRENT_SENSE_PIN A1 // Pin which is used to sample the current for phase 2
#define CURRENT_MIN_VALUE_A -5       // Min current value below which current error thrown
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
| `0x010`    | Emergency Stop (Disable Output) | None                         | `0x01` (Acknowledgment)     | Disables output and asserts a non-resettable fault.                   |
| `0x011`    | Regular Stop (Disable Output)   | None                         | `0x01` (Acknowledgment)     | Disables output without asserting a fault.                            |
| `0x012`    | Enable Output                   | None                         | `0x01` (Acknowledgment)     | Enables output if no faults are active.                               |
| `0x013`    | Reset Faults                    | None                         | `0x01` (Acknowledgment)     | Resets all faults in the fault buffer.                                |
| `0x014`    | Get Number of Faults            | None                         | `uint8_t` (Number of faults)| Returns the number of active faults.                                  |
| `0x015`    | Get Faults List                 | None                         | `char[]` (Fault names)      | Returns a comma-separated list of active fault names.                 |
| `0x016`    | Get Vin Voltage                 | None                         | `uint16_t` (Vin in volts)   | Returns the input voltage (Vin) in volts.                             |
| `0x017`    | Get Vout Voltage                | None                         | `uint16_t` (Vout in volts)  | Returns the output voltage (Vout) in volts.                           |
| `0x018`    | Get Phase 1 Current             | None                         | `uint16_t` (Current in A)   | Returns the current on Phase 1 in amperes.                            |
| `0x019`    | Get Phase 2 Current             | None                         | `uint16_t` (Current in A)   | Returns the current on Phase 2 in amperes.                            |
| `0x01A`    | Get Phase 1 Temperature         | None                         | `uint16_t` (Temp in °C)     | Returns the temperature of Phase 1 in Celsius.                        |
| `0x01B`    | Get Phase 2 Temperature         | None                         | `uint16_t` (Temp in °C)     | Returns the temperature of Phase 2 in Celsius.                        |
| `0x01C`    | Get Fan Airflow (LFM)           | None                         | `uint16_t` (Airflow in LFM) | Returns the airflow in Linear Feet per Minute (LFM).                  |
| `0x01D`    | Get Fan RPM                     | None                         | `uint16_t` (RPM)            | Returns the fan speed in RPM.                                         |
| `0x01E`    | Get Power                       | None                         | `uint16_t` (Power in W)     | Returns the total power (Vin * (I1 + I2)) in watts.                   |
| `0x01F`    | Get CAN Fault Status            | None                         | `uint8_t` (0x00 or 0x01)    | Returns `0x01` if a CANbus error is detected, otherwise `0x00`.       |
| `0x020`    | Enable Heartbeat Mode           | None                         | `0x01` (Acknowledgment)     | Enables heartbeat mode, requiring periodic pings to keep output on.   |
| `0x021`    | Heartbeat Ping                  | None                         | `0x01` (Acknowledgment)     | Resets the heartbeat timer to keep output enabled.                    |

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
          if (mcp2515.setBitrate(CAN_500KBPS) != MCP2515::ERROR_OK) {
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
              Serial.println("Failed to Send CAN Message");
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


class Indicator {
  public:
    // Constructor
    Indicator(int pin) : Pin_(pin), State_(false), previousMillis_(0),
                          blinkCodeIndex_(0), blinkState_(IDLE), blinkStartTime_(0) {
      pinMode(Pin_, OUTPUT);
    }

    // Toggle the LED state
    bool ToggleIndicator() {
      State_ = !State_;
      digitalWrite(Pin_, State_);
      return State_;
    }

    // Set the LED state explicitly
    void SetState(bool newState) {
      State_ = newState;
      digitalWrite(Pin_, State_);
    }

    // Blink the LED at a fixed frequency
    void Blink(int frequencyMs) {
      unsigned long currentMillis = millis();

      if (currentMillis - previousMillis_ >= frequencyMs) {
        previousMillis_ = currentMillis;
        ToggleIndicator();
      }
    }

    // Blink a specific pattern (e.g., "SLS" for Short-Long-Short)
    void BlinkCode(const String& blinkCode) {
      unsigned long currentMillis = millis();

      switch (blinkState_) {
        case IDLE:
          // Start the blink sequence immediately
          blinkCodeIndex_ = 0;
          blinkState_ = BLINK_ON;
          break;

        case BLINK_ON:
          // Turn on the LED and start the blink duration
          if (blinkCodeIndex_ < blinkCode.length()) {
            SetState(true);
            blinkStartTime_ = currentMillis;
            blinkState_ = BLINK_WAIT;
          } else {
            // End of sequence, wait for the end gap before repeating
            blinkState_ = END_GAP;
            blinkStartTime_ = currentMillis;
          }
          break;

        case BLINK_WAIT:
          // Wait for the blink duration to complete
          if (currentMillis - blinkStartTime_ >= getBlinkDuration(blinkCode[blinkCodeIndex_])) {
            SetState(false);
            blinkCodeIndex_++;
            blinkState_ = BLINK_GAP;
            blinkStartTime_ = currentMillis;
          }
          break;

        case BLINK_GAP:
          // Wait for the gap between blinks
          if (currentMillis - blinkStartTime_ >= BLINK_GAP_DURATION) {
            blinkState_ = BLINK_ON;
          }
          break;

        case END_GAP:
          // Wait for the end gap after the sequence ends
          if (currentMillis - blinkStartTime_ >= END_GAP_DURATION) {
            blinkState_ = IDLE; // Restart the sequence
          }
          break;
      }
    }

  private:
    // States for the blink code state machine
    enum BlinkState {
      IDLE,      // Waiting to start the blink sequence
      BLINK_ON,  // LED is on
      BLINK_WAIT, // Waiting for the blink duration to complete
      BLINK_GAP,  // Waiting for the gap between blinks
      END_GAP    // Waiting for the end gap after the sequence ends
    };

    // Constants for timing
    const unsigned long LONG_BLINK_DURATION = 1000; // Long blink duration in ms
    const unsigned long SHORT_BLINK_DURATION = 500; // Short blink duration in ms
    const unsigned long BLINK_GAP_DURATION = 500; // Gap between blinks in ms
    const unsigned long END_GAP_DURATION = 1750; // End gap after sequence in ms

    int Pin_; // Output Pin
    bool State_; // Current LED State
    unsigned long previousMillis_; // For the Blink method
    int blinkCodeIndex_; // Current position in the blink code
    BlinkState blinkState_; // Current state of the blink sequence
    unsigned long blinkStartTime_; // When the current blink started

    // Helper function to get the duration of a blink based on the code character
    unsigned long getBlinkDuration(char blinkChar) {
      return (blinkChar == 'L') ? LONG_BLINK_DURATION : SHORT_BLINK_DURATION;
    }
};

class VoltageSensor {
  public:
      // Constructor to initialize the sensor with an analog pin, voltage divider ratio, and acceptable range
      VoltageSensor(int analogPin, float dividerRatio, float vmin, float vmax, unsigned long errorDuration)
          : analogPin_(analogPin), dividerRatio_(dividerRatio), vmin_(vmin), vmax_(vmax),
            errorDuration_(errorDuration), errorStartTime_(0), error_(false) {}

      // Method to read the voltage from the sensor
      float readVoltage() {
          // Read the analog value from the pin (0-1023 for Arduino)
          int analogValue = analogRead(analogPin_);

          // Convert the analog value to a voltage (0-5V for Arduino)
          float voltage = analogValue * (5.0 / 1023.0);

          // Apply the voltage divider ratio to get the actual voltage
          voltage /= dividerRatio_;

          // Check if the voltage is within the acceptable range
          if (voltage < vmin_ || voltage > vmax_) {
              if (errorStartTime_ == 0) {
                  // Start timing the error duration
                  errorStartTime_ = millis();
              } else if (millis() - errorStartTime_ >= errorDuration_) {
                  // Error duration exceeded, set the error flag
                  error_ = true;
              }
          } else {
              // Voltage is within range, reset error tracking
              errorStartTime_ = 0;
              error_ = false;
          }

          return voltage;
      }

      // Method to check if the voltage is out of the acceptable range
      bool getError() const {
          return error_;
      }

      // Method to check if the voltage is below the minimum acceptable value
      bool isUndervoltage() const {
          return error_ && (readVoltage() < vmin_);
      }

      // Method to check if the voltage is above the maximum acceptable value
      bool isOvervoltage() const {
          return error_ && (readVoltage() > vmax_);
      }

  private:
      int analogPin_;             // Analog input pin
      float dividerRatio_;       // Voltage divider ratio
      float vmin_;               // Minimum acceptable voltage
      float vmax_;               // Maximum acceptable voltage
      unsigned long errorDuration_; // Duration for which voltage must be out of range to trigger error
      unsigned long errorStartTime_; // Time when the voltage first went out of range
      bool error_;               // Error flag indicating if voltage is out of range
};

class FanController {
    public:
        // Constructor
        FanController(int rpmPin, int pwmPin)
            : rpmPin(rpmPin), pwmPin(pwmPin), rpm(0), lastTime(0), pulseCount(0),
              lastAirflowUpdate(0), cachedAirflowLFM(0) {
            pinMode(rpmPin, INPUT_PULLUP);
            pinMode(pwmPin, OUTPUT);
            attachInterrupt(digitalPinToInterrupt(rpmPin), rpmInterruptStatic, FALLING);
        }

        // Set the fan speed (0-255)
        void setSpeed(int speed) {
            if (speed < 0) speed = 0;
            if (speed > 255) speed = 255;
            analogWrite(pwmPin, speed);
        }

        // Adjust fan speed based on temperature
        void setFanSpeed(float currentTemp) {
            int speed = BASE_FAN_SPEED;
            if (currentTemp > MAX_TEMP) {
                speed = 255;
            } else if (currentTemp > MIN_TEMP) {
                speed = map(currentTemp, MIN_TEMP, MAX_TEMP, BASE_FAN_SPEED, 255);
            }
            setSpeed(speed);
        }

        // Get the current RPM
        int getRPM() {
            noInterrupts(); // Disable interrupts to read shared variables
            unsigned long now = millis();
            rpm = (pulseCount * 60000) / (now - lastTime);
            lastTime = now;
            pulseCount = 0;
            interrupts(); // Re-enable interrupts
            return rpm;
        }

        // Get the current airflow in LFM
        int getAirflowLFM() {
          updateAirflowLFM();
          return cachedAirflowLFM;
        }

        // Calculate and print airflow in LFM
        void printAirflowLFM() {
            updateAirflowLFM();
            Serial.print("Airflow: ");
            Serial.print(cachedAirflowLFM);
            Serial.println(" LFM");
        }

        // Check if the airflow is within the acceptable range
        bool checkError() {
            updateAirflowLFM();
            return (cachedAirflowLFM < MIN_AIRFLOW_LFM) || (cachedAirflowLFM > MAX_AIRFLOW_LFM);
        }

        // Public static instance pointer
        static FanController* instance;

    private:
        int rpmPin;
        int pwmPin;
        volatile int rpm;
        volatile unsigned long lastTime;
        volatile int pulseCount;
        unsigned long lastAirflowUpdate;
        float cachedAirflowLFM;

        // Update the cached airflow value every half second
        void updateAirflowLFM() {
            unsigned long currentMillis = millis();
            if (currentMillis - lastAirflowUpdate >= 500) {
                lastAirflowUpdate = currentMillis;
                int currentRPM = getRPM();
                float airflowCFM = (currentRPM / (float)MAX_RPM) * MAX_AIRFLOW_CFM;
                cachedAirflowLFM = airflowCFM / DUCT_AREA_SQFT;
            }
        }

        // Static ISR wrapper
        static void rpmInterruptStatic() {
            // Call the actual ISR on the instance
            instance->rpmInterrupt();
        }

        // Interrupt service routine for RPM measurement
        void rpmInterrupt() {
            pulseCount++;
        }
};

// Define the static member
FanController* FanController::instance = nullptr;


class FaultManager {
  public:
    FaultManager() {
      // Initialize the fault buffer
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        faultBuffer[i].name[0] = '\0';
        faultBuffer[i].blinkCode[0] = '\0';
        faultBuffer[i].resetCount = 0;
        faultBuffer[i].startTime = 0;
        faultBuffer[i].isAutoReset = false;
        faultBuffer[i].isActive = false;
      }
    }

    // Assert a fault
    void assertFault(const char* faultName, const char* blinkCode, bool isAutoReset = true, bool overwriteFaults = false) {

      if (isOverflown) {
        return;
      }

      if (overwriteFaults) {

        // Now add our fault to the buffer
        strncpy(faultBuffer[0].name, faultName, FAULT_NAME_LENGTH);
        strncpy(faultBuffer[0].blinkCode, blinkCode, BLINK_CODE_LENGTH);
        faultBuffer[0].resetCount = 0;
        faultBuffer[0].startTime = millis();
        faultBuffer[0].isAutoReset = isAutoReset;
        faultBuffer[0].isActive = true;
        faultCount = 1;
        
        return;
      }

      // Check if the fault buffer is full
      // Serial.println(faultCount);

      if (faultCount >= FAULT_BUFFER_SIZE) {
        // Assert the FAULT_BUFFER_OVERFLOW fault if the buffer is full
        resetFaults();

        // Force-add the overflow fault, overwriting existing faults
        assertFault("FAULT_BUFFER_OVERFLOW", "SSSS", false, true);

        isOverflown = true;

      }

      // Check if the fault is already in the buffer
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (strcmp(faultBuffer[i].name, faultName) == 0) {
          // Fault is already in the buffer, update the start time
          faultBuffer[i].startTime = millis();
          faultBuffer[i].isActive = true;
          return;
        }
      }

      // Add the fault to the buffer
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] == '\0') {
          strncpy(faultBuffer[i].name, faultName, FAULT_NAME_LENGTH);
          strncpy(faultBuffer[i].blinkCode, blinkCode, BLINK_CODE_LENGTH);
          faultBuffer[i].resetCount = 0;
          faultBuffer[i].startTime = millis();
          faultBuffer[i].isAutoReset = isAutoReset;
          faultBuffer[i].isActive = true;
          faultCount++;
          break;
        }
      }
    }

    void resetFaults() {

      isOverflown = false;

      // Clear existing faults
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        faultBuffer[i].name[0] = '\0';
        faultBuffer[i].blinkCode[0] = '\0';
        faultBuffer[i].resetCount = 0;
        faultBuffer[i].startTime = 0;
        faultBuffer[i].isAutoReset = false;
        faultBuffer[i].isActive = false;
      }
      faultCount = 0;
    }

    // Clear auto-reset faults
    void clearAutoResetFaults() {
      unsigned long currentMillis = millis();
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isAutoReset) {
          unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
          if (faultDuration >= MIN_RESET_TIME_SEC * 1000) {
            if (faultBuffer[i].resetCount < MAX_AUTO_RESET_COUNT) {
              faultBuffer[i].resetCount++;
              faultBuffer[i].startTime = currentMillis;
            } else {
              faultBuffer[i].name[0] = '\0';
              faultBuffer[i].blinkCode[0] = '\0';
              faultBuffer[i].resetCount = 0;
              faultCount--;
            }
          }
        }
      }
    }

    // Forget auto-reset faults after a certain time
    void forgetAutoResetFaults() {
      unsigned long currentMillis = millis();
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isAutoReset) {
          unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
          if (faultDuration >= AUTO_RESET_FAULT_TIME * 1000) {
            faultBuffer[i].name[0] = '\0';
            faultBuffer[i].blinkCode[0] = '\0';
            faultBuffer[i].resetCount = 0;
            faultCount--;
          }
        }
      }
    }

    // Get the current fault names
    void getFaultNames(char* faultNames, int maxLength) {
      faultNames[0] = '\0';
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isActive) {
          if (faultNames[0] != '\0') {
            strncat(faultNames, ", ", maxLength - strlen(faultNames) - 1);
          }
          strncat(faultNames, faultBuffer[i].name, maxLength - strlen(faultNames) - 1);
        }
      }
    }

    // Get the blink code of the most recent fault
    void getMostRecentFaultBlinkCode(char* blinkCode, int maxLength) {
      blinkCode[0] = '\0';
      for (int i = FAULT_BUFFER_SIZE - 1; i >= 0; i--) {
        if (faultBuffer[i].name[0] != '\0' && faultBuffer[i].isActive) {
          strncpy(blinkCode, faultBuffer[i].blinkCode, maxLength);
          break;
        }
      }
    }

    // Get the current fault count
    int getFaultCount() {
      return faultCount;
    }

    // Update the fault status based on duration
    void updateFaultStatus() {
      unsigned long currentMillis = millis();
      for (int i = 0; i < FAULT_BUFFER_SIZE; i++) {
        if (faultBuffer[i].name[0] != '\0') {
          unsigned long faultDuration = currentMillis - faultBuffer[i].startTime;
          if (faultDuration >= FAULT_DURATION_THRESHOLD) {
            faultBuffer[i].isActive = true;
          } else {
            faultBuffer[i].isActive = false;
          }
        }
      }
    }

  private:
    struct Fault {
      char name[FAULT_NAME_LENGTH];
      char blinkCode[BLINK_CODE_LENGTH];
      int resetCount;
      unsigned long startTime;
      bool isAutoReset;
      bool isActive;
    };

    Fault faultBuffer[FAULT_BUFFER_SIZE];
    int faultCount = 0;
    bool isOverflown;

};

class TMP36Sensor {
  public:
      // Constructor
      TMP36Sensor(int pin, float minTempETH = -40.0, float maxTempETH = 125.0, float minTemp = -40.0, float maxTemp = 125.0)
          : pin(pin), minTempETH(minTempETH), maxTempETH(maxTempETH), minTemp(minTemp), maxTemp(maxTemp) {}

      // Method to read the temperature
      float readTemperature() {
          int sensorValue = analogRead(pin);  // Read the analog value from the sensor
          float voltage = sensorValue * (5.0 / 1023.0);  // Convert to voltage (assuming 5V reference)
          float temperatureC = (voltage - 0.5) * 100.0;  // Convert voltage to temperature in Celsius
          return temperatureC;
      }

      // Method to check if the temperature is within the specified range
      bool getFault() {
          float temperatureC = readTemperature();
          return (temperatureC < minTempETH || temperatureC > maxTempETH);
      }

  private:
      int pin;  // Analog pin where the sensor is connected
      float minTemp;  // Minimum temperature threshold
      float minTempETH;  // Minimum temperature threshold
      float maxTemp;  // Maximum temperature threshold
      float maxTempETH;  // Maximum temperature threshold
};

class ACS758CurrentSensor {
  public:
      // Constructor
      ACS758CurrentSensor(int voutPin, float sensitivity = 20.0, float quiescentVoltage = 0.6)
          : voutPin(voutPin), sensitivity(sensitivity), quiescentVoltage(quiescentVoltage) {}

      // Set the acceptable current range
      void setCurrentRange(float minCurrent, float maxCurrent) {
          this->minCurrent = minCurrent;
          this->maxCurrent = maxCurrent;
      }

      // Read the current from the sensor
      float getCurrent() {
          int sensorValue = analogRead(voutPin);
          float voltage = sensorValue * (5.0 / 1023.0); // Convert to voltage
          float current = (voltage - quiescentVoltage) / sensitivity;
          return current;
      }

      // Check if the current is within the acceptable range
      bool getError() {
          float current = getCurrent();
          return (current < minCurrent || current > maxCurrent);
      }

  private:
      int voutPin;
      float sensitivity;
      float quiescentVoltage;
      float minCurrent = 0.0;
      float maxCurrent = 200.0;
};


// -- Define Global Objects --//
Indicator IndicatorFault(LED_FAULT_PIN);
Indicator IndicatorPower(LED_INDICATOR1_PIN);
Indicator IndicatorOutput(LED_INDICATOR2_PIN);
Indicator IndicatorTempOK(LED_INDICATOR3_PIN);

VoltageSensor VoltageSensorVIN(A2, 6000.0 / 120.0, VIN_MIN_V, VIN_MAX_V, VSENSE_GRACE_TIME_MS);
VoltageSensor VoltageSensorVOUT(A3, 6000.0 / 40.0, VOUT_MIN_V, VOUT_MAX_V, VSENSE_GRACE_TIME_MS);

TMP36Sensor Phase1TempSense(PHASE_1_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);
TMP36Sensor Phase2TempSense(PHASE_2_TEMP_SENSOR_PIN, MIN_TEMP, MAX_TEMP);

ACS758CurrentSensor Phase1CurrentSense(PHASE_1_CURRENT_SENSE_PIN);
ACS758CurrentSensor Phase2CurrentSense(PHASE_2_CURRENT_SENSE_PIN);

FanController SystemFan(FAN_TACH, FAN_PWM);

FaultManager faultManager;

CANController canController(CAN_CS_PIN, CAN_INT_PIN);



bool OutputEnabled = true; // Indicates if the output has been enabled or not
bool HeartbeatOutput = false; // If true, a heartbeat must be recieved every n ms, else output is disabled.

float CurrentTemp_C = 30.0; // Temperature which is used to drive the fan control
unsigned long lastKeepAliveTime = 0;


float VIN, VOUT; // Voltage Values
float Phase1Temp_C, Phase2Temp_C; // Temperature values
float Phase1Current_A, Phase2Current_A; // Current values



// Canbus Handler
void handleCanResponse(struct can_frame* frame) {
    uint8_t dataBuffer[8]; // Buffer to hold CAN data
    uint16_t value;        // Temporary variable for 16-bit values

    switch (frame->can_id) {
        case 0x010:
            // Estop Disable Output Command
            faultManager.assertFault("CANBUS_REMOTE_ESTOP", "LSLL", false);
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Estop Disable Output Command Received");
      #endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x010, dataBuffer, 1);
            break;

        case 0x011:
            // Regular Stop Command
            OutputEnabled = false;
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Regular Stop Command Received");
      #endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x011, dataBuffer, 1);
            break;

        case 0x012:
            // Enable Output Command
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Enable Output Command Received");
      #endif
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x012, dataBuffer, 1);
            OutputEnabled = true;
            break;

        case 0x013:
            // Reset Faults Command
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Reset Faults Command Received");
      #endif
            faultManager.resetFaults();
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x013, dataBuffer, 1);
            break;

        case 0x014: {
            // Get Number of Faults Command
            uint8_t numFaults = faultManager.getFaultCount(); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Number of Faults: ");
            Serial.println(numFaults);
      #endif
            dataBuffer[0] = numFaults;
            CANController::instance().sendMessage(0x014, dataBuffer, 1);
            break;
        }

        case 0x015: {
            // Get Faults Command
            char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
            faultManager.getFaultNames(faultNames, sizeof(faultNames));
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Faults:");
            Serial.println(faultNames);
      #endif
            // Send the fault names over CANbus
            CANController::instance().sendMessage(0x015, (uint8_t*)faultNames, strlen(faultNames));
            break;
        }

        case 0x016: {
            // Get Vin Value Command
            value = int(VIN); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Vin Value: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x016, dataBuffer, 2);
            break;
        }

        case 0x017: {
            // Get Vout Value Command
            value = int(VOUT); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Vout Value: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x017, dataBuffer, 2);
            break;
        }

        case 0x018: {
            // Get Ph1 Current Command
            value = int(Phase1Current_A); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Ph1 Current: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x018, dataBuffer, 2);
            break;
        }

        case 0x019: {
            // Get Ph2 Current Command
            value = int(Phase2Current_A); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Ph2 Current: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x019, dataBuffer, 2);
            break;
        }

        case 0x01A: {
            // Get Ph1 Temperature Command
            value = int(Phase1Temp_C); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Ph1 Temperature: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x01A, dataBuffer, 2);
            break;
        }

        case 0x01B: {
            // Get Ph2 Temperature Command
            value = int(Phase2Temp_C); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Ph2 Temperature: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x01B, dataBuffer, 2);
            break;
        }

        case 0x01C: {
            // Get Fan Airflow LFM Command
            value = SystemFan.getAirflowLFM(); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Fan Airflow (LFM): ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x01C, dataBuffer, 2);
            break;
        }

        case 0x01D: {
            // Get Fan RPM Command
            value = SystemFan.getRPM(); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Fan RPM: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x01D, dataBuffer, 2);
            break;
        }

        case 0x01E: {
            // Get Power Command
            value = VIN * (Phase1Current_A + Phase2Current_A); // Hardcoded value
      #ifdef ENABLE_SERIAL_PRINT
            Serial.print("Power: ");
            Serial.println(value);
      #endif
            dataBuffer[0] = (value >> 8) & 0xFF;
            dataBuffer[1] = value & 0xFF;
            CANController::instance().sendMessage(0x01E, dataBuffer, 2);
            break;
        }

        case 0x01F: {
            // Get CAN Fault Command
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Checking CAN Faults...");
            if (CANController::instance().getError()) {
                Serial.println("CAN Error Detected");
            } else {
                Serial.println("CAN Communication OK");
            }
      #endif
            dataBuffer[0] = CANController::instance().getError() ? 0x01 : 0x00; // Boolean response
            CANController::instance().sendMessage(0x01F, dataBuffer, 1);
            break;
        }

        case 0x020:
            // Heartbeat Enable
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Heartbeat enable received.");
      #endif
            HeartbeatOutput = true;
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x020, dataBuffer, 1);
            break;

        case 0x021:
            // Heartbeat Ping
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Heartbeat ping received.");
      #endif
            lastKeepAliveTime = millis();
            dataBuffer[0] = 0x01; // Acknowledgment byte
            CANController::instance().sendMessage(0x021, dataBuffer, 1);
            break;

        default:
      #ifdef ENABLE_SERIAL_PRINT
            Serial.println("Unknown Command");
      #endif
            dataBuffer[0] = 0x02; // Error byte
            CANController::instance().sendMessage(frame->can_id, dataBuffer, 1); // Respond with the same ID
            break;
    }
}

//-- System Run Modes --//
void setup() {
    // Initialize the static instance pointer
    FanController::instance = &SystemFan;

    // Initialize serial communication
#ifdef ENABLE_SERIAL_PRINT
    Serial.begin(9600);
#endif

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
    SystemFan.setSpeed(BASE_FAN_SPEED);

    // Setup Output Enable Pin
    pinMode(OUTPUT_ENABLE_PIN, OUTPUT);

    // Init Current Sensors
    Phase1CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);
    Phase2CurrentSense.setCurrentRange(CURRENT_MIN_VALUE_A, CURRENT_MAX_VALUE_A);

    // Register Canbus Handler
    canController.begin();
    canController.registerCallback(handleCanResponse);

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
    SystemFan.setFanSpeed(CurrentTemp_C);

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
        Serial.print("VIN Voltage: ");
        Serial.print(VIN);
        Serial.println("V");

        Serial.print("VOUT Voltage: ");
        Serial.print(VOUT);
        Serial.println("V");
#endif
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



        // ---- UPDATE TEMPERATURE SENSOR DATA ---- //
        // Read Temperature Sensors
        Phase1Temp_C = Phase1TempSense.readTemperature();
        Phase2Temp_C = Phase2TempSense.readTemperature();

        // Select Max Temperature
        if (Phase1Temp_C > Phase2Temp_C) {
          // CurrentTemp_C = Phase1Temp_C;
        } else {
          // CurrentTemp_C = Phase2Temp_C;
        }

#ifdef ENABLE_SERIAL_PRINT
        Serial.print("Phase1 Temp: ");
        Serial.print(Phase1Temp_C);
        Serial.println("C");

        Serial.print("Phase2 Temp: ");
        Serial.print(Phase2Temp_C);
        Serial.println("C");
#endif
        // Temperature Sensor Faults
        if (Phase1TempSense.getFault()) {
            faultManager.assertFault("TEMPSENSE_1_OUT_OF_RANGE", "SSLS");
        }
        if (Phase2TempSense.getFault()) {
            faultManager.assertFault("TEMPSENSE_2_OUT_OF_RANGE", "SSLL");
        }

        // Update thermal indicator led
        IndicatorTempOK.SetState(!(Phase1TempSense.getFault() || Phase2TempSense.getFault() || SystemFan.checkError()));




        // ---- Current Sensor Monitoring ---- //
        Phase1Current_A = Phase1CurrentSense.getCurrent();
        Phase2Current_A = Phase2CurrentSense.getCurrent();

#ifdef ENABLE_SERIAL_PRINT
        Serial.print("Phase1 Current: ");
        Serial.print(Phase1Current_A);
        Serial.println("A");

        Serial.print("Phase2 Current: ");
        Serial.print(Phase2Current_A);
        Serial.println("A");
#endif        

        if (Phase1CurrentSense.getError()) {
            faultManager.assertFault("PHASE_A_OVERCURRENT", "LSSS");
        }
        if (Phase2CurrentSense.getError()) {
            faultManager.assertFault("PHASE_B_OVERCURRENT", "LSSL");
        }


        // ---- CANBUS FAULT CHECK ---- //
        if (canController.getError()) {
            faultManager.assertFault("CANBUS_COMM_ERROR", "LSLS");
        }


        // ---- SERIAL LOG FAULTS ---- //
        // Print current faults
        if (faultManager.getFaultCount() > 0) {
            char faultNames[FAULT_BUFFER_SIZE * FAULT_NAME_LENGTH];
            faultManager.getFaultNames(faultNames, sizeof(faultNames));
            Serial.print("Current Faults: ");
            Serial.println(faultNames);
        }

    }



    // Various faults for future reference:
    // // Trigger faults with specific blink codes
    // faultManager.assertFault("AIRFLOW_OUT_OF_RANGE", "SSSL");
    // faultManager.assertFault("TEMPSENSE_1_OUT_OF_RANGE", "SSLS");
    // faultManager.assertFault("TEMPSENSE_2_OUT_OF_RANGE", "SSLL");
    // faultManager.assertFault("VIN_UNDERVOLTAGE", "SLSS");
    // faultManager.assertFault("VIN_OVERVOLTAGE", "SLSL");
    // faultManager.assertFault("VOUT_UNDERVOLTAGE", "SLLS");
    // faultManager.assertFault("VOUT_OVERVOLTAGE", "SLLL");
    // faultManager.assertFault("PHASE_A_OVERCURRENT", "LSSS");
    // faultManager.assertFault("PHASE_B_OVERCURRENT", "LSSL");
    // faultManager.assertFault("CANBUS_COMM_ERROR", "LSLS");
    // faultManager.assertFault("CANBUS_REMOTE_ESTOP", "LSLL");
    // faultManager.assertFault("CANBUS_HEARTBEAT_TIMEOUT", "LLSS");


    // Heartbeat Logic Check
    if (millis() - lastKeepAliveTime > HEARTBEAT_SHUTDOWN_TIMER) {
      OutputEnabled = false;
      faultManager.assertFault("CANBUS_HEARTBEAT_TIMEOUT", "LLSS", false);
    }

    // Check that there are no errors, and disable output if there are
    if (faultManager.getFaultCount() != 0) {
      OutputEnabled = false;
    }
    IndicatorOutput.SetState(OutputEnabled);
    digitalWrite(OUTPUT_ENABLE_PIN, OutputEnabled);

    // Clear and forget auto-reset faults
    faultManager.clearAutoResetFaults();
    faultManager.forgetAutoResetFaults();


}
